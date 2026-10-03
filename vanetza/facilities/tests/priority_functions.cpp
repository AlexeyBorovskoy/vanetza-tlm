#include <gtest/gtest.h>
#include <vanetza/asn1/its/IntersectionAccessPoint.h>
#include <vanetza/asn1/its/RequestorType.h>
#include <vanetza/asn1/its/SignalRequestList.h>
#include <vanetza/asn1/its/SignalRequestPackage.h>
#include <vanetza/asn1/its/SignalRequesterInfo.h>
#include <vanetza/asn1/its/SignalStatus.h>
#include <vanetza/asn1/its/SignalStatusPackage.h>
#include <vanetza/asn1/its/VehicleID.h>
#include <vanetza/asn1/srem.hpp>
#include <vanetza/asn1/ssem.hpp>
#include <vanetza/facilities/priority_functions.hpp>
#include <vanetza/facilities/spat_functions.hpp>
#include <chrono>
#include <stdexcept>

using namespace vanetza;
using namespace vanetza::facilities;

TEST(PriorityFunctions, build_srem)
{
    const auto now = Clock::at("2026-10-03 10:15:30.250");

    asn1::Srem srem;
    srem->header.protocolVersion = 2;
    srem->header.messageID = ItsPduHeader__messageID_srem;
    srem->header.stationID = 4711;
    set_timestamp(srem->srm, now);
    srem->srm.sequenceNumber = asn1::allocate<MsgCount_t>();
    *srem->srm.sequenceNumber = 5;
    set_requestor(srem->srm, 4711, BasicVehicleRole_publicTransport);

    PriorityRequest request;
    request.intersection_id = 42;
    request.request_id = 1;
    request.request_type = PriorityRequestType_priorityRequest;
    request.inbound_lane = 2;
    request.outbound_lane = 10;
    request.arrival = now + std::chrono::seconds(20);
    request.duration = std::chrono::seconds(8);
    add_request(srem->srm, request);

    std::string error;
    ASSERT_TRUE(srem.validate(error)) << error;
    asn1::Srem decoded;
    ASSERT_TRUE(decoded.decode(srem.encode()));

    const SignalRequestMessage_t& srm = decoded->srm;
    ASSERT_NE(nullptr, srm.timeStamp);
    EXPECT_EQ(minute_of_the_year(now), *srm.timeStamp);
    EXPECT_EQ(30250, srm.second);
    EXPECT_EQ(VehicleID_PR_stationID, srm.requestor.id.present);
    EXPECT_EQ(4711u, srm.requestor.id.choice.stationID);
    ASSERT_NE(nullptr, srm.requestor.type);
    EXPECT_EQ(BasicVehicleRole_publicTransport, srm.requestor.type->role);
    EXPECT_EQ(nullptr, srm.requestor.type->hpmsType);

    ASSERT_NE(nullptr, srm.requests);
    ASSERT_EQ(1, srm.requests->list.count);
    const SignalRequestPackage_t& package = *srm.requests->list.array[0];
    EXPECT_EQ(42, package.request.id.id);
    EXPECT_EQ(1, package.request.requestID);
    EXPECT_EQ(PriorityRequestType_priorityRequest, package.request.requestType);
    EXPECT_EQ(IntersectionAccessPoint_PR_lane, package.request.inBoundLane.present);
    EXPECT_EQ(2, package.request.inBoundLane.choice.lane);
    ASSERT_NE(nullptr, package.request.outBoundLane);
    EXPECT_EQ(10, package.request.outBoundLane->choice.lane);
    ASSERT_NE(nullptr, package.minute);
    EXPECT_EQ(minute_of_the_year(now), *package.minute);
    ASSERT_NE(nullptr, package.second);
    EXPECT_EQ(50250, *package.second); // 10:15:50.250
    ASSERT_NE(nullptr, package.duration);
    EXPECT_EQ(8000, *package.duration);
}

TEST(PriorityFunctions, set_requestor_replaces_previous_requestor)
{
    asn1::Srem srem;
    // requestor as decoded from another station: temporary ID and vehicle type
    VehicleID_t& id = srem->srm.requestor.id;
    id.present = VehicleID_PR_entityID;
    const uint8_t temporary_id[4] = { 1, 2, 3, 4 };
    ASSERT_EQ(0, OCTET_STRING_fromBuf(&id.choice.entityID, reinterpret_cast<const char*>(temporary_id), 4));
    set_requestor(srem->srm, 4711, BasicVehicleRole_publicTransport, VehicleType_bus);
    ASSERT_NE(nullptr, srem->srm.requestor.type);
    ASSERT_NE(nullptr, srem->srm.requestor.type->hpmsType);

    set_requestor(srem->srm, 4712, BasicVehicleRole_publicTransport);

    EXPECT_EQ(VehicleID_PR_stationID, srem->srm.requestor.id.present);
    EXPECT_EQ(4712u, srem->srm.requestor.id.choice.stationID);
    ASSERT_NE(nullptr, srem->srm.requestor.type);
    EXPECT_EQ(nullptr, srem->srm.requestor.type->hpmsType);
    // the temporary ID buffer is released, LeakSanitizer reports it otherwise
}

TEST(PriorityFunctions, request_duration_out_of_range)
{
    asn1::Srem srem;
    PriorityRequest request;
    request.intersection_id = 1;
    request.request_id = 1;
    request.request_type = PriorityRequestType_priorityRequest;
    request.inbound_lane = 1;
    request.duration = std::chrono::seconds(66); // DSecond ::= INTEGER (0..65535) ms
    EXPECT_THROW(add_request(srem->srm, request), std::out_of_range);
    EXPECT_EQ(nullptr, srem->srm.requests);

    // checked before truncation to milliseconds
    request.duration = std::chrono::milliseconds(65535) + std::chrono::microseconds(1);
    EXPECT_THROW(add_request(srem->srm, request), std::out_of_range);
    request.duration = -std::chrono::microseconds(1);
    EXPECT_THROW(add_request(srem->srm, request), std::out_of_range);
    request.duration = std::chrono::milliseconds(65535);
    EXPECT_NO_THROW(add_request(srem->srm, request));
}

TEST(PriorityFunctions, build_ssem)
{
    const auto now = Clock::at("2026-10-03 10:15:31");

    asn1::Ssem ssem;
    ssem->header.protocolVersion = 2;
    ssem->header.messageID = ItsPduHeader__messageID_ssem;
    ssem->header.stationID = 1;
    set_timestamp(ssem->ssm, now);
    SignalStatus& status = add_status(ssem->ssm, 42, 3);

    PriorityResponse response;
    response.requester = 4711;
    response.request_id = 1;
    response.request_sequence = 5;
    response.role = BasicVehicleRole_publicTransport;
    response.inbound_lane = 2;
    response.outbound_lane = 10;
    response.status = PrioritizationResponseStatus_processing;
    add_response(status, response);

    std::string error;
    ASSERT_TRUE(ssem.validate(error)) << error;
    asn1::Ssem decoded;
    ASSERT_TRUE(decoded.decode(ssem.encode()));

    const SignalStatusMessage_t& ssm = decoded->ssm;
    EXPECT_EQ(31000, ssm.second);
    ASSERT_EQ(1, ssm.status.list.count);
    const SignalStatus_t& rx = *ssm.status.list.array[0];
    EXPECT_EQ(42, rx.id.id);
    EXPECT_EQ(3, rx.sequenceNumber);
    ASSERT_EQ(1, rx.sigStatus.list.count);
    const SignalStatusPackage_t& package = *rx.sigStatus.list.array[0];
    ASSERT_NE(nullptr, package.requester);
    EXPECT_EQ(4711u, package.requester->id.choice.stationID);
    EXPECT_EQ(1, package.requester->request);
    EXPECT_EQ(5, package.requester->sequenceNumber);
    ASSERT_NE(nullptr, package.requester->role);
    EXPECT_EQ(BasicVehicleRole_publicTransport, *package.requester->role);
    EXPECT_EQ(2, package.inboundOn.choice.lane);
    ASSERT_NE(nullptr, package.outboundOn);
    EXPECT_EQ(10, package.outboundOn->choice.lane);
    EXPECT_EQ(PrioritizationResponseStatus_processing, package.status);
}

TEST(PriorityFunctions, ssem_without_status_is_not_encodable)
{
    // SignalStatusList ::= SEQUENCE (SIZE(1..32)), checked by the encoder only
    asn1::Ssem ssem;
    ssem->header.protocolVersion = 2;
    ssem->header.messageID = ItsPduHeader__messageID_ssem;
    EXPECT_THROW(ssem.encode(), std::runtime_error);
}
