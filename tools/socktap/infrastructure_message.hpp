#ifndef INFRASTRUCTURE_MESSAGE_HPP_T4WN7QXB
#define INFRASTRUCTURE_MESSAGE_HPP_T4WN7QXB

#include <vanetza/asn1/its/ItsPduHeader.h>

/**
 * Check a received SPATEM, MAPEM, SREM or SSEM before using its content:
 * ASN.1 constraints, protocol version and the expected message type (ETSI TS 103 301).
 * A wrong message type may still decode, since UPER is not self-describing.
 *
 * \param message decoded message wrapper, e.g. vanetza::asn1::Srem
 * \param message_id expected messageID, e.g. ItsPduHeader__messageID_srem
 * \return true if the message can be processed
 */
template<typename MESSAGE>
bool is_supported_message(const MESSAGE& message, long message_id)
{
    return message.validate() && message->header.protocolVersion == 2 &&
        message->header.messageID == message_id;
}

#endif /* INFRASTRUCTURE_MESSAGE_HPP_T4WN7QXB */
