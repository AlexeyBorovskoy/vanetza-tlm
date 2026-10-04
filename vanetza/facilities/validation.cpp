#include <vanetza/asn1/asn1c_wrapper.hpp>
#include <vanetza/asn1/support/asn_codecs_prim.h>
#include <vanetza/asn1/support/asn_SET_OF.h>
#include <vanetza/asn1/support/BIT_STRING.h>
#include <vanetza/asn1/support/constr_CHOICE.h>
#include <vanetza/asn1/support/constr_SEQUENCE.h>
#include <vanetza/asn1/support/constr_SEQUENCE_OF.h>
#include <vanetza/asn1/support/constr_SET.h>
#include <vanetza/asn1/support/constr_SET_OF.h>
#include <vanetza/asn1/support/OCTET_STRING.h>
#include <vanetza/asn1/support/OPEN_TYPE.h>
#include <vanetza/asn1/support/per_support.h>
#include <vanetza/facilities/validation.hpp>
#include <algorithm>
#include <ostream>
#include <utility>

namespace vanetza
{
namespace facilities
{

namespace
{

// bounds the recursion if a malformed structure refers to itself
const int max_nesting = 64;

class StructureWalk
{
public:
    explicit StructureWalk(ValidationResult& result) : m_result(result), m_valid(true) {}

    bool valid() const { return m_valid; }

    void walk(const asn_TYPE_descriptor_t& type, const asn_TYPE_member_t* member, const void* structure,
        const std::string& path, int depth)
    {
        if (depth > max_nesting) {
            fail(path, "nested deeper than " + std::to_string(max_nesting) + " levels");
        } else if (type.op == &asn_OP_SEQUENCE || type.op == &asn_OP_SET) {
            walk_members(type, structure, path, depth);
        } else if (type.op == &asn_OP_CHOICE || type.op == &asn_OP_OPEN_TYPE) {
            walk_choice(type, structure, path, depth);
        } else if (type.op == &asn_OP_SEQUENCE_OF || type.op == &asn_OP_SET_OF) {
            walk_list(type, member, structure, path, depth);
        } else if (type.op == &asn_OP_BIT_STRING) {
            check_buffer(static_cast<const BIT_STRING_t*>(structure)->buf,
                static_cast<const BIT_STRING_t*>(structure)->size, path);
        } else if (type.op->free_struct == OCTET_STRING_free) {
            // OCTET STRING and the restricted character string types share OCTET_STRING_t
            check_buffer(static_cast<const OCTET_STRING_t*>(structure)->buf,
                static_cast<const OCTET_STRING_t*>(structure)->size, path);
        } else if (type.op->free_struct == ASN__PRIMITIVE_TYPE_free) {
            // INTEGER, ENUMERATED, REAL and object identifiers not mapped to native types
            check_buffer(static_cast<const ASN__PRIMITIVE_TYPE_t*>(structure)->buf,
                static_cast<const ASN__PRIMITIVE_TYPE_t*>(structure)->size, path);
        }
    }

private:
    void fail(const std::string& path, std::string message)
    {
        m_result.add(Severity::Error, "ASN.1", path, std::move(message));
        m_valid = false;
    }

    static const void* member_structure(const asn_TYPE_member_t& member, const void* structure)
    {
        const char* address = static_cast<const char*>(structure) + member.memb_offset;
        return (member.flags & ATF_POINTER) ? *reinterpret_cast<const void* const*>(address) : address;
    }

    void walk_members(const asn_TYPE_descriptor_t& type, const void* structure, const std::string& path, int depth)
    {
        for (unsigned n = 0; n < type.elements_count; ++n) {
            const asn_TYPE_member_t& member = type.elements[n];
            const std::string member_path = path + "." + member.name;
            const void* value = member_structure(member, structure);
            if (value) {
                walk(*member.type, &member, value, member_path, depth + 1);
            } else if (!member.optional) {
                fail(member_path, "mandatory element absent");
            }
        }
    }

    void walk_choice(const asn_TYPE_descriptor_t& type, const void* structure, const std::string& path, int depth)
    {
        const asn_CHOICE_specifics_t& specifics = *static_cast<const asn_CHOICE_specifics_t*>(type.specifics);
        const unsigned present = _fetch_present_idx(structure, specifics.pres_offset, specifics.pres_size);
        if (present == 0 || present > type.elements_count) {
            fail(path, "no valid alternative selected");
            return;
        }
        const asn_TYPE_member_t& member = type.elements[present - 1];
        const std::string member_path = path + "." + member.name;
        const void* value = member_structure(member, structure);
        if (value) {
            walk(*member.type, &member, value, member_path, depth + 1);
        } else if (!member.optional) {
            fail(member_path, "selected alternative absent");
        }
    }

    // a SIZE constraint of the member takes precedence over the one of its type
    static const asn_per_constraint_t* size_constraint(const asn_TYPE_descriptor_t& type,
        const asn_TYPE_member_t* member)
    {
        const asn_per_constraints_t* constraints = member && member->encoding_constraints.per_constraints ?
            member->encoding_constraints.per_constraints : type.encoding_constraints.per_constraints;
        return constraints ? &constraints->size : nullptr;
    }

    void check_size(const asn_per_constraint_t& size, int count, const std::string& path)
    {
        // the flags enumeration is nested in asn_per_constraint_s when compiled as C++
        if (size.flags & asn_per_constraint_t::APC_EXTENSIBLE) {
            return; // sizes beyond the root are encodable
        }
        const bool bounded = size.flags & asn_per_constraint_t::APC_CONSTRAINED;
        const bool lower_bounded = bounded || (size.flags & asn_per_constraint_t::APC_SEMI_CONSTRAINED);
        if ((lower_bounded && count < size.lower_bound) || (bounded && count > size.upper_bound)) {
            fail(path, std::to_string(count) + " elements, SIZE(" + std::to_string(size.lower_bound) + ".." +
                (bounded ? std::to_string(size.upper_bound) : std::string("MAX")) + ")");
        }
    }

    void walk_list(const asn_TYPE_descriptor_t& type, const asn_TYPE_member_t* member, const void* structure,
        const std::string& path, int depth)
    {
        const asn_anonymous_set_& list = *_A_CSET_FROM_VOID(structure);
        if (list.count < 0 || list.count > list.size || (list.count > 0 && !list.array)) {
            fail(path, "inconsistent list: count " + std::to_string(list.count) + ", allocated " +
                std::to_string(list.size) + (list.array ? "" : ", no array"));
            return;
        }
        const asn_per_constraint_t* size = size_constraint(type, member);
        if (size) {
            check_size(*size, list.count, path);
        }
        const asn_TYPE_member_t& element = type.elements[0];
        for (int i = 0; i < list.count; ++i) {
            const std::string element_path = path + "[" + std::to_string(i) + "]";
            if (list.array[i]) {
                walk(*element.type, &element, list.array[i], element_path, depth + 1);
            } else {
                fail(element_path, "empty list element");
            }
        }
    }

    void check_buffer(const uint8_t* buffer, std::size_t size, const std::string& path)
    {
        if (!buffer && size > 0) {
            fail(path, "size " + std::to_string(size) + " without buffer");
        }
    }

    ValidationResult& m_result;
    bool m_valid;
};

} // namespace

void ValidationResult::add(Severity severity, std::string rule, std::string path, std::string message)
{
    m_issues.push_back(ValidationIssue { severity, std::move(rule), std::move(path), std::move(message) });
}

void ValidationResult::append(const ValidationResult& other)
{
    if (&other == this) {
        const std::vector<ValidationIssue> copy = m_issues; // inserting a range of itself is undefined
        m_issues.insert(m_issues.end(), copy.begin(), copy.end());
        return;
    }
    m_issues.insert(m_issues.end(), other.m_issues.begin(), other.m_issues.end());
}

std::size_t ValidationResult::count(Severity severity) const
{
    return std::count_if(m_issues.begin(), m_issues.end(),
        [severity](const ValidationIssue& issue) { return issue.severity == severity; });
}

std::ostream& operator<<(std::ostream& os, const ValidationIssue& issue)
{
    os << (issue.severity == Severity::Error ? "error " : "warning ") << issue.rule << " " << issue.path
        << ": " << issue.message;
    return os;
}

bool check_asn1(asn_TYPE_descriptor_t& type, const void* structure, const std::string& path, ValidationResult& result)
{
    if (!structure) {
        result.add(Severity::Error, "ASN.1", path, "structure absent");
        return false;
    }
    StructureWalk walk(result);
    walk.walk(type, nullptr, structure, path, 0);
    if (!walk.valid()) {
        return false;
    }
    std::string error;
    if (!asn1::validate(type, structure, error)) {
        result.add(Severity::Error, "ASN.1", path, error);
        return false;
    }
    return true;
}

} // namespace facilities
} // namespace vanetza
