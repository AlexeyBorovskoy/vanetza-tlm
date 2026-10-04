#ifndef VALIDATION_HPP_W8NQ3LZE
#define VALIDATION_HPP_W8NQ3LZE

#include <cstddef>
#include <iosfwd>
#include <string>
#include <vector>

namespace vanetza
{
namespace facilities
{

/**
 * Results of semantic checks of infrastructure messages (SPATEM, MAPEM, SREM, SSEM).
 *
 * Every issue names the rule it is based on and the field it concerns. Rules taken from
 * a profile are only checked if the selected profile includes it, so a project preference
 * is never reported as a violation of a standard.
 */

/** Severity of a validation issue */
enum class Severity
{
    Warning, /**< suspicious or a compatibility note, the message is usable */
    Error /**< violates the cited requirement */
};

/** Requirements checked in addition to the ASN.1 module texts */
enum class ValidationProfile
{
    Standard, /**< ETSI TS 103 301 and ISO TS 19091 (DSRC module) only */
    Car2Car, /**< plus C2C-CC RS 2077 TLM/RLT automotive requirements */
    CRoads, /**< plus the C-Roads columns of C2C-CC RS 2077 Annex 7 */
    Combined /**< C2C-CC RS 2077 and C-Roads */
};

/** One finding of a validation */
struct ValidationIssue
{
    Severity severity;
    std::string rule; /**< rule identifier, e.g. "RS_ARSM_49" or "LaneID unique" */
    std::string path; /**< field path, e.g. "MapData.intersections[0].laneSet[4].connectsTo[0]" */
    std::string message; /**< what is wrong */
};

/** Collected issues of one or more validations */
class ValidationResult
{
public:
    void add(Severity, std::string rule, std::string path, std::string message);
    void append(const ValidationResult&);

    const std::vector<ValidationIssue>& issues() const { return m_issues; }
    std::size_t count(Severity) const;

    /** \return true if no issue has severity Error */
    bool valid() const { return count(Severity::Error) == 0; }

private:
    std::vector<ValidationIssue> m_issues;
};

/** Print an issue as "error <rule> <path>: <message>" */
std::ostream& operator<<(std::ostream&, const ValidationIssue&);

} // namespace facilities
} // namespace vanetza

#endif /* VALIDATION_HPP_W8NQ3LZE */
