#ifndef VALIDATION_HPP_W8NQ3LZE
#define VALIDATION_HPP_W8NQ3LZE

#include <cstddef>
#include <iosfwd>
#include <string>
#include <vector>

struct asn_TYPE_descriptor_s;

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

/**
 * Check a structure against its ASN.1 type, the precondition of any semantic check.
 *
 * The structure is walked along its asn1c type description first. Reported are absent
 * mandatory elements, a CHOICE or open type without valid alternative, an inconsistent
 * SEQUENCE OF or SET OF (negative count, count above the allocation, elements without array),
 * empty list elements, list sizes outside the PER-visible SIZE constraint and strings or
 * INTEGERs with a size but without buffer. asn1c constraint checking does not check the SIZE
 * of named SEQUENCE OF types (only inline SIZE constraints become member constraints) and reads
 * string buffers without null check, so it only runs if the walk found nothing. The lower bound
 * of SIZE(n..MAX) is checked as X.680 defines it, although the asn1c UPER encoder accepts fewer
 * elements; nesting deeper than 64 levels, possible for recursive types only, is reported.
 * Every issue has rule "ASN.1".
 *
 * \param type asn1c descriptor of the structure, e.g. asn_DEF_MapData
 * \param structure structure to check
 * \param path field path of the structure, prefix of the reported paths
 * \param result receives the issues
 * \return true if the structure is fit for semantic checks
 */
bool check_asn1(asn_TYPE_descriptor_s& type, const void* structure, const std::string& path, ValidationResult& result);

} // namespace facilities
} // namespace vanetza

#endif /* VALIDATION_HPP_W8NQ3LZE */
