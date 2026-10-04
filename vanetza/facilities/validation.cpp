#include <vanetza/facilities/validation.hpp>
#include <algorithm>
#include <ostream>
#include <utility>

namespace vanetza
{
namespace facilities
{

void ValidationResult::add(Severity severity, std::string rule, std::string path, std::string message)
{
    m_issues.push_back(ValidationIssue { severity, std::move(rule), std::move(path), std::move(message) });
}

void ValidationResult::append(const ValidationResult& other)
{
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

} // namespace facilities
} // namespace vanetza
