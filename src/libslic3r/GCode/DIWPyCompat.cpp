#include "DIWPyCompat.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace Slic3r {
namespace DIW {

static const char *non_finite(double v)
{
    if (std::isnan(v)) return "nan";
    if (std::isinf(v)) return v < 0 ? "-inf" : "inf";
    return nullptr;
}

std::string py_repr(double v)
{
    if (const char *s = non_finite(v)) return s;
    // Shortest round-trip representation in scientific form, e.g. "1.5e-05", "3e+00".
    char buf[64];
    const auto res = std::to_chars(buf, buf + sizeof(buf), v, std::chars_format::scientific);
    const std::string sci(buf, res.ptr);
    const size_t epos = sci.find('e');
    std::string mant = sci.substr(0, epos);
    const int exp10 = std::atoi(sci.c_str() + epos + 1);
    std::string sign;
    if (! mant.empty() && mant[0] == '-') {
        sign = "-";
        mant.erase(0, 1);
    }
    std::string digits;
    for (char c : mant)
        if (c != '.') digits += c;

    std::string out;
    if (exp10 >= -4 && exp10 < 16) {
        if (exp10 >= 0) {
            std::string ip = digits.substr(0, std::min(digits.size(), size_t(exp10) + 1));
            while (ip.size() < size_t(exp10) + 1) ip += '0';
            std::string fp = digits.size() > size_t(exp10) + 1 ? digits.substr(size_t(exp10) + 1) : "";
            out = ip + "." + (fp.empty() ? "0" : fp);
        } else
            out = "0." + std::string(size_t(-exp10 - 1), '0') + digits;
    } else {
        out = digits.substr(0, 1);
        if (digits.size() > 1)
            out += "." + digits.substr(1);
        char eb[16];
        snprintf(eb, sizeof(eb), "e%c%02d", exp10 < 0 ? '-' : '+', std::abs(exp10));
        out += eb;
    }
    return sign + out;
}

std::string py_g(double v)
{
    if (const char *s = non_finite(v)) return s;
    char buf[64];
    snprintf(buf, sizeof(buf), "%g", v);
    return buf;
}

std::string py_fixed(int prec, double v)
{
    if (const char *s = non_finite(v)) return s;
    const int n = snprintf(nullptr, 0, "%.*f", prec, v);
    std::string r(size_t(n > 0 ? n : 0), '\0');
    if (n > 0)
        snprintf(&r[0], size_t(n) + 1, "%.*f", prec, v);
    return r;
}

} // namespace DIW
} // namespace Slic3r
