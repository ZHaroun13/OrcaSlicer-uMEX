#include "DIWPyCompat.hpp"

#include "../LocalesUtils.hpp"

#include <algorithm>
#include <cctype>
#include <cfloat>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <string_view>

namespace Slic3r {
namespace DIW {

// ------------------------------------------------------------------------------------------------
// Text helpers (moved from DIWTranslator.cpp in increment 3a, unchanged, so the u-MEX sequencer shares them)
// ------------------------------------------------------------------------------------------------

static inline bool is_cont(unsigned char c) { return (c & 0xC0) == 0x80; }

uint32_t decode_at(const std::string &s, size_t i, size_t *len)
{
    const unsigned char c = (unsigned char) s[i];
    size_t n = c < 0x80 ? 1 : c < 0xE0 ? 2 : c < 0xF0 ? 3 : 4;
    if (i + n > s.size()) n = 1;
    uint32_t cp = n == 1 ? c : n == 2 ? (c & 0x1F) : n == 3 ? (c & 0x0F) : (c & 0x07);
    for (size_t k = 1; k < n; ++k)
        cp = (cp << 6) | ((unsigned char) s[i + k] & 0x3F);
    if (len) *len = n;
    return cp;
}

size_t prev_start(const std::string &s, size_t i)
{
    size_t k = i - 1;
    while (k > 0 && is_cont((unsigned char) s[k])) --k;
    return k;
}

// Python str.isspace(): ASCII \t \n \v \f \r, \x1c-\x1f, space, and the Unicode spaces.
bool is_space_cp(uint32_t c)
{
    return c == ' ' || (c >= 0x09 && c <= 0x0D) || (c >= 0x1C && c <= 0x1F) || c == 0x85 || c == 0xA0 || c == 0x1680 ||
           (c >= 0x2000 && c <= 0x200A) || c == 0x2028 || c == 0x2029 || c == 0x202F || c == 0x205F || c == 0x3000;
}

// Python's \w (str.isalnum() or '_'). Exact for ASCII and Latin-1; elsewhere letters and digits of all
// scripts count as word characters and the main punctuation / symbol blocks do not.
bool is_word_cp(uint32_t c)
{
    if (c < 0x80)
        return std::isalnum((int) c) || c == '_';
    if (c <= 0xBF) // Latin-1 punctuation and symbols; ª ² ³ µ ¹ º ¼ ½ ¾ are letters or numbers
        return c == 0xAA || c == 0xB2 || c == 0xB3 || c == 0xB5 || c == 0xB9 || c == 0xBA || c == 0xBC || c == 0xBD || c == 0xBE;
    if (c == 0xD7 || c == 0xF7) // × ÷
        return false;
    if (is_space_cp(c))
        return false;
    if ((c >= 0x2000 && c <= 0x206F) ||  // general punctuation
        (c >= 0x20A0 && c <= 0x20CF) ||  // currency symbols
        (c >= 0x2190 && c <= 0x245F) ||  // arrows, mathematical operators, technical symbols
        (c >= 0x2500 && c <= 0x2BFF) ||  // box drawing, shapes, dingbats, misc symbols and arrows
        (c >= 0x3000 && c <= 0x3004) || (c >= 0x3008 && c <= 0x3020) || // CJK punctuation
        (c >= 0xFE30 && c <= 0xFE4F) || (c >= 0xFF01 && c <= 0xFF0F) || c == 0xFFFD)
        return false;
    return true;
}

size_t space_len_at(const std::string &s, size_t i)
{
    size_t n;
    return is_space_cp(decode_at(s, i, &n)) ? n : 0;
}

bool space_at(const std::string &s, size_t i) { return i < s.size() && space_len_at(s, i) > 0; }
bool word_at(const std::string &s, size_t i) { return i < s.size() && is_word_cp(decode_at(s, i, nullptr)); }
bool word_before(const std::string &s, size_t i) { return i > 0 && is_word_cp(decode_at(s, prev_start(s, i), nullptr)); }

std::string to_lower(const std::string &s)
{
    std::string r(s);
    for (char &c : r) c = lower(c);
    return r;
}

std::string to_upper(const std::string &s)
{
    std::string r(s);
    for (char &c : r)
        if (c >= 'a' && c <= 'z') c = char(c - 'a' + 'A');
    return r;
}

static size_t rstrip_end(const std::string &s, size_t b)
{
    size_t e = s.size();
    while (e > b) {
        const size_t p = prev_start(s, e);
        if (p < b || space_len_at(s, p) == 0) break;
        e = p;
    }
    return e;
}

std::string strip(const std::string &s)
{
    size_t b = 0;
    for (size_t n; b < s.size() && (n = space_len_at(s, b)) > 0;) b += n;
    return s.substr(b, rstrip_end(s, b) - b);
}

std::string lstrip(const std::string &s)
{
    size_t b = 0;
    for (size_t n; b < s.size() && (n = space_len_at(s, b)) > 0;) b += n;
    return s.substr(b);
}

std::string rstrip(const std::string &s) { return s.substr(0, rstrip_end(s, 0)); }

size_t match_number(const std::string &s, size_t j)
{
    size_t k = j;
    if (k < s.size() && (s[k] == '+' || s[k] == '-')) ++k;
    const size_t d0 = k;
    while (k < s.size() && is_digit(s[k])) ++k;
    const size_t nd = k - d0;
    if (k + 1 < s.size() && s[k] == '.' && is_digit(s[k + 1])) {
        k += 2;
        while (k < s.size() && is_digit(s[k])) ++k;
        return k;
    }
    return nd >= 1 ? d0 + nd : std::string::npos;
}

double parse_number(const std::string &s, size_t b, size_t e)
{
    if (s[b] == '+') ++b;
    return string_to_double_decimal_point(std::string_view(s.data() + b, e - b));
}

WordMatch find_word(const std::string &s, const char *letters, size_t from)
{
    for (size_t i = from; i < s.size(); ++i) {
        const char c = s[i];
        bool hit = false;
        for (const char *l = letters; *l; ++l)
            if (c == *l || c == lower(*l)) { hit = true; break; }
        if (! hit || word_before(s, i))
            continue;
        const size_t e = match_number(s, i + 1);
        if (e == std::string::npos)
            continue;
        WordMatch m;
        m.begin = i;
        m.end   = e;
        m.value = parse_number(s, i + 1, e);
        return m;
    }
    return {};
}

std::string replace_words(const std::string &s, const char *letters, const std::string &replacement)
{
    std::string r;
    size_t pos = 0;
    for (WordMatch m = find_word(s, letters, 0); m; m = find_word(s, letters, pos)) {
        r.append(s, pos, m.begin - pos);
        r += replacement;
        pos = m.end;
    }
    r.append(s, pos, std::string::npos);
    return r;
}

// ------------------------------------------------------------------------------------------------
// Number formatting
// ------------------------------------------------------------------------------------------------

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

// CPython Modules/mathmodule.c vector_norm() for two coordinates (math.hypot since 3.10): lossless scaling
// by a power of two, error-free squaring (dl_mul) and summation (dl_fast_sum), sqrt, then one differential
// correction. std::fma gives dl_mul's exact low part on every platform, as CPython's fma path does.
double py_hypot(double dx, double dy)
{
    const double a = std::fabs(dx), b = std::fabs(dy);
    if (std::isnan(a) || std::isnan(b))
        return (std::isinf(a) || std::isinf(b)) ? INFINITY : NAN;
    const double max = a > b ? a : b;
    if (std::isinf(max) || max == 0.0)
        return max;
    int max_e = 0;
    std::frexp(max, &max_e);
    if (max_e < -1023) {
        // Subnormal maximum: CPython rescales by DBL_MIN and recurses.
        return DBL_MIN * py_hypot(a / DBL_MIN, b / DBL_MIN);
    }
    const double scale = std::ldexp(1.0, -max_e);
    double csum = 1.0, frac1 = 0.0, frac2 = 0.0;
    auto add_square = [&](double x) {
        const double pr_hi = x * x;
        const double pr_lo = std::fma(x, x, -pr_hi);
        const double sm_hi = csum + pr_hi;
        const double z     = sm_hi - csum;
        const double sm_lo = pr_hi - z;
        csum = sm_hi;
        frac1 += pr_lo;
        frac2 += sm_lo;
    };
    add_square(a * scale);
    add_square(b * scale);
    double h = std::sqrt(csum - 1.0 + (frac1 + frac2));
    {
        const double pr_hi = -h * h;
        const double pr_lo = std::fma(-h, h, -pr_hi);
        const double sm_hi = csum + pr_hi;
        const double z     = sm_hi - csum;
        const double sm_lo = pr_hi - z;
        csum = sm_hi;
        frac1 += pr_lo;
        frac2 += sm_lo;
    }
    const double x = csum - 1.0 + (frac1 + frac2);
    h += x / (2.0 * h);
    return h / scale;
}

void PySum::add(double x)
{
    if (! m_started) {
        m_sum     = x;   // 0 + x: the int start value turns into the first float exactly
        m_started = true;
        return;
    }
    const double t = m_sum + x;
    if (std::fabs(m_sum) >= std::fabs(x))
        m_c += (m_sum - t) + x;
    else
        m_c += (x - t) + m_sum;
    m_sum = t;
}

double PySum::result() const
{
    double f = m_sum;
    if (m_c != 0.0 && std::isfinite(m_c))
        f += m_c;
    return f;
}

} // namespace DIW
} // namespace Slic3r
