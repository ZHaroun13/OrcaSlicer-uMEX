#ifndef slic3r_GCode_DIWPyCompat_hpp_
#define slic3r_GCode_DIWPyCompat_hpp_

// Python compatibility for the DIW / u-MEX translation. The v2 post-processor (diw_engine.py) is the
// golden reference, so numbers are formatted as Python formats them (its warning texts interpolate floats
// with str()/repr() and format(v, 'g')), and its str methods and regular expressions are reproduced by
// small hand-written matchers with the same behaviour. Requires IEEE-754 doubles without FMA contraction
// (/fp:precise, the MSVC default).

#include <cstddef>
#include <cstdint>
#include <string>

namespace Slic3r {
namespace DIW {

// Python repr(float) / str(float): shortest round-trip digits; positional for 1e-4 <= |v| < 1e16
// with at least one fractional digit ("3.0"), otherwise "1.5e-05" / "1e+16"; "nan", "inf", "-inf".
std::string py_repr(double v);

// Python format(v, 'g').
std::string py_g(double v);

// Python f"{v:.<prec>f}": full length, "nan" / "inf" / "-inf" for non-finite values.
std::string py_fixed(int prec, double v);

// Python math.hypot(dx, dy) bit for bit (CPython's vector_norm, not the C library's hypot, which may
// differ in the last ulp).
double py_hypot(double dx, double dy);

// Python sum() over floats (3.12+): Neumaier-compensated summation, compensation added at the end.
// add() in iteration order, then result(). Not the same as a plain left-to-right "+=" loop.
class PySum
{
public:
    void   add(double x);
    double result() const;

private:
    double m_sum = 0.0, m_c = 0.0;
    bool   m_started = false;
};

// ------------------------------------------------------------------------------------------------
// Text helpers with Python str semantics. The text is valid UTF-8; whitespace and word characters are
// judged per code point, as Python's str methods and Unicode regexes do.
// ------------------------------------------------------------------------------------------------

inline bool is_digit(char c) { return c >= '0' && c <= '9'; }
inline char lower(char c) { return (c >= 'A' && c <= 'Z') ? char(c - 'A' + 'a') : c; }

// Code point starting at s[i]; *len receives its length in bytes.
uint32_t decode_at(const std::string &s, size_t i, size_t *len);
// Start of the code point that ends just before s[i] (i > 0).
size_t   prev_start(const std::string &s, size_t i);
// Python str.isspace() for one code point.
bool     is_space_cp(uint32_t c);
// Python's \w for one code point (exact for ASCII and Latin-1, see the definition).
bool     is_word_cp(uint32_t c);
// Byte length of the whitespace code point at s[i], 0 if s[i] does not start one.
size_t   space_len_at(const std::string &s, size_t i);
bool     space_at(const std::string &s, size_t i);
bool     word_at(const std::string &s, size_t i);
bool     word_before(const std::string &s, size_t i);

std::string to_lower(const std::string &s);   // ASCII letters only
std::string to_upper(const std::string &s);   // ASCII letters only
std::string strip(const std::string &s);      // str.strip()
std::string lstrip(const std::string &s);     // str.lstrip()
std::string rstrip(const std::string &s);     // str.rstrip()

// Word matcher equivalent to  re.compile(r"\b<L>([-+]?[0-9]*\.?[0-9]+)", re.IGNORECASE)
struct WordMatch
{
    size_t begin = std::string::npos; // position of the letter
    size_t end   = 0;                 // one past the number
    double value = 0.;
    explicit operator bool() const { return begin != std::string::npos; }
};

// Number at s[j]: [-+]?[0-9]*\.?[0-9]+ with the regex's backtracking. Returns the end or npos.
size_t      match_number(const std::string &s, size_t j);
// float() of the matched number s[b, e).
double      parse_number(const std::string &s, size_t b, size_t e);
// letters: one or more upper-case letters accepted at the word position, e.g. "E" or "PS" (re.search).
WordMatch   find_word(const std::string &s, const char *letters, size_t from = 0);
// re.sub(pattern, replacement, s) for the word pattern above (replacement taken literally).
std::string replace_words(const std::string &s, const char *letters, const std::string &replacement);

} // namespace DIW
} // namespace Slic3r

#endif // slic3r_GCode_DIWPyCompat_hpp_
