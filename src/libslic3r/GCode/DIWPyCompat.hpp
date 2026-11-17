#ifndef slic3r_GCode_DIWPyCompat_hpp_
#define slic3r_GCode_DIWPyCompat_hpp_

// Number formatting identical to Python, for the DIW / u-MEX translation. The v2 post-processor
// (diw_engine.py) is the golden reference, and its warning texts interpolate floats with str()/repr()
// and format(v, 'g'). Requires IEEE-754 doubles without FMA contraction (/fp:precise, the MSVC default).

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

} // namespace DIW
} // namespace Slic3r

#endif // slic3r_GCode_DIWPyCompat_hpp_
