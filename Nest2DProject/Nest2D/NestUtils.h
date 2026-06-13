
#include <cmath>
#include <stdexcept>
#include <limits>
#include <libnest2d/backends/clipper/geometries.hpp>
#include <libnest2d/libnest2d.hpp>

namespace NestUtils {

    inline long double NestScale()
    {
        long double scale = static_cast<long double>(libnest2d::mm());
        if (scale == 0.0L || !std::isfinite(static_cast<double>(scale))) {
            throw std::runtime_error("Nest coordinate scale is invalid.");
        }
        return scale;
    }

    inline auto ToNestCoord(double AValue) -> decltype(libnest2d::mm(0.0))
    {
        if (!std::isfinite(AValue)) {
            throw std::invalid_argument("Coordinate value is not finite.");
        }

        using TNestCoord = decltype(libnest2d::mm(0.0));

        // Keep input/output conversion consistent with libnest2d's own unit scale.
        // The Clipper backend defines 1 mm as libnest2d::mm() internal integer units.
        long double scaled = static_cast<long double>(AValue) * NestScale();

        if (scaled > static_cast<long double>(std::numeric_limits<TNestCoord>::max()) ||
            scaled < static_cast<long double>(std::numeric_limits<TNestCoord>::lowest())) {
            throw std::overflow_error("Coordinate overflow after scaling.");
        }

        return static_cast<TNestCoord>(std::llround(scaled));
    }

    inline double FromNestCoord(decltype(libnest2d::mm(0.0)) AValue)
    {
        return static_cast<double>(static_cast<long double>(AValue) / NestScale());
    }

}
