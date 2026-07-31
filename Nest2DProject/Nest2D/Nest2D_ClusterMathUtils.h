#pragma once

#include <algorithm>
#include <cmath>

namespace ET {
    namespace NEST2DMANAGERLIB {

        class CetClusterMathUtils
        {
        public:
            static bool NearlyEqual(double AFirstValue, double ASecondValue, double ARelativeTolerance)
            {
                const double Denominator = std::max(1.0, std::max(std::abs(AFirstValue), std::abs(ASecondValue)));
                return std::abs(AFirstValue - ASecondValue) <= Denominator * ARelativeTolerance;
            }
        };

    }
}
