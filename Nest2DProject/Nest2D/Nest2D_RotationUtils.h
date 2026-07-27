#pragma once

#include "Nest2D_PrivateDataType.h"

#include <vector>

namespace ET {
    namespace NEST2DMANAGERLIB {

        class CetRotationUtils
        {
        public:
            static double NormalizeAngle(double AAngle);
            static std::vector<double> BuildAllowedRotations(int ARotationCount);
            static std::vector<libnest2d::Radians> BuildAllowedLibRotations(int ARotationCount);
            static bool IsAllowedRotation(double AAngle, int ARotationCount, double ATolerance);
            static bool SnapToNearestAllowedRotation(double ATarget, int ARotationCount, double& AOutRotation);
            static bool SnapToAllowedRotation(double ATarget, int ARotationCount, double& AOutRotation, double ATolerance);
        };

    }
}
