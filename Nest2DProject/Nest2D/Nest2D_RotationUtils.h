#pragma once

#include "Nest2D_PrivateDataType.h"
#include "EtTechCore_Object.h"

#include <vector>

namespace ET {
    namespace NEST2DMANAGERLIB {

        class CetRotationUtils : public ET::CORE::CetCoreObject
        {
            Inherit_Invoke_Hook(CetRotationUtils)
        protected:
            int _Init() override { CetCoreObject::_Init(); return 0; }
            void _WrapFuncs() override { CetCoreObject::_WrapFuncs(); }
        public:
            CetRotationUtils() : CetCoreObject() {}
            ~CetRotationUtils() {}
            static double NormalizeAngle(double AAngle);
            static double AngleDistance(double ALeftAngle, double ARightAngle);
            static std::vector<double> BuildAllowedRotations(int ARotationCount);
            static std::vector<libnest2d::Radians> BuildAllowedLibRotations(int ARotationCount);
            static bool IsAllowedRotation(double AAngle, int ARotationCount, double ATolerance);
            static bool SnapToNearestAllowedRotation(double ATarget, int ARotationCount, double& AOutRotation);
            static bool SnapToAllowedRotation(double ATarget, int ARotationCount, double& AOutRotation, double ATolerance);
        };

    }
}
