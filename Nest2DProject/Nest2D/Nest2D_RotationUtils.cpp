#include "pch.h"
#include "Nest2D_RotationUtils.h"

#include <algorithm>
#include <cmath>

namespace ET {
    namespace NEST2DMANAGERLIB {

        namespace {
            constexpr double CET_ROTATION_DUPLICATE_TOLERANCE = 1e-12;

            double GetAngleDistance(double ALeftAngle, double ARightAngle)
            {
                const double LeftAngle = CetRotationUtils::NormalizeAngle(ALeftAngle);
                const double RightAngle = CetRotationUtils::NormalizeAngle(ARightAngle);
                const double Delta = std::abs(LeftAngle - RightAngle);
                return std::min(Delta, CET_CLUSTER_TWO_PI - Delta);
            }
        }

        double CetRotationUtils::NormalizeAngle(double AAngle)
        {
            if (!std::isfinite(AAngle)) {
                return 0.0;
            }

            double NormalizedAngle = std::fmod(AAngle, CET_CLUSTER_TWO_PI);
            if (NormalizedAngle < 0.0) {
                NormalizedAngle += CET_CLUSTER_TWO_PI;
            }

            if (NormalizedAngle >= CET_CLUSTER_TWO_PI) {
                NormalizedAngle = 0.0;
            }

            return NormalizedAngle;
        }

        std::vector<double> CetRotationUtils::BuildAllowedRotations(int ARotationCount)
        {
            std::vector<double> Rotations;
            if (ARotationCount <= 1) {
                Rotations.push_back(0.0);
                return Rotations;
            }

            Rotations.reserve(static_cast<std::size_t>(ARotationCount));
            const double AngleStep = CET_CLUSTER_TWO_PI / static_cast<double>(ARotationCount);
            for (int RotationIndex = 0; RotationIndex < ARotationCount; ++RotationIndex) {
                const double CandidateAngle = NormalizeAngle(static_cast<double>(RotationIndex) * AngleStep);
                bool Duplicate = false;
                for (double ExistingAngle : Rotations) {
                    if (GetAngleDistance(CandidateAngle, ExistingAngle) <= CET_ROTATION_DUPLICATE_TOLERANCE) {
                        Duplicate = true;
                        break;
                    }
                }

                if (!Duplicate) {
                    Rotations.push_back(CandidateAngle);
                }
            }

            if (Rotations.empty()) {
                Rotations.push_back(0.0);
            }

            return Rotations;
        }

        std::vector<libnest2d::Radians> CetRotationUtils::BuildAllowedLibRotations(int ARotationCount)
        {
            const std::vector<double> AllowedRotations = BuildAllowedRotations(ARotationCount);
            std::vector<libnest2d::Radians> Result;
            Result.reserve(AllowedRotations.size());
            for (double Rotation : AllowedRotations) {
                Result.push_back(libnest2d::Radians(Rotation));
            }
            return Result;
        }

        bool CetRotationUtils::IsAllowedRotation(double AAngle, int ARotationCount, double ATolerance)
        {
            double SnappedRotation = 0.0;
            return SnapToAllowedRotation(AAngle, ARotationCount, SnappedRotation, ATolerance);
        }

        bool CetRotationUtils::SnapToNearestAllowedRotation(double ATarget, int ARotationCount, double& AOutRotation)
        {
            AOutRotation = 0.0;
            if (!std::isfinite(ATarget)) {
                return false;
            }

            const std::vector<double> Rotations = BuildAllowedRotations(ARotationCount);
            const double TargetAngle = NormalizeAngle(ATarget);
            double BestDistance = 0.0;
            bool HasBest = false;

            for (double Rotation : Rotations) {
                const double Distance = GetAngleDistance(TargetAngle, Rotation);
                if (!HasBest || Distance < BestDistance) {
                    HasBest = true;
                    BestDistance = Distance;
                    AOutRotation = Rotation;
                }
            }

            return HasBest;
        }

        bool CetRotationUtils::SnapToAllowedRotation(double ATarget, int ARotationCount, double& AOutRotation, double ATolerance)
        {
            AOutRotation = 0.0;
            if (!std::isfinite(ATarget) || !std::isfinite(ATolerance) || ATolerance < 0.0) {
                return false;
            }

            const std::vector<double> Rotations = BuildAllowedRotations(ARotationCount);
            const double TargetAngle = NormalizeAngle(ATarget);
            double BestDistance = 0.0;
            bool HasBest = false;

            for (double Rotation : Rotations) {
                const double Distance = GetAngleDistance(TargetAngle, Rotation);
                if (!HasBest || Distance < BestDistance) {
                    HasBest = true;
                    BestDistance = Distance;
                    AOutRotation = Rotation;
                }
            }

            return HasBest && BestDistance <= ATolerance;
        }

    }
}
