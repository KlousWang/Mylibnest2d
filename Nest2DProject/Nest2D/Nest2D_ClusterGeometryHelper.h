#pragma once

#include "EtTechCore_Object.h"
#include "Nest2D_PrivateDataType.h"
#include "Nest2D_DataType.h"

#include <vector>

namespace ET {
    namespace NEST2DMANAGERLIB {
        constexpr std::size_t CET_CLUSTER_TARGET_COPIES_PER_BOARD = 4;

        class CetClusterGeometryHelper : public ET::CORE::CetCoreObject {
            Inherit_Invoke_Hook(CetClusterGeometryHelper)

        protected:
            int _Init() override { CetCoreObject::_Init(); return 0; }
            void _WrapFuncs() override { CetCoreObject::_WrapFuncs(); }

        public:
            CetClusterGeometryHelper();
            ~CetClusterGeometryHelper();

        public:
            CetPath GetIdentityContour(const CetNestItem& AItem) const;
            CetPath TransformContour(const CetPath& AContour, double ARotation, double ATranslationX, double ATranslationY) const;
            bool GetBounds(const CetPath& AContour, double& AOutMinX, double& AOutMinY, double& AOutMaxX, double& AOutMaxY) const;
            CetPath MakeRectangleContour(double AWidth, double AHeight) const;
            CetNestItem MakeNestItemFromProxyContour(const CetPath& AProxyContour) const;
            bool IsContourFullyContained(const CetPath& AChildContour, const CetPath& AProxyContour, double AAreaTolerance) const;
            bool FinalizeCandidate(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, TetClusterCandidate& ACandidate) const;
            bool FinalizeCandidate(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, TetClusterCandidate& ACandidate, bool AForceRectangleProxy) const;
            bool ValidateCandidateGeometry(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const TetClusterCandidate& ACandidate) const;
            bool HasValidTransformSpacing(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const std::vector<TetItemTransform>& ATransforms) const;
            bool CanPlaceCandidateCopiesOnBoard(const TetClusterCandidate& ACandidate, const TetNestOptions& AOptions, std::size_t ARequiredCopies) const;

        protected:
            bool _ValidateIndexAndTransforms(const CetTNestItemVector& AOriginalItems, const TetClusterCandidate& ACandidate) const;
            bool _ValidateChildContainment(const CetTNestItemVector& AOriginalItems, const TetClusterCandidate& ACandidate) const;
            bool _ValidateChildSpacing(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const TetClusterCandidate& ACandidate, bool ALogRejection) const;
            bool _HaveRequiredSpacing(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const TetItemTransform& AFirstTransform, const TetItemTransform& ASecondTransform) const;
            bool _FitsBoardBounds(const TetClusterCandidate& ACandidate, const TetNestOptions& AOptions) const;
            bool _BuildTransformedChildContours(const CetTNestItemVector& AOriginalItems, const TetClusterCandidate& ACandidate, ClipperLib::Paths& AOutContours) const;
            double _CalculateUnionArea(const ClipperLib::Paths& AContours) const;
            double _CalculateReservedArea(const ClipperLib::Paths& AChildContours, const TetNestOptions& AOptions, double AOccupiedArea) const;
            double _GetAreaTolerance(double AReferenceArea) const;
            bool _NormalizeContourForClipper(const CetPath& AInputContour, CetPath& AOutContour, double AAreaTolerance) const;
        };
    }
}
