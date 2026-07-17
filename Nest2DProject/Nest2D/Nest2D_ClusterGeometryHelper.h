#pragma once

#include "EtTechCore_Object.h"
#include "Nest2D_PrivateDataType.h"
#include "Nest2D_DataType.h"

namespace ET {
    namespace NEST2DMANAGERLIB {
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
            bool FinalizeCandidate(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, TetClusterCandidate& ACandidate) const;
            bool ValidateCandidateGeometry(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const TetClusterCandidate& ACandidate) const;

        protected:
            bool _ValidateIndexAndTransforms(const CetTNestItemVector& AOriginalItems, const TetClusterCandidate& ACandidate) const;
            bool _ValidateChildContainment(const CetTNestItemVector& AOriginalItems, const TetClusterCandidate& ACandidate) const;
            bool _ValidateChildSpacing(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const TetClusterCandidate& ACandidate) const;
            bool _FitsBoardBounds(const TetClusterCandidate& ACandidate, const TetNestOptions& AOptions) const;
        };
    }
}
