#pragma once

#include "EtTechCore_Object.h"
#include "Nest2D_DataType.h"
#include "Nest2D_PrivateDataType.h"

#include <vector>

namespace ET {
    namespace NEST2DMANAGERLIB {

        class CetClusterBoundary : public ET::CORE::CetCoreObject
        {
        Inherit_Invoke_Hook(CetClusterBoundary)

            protected : int _Init() override
            {
                CetCoreObject::_Init();
                return 0;
            }

            void _WrapFuncs() override
            {
                CetCoreObject::_WrapFuncs();
                _WrapFunc("BuildBoundary", Type_Class_Func(BuildBoundary));
            }

        public:
            CetClusterBoundary();
            ~CetClusterBoundary();

        public:
            bool BuildBoundary(const CetTNestItemVector &AOriginalItems, const std::vector<TetItemTransform> &ATransforms, CetPath &AOutBoundary);
            bool BuildBoundaryWithResult(const CetTNestItemVector &AOriginalItems, const std::vector<TetItemTransform> &ATransforms, const TetNestOptions &AOptions, TetClusterBoundaryResult &AOutResult);

        protected:
            bool _CollectTransformedContours(const CetTNestItemVector &AOriginalItems, const std::vector<TetItemTransform> &ATransforms, ClipperLib::Paths &AOutContours);
            bool _BuildExactUnionBoundary(const ClipperLib::Paths &AContours, CetPath &AOutBoundary);
            bool _BuildOffsetUnionBoundary(const ClipperLib::Paths &AContours, const TetNestOptions &AOptions, CetPath &AOutBoundary);
            bool _BuildConvexHullBoundary(const ClipperLib::Paths &AContours, CetPath &AOutBoundary);
            bool _BuildSingleOuterBoundary(const ClipperLib::Paths &AContours, CetPath &AOutBoundary);
            bool _BoundaryContainsAllContours(const ClipperLib::Paths &AContours, const CetPath &ABoundary, double AAreaTolerance);
            bool _GetContourBounds(const ClipperLib::Paths &AContours, double &AOutMinX, double &AOutMinY, double &AOutMaxX, double &AOutMaxY);
            double _CalculateBoundaryArea(const CetPath &ABoundary);
            double _GetContainmentAreaTolerance(const CetPath &ABoundary);
            std::vector<CetInpoint> _CollectContourPoints(const ClipperLib::Paths &AContours);
            bool _SortUniquePoints(std::vector<CetInpoint> &APoints);
            bool _BuildConvexHullFromPoints(const std::vector<CetInpoint> &APoints, CetPath &AOutBoundary);
            bool _NormalizeBoundary(CetPath &ABoundary);
            long double _CrossProduct(const CetInpoint &AOrigin, const CetInpoint &AFirstPoint, const CetInpoint &ASecondPoint);
        };

    } // namespace NEST2DMANAGERLIB
} // namespace ET
