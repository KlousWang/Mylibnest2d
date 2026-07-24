#pragma once

#include "EtTechCore_Object.h"
#include "Nest2D_PrivateDataType.h"
#include "Nest2D_DataType.h"

#include <vector>

namespace ET {
    namespace NEST2DMANAGERLIB {

        class CetClusterBoundary : public ET::CORE::CetCoreObject
        {
            Inherit_Invoke_Hook(CetClusterBoundary)

        protected:
            int _Init() override
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
            bool BuildBoundary(const CetTNestItemVector& AOriginalItems, const std::vector<TetItemTransform>& ATransforms, CetPath& AOutBoundary);

        protected:
            CetPath _GetIdentityContour(const CetNestItem& AItem);
            CetPath _TransformContour(const CetPath& AContour, double ARotation, double ATranslationX, double ATranslationY);
            bool _CollectTransformedContours(const CetTNestItemVector& AOriginalItems, const std::vector<TetItemTransform>& ATransforms, ClipperLib::Paths& AOutContours);
            bool _BuildUnionBoundary(const ClipperLib::Paths& AContours, CetPath& AOutBoundary);
            bool _BuildConvexHullBoundary(const ClipperLib::Paths& AContours, CetPath& AOutBoundary);
            std::vector<CetInpoint> _CollectContourPoints(const ClipperLib::Paths& AContours);
            bool _SortUniquePoints(std::vector<CetInpoint>& APoints);
            bool _BuildConvexHullFromPoints(const std::vector<CetInpoint>& APoints, CetPath& AOutBoundary);
            bool _NormalizeBoundary(CetPath& ABoundary);
            long double _CrossProduct(const CetInpoint& AOrigin, const CetInpoint& AFirstPoint, const CetInpoint& ASecondPoint);
        };

    }
}
