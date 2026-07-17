#pragma once
#include "EtTechCore_Object.h"
#include "Nest2D_PrivateDataType.h"
#include <vector>

namespace ET {
    namespace NEST2DMANAGERLIB {

        class CetShapeAnalyzer : public ET::CORE::CetCoreObject {
            Inherit_Invoke_Hook(CetShapeAnalyzer)
        protected:
            int _Init() override { CetCoreObject::_Init(); return 0; }
            void _WrapFuncs() override {
                CetCoreObject::_WrapFuncs();
                _WrapFunc("AnalyzeALL", Type_Class_Func(AnalyzeALL));
            }
        public:
            CetShapeAnalyzer();
            ~CetShapeAnalyzer();
            std::vector<TetShapeFeature> AnalyzeALL(const CetTNestItemVector& AItems);
        protected:
            TetShapeFeature _AnalyzeOne(const CetNestItem& AItem, int AOriginalIndex);
            void _AnalyzeTriangleFeature(const CetPath& AContour, TetShapeFeature& AFeature);
            void _AnalyzeRectangleFeature(const CetPath& AContour, TetShapeFeature& AFeature);
            void _AnalyzeArcFeature(const CetPath& AContour, TetShapeFeature& AFeature);
            void _AnalyzeEllipseFeature(const CetPath& AContour, TetShapeFeature& AFeature);
            MetShapeType _ClassifyShape(const TetShapeFeature& AFeature);
            double _Distance(const ClipperLib::IntPoint& AA, const ClipperLib::IntPoint& AB);
            double _AngleAtVertex(const ClipperLib::IntPoint& APrevious,const ClipperLib::IntPoint& ACurrent,const ClipperLib::IntPoint& ANext);
            double _CalculatePerimeter(const CetPath& AContour);
            bool _IsConvex(const CetPath& AContour);
            void _NormalizePath(CetPath& APath);
        };

    }
}


