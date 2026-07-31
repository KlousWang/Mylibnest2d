#pragma once
#include "EtTechCore_Object.h"
#include "Nest2D_DataType.h"
#include <vector>

namespace ET {
    namespace NEST2DMANAGERLIB {

        class CetAreaUsageCalculator : public ET::CORE::CetCoreObject
        {
            Inherit_Invoke_Hook(CetAreaUsageCalculator)

        protected:
            int _Init() override {
                CetCoreObject::_Init();
                return 0;
            }

            void _WrapFuncs() override {
                CetCoreObject::_WrapFuncs();
                _WrapFunc("CalculateBoardUsages", Type_Class_Func(CalculateBoardUsages));
                _WrapFunc("CalcPolygonArea", Type_Class_Func(CalcPolygonArea));
                _WrapFunc("CalcBoardArea", Type_Class_Func(CalcBoardArea));
            }

        public:
            CetAreaUsageCalculator();
            ~CetAreaUsageCalculator();

            std::vector<TetBoardUsageResult> CalculateBoardUsages(const std::vector<TetNestPolygon>& AItems, const TetNestOptions& AOptions, int AUsedBins);
            double CalcPolygonArea(const TetNestPolygon& APoly);
            double CalcBoardArea(const TetNestOptions& AOptions);

        protected:
            double CalcPointArea(const std::vector<TetNestPoint>& APoints);
            double CalcNetArea(const std::vector<TetNestPoint>& AOuter, const std::vector<std::vector<TetNestPoint>>& AHoles);
        };
    }
}
