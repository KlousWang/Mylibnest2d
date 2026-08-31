#pragma once
#include "EtTechCore_Object.h"
#include "Nest2D_DataType.h"
#include "Nest2D_PrivateDataType.h"
#include <vector>
namespace ET { namespace NEST2DMANAGERLIB {
class CetFreeSpaceEvaluator : public ET::CORE::CetCoreObject {
    Inherit_Invoke_Hook(CetFreeSpaceEvaluator)
protected:
    int _Init() override { CetCoreObject::_Init(); return 0; }
    void _WrapFuncs() override {
        CetCoreObject::_WrapFuncs();
        _WrapFunc("EvaluateBoardFreeRegionMetrics", Type_Class_Func(EvaluateBoardFreeRegionMetrics));
        _WrapFunc("EvaluatePassableFreeRegionMetrics", Type_Class_Func(EvaluatePassableFreeRegionMetrics));
        _WrapFunc("PreservesPassableFreeSpace", Type_Class_Func(PreservesPassableFreeSpace));
        _WrapFunc("IsBetterContinuousFreeSpace", Type_Class_Func(IsBetterContinuousFreeSpace));
    }
public:
    CetFreeSpaceEvaluator(); ~CetFreeSpaceEvaluator();
    void EvaluateBoardFreeRegionMetrics(const CetTNestItemVector &, const TetNestOptions &, TetTNestEvalResult &);
    void EvaluatePassableFreeRegionMetrics(const CetTNestItemVector &, const TetNestOptions &, TetTNestEvalResult &);
    bool PreservesPassableFreeSpace(const TetTNestEvalResult &, const TetTNestEvalResult &);
    bool IsBetterContinuousFreeSpace(const TetTNestEvalResult &, const TetTNestEvalResult &);
};
}}
