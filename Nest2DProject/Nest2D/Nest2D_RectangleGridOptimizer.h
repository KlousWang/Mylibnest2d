#pragma once
#include "EtTechCore_Object.h"
#include "Nest2D_DataType.h"
#include "Nest2D_PrivateDataType.h"

#include <map>
#include <vector>

namespace ET {
    namespace NEST2DMANAGERLIB {
        class CetRectangleGridOptimizer : public ET::CORE::CetCoreObject
        {
            Inherit_Invoke_Hook(CetRectangleGridOptimizer)

        protected:
            int _Init() override
            {
                CetCoreObject::_Init();
                return 0;
            }
            void _WrapFuncs() override
            {
                CetCoreObject::_WrapFuncs();
                _WrapFunc("ValidatePlacedItemsSpacing", Type_Class_Func(ValidatePlacedItemsSpacing));
                _WrapFunc("TryCompactUniformRectangleHoles", Type_Class_Func(TryCompactUniformRectangleHoles));
                _WrapFunc("TryFillRectangleGridEdgeFromCompatibleGroup", Type_Class_Func(TryFillRectangleGridEdgeFromCompatibleGroup));
                _WrapFunc("EvaluateInternalGapMetrics", Type_Class_Func(EvaluateInternalGapMetrics));
            }

        public:
            CetRectangleGridOptimizer();
            ~CetRectangleGridOptimizer();

            bool ValidatePlacedItemsSpacing(const CetTNestItemVector &AItems, const TetNestOptions &AOptions);
            bool TryCompactUniformRectangleHoles(CetTNestItemVector &AItems, const TetNestOptions &AOptions);
            bool TryFillRectangleGridEdgeFromCompatibleGroup(CetTNestItemVector &AItems, const TetNestOptions &AOptions);
            void EvaluateInternalGapMetrics(const CetTNestItemVector &AItems, const TetNestOptions &AOptions, TetTNestEvalResult &AInOutResult);

        protected:
            bool TryGetAxisAlignedRectangle(const CetNestItem &AItem, TetAxisAlignedRectangle &AOutRectangle);
            bool BuildRectangleGridGroup(const std::vector<TetAxisAlignedRectangle> &ARectangles, const TetNestOptions &AOptions, TetRectangleGridGroup &AOutGroup, bool ARequireMultipleRows = true);
            bool AreItemsInsideRectangleBoard(const CetTNestItemVector &AItems, const TetNestOptions &AOptions);
            std::size_t GetRectangleGridEdgeGapCount(const TetRectangleGridGroup &AGroup);
            bool IsRectangleGridEdgeFillCandidateValid(const CetTNestItemVector &ACandidate, const TetNestOptions &AOptions);
        };
    } // namespace NEST2DMANAGERLIB
} // namespace ET
