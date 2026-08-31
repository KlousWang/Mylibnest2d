#pragma once
#include "EtTechCore_Object.h"
#include "Nest2D_DataType.h"
#include "Nest2D_PrivateDataType.h"

#include <vector>

namespace ET {
    namespace NEST2DMANAGERLIB {
        class CetQuarterTurnOptimizer :public ET::CORE::CetCoreObject
        {
            Inherit_Invoke_Hook(CetQuarterTurnOptimizer) protected : int _Init() override
            {
                CetCoreObject::_Init();
                return 0;
            }
            void _WrapFuncs() override
            {
                CetCoreObject::_WrapFuncs();
                _WrapFunc("CollectQuarterTurnTargets", Type_Class_Func(CollectQuarterTurnTargets));
                _WrapFunc("BuildQuarterTurnCoordinates", Type_Class_Func(BuildQuarterTurnCoordinates));
                _WrapFunc("CanPlaceQuarterTurnTarget", Type_Class_Func(CanPlaceQuarterTurnTarget));
                _WrapFunc("ApplyQuarterTurnPassBest", Type_Class_Func(ApplyQuarterTurnPassBest));
            }

        public:
            CetQuarterTurnOptimizer();
            ~CetQuarterTurnOptimizer();

            std::vector<TetQuarterTurnTarget> CollectQuarterTurnTargets(const CetTNestItemVector &AItems, const std::vector<TetMetaItem> &AMetaItems, libnest2d::Coord ASpacing);
            TetQuarterTurnCoordinates BuildQuarterTurnCoordinates(const TetQuarterTurnCoordinateRequest &ARequest);
            bool CanPlaceQuarterTurnTarget(const CetTNestItemVector &AItems, std::size_t ATargetIndex, libnest2d::Coord ABinWidth, libnest2d::Coord ABinHeight, libnest2d::Coord AHalfSpacing);
            void ApplyQuarterTurnPassBest(TetLocalBestResult &AInOutBest, TetTNestEvalResult &&AEval, std::size_t ALayers, CetTNestItemVector &&AItems, std::vector<TetMetaItem> &&AMetaItems, bool AHasCluster);
        };
    } // namespace NEST2DMANAGERLIB
}

