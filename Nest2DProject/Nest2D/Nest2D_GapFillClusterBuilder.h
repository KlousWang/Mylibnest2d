#pragma once

#include "EtTechCore_Object.h"
#include "Nest2D_PrivateDataType.h"
#include "Nest2D_DataType.h"

#include <vector>

namespace ET {
    namespace NEST2DMANAGERLIB {

        struct TetGapFillCollisionItem;

        class CetGapFillClusterBuilder : public ET::CORE::CetCoreObject
        {
            Inherit_Invoke_Hook(CetGapFillClusterBuilder)

        protected:
            int _Init() override
            {
                CetCoreObject::_Init();
                return 0;
            }

            void _WrapFuncs() override
            {
                CetCoreObject::_WrapFuncs();
                _WrapFunc("BuildCandidates", Type_Class_Func(BuildCandidates));
            }

        public:
            CetGapFillClusterBuilder();
            ~CetGapFillClusterBuilder();

        public:
            void BuildCandidates(const CetTNestItemVector& AOriginalItems,const std::vector<TetShapeFeature>& AFeatures,const std::vector<TetClusterCandidate>& ABaseCandidates,const TetNestOptions& AOptions,std::vector<TetClusterCandidate>& AOutCandidates);
            bool BuildCandidateForBase(const CetTNestItemVector& AOriginalItems,const std::vector<TetShapeFeature>& AFeatures,const TetClusterCandidate& ABaseCandidate,const TetNestOptions& AOptions,const std::vector<bool>& AUsed,TetClusterCandidate& AOutCandidate);

        protected:
            bool _BuildGapFillCandidate(const CetTNestItemVector& AOriginalItems,const std::vector<TetShapeFeature>& AFeatures,const TetClusterCandidate& ABaseCandidate,const TetNestOptions& AOptions,const std::vector<bool>* AUsed,TetClusterCandidate& AOutCandidate);
            bool _TryAddFiller(const CetTNestItemVector& AOriginalItems,const std::vector<TetShapeFeature>& AFeatures,const TetClusterCandidate& ABaseCandidate,const std::vector<TetGapFillCollisionItem>& ABaseCollisionItems,int AFillerIndex,const TetNestOptions& AOptions,TetClusterCandidate& AOutCandidate);
            bool _IsSupportedBaseCandidate(const TetClusterCandidate& ACandidate);
            bool _CanUseAsFiller(const TetShapeFeature& AFeature,const TetClusterCandidate& ABaseCandidate);
            bool _ContainsOriginalIndex(const TetClusterCandidate& ACandidate,int AOriginalIndex);
            double _CalculateScore(const TetClusterCandidate& ACandidate);
        };
    }
}
