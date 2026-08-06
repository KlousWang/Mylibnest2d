#pragma once

#include "EtTechCore_Object.h"
#include "Nest2D_PrivateDataType.h"
#include "Nest2D_DataType.h"

#include <vector>

namespace ET {
    namespace NEST2DMANAGERLIB {

        class CetCustomClusterBuilder : public ET::CORE::CetCoreObject
        {
            Inherit_Invoke_Hook(CetCustomClusterBuilder)

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
            CetCustomClusterBuilder();
            ~CetCustomClusterBuilder();

        public:
            void BuildCandidates(const CetTNestItemVector& AOriginalItems,const std::vector<TetShapeFeature>& AFeatures,const std::vector<int>& AIndices,const TetNestOptions& AOptions,std::vector<TetClusterCandidate>& AOutCandidates);

        protected:
            void _BuildSameShapeClusterCandidates(const CetTNestItemVector& AOriginalItems,const std::vector<TetShapeFeature>& AFeatures,const std::vector<int>& AIndices,const TetNestOptions& AOptions,std::vector<TetClusterCandidate>& AOutCandidates);
            bool _BuildMixedShapeCandidate(const CetTNestItemVector& AOriginalItems,const std::vector<TetShapeFeature>& AFeatures,const std::vector<int>& AIndices,const TetNestOptions& AOptions,TetClusterCandidate& AOutCandidate);
            bool _FindLargestBoardFitLayout(const CetTNestItemVector& AOriginalItems,const std::vector<int>& AIndices,const TetNestOptions& AOptions,std::size_t& AOutChildCount,TetClusterCandidate& AOutCandidate);
            bool _BuildBestEdgePairCandidate(const CetTNestItemVector& AOriginalItems,const std::vector<int>& AIndices,const TetNestOptions& AOptions,TetClusterCandidate& AOutCandidate);
            bool _BuildBestLayoutCandidate(const CetTNestItemVector& AOriginalItems,const std::vector<int>& AIndices,const TetNestOptions& AOptions,TetClusterCandidate& AOutCandidate);
            bool _RemapCandidateIndices(const TetClusterCandidate& ASourceCandidate,const std::vector<int>& AIndices,TetClusterCandidate& AOutCandidate);
            bool _IsSupportedCustomShape(const TetShapeFeature& AFeature);
            double _CalculateScore(const TetClusterCandidate& ACandidate,const TetNestOptions& AOptions);
        };

    }
}
