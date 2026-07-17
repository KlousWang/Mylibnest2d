#pragma once

#include "EtTechCore_Object.h"
#include "Nest2D_PrivateDataType.h"
#include "Nest2D_DataType.h"

#include <vector>

namespace ET {
    namespace NEST2DMANAGERLIB {

        class CetCircleClusterBuilder : public ET::CORE::CetCoreObject
        {
            Inherit_Invoke_Hook(CetCircleClusterBuilder)

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
            CetCircleClusterBuilder();
            ~CetCircleClusterBuilder();

        public:
            void BuildCandidates(const CetTNestItemVector& AOriginalItems,const std::vector<TetShapeFeature>& AFeatures,const std::vector<int>& AIndices,const TetNestOptions& AOptions,std::vector<TetClusterCandidate>& AOutCandidates);

        protected:
            void _BuildPairCandidates(const CetTNestItemVector& AOriginalItems,const std::vector<TetShapeFeature>& AFeatures,const std::vector<int>& AIndices,const TetNestOptions& AOptions,std::vector<TetClusterCandidate>& AOutCandidates);
            void _BuildBlock4Candidates(const CetTNestItemVector& AOriginalItems,const std::vector<TetShapeFeature>& AFeatures,const std::vector<int>& AIndices,const TetNestOptions& AOptions,std::vector<TetClusterCandidate>& AOutCandidates);
            bool _BuildPairCandidate(const CetTNestItemVector& AOriginalItems,const std::vector<TetShapeFeature>& AFeatures,int AAIndex,int ABIndex,const TetNestOptions& AOptions,TetClusterCandidate& AOutCandidate);
            bool _BuildBlock4Candidate(const CetTNestItemVector& AOriginalItems,const std::vector<TetShapeFeature>& AFeatures,int AIndex0,int AIndex1,int AIndex2,int AIndex3,const TetNestOptions& AOptions,TetClusterCandidate& AOutCandidate);
            bool _FitsBin(double AClusterWidth,double AClusterHeight,const TetNestOptions& AOptions);
            double _CalculateScore(const TetClusterCandidate& ACandidate);
        };
    }
}