#pragma once
#include "EtTechCore_Object.h"
#include "Nest2D_PrivateDataType.h"
#include "Nest2D_DataType.h"

#include <vector>

namespace ET {
    namespace NEST2DMANAGERLIB {

        class CetArcClusterBuilder : public ET::CORE::CetCoreObject
        {
            Inherit_Invoke_Hook(CetArcClusterBuilder)

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
            CetArcClusterBuilder();
            ~CetArcClusterBuilder();

        public:
            void BuildCandidates(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, std::vector<TetClusterCandidate>& AOutCandidates);

        protected:
            void _BuildCompatibleArcClusterCandidates(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, std::vector<TetClusterCandidate>& AOutCandidates);
            bool _BuildClusterCandidate(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate);
            bool _FitsBin(double AClusterWidth, double AClusterHeight, const TetNestOptions& AOptions);
            double _CalculateScore(const TetClusterCandidate& ACandidate, const TetNestOptions& AOptions);
        };

    }
}
