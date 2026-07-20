#pragma once
#include "EtTechCore_Object.h"
#include "Nest2D_PrivateDataType.h"
#include "Nest2D_DataType.h"

namespace ET {
    namespace NEST2DMANAGERLIB {

        class CetRectangleClusterBuilder : public ET::CORE::CetCoreObject {
            Inherit_Invoke_Hook(CetRectangleClusterBuilder)
        protected:
            int _Init() override { CetCoreObject::_Init(); return 0; }
            void _WrapFuncs() override { CetCoreObject::_WrapFuncs();
            _WrapFunc("BuildCandidates", Type_Class_Func(BuildCandidates)); 
            }
        public:
            CetRectangleClusterBuilder();
            ~CetRectangleClusterBuilder();

            void BuildCandidates(const CetTNestItemVector& AItems,const std::vector<TetShapeFeature>& AFeatures,const std::vector<int>& AIndices,const TetNestOptions& AOptions,std::vector<TetClusterCandidate>& AOut);

        protected:
            bool _IsValidRectangle(const TetShapeFeature& AFeature);
            bool _AreCompatible(const TetShapeFeature& AFeatureA, const TetShapeFeature& AFeatureB);
            bool _IsSquareLike(const TetShapeFeature& AFeature);
            bool _IsQuarterTurnAllowed(const TetNestOptions& AOptions);          
            bool _MakePairCandidate(const CetTNestItemVector& AItems, const std::vector<TetShapeFeature>& AFeatures, int AIndexA, int AIndexB, bool AHorizontal, bool ARotateB90, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate);

        };

    }
}