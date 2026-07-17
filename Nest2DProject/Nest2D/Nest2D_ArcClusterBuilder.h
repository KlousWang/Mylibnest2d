#pragma once
#include "EtTechCore_Object.h"
#include "Nest2D_PrivateDataType.h"
#include "Nest2D_DataType.h"

namespace ET {
    namespace NEST2DMANAGERLIB {

        class CetArcClusterBuilder : public ET::CORE::CetCoreObject {
            Inherit_Invoke_Hook(CetArcClusterBuilder)
        protected:
            int _Init() override { CetCoreObject::_Init(); return 0; }
            void _WrapFuncs() override { CetCoreObject::_WrapFuncs();
            _WrapFunc("BuildCandidates", Type_Class_Func(BuildCandidates));
            }
        public:
            CetArcClusterBuilder();
            ~CetArcClusterBuilder();

            // 统一替换为以大写 A 开头的传参名
            void BuildCandidates(const CetTNestItemVector& AItems,const std::vector<TetShapeFeature>& AFeatures,const std::vector<int>& AIndices,const TetNestOptions& AOptions,std::vector<TetClusterCandidate>& AOut);
        };

    }
}