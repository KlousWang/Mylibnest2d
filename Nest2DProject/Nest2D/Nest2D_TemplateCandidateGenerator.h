#pragma once
#include "EtTechCore_Object.h"
#include "Nest2D_PrivateDataType.h"
#include <cstddef>
#include <map>
#include <vector>
namespace ET {
    namespace NEST2DMANAGERLIB {
        class CetTemplateCandidateGenerator : public ET::CORE::CetCoreObject
        {
        Inherit_Invoke_Hook(CetTemplateCandidateGenerator) protected : int _Init() override
            {
                CetCoreObject::_Init();
                return 0;
            }
            void _WrapFuncs() override {
                CetCoreObject::_WrapFuncs(); 
            }

        public:
            CetTemplateCandidateGenerator();
            ~CetTemplateCandidateGenerator();
            void CollectTemplateShapeIndices(const std::vector<TetShapeFeature> &, std::map<MetShapeType, std::vector<int>> &);
            void BuildTemplateCandidates(const CetTNestItemVector &, const std::vector<TetShapeFeature> &, const TetNestOptions &, const std::map<MetShapeType, std::vector<int>> &, std::vector<TetClusterCandidate> &);
        };
    } // namespace NEST2DMANAGERLIB
} // namespace ET
