#pragma once
#include "EtTechCore_Object.h"
#include "Nest2D_DataType.h"
#include "Nest2D_PrivateDataType.h"
#include <map>
#include <vector>

namespace ET { namespace NEST2DMANAGERLIB {

class CetClusterTemplateFillOptimizer : public ET::CORE::CetCoreObject {
    Inherit_Invoke_Hook(CetClusterTemplateFillOptimizer)

protected:
    int _Init() override { CetCoreObject::_Init(); return 0; }
    void _WrapFuncs() override {
        CetCoreObject::_WrapFuncs();
        _WrapFunc("BuildTemplateClusters", Type_Class_Func(BuildTemplateClusters));
    }

public:
    CetClusterTemplateFillOptimizer();
    ~CetClusterTemplateFillOptimizer();

    TetClusterBuildResult BuildTemplateClusters(const CetTNestItemVector&, const std::vector<TetShapeFeature>&, const TetNestOptions&);

protected:
    void _CollectTemplateShapeIndices(const std::vector<TetShapeFeature>&, std::map<MetShapeType, std::vector<int>>&);
    void _BuildTemplateCandidates(const CetTNestItemVector&, const std::vector<TetShapeFeature>&, const TetNestOptions&, const std::map<MetShapeType, std::vector<int>>&, std::vector<TetClusterCandidate>&);
    void _BuildFilledTemplateCandidateVariants(const CetTNestItemVector&, const std::vector<TetShapeFeature>&, const TetNestOptions&, const std::vector<TetClusterCandidate>&, std::vector<TetClusterCandidate>&);
    std::vector<TetClusterCandidate> _SelectTemplateCandidates(const CetTNestItemVector&, const std::vector<TetShapeFeature>&, const TetNestOptions&, const std::vector<TetClusterCandidate>&, std::vector<bool>&);
    std::vector<TetClusterCandidate> _SelectAndOptimizeTemplateCandidates(const CetTNestItemVector&, const std::vector<TetShapeFeature>&, const TetNestOptions&, const std::vector<TetClusterCandidate>&, std::vector<bool>&, int);
    int _OptimizePairClusterSelection(const std::vector<TetClusterCandidate>&, std::vector<TetClusterCandidate>&, int);
    double _CalculateCandidateSelectionScore(const std::vector<TetClusterCandidate>&);
    bool _ValidateClusterSelection(const std::vector<TetClusterCandidate>&, int);
    bool _CanAcceptClusterCandidate(const CetTNestItemVector&, const TetNestOptions&, const TetClusterCandidate&, const std::vector<bool>&, int);
};

}}
