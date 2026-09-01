#pragma once

#include "EtTechCore_Object.h"
#include "Nest2D_PrivateDataType.h"

#include <set>
#include <utility>
#include <vector>

namespace ET { namespace NEST2DMANAGERLIB {

class CetClusterInventoryRebalancer : public ET::CORE::CetCoreObject {
    Inherit_Invoke_Hook(CetClusterInventoryRebalancer)

protected:
    int _Init() override { CetCoreObject::_Init(); return 0; }
    void _WrapFuncs() override { CetCoreObject::_WrapFuncs(); }

public:
    CetClusterInventoryRebalancer();
    ~CetClusterInventoryRebalancer();

    bool TryBindCandidateInventory(const TetClusterCandidate &, const std::vector<TetShapeFeature> &, const std::vector<bool> &, TetClusterCandidate &);
    void RebalanceAcceptedClusterInventory(const CetTNestItemVector &, const std::vector<TetShapeFeature> &, const TetNestOptions &, std::vector<TetClusterCandidate> &, std::vector<bool> &);

protected:
    bool _HasValidCandidateInventory(const TetClusterCandidate &, const std::vector<TetShapeFeature> &, const std::vector<bool> &);
    int _FindAvailableFamilyItem(const std::vector<TetShapeFeature> &, const std::vector<bool> &, const std::set<int> &, int);
    bool _IsRebalanceOrderBetter(const TetClusterCandidate &, const TetClusterCandidate &);
    bool _IsInventoryRebalanceCandidate(const TetClusterCandidate &);
    void _PreserveInventoryProxy(const TetClusterCandidate &, TetClusterCandidate &);
    bool _TryAppendInventoryFiller(const CetTNestItemVector &, const std::vector<TetShapeFeature> &, const TetNestOptions &, const TetClusterCandidate &, int, TetClusterCandidate &);
    bool _TryRemoveInventoryFiller(const CetTNestItemVector &, const TetNestOptions &, const TetClusterCandidate &, std::size_t, TetClusterCandidate &);
    bool _IsTriangleBuilderCandidate(const TetClusterCandidate &);
    bool _HasExactCandidateInventory(const TetClusterCandidate &, const std::vector<int> &);
    bool _TryBuildReducedTriangleCandidate(const CetTNestItemVector &, const std::vector<TetShapeFeature> &, const TetNestOptions &, const TetClusterCandidate &, const std::set<int> &, TetClusterCandidate &);
    bool _TryTransferTrianglePair(const TetClusterFillContext &, TetClusterCandidate &, TetClusterCandidate &, std::pair<int, int> &);
    bool _IsTransferPairGloballyUnique(const std::vector<TetClusterCandidate> &, std::size_t, const std::pair<int, int> &);
    bool _IsTriangleTransferWorthKeeping(const TetClusterCandidate &, const TetClusterCandidate &, const TetClusterCandidate &, const TetClusterCandidate &);
    bool _IsInventoryTransferWorthKeeping(const TetClusterCandidate &, const TetClusterCandidate &, const TetClusterCandidate &, const TetClusterCandidate &);
    bool _TryBuildInventorySkeletonEnvelope(const CetTNestItemVector &, const TetNestOptions &, const TetClusterCandidate &, TetClusterCandidate &);
};

}}
