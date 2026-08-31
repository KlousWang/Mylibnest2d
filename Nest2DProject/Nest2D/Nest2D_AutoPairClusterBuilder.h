#pragma once
#include "EtTechCore_Object.h"
#include "Nest2D_DataType.h"
#include "Nest2D_PrivateDataType.h"

#include <vector>

namespace ET { namespace NEST2DMANAGERLIB {

class CetAutoPairClusterBuilder : public ET::CORE::CetCoreObject {
    Inherit_Invoke_Hook(CetAutoPairClusterBuilder)

protected:
    int _Init() override { CetCoreObject::_Init(); return 0; }
    void _WrapFuncs() override {
        CetCoreObject::_WrapFuncs();
        _WrapFunc("BuildAutoPairClusters", Type_Class_Func(BuildAutoPairClusters));
    }

public:
    CetAutoPairClusterBuilder();
    ~CetAutoPairClusterBuilder();

    TetClusterBuildResult BuildAutoPairClusters(const CetTNestItemVector& AItems, const TetNestOptions& AOptions);

protected:
    bool _TryFindBestEdgePairCandidate(const CetTNestItemVector&, int, int, const TetNestOptions&, TetAutoPairCandidate&);
    bool _TryFindBestAutoPairCandidate(const CetTNestItemVector&, int, int, const TetNestOptions&, TetAutoPairCandidate&);
    bool _TryBuildAutoPairAt(const CetTNestItemVector&, const TetNestOptions&, const TetAutoPairBuildInput&, TetAutoPairCandidate&);
    void _AddAutoPairCluster(const CetTNestItemVector&, const TetNestOptions&, const TetAutoPairCandidate&, TetClusterBuildResult&);
    double _CalcAutoPairScore(double, double, double, double, double);
    bool _RunAutoPairGridSearch(const CetTNestItemVector&, int, int, const TetNestOptions&, const TetAutoPairGridConfig&, TetAutoPairCandidate&);
    CetNestItem _MakeUnionNestItemFromCandidate(const CetTNestItemVector&, const TetNestOptions&, const TetAutoPairCandidate&);
    double _CalcEdgeLength(const ClipperLib::IntPoint&, const ClipperLib::IntPoint&);
    std::vector<TetEdgeInfo> _CollectEdges(const ClipperLib::Path&);
    bool _IsSimilarTriangleByEdges(std::vector<TetEdgeInfo>, std::vector<TetEdgeInfo>);
    bool _SnapToAllowedRotation(double, int, double&);
    bool _EvaluateEdgePair(const TetEdgePairContext&, const TetEdgeInfo&, const TetEdgeInfo&, TetAutoPairCandidate&);
    bool _TestEdgeOffsets(const TetEdgePairContext&, const TetEdgeMatchState&, const TetEdgeInfo&, TetAutoPairCandidate&);
    bool _RunGridSearchAllAngles(const TetAutoPairContext&, const std::vector<double>&, TetAutoPairCandidate&);
    bool _EvaluateRotationPair(const TetAutoPairContext&, double, double, TetAutoPairCandidate&);

private:
    void _AddSingleItem(const CetTNestItemVector&, int, TetClusterBuildResult&);
    double _GetItemWidth(const CetNestItem&);
    double _GetItemHeight(const CetNestItem&);
};

}}


