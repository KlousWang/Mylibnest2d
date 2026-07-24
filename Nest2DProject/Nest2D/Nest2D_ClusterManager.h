#pragma once

#include "EtTechCore_Object.h"
#include "NestUtils.h"
#include "Nest2D_PrivateDataType.h"
#include "Nest2D_DataType.h"

#include <vector>
#include <string>
#include <map>

namespace ET {
    namespace NEST2DMANAGERLIB {

        class CetClusterManager : public ET::CORE::CetCoreObject {
            Inherit_Invoke_Hook(CetClusterManager)

        protected:
            int _Init() override { CetCoreObject::_Init(); return 0; }
            void _WrapFuncs() override {
                CetCoreObject::_WrapFuncs();
                _WrapFunc("BuildClusterItems", Type_Class_Func(BuildClusterItems));
                _WrapFunc("ExpandClusterResultToOriginalItems", Type_Class_Func(ExpandClusterResultToOriginalItems));
                _WrapFunc("BuildClusterItemsWithFeatures", Type_Class_Func(BuildClusterItemsWithFeatures));
            }

        public:
            CetClusterManager();
            ~CetClusterManager();

        public:
            TetClusterBuildResult BuildClusterItems(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, MetClusterStrategy AStrategy);
            TetClusterBuildResult BuildClusterItemsWithFeatures(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions, MetClusterStrategy AStrategy);
            void ExpandClusterResultToOriginalItems(const CetTNestItemVector& AOriginalItems, const CetTNestItemVector& APackedItems, const std::vector<TetMetaItem>& AMetaItems, CetTNestItemVector& AOutOriginalItems);

        protected:
            TetClusterBuildResult _BuildTemplateClusters(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions);
            TetClusterBuildResult _BuildAllSingles(const CetTNestItemVector& AOriginalItems);
            bool _ValidateBuildResultCoverage(const TetClusterBuildResult& AResult, int AOriginalCount);
            void _AddSingleItem(const CetTNestItemVector& AOriginalItems, int AOriginalIndex, TetClusterBuildResult& AResult);
            bool _CanAcceptClusterCandidate(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const TetClusterCandidate& ACandidate, const std::vector<bool>& AUsed, int AOriginalCount);
            CetNestItem _MakeClusterProxyItem(const TetClusterCandidate& ACandidate);
            bool _AddClusterCandidate(const TetClusterCandidate& ACandidate, TetClusterBuildResult& AResult);
            void _ExpandClusterChildren(const CetNestItem& PackedItem, const TetMetaItem& Meta, CetTNestItemVector& AOutOriginalItems);
            double _GetItemWidth(const CetNestItem& AItem);
            double _GetItemHeight(const CetNestItem& AItem);
            TetClusterBuildResult _BuildAutoPairClusters(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions);
            bool _TryFindBestEdgePairCandidate(const CetTNestItemVector& AOriginalItems, int AIndex, int BIndex, const TetNestOptions& AOptions, TetAutoPairCandidate& ABestCandidate);
            bool _TryFindBestAutoPairCandidate(const CetTNestItemVector& AOriginalItems, int AIndex, int BIndex, const TetNestOptions& AOptions, TetAutoPairCandidate& ABestCandidate);
            bool _TryBuildAutoPairAt(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const TetAutoPairBuildInput& AInput, TetAutoPairCandidate& ACandidate);
            void _AddAutoPairCluster(const CetTNestItemVector& AOriginalItems, const TetAutoPairCandidate& ACandidate, TetClusterBuildResult& AResult);
            double _CalcAutoPairScore(double ABeforeBBoxArea, double AAfterBBoxArea, double ARealArea, double AClusterW, double AClusterH);
            bool _RunAutoPairGridSearch(const CetTNestItemVector& AOriginalItems, int AIndex, int BIndex, const TetNestOptions& AOptions, const TetAutoPairGridConfig& AConfig, TetAutoPairCandidate& OutBest);
            CetNestItem _MakeUnionNestItemFromCandidate(const CetTNestItemVector& AOriginalItems, const TetAutoPairCandidate& ACandidate);
            double _CalcEdgeLength(const ClipperLib::IntPoint& A, const ClipperLib::IntPoint& B);
            std::vector<TetEdgeInfo> _CollectEdges(const ClipperLib::Path& AContour);
            bool _IsSimilarTriangleByEdges(std::vector<TetEdgeInfo> AEdges, std::vector<TetEdgeInfo> BEdges);
            bool _SnapToAllowedRotation(double ATarget, int ARotations, double& AOutRotation);
            bool _EvaluateEdgePair(const TetEdgePairContext& ctx, const TetEdgeInfo& EdgeA, const TetEdgeInfo& EdgeB, TetAutoPairCandidate& ABestCandidate);
            bool _TestEdgeOffsets(const TetEdgePairContext& ctx, const TetEdgeMatchState& state, const TetEdgeInfo& EdgeA, TetAutoPairCandidate& ABestCandidate);
            bool _RunGridSearchAllAngles(const TetAutoPairContext& ctx, const std::vector<double>& rotations, TetAutoPairCandidate& ABestCandidate);
            bool _EvaluateRotationPair(const TetAutoPairContext& ctx, double ARot, double BRot, TetAutoPairCandidate& ABestCandidate);
           //long long MakeRelativeSizeBucket(double Value);
        };
    }
}
