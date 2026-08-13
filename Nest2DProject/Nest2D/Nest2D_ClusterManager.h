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
            std::vector<int> RankBoardCompositeSkeletons(const CetTNestItemVector& AItems, const std::vector<TetShapeFeature>& AFeatures, int ATargetBin, const TetClusterFreeRegion& AFreeRegion, std::size_t AMaxCount) const;
            std::vector<int> RankExistingBoardCompositeSkeletons(const CetTNestItemVector& AItems, const std::vector<TetShapeFeature>& AFeatures, int ATargetBin, const TetClusterFreeRegion& AFreeRegion, std::size_t AMaxCount) const;
            TetClusterBuildResult BuildClusterItems(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, MetClusterStrategy AStrategy);
            TetClusterBuildResult BuildClusterItemsWithFeatures(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions, MetClusterStrategy AStrategy);
            void ExpandClusterResultToOriginalItems(const CetTNestItemVector& AOriginalItems, const CetTNestItemVector& APackedItems, const std::vector<TetMetaItem>& AMetaItems, CetTNestItemVector& AOutOriginalItems);
            bool ValidatePackedResultSpacing(const CetTNestItemVector& AOriginalItems, const CetTNestItemVector& APackedItems, const std::vector<TetMetaItem>& AMetaItems, const TetNestOptions& AOptions, TetExpandedSpacingFailure* AOutFailure = nullptr);

        protected:
            TetClusterBuildResult _BuildTemplateClusters(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions);
            void _CollectTemplateShapeIndices(const std::vector<TetShapeFeature>& AFeatures, std::map<MetShapeType, std::vector<int>>& AIndicesByType);
            void _BuildTemplateCandidates(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions, const std::map<MetShapeType, std::vector<int>>& AIndicesByType, std::vector<TetClusterCandidate>& ABaseCandidates);
            void _BuildFilledTemplateCandidateVariants(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions, const std::vector<TetClusterCandidate>& ABaseCandidates, std::vector<TetClusterCandidate>& AOutCandidates);
            std::vector<TetClusterCandidate> _SelectTemplateCandidates(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const std::vector<TetClusterCandidate>& ABaseCandidates, std::vector<bool>& AUsed);
            std::vector<TetClusterCandidate> _SelectAndOptimizeTemplateCandidates(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const std::vector<TetClusterCandidate>& ABaseCandidates, std::vector<bool>& AUsed, int AOriginalItemCount);
            int _OptimizePairClusterSelection(const std::vector<TetClusterCandidate>& AAllCandidates, std::vector<TetClusterCandidate>& AAcceptedCandidates, int AOriginalItemCount);
            double _CalculateCandidateSelectionScore(const std::vector<TetClusterCandidate>& ACandidates);
            bool _ValidateClusterSelection(const std::vector<TetClusterCandidate>& ACandidates, int AOriginalItemCount);
           // void _AppendTemplateSingles(const CetTNestItemVector& AOriginalItems, std::vector<bool>& AUsed, TetClusterBuildResult& AResult);

            TetClusterBuildResult _BuildAllSingles(const CetTNestItemVector& AOriginalItems);
            bool _ValidateBuildResultCoverage(const TetClusterBuildResult& AResult, int AOriginalCount);
            void _AddSingleItem(const CetTNestItemVector& AOriginalItems, int AOriginalIndex, TetClusterBuildResult& AResult);
            bool _CanAcceptClusterCandidate(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const TetClusterCandidate& ACandidate, const std::vector<bool>& AUsed, int AOriginalCount);
            CetNestItem _MakeClusterProxyItem(const TetClusterCandidate& ACandidate);
            bool _AddClusterCandidate(const TetClusterCandidate& ACandidate, TetClusterBuildResult& AResult);
            void _ExpandClusterChildren(const CetNestItem& APackedItem, const TetMetaItem& AMeta, CetTNestItemVector& AOutOriginalItems, bool ALog = true);
            double _GetItemWidth(const CetNestItem& AItem);
            double _GetItemHeight(const CetNestItem& AItem);
            TetClusterBuildResult _BuildAutoPairClusters(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions);
            bool _TryFindBestEdgePairCandidate(const CetTNestItemVector& AOriginalItems, int AIndex, int ABIndex, const TetNestOptions& AOptions, TetAutoPairCandidate& ABestCandidate);
            bool _TryFindBestAutoPairCandidate(const CetTNestItemVector& AOriginalItems, int AIndex, int ABIndex, const TetNestOptions& AOptions, TetAutoPairCandidate& ABestCandidate);
            bool _TryBuildAutoPairAt(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const TetAutoPairBuildInput& AInput, TetAutoPairCandidate& ACandidate);
            void _AddAutoPairCluster(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const TetAutoPairCandidate& ACandidate, TetClusterBuildResult& AResult);
            double _CalcAutoPairScore(double ABeforeBBoxArea, double AAfterBBoxArea, double ARealArea, double AClusterW, double AClusterH);
            bool _RunAutoPairGridSearch(const CetTNestItemVector& AOriginalItems, int AIndex, int ABIndex, const TetNestOptions& AOptions, const TetAutoPairGridConfig& AConfig, TetAutoPairCandidate& AOutBest);
            CetNestItem _MakeUnionNestItemFromCandidate(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const TetAutoPairCandidate& ACandidate);
            double _CalcEdgeLength(const ClipperLib::IntPoint& A, const ClipperLib::IntPoint& AB);
            std::vector<TetEdgeInfo> _CollectEdges(const ClipperLib::Path& AContour);
            bool _IsSimilarTriangleByEdges(std::vector<TetEdgeInfo> AEdges, std::vector<TetEdgeInfo> ABEdges);
            bool _SnapToAllowedRotation(double ATarget, int ARotations, double& AOutRotation);
            bool _EvaluateEdgePair(const TetEdgePairContext& Actx, const TetEdgeInfo& AEdgeA, const TetEdgeInfo& AEdgeB, TetAutoPairCandidate& ABestCandidate);
            bool _TestEdgeOffsets(const TetEdgePairContext& Actx, const TetEdgeMatchState& Astate, const TetEdgeInfo& AEdgeA, TetAutoPairCandidate& ABestCandidate);
            bool _RunGridSearchAllAngles(const TetAutoPairContext& Actx, const std::vector<double>& Arotations, TetAutoPairCandidate& ABestCandidate);
            bool _EvaluateRotationPair(const TetAutoPairContext& Actx, double ARot, double ABRot, TetAutoPairCandidate& ABestCandidate);
           //long long MakeRelativeSizeBucket(double Value);
        };
    }
}
