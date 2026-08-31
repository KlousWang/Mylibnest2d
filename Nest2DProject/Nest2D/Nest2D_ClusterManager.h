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
                _WrapFunc("BuildClusterResultFromCandidates", Type_Class_Func(BuildClusterResultFromCandidates));
            }

        public:
            CetClusterManager();
            ~CetClusterManager();

        public:
            std::vector<int> RankBoardCompositeSkeletons(const CetTNestItemVector& AItems, const std::vector<TetShapeFeature>& AFeatures, int ATargetBin, const TetClusterFreeRegion& AFreeRegion, std::size_t AMaxCount) const;
            std::vector<int> RankExistingBoardCompositeSkeletons(const CetTNestItemVector& AItems, const std::vector<TetShapeFeature>& AFeatures, int ATargetBin, const TetClusterFreeRegion& AFreeRegion, std::size_t AMaxCount) const;
            TetClusterBuildResult BuildClusterItems(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, MetClusterStrategy AStrategy);
            TetClusterBuildResult BuildClusterItemsWithFeatures(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions, MetClusterStrategy AStrategy);
            TetClusterBuildResult BuildClusterResultFromCandidates(const CetTNestItemVector& AOriginalItems, const std::vector<TetClusterCandidate>& ACandidates, const TetNestOptions& AOptions);
            void ExpandClusterResultToOriginalItems(const CetTNestItemVector& AOriginalItems, const CetTNestItemVector& APackedItems, const std::vector<TetMetaItem>& AMetaItems, CetTNestItemVector& AOutOriginalItems, bool ALog = true);
            bool ValidatePackedResultSpacing(const CetTNestItemVector& AOriginalItems, const CetTNestItemVector& APackedItems, const std::vector<TetMetaItem>& AMetaItems, const TetNestOptions& AOptions, TetExpandedSpacingFailure* AOutFailure = nullptr);

        protected:
            void _AddSingleItem(const CetTNestItemVector& AOriginalItems, int AOriginalIndex, TetClusterBuildResult& AResult);
            CetNestItem _MakeClusterProxyItem(const TetClusterCandidate& ACandidate);
            bool _AddClusterCandidate(const TetClusterCandidate& ACandidate, TetClusterBuildResult& AResult);
            TetClusterBuildResult _BuildAllSingles(const CetTNestItemVector& AOriginalItems);
            bool _ValidateBuildResultCoverage(const TetClusterBuildResult& AResult, int AOriginalCount);
            void _ExpandClusterChildren(const CetNestItem& APackedItem, const TetMetaItem& AMeta, CetTNestItemVector& AOutOriginalItems, bool ALog = true);
            double _GetItemWidth(const CetNestItem& AItem);
            double _GetItemHeight(const CetNestItem& AItem);
        };
    }
}
