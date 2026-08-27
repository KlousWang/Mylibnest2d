#pragma once
#include "EtTechCore_Object.h"
#include "Nest2D_DataType.h"
#include "Nest2D_PrivateDataType.h"

#include <chrono>
#include <cstddef>
#include <iostream>
#include <vector>
namespace ET {
    namespace NEST2DMANAGERLIB {
        class CetPolygonBoardRepairer : public ET::CORE::CetCoreObject
        {
        Inherit_Invoke_Hook(CetPolygonBoardRepairer) protected : int _Init() override
            {
                CetCoreObject::_Init();
                return 0;
            }
            void _WrapFuncs() override
            {
                CetCoreObject::_WrapFuncs();
                _WrapFunc("Repair", Type_Class_Func(Repair));
                _WrapFunc("SetContext", Type_Class_Func(SetContext));
                _WrapFunc("PackFromScratch", Type_Class_Func(PackFromScratch));
                //_WrapFunc("InsertTextBeforeSvgEnd", Type_Class_Func(InsertTextBeforeSvgEnd));
            }

        public:
            CetPolygonBoardRepairer();
            CetPolygonBoardRepairer(CetTNestItemVector &ANestItems, const TetNestOptions &AOptions, const CetPolygonImpl &ABinPoly, double ABoardBinWidth, double ABoardBinHeight);
            ~CetPolygonBoardRepairer();

        public:
            void SetContext(CetTNestItemVector &ANestItems, const TetNestOptions &AOptions, const CetPolygonImpl &ABinPoly, double ABoardBinWidth, double ABoardBinHeight);
            void Repair(std::size_t &ALayers);
            bool RepairLockedEnvelope(std::size_t &ALayers, const std::vector<std::size_t> &ALockedItems);
            bool HadBoardFillChanges() const { return _HadBoardFillChanges; }
            bool EvacuateLastBin(std::size_t &ALayers, TetLastBinEvacuationStats &AStats);

            // CetPolygonBoardRepairer(CetTNestItemVector& ANestItems,const TetNestOptions& AOptions,const CetPolygonImpl& ABinPoly,double ABoardBinWidth,double ABoardBinHeight);
        protected:
            void PackFromScratch(std::size_t &ALayers);
            void _BuildRotations();

            std::size_t _CompactItemBins();

            bool _TryPlaceItemInBinByGrid(std::size_t AItemIndex, int ATargetBin);

            bool _CanPlaceAt(const TetPlacementCandidate &APlacement);

            void _FillTranslationForBBoxMin(TetPlacementCandidate &APlacement, double ATargetMinX, double ATargetMinY);
            bool _IsCurrentPlacementValid(std::size_t AItemIndex);

            void _FixInvalidItems(std::size_t &ALayers);

            void _FillHoles(std::size_t &ALayers);
            bool _RunBoardCompositePass(std::size_t &ALayers, const std::chrono::steady_clock::time_point &ADeadline, long long ATimePerBinMs, std::vector<long long> &AExactChecksByBin, std::vector<std::size_t> &ARollbackBins, std::vector<std::size_t> &AMovedItems, TetBoardCompositeSearchStats &AStats, std::size_t &AAcceptedMoves);
            bool _RunLegacyBoardFillPass(std::size_t ALayers, std::size_t &AAcceptedMoves);
            bool _ExtractBoardFreeRegions(int ATargetBin, std::vector<TetClusterFreeRegion> &AOutRegions) const;
            bool _BuildBoardReservedContours(int ATargetBin, ClipperLib::Paths &AOutContours) const;
            bool _AppendBoardFreeRegion(const ClipperLib::PolyNode &ANode, std::vector<TetClusterFreeRegion> &AOutRegions) const;
            bool _FindBestBoardCompositeForBin(int ATargetBin, const std::vector<TetClusterFreeRegion> &AFreeRegions, long long &AInOutExactChecks, TetBoardCompositeCandidate &AOutCandidate, TetBoardCompositeSearchStats &AInOutStats);
            bool _BuildBoardCompositeForSkeleton(int ATargetBin, const TetClusterFreeRegion &AFreeRegion, int ASkeletonIndex, const std::vector<TetShapeFeature> &AFeatures, long long &AInOutExactChecks, TetBoardCompositeCandidate &AOutCandidate, TetBoardCompositeSearchStats &AInOutStats);
            bool _BuildAnchoredBoardComposite(int ATargetBin, const TetClusterFreeRegion &AFreeRegion, int ASkeletonIndex, const std::vector<TetShapeFeature> &AFeatures, long long &AInOutExactChecks, TetBoardCompositeCandidate &AOutCandidate, TetBoardCompositeSearchStats &AInOutStats);
            bool _BuildAnchoredCandidateForFiller(int ATargetBin, int ASkeletonIndex, int AFillerIndex, const TetClusterCandidate &ASkeleton, const std::vector<TetShapeFeature> &AFeatures, long long &AInOutExactChecks, TetBoardCompositeCandidate &AOutCandidate, TetBoardCompositeSearchStats &AInOutStats);
            void _TranslateFreeRegions(std::vector<TetClusterFreeRegion> &ARegions, double AOffsetX, double AOffsetY) const;
            bool _BuildCompositeSkeleton(int ASkeletonIndex, const TetShapeFeature &AFeature, TetClusterCandidate &AOutCluster) const;
            bool _BuildAnchoredCompositeSkeleton(int ASkeletonIndex, const TetShapeFeature &AFeature, TetClusterCandidate &AOutCluster) const;
            std::vector<int> _CollectCompositeFillers(int ATargetBin, const TetClusterCandidate &ACluster, const TetClusterFreeRegion &AFreeRegion, const std::vector<TetShapeFeature> &AFeatures, bool AAllowTargetBin) const;
            bool _ExpandCompositeBeam(const TetClusterCandidate &ASkeleton, const std::vector<int> &AFillers, const std::vector<TetShapeFeature> &AFeatures, TetClusterCandidate &AOutCluster) const;
            bool _PlaceBoardCompositeInFreeRegion(TetBoardCompositeCandidate &AInOutCandidate, long long &AInOutExactChecks, TetBoardCompositeSearchStats &AInOutStats);
            bool _PlaceAnchoredBoardComposite(TetBoardCompositeCandidate &AInOutCandidate, long long &AInOutExactChecks, TetBoardCompositeSearchStats &AInOutStats);
            bool _BuildCompositePlacements(const TetBoardCompositeCandidate &ACandidate, double ARotation, double ATranslationX, double ATranslationY, std::vector<TetHoleFillCandidate> &AOutPlacements) const;
            bool _ValidateBoardCompositePlacements(const TetBoardCompositeCandidate &ACandidate, const std::vector<TetHoleFillCandidate> &APlacements) const;
            void _ScoreBoardComposite(TetBoardCompositeCandidate &AInOutCandidate) const;
            bool _IsBoardCompositeBetter(const TetBoardCompositeCandidate &AFirst, const TetBoardCompositeCandidate &ASecond) const;
            bool _HasBoardCompositeGlobalGain(const CetTNestItemVector &ABeforeItems, std::size_t ABeforeLayers, const TetBoardCompositeCandidate &ACandidate, std::size_t AAfterLayers) const;
            bool _HasAnchoredRelocationGain(const CetTNestItemVector &ABeforeItems, const TetBoardCompositeCandidate &ACandidate) const;
            bool _ApplyBoardCompositeCandidate(const TetBoardCompositeCandidate &ACandidate, std::size_t &AInOutLayers, TetBoardCompositeSearchStats &AInOutStats);
            bool _FindBestLocalCandidateForTargetBin(int ATargetBin, const std::vector<TetClusterFreeRegion> &AFreeRegions, TetBoardLocalFillCandidate &ABestCandidate);
            bool _BuildLocalCandidateForFreeRegion(int ATargetBin, const TetClusterFreeRegion &AFreeRegion, TetBoardLocalFillCandidate &AOutCandidate);
            std::vector<std::size_t> _CollectLocalFillCandidates(int ATargetBin, const TetClusterFreeRegion &AFreeRegion) const;
            bool _ApplyLocalFillCandidate(const TetBoardLocalFillCandidate &ACandidate);
            bool _IsBoardLocalCandidateBetter(const TetBoardLocalFillCandidate &AFirst, const TetBoardLocalFillCandidate &ASecond) const;
            void _UpdateBoardLocalEnvelope(const std::vector<TetHoleFillCandidate> &APlacements, double &AOutArea, double &AOutFillRatio) const;
            bool _FindBestCandidateForTargetBin(int ATargetBin, const std::vector<TetClusterFreeRegion> &AFreeRegions, TetHoleFillCandidate &ABestCandidate);
            bool _TryFindBestPlacementInBin(std::size_t AItemIndex, int ATargetBin, const std::vector<TetClusterFreeRegion> &AFreeRegions, TetHoleFillCandidate &ABestCandidate, long long ACheckLimit = 0);
            bool _EvaluateBoardFillPlacement(std::size_t AItemIndex, int ATargetBin, int AOldBin, const TetClusterFreeRegion &AFreeRegion, const libnest2d::Radians &ARotation, const libnest2d::Point &ATranslation, long long ACheckLimit, long long &ACheckedCount, TetHoleFillCandidate &ABestCandidate);
            void _ProbeContourContactPlacements(std::size_t AItemIndex, int ATargetBin, int AOldBin, const TetClusterFreeRegion &AFreeRegion, const libnest2d::Radians &ARotation, const CetPath &ARotatedContour, long long AProbeLimit, long long ACheckLimit, long long &ACheckedCount, TetHoleFillCandidate &ABestCandidate);
            std::vector<std::size_t> _SelectContactVertexIndices(const CetPath &AContour) const;
            bool _IsPlacementInsideFreeRegion(const TetPlacementCandidate &APlacement, const TetClusterFreeRegion &AFreeRegion) const;
            bool _ApplyHoleFillCandidate(const TetHoleFillCandidate &ACandidate);
            double _CalcHoleFillScore(std::size_t AItemIndex, int AOldBin, int ATargetBin, const libnest2d::Point &ATranslation);
            double _CalculateBinOccupiedArea(int ABinId) const;
            std::vector<std::size_t> _CollectLastBinItems(int ALastBinId) const;
            void _CaptureLastBinStats(const std::vector<std::size_t> &AItemIndices, TetLastBinEvacuationStats &AStats) const;
            int _CountUsedBins() const;
            std::vector<int> _BuildLastBinTargetOrder(int ALastBinId) const;
            bool _HasEnoughFreeAreaForLastBin(int ALastBinId, double ALastBinArea) const;
            bool _TryEvacuateItemDirect(std::size_t AItemIndex, const std::vector<int> &ATargetBins);
            std::vector<std::size_t> _CollectSmallRelocationCandidates(int ATargetBin) const;
            bool _TryEvacuateItemWithRelocation(std::size_t AItemIndex, int ALastBinId, const std::vector<int> &ATargetBins, TetLastBinEvacuationStats &AStats);
            bool _TryRelocateSmallItemWithinSameBin(std::size_t AItemIndex, int ABinId, double &AScore);
            bool _TryFindBestSmallRelocationInBin(std::size_t AItemIndex, int ATargetBin, TetPlacementCandidate &ABestPlacement, double &ABestScore);
            double _CalcSmallRelocationScore(const TetPlacementCandidate &APlacement);
            bool _ValidateAllItemsWithSpacing(int ALastBinId) const;
            bool _CanContinueSearch();
            double _GetEffectiveGridStep(long long ACheckLimit) const;

        protected:
            CetTNestItemVector *_Items = nullptr;
            const TetNestOptions *_Options = nullptr;
            const CetPolygonImpl *_BinPoly = nullptr;

            double m_BoardBinWidth = 0.0;
            double m_BoardBinHeight = 0.0;

            double m_StepMm = 3.0;
            long long m_SpacingCoord = 0;
            long long m_RemainingPlacementChecks = 0;
            long long m_PlacementChecks = 0;
            long long m_PerItemPlacementCheckLimit = 0;
            std::chrono::steady_clock::time_point m_SearchDeadline{};
            bool m_SearchBudgetReached = false;
            std::vector<std::size_t> m_LockedItemIndices;
            static thread_local bool _HadBoardFillChanges;

            std::vector<libnest2d::Radians> m_Rotations;
        };

    } // namespace NEST2DMANAGERLIB
} // namespace ET
