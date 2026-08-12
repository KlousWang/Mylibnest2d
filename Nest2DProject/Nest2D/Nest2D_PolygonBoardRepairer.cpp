#include "pch.h"
#include "Nest2D_PolygonBoardRepairer.h"

#include "Nest2D_ClusterGeometryHelper.h"
#include "Nest2D_RotationUtils.h"

#include "NestUtils.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <set>

using namespace ClipperLib;
using namespace libnest2d;

namespace ET {
    namespace NEST2DMANAGERLIB {

      /*  struct TetPlacementCandidate
        {
            TetPlacementCandidate(): ItemIndex(0), TargetBin(-1), Translation(Point(0, 0)), Rotation(libnest2d::Radians(0.0)){
            }

            std::size_t ItemIndex;
            int TargetBin;
            Point Translation;
            libnest2d::Radians Rotation;
        };*/

        CetPolygonBoardRepairer::CetPolygonBoardRepairer(): CetCoreObject()
        {
        }

        CetPolygonBoardRepairer::~CetPolygonBoardRepairer()
        {
        }

        CetPolygonBoardRepairer::CetPolygonBoardRepairer(CetTNestItemVector& ANestItems,const TetNestOptions& AOptions,const CetPolygonImpl& ABinPoly,double ABoardBinWidth,double ABoardBinHeight): CetCoreObject()
        {
            SetContext(ANestItems,AOptions,ABinPoly,ABoardBinWidth,ABoardBinHeight);
        }
        void CetPolygonBoardRepairer::SetContext(CetTNestItemVector& ANestItems,const TetNestOptions& AOptions,const CetPolygonImpl& ABinPoly,double ABoardBinWidth,double ABoardBinHeight)
        {
            _Items = &ANestItems;
            _Options = &AOptions;
            _BinPoly = &ABinPoly;
            m_BoardBinWidth = ABoardBinWidth;
            m_BoardBinHeight = ABoardBinHeight;

            m_StepMm = std::max(0.5, static_cast<double>(AOptions.Placer.Accuracy));
            if (!std::isfinite(m_StepMm) || m_StepMm <= 0.0) {
                m_StepMm = 1.0;
            }
            m_SpacingCoord = NestUtils::ToNestCoord(_Options->Spacing);
            m_RemainingPlacementChecks = CET_REPAIR_MAX_TOTAL_PLACEMENT_CHECKS;
            m_PerItemPlacementCheckLimit = CET_REPAIR_MAX_PLACEMENT_CHECKS_PER_ITEM;
            m_PlacementChecks = 0;
            m_SearchDeadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(CET_REPAIR_MAX_SEARCH_TIME_MS);
            m_SearchBudgetReached = false;

            _BuildRotations();
        }

        void CetPolygonBoardRepairer::Repair(std::size_t& ALayers)
        {
            if (_Items == nullptr || _Options == nullptr || _BinPoly == nullptr){
                std::cout << "[REPAIR][ERROR] Repairer context is null." << std::endl;
                return;
            }
            if (ALayers == 0){
                PackFromScratch(ALayers);
                return;
            }      
            auto& Items = *_Items;
            std::cout << "[REPAIR] start polygon board repair. Layers = " << ALayers << ", StepMm = " << m_StepMm << std::endl;
            _FixInvalidItems(ALayers);
            if (ALayers <= 1){
                ALayers = _CompactItemBins();

                std::cout << "[REPAIR] finish polygon board repair. Layers = " << ALayers << std::endl;
                return;
            }

			_FillHoles(ALayers);
			if (m_SearchBudgetReached) {
				std::cout << "[REPAIR][SEARCH LIMIT] Checks=" << m_PlacementChecks << std::endl;
			}

            ALayers = _CompactItemBins();

            std::cout << "[REPAIR] finish polygon board repair. Layers = " << ALayers << std::endl;
        }

        bool CetPolygonBoardRepairer::EvacuateLastBin(std::size_t& ALayers, TetLastBinEvacuationStats& AStats)
        {
            AStats = TetLastBinEvacuationStats{};
            const auto StartTime = std::chrono::steady_clock::now();
            const auto Finish = [&]() {
                AStats.PlacementChecks = m_PlacementChecks;
                AStats.SearchBudgetReached = m_SearchBudgetReached;
                AStats.TimeMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - StartTime).count();
                };
            AStats.Started = true;
            if (_Items == nullptr || _Options == nullptr || _BinPoly == nullptr || ALayers <= 1) {
                Finish();
                return false;
            }

            const CetTNestItemVector OriginalSolution = *_Items;
            const std::size_t OriginalLayers = ALayers;
            m_RemainingPlacementChecks = CET_LAST_BIN_MAX_TOTAL_PLACEMENT_CHECKS;
            m_PerItemPlacementCheckLimit = CET_LAST_BIN_MAX_PLACEMENT_CHECKS_PER_ITEM;
            m_PlacementChecks = 0;
            m_SearchDeadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(CET_LAST_BIN_MAX_SEARCH_TIME_MS);
            m_SearchBudgetReached = false;
            AStats.BeforeUsedBins = _CountUsedBins();
            AStats.AfterUsedBins = AStats.BeforeUsedBins;
            const std::vector<std::size_t> LastBinItems = _CollectLastBinItems(-1);
            _CaptureLastBinStats(LastBinItems, AStats);
            AStats.RemainingItems = static_cast<int>(LastBinItems.size());
            if (LastBinItems.empty() || AStats.LastBinId < 1) {
                Finish();
                return false;
            }
            const std::vector<int> TargetBins = _BuildLastBinTargetOrder(AStats.LastBinId);
            if (TargetBins.empty()) {
                Finish();
                return false;
            }

            if (!_HasEnoughFreeAreaForLastBin(AStats.LastBinId, AStats.LastBinArea)) {
                AStats.InsufficientFreeArea = true;
                Finish();
                return false;
            }

            for (int Pass = 0; Pass < CET_LAST_BIN_MAX_EVACUATION_PASSES; ++Pass) {
                bool Changed = false;
                const std::vector<std::size_t> CurrentItems = _CollectLastBinItems(AStats.LastBinId);
                for (std::size_t ItemIndex : CurrentItems) {
                    if (!_CanContinueSearch() || ItemIndex >= _Items->size() || (*_Items)[ItemIndex].binId() != AStats.LastBinId) {
                        continue;
                    }
                    if (_TryEvacuateItemDirect(ItemIndex, TargetBins)) {
                        ++AStats.DirectMoves;
                        Changed = true;
                        std::cout << "[LAST_BIN][MOVE] Item=" << ItemIndex << " From=" << AStats.LastBinId
                            << " To=" << (*_Items)[ItemIndex].binId() << " Mode=Direct" << std::endl;
                        continue;
                    }
                    if (_TryEvacuateItemWithRelocation(ItemIndex, AStats.LastBinId, TargetBins, AStats)) {
                        Changed = true;
                        continue;
                    }
                    ++AStats.NoCandidatePosition;
                    std::cout << "[LAST_BIN][ITEM FAILED] Item=" << ItemIndex
                        << " Area=" << std::abs(static_cast<double>((*_Items)[ItemIndex].area()))
                        << " TriedBins=" << TargetBins.size() << " DirectFailed=1" << std::endl;
                }
                if (!Changed) {
                    break;
                }
            }

            AStats.RemainingItems = static_cast<int>(_CollectLastBinItems(AStats.LastBinId).size());
            AStats.ValidationPassed = AStats.RemainingItems == 0 && _ValidateAllItemsWithSpacing(AStats.LastBinId);
            if (!AStats.ValidationPassed) {
                *_Items = OriginalSolution;
                ALayers = OriginalLayers;
                AStats.RolledBack = true;
                Finish();
                return false;
            }

            ALayers = _CompactItemBins();
            AStats.AfterUsedBins = static_cast<int>(ALayers);
            AStats.Success = AStats.AfterUsedBins < AStats.BeforeUsedBins;
            if (!AStats.Success) {
                *_Items = OriginalSolution;
                AStats.RolledBack = true;
                ALayers = OriginalLayers;
                AStats.ValidationPassed = false;
            }
            Finish();
            return AStats.Success;
        }

        void CetPolygonBoardRepairer::PackFromScratch(std::size_t& ALayers)
        {
            if (_Items == nullptr || _Options == nullptr || _BinPoly == nullptr){
                std::cout << "[PACK][ERROR] context is null." << std::endl;
                ALayers = 0;
                return;
            }

            if (!_Options->Board.Enabled || _Options->Board.Vertices.size() < 3){
                ALayers = 0;
                return;
            }

            auto& Items = *_Items;

            std::cout << "[PACK] start pack from scratch. Items = " << Items.size() << ", StepMm = " << m_StepMm << std::endl;

            
            for (auto& Item : Items){
                Item.binId(-1);
                Item.translation(Point(0, 0));
                Item.rotation(libnest2d::Radians(0.0));
            }

            std::size_t UsedLayers = 0;

            for (std::size_t i = 0; i < Items.size(); ++i){
                bool Placed = false;

                
                for (int TargetBin = 0; TargetBin < static_cast<int>(UsedLayers); ++TargetBin){
                    if (_TryPlaceItemInBinByGrid(i, TargetBin)){
                        Placed = true;
                        break;
                    }
                }

                
                if (!Placed){
                    int NewBin = static_cast<int>(UsedLayers);

                    if (_TryPlaceItemInBinByGrid(i, NewBin)){
                        Placed = true;
                        ++UsedLayers;

                        std::cout << "[PACK] item " << i << " placed in new bin " << NewBin << std::endl;
                    }
                }

                
                if (!Placed){
                    Items[i].binId(-1);

                    std::cout << "[PACK][WARN] item " << i << " cannot be placed in polygon board." << std::endl;
                }
            }

            ALayers = _CompactItemBins();

            std::cout << "[PACK] finish pack from scratch. Layers = " << ALayers << std::endl;
        }

        void CetPolygonBoardRepairer::_BuildRotations()
        {
            const int RotationCount = _Options == nullptr ? 0 : _Options->Rotations;
            m_Rotations = CetRotationUtils::BuildAllowedLibRotations(RotationCount);
        }

        std::size_t CetPolygonBoardRepairer::_CompactItemBins()
        {
            if (_Items == nullptr){
                return 0;
            }

            auto& Items = *_Items;

            std::map<int, int> Remap;
            int NextBin = 0;

            for (auto& Item : Items){
                int OldBin = static_cast<int>(Item.binId());

                if (OldBin < 0){
                    continue;
                }

                auto It = Remap.find(OldBin);

                if (It == Remap.end()){
                    Remap[OldBin] = NextBin;
                    Item.binId(NextBin);
                    ++NextBin;
                }
                else {
                    Item.binId(It->second);
                }
            }

            return static_cast<std::size_t>(NextBin);
        }

        std::vector<std::size_t> CetPolygonBoardRepairer::_CollectLastBinItems(int ALastBinId) const
        {
            std::vector<std::size_t> Result;
            if (_Items == nullptr) {
                return Result;
            }
            int LastBinId = ALastBinId;
            if (LastBinId < 0) {
                for (const CetNestItem& Item : *_Items) {
                    LastBinId = std::max(LastBinId, static_cast<int>(Item.binId()));
                }
            }
            for (std::size_t Index = 0; Index < _Items->size(); ++Index) {
                if ((*_Items)[Index].binId() == LastBinId) {
                    Result.push_back(Index);
                }
            }
            std::stable_sort(Result.begin(), Result.end(), [&](std::size_t A, std::size_t B) {
                const double AreaA = std::abs(static_cast<double>((*_Items)[A].area()));
                const double AreaB = std::abs(static_cast<double>((*_Items)[B].area()));
                if (std::abs(AreaA - AreaB) > 1e-9) {
                    return AreaA > AreaB;
                }
                const auto BoundsA = (*_Items)[A].boundingBox();
                const auto BoundsB = (*_Items)[B].boundingBox();
                return std::max(static_cast<double>(BoundsA.width()), static_cast<double>(BoundsA.height())) > std::max(static_cast<double>(BoundsB.width()), static_cast<double>(BoundsB.height()));
                });
            return Result;
        }

        void CetPolygonBoardRepairer::_CaptureLastBinStats(const std::vector<std::size_t>& AItemIndices, TetLastBinEvacuationStats& AStats) const
        {
            AStats.LastBinId = -1;
            AStats.LastBinItemCount = static_cast<int>(AItemIndices.size());
            AStats.LastBinArea = 0.0;
            if (_Items == nullptr) {
                return;
            }
            for (std::size_t Index : AItemIndices) {
                if (Index >= _Items->size()) {
                    continue;
                }
                AStats.LastBinId = std::max(AStats.LastBinId, static_cast<int>((*_Items)[Index].binId()));
                AStats.LastBinArea += std::abs(static_cast<double>((*_Items)[Index].area()));
            }
        }

        int CetPolygonBoardRepairer::_CountUsedBins() const
        {
            std::set<int> UsedBins;
            if (_Items != nullptr) {
                for (const CetNestItem& Item : *_Items) {
                    if (Item.binId() >= 0) {
                        UsedBins.insert(Item.binId());
                    }
                }
            }
            return static_cast<int>(UsedBins.size());
        }

        std::vector<int> CetPolygonBoardRepairer::_BuildLastBinTargetOrder(int ALastBinId) const
        {
            std::vector<std::pair<int, double>> Candidates;
            if (_Items == nullptr || _Options == nullptr || _BinPoly == nullptr || ALastBinId <= 0) {
                return {};
            }
            const double BoardArea = std::abs(static_cast<double>(sl::area(*_BinPoly)));
            for (int BinId = 0; BinId < ALastBinId; ++BinId) {
                double OccupiedArea = 0.0;
                for (const CetNestItem& Item : *_Items) {
                    if (Item.binId() == BinId) {
                        OccupiedArea += std::abs(static_cast<double>(Item.area()));
                    }
                }
                Candidates.emplace_back(BinId, BoardArea - OccupiedArea);
            }
            std::stable_sort(Candidates.begin(), Candidates.end(), [](const auto& A, const auto& B) {
                if (std::abs(A.second - B.second) > 1e-9) {
                    return A.second > B.second;
                }
                return A.first < B.first;
                });
            std::vector<int> Result;
            for (const auto& Candidate : Candidates) {
                Result.push_back(Candidate.first);
                if (Result.size() >= CET_LAST_BIN_MAX_TARGET_BINS_PER_ITEM) {
                    break;
                }
            }
            return Result;
        }


        bool CetPolygonBoardRepairer::_HasEnoughFreeAreaForLastBin(int ALastBinId, double ALastBinArea) const
        {
            if (_Items == nullptr || ALastBinId <= 0 || ALastBinArea <= 0.0) {
                return false;
            }
            const double BoardArea = _BinPoly == nullptr ? 0.0 : std::abs(static_cast<double>(sl::area(*_BinPoly)));
            if (BoardArea <= 0.0) {
                return false;
            }
            std::vector<double> OccupiedAreas(static_cast<std::size_t>(ALastBinId), 0.0);
            for (const CetNestItem& Item : *_Items) {
                const int BinId = static_cast<int>(Item.binId());
                if (BinId >= 0 && BinId < ALastBinId) {
                    OccupiedAreas[static_cast<std::size_t>(BinId)] += std::abs(static_cast<double>(Item.area()));
                }
            }
            double TotalFreeArea = 0.0;
            for (double OccupiedArea : OccupiedAreas) {
                TotalFreeArea += std::max(0.0, BoardArea - OccupiedArea);
            }
            return TotalFreeArea + 1e-9 >= ALastBinArea;
        }


        bool CetPolygonBoardRepairer::_TryEvacuateItemDirect(std::size_t AItemIndex, const std::vector<int>& ATargetBins)
        {
            if (_Items == nullptr || AItemIndex >= _Items->size()) {
                return false;
            }
            for (int TargetBin : ATargetBins) {
                if (_TryPlaceItemInBinByGrid(AItemIndex, TargetBin)) {
                    return true;
                }
            }
            return false;
        }

        std::vector<std::size_t> CetPolygonBoardRepairer::_CollectSmallRelocationCandidates(int ATargetBin) const
        {
            std::vector<std::size_t> Result;
            if (_Items == nullptr || _Options == nullptr || _BinPoly == nullptr) {
                return Result;
            }
            const double BoardArea = std::abs(static_cast<double>(sl::area(*_BinPoly)));
            const double MaxSmallArea = BoardArea * CET_LAST_BIN_SMALL_ITEM_AREA_RATIO;
            double MaxUsedX = 0.0;
            double MaxUsedY = 0.0;
            for (std::size_t Index = 0; Index < _Items->size(); ++Index) {
                const CetNestItem& Item = (*_Items)[Index];
                if (Item.binId() != ATargetBin) {
                    continue;
                }
                const auto Bounds = Item.boundingBox();
                MaxUsedX = std::max(MaxUsedX, static_cast<double>(getX(Bounds.maxCorner())));
                MaxUsedY = std::max(MaxUsedY, static_cast<double>(getY(Bounds.maxCorner())));
                if (std::abs(static_cast<double>(Item.area())) <= MaxSmallArea) {
                    Result.push_back(Index);
                }
            }
            const double BoardWidth = static_cast<double>(NestUtils::ToNestCoord(m_BoardBinWidth));
            const double BoardHeight = static_cast<double>(NestUtils::ToNestCoord(m_BoardBinHeight));
            const auto Rank = [&](std::size_t AIndex) {
                const CetNestItem& Item = (*_Items)[AIndex];
                const auto Bounds = Item.boundingBox();
                const double MinX = static_cast<double>(getX(Bounds.minCorner()));
                const double MinY = static_cast<double>(getY(Bounds.minCorner()));
                const double MaxX = static_cast<double>(getX(Bounds.maxCorner()));
                const double MaxY = static_cast<double>(getY(Bounds.maxCorner()));
                const double UsedBoundaryGap = std::min(std::abs(MaxUsedX - MaxX), std::abs(MaxUsedY - MaxY));
                const double BoardEdgeGap = std::min({ MinX, MinY, BoardWidth - MaxX, BoardHeight - MaxY });
                return std::array<double, 3>{ UsedBoundaryGap, BoardEdgeGap, std::abs(static_cast<double>(Item.area())) };
                };
            std::stable_sort(Result.begin(), Result.end(), [&](std::size_t A, std::size_t B) {
                return Rank(A) < Rank(B);
                });
            if (Result.size() > CET_LAST_BIN_MAX_RELOCATION_CANDIDATES) {
                Result.resize(CET_LAST_BIN_MAX_RELOCATION_CANDIDATES);
            }
            return Result;
        }

        bool CetPolygonBoardRepairer::_TryEvacuateItemWithRelocation(std::size_t AItemIndex, int ALastBinId, const std::vector<int>& ATargetBins, TetLastBinEvacuationStats& AStats)
        {
            if (_Items == nullptr || AItemIndex >= _Items->size()) {
                return false;
            }
            for (int TargetBin : ATargetBins) {
                std::vector<std::size_t> SmallItems = _CollectSmallRelocationCandidates(TargetBin);
                if (SmallItems.empty()) {
                    continue;
                }
                for (std::size_t Start = 0; Start < SmallItems.size(); ++Start) {
                    const CetTNestItemVector AttemptSolution = *_Items;
                    int RelocatedCount = 0;
                    for (std::size_t Offset = 0; Offset < SmallItems.size() - Start && Offset < CET_LAST_BIN_MAX_RELOCATED_SMALL_ITEMS; ++Offset) {
                        const std::size_t SmallIndex = SmallItems[Start + Offset];
                        if (SmallIndex >= _Items->size() || (*_Items)[SmallIndex].binId() != TargetBin) {
                            continue;
                        }
                        double RelocationScore = 0.0;
                        if (!_TryRelocateSmallItemWithinSameBin(SmallIndex, TargetBin, RelocationScore)) {
                            break;
                        }
                        ++RelocatedCount;
                        if (_TryPlaceItemInBinByGrid(AItemIndex, TargetBin)) {
                            AStats.SameBinRelocations += RelocatedCount;
                            AStats.RelocatedExistingSmallItemCount += RelocatedCount;
                            std::cout << "[LAST_BIN][MOVE] Item=" << AItemIndex << " From=" << ALastBinId
                                << " To=" << TargetBin << " Mode=AfterRelocation" << std::endl;
                            return true;
                        }
                    }
                    *_Items = AttemptSolution;
                }
                ++AStats.RelocationFailed;
            }
            return false;
        }

        bool CetPolygonBoardRepairer::_TryRelocateSmallItemWithinSameBin(std::size_t AItemIndex, int ABinId, double& AScore)
        {
            if (_Items == nullptr || AItemIndex >= _Items->size() || (*_Items)[AItemIndex].binId() != ABinId) {
                return false;
            }
            CetNestItem& Item = (*_Items)[AItemIndex];
            const Point OldTranslation = Item.translation();
            const Radians OldRotation = Item.rotation();
            Item.binId(-1);
            TetPlacementCandidate BestPlacement;
            if (!_TryFindBestSmallRelocationInBin(AItemIndex, ABinId, BestPlacement, AScore)) {
                Item.translation(OldTranslation);
                Item.rotation(OldRotation);
                Item.binId(ABinId);
                return false;
            }
            Item.translation(BestPlacement.Translation);
            Item.rotation(BestPlacement.Rotation);
            Item.binId(ABinId);
            std::cout << "[LAST_BIN][SAME_BIN_RELOCATE] Item=" << AItemIndex << " Bin=" << ABinId
                << " Old=(" << NestUtils::FromNestCoord(OldTranslation.X) << "," << NestUtils::FromNestCoord(OldTranslation.Y)
                << ") New=(" << NestUtils::FromNestCoord(BestPlacement.Translation.X) << ","
                << NestUtils::FromNestCoord(BestPlacement.Translation.Y) << ") Score=" << AScore << std::endl;
            return true;
        }

        bool CetPolygonBoardRepairer::_TryFindBestSmallRelocationInBin(std::size_t AItemIndex, int ATargetBin, TetPlacementCandidate& ABestPlacement, double& ABestScore)
        {
            if (_Items == nullptr || AItemIndex >= _Items->size() || !_CanContinueSearch()) {
                return false;
            }
            const Point OldTranslation = (*_Items)[AItemIndex].translation();
            const Radians OldRotation = (*_Items)[AItemIndex].rotation();
            const long long CheckLimit = std::min(m_PerItemPlacementCheckLimit, m_RemainingPlacementChecks);
            const double GridStep = _GetEffectiveGridStep(CheckLimit);
            long long CheckedCount = 0;
            bool Found = false;
            ABestScore = -std::numeric_limits<double>::max();
            for (const Radians Angle : m_Rotations) {
                for (double Y = 0.0; Y < m_BoardBinHeight; Y += GridStep) {
                    for (double X = 0.0; X < m_BoardBinWidth; X += GridStep) {
                        if (CheckedCount >= CheckLimit || !_CanContinueSearch()) {
                            return Found;
                        }
                        ++CheckedCount;
                        ++m_PlacementChecks;
                        --m_RemainingPlacementChecks;
                        TetPlacementCandidate Placement;
                        Placement.ItemIndex = AItemIndex;
                        Placement.TargetBin = ATargetBin;
                        Placement.Rotation = Angle;
                        _FillTranslationForBBoxMin(Placement, X, Y);
                        const bool SamePosition = Placement.Translation == OldTranslation &&
                            std::abs(static_cast<double>(Placement.Rotation - OldRotation)) <= CET_ROTATION_DUPLICATE_TOLERANCE;
                        if (SamePosition || !_CanPlaceAt(Placement)) {
                            continue;
                        }
                        const double Score = _CalcSmallRelocationScore(Placement);
                        if (!Found || Score > ABestScore) {
                            ABestPlacement = Placement;
                            ABestScore = Score;
                            Found = true;
                        }
                    }
                }
            }
            return Found;
        }

        double CetPolygonBoardRepairer::_CalcSmallRelocationScore(const TetPlacementCandidate& APlacement)
        {
            if (_Items == nullptr || APlacement.ItemIndex >= _Items->size()) {
                return -std::numeric_limits<double>::max();
            }
            CetNestItem& Candidate = (*_Items)[APlacement.ItemIndex];
            const Point OldTranslation = Candidate.translation();
            const Radians OldRotation = Candidate.rotation();
            const int OldBin = Candidate.binId();
            Candidate.translation(APlacement.Translation);
            Candidate.rotation(APlacement.Rotation);
            Candidate.binId(APlacement.TargetBin);
            const auto CandidateBounds = Candidate.boundingBox();
            double UsedMinX = std::numeric_limits<double>::max();
            double UsedMinY = std::numeric_limits<double>::max();
            double UsedMaxX = std::numeric_limits<double>::lowest();
            double UsedMaxY = std::numeric_limits<double>::lowest();
            double NeighborContactScore = 0.0;
            bool HasNeighbor = false;
            for (std::size_t Index = 0; Index < _Items->size(); ++Index) {
                if (Index == APlacement.ItemIndex || (*_Items)[Index].binId() != APlacement.TargetBin) {
                    continue;
                }
                HasNeighbor = true;
                const auto OtherBounds = (*_Items)[Index].boundingBox();
                UsedMinX = std::min(UsedMinX, static_cast<double>(getX(OtherBounds.minCorner())));
                UsedMinY = std::min(UsedMinY, static_cast<double>(getY(OtherBounds.minCorner())));
                UsedMaxX = std::max(UsedMaxX, static_cast<double>(getX(OtherBounds.maxCorner())));
                UsedMaxY = std::max(UsedMaxY, static_cast<double>(getY(OtherBounds.maxCorner())));
                const double GapX = std::max({ 0.0, static_cast<double>(getX(OtherBounds.minCorner()) - getX(CandidateBounds.maxCorner())), static_cast<double>(getX(CandidateBounds.minCorner()) - getX(OtherBounds.maxCorner())) });
                const double GapY = std::max({ 0.0, static_cast<double>(getY(OtherBounds.minCorner()) - getY(CandidateBounds.maxCorner())), static_cast<double>(getY(CandidateBounds.minCorner()) - getY(OtherBounds.maxCorner())) });
                const double DistanceMm = std::hypot(GapX, GapY) / static_cast<double>(NestUtils::NestScale());
                NeighborContactScore += 1.0 / (1.0 + DistanceMm);
            }
            const double Result = HasNeighbor ? NeighborContactScore * 100.0 : -std::numeric_limits<double>::max();
            const double OldArea = HasNeighbor ? (UsedMaxX - UsedMinX) * (UsedMaxY - UsedMinY) : 0.0;
            const double NewMinX = std::min(UsedMinX, static_cast<double>(getX(CandidateBounds.minCorner())));
            const double NewMinY = std::min(UsedMinY, static_cast<double>(getY(CandidateBounds.minCorner())));
            const double NewMaxX = std::max(UsedMaxX, static_cast<double>(getX(CandidateBounds.maxCorner())));
            const double NewMaxY = std::max(UsedMaxY, static_cast<double>(getY(CandidateBounds.maxCorner())));
            const double BoardArea = std::max(1.0, static_cast<double>(NestUtils::ToNestCoord(m_BoardBinWidth)) * NestUtils::ToNestCoord(m_BoardBinHeight));
            const double ExpansionPenalty = std::max(0.0, (NewMaxX - NewMinX) * (NewMaxY - NewMinY) - OldArea) / BoardArea;
            const double EdgePenalty = std::max(NewMaxX / std::max(1.0, UsedMaxX), NewMaxY / std::max(1.0, UsedMaxY));
            Candidate.translation(OldTranslation);
            Candidate.rotation(OldRotation);
            Candidate.binId(OldBin);
            return Result - ExpansionPenalty * 1000.0 - EdgePenalty * 10.0;
        }

        bool CetPolygonBoardRepairer::_ValidateAllItemsWithSpacing(int ALastBinId) const
        {
            if (_Items == nullptr || _BinPoly == nullptr || ALastBinId < 0) {
                return false;
            }
            for (std::size_t Index = 0; Index < _Items->size(); ++Index) {
                CetNestItem Item = (*_Items)[Index];
                if (Item.binId() < 0 || Item.binId() == ALastBinId) {
                    return false;
                }
                Item.inflation(0);
                const auto Bounds = Item.boundingBox();
                if (getX(Bounds.minCorner()) < 0 || getY(Bounds.minCorner()) < 0 ||
                    getX(Bounds.maxCorner()) > NestUtils::ToNestCoord(m_BoardBinWidth) ||
                    getY(Bounds.maxCorner()) > NestUtils::ToNestCoord(m_BoardBinHeight) || !Item.isInside(*_BinPoly)) {
                    return false;
                }
                for (std::size_t OtherIndex = Index + 1; OtherIndex < _Items->size(); ++OtherIndex) {
                    if ((*_Items)[OtherIndex].binId() != Item.binId()) {
                        continue;
                    }
                    CetNestItem Other = (*_Items)[OtherIndex];
                    Other.inflation(0);
                    if (m_SpacingCoord > 0) {
                        Item.inflation(static_cast<decltype(Item.inflation())>(m_SpacingCoord));
                    }
                    if (CetNestItem::intersects(Item, Other)) {
                        return false;
                    }
                }
            }
            return _CountUsedBins() > 0;
        }

        bool CetPolygonBoardRepairer::_CanContinueSearch()
        {
            const bool HasChecks = m_RemainingPlacementChecks > 0;
            const bool HasTime = std::chrono::steady_clock::now() < m_SearchDeadline;
            if (!HasChecks || !HasTime) {
                m_SearchBudgetReached = true;
                return false;
            }
            return true;
        }

        double CetPolygonBoardRepairer::_GetEffectiveGridStep(long long ACheckLimit) const
        {
            if (ACheckLimit <= 0 || m_Rotations.empty() || m_StepMm <= 0.0) {
                return m_StepMm;
            }
            const long double XCount = std::ceil(m_BoardBinWidth / m_StepMm);
            const long double YCount = std::ceil(m_BoardBinHeight / m_StepMm);
            const long double TotalChecks = XCount * YCount * static_cast<long double>(m_Rotations.size());
            if (TotalChecks <= static_cast<long double>(ACheckLimit)) {
                return m_StepMm;
            }
            const double Scale = std::ceil(std::sqrt(static_cast<double>(TotalChecks / ACheckLimit)));
            return m_StepMm * std::max(1.0, Scale);
        }

        bool CetPolygonBoardRepairer::_TryPlaceItemInBinByGrid(std::size_t AItemIndex,int ATargetBin)
        {
            if (_Items == nullptr || _Options == nullptr || _BinPoly == nullptr){
                return false;
            }

            auto& Items = *_Items;

            if (AItemIndex >= Items.size()){
                return false;
            }

            if (!_CanContinueSearch()) {
                return false;
            }
            const long long CheckLimit = std::min(m_PerItemPlacementCheckLimit, m_RemainingPlacementChecks);
            const double GridStep = _GetEffectiveGridStep(CheckLimit);
            long long CheckedCount = 0;
            for (const auto Angle : m_Rotations){
                for (double Y = 0.0; Y < m_BoardBinHeight; Y += GridStep){
                    for (double X = 0.0; X < m_BoardBinWidth; X += GridStep){
                        if (CheckedCount >= CheckLimit || !_CanContinueSearch()) {
                            return false;
                        }
                        ++CheckedCount;
                        ++m_PlacementChecks;
                        --m_RemainingPlacementChecks;
                        TetPlacementCandidate Placement;

                        Placement.ItemIndex = AItemIndex;
                        Placement.TargetBin = ATargetBin;
                        Placement.Rotation = Angle;

                        _FillTranslationForBBoxMin(Placement,X,Y);

                        if (_CanPlaceAt(Placement)){
                            Items[AItemIndex].translation(Placement.Translation);
                            Items[AItemIndex].rotation(Placement.Rotation);
                            Items[AItemIndex].binId(ATargetBin);

                            std::cout << "[REPAIR] item " << AItemIndex << " moved to bin " << ATargetBin << ", x = " << X << ", y = " << Y << std::endl;

                            return true;
                        }
                    }
                }
            }

            return false;
        }

        bool CetPolygonBoardRepairer::_CanPlaceAt(const TetPlacementCandidate& APlacement)
        {
            if (_Items == nullptr || _BinPoly == nullptr){
                return false;
            }

            auto& Items = *_Items;
            const auto& BinPoly = *_BinPoly;

            using NestItemType = CetTNestItemVector::value_type;

            if (APlacement.ItemIndex >= Items.size()){
                return false;
            }

            auto& Candidate = Items[APlacement.ItemIndex];

            Point OldTranslation = Candidate.translation();
            libnest2d::Radians OldRotation = Candidate.rotation();
            int OldBin = static_cast<int>(Candidate.binId());
            auto OldInflation = Candidate.inflation();

            Candidate.translation(APlacement.Translation);
            Candidate.rotation(APlacement.Rotation);
            Candidate.binId(APlacement.TargetBin);
            Candidate.inflation(0);

            bool CanPlace = true;

            auto BB = Candidate.boundingBox();

            if (getX(BB.minCorner()) < 0 ||getY(BB.minCorner()) < 0){
                CanPlace = false;
            }
            if (CanPlace &&(getX(BB.maxCorner()) > NestUtils::ToNestCoord(m_BoardBinWidth) ||getY(BB.maxCorner()) > NestUtils::ToNestCoord(m_BoardBinHeight))){
                CanPlace = false;
            }
            
            if (CanPlace && !Candidate.isInside(BinPoly)){
                CanPlace = false;
            }

            
            if (CanPlace && m_SpacingCoord > 0){
                Candidate.inflation(static_cast<decltype(OldInflation)>(m_SpacingCoord));
            }

            const auto InflatedBounds = Candidate.boundingBox();

            if (CanPlace){
                for (std::size_t i = 0; i < Items.size(); ++i){
                    if (i == APlacement.ItemIndex){
                        continue;
                    }

                    const auto& Other = Items[i];
                    int OtherBin = static_cast<int>(Other.binId());

                    if (OtherBin != APlacement.TargetBin || OtherBin < 0){
                        continue;
                    }

                    const auto OtherBounds = Other.boundingBox();
                    if (getX(InflatedBounds.maxCorner()) < getX(OtherBounds.minCorner()) ||
                        getX(InflatedBounds.minCorner()) > getX(OtherBounds.maxCorner()) ||
                        getY(InflatedBounds.maxCorner()) < getY(OtherBounds.minCorner()) ||
                        getY(InflatedBounds.minCorner()) > getY(OtherBounds.maxCorner())) {
                        continue;
                    }

                    if (NestItemType::intersects(Candidate, Other)){
                        CanPlace = false;
                        break;
                    }
                }
            }

            Candidate.translation(OldTranslation);
            Candidate.rotation(OldRotation);
            Candidate.binId(OldBin);
            Candidate.inflation(OldInflation);

            return CanPlace;
        }

        void CetPolygonBoardRepairer::_FillTranslationForBBoxMin(TetPlacementCandidate& APlacement,double ATargetMinX,double ATargetMinY)
        {
            if (_Items == nullptr){
                APlacement.Translation = Point(0, 0);
                return;
            }

            auto& Items = *_Items;

            if (APlacement.ItemIndex >= Items.size()){
                APlacement.Translation = Point(0, 0);
                return;
            }

            auto& Item = Items[APlacement.ItemIndex];

            Point OldTranslation = Item.translation();
            libnest2d::Radians OldRotation = Item.rotation();

            
            Item.translation(Point(0, 0));
            Item.rotation(APlacement.Rotation);

            auto BB = Item.boundingBox();

            Point DesiredMin(NestUtils::ToNestCoord(ATargetMinX),NestUtils::ToNestCoord(ATargetMinY));

            
            APlacement.Translation = DesiredMin - BB.minCorner();

            Item.translation(OldTranslation);
            Item.rotation(OldRotation);
        }

        bool CetPolygonBoardRepairer::_IsCurrentPlacementValid(std::size_t AItemIndex)
        {
            if (_Items == nullptr){
                return false;
            }

            auto& Items = *_Items;

            if (AItemIndex >= Items.size()){
                return false;
            }

            int CurrentBin = static_cast<int>(Items[AItemIndex].binId());

            if (CurrentBin < 0){
                return false;
            }

            TetPlacementCandidate Placement;

            Placement.ItemIndex = AItemIndex;
            Placement.TargetBin = CurrentBin;
            Placement.Translation = Items[AItemIndex].translation();
            Placement.Rotation = Items[AItemIndex].rotation();

            return _CanPlaceAt(Placement);
        }

        void CetPolygonBoardRepairer::_FixInvalidItems(std::size_t& ALayers)
        {
            if (_Items == nullptr || _Options == nullptr || _BinPoly == nullptr){
                return;
            }

            auto& Items = *_Items;

            if (ALayers == 0){
                return;
            }

            for (std::size_t i = 0; i < Items.size(); ++i){
                if (!_CanContinueSearch()) {
                    std::cout << "[REPAIR][SEARCH LIMIT] Stop invalid-item scan at item " << i << std::endl;
                    break;
                }
                int OriginalBin = static_cast<int>(Items[i].binId());

                if (OriginalBin < 0){
                    continue;
                }

                if (_IsCurrentPlacementValid(i)){
                    continue;
                }

                std::cout << "[REPAIR] item " << i << " current placement invalid. Try relocate." << std::endl;

                Point OldTranslation = Items[i].translation();
                libnest2d::Radians OldRotation = Items[i].rotation();
                int OldBin = OriginalBin;

                
                Items[i].binId(-1);

                bool Placed = false;

                
                for (int TargetBin = 0; TargetBin < static_cast<int>(ALayers); ++TargetBin){
                    if (_TryPlaceItemInBinByGrid(i, TargetBin)){
                        Placed = true;

                        std::cout << "[REPAIR] invalid item " << i << " relocated to existing bin " << TargetBin << std::endl;

                        break;
                    }
                }

                
                if (!Placed){
                    int NewBin = static_cast<int>(ALayers);

                    if (_TryPlaceItemInBinByGrid(i, NewBin)){
                        Placed = true;
                        ++ALayers;

                        std::cout << "[REPAIR] invalid item " << i << " relocated to new bin " << NewBin << std::endl;
                    }
                }

                
                if (!Placed){
                    Items[i].translation(OldTranslation);
                    Items[i].rotation(OldRotation);
                    Items[i].binId(OldBin);

                    std::cout << "[REPAIR][WARN] item " << i << " cannot be relocated." << std::endl;
                }
            }
        }

        void CetPolygonBoardRepairer::_FillHoles(std::size_t& ALayers)
        {
            if (_Items == nullptr || ALayers <= 1){
                return;
            }
			auto& Items = *_Items;
			const int BeforeUsedBins = _CountUsedBins();
			const double BeforeFirstBinArea = _CalculateBinOccupiedArea(0);
			const long long BeforePlacementChecks = m_PlacementChecks;
			std::size_t AcceptedMoves = 0;
			bool Changed = true;
			int Iteration = 0;
			int MaxIterations = static_cast<int>(Items.size())*3;

			while (Changed && Iteration < MaxIterations){
				if (!_CanContinueSearch()) {
					break;
				}
                Changed = false;
                ++Iteration;

                for(int TargetBin = 0; TargetBin < static_cast<int>(ALayers); ++TargetBin){
					if (!_CanContinueSearch()) {
						break;
					}
					std::vector<TetClusterFreeRegion> FreeRegions;
					const bool ExtractedFreeRegions = _ExtractBoardFreeRegions(TargetBin, FreeRegions);
					std::cout << "[BOARD FILL][SEARCH] Bin=" << TargetBin
						<< " FreeRegionCount=" << FreeRegions.size()
						<< " Extracted=" << ExtractedFreeRegions << std::endl;
					TetHoleFillCandidate BestCandidate;
                    if (!_FindBestCandidateForTargetBin(TargetBin, FreeRegions, BestCandidate)){
                        continue;
                    }
					if (!_ApplyHoleFillCandidate(BestCandidate)) {
						continue;
					}
                    Changed = true;
					++AcceptedMoves;
					std::cout << "[BOARD FILL][MOVE] Item=" << BestCandidate.ItemIndex
						<< " From=" << BestCandidate.OldBin << " To=" << BestCandidate.TargetBin
						<< " Score=" << BestCandidate.Score << std::endl;
                }
                ALayers = _CompactItemBins();
            }
			std::cout << "[BOARD FILL][SUMMARY] MoveAttempts=" << (m_PlacementChecks - BeforePlacementChecks)
				<< " AcceptedMoves=" << AcceptedMoves << " BeforeFirstBinArea=" << BeforeFirstBinArea
				<< " AfterFirstBinArea=" << _CalculateBinOccupiedArea(0)
				<< " BeforeUsedBins=" << BeforeUsedBins << " AfterUsedBins=" << _CountUsedBins()
				<< " Iterations=" << Iteration << std::endl;
        }

        bool CetPolygonBoardRepairer::_ExtractBoardFreeRegions(int ATargetBin, std::vector<TetClusterFreeRegion>& AOutRegions) const
        {
            AOutRegions.clear();
            if (_Items == nullptr || _BinPoly == nullptr || ATargetBin < 0 || _BinPoly->Contour.size() < 3) return false;
            ClipperLib::Paths ReservedContours;
            if (!_BuildBoardReservedContours(ATargetBin, ReservedContours)) return false;
            ClipperLib::Clipper DifferenceClipper;
            if (!DifferenceClipper.AddPath(_BinPoly->Contour, ClipperLib::ptSubject, true)) return false;
            if (!_BinPoly->Holes.empty() && !DifferenceClipper.AddPaths(_BinPoly->Holes, ClipperLib::ptSubject, true)) return false;
            if (!ReservedContours.empty() && !DifferenceClipper.AddPaths(ReservedContours, ClipperLib::ptClip, true)) return false;
            ClipperLib::PolyTree Tree;
            if (!DifferenceClipper.Execute(ClipperLib::ctDifference, Tree, ClipperLib::pftNonZero, ClipperLib::pftNonZero)) return false;
            for (const ClipperLib::PolyNode* Node : Tree.Childs) if (Node != nullptr && !_AppendBoardFreeRegion(*Node, AOutRegions)) return false;
            std::stable_sort(AOutRegions.begin(), AOutRegions.end(), [](const TetClusterFreeRegion& AFirst, const TetClusterFreeRegion& ASecond) {
                if (std::abs(AFirst.Area - ASecond.Area) > CET_CLUSTER_GEOMETRY_AREA_TOLERANCE) return AFirst.Area > ASecond.Area;
                if (AFirst.MinY != ASecond.MinY) return AFirst.MinY < ASecond.MinY;
                return AFirst.MinX < ASecond.MinX;
                });
            if (AOutRegions.size() > CET_BOARD_FILL_MAX_FREE_REGIONS) AOutRegions.resize(CET_BOARD_FILL_MAX_FREE_REGIONS);
            return !AOutRegions.empty();
        }

        bool CetPolygonBoardRepairer::_BuildBoardReservedContours(int ATargetBin, ClipperLib::Paths& AOutContours) const
        {
            AOutContours.clear();
            if (_Items == nullptr || ATargetBin < 0) return false;
            for (const CetNestItem& SourceItem : *_Items) {
                if (SourceItem.binId() != ATargetBin) continue;
                CetNestItem Item = SourceItem;
                Item.inflation(0);
                const CetPolygonImpl& Shape = Item.transformedShape();
                CetPath Contour = Shape.Contour;
                ClipperLib::CleanPolygon(Contour, 1.0);
                if (Contour.size() < 3 || std::abs(ClipperLib::Area(Contour)) <= 0.0) return false;
                if (!ClipperLib::Orientation(Contour)) std::reverse(Contour.begin(), Contour.end());
                AOutContours.push_back(std::move(Contour));
                for (CetPath Hole : Shape.Holes) {
                    ClipperLib::CleanPolygon(Hole, 1.0);
                    if (Hole.size() < 3 || std::abs(ClipperLib::Area(Hole)) <= 0.0) return false;
                    if (ClipperLib::Orientation(Hole)) std::reverse(Hole.begin(), Hole.end());
                    AOutContours.push_back(std::move(Hole));
                }
            }
            if (AOutContours.empty() || m_SpacingCoord <= 0) return true;
            ClipperLib::Paths OffsetContours;
            ClipperLib::ClipperOffset OffsetBuilder(2.0, std::max(1.0, static_cast<double>(m_SpacingCoord) * 0.02));
            OffsetBuilder.AddPaths(AOutContours, ClipperLib::jtRound, ClipperLib::etClosedPolygon);
            OffsetBuilder.Execute(OffsetContours, m_SpacingCoord);
            if (OffsetContours.empty()) return false;
            AOutContours = std::move(OffsetContours);
            return true;
        }

        bool CetPolygonBoardRepairer::_AppendBoardFreeRegion(const ClipperLib::PolyNode& ANode, std::vector<TetClusterFreeRegion>& AOutRegions) const
        {
            if (!ANode.IsHole() && ANode.Contour.size() >= 3) {
                TetClusterFreeRegion Region;
                Region.Contour = ANode.Contour;
                Region.IsClosed = true;
                Region.Area = std::abs(static_cast<double>(ClipperLib::Area(Region.Contour)));
                if (!std::isfinite(Region.Area) || Region.Area <= 0.0) return false;
                Region.MinX = Region.MaxX = static_cast<double>(Region.Contour.front().X);
                Region.MinY = Region.MaxY = static_cast<double>(Region.Contour.front().Y);
                for (const ClipperLib::IntPoint& Point : Region.Contour) {
                    Region.MinX = std::min(Region.MinX, static_cast<double>(Point.X));
                    Region.MinY = std::min(Region.MinY, static_cast<double>(Point.Y));
                    Region.MaxX = std::max(Region.MaxX, static_cast<double>(Point.X));
                    Region.MaxY = std::max(Region.MaxY, static_cast<double>(Point.Y));
                }
                for (const ClipperLib::PolyNode* Child : ANode.Childs) {
                    if (Child == nullptr || !Child->IsHole()) continue;
                    const double HoleArea = std::abs(static_cast<double>(ClipperLib::Area(Child->Contour)));
                    if (!std::isfinite(HoleArea) || HoleArea >= Region.Area) return false;
                    Region.Area -= HoleArea;
                    Region.Holes.push_back(Child->Contour);
                }
                Region.Width = Region.MaxX - Region.MinX;
                Region.Height = Region.MaxY - Region.MinY;
                if (Region.Area <= 0.0 || Region.Width <= 0.0 || Region.Height <= 0.0) return false;
                AOutRegions.push_back(std::move(Region));
            }
            for (const ClipperLib::PolyNode* Child : ANode.Childs) if (Child != nullptr && !_AppendBoardFreeRegion(*Child, AOutRegions)) return false;
            return true;
        }

        bool CetPolygonBoardRepairer::_FindBestCandidateForTargetBin(int ATargetBin, const std::vector<TetClusterFreeRegion>& AFreeRegions, TetHoleFillCandidate& ABestCandidate)
        {
            if (_Items == nullptr) return false;
            bool Found = false;
            for (std::size_t i = 0; i < _Items->size(); ++i) {
                const int OldBin = static_cast<int>((*_Items)[i].binId());
                if (OldBin >= 0 && OldBin <= ATargetBin) continue;
                TetHoleFillCandidate Candidate;
                if (_TryFindBestPlacementInBin(i, ATargetBin, AFreeRegions, Candidate) && (!Found || Candidate.Score > ABestCandidate.Score)) {
                    ABestCandidate = Candidate;
                    Found = true;
                }
            }
            return Found;
        }

        bool CetPolygonBoardRepairer::_TryFindBestPlacementInBin(std::size_t AItemIndex, int ATargetBin, const std::vector<TetClusterFreeRegion>& AFreeRegions, TetHoleFillCandidate& ABestCandidate)
        {
            if (_Items == nullptr || _Options == nullptr || _BinPoly == nullptr || AItemIndex >= _Items->size() || !_CanContinueSearch()) return false;
            const int OldBin = static_cast<int>((*_Items)[AItemIndex].binId());
            const long long CheckLimit = std::min(m_PerItemPlacementCheckLimit, m_RemainingPlacementChecks);
            const double GridStep = _GetEffectiveGridStep(CheckLimit);
            long long CheckedCount = 0;
            bool Found = false;
            for (const Radians Angle : m_Rotations) {
                CetNestItem RotatedItem = (*_Items)[AItemIndex];
                RotatedItem.translation(Point(0, 0));
                RotatedItem.rotation(Angle);
                RotatedItem.inflation(0);
                const auto RotatedBounds = RotatedItem.boundingBox();
                const double ItemWidth = static_cast<double>(RotatedBounds.width());
                const double ItemHeight = static_cast<double>(RotatedBounds.height());
                for (const TetClusterFreeRegion& FreeRegion : AFreeRegions) {
                    if (ItemWidth > FreeRegion.Width || ItemHeight > FreeRegion.Height) continue;
                    for (double Y = NestUtils::FromNestCoord(static_cast<decltype(NestUtils::ToNestCoord(0.0))>(std::llround(FreeRegion.MinY))); Y <= NestUtils::FromNestCoord(static_cast<decltype(NestUtils::ToNestCoord(0.0))>(std::llround(FreeRegion.MaxY))); Y += GridStep) {
                        for (double X = NestUtils::FromNestCoord(static_cast<decltype(NestUtils::ToNestCoord(0.0))>(std::llround(FreeRegion.MinX))); X <= NestUtils::FromNestCoord(static_cast<decltype(NestUtils::ToNestCoord(0.0))>(std::llround(FreeRegion.MaxX))); X += GridStep) {
                            if (CheckedCount >= CheckLimit || !_CanContinueSearch()) return Found;
                            ++CheckedCount;
                            ++m_PlacementChecks;
                            --m_RemainingPlacementChecks;
                            TetPlacementCandidate Placement;
                            Placement.ItemIndex = AItemIndex;
                            Placement.TargetBin = ATargetBin;
                            Placement.Rotation = Angle;
                            _FillTranslationForBBoxMin(Placement, X, Y);
                            if (!_IsPlacementInsideFreeRegion(Placement, FreeRegion) || !_CanPlaceAt(Placement)) continue;
                            const double Score = _CalcHoleFillScore(AItemIndex, OldBin, ATargetBin, Placement.Translation);
                            if (!Found || Score > ABestCandidate.Score) {
                                ABestCandidate = { true, AItemIndex, OldBin, ATargetBin, Placement.Translation, Placement.Rotation, Score };
                                Found = true;
                            }
                        }
                    }
                }
            }
            return Found;
        }

        bool CetPolygonBoardRepairer::_IsPlacementInsideFreeRegion(const TetPlacementCandidate& APlacement, const TetClusterFreeRegion& AFreeRegion) const
        {
            if (_Items == nullptr || APlacement.ItemIndex >= _Items->size()) return false;
            CetNestItem Item = (*_Items)[APlacement.ItemIndex];
            Item.translation(APlacement.Translation);
            Item.rotation(APlacement.Rotation);
            Item.inflation(0);
            const double Tolerance = std::max(1.0, AFreeRegion.Area * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE);
            CetClusterGeometryHelper Geometry;
            return Geometry.IsContourInsideFreeRegion(Item.transformedShape().Contour, AFreeRegion, Tolerance);
        }

        bool CetPolygonBoardRepairer::_ApplyHoleFillCandidate(const TetHoleFillCandidate& ACandidate)
        {
            if (_Items == nullptr || !ACandidate.Valid || ACandidate.ItemIndex >= _Items->size()) return false;
            CetNestItem& Item = (*_Items)[ACandidate.ItemIndex];
            const Point OldTranslation = Item.translation();
            const Radians OldRotation = Item.rotation();
            const int OldBin = Item.binId();
            Item.translation(ACandidate.Translation);
            Item.rotation(ACandidate.Rotation);
            Item.binId(ACandidate.TargetBin);
            if (_IsCurrentPlacementValid(ACandidate.ItemIndex)) return true;
            Item.translation(OldTranslation);
            Item.rotation(OldRotation);
            Item.binId(OldBin);
            return false;
        }

        double CetPolygonBoardRepairer::_CalcHoleFillScore(std::size_t AItemIndex, int AOldBin, int ATargetBin, const libnest2d::Point& ATranslation)
        {
            if (_Items == nullptr || AItemIndex >= _Items->size()){
                return -std::numeric_limits<double>::max();
            }
            const auto& Item = (*_Items)[AItemIndex];
            double ItemArea = std::abs(static_cast<double>(Item.area()));
            
            double BinImprove = static_cast<double>(AOldBin - ATargetBin);
           
            double XPenalty = static_cast<double>(ATranslation.X) * 0.000001;
            double YPenalty = static_cast<double>(ATranslation.Y) * 0.000001;
            double Score =ItemArea * 10.0+ BinImprove * 1000000.0- XPenalty- YPenalty;
            return Score;
        }

        double CetPolygonBoardRepairer::_CalculateBinOccupiedArea(int ABinId) const
        {
            if (_Items == nullptr || ABinId < 0) return 0.0;
            double Area = 0.0;
            for (const CetNestItem& Item : *_Items) {
                if (Item.binId() == ABinId) Area += std::abs(static_cast<double>(Item.area()));
            }
            return Area;
        }

    } // namespace NEST2DMANAGERLIB
} // namespace ET
