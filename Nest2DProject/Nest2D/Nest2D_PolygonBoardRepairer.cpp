#include "pch.h"
#include "Nest2D_PolygonBoardRepairer.h"
#include "Nest2D_ClusterGeometryHelper.h"
#include "Nest2D_ClusterManager.h"
#include "Nest2D_RectangleFillClusterBuilder.h"
#include "Nest2D_RotationUtils.h"
#include "Nest2D_SelfFunction.h"
#include "Nest2D_ShapeAnalyzer.h"
#include "Nest2D_StrategyManager.h"
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
        thread_local bool CetPolygonBoardRepairer::_HadBoardFillChanges = false;
        /*  struct TetPlacementCandidate
          {
              TetPlacementCandidate(): ItemIndex(0), TargetBin(-1), Translation(Point(0, 0)), Rotation(libnest2d::Radians(0.0)){
              }

              std::size_t ItemIndex;
              int TargetBin;
              Point Translation;
              libnest2d::Radians Rotation;
          };*/
        CetPolygonBoardRepairer::CetPolygonBoardRepairer() : CetCoreObject() {}
        CetPolygonBoardRepairer::~CetPolygonBoardRepairer() {}
        CetPolygonBoardRepairer::CetPolygonBoardRepairer(CetTNestItemVector &ANestItems, const TetNestOptions &AOptions, const CetPolygonImpl &ABinPoly, double ABoardBinWidth, double ABoardBinHeight) : CetCoreObject() { SetContext(ANestItems, AOptions, ABinPoly, ABoardBinWidth, ABoardBinHeight); }
        void CetPolygonBoardRepairer::SetContext(CetTNestItemVector &ANestItems, const TetNestOptions &AOptions, const CetPolygonImpl &ABinPoly, double ABoardBinWidth, double ABoardBinHeight)
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
            _HadBoardFillChanges = false;
            m_LockedItemIndices.clear();
            _BuildRotations();
        }
        void CetPolygonBoardRepairer::Repair(std::size_t &ALayers)
        {
            if (_Items == nullptr || _Options == nullptr || _BinPoly == nullptr) {
                std::cout << "[REPAIR][ERROR] Repairer context is null." << std::endl;
                return;
            }
            if (ALayers == 0) {
                PackFromScratch(ALayers);
                return;
            }
            auto &Items = *_Items;
            std::cout << "[REPAIR] start polygon board repair. Layers = " << ALayers << ", StepMm = " << m_StepMm << std::endl;
            _FixInvalidItems(ALayers);
            if (ALayers <= 1) {
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
        bool CetPolygonBoardRepairer::RepairLockedEnvelope(std::size_t &ALayers, const std::vector<std::size_t> &ALockedItems)
        {
            if (_Items == nullptr || _Options == nullptr || _BinPoly == nullptr || ALayers == 0)
                return false;
            m_LockedItemIndices = ALockedItems;
            m_RemainingPlacementChecks = std::min(m_RemainingPlacementChecks, CET_LOCKED_ENVELOPE_LOCAL_FILL_MAX_PLACEMENT_CHECKS);
            m_SearchDeadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(CET_LOCKED_ENVELOPE_LOCAL_FILL_MAX_SEARCH_TIME_MS);
            m_SearchBudgetReached = false;
            std::vector<int> TargetBins;
            for (std::size_t Index : m_LockedItemIndices)
                if (Index < _Items->size()) {
                    const int Bin = static_cast<int>((*_Items)[Index].binId());
                    if (Bin >= 0 && std::find(TargetBins.begin(), TargetBins.end(), Bin) == TargetBins.end())
                        TargetBins.push_back(Bin);
                }
            for (int Bin = 0; Bin < static_cast<int>(ALayers); ++Bin)
                if (std::find(TargetBins.begin(), TargetBins.end(), Bin) == TargetBins.end())
                    TargetBins.push_back(Bin);
            bool Changed = false;
            for (int Pass = 0; Pass < CET_LOCKED_ENVELOPE_LOCAL_FILL_MAX_PASSES && _CanContinueSearch(); ++Pass) {
                bool PassChanged = false;
                for (int Bin : TargetBins) {
                    if (Bin >= static_cast<int>(ALayers) || !_CanContinueSearch())
                        break;
                    std::vector<TetClusterFreeRegion> FreeRegions;
                    if (!_ExtractBoardFreeRegions(Bin, FreeRegions))
                        continue;
                    TetBoardLocalFillCandidate Candidate;
                    if (!_FindBestLocalCandidateForTargetBin(Bin, FreeRegions, Candidate) || !_ApplyLocalFillCandidate(Candidate))
                        continue;
                    _HadBoardFillChanges = true;
                    Changed = true;
                    PassChanged = true;
                    std::cout << "[LOCKED ENVELOPE][LOCAL FILL] Bin=" << Bin << " Parts=" << Candidate.Placements.size() << " AreaGain=" << Candidate.OccupiedAreaGain << std::endl;
                    break;
                }
                if (!PassChanged)
                    break;
                ALayers = _CompactItemBins();
            }
            if (m_SearchBudgetReached)
                std::cout << "[LOCKED ENVELOPE][LOCAL FILL] Search limit reached." << std::endl;
            m_LockedItemIndices.clear();
            return Changed;
        }
        bool CetPolygonBoardRepairer::EvacuateLastBin(std::size_t &ALayers, TetLastBinEvacuationStats &AStats)
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
                        std::cout << "[LAST_BIN][MOVE] Item=" << ItemIndex << " From=" << AStats.LastBinId << " To=" << (*_Items)[ItemIndex].binId() << " Mode=Direct" << std::endl;
                        continue;
                    }
                    if (_TryEvacuateItemWithRelocation(ItemIndex, AStats.LastBinId, TargetBins, AStats)) {
                        Changed = true;
                        continue;
                    }
                    ++AStats.NoCandidatePosition;
                    std::cout << "[LAST_BIN][ITEM FAILED] Item=" << ItemIndex << " Area=" << std::abs(static_cast<double>((*_Items)[ItemIndex].area())) << " TriedBins=" << TargetBins.size() << " DirectFailed=1" << std::endl;
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
        void CetPolygonBoardRepairer::PackFromScratch(std::size_t &ALayers)
        {
            if (_Items == nullptr || _Options == nullptr || _BinPoly == nullptr) {
                std::cout << "[PACK][ERROR] context is null." << std::endl;
                ALayers = 0;
                return;
            }
            if (!_Options->Board.Enabled || _Options->Board.Vertices.size() < 3) {
                ALayers = 0;
                return;
            }
            auto &Items = *_Items;
            std::cout << "[PACK] start pack from scratch. Items = " << Items.size() << ", StepMm = " << m_StepMm << std::endl;
            for (auto &Item : Items) {
                Item.binId(-1);
                Item.translation(Point(0, 0));
                Item.rotation(libnest2d::Radians(0.0));
            }
            std::size_t UsedLayers = 0;
            for (std::size_t i = 0; i < Items.size(); ++i) {
                bool Placed = false;
                for (int TargetBin = 0; TargetBin < static_cast<int>(UsedLayers); ++TargetBin) {
                    if (_TryPlaceItemInBinByGrid(i, TargetBin)) {
                        Placed = true;
                        break;
                    }
                }
                if (!Placed) {
                    int NewBin = static_cast<int>(UsedLayers);
                    if (_TryPlaceItemInBinByGrid(i, NewBin)) {
                        Placed = true;
                        ++UsedLayers;
                        std::cout << "[PACK] item " << i << " placed in new bin " << NewBin << std::endl;
                    }
                }
                if (!Placed) {
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
            if (_Items == nullptr) {
                return 0;
            }
            auto &Items = *_Items;
            std::map<int, int> Remap;
            int NextBin = 0;
            for (auto &Item : Items) {
                int OldBin = static_cast<int>(Item.binId());
                if (OldBin < 0) {
                    continue;
                }
                auto It = Remap.find(OldBin);
                if (It == Remap.end()) {
                    Remap[OldBin] = NextBin;
                    Item.binId(NextBin);
                    ++NextBin;
                } else {
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
                for (const CetNestItem &Item : *_Items) {
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
        void CetPolygonBoardRepairer::_CaptureLastBinStats(const std::vector<std::size_t> &AItemIndices, TetLastBinEvacuationStats &AStats) const
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
                for (const CetNestItem &Item : *_Items) {
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
                for (const CetNestItem &Item : *_Items) {
                    if (Item.binId() == BinId) {
                        OccupiedArea += std::abs(static_cast<double>(Item.area()));
                    }
                }
                Candidates.emplace_back(BinId, BoardArea - OccupiedArea);
            }
            std::stable_sort(Candidates.begin(), Candidates.end(), [](const auto &A, const auto &B) {
                if (std::abs(A.second - B.second) > 1e-9) {
                    return A.second > B.second;
                }
                return A.first < B.first;
            });
            std::vector<int> Result;
            for (const auto &Candidate : Candidates) {
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
            for (const CetNestItem &Item : *_Items) {
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
        bool CetPolygonBoardRepairer::_TryEvacuateItemDirect(std::size_t AItemIndex, const std::vector<int> &ATargetBins)
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
                const CetNestItem &Item = (*_Items)[Index];
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
                const CetNestItem &Item = (*_Items)[AIndex];
                const auto Bounds = Item.boundingBox();
                const double MinX = static_cast<double>(getX(Bounds.minCorner()));
                const double MinY = static_cast<double>(getY(Bounds.minCorner()));
                const double MaxX = static_cast<double>(getX(Bounds.maxCorner()));
                const double MaxY = static_cast<double>(getY(Bounds.maxCorner()));
                const double UsedBoundaryGap = std::min(std::abs(MaxUsedX - MaxX), std::abs(MaxUsedY - MaxY));
                const double BoardEdgeGap = std::min({MinX, MinY, BoardWidth - MaxX, BoardHeight - MaxY});
                return std::array<double, 3>{UsedBoundaryGap, BoardEdgeGap, std::abs(static_cast<double>(Item.area()))};
            };
            std::stable_sort(Result.begin(), Result.end(), [&](std::size_t A, std::size_t B) { return Rank(A) < Rank(B); });
            if (Result.size() > CET_LAST_BIN_MAX_RELOCATION_CANDIDATES) {
                Result.resize(CET_LAST_BIN_MAX_RELOCATION_CANDIDATES);
            }
            return Result;
        }
        bool CetPolygonBoardRepairer::_TryEvacuateItemWithRelocation(std::size_t AItemIndex, int ALastBinId, const std::vector<int> &ATargetBins, TetLastBinEvacuationStats &AStats)
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
                            std::cout << "[LAST_BIN][MOVE] Item=" << AItemIndex << " From=" << ALastBinId << " To=" << TargetBin << " Mode=AfterRelocation" << std::endl;
                            return true;
                        }
                    }
                    *_Items = AttemptSolution;
                }
                ++AStats.RelocationFailed;
            }
            return false;
        }
        bool CetPolygonBoardRepairer::_TryRelocateSmallItemWithinSameBin(std::size_t AItemIndex, int ABinId, double &AScore)
        {
            if (_Items == nullptr || AItemIndex >= _Items->size() || (*_Items)[AItemIndex].binId() != ABinId) {
                return false;
            }
            CetNestItem &Item = (*_Items)[AItemIndex];
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
            std::cout << "[LAST_BIN][SAME_BIN_RELOCATE] Item=" << AItemIndex << " Bin=" << ABinId << " Old=(" << NestUtils::FromNestCoord(OldTranslation.X) << "," << NestUtils::FromNestCoord(OldTranslation.Y) << ") New=(" << NestUtils::FromNestCoord(BestPlacement.Translation.X) << "," << NestUtils::FromNestCoord(BestPlacement.Translation.Y) << ") Score=" << AScore << std::endl;
            return true;
        }
        bool CetPolygonBoardRepairer::_TryFindBestSmallRelocationInBin(std::size_t AItemIndex, int ATargetBin, TetPlacementCandidate &ABestPlacement, double &ABestScore)
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
                        const bool SamePosition = Placement.Translation == OldTranslation && std::abs(static_cast<double>(Placement.Rotation - OldRotation)) <= CET_ROTATION_DUPLICATE_TOLERANCE;
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
        double CetPolygonBoardRepairer::_CalcSmallRelocationScore(const TetPlacementCandidate &APlacement)
        {
            if (_Items == nullptr || APlacement.ItemIndex >= _Items->size()) {
                return -std::numeric_limits<double>::max();
            }
            CetNestItem &Candidate = (*_Items)[APlacement.ItemIndex];
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
                const double GapX = std::max({0.0, static_cast<double>(getX(OtherBounds.minCorner()) - getX(CandidateBounds.maxCorner())), static_cast<double>(getX(CandidateBounds.minCorner()) - getX(OtherBounds.maxCorner()))});
                const double GapY = std::max({0.0, static_cast<double>(getY(OtherBounds.minCorner()) - getY(CandidateBounds.maxCorner())), static_cast<double>(getY(CandidateBounds.minCorner()) - getY(OtherBounds.maxCorner()))});
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
                if (getX(Bounds.minCorner()) < 0 || getY(Bounds.minCorner()) < 0 || getX(Bounds.maxCorner()) > NestUtils::ToNestCoord(m_BoardBinWidth) || getY(Bounds.maxCorner()) > NestUtils::ToNestCoord(m_BoardBinHeight) || !Item.isInside(*_BinPoly)) {
                    return false;
                }
                for (std::size_t OtherIndex = Index + 1; OtherIndex < _Items->size(); ++OtherIndex) {
                    if ((*_Items)[OtherIndex].binId() != Item.binId()) {
                        continue;
                    }
                    CetNestItem Other = (*_Items)[OtherIndex];
                    Other.inflation(0);
                    if (m_SpacingCoord > 0) {
                        const auto HalfSpacing = static_cast<decltype(Item.inflation())>(std::ceil(static_cast<double>(m_SpacingCoord) * 0.5));
                        Item.inflation(HalfSpacing);
                        Other.inflation(HalfSpacing);
                    }
                    if (CetNestItem::intersects(Item, Other) && !CetNestItem::touches(Item, Other)) {
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
        bool CetPolygonBoardRepairer::_TryPlaceItemInBinByGrid(std::size_t AItemIndex, int ATargetBin)
        {
            if (_Items == nullptr || _Options == nullptr || _BinPoly == nullptr) {
                return false;
            }
            auto &Items = *_Items;
            if (AItemIndex >= Items.size()) {
                return false;
            }
            if (!_CanContinueSearch()) {
                return false;
            }
            const long long CheckLimit = std::min(m_PerItemPlacementCheckLimit, m_RemainingPlacementChecks);
            const double GridStep = _GetEffectiveGridStep(CheckLimit);
            long long CheckedCount = 0;
            for (const auto Angle : m_Rotations) {
                for (double Y = 0.0; Y < m_BoardBinHeight; Y += GridStep) {
                    for (double X = 0.0; X < m_BoardBinWidth; X += GridStep) {
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
                        _FillTranslationForBBoxMin(Placement, X, Y);
                        if (_CanPlaceAt(Placement)) {
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
        bool CetPolygonBoardRepairer::_CanPlaceAt(const TetPlacementCandidate &APlacement)
        {
            if (_Items == nullptr || _BinPoly == nullptr) {
                return false;
            }
            auto &Items = *_Items;
            const auto &BinPoly = *_BinPoly;
            using NestItemType = CetTNestItemVector::value_type;
            if (APlacement.ItemIndex >= Items.size()) {
                return false;
            }
            auto &Candidate = Items[APlacement.ItemIndex];
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
            if (getX(BB.minCorner()) < 0 || getY(BB.minCorner()) < 0) {
                CanPlace = false;
            }
            if (CanPlace && (getX(BB.maxCorner()) > NestUtils::ToNestCoord(m_BoardBinWidth) || getY(BB.maxCorner()) > NestUtils::ToNestCoord(m_BoardBinHeight))) {
                CanPlace = false;
            }
            if (CanPlace && !Candidate.isInside(BinPoly)) {
                CanPlace = false;
            }
            if (CanPlace && m_SpacingCoord > 0) {
                Candidate.inflation(static_cast<decltype(OldInflation)>(std::ceil(static_cast<double>(m_SpacingCoord) * 0.5)));
            }
            const auto InflatedBounds = Candidate.boundingBox();
            if (CanPlace) {
                for (std::size_t i = 0; i < Items.size(); ++i) {
                    if (i == APlacement.ItemIndex) {
                        continue;
                    }
                    CetNestItem Other = Items[i];
                    int OtherBin = static_cast<int>(Other.binId());
                    if (OtherBin != APlacement.TargetBin || OtherBin < 0) {
                        continue;
                    }
                    const auto OtherBounds = Other.boundingBox();
                    if (getX(InflatedBounds.maxCorner()) < getX(OtherBounds.minCorner()) || getX(InflatedBounds.minCorner()) > getX(OtherBounds.maxCorner()) || getY(InflatedBounds.maxCorner()) < getY(OtherBounds.minCorner()) || getY(InflatedBounds.minCorner()) > getY(OtherBounds.maxCorner())) {
                        continue;
                    }
                    Other.inflation(0);
                    if (m_SpacingCoord > 0) {
                        Other.inflation(static_cast<decltype(Other.inflation())>(std::ceil(static_cast<double>(m_SpacingCoord) * 0.5)));
                    }
                    if (NestItemType::intersects(Candidate, Other) && !NestItemType::touches(Candidate, Other)) {
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
        void CetPolygonBoardRepairer::_FillTranslationForBBoxMin(TetPlacementCandidate &APlacement, double ATargetMinX, double ATargetMinY)
        {
            if (_Items == nullptr) {
                APlacement.Translation = Point(0, 0);
                return;
            }
            auto &Items = *_Items;
            if (APlacement.ItemIndex >= Items.size()) {
                APlacement.Translation = Point(0, 0);
                return;
            }
            auto &Item = Items[APlacement.ItemIndex];
            Point OldTranslation = Item.translation();
            libnest2d::Radians OldRotation = Item.rotation();
            Item.translation(Point(0, 0));
            Item.rotation(APlacement.Rotation);
            auto BB = Item.boundingBox();
            Point DesiredMin(NestUtils::ToNestCoord(ATargetMinX), NestUtils::ToNestCoord(ATargetMinY));
            APlacement.Translation = DesiredMin - BB.minCorner();
            Item.translation(OldTranslation);
            Item.rotation(OldRotation);
        }
        bool CetPolygonBoardRepairer::_IsCurrentPlacementValid(std::size_t AItemIndex)
        {
            if (_Items == nullptr) {
                return false;
            }
            auto &Items = *_Items;
            if (AItemIndex >= Items.size()) {
                return false;
            }
            int CurrentBin = static_cast<int>(Items[AItemIndex].binId());
            if (CurrentBin < 0) {
                return false;
            }
            TetPlacementCandidate Placement;
            Placement.ItemIndex = AItemIndex;
            Placement.TargetBin = CurrentBin;
            Placement.Translation = Items[AItemIndex].translation();
            Placement.Rotation = Items[AItemIndex].rotation();
            return _CanPlaceAt(Placement);
        }
        void CetPolygonBoardRepairer::_FixInvalidItems(std::size_t &ALayers)
        {
            if (_Items == nullptr || _Options == nullptr || _BinPoly == nullptr) {
                return;
            }
            auto &Items = *_Items;
            if (ALayers == 0) {
                return;
            }
            for (std::size_t i = 0; i < Items.size(); ++i) {
                if (!_CanContinueSearch()) {
                    std::cout << "[REPAIR][SEARCH LIMIT] Stop invalid-item scan at item " << i << std::endl;
                    break;
                }
                int OriginalBin = static_cast<int>(Items[i].binId());
                if (OriginalBin < 0) {
                    continue;
                }
                if (_IsCurrentPlacementValid(i)) {
                    continue;
                }
                std::cout << "[REPAIR] item " << i << " current placement invalid. Try relocate." << std::endl;
                Point OldTranslation = Items[i].translation();
                libnest2d::Radians OldRotation = Items[i].rotation();
                int OldBin = OriginalBin;
                Items[i].binId(-1);
                bool Placed = false;
                for (int TargetBin = 0; TargetBin < static_cast<int>(ALayers); ++TargetBin) {
                    if (_TryPlaceItemInBinByGrid(i, TargetBin)) {
                        Placed = true;
                        std::cout << "[REPAIR] invalid item " << i << " relocated to existing bin " << TargetBin << std::endl;
                        break;
                    }
                }
                if (!Placed) {
                    int NewBin = static_cast<int>(ALayers);
                    if (_TryPlaceItemInBinByGrid(i, NewBin)) {
                        Placed = true;
                        ++ALayers;
                        std::cout << "[REPAIR] invalid item " << i << " relocated to new bin " << NewBin << std::endl;
                    }
                }
                if (!Placed) {
                    Items[i].translation(OldTranslation);
                    Items[i].rotation(OldRotation);
                    Items[i].binId(OldBin);
                    std::cout << "[REPAIR][WARN] item " << i << " cannot be relocated." << std::endl;
                }
            }
        }
        void CetPolygonBoardRepairer::_FillHoles(std::size_t &ALayers)
        {
            if (_Items == nullptr || ALayers <= 1) {
                return;
            }
            auto &Items = *_Items;
            const bool LargeOrder = Items.size() > CET_NEST_FULL_STRATEGY_ITEM_LIMIT;
            const long long CompositeTimeMs = LargeOrder ? CET_BOARD_COMPOSITE_LARGE_ORDER_MAX_SEARCH_TIME_MS : CET_BOARD_COMPOSITE_MAX_SEARCH_TIME_MS;
            const long long CompositeTimePerBinMs = LargeOrder ? CET_BOARD_COMPOSITE_LARGE_ORDER_MAX_SEARCH_TIME_PER_BIN_MS : CET_BOARD_COMPOSITE_MAX_SEARCH_TIME_PER_BIN_MS;
            const auto CompositeDeadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(CompositeTimeMs);
            m_SearchDeadline = CompositeDeadline;
            m_SearchBudgetReached = false;
            const int BeforeUsedBins = _CountUsedBins();
            const double BeforeFirstBinArea = _CalculateBinOccupiedArea(0);
            const long long BeforePlacementChecks = m_PlacementChecks;
            TetBoardCompositeSearchStats CompositeStats;
            std::vector<long long> CompositeExactChecksByBin(ALayers, 0);
            std::vector<std::size_t> CompositeRollbackBins;
            std::vector<std::size_t> CompositeMovedItems;
            std::size_t AcceptedMoves = 0;
            bool Changed = true;
            int Iteration = 0;
            int MaxIterations = static_cast<int>(Items.size()) * 3;
            while (Changed && Iteration < MaxIterations) {
                if (!_CanContinueSearch()) {
                    break;
                }
                Changed = false;
                ++Iteration;
                Changed = _RunBoardCompositePass({ALayers, CompositeDeadline, CompositeTimePerBinMs, CompositeExactChecksByBin, CompositeRollbackBins, CompositeMovedItems, CompositeStats, AcceptedMoves});
                if (!Changed)
                    Changed = _RunLegacyBoardFillPass(ALayers, AcceptedMoves);
                ALayers = _CompactItemBins();
            }
            std::cout << "[BOARD FILL][SUMMARY] MoveAttempts=" << (m_PlacementChecks - BeforePlacementChecks) << " AcceptedMoves=" << AcceptedMoves << " BeforeFirstBinArea=" << BeforeFirstBinArea << " AfterFirstBinArea=" << _CalculateBinOccupiedArea(0) << " BeforeUsedBins=" << BeforeUsedBins << " AfterUsedBins=" << _CountUsedBins() << " Iterations=" << Iteration << std::endl;
            std::cout << "[BOARD COMPOSITE][SUMMARY] Candidates=" << CompositeStats.CandidateCount << " Accepted=" << CompositeStats.AcceptedCount << " Rollbacks=" << CompositeStats.RollbackCount << " Fillers=" << CompositeStats.FillerCount << " ExactChecks=" << CompositeStats.ExactPlacementChecks << " AnchoredRanked=" << CompositeStats.RankedAnchoredSkeletons << " MovingRanked=" << CompositeStats.RankedMovingSkeletons << " BuildFail=" << CompositeStats.SkeletonBuildFailures << " EmptyFillers=" << CompositeStats.EmptyFillerSets << " BeamFail=" << CompositeStats.BeamExpansionFailures << " PlacementFail=" << CompositeStats.PlacementFailures << std::endl;
        }
        bool CetPolygonBoardRepairer::_RunBoardCompositePass(const TetBoardCompositePassRequest &ARequest)
        {
            std::size_t &ALayers = ARequest.Layers; const auto &ADeadline = ARequest.Deadline; const long long ATimePerBinMs = ARequest.TimePerBinMs; auto &AExactChecksByBin = ARequest.ExactChecksByBin; auto &ARollbackBins = ARequest.RollbackBins; auto &AMovedItems = ARequest.MovedItems; auto &AStats = ARequest.Stats; std::size_t &AAcceptedMoves = ARequest.AcceptedMoves;
            bool Changed = false;
            for (int TargetBin = 0; TargetBin < static_cast<int>(ALayers); ++TargetBin) {
                if (std::chrono::steady_clock::now() >= ADeadline)
                    break;
                std::vector<TetClusterFreeRegion> FreeRegions;
                const bool Extracted = _ExtractBoardFreeRegions(TargetBin, FreeRegions);
                std::cout << "[BOARD FILL][SEARCH] Bin=" << TargetBin << " FreeRegionCount=" << FreeRegions.size() << " Extracted=" << Extracted << std::endl;
                if (TargetBin >= static_cast<int>(AExactChecksByBin.size()))
                    AExactChecksByBin.resize(TargetBin + 1, 0);
                m_SearchDeadline = std::min(ADeadline, std::chrono::steady_clock::now() + std::chrono::milliseconds(ATimePerBinMs));
                m_SearchBudgetReached = false;
                TetBoardCompositeCandidate Candidate;
                const bool CanRollback = std::count(ARollbackBins.begin(), ARollbackBins.end(), static_cast<std::size_t>(TargetBin)) < CET_BOARD_COMPOSITE_MAX_ROLLBACKS_PER_BIN;
                if (CanRollback && _FindBestBoardCompositeForBin(TargetBin, FreeRegions, AExactChecksByBin[TargetBin], Candidate, AStats)) {
                    const bool ReusesMovedItem = std::any_of(Candidate.Placements.begin(), Candidate.Placements.end(), [&](const TetHoleFillCandidate &APlacement) { return std::find(AMovedItems.begin(), AMovedItems.end(), APlacement.ItemIndex) != AMovedItems.end(); });
                    if (ReusesMovedItem)
                        continue;
                    if (_ApplyBoardCompositeCandidate(Candidate, ALayers, AStats)) {
                        Changed = true;
                        _HadBoardFillChanges = true;
                        AAcceptedMoves += Candidate.Placements.size();
                        for (const TetHoleFillCandidate &Placement : Candidate.Placements)
                            AMovedItems.push_back(Placement.ItemIndex);
                    } else
                        ARollbackBins.push_back(static_cast<std::size_t>(TargetBin));
                }
            }
            m_SearchDeadline = ADeadline;
            m_SearchBudgetReached = false;
            return Changed;
        }
        bool CetPolygonBoardRepairer::_RunLegacyBoardFillPass(std::size_t ALayers, std::size_t &AAcceptedMoves)
        {
            for (int TargetBin = 0; TargetBin < static_cast<int>(ALayers) && _CanContinueSearch(); ++TargetBin) {
                std::vector<TetClusterFreeRegion> FreeRegions;
                _ExtractBoardFreeRegions(TargetBin, FreeRegions);
                TetBoardLocalFillCandidate LocalCandidate;
                if (_FindBestLocalCandidateForTargetBin(TargetBin, FreeRegions, LocalCandidate) && _ApplyLocalFillCandidate(LocalCandidate)) {
                    _HadBoardFillChanges = true;
                    AAcceptedMoves += LocalCandidate.Placements.size();
                    std::cout << "[BOARD LOCAL FILL][ACCEPT] Bin=" << TargetBin << " Parts=" << LocalCandidate.Placements.size() << " AreaGain=" << LocalCandidate.OccupiedAreaGain << " EnvelopeFill=" << LocalCandidate.EnvelopeFillRatio << std::endl;
                    return true;
                }
                TetHoleFillCandidate Candidate;
                if (_FindBestCandidateForTargetBin(TargetBin, FreeRegions, Candidate) && _ApplyHoleFillCandidate(Candidate)) {
                    _HadBoardFillChanges = true;
                    ++AAcceptedMoves;
                    std::cout << "[BOARD FILL][MOVE] Item=" << Candidate.ItemIndex << " From=" << Candidate.OldBin << " To=" << Candidate.TargetBin << " Score=" << Candidate.Score << std::endl;
                    return true;
                }
            }
            return false;
        }
        bool CetPolygonBoardRepairer::_ExtractBoardFreeRegions(int ATargetBin, std::vector<TetClusterFreeRegion> &AOutRegions) const
        {
            AOutRegions.clear();
            if (_Items == nullptr || _BinPoly == nullptr || ATargetBin < 0 || _BinPoly->Contour.size() < 3)
                return false;
            ClipperLib::Paths ReservedContours;
            if (!_BuildBoardReservedContours(ATargetBin, ReservedContours))
                return false;
            ClipperLib::Clipper DifferenceClipper;
            if (!DifferenceClipper.AddPath(_BinPoly->Contour, ClipperLib::ptSubject, true))
                return false;
            if (!_BinPoly->Holes.empty() && !DifferenceClipper.AddPaths(_BinPoly->Holes, ClipperLib::ptSubject, true))
                return false;
            if (!ReservedContours.empty() && !DifferenceClipper.AddPaths(ReservedContours, ClipperLib::ptClip, true))
                return false;
            ClipperLib::PolyTree Tree;
            if (!DifferenceClipper.Execute(ClipperLib::ctDifference, Tree, ClipperLib::pftNonZero, ClipperLib::pftNonZero))
                return false;
            for (const ClipperLib::PolyNode *Node : Tree.Childs)
                if (Node != nullptr && !_AppendBoardFreeRegion(*Node, AOutRegions))
                    return false;
            std::stable_sort(AOutRegions.begin(), AOutRegions.end(), [](const TetClusterFreeRegion &AFirst, const TetClusterFreeRegion &ASecond) {
                if (std::abs(AFirst.Area - ASecond.Area) > CET_CLUSTER_GEOMETRY_AREA_TOLERANCE)
                    return AFirst.Area > ASecond.Area;
                if (AFirst.MinY != ASecond.MinY)
                    return AFirst.MinY < ASecond.MinY;
                return AFirst.MinX < ASecond.MinX;
            });
            if (AOutRegions.size() > CET_BOARD_FILL_MAX_FREE_REGIONS)
                AOutRegions.resize(CET_BOARD_FILL_MAX_FREE_REGIONS);
            return !AOutRegions.empty();
        }
        bool CetPolygonBoardRepairer::_BuildBoardReservedContours(int ATargetBin, ClipperLib::Paths &AOutContours) const
        {
            AOutContours.clear();
            if (_Items == nullptr || ATargetBin < 0)
                return false;
            for (const CetNestItem &SourceItem : *_Items) {
                if (SourceItem.binId() != ATargetBin)
                    continue;
                CetNestItem Item = SourceItem;
                Item.inflation(0);
                const CetPolygonImpl &Shape = Item.transformedShape();
                CetPath Contour = Shape.Contour;
                ClipperLib::CleanPolygon(Contour, 1.0);
                if (Contour.size() < 3 || std::abs(ClipperLib::Area(Contour)) <= 0.0)
                    return false;
                if (!ClipperLib::Orientation(Contour))
                    std::reverse(Contour.begin(), Contour.end());
                AOutContours.push_back(std::move(Contour));
                for (CetPath Hole : Shape.Holes) {
                    ClipperLib::CleanPolygon(Hole, 1.0);
                    if (Hole.size() < 3 || std::abs(ClipperLib::Area(Hole)) <= 0.0)
                        return false;
                    if (ClipperLib::Orientation(Hole))
                        std::reverse(Hole.begin(), Hole.end());
                    AOutContours.push_back(std::move(Hole));
                }
            }
            if (AOutContours.empty() || m_SpacingCoord <= 0)
                return true;
            ClipperLib::Paths OffsetContours;
            ClipperLib::ClipperOffset OffsetBuilder(2.0, std::max(1.0, static_cast<double>(m_SpacingCoord) * 0.02));
            OffsetBuilder.AddPaths(AOutContours, ClipperLib::jtRound, ClipperLib::etClosedPolygon);
            OffsetBuilder.Execute(OffsetContours, m_SpacingCoord);
            if (OffsetContours.empty())
                return false;
            AOutContours = std::move(OffsetContours);
            return true;
        }
        bool CetPolygonBoardRepairer::_AppendBoardFreeRegion(const ClipperLib::PolyNode &ANode, std::vector<TetClusterFreeRegion> &AOutRegions) const
        {
            if (!ANode.IsHole() && ANode.Contour.size() >= 3) {
                TetClusterFreeRegion Region;
                Region.Contour = ANode.Contour;
                Region.IsClosed = true;
                Region.Area = std::abs(static_cast<double>(ClipperLib::Area(Region.Contour)));
                if (!std::isfinite(Region.Area) || Region.Area <= 0.0)
                    return false;
                Region.MinX = Region.MaxX = static_cast<double>(Region.Contour.front().X);
                Region.MinY = Region.MaxY = static_cast<double>(Region.Contour.front().Y);
                for (const ClipperLib::IntPoint &Point : Region.Contour) {
                    Region.MinX = std::min(Region.MinX, static_cast<double>(Point.X));
                    Region.MinY = std::min(Region.MinY, static_cast<double>(Point.Y));
                    Region.MaxX = std::max(Region.MaxX, static_cast<double>(Point.X));
                    Region.MaxY = std::max(Region.MaxY, static_cast<double>(Point.Y));
                }
                for (const ClipperLib::PolyNode *Child : ANode.Childs) {
                    if (Child == nullptr || !Child->IsHole())
                        continue;
                    const double HoleArea = std::abs(static_cast<double>(ClipperLib::Area(Child->Contour)));
                    if (!std::isfinite(HoleArea) || HoleArea >= Region.Area)
                        return false;
                    Region.Area -= HoleArea;
                    Region.Holes.push_back(Child->Contour);
                }
                Region.Width = Region.MaxX - Region.MinX;
                Region.Height = Region.MaxY - Region.MinY;
                if (Region.Area <= 0.0 || Region.Width <= 0.0 || Region.Height <= 0.0)
                    return false;
                AOutRegions.push_back(std::move(Region));
            }
            for (const ClipperLib::PolyNode *Child : ANode.Childs)
                if (Child != nullptr && !_AppendBoardFreeRegion(*Child, AOutRegions))
                    return false;
            return true;
        }
        bool CetPolygonBoardRepairer::_BuildCompositeSkeleton(int ASkeletonIndex, const TetShapeFeature &AFeature, TetClusterCandidate &AOutCluster) const
        {
            AOutCluster = TetClusterCandidate{};
            if (_Items == nullptr || _Options == nullptr || ASkeletonIndex < 0 || ASkeletonIndex >= static_cast<int>(_Items->size()))
                return false;
            const bool IsSupported = AFeature.ShapeType == MetShapeType::CircleLike || AFeature.ShapeType == MetShapeType::EllipseLike || AFeature.ShapeType == MetShapeType::TriangleLike || AFeature.ShapeType == MetShapeType::RectangleLike || AFeature.ShapeType == MetShapeType::ArcLike || AFeature.ShapeType == MetShapeType::QuadrilateralLike || AFeature.ShapeType == MetShapeType::ConvexPolygon || AFeature.ShapeType == MetShapeType::ConcavePolygon;
            if (!IsSupported)
                return false;
            CetClusterGeometryHelper Geometry;
            const CetPath Contour = Geometry.GetIdentityContour((*_Items)[ASkeletonIndex]);
            double MinX = 0.0, MinY = 0.0, MaxX = 0.0, MaxY = 0.0;
            if (!Geometry.GetBounds(Contour, MinX, MinY, MaxX, MaxY))
                return false;
            AOutCluster.OriginalIndices = {ASkeletonIndex};
            AOutCluster.Transforms = {{ASkeletonIndex, 0.0, 0.0, 0.0}};
            AOutCluster.BuilderName = "BoardCompositeSkeleton";
            AOutCluster.ClusterType = "BoardComposite";
            return Geometry.FinalizeCandidateInRectangle(*_Items, *_Options, AOutCluster, MaxX - MinX, MaxY - MinY);
        }
        bool CetPolygonBoardRepairer::_BuildAnchoredCompositeSkeleton(int ASkeletonIndex, const TetShapeFeature &AFeature, TetClusterCandidate &AOutCluster) const
        {
            AOutCluster = TetClusterCandidate{};
            if (_Items == nullptr || _Options == nullptr || ASkeletonIndex < 0 || ASkeletonIndex >= static_cast<int>(_Items->size()))
                return false;
            const bool IsSupported = AFeature.ShapeType == MetShapeType::CircleLike || AFeature.ShapeType == MetShapeType::EllipseLike || AFeature.ShapeType == MetShapeType::TriangleLike || AFeature.ShapeType == MetShapeType::RectangleLike || AFeature.ShapeType == MetShapeType::ArcLike || AFeature.ShapeType == MetShapeType::QuadrilateralLike || AFeature.ShapeType == MetShapeType::ConvexPolygon || AFeature.ShapeType == MetShapeType::ConcavePolygon;
            if (!IsSupported)
                return false;
            const CetNestItem &Item = (*_Items)[ASkeletonIndex];
            const auto Bounds = Item.boundingBox();
            const double Width = static_cast<double>(Bounds.width());
            const double Height = static_cast<double>(Bounds.height());
            if (Width <= 0.0 || Height <= 0.0)
                return false;
            AOutCluster = TetClusterCandidate{};
            AOutCluster.OriginalIndices = {ASkeletonIndex};
            AOutCluster.Transforms = {{ASkeletonIndex, static_cast<double>(Item.translation().X), static_cast<double>(Item.translation().Y), static_cast<double>(Item.rotation())}};
            AOutCluster.BuilderName = "BoardCompositeAnchoredSkeleton";
            AOutCluster.ClusterType = "BoardCompositeAnchored";
            CetClusterGeometryHelper Geometry;
            return Geometry.FinalizeCandidateInRectangle(*_Items, *_Options, AOutCluster, Width, Height);
        }
        std::vector<int> CetPolygonBoardRepairer::_CollectCompositeFillers(int ATargetBin, const TetClusterCandidate &ACluster, const TetClusterFreeRegion &AFreeRegion, const std::vector<TetShapeFeature> &AFeatures, bool AAllowTargetBin) const
        {
            if (_Items == nullptr)
                return {};
            const bool Large = _Items->size() > CET_NEST_FULL_STRATEGY_ITEM_LIMIT;
            const std::size_t Limit = Large ? CET_BOARD_COMPOSITE_LARGE_ORDER_MAX_FILLERS_PER_STATE : CET_BOARD_COMPOSITE_MAX_FILLERS_PER_STATE;
            std::vector<int> Result;
            for (std::size_t Index = 0; Index < _Items->size(); ++Index) {
                const int SourceBin = static_cast<int>((*_Items)[Index].binId());
                if (SourceBin < 0 || SourceBin < ATargetBin || (!AAllowTargetBin && SourceBin == ATargetBin))
                    continue;
                if (std::find(ACluster.OriginalIndices.begin(), ACluster.OriginalIndices.end(), static_cast<int>(Index)) != ACluster.OriginalIndices.end())
                    continue;
                if (Index >= AFeatures.size() || AFeatures[Index].BoxArea > AFreeRegion.Area)
                    continue;
                const bool FitsEnvelope = (AFeatures[Index].Width <= ACluster.ClusterWidth && AFeatures[Index].Height <= ACluster.ClusterHeight) || (AFeatures[Index].Height <= ACluster.ClusterWidth && AFeatures[Index].Width <= ACluster.ClusterHeight);
                if (!FitsEnvelope)
                    continue;
                Result.push_back(static_cast<int>(Index));
            }
            std::stable_sort(Result.begin(), Result.end(), [&](int A, int B) {
                const double FreeArea = std::max(1.0, ACluster.ProxyArea - ACluster.ReservedArea);
                const double TargetArea = FreeArea / static_cast<double>(std::max<std::size_t>(1, Limit));
                auto Score = [&](int Index) {
                    const TetShapeFeature &Feature = AFeatures[Index];
                    const double AreaMatch = std::min(Feature.Area, TargetArea) / std::max(Feature.Area, TargetArea);
                    const double FillerShort = std::min(Feature.Width, Feature.Height);
                    const double FillerLong = std::max(Feature.Width, Feature.Height);
                    const double EnvelopeShort = std::max(1.0, std::min(ACluster.ClusterWidth, ACluster.ClusterHeight));
                    const double EnvelopeLong = std::max(1.0, std::max(ACluster.ClusterWidth, ACluster.ClusterHeight));
                    const double ShortSideFit = 1.0 - std::clamp(FillerShort / EnvelopeShort, 0.0, 1.0);
                    const double LongSideFit = 1.0 - std::clamp(FillerLong / EnvelopeLong, 0.0, 1.0);
                    return ShortSideFit * 3.0 + LongSideFit + AreaMatch * 0.5;
                };
                const double ScoreA = Score(A), ScoreB = Score(B);
                if (std::abs(ScoreA - ScoreB) > 1e-9)
                    return ScoreA > ScoreB;
                if (std::abs(AFeatures[A].Area - AFeatures[B].Area) > CET_BOARD_COMPOSITE_SCORE_COMPARISON_TOLERANCE)
                    return AFeatures[A].Area < AFeatures[B].Area;
                return A < B;
            });
            if (Result.size() > Limit)
                Result.resize(Limit);
            return Result;
        }
        bool CetPolygonBoardRepairer::_ExpandCompositeBeam(const TetClusterCandidate &ASkeleton, const std::vector<int> &AFillers, const std::vector<TetShapeFeature> &AFeatures, TetClusterCandidate &AOutCluster) const
        {
            if (_Items == nullptr || _Options == nullptr || AFillers.empty())
                return false;
            CetRectangleFillClusterBuilder Builder;
            CetClusterGeometryHelper Geometry;
            std::vector<TetClusterCandidate> Beam{ASkeleton};
            std::vector<TetClusterCandidate> Best;
            const bool Large = _Items->size() > CET_NEST_FULL_STRATEGY_ITEM_LIMIT;
            const std::size_t DepthLimit = Large ? CET_BOARD_COMPOSITE_LARGE_ORDER_MAX_DEPTH : CET_BOARD_COMPOSITE_MAX_DEPTH;
            const std::size_t BeamLimit = Large ? CET_BOARD_COMPOSITE_LARGE_ORDER_BEAM_WIDTH : CET_BOARD_COMPOSITE_BEAM_WIDTH;
            for (std::size_t Depth = 0; Depth < DepthLimit && !Beam.empty(); ++Depth) {
                std::vector<TetClusterCandidate> Next;
                for (const TetClusterCandidate &State : Beam)
                    for (int Filler : AFillers) {
                        std::vector<TetClusterFreeRegion> InnerRegions;
                        TetClusterCandidate Candidate;
                        if (Geometry.ExtractCandidateFreeRegions(*_Items, *_Options, State, InnerRegions) && Builder.TryAppendFillerInRectangleEnvelope({*_Items, AFeatures, *_Options, ASkeleton, ASkeleton, State, &InnerRegions, Filler, ASkeleton.ClusterWidth, ASkeleton.ClusterHeight}, Candidate))
                            Next.push_back(std::move(Candidate));
                    }
                if (Next.empty())
                    break;
                std::stable_sort(Next.begin(), Next.end(), [](const TetClusterCandidate &A, const TetClusterCandidate &B) { return A.FillRatio > B.FillRatio; });
                if (Next.size() > BeamLimit)
                    Next.resize(BeamLimit);
                Best.insert(Best.end(), Next.begin(), Next.end());
                Beam = std::move(Next);
            }
            if (Best.empty())
                return false;
            std::stable_sort(Best.begin(), Best.end(), [](const TetClusterCandidate &A, const TetClusterCandidate &B) { return A.FillRatio > B.FillRatio; });
            AOutCluster = Best.front();
            return AOutCluster.OriginalIndices.size() >= 2;
        }
        bool CetPolygonBoardRepairer::_BuildBoardCompositeForSkeleton(const TetBoardCompositeBuildRequest &ARequest)
        {
            const int ATargetBin = ARequest.TargetBin; const TetClusterFreeRegion &AFreeRegion = *ARequest.FreeRegion; const int ASkeletonIndex = ARequest.SkeletonIndex; const auto &AFeatures = ARequest.Features; long long &AInOutExactChecks = ARequest.ExactChecks; TetBoardCompositeCandidate &AOutCandidate = ARequest.OutCandidate; TetBoardCompositeSearchStats &AInOutStats = ARequest.Stats;
            AOutCandidate = TetBoardCompositeCandidate{};
            if (_Items == nullptr || _Options == nullptr || ASkeletonIndex < 0 || ASkeletonIndex >= static_cast<int>(AFeatures.size()))
                return false;
            TetClusterCandidate Skeleton;
            if (!_BuildCompositeSkeleton(ASkeletonIndex, AFeatures[ASkeletonIndex], Skeleton)) {
                ++AInOutStats.SkeletonBuildFailures;
                return false;
            }
            const std::vector<int> Fillers = _CollectCompositeFillers(ATargetBin, Skeleton, AFreeRegion, AFeatures, false);
            if (Fillers.empty()) {
                ++AInOutStats.EmptyFillerSets;
                return false;
            }
            AOutCandidate.TargetBin = ATargetBin;
            AOutCandidate.FreeRegion = AFreeRegion;
            AOutCandidate.SkeletonIndex = ASkeletonIndex;
            if (!_ExpandCompositeBeam(Skeleton, Fillers, AFeatures, AOutCandidate.Cluster)) {
                ++AInOutStats.BeamExpansionFailures;
                return false;
            }
            ++AInOutStats.CandidateCount;
            if (_PlaceBoardCompositeInFreeRegion(AOutCandidate, AInOutExactChecks, AInOutStats))
                return true;
            ++AInOutStats.PlacementFailures;
            return false;
        }
        bool CetPolygonBoardRepairer::_BuildAnchoredBoardComposite(const TetBoardCompositeBuildRequest &ARequest)
        {
            const int ATargetBin = ARequest.TargetBin; const TetClusterFreeRegion &AFreeRegion = *ARequest.FreeRegion; const int ASkeletonIndex = ARequest.SkeletonIndex; const auto &AFeatures = ARequest.Features; long long &AInOutExactChecks = ARequest.ExactChecks; TetBoardCompositeCandidate &AOutCandidate = ARequest.OutCandidate; TetBoardCompositeSearchStats &AInOutStats = ARequest.Stats;
            AOutCandidate = TetBoardCompositeCandidate{};
            if (_Items == nullptr || ASkeletonIndex < 0 || ASkeletonIndex >= static_cast<int>(AFeatures.size()))
                return false;
            TetClusterCandidate Skeleton;
            if (!_BuildAnchoredCompositeSkeleton(ASkeletonIndex, AFeatures[ASkeletonIndex], Skeleton)) {
                ++AInOutStats.SkeletonBuildFailures;
                return false;
            }
            const std::vector<int> Fillers = _CollectCompositeFillers(ATargetBin, Skeleton, AFreeRegion, AFeatures, true);
            if (Fillers.empty()) {
                ++AInOutStats.EmptyFillerSets;
                return false;
            }
            for (int Filler : Fillers) {
                TetBoardCompositeCandidate Candidate;
                Candidate.FreeRegion = AFreeRegion;
                if (_BuildAnchoredCandidateForFiller({ATargetBin, &AFreeRegion, ASkeletonIndex, Filler, &Skeleton, AFeatures, AInOutExactChecks, Candidate, AInOutStats})) {
                    AOutCandidate = std::move(Candidate);
                    return true;
                }
                if (!_CanContinueSearch())
                    break;
            }
            ++AInOutStats.BeamExpansionFailures;
            return false;
        }
        bool CetPolygonBoardRepairer::_BuildAnchoredCandidateForFiller(const TetBoardCompositeBuildRequest &ARequest)
        {
            const int ATargetBin = ARequest.TargetBin; const int ASkeletonIndex = ARequest.SkeletonIndex; const int AFillerIndex = ARequest.FillerIndex; const TetClusterCandidate &ASkeleton = *ARequest.Skeleton; const auto &AFeatures = ARequest.Features; long long &AInOutExactChecks = ARequest.ExactChecks; TetBoardCompositeCandidate &AOutCandidate = ARequest.OutCandidate; TetBoardCompositeSearchStats &AInOutStats = ARequest.Stats;
            if (_Items == nullptr || _Options == nullptr || AFillerIndex < 0 || AFillerIndex >= static_cast<int>(_Items->size()))
                return false;
            const int OldBin = static_cast<int>((*_Items)[AFillerIndex].binId());
            if (OldBin == ATargetBin)
                (*_Items)[AFillerIndex].binId(-1);
            std::vector<TetClusterFreeRegion> BoardRegions;
            const bool Extracted = _ExtractBoardFreeRegions(ATargetBin, BoardRegions);
            if (OldBin == ATargetBin)
                (*_Items)[AFillerIndex].binId(OldBin);
            if (!Extracted || BoardRegions.empty())
                return false;
            const auto Bounds = (*_Items)[ASkeletonIndex].boundingBox();
            _TranslateFreeRegions(BoardRegions, -static_cast<double>(getX(Bounds.minCorner())), -static_cast<double>(getY(Bounds.minCorner())));
            CetClusterGeometryHelper Geometry;
            std::vector<TetClusterFreeRegion> EnvelopeRegions;
            if (!Geometry.IntersectFreeRegionsWithRectangle(BoardRegions, ASkeleton.ClusterWidth, ASkeleton.ClusterHeight, EnvelopeRegions))
                return false;
            _TranslateFreeRegions(EnvelopeRegions, static_cast<double>(getX(Bounds.minCorner())), static_cast<double>(getY(Bounds.minCorner())));
            const bool Large = _Items->size() > CET_NEST_FULL_STRATEGY_ITEM_LIMIT;
            const long long GridLimit = Large ? CET_BOARD_COMPOSITE_LARGE_ORDER_MAX_GRID_PLACEMENT_CHECKS_PER_FILLER : CET_BOARD_COMPOSITE_MAX_GRID_PLACEMENT_CHECKS_PER_FILLER;
            TetHoleFillCandidate GridPlacement;
            const bool GridFound = _TryFindBestPlacementInBin(static_cast<std::size_t>(AFillerIndex), ATargetBin, EnvelopeRegions, GridPlacement, GridLimit);
            if (GridFound) {
                TetClusterCandidate Cluster = ASkeleton;
                Cluster.OriginalIndices.push_back(AFillerIndex);
                Cluster.Transforms.push_back({AFillerIndex, static_cast<double>(GridPlacement.Translation.X - getX(Bounds.minCorner())), static_cast<double>(GridPlacement.Translation.Y - getY(Bounds.minCorner())), static_cast<double>(GridPlacement.Rotation)});
                if (Geometry.FinalizeCandidateInRectangle(*_Items, *_Options, Cluster, ASkeleton.ClusterWidth, ASkeleton.ClusterHeight)) {
                    AOutCandidate.AnchoredSkeleton = true;
                    AOutCandidate.SkeletonIndex = ASkeletonIndex;
                    AOutCandidate.TargetBin = ATargetBin;
                    AOutCandidate.Cluster = std::move(Cluster);
                    ++AInOutStats.CandidateCount;
                    if (_PlaceAnchoredBoardComposite(AOutCandidate, AInOutExactChecks, AInOutStats))
                        return true;
                    ++AInOutStats.PlacementFailures;
                }
            }
            _TranslateFreeRegions(EnvelopeRegions, -static_cast<double>(getX(Bounds.minCorner())), -static_cast<double>(getY(Bounds.minCorner())));
            CetRectangleFillClusterBuilder Builder;
            TetClusterCandidate Cluster;
            if (!Builder.TryAppendFillerInRectangleEnvelope({*_Items, AFeatures, *_Options, ASkeleton, ASkeleton, ASkeleton, &EnvelopeRegions, AFillerIndex, ASkeleton.ClusterWidth, ASkeleton.ClusterHeight}, Cluster)) {
                std::cout << "[BOARD COMPOSITE][CANDIDATE] Bin=" << ATargetBin << " Skeleton=" << ASkeletonIndex << " Filler=" << AFillerIndex << " Stage=" << (GridFound ? "GridFinalizeMiss" : "GridAndEnvelopeMiss") << " Regions=" << EnvelopeRegions.size() << std::endl;
                return false;
            }
            AOutCandidate.AnchoredSkeleton = true;
            AOutCandidate.SkeletonIndex = ASkeletonIndex;
            AOutCandidate.TargetBin = ATargetBin;
            AOutCandidate.Cluster = std::move(Cluster);
            ++AInOutStats.CandidateCount;
            if (_PlaceAnchoredBoardComposite(AOutCandidate, AInOutExactChecks, AInOutStats))
                return true;
            ++AInOutStats.PlacementFailures;
            return false;
        }
        void CetPolygonBoardRepairer::_TranslateFreeRegions(std::vector<TetClusterFreeRegion> &ARegions, double AOffsetX, double AOffsetY) const
        {
            const auto DX = static_cast<ClipperLib::cInt>(std::llround(AOffsetX));
            const auto DY = static_cast<ClipperLib::cInt>(std::llround(AOffsetY));
            for (TetClusterFreeRegion &Region : ARegions) {
                for (ClipperLib::IntPoint &Point : Region.Contour) {
                    Point.X += DX;
                    Point.Y += DY;
                }
                for (CetPath &Hole : Region.Holes)
                    for (ClipperLib::IntPoint &Point : Hole) {
                        Point.X += DX;
                        Point.Y += DY;
                    }
                Region.MinX += AOffsetX;
                Region.MaxX += AOffsetX;
                Region.MinY += AOffsetY;
                Region.MaxY += AOffsetY;
            }
        }
        bool CetPolygonBoardRepairer::_BuildCompositePlacements(const TetBoardCompositeCandidate &ACandidate, double ARotation, double ATranslationX, double ATranslationY, std::vector<TetHoleFillCandidate> &AOutPlacements) const
        {
            AOutPlacements.clear();
            const double Cosine = std::cos(ARotation), Sine = std::sin(ARotation);
            for (const TetItemTransform &Transform : ACandidate.Cluster.Transforms) {
                if (Transform.OriginalId < 0)
                    return false;
                const double X = ATranslationX + Transform.RelativeX * Cosine - Transform.RelativeY * Sine;
                const double Y = ATranslationY + Transform.RelativeX * Sine + Transform.RelativeY * Cosine;
                AOutPlacements.push_back({true, static_cast<std::size_t>(Transform.OriginalId), -1, ACandidate.TargetBin, Point(static_cast<ClipperLib::cInt>(std::llround(X)), static_cast<ClipperLib::cInt>(std::llround(Y))), Radians(ARotation + Transform.RelativeRotation), 0.0});
            }
            return !AOutPlacements.empty();
        }
        bool CetPolygonBoardRepairer::_ValidateBoardCompositePlacements(const TetBoardCompositeCandidate &ACandidate, const std::vector<TetHoleFillCandidate> &APlacements) const
        {
            const std::size_t ExpectedCount = ACandidate.Cluster.Transforms.size() - (ACandidate.AnchoredSkeleton ? 1 : 0);
            if (_Items == nullptr || APlacements.size() != ExpectedCount)
                return false;
            CetTNestItemVector Snapshot = *_Items;
            auto *Mutable = const_cast<CetPolygonBoardRepairer *>(this);
            for (const TetHoleFillCandidate &Placement : APlacements) {
                if (Placement.ItemIndex >= _Items->size()) {
                    *_Items = Snapshot;
                    return false;
                }
                const int OldBin = static_cast<int>((*_Items)[Placement.ItemIndex].binId());
                const bool InvalidSource = OldBin < ACandidate.TargetBin || (!ACandidate.AnchoredSkeleton && OldBin == ACandidate.TargetBin);
                if (!Placement.Valid || InvalidSource) {
                    *_Items = Snapshot;
                    return false;
                }
                (*_Items)[Placement.ItemIndex].binId(-1);
            }
            std::vector<TetClusterFreeRegion> CurrentFreeRegions;
            if (!Mutable->_ExtractBoardFreeRegions(ACandidate.TargetBin, CurrentFreeRegions)) {
                *_Items = Snapshot;
                return false;
            }
            for (const TetHoleFillCandidate &Placement : APlacements) {
                CetNestItem &Item = (*_Items)[Placement.ItemIndex];
                Item.translation(Placement.Translation);
                Item.rotation(Placement.Rotation);
                Item.binId(Placement.TargetBin);
                TetPlacementCandidate Check;
                Check.ItemIndex = Placement.ItemIndex;
                Check.TargetBin = Placement.TargetBin;
                Check.Translation = Placement.Translation;
                Check.Rotation = Placement.Rotation;
                const bool InsideFreeRegion = std::any_of(CurrentFreeRegions.begin(), CurrentFreeRegions.end(), [&](const TetClusterFreeRegion &ARegion) { return Mutable->_IsPlacementInsideFreeRegion(Check, ARegion); });
                if (!InsideFreeRegion) {
                    std::cout << "[BOARD COMPOSITE][ROLLBACK] Bin=" << ACandidate.TargetBin << " Item=" << Placement.ItemIndex << " Reason=outside-recomputed-free-region" << std::endl;
                    *_Items = Snapshot;
                    return false;
                }
                if (!Mutable->_CanPlaceAt(Check)) {
                    std::cout << "[BOARD COMPOSITE][ROLLBACK] Bin=" << ACandidate.TargetBin << " Item=" << Placement.ItemIndex << " Reason=exact-collision-or-spacing" << std::endl;
                    *_Items = Snapshot;
                    return false;
                }
            }
            *_Items = Snapshot;
            return true;
        }
        bool CetPolygonBoardRepairer::_PlaceBoardCompositeInFreeRegion(TetBoardCompositeCandidate &AInOutCandidate, long long &AInOutExactChecks, TetBoardCompositeSearchStats &AInOutStats)
        {
            if (_Items == nullptr || _Options == nullptr)
                return false;
            const bool Large = _Items->size() > CET_NEST_FULL_STRATEGY_ITEM_LIMIT;
            const long long Limit = Large ? CET_BOARD_COMPOSITE_LARGE_ORDER_MAX_EXACT_PLACEMENT_CHECKS : CET_BOARD_COMPOSITE_MAX_EXACT_PLACEMENT_CHECKS;
            CetClusterGeometryHelper Geometry;
            const std::vector<double> Rotations = CetRotationUtils::BuildAllowedRotations(_Options->Rotations);
            long long Checks = 0;
            for (double Rotation : Rotations) {
                const CetPath Proxy = Geometry.TransformContour(AInOutCandidate.Cluster.ProxyContour, Rotation, 0.0, 0.0);
                double MinX = 0.0, MinY = 0.0, MaxX = 0.0, MaxY = 0.0;
                if (!Geometry.GetBounds(Proxy, MinX, MinY, MaxX, MaxY))
                    continue;
                const std::array<std::pair<double, double>, 4> Anchors{{{AInOutCandidate.FreeRegion.MinX - MinX, AInOutCandidate.FreeRegion.MinY - MinY}, {AInOutCandidate.FreeRegion.MaxX - MaxX, AInOutCandidate.FreeRegion.MinY - MinY}, {AInOutCandidate.FreeRegion.MinX - MinX, AInOutCandidate.FreeRegion.MaxY - MaxY}, {AInOutCandidate.FreeRegion.MaxX - MaxX, AInOutCandidate.FreeRegion.MaxY - MaxY}}};
                for (const auto &Anchor : Anchors) {
                    if (AInOutExactChecks >= Limit || !_CanContinueSearch())
                        return false;
                    ++Checks;
                    ++AInOutExactChecks;
                    std::vector<TetHoleFillCandidate> Placements;
                    if (!_BuildCompositePlacements(AInOutCandidate, Rotation, Anchor.first, Anchor.second, Placements))
                        continue;
                    if (!_ValidateBoardCompositePlacements(AInOutCandidate, Placements))
                        continue;
                    AInOutCandidate.Placements = std::move(Placements);
                    _ScoreBoardComposite(AInOutCandidate);
                    AInOutCandidate.Valid = true;
                    AInOutStats.ExactPlacementChecks += Checks;
                    return true;
                }
            }
            AInOutStats.ExactPlacementChecks += Checks;
            return false;
        }
        bool CetPolygonBoardRepairer::_PlaceAnchoredBoardComposite(TetBoardCompositeCandidate &AInOutCandidate, long long &AInOutExactChecks, TetBoardCompositeSearchStats &AInOutStats)
        {
            if (_Items == nullptr || AInOutCandidate.SkeletonIndex < 0 || AInOutCandidate.SkeletonIndex >= static_cast<int>(_Items->size()))
                return false;
            const bool Large = _Items->size() > CET_NEST_FULL_STRATEGY_ITEM_LIMIT;
            const long long Limit = Large ? CET_BOARD_COMPOSITE_LARGE_ORDER_MAX_EXACT_PLACEMENT_CHECKS : CET_BOARD_COMPOSITE_MAX_EXACT_PLACEMENT_CHECKS;
            if (AInOutExactChecks >= Limit)
                return false;
            const auto Bounds = (*_Items)[AInOutCandidate.SkeletonIndex].boundingBox();
            std::vector<TetHoleFillCandidate> Placements;
            if (!_BuildCompositePlacements(AInOutCandidate, 0.0, static_cast<double>(getX(Bounds.minCorner())), static_cast<double>(getY(Bounds.minCorner())), Placements))
                return false;
            Placements.erase(std::remove_if(Placements.begin(), Placements.end(), [&](const TetHoleFillCandidate &APlacement) { return static_cast<int>(APlacement.ItemIndex) == AInOutCandidate.SkeletonIndex; }), Placements.end());
            ++AInOutExactChecks;
            ++AInOutStats.ExactPlacementChecks;
            if (Placements.empty() || !_ValidateBoardCompositePlacements(AInOutCandidate, Placements))
                return false;
            AInOutCandidate.Placements = std::move(Placements);
            _ScoreBoardComposite(AInOutCandidate);
            AInOutCandidate.Valid = true;
            return true;
        }
        void CetPolygonBoardRepairer::_ScoreBoardComposite(TetBoardCompositeCandidate &AInOutCandidate) const
        {
            const TetClusterCandidate &Cluster = AInOutCandidate.Cluster;
            TetBoardCompositeScore &Score = AInOutCandidate.Score;
            Score.InternalFillRatio = Cluster.FillRatio;
            Score.OccupiedAreaGain = 0.0;
            if (_Items != nullptr)
                for (const TetHoleFillCandidate &Placement : AInOutCandidate.Placements) {
                    if (Placement.ItemIndex < _Items->size())
                        Score.OccupiedAreaGain += std::abs(static_cast<double>((*_Items)[Placement.ItemIndex].area()));
                }
            const double RegionAspect = AInOutCandidate.FreeRegion.Height > 0.0 ? AInOutCandidate.FreeRegion.Width / AInOutCandidate.FreeRegion.Height : 0.0;
            const double ClusterAspect = Cluster.ClusterHeight > 0.0 ? Cluster.ClusterWidth / Cluster.ClusterHeight : 0.0;
            Score.AspectMatch = RegionAspect > 0.0 && ClusterAspect > 0.0 ? std::min(RegionAspect, ClusterAspect) / std::max(RegionAspect, ClusterAspect) : 0.0;
            Score.RemainingShortSide = std::max(0.0, std::min(AInOutCandidate.FreeRegion.Width - Cluster.ClusterWidth, AInOutCandidate.FreeRegion.Height - Cluster.ClusterHeight));
            Score.Continuity = AInOutCandidate.FreeRegion.Area > 0.0 ? std::clamp(Cluster.ProxyArea / AInOutCandidate.FreeRegion.Area, 0.0, 1.0) : 0.0;
            Score.FragmentationPenalty = Cluster.FragmentationRisk;
            Score.ReusableRemnantValue = Score.RemainingShortSide * Score.Continuity;
            Score.SourceBinReduction = 0.0;
            if (_Items != nullptr)
                for (const TetHoleFillCandidate &Placement : AInOutCandidate.Placements) {
                    if (Placement.ItemIndex < _Items->size() && (*_Items)[Placement.ItemIndex].binId() > AInOutCandidate.TargetBin)
                        Score.SourceBinReduction += 1.0;
                }
            Score.Total = Score.OccupiedAreaGain + Score.InternalFillRatio * CET_BOARD_COMPOSITE_SCORE_FILL_WEIGHT + Score.AspectMatch * CET_BOARD_COMPOSITE_SCORE_ASPECT_WEIGHT + Score.Continuity * CET_BOARD_COMPOSITE_SCORE_CONTINUITY_WEIGHT + Score.ReusableRemnantValue * CET_BOARD_COMPOSITE_SCORE_REMNANT_WEIGHT + Score.SourceBinReduction * CET_BOARD_COMPOSITE_SCORE_SOURCE_BIN_WEIGHT - Score.FragmentationPenalty * CET_BOARD_COMPOSITE_SCORE_FRAGMENTATION_WEIGHT;
        }
        bool CetPolygonBoardRepairer::_IsBoardCompositeBetter(const TetBoardCompositeCandidate &AFirst, const TetBoardCompositeCandidate &ASecond) const
        {
            if (std::abs(AFirst.Score.Total - ASecond.Score.Total) > CET_BOARD_COMPOSITE_SCORE_COMPARISON_TOLERANCE)
                return AFirst.Score.Total > ASecond.Score.Total;
            if (std::abs(AFirst.Score.OccupiedAreaGain - ASecond.Score.OccupiedAreaGain) > CET_BOARD_COMPOSITE_SCORE_COMPARISON_TOLERANCE)
                return AFirst.Score.OccupiedAreaGain > ASecond.Score.OccupiedAreaGain;
            if (std::abs(AFirst.Score.FragmentationPenalty - ASecond.Score.FragmentationPenalty) > 1e-9)
                return AFirst.Score.FragmentationPenalty < ASecond.Score.FragmentationPenalty;
            return AFirst.Cluster.OriginalIndices < ASecond.Cluster.OriginalIndices;
        }
        bool CetPolygonBoardRepairer::_HasBoardCompositeGlobalGain(const CetTNestItemVector &ABeforeItems, std::size_t ABeforeLayers, const TetBoardCompositeCandidate &ACandidate, std::size_t AAfterLayers) const
        {
            if (_Items == nullptr || AAfterLayers > ABeforeLayers || ACandidate.TargetBin < 0)
                return false;
            double TargetBefore = 0.0;
            double TargetAfter = 0.0;
            double SourceBefore = 0.0;
            double SourceAfter = 0.0;
            for (const CetNestItem &Item : ABeforeItems) {
                const double Area = std::abs(static_cast<double>(Item.area()));
                if (Item.binId() == ACandidate.TargetBin)
                    TargetBefore += Area;
                if (Item.binId() > ACandidate.TargetBin)
                    SourceBefore += Area;
            }
            for (const CetNestItem &Item : *_Items) {
                const double Area = std::abs(static_cast<double>(Item.area()));
                if (Item.binId() == ACandidate.TargetBin)
                    TargetAfter += Area;
                if (Item.binId() > ACandidate.TargetBin)
                    SourceAfter += Area;
            }
            const bool TargetImproved = TargetAfter > TargetBefore + CET_BOARD_COMPOSITE_SCORE_COMPARISON_TOLERANCE;
            const bool LaterLoadReduced = SourceAfter < SourceBefore - CET_BOARD_COMPOSITE_SCORE_COMPARISON_TOLERANCE;
            if (TargetImproved && LaterLoadReduced && ACandidate.Score.FragmentationPenalty >= 0.0)
                return true;
            return ACandidate.AnchoredSkeleton && AAfterLayers == ABeforeLayers && _HasAnchoredRelocationGain(ABeforeItems, ACandidate);
        }
        bool CetPolygonBoardRepairer::_HasAnchoredRelocationGain(const CetTNestItemVector &ABeforeItems, const TetBoardCompositeCandidate &ACandidate) const
        {
            if (_Items == nullptr || ACandidate.TargetBin < 0 || ACandidate.SkeletonIndex < 0 || ACandidate.SkeletonIndex >= static_cast<int>(ABeforeItems.size()))
                return false;
            auto Measure = [&](const CetTNestItemVector &AItems) {
                double MaxX = 0.0, MaxY = 0.0;
                for (const CetNestItem &Item : AItems) {
                    if (Item.binId() != ACandidate.TargetBin)
                        continue;
                    const auto Bounds = Item.boundingBox();
                    MaxX = std::max(MaxX, static_cast<double>(getX(Bounds.maxCorner())));
                    MaxY = std::max(MaxY, static_cast<double>(getY(Bounds.maxCorner())));
                }
                return std::pair<double, double>{MaxX, MaxY};
            };
            const auto Before = Measure(ABeforeItems);
            const auto After = Measure(*_Items);
            const double Tolerance = CET_BOARD_COMPOSITE_SCORE_COMPARISON_TOLERANCE;
            const bool WidthImproved = After.first < Before.first - Tolerance && After.second <= Before.second + Tolerance;
            const bool DepthImproved = After.second < Before.second - Tolerance && After.first <= Before.first + Tolerance;
            if (WidthImproved || DepthImproved)
                return true;
            if (After.first > Before.first + Tolerance || After.second > Before.second + Tolerance)
                return false;
            const auto SkeletonBounds = ABeforeItems[ACandidate.SkeletonIndex].boundingBox();
            const double SkeletonBoxArea = std::max(1.0, static_cast<double>(SkeletonBounds.width()) * static_cast<double>(SkeletonBounds.height()));
            const double SkeletonFillRatio = std::abs(static_cast<double>(ABeforeItems[ACandidate.SkeletonIndex].area())) / SkeletonBoxArea;
            const bool FillImproved = ACandidate.Cluster.FillRatio > SkeletonFillRatio + CET_CLUSTER_ENVELOPE_FILL_MIN_FILL_RATIO_GAIN * 0.1;
            auto OverlapWithSkeleton = [&](const auto &ABounds) {
                const double Width = std::max(0.0, static_cast<double>(std::min(getX(SkeletonBounds.maxCorner()), getX(ABounds.maxCorner())) - std::max(getX(SkeletonBounds.minCorner()), getX(ABounds.minCorner()))));
                const double Height = std::max(0.0, static_cast<double>(std::min(getY(SkeletonBounds.maxCorner()), getY(ABounds.maxCorner())) - std::max(getY(SkeletonBounds.minCorner()), getY(ABounds.minCorner()))));
                return Width * Height;
            };
            auto GapToSkeleton = [&](const auto &ABounds) {
                const double GapX = std::max({0.0, static_cast<double>(getX(SkeletonBounds.minCorner()) - getX(ABounds.maxCorner())), static_cast<double>(getX(ABounds.minCorner()) - getX(SkeletonBounds.maxCorner()))});
                const double GapY = std::max({0.0, static_cast<double>(getY(SkeletonBounds.minCorner()) - getY(ABounds.maxCorner())), static_cast<double>(getY(ABounds.minCorner()) - getY(SkeletonBounds.maxCorner()))});
                return std::hypot(GapX, GapY);
            };
            bool Consolidated = false;
            for (const TetHoleFillCandidate &Placement : ACandidate.Placements) {
                if (Placement.ItemIndex >= ABeforeItems.size() || ABeforeItems[Placement.ItemIndex].binId() != ACandidate.TargetBin)
                    continue;
                const auto OldBounds = ABeforeItems[Placement.ItemIndex].boundingBox();
                const auto NewBounds = (*_Items)[Placement.ItemIndex].boundingBox();
                const bool MoreOverlap = OverlapWithSkeleton(NewBounds) > OverlapWithSkeleton(OldBounds) + Tolerance;
                const bool LessGap = GapToSkeleton(NewBounds) + Tolerance < GapToSkeleton(OldBounds);
                auto CenterDistance = [&](const auto &ABounds) {
                    const double DX = static_cast<double>(getX(ABounds.minCorner()) + getX(ABounds.maxCorner()) - getX(SkeletonBounds.minCorner()) - getX(SkeletonBounds.maxCorner())) * 0.5;
                    const double DY = static_cast<double>(getY(ABounds.minCorner()) + getY(ABounds.maxCorner()) - getY(SkeletonBounds.minCorner()) - getY(SkeletonBounds.maxCorner())) * 0.5;
                    return std::hypot(DX, DY);
                };
                const bool CloserCenter = CenterDistance(NewBounds) + Tolerance < CenterDistance(OldBounds);
                Consolidated = Consolidated || MoreOverlap || LessGap || CloserCenter;
            }
            return FillImproved && Consolidated;
        }
        bool CetPolygonBoardRepairer::_FindBestBoardCompositeForBin(int ATargetBin, const std::vector<TetClusterFreeRegion> &AFreeRegions, long long &AInOutExactChecks, TetBoardCompositeCandidate &AOutCandidate, TetBoardCompositeSearchStats &AInOutStats)
        {
            AOutCandidate = TetBoardCompositeCandidate{};
            if (_Items == nullptr || _Options == nullptr || AFreeRegions.empty())
                return false;
            const bool Large = _Items->size() > CET_NEST_FULL_STRATEGY_ITEM_LIMIT;
            const std::size_t RegionLimit = std::min(AFreeRegions.size(), CET_BOARD_COMPOSITE_MAX_FREE_REGIONS_PER_BIN);
            const std::size_t SkeletonLimit = Large ? CET_BOARD_COMPOSITE_LARGE_ORDER_MAX_SKELETONS_PER_REGION : CET_BOARD_COMPOSITE_MAX_SKELETONS_PER_REGION;
            const std::size_t CandidateLimit = Large ? CET_BOARD_COMPOSITE_LARGE_ORDER_MAX_CANDIDATES_PER_BIN : CET_BOARD_COMPOSITE_MAX_CANDIDATES_PER_BIN;
            const std::size_t AttemptLimit = Large ? CET_BOARD_COMPOSITE_LARGE_ORDER_MAX_SKELETON_ATTEMPTS_PER_BIN : CET_BOARD_COMPOSITE_MAX_SKELETON_ATTEMPTS_PER_BIN;
            if (ATargetBin < 0)
                return false;
            long long &ExactChecks = AInOutExactChecks;
            CetShapeAnalyzer Analyzer;
            const std::vector<TetShapeFeature> Features = Analyzer.AnalyzeALL(*_Items);
            CetClusterManager Manager;
            bool Found = false;
            const std::size_t CandidateStart = AInOutStats.CandidateCount;
            std::size_t Attempts = 0;
            for (std::size_t RegionIndex = 0; RegionIndex < RegionLimit && AInOutStats.CandidateCount - CandidateStart < CandidateLimit && Attempts < AttemptLimit; ++RegionIndex) {
                const TetClusterFreeRegion &Region = AFreeRegions[RegionIndex];
                const std::vector<int> AnchoredSkeletons = Manager.RankExistingBoardCompositeSkeletons(*_Items, Features, ATargetBin, Region, SkeletonLimit);
                AInOutStats.RankedAnchoredSkeletons += AnchoredSkeletons.size();
                for (int Skeleton : AnchoredSkeletons) {
                    ++Attempts;
                    TetBoardCompositeCandidate Candidate;
                    if (_BuildAnchoredBoardComposite({ATargetBin, &Region, Skeleton, -1, nullptr, Features, ExactChecks, Candidate, AInOutStats})) {
                        std::cout << "[BOARD COMPOSITE][CANDIDATE] Bin=" << ATargetBin << " Skeleton=" << Skeleton << " Mode=Anchored Parts=" << Candidate.Placements.size() << " Fill=" << Candidate.Score.InternalFillRatio << " Score=" << Candidate.Score.Total << std::endl;
                        if (!Found || _IsBoardCompositeBetter(Candidate, AOutCandidate)) {
                            AOutCandidate = std::move(Candidate);
                            Found = true;
                        }
                    }
                    if (Attempts >= AttemptLimit || AInOutStats.CandidateCount - CandidateStart >= CandidateLimit || ExactChecks >= (Large ? CET_BOARD_COMPOSITE_LARGE_ORDER_MAX_EXACT_PLACEMENT_CHECKS : CET_BOARD_COMPOSITE_MAX_EXACT_PLACEMENT_CHECKS) || !_CanContinueSearch())
                        break;
                }
                if (Attempts >= AttemptLimit || !_CanContinueSearch())
                    break;
                const std::vector<int> Skeletons = Manager.RankBoardCompositeSkeletons(*_Items, Features, ATargetBin, Region, SkeletonLimit);
                AInOutStats.RankedMovingSkeletons += Skeletons.size();
                for (int Skeleton : Skeletons) {
                    ++Attempts;
                    TetBoardCompositeCandidate Candidate;
                    if (_BuildBoardCompositeForSkeleton({ATargetBin, &Region, Skeleton, -1, nullptr, Features, ExactChecks, Candidate, AInOutStats})) {
                        std::cout << "[BOARD COMPOSITE][CANDIDATE] Bin=" << ATargetBin << " Skeleton=" << Skeleton << " Parts=" << Candidate.Placements.size() << " Fill=" << Candidate.Score.InternalFillRatio << " Score=" << Candidate.Score.Total << std::endl;
                        if (!Found || _IsBoardCompositeBetter(Candidate, AOutCandidate)) {
                            AOutCandidate = std::move(Candidate);
                            Found = true;
                        }
                    }
                    const long long ExactLimit = Large ? CET_BOARD_COMPOSITE_LARGE_ORDER_MAX_EXACT_PLACEMENT_CHECKS : CET_BOARD_COMPOSITE_MAX_EXACT_PLACEMENT_CHECKS;
                    if (Attempts >= AttemptLimit || AInOutStats.CandidateCount - CandidateStart >= CandidateLimit || ExactChecks >= ExactLimit || !_CanContinueSearch())
                        break;
                }
            }
            std::cout << "[BOARD COMPOSITE][SEARCH] Bin=" << ATargetBin << " Regions=" << RegionLimit << " Candidates=" << (AInOutStats.CandidateCount - CandidateStart) << " Attempts=" << Attempts << " ExactChecks=" << ExactChecks << " Found=" << Found << std::endl;
            return Found;
        }
        bool CetPolygonBoardRepairer::_ApplyBoardCompositeCandidate(const TetBoardCompositeCandidate &ACandidate, std::size_t &AInOutLayers, TetBoardCompositeSearchStats &AInOutStats)
        {
            const std::size_t MinimumPlacements = ACandidate.AnchoredSkeleton ? 1 : 2;
            if (_Items == nullptr || !ACandidate.Valid || ACandidate.Placements.size() < MinimumPlacements)
                return false;
            const CetTNestItemVector Snapshot = *_Items;
            const std::size_t BeforeLayers = AInOutLayers;
            const TetTNestEvalResult BeforeEval = Nest2DUtils->Nest2DStrategy->EvaluateNestResult(Snapshot, BeforeLayers);
            for (const TetHoleFillCandidate &Placement : ACandidate.Placements) {
                if (Placement.ItemIndex >= _Items->size()) {
                    *_Items = Snapshot;
                    return false;
                }
                const int OldBin = static_cast<int>((*_Items)[Placement.ItemIndex].binId());
                const bool InvalidSource = OldBin < ACandidate.TargetBin || (!ACandidate.AnchoredSkeleton && OldBin == ACandidate.TargetBin);
                if (InvalidSource) {
                    *_Items = Snapshot;
                    return false;
                }
                CetNestItem &Item = (*_Items)[Placement.ItemIndex];
                Item.translation(Placement.Translation);
                Item.rotation(Placement.Rotation);
                Item.binId(Placement.TargetBin);
            }
            for (const TetHoleFillCandidate &Placement : ACandidate.Placements)
                if (!_IsCurrentPlacementValid(Placement.ItemIndex)) {
                    *_Items = Snapshot;
                    ++AInOutStats.RollbackCount;
                    return false;
                }
            AInOutLayers = _CompactItemBins();
            const TetTNestEvalResult AfterEval = Nest2DUtils->Nest2DStrategy->EvaluateNestResult(*_Items, AInOutLayers);
            const bool StrategyGain = Nest2DUtils->Nest2DStrategy->IsBetterNestResult(AfterEval, BeforeEval);
            const bool BoardGain = _HasBoardCompositeGlobalGain(Snapshot, BeforeLayers, ACandidate, AInOutLayers);
            if (!StrategyGain && !BoardGain) {
                *_Items = Snapshot;
                AInOutLayers = BeforeLayers;
                ++AInOutStats.RollbackCount;
                std::cout << "[BOARD COMPOSITE][ROLLBACK] Bin=" << ACandidate.TargetBin << " Reason=no-global-or-board-gain" << std::endl;
                return false;
            }
            ++AInOutStats.AcceptedCount;
            AInOutStats.FillerCount += ACandidate.Placements.size() - (ACandidate.AnchoredSkeleton ? 0 : 1);
            std::cout << "[BOARD COMPOSITE][ACCEPT] Bin=" << ACandidate.TargetBin << " Parts=" << ACandidate.Placements.size() << " BeforeLayers=" << BeforeLayers << " AfterLayers=" << AInOutLayers << " StrategyGain=" << StrategyGain << " BoardGain=" << BoardGain << std::endl;
            return true;
        }
        std::vector<std::size_t> CetPolygonBoardRepairer::_CollectLocalFillCandidates(int ATargetBin, const TetClusterFreeRegion &AFreeRegion) const
        {
            std::vector<std::size_t> Result;
            if (_Items == nullptr || ATargetBin < 0 || AFreeRegion.Area <= 0.0)
                return Result;
            const double AreaTolerance = std::max(1.0, AFreeRegion.Area * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE);
            for (std::size_t Index = 0; Index < _Items->size(); ++Index) {
                const CetNestItem &Item = (*_Items)[Index];
                if (Item.binId() >= 0 && Item.binId() <= ATargetBin)
                    continue;
                if (std::find(m_LockedItemIndices.begin(), m_LockedItemIndices.end(), Index) != m_LockedItemIndices.end())
                    continue;
                const double Area = std::abs(static_cast<double>(Item.area()));
                if (std::isfinite(Area) && Area > 0.0 && Area <= AFreeRegion.Area + AreaTolerance)
                    Result.push_back(Index);
            }
            const std::size_t Limit = _Items->size() > CET_NEST_FULL_STRATEGY_ITEM_LIMIT ? CET_BOARD_LOCAL_FILL_LARGE_ORDER_MAX_CANDIDATE_ITEMS : CET_BOARD_LOCAL_FILL_MAX_CANDIDATE_ITEMS;
            if (Result.size() <= Limit)
                return Result;
            const double RegionAspect = AFreeRegion.Height > 0.0 ? AFreeRegion.Width / AFreeRegion.Height : 1.0;
            auto Area = [&](std::size_t Index) { return std::abs(static_cast<double>((*_Items)[Index].area())); };
            auto AspectDistance = [&](std::size_t Index) {
                const auto Bounds = (*_Items)[Index].boundingBox();
                const double Width = std::max(1.0, static_cast<double>(Bounds.width()));
                const double Height = std::max(1.0, static_cast<double>(Bounds.height()));
                return std::abs(std::log((Width / Height) / std::max(1e-9, RegionAspect)));
            };
            auto Select = [&](const auto &Compare) {
                const std::size_t Best = *std::min_element(Result.begin(), Result.end(), Compare);
                if (std::find(Result.begin(), Result.end(), Best) != Result.end())
                    return Best;
                return Result.front();
            };
            const double TargetArea = AFreeRegion.Area / static_cast<double>(Limit);
            const std::array<std::size_t, 4> Picks{Select([&](std::size_t A, std::size_t B) { return Area(A) > Area(B); }), Select([&](std::size_t A, std::size_t B) { return Area(A) < Area(B); }), Select([&](std::size_t A, std::size_t B) { return AspectDistance(A) < AspectDistance(B); }), Select([&](std::size_t A, std::size_t B) { return std::abs(Area(A) - TargetArea) < std::abs(Area(B) - TargetArea); })};
            std::vector<std::size_t> Selected;
            for (std::size_t Index : Picks)
                if (std::find(Selected.begin(), Selected.end(), Index) == Selected.end())
                    Selected.push_back(Index);
            std::stable_sort(Result.begin(), Result.end(), [&](std::size_t A, std::size_t B) { return Area(A) > Area(B); });
            for (std::size_t Index : Result)
                if (Selected.size() < Limit && std::find(Selected.begin(), Selected.end(), Index) == Selected.end())
                    Selected.push_back(Index);
            return Selected;
        }
        void CetPolygonBoardRepairer::_UpdateBoardLocalEnvelope(const std::vector<TetHoleFillCandidate> &APlacements, double &AOutArea, double &AOutFillRatio) const
        {
            AOutArea = 0.0;
            AOutFillRatio = 0.0;
            if (_Items == nullptr || APlacements.empty())
                return;
            double MinX = std::numeric_limits<double>::max(), MinY = MinX;
            double MaxX = -MinX, MaxY = -MinX, PartArea = 0.0;
            for (const TetHoleFillCandidate &Placement : APlacements) {
                if (Placement.ItemIndex >= _Items->size())
                    return;
                CetNestItem Item = (*_Items)[Placement.ItemIndex];
                Item.translation(Placement.Translation);
                Item.rotation(Placement.Rotation);
                Item.inflation(0);
                const auto Bounds = Item.boundingBox();
                MinX = std::min(MinX, static_cast<double>(getX(Bounds.minCorner())));
                MinY = std::min(MinY, static_cast<double>(getY(Bounds.minCorner())));
                MaxX = std::max(MaxX, static_cast<double>(getX(Bounds.maxCorner())));
                MaxY = std::max(MaxY, static_cast<double>(getY(Bounds.maxCorner())));
                PartArea += std::abs(static_cast<double>(Item.area()));
            }
            AOutArea = std::max(0.0, MaxX - MinX) * std::max(0.0, MaxY - MinY);
            AOutFillRatio = AOutArea > 0.0 ? std::clamp(PartArea / AOutArea, 0.0, 1.0) : 0.0;
        }
        bool CetPolygonBoardRepairer::_IsBoardLocalCandidateBetter(const TetBoardLocalFillCandidate &AFirst, const TetBoardLocalFillCandidate &ASecond) const
        {
            if (std::abs(AFirst.OccupiedAreaGain - ASecond.OccupiedAreaGain) > 1.0)
                return AFirst.OccupiedAreaGain > ASecond.OccupiedAreaGain;
            if (std::abs(AFirst.EnvelopeFillRatio - ASecond.EnvelopeFillRatio) > 1e-9)
                return AFirst.EnvelopeFillRatio > ASecond.EnvelopeFillRatio;
            if (AFirst.Placements.size() != ASecond.Placements.size())
                return AFirst.Placements.size() > ASecond.Placements.size();
            if (AFirst.FreeRegion.MinY != ASecond.FreeRegion.MinY)
                return AFirst.FreeRegion.MinY < ASecond.FreeRegion.MinY;
            return AFirst.FreeRegion.MinX < ASecond.FreeRegion.MinX;
        }
        bool CetPolygonBoardRepairer::_BuildLocalCandidateForFreeRegion(int ATargetBin, const TetClusterFreeRegion &AFreeRegion, TetBoardLocalFillCandidate &AOutCandidate)
        {
            AOutCandidate = TetBoardLocalFillCandidate{};
            if (_Items == nullptr || !_CanContinueSearch())
                return false;
            const std::vector<std::size_t> ItemIndices = _CollectLocalFillCandidates(ATargetBin, AFreeRegion);
            if (ItemIndices.size() < 2)
                return false;
            const long long RegionStartChecks = m_RemainingPlacementChecks;
            std::vector<TetBoardLocalFillCandidate> Beam(1);
            Beam.front().TargetBin = ATargetBin;
            Beam.front().FreeRegion = AFreeRegion;
            bool Found = false;
            for (std::size_t Depth = 0; Depth < CET_BOARD_LOCAL_FILL_MAX_DEPTH && !Beam.empty(); ++Depth) {
                std::vector<TetBoardLocalFillCandidate> NextBeam;
                for (const TetBoardLocalFillCandidate &State : Beam) {
                    for (std::size_t ItemIndex : ItemIndices) {
                        if (RegionStartChecks - m_RemainingPlacementChecks >= CET_BOARD_LOCAL_FILL_MAX_PLACEMENT_CHECKS_PER_REGION || !_CanContinueSearch())
                            break;
                        if (std::any_of(State.Placements.begin(), State.Placements.end(), [&](const TetHoleFillCandidate &P) { return P.ItemIndex == ItemIndex; }))
                            continue;
                        const CetTNestItemVector OriginalItems = *_Items;
                        for (const TetHoleFillCandidate &P : State.Placements) {
                            CetNestItem &Item = (*_Items)[P.ItemIndex];
                            Item.translation(P.Translation);
                            Item.rotation(P.Rotation);
                            Item.binId(P.TargetBin);
                        }
                        TetHoleFillCandidate Placement;
                        const bool Placed = _TryFindBestPlacementInBin(ItemIndex, ATargetBin, {AFreeRegion}, Placement, CET_BOARD_LOCAL_FILL_MAX_PLACEMENT_CHECKS_PER_ITEM);
                        *_Items = OriginalItems;
                        if (!Placed)
                            continue;
                        TetBoardLocalFillCandidate Next = State;
                        Next.Placements.push_back(Placement);
                        Next.OccupiedAreaGain += std::abs(static_cast<double>(OriginalItems[ItemIndex].area()));
                        _UpdateBoardLocalEnvelope(Next.Placements, Next.EnvelopeArea, Next.EnvelopeFillRatio);
                        Next.Score = Next.OccupiedAreaGain + Next.EnvelopeFillRatio;
                        Next.Valid = Next.Placements.size() >= 2;
                        NextBeam.push_back(Next);
                        if (Next.Valid && (!Found || _IsBoardLocalCandidateBetter(Next, AOutCandidate))) {
                            AOutCandidate = Next;
                            Found = true;
                        }
                    }
                }
                std::stable_sort(NextBeam.begin(), NextBeam.end(), [&](const TetBoardLocalFillCandidate &A, const TetBoardLocalFillCandidate &B) { return _IsBoardLocalCandidateBetter(A, B); });
                if (NextBeam.size() > CET_BOARD_LOCAL_FILL_MAX_VARIANTS_PER_REGION)
                    NextBeam.resize(CET_BOARD_LOCAL_FILL_MAX_VARIANTS_PER_REGION);
                if (NextBeam.size() > CET_BOARD_LOCAL_FILL_BEAM_WIDTH)
                    NextBeam.resize(CET_BOARD_LOCAL_FILL_BEAM_WIDTH);
                Beam = std::move(NextBeam);
            }
            return Found;
        }
        bool CetPolygonBoardRepairer::_FindBestLocalCandidateForTargetBin(int ATargetBin, const std::vector<TetClusterFreeRegion> &AFreeRegions, TetBoardLocalFillCandidate &ABestCandidate)
        {
            ABestCandidate = TetBoardLocalFillCandidate{};
            bool Found = false;
            for (const TetClusterFreeRegion &FreeRegion : AFreeRegions) {
                if (!_CanContinueSearch())
                    break;
                TetBoardLocalFillCandidate Candidate;
                if (_BuildLocalCandidateForFreeRegion(ATargetBin, FreeRegion, Candidate) && (!Found || _IsBoardLocalCandidateBetter(Candidate, ABestCandidate))) {
                    ABestCandidate = std::move(Candidate);
                    Found = true;
                }
            }
            if (Found)
                std::cout << "[BOARD LOCAL FILL][SEARCH] Bin=" << ATargetBin << " Parts=" << ABestCandidate.Placements.size() << " Gain=" << ABestCandidate.OccupiedAreaGain << std::endl;
            return Found;
        }
        bool CetPolygonBoardRepairer::_ApplyLocalFillCandidate(const TetBoardLocalFillCandidate &ACandidate)
        {
            if (_Items == nullptr || !ACandidate.Valid || ACandidate.Placements.size() < 2)
                return false;
            const CetTNestItemVector OriginalItems = *_Items;
            const double BeforeArea = _CalculateBinOccupiedArea(ACandidate.TargetBin);
            for (const TetHoleFillCandidate &Placement : ACandidate.Placements) {
                if (!Placement.Valid || Placement.ItemIndex >= _Items->size() || ((*_Items)[Placement.ItemIndex].binId() >= 0 && (*_Items)[Placement.ItemIndex].binId() <= ACandidate.TargetBin)) {
                    *_Items = OriginalItems;
                    return false;
                }
                CetNestItem &Item = (*_Items)[Placement.ItemIndex];
                Item.translation(Placement.Translation);
                Item.rotation(Placement.Rotation);
                Item.binId(Placement.TargetBin);
            }
            for (const TetHoleFillCandidate &Placement : ACandidate.Placements)
                if (!_IsCurrentPlacementValid(Placement.ItemIndex)) {
                    *_Items = OriginalItems;
                    return false;
                }
            if (_CalculateBinOccupiedArea(ACandidate.TargetBin) <= BeforeArea + 1.0) {
                *_Items = OriginalItems;
                return false;
            }
            return true;
        }
        bool CetPolygonBoardRepairer::_FindBestCandidateForTargetBin(int ATargetBin, const std::vector<TetClusterFreeRegion> &AFreeRegions, TetHoleFillCandidate &ABestCandidate)
        {
            if (_Items == nullptr)
                return false;
            bool Found = false;
            for (std::size_t i = 0; i < _Items->size(); ++i) {
                const int OldBin = static_cast<int>((*_Items)[i].binId());
                if (OldBin >= 0 && OldBin <= ATargetBin)
                    continue;
                TetHoleFillCandidate Candidate;
                if (_TryFindBestPlacementInBin(i, ATargetBin, AFreeRegions, Candidate) && (!Found || Candidate.Score > ABestCandidate.Score)) {
                    ABestCandidate = Candidate;
                    Found = true;
                }
            }
            return Found;
        }
        bool CetPolygonBoardRepairer::_EvaluateBoardFillPlacement(const TetBoardFillPlacementRequest &ARequest, long long &ACheckedCount, TetHoleFillCandidate &ABestCandidate)
        {
            const std::size_t AItemIndex = ARequest.ItemIndex; const int ATargetBin = ARequest.TargetBin; const int AOldBin = ARequest.OldBin; const auto &AFreeRegion = ARequest.FreeRegion; const auto &ARotation = ARequest.Rotation; const auto &ATranslation = ARequest.Translation; const long long ACheckLimit = ARequest.CheckLimit;
            if (ACheckedCount >= ACheckLimit || !_CanContinueSearch())
                return false;
            ++ACheckedCount;
            ++m_PlacementChecks;
            --m_RemainingPlacementChecks;
            TetPlacementCandidate Placement;
            Placement.ItemIndex = AItemIndex;
            Placement.TargetBin = ATargetBin;
            Placement.Rotation = ARotation;
            Placement.Translation = ATranslation;
            if (!_IsPlacementInsideFreeRegion(Placement, AFreeRegion) || !_CanPlaceAt(Placement))
                return false;
            const double Score = _CalcHoleFillScore(AItemIndex, AOldBin, ATargetBin, ATranslation);
            if (!ABestCandidate.Valid || Score > ABestCandidate.Score) {
                ABestCandidate = {true, AItemIndex, AOldBin, ATargetBin, ATranslation, ARotation, Score};
            }
            return true;
        }
        std::vector<std::size_t> CetPolygonBoardRepairer::_SelectContactVertexIndices(const CetPath &AContour) const
        {
            std::vector<std::size_t> Result;
            if (AContour.empty())
                return Result;
            std::array<std::size_t, 4> Extremes{0, 0, 0, 0};
            for (std::size_t Index = 1; Index < AContour.size(); ++Index) {
                if (AContour[Index].X < AContour[Extremes[0]].X)
                    Extremes[0] = Index;
                if (AContour[Index].X > AContour[Extremes[1]].X)
                    Extremes[1] = Index;
                if (AContour[Index].Y < AContour[Extremes[2]].Y)
                    Extremes[2] = Index;
                if (AContour[Index].Y > AContour[Extremes[3]].Y)
                    Extremes[3] = Index;
            }
            for (std::size_t Index : Extremes) {
                if (std::find(Result.begin(), Result.end(), Index) == Result.end())
                    Result.push_back(Index);
            }
            for (std::size_t Slot = 0; Result.size() < CET_BOARD_COMPOSITE_MAX_CONTACT_VERTICES_PER_CONTOUR && Slot < AContour.size(); ++Slot) {
                const std::size_t Index = Slot * AContour.size() / CET_BOARD_COMPOSITE_MAX_CONTACT_VERTICES_PER_CONTOUR;
                if (std::find(Result.begin(), Result.end(), Index) == Result.end())
                    Result.push_back(Index);
            }
            return Result;
        }
        void CetPolygonBoardRepairer::_ProbeContourContactPlacements(const TetBoardFillPlacementRequest &ARequest, long long &ACheckedCount, TetHoleFillCandidate &ABestCandidate)
        {
            const std::size_t AItemIndex = ARequest.ItemIndex; const int ATargetBin = ARequest.TargetBin; const int AOldBin = ARequest.OldBin; const auto &AFreeRegion = ARequest.FreeRegion; const auto &ARotation = ARequest.Rotation; const CetPath &ARotatedContour = *ARequest.RotatedContour; const long long AProbeLimit = ARequest.ProbeLimit; const long long ACheckLimit = ARequest.CheckLimit;
            if (ARotatedContour.empty() || AFreeRegion.Contour.empty() || AProbeLimit <= 0)
                return;
            const std::vector<std::size_t> ItemVertices = _SelectContactVertexIndices(ARotatedContour);
            const std::vector<std::size_t> RegionVertices = _SelectContactVertexIndices(AFreeRegion.Contour);
            if (ItemVertices.empty() || RegionVertices.empty())
                return;
            double ItemMinX = static_cast<double>(ARotatedContour.front().X);
            double ItemMaxX = ItemMinX, ItemMinY = static_cast<double>(ARotatedContour.front().Y), ItemMaxY = ItemMinY;
            for (const ClipperLib::IntPoint &Point : ARotatedContour) {
                ItemMinX = std::min(ItemMinX, static_cast<double>(Point.X));
                ItemMaxX = std::max(ItemMaxX, static_cast<double>(Point.X));
                ItemMinY = std::min(ItemMinY, static_cast<double>(Point.Y));
                ItemMaxY = std::max(ItemMaxY, static_cast<double>(Point.Y));
            }
            const double InsetStep = std::min(ItemMaxX - ItemMinX, ItemMaxY - ItemMinY) * CET_BOARD_COMPOSITE_CONTACT_INSET_STEP_RATIO;
            const double RegionCenterX = (AFreeRegion.MinX + AFreeRegion.MaxX) * 0.5;
            const double RegionCenterY = (AFreeRegion.MinY + AFreeRegion.MaxY) * 0.5;
            const long long StartChecks = ACheckedCount;
            const std::size_t PairCount = ItemVertices.size() * RegionVertices.size();
            for (std::size_t Pair = 0; Pair < PairCount && ACheckedCount - StartChecks < AProbeLimit; ++Pair) {
                const std::size_t ItemSlot = Pair % ItemVertices.size();
                const std::size_t RegionSlot = Pair / ItemVertices.size();
                const ClipperLib::IntPoint &ItemPoint = ARotatedContour[ItemVertices[ItemSlot]];
                const ClipperLib::IntPoint &RegionPoint = AFreeRegion.Contour[RegionVertices[RegionSlot]];
                const double DirectionX = RegionCenterX - static_cast<double>(RegionPoint.X);
                const double DirectionY = RegionCenterY - static_cast<double>(RegionPoint.Y);
                const double DirectionLength = std::hypot(DirectionX, DirectionY);
                for (std::size_t Level = 0; Level < CET_BOARD_COMPOSITE_CONTACT_INSET_LEVELS && ACheckedCount - StartChecks < AProbeLimit; ++Level) {
                    const double Inset = InsetStep * static_cast<double>(Level);
                    const double UnitX = DirectionLength > 0.0 ? DirectionX / DirectionLength : 0.0;
                    const double UnitY = DirectionLength > 0.0 ? DirectionY / DirectionLength : 0.0;
                    const Point Translation(static_cast<ClipperLib::cInt>(std::llround(RegionPoint.X - ItemPoint.X + UnitX * Inset)), static_cast<ClipperLib::cInt>(std::llround(RegionPoint.Y - ItemPoint.Y + UnitY * Inset)));
                    _EvaluateBoardFillPlacement({AItemIndex, ATargetBin, AOldBin, AFreeRegion, ARotation, Translation, nullptr, 0, ACheckLimit}, ACheckedCount, ABestCandidate);
                }
            }
        }
        bool CetPolygonBoardRepairer::_TryFindBestPlacementInBin(std::size_t AItemIndex, int ATargetBin, const std::vector<TetClusterFreeRegion> &AFreeRegions, TetHoleFillCandidate &ABestCandidate, long long ACheckLimit)
        {
            if (_Items == nullptr || _Options == nullptr || _BinPoly == nullptr || AItemIndex >= _Items->size() || AFreeRegions.empty() || !_CanContinueSearch())
                return false;
            const int OldBin = static_cast<int>((*_Items)[AItemIndex].binId());
            const long long CheckLimit = ACheckLimit > 0 ? std::min({ACheckLimit, m_PerItemPlacementCheckLimit, m_RemainingPlacementChecks}) : std::min(m_PerItemPlacementCheckLimit, m_RemainingPlacementChecks);
            const std::size_t PairCount = std::max<std::size_t>(1, m_Rotations.size() * AFreeRegions.size());
            const long long ContactLimit = static_cast<long long>(std::floor(static_cast<double>(CheckLimit) * CET_BOARD_COMPOSITE_CONTACT_PROBE_BUDGET_RATIO));
            const long long ContactPerPair = std::max<long long>(1, ContactLimit / static_cast<long long>(PairCount));
            long long CheckedCount = 0;
            for (const Radians Angle : m_Rotations) {
                CetNestItem RotatedItem = (*_Items)[AItemIndex];
                RotatedItem.translation(Point(0, 0));
                RotatedItem.rotation(Angle);
                RotatedItem.inflation(0);
                const auto Bounds = RotatedItem.boundingBox();
                const CetPath RotatedContour = RotatedItem.transformedShape().Contour;
                for (const TetClusterFreeRegion &Region : AFreeRegions) {
                    if (Bounds.width() > Region.Width || Bounds.height() > Region.Height)
                        continue;
                    _ProbeContourContactPlacements({AItemIndex, ATargetBin, OldBin, Region, Angle, Point(0, 0), &RotatedContour, ContactPerPair, ContactLimit}, CheckedCount, ABestCandidate);
                    if (CheckedCount >= ContactLimit || !_CanContinueSearch())
                        break;
                }
                if (CheckedCount >= ContactLimit || !_CanContinueSearch())
                    break;
            }
            const double GridStep = _GetEffectiveGridStep(std::max<long long>(1, CheckLimit - CheckedCount));
            for (const Radians Angle : m_Rotations)
                for (const TetClusterFreeRegion &Region : AFreeRegions) {
                    for (double Y = NestUtils::FromNestCoord(static_cast<ClipperLib::cInt>(std::llround(Region.MinY))); Y <= NestUtils::FromNestCoord(static_cast<ClipperLib::cInt>(std::llround(Region.MaxY))); Y += GridStep) {
                        for (double X = NestUtils::FromNestCoord(static_cast<ClipperLib::cInt>(std::llround(Region.MinX))); X <= NestUtils::FromNestCoord(static_cast<ClipperLib::cInt>(std::llround(Region.MaxX))); X += GridStep) {
                            if (CheckedCount >= CheckLimit || !_CanContinueSearch())
                                return ABestCandidate.Valid;
                            TetPlacementCandidate Placement;
                            Placement.ItemIndex = AItemIndex;
                            Placement.TargetBin = ATargetBin;
                            Placement.Translation = Point(0, 0);
                            Placement.Rotation = Angle;
                            _FillTranslationForBBoxMin(Placement, X, Y);
                            _EvaluateBoardFillPlacement({AItemIndex, ATargetBin, OldBin, Region, Angle, Placement.Translation, nullptr, 0, CheckLimit}, CheckedCount, ABestCandidate);
                        }
                    }
                }
            return ABestCandidate.Valid;
        }
        bool CetPolygonBoardRepairer::_IsPlacementInsideFreeRegion(const TetPlacementCandidate &APlacement, const TetClusterFreeRegion &AFreeRegion) const
        {
            if (_Items == nullptr || APlacement.ItemIndex >= _Items->size())
                return false;
            CetNestItem Item = (*_Items)[APlacement.ItemIndex];
            Item.translation(APlacement.Translation);
            Item.rotation(APlacement.Rotation);
            Item.inflation(0);
            const double Tolerance = std::max(1.0, AFreeRegion.Area * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE);
            CetClusterGeometryHelper Geometry;
            return Geometry.IsContourInsideFreeRegion(Item.transformedShape().Contour, AFreeRegion, Tolerance);
        }
        bool CetPolygonBoardRepairer::_ApplyHoleFillCandidate(const TetHoleFillCandidate &ACandidate)
        {
            if (_Items == nullptr || !ACandidate.Valid || ACandidate.ItemIndex >= _Items->size())
                return false;
            CetNestItem &Item = (*_Items)[ACandidate.ItemIndex];
            const Point OldTranslation = Item.translation();
            const Radians OldRotation = Item.rotation();
            const int OldBin = Item.binId();
            Item.translation(ACandidate.Translation);
            Item.rotation(ACandidate.Rotation);
            Item.binId(ACandidate.TargetBin);
            if (_IsCurrentPlacementValid(ACandidate.ItemIndex))
                return true;
            Item.translation(OldTranslation);
            Item.rotation(OldRotation);
            Item.binId(OldBin);
            return false;
        }
        double CetPolygonBoardRepairer::_CalcHoleFillScore(std::size_t AItemIndex, int AOldBin, int ATargetBin, const libnest2d::Point &ATranslation)
        {
            if (_Items == nullptr || AItemIndex >= _Items->size()) {
                return -std::numeric_limits<double>::max();
            }
            const auto &Item = (*_Items)[AItemIndex];
            double ItemArea = std::abs(static_cast<double>(Item.area()));
            double BinImprove = static_cast<double>(AOldBin - ATargetBin);
            double XPenalty = static_cast<double>(ATranslation.X) * 0.000001;
            double YPenalty = static_cast<double>(ATranslation.Y) * 0.000001;
            double Score = ItemArea * 10.0 + BinImprove * 1000000.0 - XPenalty - YPenalty;
            return Score;
        }
        double CetPolygonBoardRepairer::_CalculateBinOccupiedArea(int ABinId) const
        {
            if (_Items == nullptr || ABinId < 0)
                return 0.0;
            double Area = 0.0;
            for (const CetNestItem &Item : *_Items) {
                if (Item.binId() == ABinId)
                    Area += std::abs(static_cast<double>(Item.area()));
            }
            return Area;
        }
    } // namespace NEST2DMANAGERLIB
} // namespace ET
