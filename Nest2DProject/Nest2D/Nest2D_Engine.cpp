#include "pch.h"
#include "Nest2D_Engine.h"
#include "Nest2D_ClusterManager.h"
#include "Nest2D_DataConst.h"
#include "Nest2D_LocalCompactor.h"
#include "Nest2D_QuarterTurnOptimizer.h"
#include "Nest2D_RectangleGridOptimizer.h"
#include "Nest2D_PolygonBoardRepairer.h"
#include "Nest2D_PrivateDataType.h"
#include "Nest2D_RotationUtils.h"
#include "Nest2D_SelfFunction.h"
#include "Nest2D_ShapeAnalyzer.h"
#include "NestUtils.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <tuple>
#include <vector>
// #include"libnest2d/optimizers/nlopt/subplex.hpp"
using namespace ClipperLib;
using namespace libnest2d;
namespace ET {
    namespace NEST2DMANAGERLIB {
        CetNest2DEngine::CetNest2DEngine() : CetCoreObject() {}
        CetNest2DEngine::~CetNest2DEngine() {}
        static void FillRotations(std::vector<libnest2d::Radians> &ARotations, int ARotationCount) { ARotations = CetRotationUtils::BuildAllowedLibRotations(ARotationCount); }
        template <typename TSelector> static std::size_t RunRectangleNestWithSelector(CetTNestItemVector &AItems, const TetNestOptions &AOptions, TetNestProgressTracker &ATracker, bool AAllowRotations)
        {
            const auto Width = NestUtils::ToNestCoord(AOptions.BinWidth);
            const auto Height = NestUtils::ToNestCoord(AOptions.BinHeight);
            Box Bin(Width, Height, {Width / 2, Height / 2});
            using TPlacer = placers::_BottomLeftPlacer<CetPolygonImpl>;
            NestConfig<TPlacer, TSelector> Config;
            Config.placer_config.min_obj_distance = NestUtils::ToNestCoord(AOptions.Spacing);
            Config.placer_config.epsilon = 1;
            Config.placer_config.allow_rotations = AAllowRotations && CetRotationUtils::IsAllowedRotation(CET_CLUSTER_HALF_PI, AOptions.Rotations, 1e-9);
            std::cout << "================ DEBUG INFO ================" << std::endl;
            std::cout << "UsePolygonBoard: false" << std::endl;
            std::cout << "Bin Width: " << Bin.width() << ", Height: " << Bin.height() << std::endl;
            std::cout << "Spacing: " << NestUtils::ToNestCoord(AOptions.Spacing) << std::endl;
            std::cout << "============================================" << std::endl;
            const std::size_t Layers = nest(AItems, Bin, NestUtils::ToNestCoord(AOptions.Spacing), Config, ProgressFunction{ATracker});
            std::cout << "[NEST] Layers = " << Layers << std::endl;
            Nest2DUtils->Nest2DStrategy->PrintBinCount(AItems);
            return Layers;
        }
        static std::size_t RunRectangleNestFromOppositeEdge(CetTNestItemVector &AItems, const TetNestOptions &AOptions, TetNestProgressTracker &ATracker)
        {
            const auto Width = NestUtils::ToNestCoord(AOptions.BinWidth);
            const auto Height = NestUtils::ToNestCoord(AOptions.BinHeight);
            Box Bin(Width, Height, {Width / 2, Height / 2});
            using TPlacer = placers::_NofitPolyPlacer<CetPolygonImpl>;
            using TSelector = selections::_FirstFitSelection<CetPolygonImpl>;
            NestConfig<TPlacer, TSelector> Config;
            Config.placer_config.accuracy = AOptions.Placer.Accuracy;
            Config.placer_config.parallel = AOptions.Placer.Parallel;
            Config.placer_config.explore_holes = false;
            FillRotations(Config.placer_config.rotations, AOptions.Rotations);
            Config.placer_config.alignment = placers::NfpPConfig<CetPolygonImpl>::Alignment::TOP_RIGHT;
            Config.placer_config.starting_point = placers::NfpPConfig<CetPolygonImpl>::Alignment::TOP_RIGHT;
            const std::size_t Layers = nest(AItems, Bin, NestUtils::ToNestCoord(AOptions.Spacing), Config, ProgressFunction{ATracker});
            std::cout << "[NEST][OPPOSITE EDGE] Layers=" << Layers << std::endl;
            Nest2DUtils->Nest2DStrategy->PrintBinCount(AItems);
            return Layers;
        }
        static void ApplyClusterEdgeClearance(CetTNestItemVector &AItems, const std::vector<TetMetaItem> &AMetaItems, const TetNestOptions &AOptions)
        {
            const auto Clearance = static_cast<libnest2d::Coord>(std::ceil(static_cast<double>(NestUtils::ToNestCoord(std::max(0.0, AOptions.Spacing))) * 0.5));
            for (std::size_t Index = 0; Index < AItems.size() && Index < AMetaItems.size(); ++Index) {
                if (AMetaItems[Index].IsCluster)
                    AItems[Index].inflation(Clearance);
            }
        }
        static void ClearItemInflation(CetTNestItemVector &AItems)
        {
            for (CetNestItem &Item : AItems)
                Item.inflation(0);
        }
        static placers::NfpPConfig<CetPolygonImpl>::Alignment ToLibNestAlignment(MetNestAlignment AAlignment)
        {
            using CetAlignment = placers::NfpPConfig<CetPolygonImpl>::Alignment;
            switch (AAlignment) {
            case MetNestAlignment::DontAlign:
                return CetAlignment::DONT_ALIGN;
            case MetNestAlignment::BottomLeft:
            default:
                return CetAlignment::BOTTOM_LEFT;
            }
        }
        static bool IsLockedEnvelopeCluster(const TetMetaItem &AMeta)
        {
            if (!AMeta.IsCluster)
                return false;
            const bool IsCircleEnvelope = AMeta.ClusterType.find("Circle") == 0;
            const bool IsFilledEllipseEnvelope = AMeta.ClusterType.find("Ellipse") == 0 && AMeta.ClusterType.find("_EnvelopeFill") != std::string::npos;
            return IsCircleEnvelope || IsFilledEllipseEnvelope;
        }
        static bool HasLockedEnvelopeCluster(const std::vector<TetMetaItem> &AMetaItems)
        {
            for (const TetMetaItem &Meta : AMetaItems) {
                if (IsLockedEnvelopeCluster(Meta))
                    return true;
            }
            return false;
        }
        static std::vector<std::size_t> CollectLockedEnvelopeChildren(const std::vector<TetMetaItem> &AMetaItems)
        {
            std::vector<std::size_t> Indices;
            for (const TetMetaItem &Meta : AMetaItems) {
                if (!IsLockedEnvelopeCluster(Meta))
                    continue;
                for (const TetItemTransform &Transform : Meta.TransformData)
                    if (Transform.OriginalId >= 0)
                        Indices.push_back(static_cast<std::size_t>(Transform.OriginalId));
            }
            std::sort(Indices.begin(), Indices.end());
            Indices.erase(std::unique(Indices.begin(), Indices.end()), Indices.end());
            return Indices;
        }
        static bool PreservesLockedChildren(const CetTNestItemVector &ABefore, const CetTNestItemVector &AAfter, const std::vector<std::size_t> &AIndices)
        {
            if (ABefore.size() != AAfter.size())
                return false;
            for (std::size_t Index : AIndices) {
                if (Index >= ABefore.size() || Index >= AAfter.size())
                    return false;
                const Point BeforePoint = ABefore[Index].translation();
                const Point AfterPoint = AAfter[Index].translation();
                if (BeforePoint.X != AfterPoint.X || BeforePoint.Y != AfterPoint.Y || std::abs(static_cast<double>(ABefore[Index].rotation()) - static_cast<double>(AAfter[Index].rotation())) > CET_CLUSTER_FILL_VARIANT_ROTATION_TOLERANCE)
                    return false;
            }
            return true;
        }
        // Refill sheets with complete cluster proxies. Children stay glued through
        // their metadata and are expanded only after this multi-sheet pass.
        using ClusterBackfillPlacer = placers::_BottomLeftPlacer<CetPolygonImpl>;
        using ClusterBackfillConfig = placers::BLConfig<CetPolygonImpl>;
        bool RepackClusterItems(CetTNestItemVector &AItems, const std::vector<std::size_t> &AIndices, const Box &ABin, const ClusterBackfillConfig &AConfig, int ABinId, long long AInflation)
        {
            std::vector<std::size_t> OrderedIndices = AIndices;
            std::stable_sort(OrderedIndices.begin(), OrderedIndices.end(), [&](std::size_t A, std::size_t B) { return std::abs(static_cast<double>(AItems[A].area())) > std::abs(static_cast<double>(AItems[B].area())); });
            CetTNestItemVector Repacked;
            Repacked.reserve(OrderedIndices.size());
            for (std::size_t Index : OrderedIndices) {
                CetNestItem Copy = AItems[Index];
                Copy.translation(ClipperLib::IntPoint(0, 0));
                Copy.rotation(0.0);
                Copy.inflation(static_cast<decltype(Copy.inflation())>(AInflation));
                Repacked.push_back(std::move(Copy));
            }
            ClusterBackfillPlacer Repacker(ABin);
            Repacker.configure(AConfig);
            for (CetNestItem &Item : Repacked) {
                if (!Repacker.pack(Item))
                    return false;
                Item.binId(ABinId);
                Item.inflation(0);
            }
            for (std::size_t Position = 0; Position < OrderedIndices.size(); ++Position)
                AItems[OrderedIndices[Position]] = std::move(Repacked[Position]);
            return true;
        }
        static std::size_t BackfillClusterSheets(CetTNestItemVector &AItems, const TetNestOptions &AOptions, std::size_t ALayers)
        {
            if (ALayers <= 1 || AItems.empty())
                return ALayers;
            if (AItems.size() > CET_NEST_FULL_STRATEGY_ITEM_LIMIT) {
                // This optional repack repeatedly invokes the bottom-left placer. Once
                // local spacing fallback has expanded a large proxy set, doing it here
                // can cost minutes while not affecting the already valid main nest.
                std::cout << "[NEST][CLUSTER BACKFILL][SKIP] PackedItems=" << AItems.size() << ", Limit=" << CET_NEST_FULL_STRATEGY_ITEM_LIMIT << std::endl;
                return ALayers;
            }
            const auto Width = NestUtils::ToNestCoord(AOptions.BinWidth);
            const auto Height = NestUtils::ToNestCoord(AOptions.BinHeight);
            Box Bin(Width, Height, {Width / 2, Height / 2});
            placers::BLConfig<CetPolygonImpl> Config;
            // A proxy only approximates its expanded children. Reserve one spacing
            // margin on each side while repacking so the later child validation does
            // not reopen a sheet because a proxy boundary merely touched a single.
            Config.min_obj_distance = 0;
            Config.epsilon = 1;
            Config.allow_rotations = CetRotationUtils::IsAllowedRotation(CET_CLUSTER_HALF_PI, AOptions.Rotations, 1e-9);
            const long long BackfillInflation = static_cast<long long>(std::ceil(static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)) * 0.5));
            std::vector<std::size_t> AllIndices(AItems.size());
            std::iota(AllIndices.begin(), AllIndices.end(), 0);
            if (RepackClusterItems(AItems, AllIndices, Bin, Config, 0, BackfillInflation)) {
                std::cout << "[NEST][CLUSTER BACKFILL][FULL] All packed proxies fit on bin 0." << std::endl;
                return 1;
            }
            std::set<int> AffectedBins;
            std::size_t Moved = 0;
            for (std::size_t Target = 0; Target + 1 < ALayers; ++Target) {
                std::vector<std::size_t> TargetIndices;
                for (std::size_t Index = 0; Index < AItems.size(); ++Index) {
                    if (AItems[Index].binId() == static_cast<int>(Target))
                        TargetIndices.push_back(Index);
                }
                std::vector<std::size_t> Candidates;
                for (std::size_t Index = 0; Index < AItems.size(); ++Index) {
                    if (AItems[Index].binId() > static_cast<int>(Target))
                        Candidates.push_back(Index);
                }
                std::stable_sort(Candidates.begin(), Candidates.end(), [&](std::size_t A, std::size_t B) { return AItems[A].area() < AItems[B].area(); });
                std::size_t Attempts = 0;
                for (std::size_t Index : Candidates) {
                    if (Attempts++ >= 32)
                        break;
                    if (AItems[Index].binId() <= static_cast<int>(Target))
                        continue;
                    const int Source = AItems[Index].binId();
                    std::vector<std::size_t> TrialIndices = TargetIndices;
                    TrialIndices.push_back(Index);
                    // Repack the target and candidate together while preserving glued proxies.
                    if (!RepackClusterItems(AItems, TrialIndices, Bin, Config, static_cast<int>(Target), BackfillInflation))
                        continue;
                    TargetIndices.push_back(Index);
                    AffectedBins.insert(Source);
                    ++Moved;
                }
                std::cout << "[NEST][CLUSTER BACKFILL] Target=" << Target << ", Attempts=" << Attempts << std::endl;
            }
            // Repack only source sheets affected by proxy moves. All proxies remain
            // intact, so their child spacing and glued geometry are preserved.
            for (int SourceBin : AffectedBins) {
                std::vector<std::size_t> Indices;
                for (std::size_t Index = 0; Index < AItems.size(); ++Index) {
                    if (AItems[Index].binId() == SourceBin)
                        Indices.push_back(Index);
                }
                const bool Success = RepackClusterItems(AItems, Indices, Bin, Config, SourceBin, BackfillInflation);
                if (Success) {
                    // RepackClusterItems has already written the packed items back.
                }
                std::cout << "[NEST][CLUSTER REPACK] Bin=" << SourceBin << ", Items=" << Indices.size() << ", Applied=" << (Success ? 1 : 0) << std::endl;
            }
            std::set<int> UsedBins;
            for (const CetNestItem &Item : AItems)
                if (Item.binId() >= 0)
                    UsedBins.insert(Item.binId());
            std::map<int, int> Dense;
            int NextBin = 0;
            for (int BinId : UsedBins)
                Dense[BinId] = NextBin++;
            for (CetNestItem &Item : AItems) {
                auto It = Dense.find(Item.binId());
                if (It != Dense.end())
                    Item.binId(It->second);
            }
            std::cout << "[NEST][CLUSTER BACKFILL SUMMARY] Moved=" << Moved << ", LayersBefore=" << ALayers << ", LayersAfter=" << UsedBins.size() << std::endl;
            return UsedBins.size();
        }
        static std::vector<MetClusterStrategy> BuildClusterStrategies(const std::vector<TetShapeFeature> &AFeatures)
        {
            if (AFeatures.size() > CET_NEST_FULL_STRATEGY_ITEM_LIMIT) {
                // TemplateCluster preserves every unclustered item as a single and
                // validates full coverage. Avoid an additional full NFP pass over a
                // large original order before evaluating the reduced proxy set.
                return {MetClusterStrategy::TemplateCluster};
            }
            const std::size_t CustomShapeCount = static_cast<std::size_t>(std::count_if(AFeatures.begin(), AFeatures.end(), [](const TetShapeFeature &AFeature) { return AFeature.ShapeType == MetShapeType::QuadrilateralLike || AFeature.ShapeType == MetShapeType::ConvexPolygon || AFeature.ShapeType == MetShapeType::ConcavePolygon; }));
            const bool HasLargeCustomMajority = AFeatures.size() >= 32 && CustomShapeCount * 2 >= AFeatures.size();
            if (HasLargeCustomMajority) {
                return {MetClusterStrategy::TemplateCluster};
            }
            return {MetClusterStrategy::None, MetClusterStrategy::TemplateCluster};
        }
        static TetClusterBuildResult DissolvePackedClusters(const CetTNestItemVector &AOriginalItems, const TetClusterBuildResult &ASource, const std::set<int> &APackedIndices)
        {
            TetClusterBuildResult Result;
            Result.NestItems.reserve(ASource.NestItems.size() + APackedIndices.size() * 4);
            Result.MetaItems.reserve(ASource.MetaItems.size() + APackedIndices.size() * 4);
            for (std::size_t PackedIndex = 0; PackedIndex < ASource.NestItems.size() && PackedIndex < ASource.MetaItems.size(); ++PackedIndex) {
                const TetMetaItem &SourceMeta = ASource.MetaItems[PackedIndex];
                const bool Dissolve = APackedIndices.find(static_cast<int>(PackedIndex)) != APackedIndices.end() && SourceMeta.IsCluster;
                if (!Dissolve) {
                    Result.NestItems.push_back(ASource.NestItems[PackedIndex]);
                    TetMetaItem Meta = SourceMeta;
                    Meta.PackedItemIndex = static_cast<int>(Result.MetaItems.size());
                    Result.MetaItems.push_back(std::move(Meta));
                    continue;
                }
                for (const TetItemTransform &Transform : SourceMeta.TransformData) {
                    if (Transform.OriginalId < 0 || Transform.OriginalId >= static_cast<int>(AOriginalItems.size())) {
                        continue;
                    }
                    Result.NestItems.push_back(AOriginalItems[Transform.OriginalId]);
                    TetMetaItem Meta;
                    Meta.PackedItemIndex = static_cast<int>(Result.MetaItems.size());
                    Meta.IsCluster = false;
                    Meta.ClusterType = "SpacingFallbackSingle";
                    Meta.TransformData.push_back({Transform.OriginalId, 0.0, 0.0, 0.0});
                    Result.MetaItems.push_back(std::move(Meta));
                }
            }
            return Result;
        }
        bool CetNest2DEngine::_RunLastBinEvacuation(CetTNestItemVector &ANestItems, const TetNestOptions &AOptions, std::size_t &ALayers)
        {
            if (!AOptions.EnableLastBinEvacuation || ANestItems.empty() || ALayers <= 1) {
                return false;
            }
            const CetTNestItemVector OriginalSolution = ANestItems;
            double BoardBinWidth = AOptions.BinWidth;
            double BoardBinHeight = AOptions.BinHeight;
            CetPolygonImpl BinPoly = Nest2DUtils->Nest2DBord->BuildBinPolygonFromOptions(AOptions, BoardBinWidth, BoardBinHeight);
            CetPolygonBoardRepairer Repairer(ANestItems, AOptions, BinPoly, BoardBinWidth, BoardBinHeight);
            TetLastBinEvacuationStats Stats;
            const bool Success = Repairer.EvacuateLastBin(ALayers, Stats);
            if (!Success) {
                ANestItems = OriginalSolution;
            }
#ifdef _DEBUG
            std::cout << "[LAST_BIN] Start UsedBins=" << Stats.BeforeUsedBins << ", LastBin=" << Stats.LastBinId << ", LastBinItems=" << Stats.LastBinItemCount << ", LastBinArea=" << Stats.LastBinArea << std::endl;
            std::cout << (Success ? "[LAST_BIN][SUCCESS]" : "[LAST_BIN][FAILED]") << " UsedBins " << Stats.BeforeUsedBins << " -> " << Stats.AfterUsedBins << ", DirectMoves=" << Stats.DirectMoves << ", SameBinRelocations=" << Stats.SameBinRelocations << ", RelocatedExistingSmallItems=" << Stats.RelocatedExistingSmallItemCount << ", PlacementChecks=" << Stats.PlacementChecks << ", SearchBudgetReached=" << Stats.SearchBudgetReached << ", RemainingItems=" << Stats.RemainingItems << ", TimeMs=" << Stats.TimeMs << ", Rollback=" << Stats.RolledBack << std::endl;
            if (!Success) {
                std::cout << "[LAST_BIN][FAIL SUMMARY] Remaining=" << Stats.RemainingItems << ", NoCandidatePosition=" << Stats.NoCandidatePosition << ", RelocationFailed=" << Stats.RelocationFailed << ", InsufficientFreeArea=" << Stats.InsufficientFreeArea << std::endl;
            }
#endif
            return Success;
        }
        bool CetNest2DEngine::_RepairAndEvacuate(CetTNestItemVector &ANestItems, const TetNestOptions &AOptions, const CetPolygonImpl &ABinPoly, double ABinWidth, double ABinHeight, std::size_t &ALayers)
        {
            const CetTNestItemVector OriginalSolution = ANestItems;
            const std::size_t OriginalLayers = ALayers;
            CetPolygonBoardRepairer Repairer(ANestItems, AOptions, ABinPoly, ABinWidth, ABinHeight);
            const auto RepairStart = std::chrono::steady_clock::now();
            Repairer.Repair(ALayers);
            const double BeforeLastBinMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - RepairStart).count();
            std::cout << "[NEST][TIMING] BeforeLastBinMs=" << BeforeLastBinMs << std::endl;
            const bool BoardFillImproved = Repairer.HadBoardFillChanges();
            const bool LastBinImproved = _RunLastBinEvacuation(ANestItems, AOptions, ALayers);
            if (!Nest2DUtils->Nest2dRectangleGridOptimizer->ValidatePlacedItemsSpacing(ANestItems, AOptions)) {
                std::cout << "[NEST][REPAIR][ROLLBACK] Repair produced an invalid spacing result." << std::endl;
                ANestItems = OriginalSolution;
                ALayers = OriginalLayers;
                return false;
            }
            return BoardFillImproved || LastBinImproved;
        }
        bool CetNest2DEngine::_TryLockedEnvelopeBoardRepair(const TetLockedEnvelopeRepairRequest &ARequest)
        {
            CetTNestItemVector &ANestItems = ARequest.Items; const TetNestOptions &AOptions = ARequest.Options; const CetPolygonImpl &ABinPoly = ARequest.BinPolygon; const double ABinWidth = ARequest.BinWidth; const double ABinHeight = ARequest.BinHeight; const auto &ALockedChildren = ARequest.LockedChildren; std::size_t &ALayers = ARequest.Layers;
            if (ANestItems.empty() || ALockedChildren.empty() || ALayers == 0)
                return false;
            const CetTNestItemVector BeforeItems = ANestItems;
            const std::size_t BeforeLayers = ALayers;
            const TetTNestEvalResult BeforeEval = Nest2DUtils->Nest2DStrategy->EvaluateNestResult(BeforeItems, BeforeLayers);
            CetPolygonBoardRepairer Repairer(ANestItems, AOptions, ABinPoly, ABinWidth, ABinHeight);
            Repairer.RepairLockedEnvelope(ALayers, ALockedChildren);
            const bool Preserved = PreservesLockedChildren(BeforeItems, ANestItems, ALockedChildren);
            const bool Valid = Preserved && Nest2DUtils->Nest2dRectangleGridOptimizer->ValidatePlacedItemsSpacing(ANestItems, AOptions);
            const TetTNestEvalResult AfterEval = Nest2DUtils->Nest2DStrategy->EvaluateNestResult(ANestItems, ALayers);
            const bool Improved = Valid && Nest2DUtils->Nest2DStrategy->IsBetterNestResult(AfterEval, BeforeEval);
            if (!Improved) {
                ANestItems = BeforeItems;
                ALayers = BeforeLayers;
                std::cout << "[NEST][LOCKED ENVELOPE][BOARD REPAIR] Rollback Preserved=" << Preserved << " Valid=" << Valid << " Improved=" << Improved << std::endl;
                return false;
            }
            std::cout << "[NEST][LOCKED ENVELOPE][BOARD REPAIR] Accepted Layers=" << BeforeLayers << " -> " << ALayers << std::endl;
            return true;
        }    
        static TetAllBinRemnantMetric EvaluateAllBinRemnantMetric(const CetTNestItemVector &AItems, const TetNestOptions &AOptions, std::size_t ALayers)
        {
            TetAllBinRemnantMetric Result;
            if ((AOptions.Board.Enabled && AOptions.Board.Vertices.size() >= 3) || ALayers == 0)
                return Result;
            const double BinWidth = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth));
            const double BinHeight = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinHeight));
            if (BinWidth <= 0.0 || BinHeight <= 0.0)
                return Result;
            std::vector<std::vector<TetRemnantPartBounds>> BoundsByBin(ALayers);
            for (const CetNestItem &Item : AItems) {
                const int BinId = Item.binId();
                if (BinId < 0 || static_cast<std::size_t>(BinId) >= ALayers)
                    continue;
                const auto Bounds = Item.boundingBox();
                TetRemnantPartBounds PartBounds;
                PartBounds.MinX = static_cast<double>(getX(Bounds.minCorner()));
                PartBounds.MinY = static_cast<double>(getY(Bounds.minCorner()));
                PartBounds.MaxX = static_cast<double>(getX(Bounds.maxCorner()));
                PartBounds.MaxY = static_cast<double>(getY(Bounds.maxCorner()));
                if (PartBounds.MaxX > PartBounds.MinX && PartBounds.MaxY > PartBounds.MinY) {
                    BoundsByBin[static_cast<std::size_t>(BinId)].push_back(PartBounds);
                }
            }
            Result.BinReusableStripAreas.resize(ALayers, 0.0);
            for (std::size_t BinId = 0; BinId < BoundsByBin.size(); ++BinId) {
                const std::vector<TetRemnantPartBounds> &BinBounds = BoundsByBin[BinId];
                if (BinBounds.empty())
                    continue;
                std::array<double, CET_REMNANT_SKYLINE_SAMPLES> HorizontalSkyline{};
                std::array<double, CET_REMNANT_SKYLINE_SAMPLES> VerticalSkyline{};
                double UsedMaxX = 0.0;
                double UsedMaxY = 0.0;
                for (const TetRemnantPartBounds &Bounds : BinBounds) {
                    const double MinX = std::clamp(Bounds.MinX, 0.0, BinWidth);
                    const double MaxX = std::clamp(Bounds.MaxX, 0.0, BinWidth);
                    const double MinY = std::clamp(Bounds.MinY, 0.0, BinHeight);
                    const double MaxY = std::clamp(Bounds.MaxY, 0.0, BinHeight);
                    UsedMaxX = std::max(UsedMaxX, MaxX);
                    UsedMaxY = std::max(UsedMaxY, MaxY);
                    const std::size_t StartX = std::min(CET_REMNANT_SKYLINE_SAMPLES - 1, static_cast<std::size_t>(std::floor(MinX / BinWidth * CET_REMNANT_SKYLINE_SAMPLES)));
                    const std::size_t EndX = std::min(CET_REMNANT_SKYLINE_SAMPLES, static_cast<std::size_t>(std::ceil(MaxX / BinWidth * CET_REMNANT_SKYLINE_SAMPLES)));
                    for (std::size_t Sample = StartX; Sample < EndX; ++Sample) {
                        HorizontalSkyline[Sample] = std::max(HorizontalSkyline[Sample], MaxY);
                    }
                    const std::size_t StartY = std::min(CET_REMNANT_SKYLINE_SAMPLES - 1, static_cast<std::size_t>(std::floor(MinY / BinHeight * CET_REMNANT_SKYLINE_SAMPLES)));
                    const std::size_t EndY = std::min(CET_REMNANT_SKYLINE_SAMPLES, static_cast<std::size_t>(std::ceil(MaxY / BinHeight * CET_REMNANT_SKYLINE_SAMPLES)));
                    for (std::size_t Sample = StartY; Sample < EndY; ++Sample) {
                        VerticalSkyline[Sample] = std::max(VerticalSkyline[Sample], MaxX);
                    }
                }
                const double HorizontalStep = BinWidth / static_cast<double>(CET_REMNANT_SKYLINE_SAMPLES);
                const double VerticalStep = BinHeight / static_cast<double>(CET_REMNANT_SKYLINE_SAMPLES);
                const std::size_t UsedHorizontalSamples = std::min(CET_REMNANT_SKYLINE_SAMPLES, static_cast<std::size_t>(std::ceil(UsedMaxX / BinWidth * CET_REMNANT_SKYLINE_SAMPLES)));
                const std::size_t UsedVerticalSamples = std::min(CET_REMNANT_SKYLINE_SAMPLES, static_cast<std::size_t>(std::ceil(UsedMaxY / BinHeight * CET_REMNANT_SKYLINE_SAMPLES)));
                double TopWaste = 0.0;
                for (std::size_t Sample = 0; Sample < UsedHorizontalSamples; ++Sample) {
                    TopWaste += std::max(0.0, UsedMaxY - HorizontalSkyline[Sample]) * HorizontalStep;
                }
                double RightWaste = 0.0;
                for (std::size_t Sample = 0; Sample < UsedVerticalSamples; ++Sample) {
                    RightWaste += std::max(0.0, UsedMaxX - VerticalSkyline[Sample]) * VerticalStep;
                }
                const double TopArea = BinWidth * std::max(0.0, BinHeight - UsedMaxY);
                const double RightArea = BinHeight * std::max(0.0, BinWidth - UsedMaxX);
                const bool PreferTop = TopArea > RightArea || (std::abs(TopArea - RightArea) <= 1.0 && TopWaste <= RightWaste);
                const double ReusableArea = PreferTop ? TopArea : RightArea;
                Result.BinReusableStripAreas[BinId] = ReusableArea;
                Result.ReusableStripArea += ReusableArea;
                Result.SkylineWasteArea += PreferTop ? TopWaste : RightWaste;
                Result.UsedEnvelopeArea += UsedMaxX * UsedMaxY;
                Result.Valid = true;
            }
            return Result;
        }
        bool CetNest2DEngine::_TryBoardFeedbackNest(CetTNestItemVector &ANestItems, const TetNestOptions &AOptions, TetNestProgressTracker &ATracker, std::size_t &ALayers)
        {
            if (ANestItems.empty() || ALayers <= 1 || ANestItems.size() > CET_BOARD_FEEDBACK_NEST_MAX_ITEM_COUNT) {
                std::cout << "[BOARD FEEDBACK][SKIP] Items=" << ANestItems.size() << " Layers=" << ALayers << " Limit=" << CET_BOARD_FEEDBACK_NEST_MAX_ITEM_COUNT << std::endl;
                return false;
            }
            const CetTNestItemVector OriginalSolution = ANestItems;
            const std::size_t OriginalLayers = ALayers;
            const TetTNestEvalResult OriginalEval = Nest2DUtils->Nest2DStrategy->EvaluateNestResult(OriginalSolution, OriginalLayers);
            std::vector<std::size_t> Order(ANestItems.size());
            std::iota(Order.begin(), Order.end(), 0);
            std::stable_sort(Order.begin(), Order.end(), [&](std::size_t A, std::size_t B) {
                const CetNestItem &First = OriginalSolution[A];
                const CetNestItem &Second = OriginalSolution[B];
                if (First.binId() != Second.binId())
                    return First.binId() < Second.binId();
                const Point FirstTranslation = First.translation();
                const Point SecondTranslation = Second.translation();
                if (FirstTranslation.Y != SecondTranslation.Y)
                    return FirstTranslation.Y < SecondTranslation.Y;
                if (FirstTranslation.X != SecondTranslation.X)
                    return FirstTranslation.X < SecondTranslation.X;
                return A < B;
            });
            CetTNestItemVector FeedbackItems;
            FeedbackItems.reserve(Order.size());
            for (std::size_t Index : Order) {
                CetNestItem Item = OriginalSolution[Index];
                Item.binId(-1);
                Item.translation(Point(0, 0));
                Item.rotation(Radians(0.0));
                Item.inflation(0);
                FeedbackItems.push_back(std::move(Item));
            }
            const bool UsePolygonBoard = AOptions.Board.Enabled && AOptions.Board.Vertices.size() >= 3;
            std::size_t FeedbackLayers = UsePolygonBoard ? RunPolygonNestOnce(FeedbackItems, AOptions, ATracker) : RunRectangleNestOnce(FeedbackItems, AOptions, ATracker);
            CetTNestItemVector FeedbackSolution = OriginalSolution;
            for (std::size_t Position = 0; Position < Order.size(); ++Position)
                FeedbackSolution[Order[Position]] = std::move(FeedbackItems[Position]);
            const TetTNestEvalResult FeedbackEval = Nest2DUtils->Nest2DStrategy->EvaluateNestResult(FeedbackSolution, FeedbackLayers);
            const bool Valid = FeedbackLayers > 0 && Nest2DUtils->Nest2dRectangleGridOptimizer->ValidatePlacedItemsSpacing(FeedbackSolution, AOptions);
            const bool Improved = Valid && Nest2DUtils->Nest2DStrategy->IsBetterNestResult(FeedbackEval, OriginalEval);
            std::cout << "[BOARD FEEDBACK][RESULT] BeforeLayers=" << OriginalLayers << " AfterLayers=" << FeedbackLayers << " BeforeFirstArea=" << OriginalEval.FirstBinArea << " AfterFirstArea=" << FeedbackEval.FirstBinArea << " Valid=" << Valid << " Improved=" << Improved << std::endl;
            if (!Improved)
                return false;
            ANestItems = std::move(FeedbackSolution);
            ALayers = FeedbackLayers;
            return true;
        }
        int CetNest2DEngine::RunNesting_Impl(CetTNestItemVector &ANestItems, const TetNestOptions &AOptions, std::size_t *AUsedBins)
        {
            std::cout << "[DLL]this is running nesting" << std::endl;
            if (AUsedBins != nullptr) {
                *AUsedBins = 0;
            }
            if (ANestItems.empty()) {
                return NEST2D_ERR_CORE_EMPTY_INPUT;
            }
            const bool UsePolygonBoard = AOptions.Board.Enabled && AOptions.Board.Vertices.size() >= 3;
            int TotalItems = static_cast<int>(ANestItems.size());
            TetNestProgressTracker Tracker(TotalItems, AOptions.ProgressCallback);
            std::size_t Layers = 0;
            if (UsePolygonBoard) {
                Layers = RunPolygonBoardNesting(ANestItems, AOptions, Tracker);
            } else {
                Layers = RunRectangleBoardNesting(ANestItems, AOptions, Tracker);
            }
            std::cout << "[NEST] after polygon nest, Layers = " << Layers << std::endl;
            if (Layers == 0) {
                return NEST2D_ERR_CORE_NESTING_FAILED;
            }
            if (!Nest2DUtils->Nest2dRectangleGridOptimizer->ValidatePlacedItemsSpacing(ANestItems, AOptions)) {
                std::cout << "[NEST][FINAL][ERROR] Result does not meet spacing requirements." << std::endl;
                return NEST2D_ERR_CORE_NESTING_FAILED;
            }
            if (AUsedBins != nullptr) {
                *AUsedBins = Layers;
            }
            return Nest2D_Success;
        }
        std::size_t CetNest2DEngine::RunPolygonBoardNesting(CetTNestItemVector &ANestItems, const TetNestOptions &AOptions, TetNestProgressTracker &ATracker)
        {
            std::cout << "[NEST] use custom polygon board with strategy loop" << std::endl;
            CetTNestItemVector OriginalItems = ANestItems;
            const std::vector<TetShapeFeature> Features = Nest2DUtils->Nest2DShape->AnalyzeALL(OriginalItems);
            std::cout << "[SHAPE ANALYZER][DONE]" << " ItemCount = " << OriginalItems.size() << ", FeatureCount = " << Features.size() << std::endl;
            bool HasBest = false;
            CetTNestItemVector BestItems;
            TetTNestEvalResult BestEval{};
            std::size_t BestLayers = 0;
            std::vector<TetMetaItem> BestMetaItems;
            bool BestHasCluster = false;
            bool BestHasLockedEnvelope = false;
            const std::vector<MetClusterStrategy> ClusterStrategies = BuildClusterStrategies(Features);
            for (auto ClusterStrategy : ClusterStrategies) {
                TetClusterBuildResult ClusterResult = Nest2DUtils->Nest2DCluster->BuildClusterItemsWithFeatures(OriginalItems, Features, AOptions, ClusterStrategy);
                int ClusterCount = 0;
                for (const auto &Meta : ClusterResult.MetaItems) {
                    if (Meta.IsCluster) {
                        ClusterCount++;
                    }
                }
                std::cout << "[POLYGON][CLUSTER][BUILD] Strategy = " << static_cast<int>(ClusterStrategy) << ", OriginalItems = " << OriginalItems.size() << ", PackedItems = " << ClusterResult.NestItems.size() << ", MetaItems = " << ClusterResult.MetaItems.size() << ", ClusterCount = " << ClusterCount << std::endl;
                TetExpandedSpacingFailure SpacingFailure;
                TetLocalBestResult LocalResult = EvaluateSortingStrategies(ClusterResult, OriginalItems, AOptions, ATracker, &SpacingFailure);
                if (!LocalResult.HasBest && SpacingFailure.Valid) {
                    LocalResult = _TryLocalClusterSpacingFallback(ClusterResult, OriginalItems, AOptions, ATracker, SpacingFailure);
                }
                const bool LocalHasLockedEnvelope = HasLockedEnvelopeCluster(LocalResult.MetaItems);
                bool Better = ShoouldUpdateGlobalBest(LocalResult, HasBest, BestEval, BestLayers, BestHasCluster);
                if (!Better && LocalHasLockedEnvelope && !BestHasLockedEnvelope) {
                    // A locked envelope protects its fixed outer contour during repair,
                    // but it must not displace a layout with better board utilization.
                    Better = !Nest2DUtils->Nest2DStrategy->IsBetterNestResult(BestEval, LocalResult.Eval);
                }
                if (Better) {
                    HasBest = true;
                    BestEval = LocalResult.Eval;
                    BestLayers = LocalResult.Layers;
                    BestItems = std::move(LocalResult.Items);
                    BestMetaItems = std::move(LocalResult.MetaItems);
                    BestHasCluster = LocalResult.HasCluster;
                    BestHasLockedEnvelope = LocalHasLockedEnvelope;
                    std::cout << "[POLYGON][GLOBAL BEST UPDATE] HasCluster = " << BestHasCluster << ", count = " << BestEval.FirstBinCount << ", area = " << BestEval.FirstBinArea << ", layers = " << BestEval.Layers << ", packedItems = " << BestItems.size() << std::endl;
                }
            }
            if (!HasBest) {
                std::cout << "[NEST][SPACING FALLBACK][FULL SINGLE] Trigger=all clustered candidates rejected." << std::endl;
                TetClusterBuildResult SingleItems = Nest2DUtils->Nest2DCluster->BuildClusterItems(OriginalItems, AOptions, MetClusterStrategy::None);
                TetLocalBestResult FallbackResult = _EvaluateSingleSortingStrategy(SingleItems, OriginalItems, AOptions, ATracker, MetENestOrderStrategy::LargeFirst);
                if (FallbackResult.HasBest) {
                    HasBest = true;
                    BestEval = FallbackResult.Eval;
                    BestLayers = FallbackResult.Layers;
                    BestItems = std::move(FallbackResult.Items);
                    BestMetaItems = std::move(FallbackResult.MetaItems);
                    BestHasCluster = false;
                    std::cout << "[NEST][SPACING FALLBACK][FULL SINGLE][VALID] Layers=" << BestLayers << std::endl;
                } else {
                    std::cout << "[POLYGON][FINAL] no valid best result." << std::endl;
                    return 0;
                }
            }
            if (!BestHasCluster) {
                std::cout << "[POLYGON][FINAL BEST] Restore normal item order." << std::endl;
            } else {
                std::cout << "[POLYGON][FINAL BEST] Use cluster expand." << std::endl;
            }
            const long double CoordinateScale = NestUtils::NestScale();
            std::cout << "[POLYGON][FINAL REMNANT] MetricsAvailable=" << BestEval.HasRemnantMetrics << ", AreaMm2=" << static_cast<double>(BestEval.ReusableRemnantArea / (CoordinateScale * CoordinateScale)) << ", ShortSideMm=" << static_cast<double>(BestEval.ReusableRemnantShortSide / CoordinateScale) << ", UsedDepthMm=" << static_cast<double>(BestEval.UsedDepth / CoordinateScale) << ", Direction=" << (BestEval.RemnantIsTopStrip ? "Top" : "Right") << std::endl;
            // Sorting strategies reorder packed items, so metadata restoration is required for singles and clusters.
            Nest2DUtils->Nest2DCluster->ExpandClusterResultToOriginalItems(OriginalItems, BestItems, BestMetaItems, ANestItems);
            double BoardBinWidth = AOptions.BinWidth;
            double BoardBinHeight = AOptions.BinHeight;
            CetPolygonImpl BinPoly = Nest2DUtils->Nest2DBord->BuildBinPolygonFromOptions(AOptions, BoardBinWidth, BoardBinHeight);
            if (!BestHasLockedEnvelope && _RepairAndEvacuate(ANestItems, AOptions, BinPoly, BoardBinWidth, BoardBinHeight, BestLayers)) {
                _TryBoardFeedbackNest(ANestItems, AOptions, ATracker, BestLayers);
            } else if (BestHasLockedEnvelope) {
                std::cout << "[POLYGON][LOCKED ENVELOPE] Skip expanded-item repair." << std::endl;
            }
            BestEval = Nest2DUtils->Nest2DStrategy->EvaluateNestResult(ANestItems, BestLayers);
            Nest2DUtils->Nest2dLocalCompactor->RunLocalCompactPass(ANestItems, AOptions, &BestMetaItems);
            BestEval = Nest2DUtils->Nest2DStrategy->EvaluateNestResult(ANestItems, BestLayers);
            std::cout << "================ POLYGON BEST NEST RESULT ================" << std::endl;
            std::cout << "[POLYGON BEST] bin0 count = " << BestEval.FirstBinCount << ", bin0 area = " << BestEval.FirstBinArea << ", layers = " << BestLayers << std::endl;
            Nest2DUtils->Nest2DStrategy->PrintBinCount(ANestItems);
            std::cout << "===========================================================" << std::endl;
            return BestLayers;
        }
        std::size_t CetNest2DEngine::RunPolygonNestOnce(CetTNestItemVector &ATestItems, const TetNestOptions &AOptions, TetNestProgressTracker &ATracker)
        {
            double BoardBinWidth = AOptions.BinWidth;
            double BoardBinHeight = AOptions.BinHeight;
            CetPolygonImpl BinPoly = Nest2DUtils->Nest2DBord->BuildBinPolygonFromOptions(AOptions, BoardBinWidth, BoardBinHeight);
            using CetMyPlacer = placers::_NofitPolyPlacer<CetPolygonImpl, CetPolygonImpl>;
            // Keep the primary ordering stable; expanded items are backfilled after nesting.
            using CetMySelector = selections::_FirstFitSelection<CetPolygonImpl>;
            NestConfig<CetMyPlacer, CetMySelector> cfg;
            cfg.placer_config.alignment = placers::NfpPConfig<CetPolygonImpl>::Alignment::DONT_ALIGN;
            cfg.placer_config.starting_point = placers::NfpPConfig<CetPolygonImpl>::Alignment::BOTTOM_LEFT;
            cfg.placer_config.accuracy = 1.0f;
            cfg.placer_config.parallel = true;
            cfg.placer_config.explore_holes = false;
            FillRotations(cfg.placer_config.rotations, AOptions.Rotations);
            std::cout << "================ POLYGON ONCE DEBUG ================" << std::endl;
            std::cout << "UsePolygonBoard: true" << std::endl;
            std::cout << "BoardBinWidth: " << BoardBinWidth << ", BoardBinHeight: " << BoardBinHeight << std::endl;
            std::cout << "Spacing: " << NestUtils::ToNestCoord(AOptions.Spacing) << std::endl;
            std::cout << "Board.Vertices.size: " << AOptions.Board.Vertices.size() << std::endl;
            std::cout << "====================================================" << std::endl;
            std::size_t Layers = nest(ATestItems, BinPoly, NestUtils::ToNestCoord(AOptions.Spacing), cfg, ProgressFunction{ATracker});
            std::cout << "[POLYGON ONCE] before repair, Layers = " << Layers << std::endl;
            Nest2DUtils->Nest2DPolygonBord->SetContext(ATestItems, AOptions, BinPoly, BoardBinWidth, BoardBinHeight);
            Nest2DUtils->Nest2DPolygonBord->Repair(Layers);
            std::cout << "[POLYGON ONCE] after repair, Layers = " << Layers << std::endl;
            Nest2DUtils->Nest2DStrategy->PrintBinCount(ATestItems);
            return Layers;
        }
        std::size_t CetNest2DEngine::RunRectangleBoardNesting(CetTNestItemVector &ANestItems, const TetNestOptions &AOptions, TetNestProgressTracker &ATracker)
        {
            std::cout << "[NEST] use original rectangle BIN" << std::endl;
            CetTNestItemVector OriginalItems = ANestItems;
            // CetShapeAnalyzer ShapeAnalyzer;
            const std::vector<TetShapeFeature> Features = Nest2DUtils->Nest2DShape->AnalyzeALL(OriginalItems);
            std::cout << "[SHAPE ANALYZER][DONE]" << " ItemCount = " << OriginalItems.size() << ", FeatureCount = " << Features.size() << std::endl;
            bool HasBest = false;
            CetTNestItemVector BestItems;
            TetTNestEvalResult BestEval{};
            std::size_t BestLayers = 0;
            std::vector<TetMetaItem> BestMetaItems;
            bool BestHasCluster = false;
            bool BestHasLockedEnvelope = false;
            const std::vector<MetClusterStrategy> ClusterStrategies = BuildClusterStrategies(Features);
            for (auto ClusterStrategy : ClusterStrategies) {
                //	TetClusterBuildResult ClusterResult = Nest2DUtils->Nest2DCluster->BuildClusterItems(OriginalItems, AOptions, ClusterStrategy);
                TetClusterBuildResult ClusterResult = Nest2DUtils->Nest2DCluster->BuildClusterItemsWithFeatures(OriginalItems, Features, AOptions, ClusterStrategy);
                int ClusterCount = 0;
                for (const auto &Meta : ClusterResult.MetaItems) {
                    if (Meta.IsCluster)
                        ClusterCount++;
                }
                std::cout << "[CLUSTER][BUILD] Strategy = " << static_cast<int>(ClusterStrategy) << ", OriginalItems = " << OriginalItems.size() << ", PackedItems = " << ClusterResult.NestItems.size() << ", MetaItems = " << ClusterResult.MetaItems.size() << ", ClusterCount = " << ClusterCount << std::endl;
                TetExpandedSpacingFailure SpacingFailure;
                TetLocalBestResult LocalResult = EvaluateSortingStrategies(ClusterResult, OriginalItems, AOptions, ATracker, &SpacingFailure);
                if (!LocalResult.HasBest && SpacingFailure.Valid) {
                    LocalResult = _TryLocalClusterSpacingFallback(ClusterResult, OriginalItems, AOptions, ATracker, SpacingFailure);
                }
                const bool LocalHasLockedEnvelope = HasLockedEnvelopeCluster(LocalResult.MetaItems);
                bool Better = ShoouldUpdateGlobalBest(LocalResult, HasBest, BestEval, BestLayers, BestHasCluster);
                if (!Better && LocalHasLockedEnvelope && !BestHasLockedEnvelope) {
                    // Keep the fixed outline intact only when it does not regress the
                    // evaluated layout; the protection is not a selection score.
                    Better = !Nest2DUtils->Nest2DStrategy->IsBetterNestResult(BestEval, LocalResult.Eval);
                }
                if (Better) {
                    HasBest = true;
                    BestEval = LocalResult.Eval;
                    BestLayers = LocalResult.Layers;
                    BestItems = std::move(LocalResult.Items);
                    BestMetaItems = std::move(LocalResult.MetaItems);
                    BestHasCluster = LocalResult.HasCluster;
                    BestHasLockedEnvelope = LocalHasLockedEnvelope;
                    if (BestHasLockedEnvelope) {
                        std::cout << "[NEST][LOCKED ENVELOPE] Preserve completed envelope-fill composite." << std::endl;
                    }
                    std::cout << "[NEST][GLOBAL BEST UPDATE] HasCluster = " << BestHasCluster << ", count = " << BestEval.FirstBinCount << ", area = " << BestEval.FirstBinArea << ", layers = " << BestEval.Layers << ", packedItems = " << BestItems.size() << std::endl;
                }
            }
            if (!HasBest) {
                std::cout << "[NEST][SPACING FALLBACK][FULL SINGLE] Trigger=all clustered candidates rejected." << std::endl;
                TetClusterBuildResult SingleItems = Nest2DUtils->Nest2DCluster->BuildClusterItems(OriginalItems, AOptions, MetClusterStrategy::None);
                TetLocalBestResult FallbackResult = _EvaluateSingleSortingStrategy(SingleItems, OriginalItems, AOptions, ATracker, MetENestOrderStrategy::LargeFirst);
                if (FallbackResult.HasBest) {
                    HasBest = true;
                    BestEval = FallbackResult.Eval;
                    BestLayers = FallbackResult.Layers;
                    BestItems = std::move(FallbackResult.Items);
                    BestMetaItems = std::move(FallbackResult.MetaItems);
                    BestHasCluster = false;
                    std::cout << "[NEST][SPACING FALLBACK][FULL SINGLE][VALID] Layers=" << BestLayers << std::endl;
                }
            }
            if (HasBest)
                _FinalizeRectangleBest({OriginalItems, BestItems, BestMetaItems, AOptions, ATracker, BestHasCluster, BestHasLockedEnvelope, BestEval, BestLayers, ANestItems});
            std::cout << "================ BEST NEST RESULT ================" << std::endl;
            std::cout << "[NEST BEST] bin0 count = " << BestEval.FirstBinCount << ", bin0 area = " << BestEval.FirstBinArea << ", layers = " << BestEval.Layers << ", internal gap area = " << BestEval.InternalGapArea << ", internal gap count = " << BestEval.InternalGapCount << std::endl;
            Nest2DUtils->Nest2DStrategy->PrintBinCount(ANestItems);
            std::cout << "==================================================" << std::endl;
            return BestLayers;
        }
        void CetNest2DEngine::_FinalizeRectangleBest(const TetRectangleBestFinalizeRequest &ARequest)
        {
            std::cout << "[NEST][FINAL BEST] BestHasCluster = " << ARequest.BestHasCluster << ", BestItems.size = " << ARequest.BestItems.size() << ", BestMetaItems.size = " << ARequest.BestMetaItems.size() << std::endl;
            std::cout << "[NEST][FINAL BEST] " << (ARequest.BestHasCluster ? "Use cluster expand." : "Restore normal item order.") << std::endl;
            const long double CoordinateScale = NestUtils::NestScale();
            std::cout << "[NEST][FINAL REMNANT] MetricsAvailable=" << ARequest.BestEval.HasRemnantMetrics << ", AreaMm2=" << static_cast<double>(ARequest.BestEval.ReusableRemnantArea / (CoordinateScale * CoordinateScale)) << ", ShortSideMm=" << static_cast<double>(ARequest.BestEval.ReusableRemnantShortSide / CoordinateScale) << ", SkylineWasteMm2=" << static_cast<double>(ARequest.BestEval.SkylineWasteArea / (CoordinateScale * CoordinateScale)) << ", UsedDepthMm=" << static_cast<double>(ARequest.BestEval.UsedDepth / CoordinateScale) << ", Direction=" << (ARequest.BestEval.RemnantIsTopStrip ? "Top" : "Right") << std::endl;
            const CetTNestItemVector ItemsBeforeBackfill = ARequest.BestItems;
            const std::size_t LayersBeforeBackfill = ARequest.BestLayers;
            ARequest.BestLayers = BackfillClusterSheets(ARequest.BestItems, ARequest.Options, ARequest.BestLayers);
            if (!Nest2DUtils->Nest2DCluster->ValidatePackedResultSpacing(ARequest.OriginalItems, ARequest.BestItems, ARequest.BestMetaItems, ARequest.Options)) {
                std::cout << "[NEST][CLUSTER BACKFILL][ROLLBACK] Expanded validation failed." << std::endl;
                ARequest.BestItems = ItemsBeforeBackfill;
                ARequest.BestLayers = LayersBeforeBackfill;
            } else {
                ARequest.BestEval = Nest2DUtils->Nest2DStrategy->EvaluatePackedResultWithMeta(ARequest.BestItems, ARequest.BestMetaItems, ARequest.OriginalItems, ARequest.Options, ARequest.BestLayers);
                std::cout << "[NEST][CLUSTER BACKFILL][VALID] FirstBinCount=" << ARequest.BestEval.FirstBinCount << ", Layers=" << ARequest.BestEval.Layers << std::endl;
            }
            Nest2DUtils->Nest2DCluster->ExpandClusterResultToOriginalItems(ARequest.OriginalItems, ARequest.BestItems, ARequest.BestMetaItems, ARequest.OutNestItems);
            CetPolygonImpl RectBinPoly = Nest2DUtils->Nest2DBord->BuildRectangleBinPolygon(ARequest.Options.BinWidth, ARequest.Options.BinHeight);
            if (!ARequest.BestHasLockedEnvelope && _RepairAndEvacuate(ARequest.OutNestItems, ARequest.Options, RectBinPoly, ARequest.Options.BinWidth, ARequest.Options.BinHeight, ARequest.BestLayers)) {
                _TryBoardFeedbackNest(ARequest.OutNestItems, ARequest.Options, ARequest.Tracker, ARequest.BestLayers);
            } else if (ARequest.BestHasLockedEnvelope) {
                const CetTNestItemVector BeforeEvacuation = ARequest.OutNestItems;
                const std::size_t LayersBeforeEvacuation = ARequest.BestLayers;
                const std::vector<std::size_t> LockedChildren = CollectLockedEnvelopeChildren(ARequest.BestMetaItems);
                if (!_TryLockedEnvelopeBoardRepair({ARequest.OutNestItems, ARequest.Options, RectBinPoly, ARequest.Options.BinWidth, ARequest.Options.BinHeight, LockedChildren, ARequest.BestLayers})) {
                    TetNestOptions EvacuationOptions = ARequest.Options;
                    EvacuationOptions.EnableLastBinEvacuation = true;
                    if (!_RunLastBinEvacuation(ARequest.OutNestItems, EvacuationOptions, ARequest.BestLayers) || !PreservesLockedChildren(BeforeEvacuation, ARequest.OutNestItems, LockedChildren)) {
                        ARequest.OutNestItems = BeforeEvacuation;
                        ARequest.BestLayers = LayersBeforeEvacuation;
                    }
                }
            }
            Nest2DUtils->Nest2dRectangleGridOptimizer->TryCompactUniformRectangleHoles(ARequest.OutNestItems, ARequest.Options);
            Nest2DUtils->Nest2dRectangleGridOptimizer->TryFillRectangleGridEdgeFromCompatibleGroup(ARequest.OutNestItems, ARequest.Options);
            if (!ARequest.BestHasCluster)
                Nest2DUtils->Nest2dLocalCompactor->RunLocalCompactPass(ARequest.OutNestItems, ARequest.Options, &ARequest.BestMetaItems);
            ARequest.BestEval = Nest2DUtils->Nest2DStrategy->EvaluateNestResult(ARequest.OutNestItems, ARequest.BestLayers);
            Nest2DUtils->Nest2dRectangleGridOptimizer->EvaluateInternalGapMetrics(ARequest.OutNestItems, ARequest.Options, ARequest.BestEval);
        }
        std::size_t CetNest2DEngine::RunRectangleNestOnce(CetTNestItemVector &ATestItems, const TetNestOptions &AOptions, TetNestProgressTracker &ATracker, bool AUseFillerSelector, bool AAllowRotations)
        {
            if (AUseFillerSelector) {
                return RunRectangleNestWithSelector<selections::_FillerSelection<CetPolygonImpl>>(ATestItems, AOptions, ATracker, AAllowRotations);
            }
            return RunRectangleNestWithSelector<selections::_FirstFitSelection<CetPolygonImpl>>(ATestItems, AOptions, ATracker, AAllowRotations);
        }
        bool CetNest2DEngine::_HasClusterItems(const std::vector<TetMetaItem> &AMetaItems) const
        {
            return std::any_of(AMetaItems.begin(), AMetaItems.end(), [](const TetMetaItem &AMeta) { return AMeta.IsCluster; });
        }
        std::vector<std::size_t> CetNest2DEngine::_BuildPriorityOrder(CetTNestItemVector &AItems, const TetNestOptions &AOptions, MetENestOrderStrategy AStrategy) const
        {
            Nest2DUtils->Nest2DStrategy->ApplyNestPriorityStrategy(AItems, AOptions, AStrategy);
            std::vector<std::size_t> Indices(AItems.size());
            std::iota(Indices.begin(), Indices.end(), 0);
            std::stable_sort(Indices.begin(), Indices.end(), [&](std::size_t A, std::size_t AB) {
                const int PriorityA = AItems[A].priority();
                const int PriorityB = AItems[AB].priority();
                if (PriorityA != PriorityB)
                    return PriorityA > PriorityB;
                const double AreaA = std::abs(static_cast<double>(AItems[A].area()));
                const double AreaB = std::abs(static_cast<double>(AItems[AB].area()));
                return std::abs(AreaA - AreaB) > 1e-6 ? AreaA > AreaB : A < AB;
            });
            return Indices;
        }
        void CetNest2DEngine::_BuildSortedTestData(CetTNestItemVector &APriorityItems, const std::vector<TetMetaItem> &AMetaItems, const std::vector<std::size_t> &ASortedIndices, CetTNestItemVector &AOutItems, std::vector<TetMetaItem> &AOutMetaItems) const
        {
            AOutItems.reserve(APriorityItems.size());
            AOutMetaItems.reserve(AMetaItems.size());
            for (std::size_t Index : ASortedIndices) {
                AOutItems.push_back(std::move(APriorityItems[Index]));
                AOutMetaItems.push_back(AMetaItems[Index]);
                AOutMetaItems.back().PackedItemIndex = static_cast<int>(AOutMetaItems.size() - 1);
            }
        }
        void CetNest2DEngine::_UpdateLocalBest(TetLocalBestResult &ALocalBest, TetTNestEvalResult AEvaluation, std::size_t ALayers, CetTNestItemVector &AItems, std::vector<TetMetaItem> &AMetaItems, bool AHasCluster) const
        {
            bool Better = !ALocalBest.HasBest;
            if (!Better && Nest2DUtils->Nest2DStrategy->IsBetterNestResult(AEvaluation, ALocalBest.Eval))
                Better = true;
            if (!Better && AHasCluster && !ALocalBest.HasCluster)
                Better = !Nest2DUtils->Nest2DStrategy->IsBetterNestResult(ALocalBest.Eval, AEvaluation);
            if (!Better)
                return;
            ALocalBest.HasBest = true;
            ALocalBest.Eval = AEvaluation;
            ALocalBest.Layers = ALayers;
            ALocalBest.Items = std::move(AItems);
            ALocalBest.MetaItems = std::move(AMetaItems);
            ALocalBest.HasCluster = AHasCluster;
            std::cout << "[NEST][LOCAL BEST UPDATE] HasCluster = " << ALocalBest.HasCluster << ", count = " << ALocalBest.Eval.FirstBinCount << ", area = " << ALocalBest.Eval.FirstBinArea << ", layers = " << ALocalBest.Eval.Layers << ", packedItems = " << ALocalBest.Items.size() << std::endl;
        }
        void CetNest2DEngine::_TryQuarterTurnCandidates(TetLocalBestResult &ALocalBest, const CetTNestItemVector &AOriginalItems, const TetNestOptions &AOptions, TetNestProgressTracker &ATracker, bool AHasCluster)
        {
            if (!ALocalBest.HasBest || ALocalBest.Items.size() > CET_NEST_FULL_STRATEGY_ITEM_LIMIT || AOptions.Board.Enabled || !CetRotationUtils::IsAllowedRotation(CET_CLUSTER_HALF_PI, AOptions.Rotations, 1e-9))
                return;
            (void)ATracker;
            constexpr std::size_t MaxPlacementsPerTarget = 48;
            const auto StartTime = std::chrono::steady_clock::now();
            constexpr auto Budget = std::chrono::milliseconds(250);
            const auto BinWidth = NestUtils::ToNestCoord(AOptions.BinWidth), BinHeight = NestUtils::ToNestCoord(AOptions.BinHeight);
            const auto Spacing = NestUtils::ToNestCoord(std::max(0.0, AOptions.Spacing));
            const auto HalfSpacing = static_cast<libnest2d::Coord>(std::ceil(static_cast<double>(Spacing) * 0.5));
            if (BinWidth <= 0 || BinHeight <= 0) return;
            std::vector<TetQuarterTurnTarget> Targets = Nest2DUtils->Nest2dQuarterTurnOptimizer->CollectQuarterTurnTargets(ALocalBest.Items, ALocalBest.MetaItems, Spacing);
            std::set<std::size_t> AppliedTargets;
            std::size_t Pass = 0;
            while (AppliedTargets.size() < Targets.size()) {
                if (std::chrono::steady_clock::now() - StartTime >= Budget) break;
                ++Pass;
                const CetTNestItemVector PassBaseItems = ALocalBest.Items; const std::vector<TetMetaItem> PassBaseMetaItems = ALocalBest.MetaItems;
                const TetTNestEvalResult PassBaseEval = ALocalBest.Eval;
                CetTNestItemVector ExpandedBaseline;
                Nest2DUtils->Nest2DCluster->ExpandClusterResultToOriginalItems(AOriginalItems, PassBaseItems, PassBaseMetaItems, ExpandedBaseline, false);
                const TetAllBinRemnantMetric PassBaseRemnant = EvaluateAllBinRemnantMetric(ExpandedBaseline, AOptions, PassBaseEval.Layers);
                bool HasPassBest = false; std::size_t PassBestTarget = 0, PassBestLayers = 0;
                TetTNestEvalResult PassBestEval{};
                TetAllBinRemnantMetric PassBestRemnant = PassBaseRemnant;
                CetTNestItemVector PassBestItems;
                std::vector<TetMetaItem> PassBestMetaItems;
                for (const auto &Target : Targets) {
                    if (std::chrono::steady_clock::now() - StartTime >= Budget)
                        break;
                    const std::size_t TargetIndex = Target.Index;
                    if (AppliedTargets.find(TargetIndex) != AppliedTargets.end())
                        continue;
                    const std::string &ClusterType = PassBaseMetaItems[TargetIndex].ClusterType;
                    std::cout << "[NEST][QUARTER TURN CANDIDATE] Pass=" << Pass + 1 << ", Index=" << TargetIndex << ", Type=" << ClusterType << ", Aspect=" << Target.Score << std::endl;
                    CetNestItem RotatedTarget = PassBaseItems[TargetIndex];
                    RotatedTarget.inflation(0); RotatedTarget.rotation(Radians(static_cast<double>(RotatedTarget.rotation()) + CET_CLUSTER_HALF_PI));
                    const auto RotatedBounds = RotatedTarget.boundingBox();
                    const double RotatedWidth = static_cast<double>(RotatedBounds.width()), RotatedHeight = static_cast<double>(RotatedBounds.height());
                    if (RotatedWidth <= 0.0 || RotatedHeight <= 0.0 || RotatedWidth > static_cast<double>(BinWidth) || RotatedHeight > static_cast<double>(BinHeight))
                        continue;
                    // Moving a cluster to another already-used sheet may improve an aggregate
                    // score while consuming a clean remnant there. Cross-sheet relocation is
                    // reserved for the separate sheet-elimination pass.
                    const int CandidateBin = PassBaseItems[TargetIndex].binId();
                    if (CandidateBin < 0)
                        continue;
                    const TetLocalCompactEnvelope CandidateBaseEnvelope = Nest2DUtils->Nest2dLocalCompactor->CalculateEnvelope(PassBaseItems, CandidateBin);
                    std::size_t CheckedPlacements = 0, ValidPlacements = 0;
                    const TetQuarterTurnCoordinates Coordinates = Nest2DUtils->Nest2dQuarterTurnOptimizer->BuildQuarterTurnCoordinates({PassBaseItems, TargetIndex, CandidateBin, RotatedWidth, RotatedHeight, Spacing, BinWidth, BinHeight});
                    const std::vector<ClipperLib::cInt> &XCoordinates = Coordinates.X;
                    const std::vector<ClipperLib::cInt> &YCoordinates = Coordinates.Y;
                    for (ClipperLib::cInt MinY : YCoordinates) {
                        for (ClipperLib::cInt MinX : XCoordinates) {
                            if (CheckedPlacements >= MaxPlacementsPerTarget)
                                break;
                            ++CheckedPlacements;
                            CetTNestItemVector TestItems = PassBaseItems;
                            CetNestItem &TestTarget = TestItems[TargetIndex];
                            TestTarget.inflation(0); TestTarget.rotation(RotatedTarget.rotation()); TestTarget.binId(CandidateBin);
                            const auto Bounds = TestTarget.boundingBox();
                            const Point Translation = TestTarget.translation();
                            TestTarget.translation(Point(Translation.X + MinX - getX(Bounds.minCorner()), Translation.Y + MinY - getY(Bounds.minCorner())));
                            if (!Nest2DUtils->Nest2dQuarterTurnOptimizer->CanPlaceQuarterTurnTarget(TestItems, TargetIndex, BinWidth, BinHeight, HalfSpacing))
                                continue;
                            const TetLocalCompactEnvelope TestEnvelope = Nest2DUtils->Nest2dLocalCompactor->CalculateEnvelope(TestItems, CandidateBin);
                            const double EnvelopeTolerance = std::max(1.0, static_cast<double>(Spacing));
                            const bool BetterEnvelope = TestEnvelope.Valid && CandidateBaseEnvelope.Valid && (TestEnvelope.Area + EnvelopeTolerance * EnvelopeTolerance < CandidateBaseEnvelope.Area || (std::abs(TestEnvelope.Area - CandidateBaseEnvelope.Area) <= EnvelopeTolerance * EnvelopeTolerance && TestEnvelope.LongSide + EnvelopeTolerance < CandidateBaseEnvelope.LongSide));
                            if (!BetterEnvelope)
                                continue;
                            ++ValidPlacements;
                            CetTNestItemVector ExpandedItems;
                            Nest2DUtils->Nest2DCluster->ExpandClusterResultToOriginalItems(AOriginalItems, TestItems, PassBaseMetaItems, ExpandedItems, false);
                            const TetAllBinRemnantMetric Remnant = EvaluateAllBinRemnantMetric(ExpandedItems, AOptions, PassBaseEval.Layers);
                            TetExpandedSpacingFailure Failure;
                            if (AHasCluster && !Nest2DUtils->Nest2DCluster->ValidatePackedResultSpacing(AOriginalItems, TestItems, PassBaseMetaItems, AOptions, &Failure))
                                continue;
                            TetTNestEvalResult Eval = Nest2DUtils->Nest2DStrategy->EvaluatePackedResultWithMeta(TestItems, PassBaseMetaItems, AOriginalItems, AOptions, PassBaseEval.Layers);
                            Nest2DUtils->Nest2dRectangleGridOptimizer->EvaluateInternalGapMetrics(ExpandedItems, AOptions, Eval);
                            Nest2DUtils->Nest2dFreeSpaceEvaluator->EvaluateBoardFreeRegionMetrics(ExpandedItems, AOptions, Eval);
                            Nest2DUtils->Nest2dFreeSpaceEvaluator->EvaluatePassableFreeRegionMetrics(ExpandedItems, AOptions, Eval);
                            const TetTNestEvalResult &Comparison = HasPassBest ? PassBestEval : PassBaseEval;
                            if (HasPassBest && !Nest2DUtils->Nest2dFreeSpaceEvaluator->IsBetterContinuousFreeSpace(Eval, Comparison))
                                continue;
                            HasPassBest = true; PassBestTarget = TargetIndex; PassBestLayers = PassBaseEval.Layers;
                            PassBestEval = std::move(Eval);
                            PassBestRemnant = Remnant;
                            PassBestItems = std::move(TestItems);
                            PassBestMetaItems = PassBaseMetaItems;
                        }
                    }
                    std::cout << "[NEST][QUARTER TURN LOCAL EVAL] Type=" << ClusterType << ", Bin=" << CandidateBin << ", Checked=" << CheckedPlacements << ", Valid=" << ValidPlacements << ", Improved=" << (HasPassBest && PassBestTarget == TargetIndex ? 1 : 0) << std::endl;
                }
                if (!HasPassBest)
                    break;
                AppliedTargets.insert(PassBestTarget); Nest2DUtils->Nest2dQuarterTurnOptimizer->ApplyQuarterTurnPassBest(ALocalBest, std::move(PassBestEval), PassBestLayers, std::move(PassBestItems), std::move(PassBestMetaItems), AHasCluster);
                std::cout << "[NEST][QUARTER TURN LOCAL ACCEPT] Pass=" << Pass + 1 << ", Index=" << PassBestTarget << ", Type=" << ALocalBest.MetaItems[PassBestTarget].ClusterType << ", ReusableStrip=" << PassBestRemnant.ReusableStripArea << ", SkylineWaste=" << PassBestRemnant.SkylineWasteArea << std::endl;
            }
        }
        void CetNest2DEngine::_TryOppositeEdgeCandidate(TetLocalBestResult &ALocalBest, const TetClusterBuildResult &AClusterResult, const CetTNestItemVector &AOriginalItems, const TetNestOptions &AOptions, TetNestProgressTracker &ATracker, bool AHasCluster)
        {
            if (!ALocalBest.HasBest || AClusterResult.NestItems.size() > CET_NEST_FULL_STRATEGY_ITEM_LIMIT || AClusterResult.NestItems.size() != AClusterResult.MetaItems.size())
                return;
            const TetTNestEvalResult BaselineEval = ALocalBest.Eval;
            const std::array<MetENestOrderStrategy, 2> Strategies{MetENestOrderStrategy::LargeFirst, MetENestOrderStrategy::AreaDensityFirst};
            for (const MetENestOrderStrategy Strategy : Strategies) {
                CetTNestItemVector PriorityItems = AClusterResult.NestItems;
                const std::vector<std::size_t> Order = _BuildPriorityOrder(PriorityItems, AOptions, Strategy);
                CetTNestItemVector TestItems;
                std::vector<TetMetaItem> TestMetaItems;
                _BuildSortedTestData(PriorityItems, AClusterResult.MetaItems, Order, TestItems, TestMetaItems);
                ApplyClusterEdgeClearance(TestItems, TestMetaItems, AOptions);
                const std::size_t Layers = RunRectangleNestFromOppositeEdge(TestItems, AOptions, ATracker);
                ClearItemInflation(TestItems);
                if (Layers == 0)
                    continue;
                TetExpandedSpacingFailure Failure;
                if (AHasCluster && !Nest2DUtils->Nest2DCluster->ValidatePackedResultSpacing(AOriginalItems, TestItems, TestMetaItems, AOptions, &Failure)) {
                    std::cout << "[NEST][OPPOSITE EDGE EVAL][REJECT] Strategy=" << static_cast<int>(Strategy) << ", reason=expanded cluster spacing violation" << std::endl;
                    continue;
                }
                TetTNestEvalResult Eval = Nest2DUtils->Nest2DStrategy->EvaluatePackedResultWithMeta(TestItems, TestMetaItems, AOriginalItems, AOptions, Layers);
                CetTNestItemVector ExpandedItems;
                Nest2DUtils->Nest2DCluster->ExpandClusterResultToOriginalItems(AOriginalItems, TestItems, TestMetaItems, ExpandedItems, false);
                Nest2DUtils->Nest2dRectangleGridOptimizer->EvaluateInternalGapMetrics(ExpandedItems, AOptions, Eval);
                Nest2DUtils->Nest2dFreeSpaceEvaluator->EvaluateBoardFreeRegionMetrics(ExpandedItems, AOptions, Eval);
                Nest2DUtils->Nest2dFreeSpaceEvaluator->EvaluatePassableFreeRegionMetrics(ExpandedItems, AOptions, Eval);
                bool PreservesBoardUsage = Eval.BinAreas.size() == BaselineEval.BinAreas.size();
                for (std::size_t Bin = 0; PreservesBoardUsage && Bin < Eval.BinAreas.size(); ++Bin) {
                    PreservesBoardUsage = Eval.BinAreas[Bin] + 1.0 >= BaselineEval.BinAreas[Bin];
                }
                const bool PreservesFreeSpace = Nest2DUtils->Nest2dFreeSpaceEvaluator->PreservesPassableFreeSpace(Eval, BaselineEval);
                if (!PreservesBoardUsage || !PreservesFreeSpace) {
                    std::cout << "[NEST][OPPOSITE EDGE EVAL][REJECT] Strategy=" << static_cast<int>(Strategy) << ", BoardUsage=" << PreservesBoardUsage << ", PassableFreeSpace=" << PreservesFreeSpace << std::endl;
                    continue;
                }
                std::cout << "[NEST][OPPOSITE EDGE EVAL] Strategy=" << static_cast<int>(Strategy) << ", FirstBinArea=" << Eval.FirstBinArea << ", Layers=" << Eval.Layers << ", PassableRegions=" << Eval.PassableFreeRegionCount << std::endl;
                _UpdateLocalBest(ALocalBest, Eval, Layers, TestItems, TestMetaItems, AHasCluster);
            }
        }
        TetLocalBestResult CetNest2DEngine::_EvaluateSingleSortingStrategy(const TetClusterBuildResult &AClusterResult, const CetTNestItemVector &AOriginalItems, const TetNestOptions &AOptions, TetNestProgressTracker &ATracker, MetENestOrderStrategy AStrategy, TetExpandedSpacingFailure *AOutSpacingFailure)
        {
            TetLocalBestResult LocalBest;
            if (AOutSpacingFailure != nullptr) {
                *AOutSpacingFailure = TetExpandedSpacingFailure{};
            }
            if (AClusterResult.NestItems.size() != AClusterResult.MetaItems.size()) {
                return LocalBest;
            }
            const bool UsePolygonBoard = AOptions.Board.Enabled && AOptions.Board.Vertices.size() >= 3;
            const bool HasCluster = _HasClusterItems(AClusterResult.MetaItems);
            CetTNestItemVector PriorityItems = AClusterResult.NestItems;
            const std::vector<std::size_t> SortedIndices = _BuildPriorityOrder(PriorityItems, AOptions, AStrategy);
            CetTNestItemVector TestItems;
            std::vector<TetMetaItem> TestMetaItems;
            _BuildSortedTestData(PriorityItems, AClusterResult.MetaItems, SortedIndices, TestItems, TestMetaItems);
            const std::size_t Layers = UsePolygonBoard ? RunPolygonNestOnce(TestItems, AOptions, ATracker) : RunRectangleNestOnce(TestItems, AOptions, ATracker);
            if (Layers == 0) {
                std::cout << "[NEST][SPACING FALLBACK][SKIP] Strategy=" << static_cast<int>(AStrategy) << ", reason=no packed layers" << std::endl;
                return LocalBest;
            }
            if (HasCluster && !Nest2DUtils->Nest2DCluster->ValidatePackedResultSpacing(AOriginalItems, TestItems, TestMetaItems, AOptions, AOutSpacingFailure)) {
                std::cout << "[NEST][SPACING FALLBACK][SKIP] Strategy=" << static_cast<int>(AStrategy) << ", reason=expanded cluster spacing violation" << std::endl;
                return LocalBest;
            }
            TetTNestEvalResult Eval = Nest2DUtils->Nest2DStrategy->EvaluatePackedResultWithMeta(TestItems, TestMetaItems, AOriginalItems, AOptions, Layers);
            CetTNestItemVector ExpandedItems;
            Nest2DUtils->Nest2DCluster->ExpandClusterResultToOriginalItems(AOriginalItems, TestItems, TestMetaItems, ExpandedItems, false);
            Nest2DUtils->Nest2dRectangleGridOptimizer->EvaluateInternalGapMetrics(ExpandedItems, AOptions, Eval);
            Nest2DUtils->Nest2dFreeSpaceEvaluator->EvaluateBoardFreeRegionMetrics(ExpandedItems, AOptions, Eval);
            Nest2DUtils->Nest2dFreeSpaceEvaluator->EvaluatePassableFreeRegionMetrics(ExpandedItems, AOptions, Eval);
            _UpdateLocalBest(LocalBest, Eval, Layers, TestItems, TestMetaItems, HasCluster);
            return LocalBest;
        }
        TetLocalBestResult CetNest2DEngine::_TryLocalClusterSpacingFallback(const TetClusterBuildResult &AClusterResult, const CetTNestItemVector &AOriginalItems, const TetNestOptions &AOptions, TetNestProgressTracker &ATracker, const TetExpandedSpacingFailure &AInitialFailure)
        {
            TetLocalBestResult NoResult;
            if (!AInitialFailure.Valid || AClusterResult.NestItems.size() != AClusterResult.MetaItems.size()) {
                return NoResult;
            }
            std::set<int> PackedIndices;
            TetExpandedSpacingFailure Failure = AInitialFailure;
            constexpr int MaxLocalRetries = 2;
            for (int Attempt = 0; Attempt < MaxLocalRetries && Failure.Valid; ++Attempt) {
                const std::size_t ClusterCountBefore = PackedIndices.size();
                for (std::size_t PackedIndex = 0; PackedIndex < AClusterResult.MetaItems.size(); ++PackedIndex) {
                    const TetMetaItem &Meta = AClusterResult.MetaItems[PackedIndex];
                    if (!Meta.IsCluster) {
                        continue;
                    }
                    for (const TetItemTransform &Transform : Meta.TransformData) {
                        if (Transform.OriginalId == Failure.FirstOriginalIndex || Transform.OriginalId == Failure.SecondOriginalIndex) {
                            PackedIndices.insert(static_cast<int>(PackedIndex));
                            break;
                        }
                    }
                }
                if (PackedIndices.size() == ClusterCountBefore) {
                    std::cout << "[NEST][SPACING FALLBACK][LOCAL][STOP] Attempt=" << Attempt + 1 << ", reason=no additional conflicting cluster." << std::endl;
                    break;
                }
                std::cout << "[NEST][SPACING FALLBACK][LOCAL] Attempt=" << Attempt + 1 << ", OriginalPair=" << Failure.FirstOriginalIndex << "," << Failure.SecondOriginalIndex << ", DissolvedClusters=";
                for (int PackedIndex : PackedIndices)
                    std::cout << PackedIndex << " ";
                std::cout << ", RawOverlap=" << (Failure.RawContoursIntersect ? 1 : 0) << std::endl;
                const TetClusterBuildResult DissolvedResult = DissolvePackedClusters(AOriginalItems, AClusterResult, PackedIndices);
                TetExpandedSpacingFailure RetryFailure;
                TetLocalBestResult LocalResult = _EvaluateSingleSortingStrategy(DissolvedResult, AOriginalItems, AOptions, ATracker, MetENestOrderStrategy::LargeFirst, &RetryFailure);
                if (LocalResult.HasBest) {
                    std::cout << "[NEST][SPACING FALLBACK][LOCAL][VALID] Attempt=" << Attempt + 1 << ", Layers=" << LocalResult.Layers << ", PackedItems=" << LocalResult.Items.size() << std::endl;
                    return LocalResult;
                }
                Failure = RetryFailure;
            }
            std::cout << "[NEST][SPACING FALLBACK][LOCAL][FAILED] Attempts=" << MaxLocalRetries << std::endl;
            return NoResult;
        }
        TetLocalBestResult CetNest2DEngine::EvaluateSortingStrategies(const TetClusterBuildResult &AClusterResult, const CetTNestItemVector &AOriginalItems, const TetNestOptions &AOptions, TetNestProgressTracker &ATracker, TetExpandedSpacingFailure *AOutSpacingFailure)
        {
            TetLocalBestResult LocalBest;
            if (AOutSpacingFailure != nullptr) {
                *AOutSpacingFailure = TetExpandedSpacingFailure{};
            }
            if (AClusterResult.NestItems.size() != AClusterResult.MetaItems.size()) {
                std::cout << "[NEST][EVAL][ERROR] Cluster NestItems size != MetaItems size. NestItems = " << AClusterResult.NestItems.size() << ", MetaItems = " << AClusterResult.MetaItems.size() << std::endl;
                return LocalBest;
            }
            const bool UsePolygonBoard = AOptions.Board.Enabled && AOptions.Board.Vertices.size() >= 3;
            const bool CurrentHasCluster = _HasClusterItems(AClusterResult.MetaItems);
            std::vector<MetENestOrderStrategy> Strategies;
            if (AClusterResult.NestItems.size() > CET_NEST_REDUCED_STRATEGY_ITEM_LIMIT) {
                Strategies = {MetENestOrderStrategy::LargeFirst, MetENestOrderStrategy::AreaDensityFirst};
            } else if (AClusterResult.NestItems.size() > CET_NEST_FULL_STRATEGY_ITEM_LIMIT) {
                Strategies = {MetENestOrderStrategy::LargeFirst, MetENestOrderStrategy::AreaDensityFirst, MetENestOrderStrategy::LongSideFirst};
            } else {
                Strategies = {MetENestOrderStrategy::LargeFirst, MetENestOrderStrategy::AreaDensityFirst, MetENestOrderStrategy::SmallFirst, MetENestOrderStrategy::LongSideFirst, MetENestOrderStrategy::ThinFirst};
            }
            std::set<std::vector<std::size_t>> EvaluatedOrders;
            for (MetENestOrderStrategy Strategy : Strategies) {
                CetTNestItemVector PriorityItems = AClusterResult.NestItems;
                const std::vector<std::size_t> SortedIndices = _BuildPriorityOrder(PriorityItems, AOptions, Strategy);
                if (!EvaluatedOrders.insert(SortedIndices).second) {
                    std::cout << "[NEST][EVAL][SKIP] Strategy = " << static_cast<int>(Strategy) << ", reason = duplicate item order" << std::endl;
                    continue;
                }
                CetTNestItemVector TestItems;
                std::vector<TetMetaItem> TestMetaItems;
                _BuildSortedTestData(PriorityItems, AClusterResult.MetaItems, SortedIndices, TestItems, TestMetaItems);
                const std::size_t Layers = UsePolygonBoard ? RunPolygonNestOnce(TestItems, AOptions, ATracker) : RunRectangleNestOnce(TestItems, AOptions, ATracker);
                if (Layers == 0) {
                    std::cout << "[NEST][EVAL][SKIP] Strategy = " << static_cast<int>(Strategy) << ", reason = no packed layers" << std::endl;
                    continue;
                }
                TetExpandedSpacingFailure SpacingFailure;
                if (CurrentHasCluster && !Nest2DUtils->Nest2DCluster->ValidatePackedResultSpacing(AOriginalItems, TestItems, TestMetaItems, AOptions, &SpacingFailure)) {
                    if (AOutSpacingFailure != nullptr && !AOutSpacingFailure->Valid) {
                        *AOutSpacingFailure = SpacingFailure;
                    }
                    std::cout << "[NEST][EVAL][SKIP] Strategy = " << static_cast<int>(Strategy) << ", reason = expanded cluster spacing violation" << std::endl;
                    continue;
                }
                TetTNestEvalResult Eval = Nest2DUtils->Nest2DStrategy->EvaluatePackedResultWithMeta(TestItems, TestMetaItems, AOriginalItems, AOptions, Layers);
                CetTNestItemVector ExpandedItems;
                Nest2DUtils->Nest2DCluster->ExpandClusterResultToOriginalItems(AOriginalItems, TestItems, TestMetaItems, ExpandedItems, false);
                Nest2DUtils->Nest2dRectangleGridOptimizer->EvaluateInternalGapMetrics(ExpandedItems, AOptions, Eval);
                Nest2DUtils->Nest2dFreeSpaceEvaluator->EvaluateBoardFreeRegionMetrics(ExpandedItems, AOptions, Eval);
                Nest2DUtils->Nest2dFreeSpaceEvaluator->EvaluatePassableFreeRegionMetrics(ExpandedItems, AOptions, Eval);
                std::cout << "[NEST][EVAL] Strategy = " << static_cast<int>(Strategy) << ", HasCluster = " << CurrentHasCluster << ", Eval.FirstBinCount = " << Eval.FirstBinCount << ", Eval.FirstBinArea = " << Eval.FirstBinArea << ", Eval.Layers = " << Eval.Layers << ", Eval.InternalGapArea = " << Eval.InternalGapArea << ", Eval.InternalGapCount = " << Eval.InternalGapCount << ", Eval.FreeRegions = " << Eval.BoardFreeRegionCount << ", Eval.FragmentedFreeArea = " << Eval.FragmentedFreeArea << ", Eval.LargestFreeArea = " << Eval.LargestFreeRegionArea << ", Eval.PassableRegions = " << Eval.PassableFreeRegionCount << ", Eval.PassableFragmentedArea = " << Eval.FragmentedPassableFreeArea << ", Eval.LargestPassableArea = " << Eval.LargestPassableFreeRegionArea << ", Eval.PassableWidth = " << Eval.MinimumPassableWidth << ", Eval.RemnantArea = " << Eval.ReusableRemnantArea << ", Eval.RemnantShortSide = " << Eval.ReusableRemnantShortSide << ", Eval.SkylineWaste = " << Eval.SkylineWasteArea << ", Eval.RemnantDirection = " << (Eval.RemnantIsTopStrip ? "Top" : "Right") << ", LocalBest.FirstBinCount = " << LocalBest.Eval.FirstBinCount << ", LocalBest.FirstBinArea = " << LocalBest.Eval.FirstBinArea << ", LocalBest.Layers = " << LocalBest.Eval.Layers << std::endl;
                _UpdateLocalBest(LocalBest, Eval, Layers, TestItems, TestMetaItems, CurrentHasCluster);
                if (AOriginalItems.size() > CET_NEST_FULL_STRATEGY_ITEM_LIMIT && LocalBest.HasBest && LocalBest.Layers == 1) {
                    // One sheet is already the minimum possible. On large orders a
                    // second full NFP pass can cost minutes for only a secondary
                    // remnant-shape comparison; the clustered strategy is still
                    // evaluated separately and can replace this result.
                    std::cout << "[NEST][EVAL][SKIP REMAINING] OriginalCount=" << AOriginalItems.size() << ", PackedCount=" << AClusterResult.NestItems.size() << ", reason=one-sheet optimum at large-order limit" << std::endl;
                    break;
                }
            }
            if (!UsePolygonBoard) {
                if (AOptions.EnableLocalCompactPass && CurrentHasCluster) {
                    _TryQuarterTurnCandidates(LocalBest, AOriginalItems, AOptions, ATracker, CurrentHasCluster);
                }
                _TryOppositeEdgeCandidate(LocalBest, AClusterResult, AOriginalItems, AOptions, ATracker, CurrentHasCluster);
            }
            return LocalBest;
        }
        bool CetNest2DEngine::ShoouldUpdateGlobalBest(const TetLocalBestResult &ALocalResult, bool AHasBest, const TetTNestEvalResult &ABestEval, std::size_t ABestLayers, bool ABestHasCluster)
        {
            if (!ALocalResult.HasBest) {
                return false;
            }
            if (!AHasBest) {
                return true;
            }
            if (Nest2DUtils->Nest2DStrategy->IsBetterNestResult(ALocalResult.Eval, ABestEval)) {
                return true;
            }
            const bool Equivalent = !Nest2DUtils->Nest2DStrategy->IsBetterNestResult(ABestEval, ALocalResult.Eval);
            if (Equivalent) {
                if (ALocalResult.HasCluster && !ABestHasCluster) {
                    return true;
                }
            }
            return false;
        }
    } // namespace NEST2DMANAGERLIB
} // namespace ET
