
#include "pch.h"
#include "Nest2D_Engine.h"
#include "Nest2D_DataConst.h"
#include "Nest2D_PolygonBoardRepairer.h"
#include "Nest2D_PrivateDataType.h"
#include "Nest2D_RotationUtils.h"
#include "NestUtils.h"
#include "Nest2D_SelfFunction.h"
#include"Nest2D_ClusterManager.h"
#include"Nest2D_ShapeAnalyzer.h"
#include <map>
#include<vector>
#include<algorithm>
#include<limits>
#include<cmath>
#include<numeric>
#include<set>
#include<chrono>

//#include"libnest2d/optimizers/nlopt/subplex.hpp"

using namespace ClipperLib;
using namespace libnest2d;
namespace ET {
	namespace NEST2DMANAGERLIB {

		CetNest2DEngine::CetNest2DEngine() :CetCoreObject()
		{
		}
		CetNest2DEngine::~CetNest2DEngine()
		{
		}
		static void FillRotations(std::vector<libnest2d::Radians>& ARotations, int ARotationCount)
		{
			ARotations = CetRotationUtils::BuildAllowedLibRotations(ARotationCount);
		}
		static placers::NfpPConfig<CetPolygonImpl>::Alignment ToLibNestAlignment(MetNestAlignment AAlignment)
		{
			using CetAlignment = placers::NfpPConfig<CetPolygonImpl>::Alignment;

			switch (AAlignment){
			case MetNestAlignment::DontAlign:
				return CetAlignment::DONT_ALIGN;

			case MetNestAlignment::BottomLeft:
			default:
				return CetAlignment::BOTTOM_LEFT;
			}
		}

		static bool ValidatePlacedItemsSpacing(const CetTNestItemVector& AItems, const TetNestOptions& AOptions)
		{
			const auto SpacingCoord = NestUtils::ToNestCoord(std::max(0.0, AOptions.Spacing));
			for (std::size_t FirstIndex = 0; FirstIndex < AItems.size(); ++FirstIndex){
				const CetNestItem& SourceItem = AItems[FirstIndex];
				if (SourceItem.binId() < 0){
					std::cout << "[NEST][SPACING][REJECT] Item " << FirstIndex << " was not placed on a bin." << std::endl;
					return false;
				}
				CetNestItem FirstItem = SourceItem;
				FirstItem.inflation(0);
				if (SpacingCoord > 0){
					FirstItem.inflation(static_cast<decltype(FirstItem.inflation())>(std::ceil(static_cast<double>(SpacingCoord) * 0.5)));
				}
				for (std::size_t SecondIndex = FirstIndex + 1; SecondIndex < AItems.size(); ++SecondIndex){
					if (AItems[SecondIndex].binId() != SourceItem.binId()){
						continue;
					}
					CetNestItem SecondItem = AItems[SecondIndex];
					SecondItem.inflation(0);
					if (SpacingCoord > 0){
						SecondItem.inflation(static_cast<decltype(SecondItem.inflation())>(std::ceil(static_cast<double>(SpacingCoord) * 0.5)));
					}
					// libnest2d packs every item with half of the requested spacing.
					// At exactly the requested clearance those expanded outlines touch;
					// touching is legal, while an interior intersection is not.
					if (CetNestItem::intersects(FirstItem, SecondItem) && !CetNestItem::touches(FirstItem, SecondItem)){
						std::cout << "[NEST][SPACING][REJECT] "
							<< (SpacingCoord > 0 ? "Spacing violation" : "Overlap")
							<< " between items " << FirstIndex << " and " << SecondIndex
							<< " on bin " << SourceItem.binId() << std::endl;
						return false;
					}
				}
			}
			return true;
		}

		// Refill sheets with complete cluster proxies. Children stay glued through
		// their metadata and are expanded only after this multi-sheet pass.
		using ClusterBackfillPlacer = placers::_BottomLeftPlacer<CetPolygonImpl>;
		using ClusterBackfillConfig = placers::BLConfig<CetPolygonImpl>;

		bool RepackClusterItems(CetTNestItemVector& AItems, const std::vector<std::size_t>& AIndices, const Box& ABin, const ClusterBackfillConfig& AConfig, int ABinId)
		{
			CetTNestItemVector Repacked;
			Repacked.reserve(AIndices.size());
			for (std::size_t Index : AIndices){
				CetNestItem Copy = AItems[Index];
				Copy.translation(ClipperLib::IntPoint(0,0));
				Copy.rotation(0.0);
				Copy.inflation(0);
				Repacked.push_back(std::move(Copy));
			}
			ClusterBackfillPlacer Repacker(ABin);
			Repacker.configure(AConfig);
			for (CetNestItem& Item : Repacked){
				if (!Repacker.pack(Item)) return false;
				Item.binId(ABinId);
			}
			for (std::size_t Position = 0; Position < AIndices.size(); ++Position) AItems[AIndices[Position]] = std::move(Repacked[Position]);
			return true;
		}

		static std::size_t BackfillClusterSheets(CetTNestItemVector& AItems, const TetNestOptions& AOptions, std::size_t ALayers)
		{
			if (ALayers <= 1 || AItems.empty()) return ALayers;
			if (AItems.size() > CET_NEST_FULL_STRATEGY_ITEM_LIMIT) {
				// This optional repack repeatedly invokes the bottom-left placer. Once
				// local spacing fallback has expanded a large proxy set, doing it here
				// can cost minutes while not affecting the already valid main nest.
				std::cout << "[NEST][CLUSTER BACKFILL][SKIP] PackedItems=" << AItems.size()
					<< ", Limit=" << CET_NEST_FULL_STRATEGY_ITEM_LIMIT << std::endl;
				return ALayers;
			}
			const auto Width = NestUtils::ToNestCoord(AOptions.BinWidth);
			const auto Height = NestUtils::ToNestCoord(AOptions.BinHeight);
			Box Bin(Width, Height, { Width / 2, Height / 2 });
			placers::BLConfig<CetPolygonImpl> Config;
			Config.min_obj_distance = NestUtils::ToNestCoord(AOptions.Spacing);
			Config.epsilon = 1;
			Config.allow_rotations = CetRotationUtils::IsAllowedRotation(CET_CLUSTER_HALF_PI, AOptions.Rotations, 1e-9);
			std::set<int> AffectedBins;
			std::size_t Moved = 0;

			for (std::size_t Target = 0; Target + 1 < ALayers; ++Target){
				std::vector<std::size_t> TargetIndices;
				for (std::size_t Index = 0; Index < AItems.size(); ++Index){
					if (AItems[Index].binId() == static_cast<int>(Target)) TargetIndices.push_back(Index);
				}

				std::vector<std::size_t> Candidates;
				for (std::size_t Index = 0; Index < AItems.size(); ++Index){
					if (AItems[Index].binId() > static_cast<int>(Target)) Candidates.push_back(Index);
				}
				std::stable_sort(Candidates.begin(), Candidates.end(), [&](std::size_t A, std::size_t B) {
					return AItems[A].area() < AItems[B].area();
				});
				std::size_t Attempts = 0;
				for (std::size_t Index : Candidates){
					if (Attempts++ >= 32) break;
					if (AItems[Index].binId() <= static_cast<int>(Target)) continue;
					const int Source = AItems[Index].binId();
					std::vector<std::size_t> TrialIndices = TargetIndices;
					TrialIndices.push_back(Index);
					// Repack the target and candidate together while preserving glued proxies.
					if (!RepackClusterItems(AItems, TrialIndices, Bin, Config, static_cast<int>(Target))) continue;
					TargetIndices.push_back(Index);
					AffectedBins.insert(Source);
					++Moved;
				}
				std::cout << "[NEST][CLUSTER BACKFILL] Target=" << Target
					<< ", Attempts=" << Attempts << std::endl;
			}

			// Repack only source sheets affected by proxy moves. All proxies remain
			// intact, so their child spacing and glued geometry are preserved.
			for (int SourceBin : AffectedBins){
				std::vector<std::size_t> Indices;
				for (std::size_t Index = 0; Index < AItems.size(); ++Index){
					if (AItems[Index].binId() == SourceBin) Indices.push_back(Index);
				}
				const bool Success = RepackClusterItems(AItems, Indices, Bin, Config, SourceBin);
				if (Success){
					// RepackClusterItems has already written the packed items back.
				}
				std::cout << "[NEST][CLUSTER REPACK] Bin=" << SourceBin
					<< ", Items=" << Indices.size()
					<< ", Applied=" << (Success ? 1 : 0) << std::endl;
			}

			std::set<int> UsedBins;
			for (const CetNestItem& Item : AItems) if (Item.binId() >= 0) UsedBins.insert(Item.binId());
			std::map<int,int> Dense;
			int NextBin = 0;
			for (int BinId : UsedBins) Dense[BinId] = NextBin++;
			for (CetNestItem& Item : AItems){
				auto It = Dense.find(Item.binId());
				if (It != Dense.end()) Item.binId(It->second);
			}
			std::cout << "[NEST][CLUSTER BACKFILL SUMMARY] Moved=" << Moved
				<< ", LayersBefore=" << ALayers
				<< ", LayersAfter=" << UsedBins.size() << std::endl;
			return UsedBins.size();
		}

		static std::vector<MetClusterStrategy> BuildClusterStrategies(const std::vector<TetShapeFeature>& AFeatures)
		{
			if (AFeatures.size() > CET_NEST_FULL_STRATEGY_ITEM_LIMIT){
				// TemplateCluster preserves every unclustered item as a single and
				// validates full coverage. Avoid an additional full NFP pass over a
				// large original order before evaluating the reduced proxy set.
				return { MetClusterStrategy::TemplateCluster };
			}
			const std::size_t CustomShapeCount = static_cast<std::size_t>(std::count_if(AFeatures.begin(), AFeatures.end(), [](const TetShapeFeature& AFeature) {
				return AFeature.ShapeType == MetShapeType::QuadrilateralLike ||
					AFeature.ShapeType == MetShapeType::ConvexPolygon ||
					AFeature.ShapeType == MetShapeType::ConcavePolygon;
				}));
			const bool HasLargeCustomMajority = AFeatures.size() >= 32 && CustomShapeCount * 2 >= AFeatures.size();
			if (HasLargeCustomMajority){
				return { MetClusterStrategy::TemplateCluster };
			}

			return { MetClusterStrategy::None, MetClusterStrategy::TemplateCluster };
		}

		static TetClusterBuildResult DissolvePackedClusters(const CetTNestItemVector& AOriginalItems, const TetClusterBuildResult& ASource, const std::set<int>& APackedIndices)
		{
			TetClusterBuildResult Result;
			Result.NestItems.reserve(ASource.NestItems.size() + APackedIndices.size() * 4);
			Result.MetaItems.reserve(ASource.MetaItems.size() + APackedIndices.size() * 4);
			for (std::size_t PackedIndex = 0; PackedIndex < ASource.NestItems.size() && PackedIndex < ASource.MetaItems.size(); ++PackedIndex) {
				const TetMetaItem& SourceMeta = ASource.MetaItems[PackedIndex];
				const bool Dissolve = APackedIndices.find(static_cast<int>(PackedIndex)) != APackedIndices.end() && SourceMeta.IsCluster;
				if (!Dissolve) {
					Result.NestItems.push_back(ASource.NestItems[PackedIndex]);
					TetMetaItem Meta = SourceMeta;
					Meta.PackedItemIndex = static_cast<int>(Result.MetaItems.size());
					Result.MetaItems.push_back(std::move(Meta));
					continue;
				}

				for (const TetItemTransform& Transform : SourceMeta.TransformData) {
					if (Transform.OriginalId < 0 || Transform.OriginalId >= static_cast<int>(AOriginalItems.size())) {
						continue;
					}
					Result.NestItems.push_back(AOriginalItems[Transform.OriginalId]);
					TetMetaItem Meta;
					Meta.PackedItemIndex = static_cast<int>(Result.MetaItems.size());
					Meta.IsCluster = false;
					Meta.ClusterType = "SpacingFallbackSingle";
					Meta.TransformData.push_back({ Transform.OriginalId, 0.0, 0.0, 0.0 });
					Result.MetaItems.push_back(std::move(Meta));
				}
			}
			return Result;
		}

		bool CetNest2DEngine::_RunLastBinEvacuation(CetTNestItemVector& ANestItems, const TetNestOptions& AOptions, std::size_t& ALayers)
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
			std::cout << (Success ? "[LAST_BIN][SUCCESS]" : "[LAST_BIN][FAILED]")
				<< " UsedBins " << Stats.BeforeUsedBins << " -> " << Stats.AfterUsedBins
				<< ", DirectMoves=" << Stats.DirectMoves
				<< ", SameBinRelocations=" << Stats.SameBinRelocations
				<< ", RelocatedExistingSmallItems=" << Stats.RelocatedExistingSmallItemCount
				<< ", PlacementChecks=" << Stats.PlacementChecks
				<< ", SearchBudgetReached=" << Stats.SearchBudgetReached
				<< ", RemainingItems=" << Stats.RemainingItems
				<< ", TimeMs=" << Stats.TimeMs
				<< ", Rollback=" << Stats.RolledBack << std::endl;
			if (!Success) {
				std::cout << "[LAST_BIN][FAIL SUMMARY] Remaining=" << Stats.RemainingItems
					<< ", NoCandidatePosition=" << Stats.NoCandidatePosition
					<< ", RelocationFailed=" << Stats.RelocationFailed
					<< ", InsufficientFreeArea=" << Stats.InsufficientFreeArea << std::endl;
			}
#endif
			return Success;
		}

		bool CetNest2DEngine::_RepairAndEvacuate(CetTNestItemVector& ANestItems, const TetNestOptions& AOptions, const CetPolygonImpl& ABinPoly, double ABinWidth, double ABinHeight, std::size_t& ALayers)
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
			if (!ValidatePlacedItemsSpacing(ANestItems, AOptions)){
				std::cout << "[NEST][REPAIR][ROLLBACK] Repair produced an invalid spacing result." << std::endl;
				ANestItems = OriginalSolution;
				ALayers = OriginalLayers;
				return false;
			}
			return BoardFillImproved || LastBinImproved;
		}

		bool CetNest2DEngine::_TryBoardFeedbackNest(CetTNestItemVector& ANestItems, const TetNestOptions& AOptions, TetNestProgressTracker& ATracker, std::size_t& ALayers)
		{
			if (ANestItems.empty() || ALayers <= 1 || ANestItems.size() > CET_BOARD_FEEDBACK_NEST_MAX_ITEM_COUNT) {
				std::cout << "[BOARD FEEDBACK][SKIP] Items=" << ANestItems.size()
					<< " Layers=" << ALayers
					<< " Limit=" << CET_BOARD_FEEDBACK_NEST_MAX_ITEM_COUNT << std::endl;
				return false;
			}
			const CetTNestItemVector OriginalSolution = ANestItems;
			const std::size_t OriginalLayers = ALayers;
			const TetTNestEvalResult OriginalEval = Nest2DUtils->Nest2DStrategy->EvaluateNestResult(OriginalSolution, OriginalLayers);
			std::vector<std::size_t> Order(ANestItems.size());
			std::iota(Order.begin(), Order.end(), 0);
			std::stable_sort(Order.begin(), Order.end(), [&](std::size_t A, std::size_t B) {
				const CetNestItem& First = OriginalSolution[A];
				const CetNestItem& Second = OriginalSolution[B];
				if (First.binId() != Second.binId()) return First.binId() < Second.binId();
				const Point FirstTranslation = First.translation();
				const Point SecondTranslation = Second.translation();
				if (FirstTranslation.Y != SecondTranslation.Y) return FirstTranslation.Y < SecondTranslation.Y;
				if (FirstTranslation.X != SecondTranslation.X) return FirstTranslation.X < SecondTranslation.X;
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
			std::size_t FeedbackLayers = UsePolygonBoard
				? RunPolygonNestOnce(FeedbackItems, AOptions, ATracker)
				: RunRectangleNestOnce(FeedbackItems, AOptions, ATracker);
			CetTNestItemVector FeedbackSolution = OriginalSolution;
			for (std::size_t Position = 0; Position < Order.size(); ++Position) FeedbackSolution[Order[Position]] = std::move(FeedbackItems[Position]);
			const TetTNestEvalResult FeedbackEval = Nest2DUtils->Nest2DStrategy->EvaluateNestResult(FeedbackSolution, FeedbackLayers);
			const bool Valid = FeedbackLayers > 0 && ValidatePlacedItemsSpacing(FeedbackSolution, AOptions);
			const bool Improved = Valid && Nest2DUtils->Nest2DStrategy->IsBetterNestResult(FeedbackEval, OriginalEval);
			std::cout << "[BOARD FEEDBACK][RESULT] BeforeLayers=" << OriginalLayers
				<< " AfterLayers=" << FeedbackLayers
				<< " BeforeFirstArea=" << OriginalEval.FirstBinArea
				<< " AfterFirstArea=" << FeedbackEval.FirstBinArea
				<< " Valid=" << Valid << " Improved=" << Improved << std::endl;
			if (!Improved) return false;
			ANestItems = std::move(FeedbackSolution);
			ALayers = FeedbackLayers;
			return true;
		}
		
		int CetNest2DEngine::RunNesting_Impl(CetTNestItemVector& ANestItems, const TetNestOptions& AOptions, std::size_t* AUsedBins)
		{
			std::cout << "[DLL]this is running nesting" << std::endl;
			if (AUsedBins != nullptr){
				*AUsedBins = 0;
			}

			if (ANestItems.empty()){
				return NEST2D_ERR_CORE_EMPTY_INPUT;
			}
			const bool UsePolygonBoard = AOptions.Board.Enabled && AOptions.Board.Vertices.size() >= 3;

			int TotalItems = static_cast<int>(ANestItems.size());
			TetNestProgressTracker Tracker(TotalItems, AOptions.ProgressCallback);

			std::size_t Layers = 0;

			if (UsePolygonBoard){
				Layers = RunPolygonBoardNesting(ANestItems, AOptions, Tracker);
			}
			else {
				Layers = RunRectangleBoardNesting(ANestItems, AOptions, Tracker);
			}
			std::cout << "[NEST] after polygon nest, Layers = " << Layers << std::endl;

			if (Layers == 0){
				return NEST2D_ERR_CORE_NESTING_FAILED;
			}
			if (!ValidatePlacedItemsSpacing(ANestItems, AOptions)){
				std::cout << "[NEST][FINAL][ERROR] Result does not meet spacing requirements." << std::endl;
				return NEST2D_ERR_CORE_NESTING_FAILED;
			}

			if (AUsedBins != nullptr){
				*AUsedBins = Layers;
			}

			return Nest2D_Success;
		}

		std::size_t CetNest2DEngine::RunPolygonBoardNesting(CetTNestItemVector& ANestItems, const TetNestOptions& AOptions, TetNestProgressTracker& ATracker)
		{
			std::cout << "[NEST] use custom polygon board with strategy loop" << std::endl;

			CetTNestItemVector OriginalItems = ANestItems;
			const std::vector<TetShapeFeature> Features =Nest2DUtils->Nest2DShape->AnalyzeALL(OriginalItems);
			std::cout << "[SHAPE ANALYZER][DONE]" << " ItemCount = " << OriginalItems.size() << ", FeatureCount = " << Features.size() << std::endl;

			bool HasBest = false;
			CetTNestItemVector BestItems;
			TetTNestEvalResult BestEval{};
			std::size_t BestLayers = 0;
			std::vector<TetMetaItem> BestMetaItems;
			bool BestHasCluster = false;

			const std::vector<MetClusterStrategy> ClusterStrategies = BuildClusterStrategies(Features);

			for (auto ClusterStrategy : ClusterStrategies){
				TetClusterBuildResult ClusterResult =Nest2DUtils->Nest2DCluster->BuildClusterItemsWithFeatures(OriginalItems,Features,AOptions,ClusterStrategy);
				int ClusterCount = 0;
				for (const auto& Meta : ClusterResult.MetaItems){
					if (Meta.IsCluster){
						ClusterCount++;
					}
				}

				std::cout << "[POLYGON][CLUSTER][BUILD] Strategy = "
					<< static_cast<int>(ClusterStrategy)
					<< ", OriginalItems = " << OriginalItems.size()
					<< ", PackedItems = " << ClusterResult.NestItems.size()
					<< ", MetaItems = " << ClusterResult.MetaItems.size()
					<< ", ClusterCount = " << ClusterCount
					<< std::endl;

				TetExpandedSpacingFailure SpacingFailure;
				TetLocalBestResult LocalResult = EvaluateSortingStrategies(ClusterResult, OriginalItems, AOptions, ATracker, &SpacingFailure);
				if (!LocalResult.HasBest && SpacingFailure.Valid) {
					LocalResult = _TryLocalClusterSpacingFallback(ClusterResult, OriginalItems, AOptions, ATracker, SpacingFailure);
				}

				bool Better = ShoouldUpdateGlobalBest(LocalResult,HasBest,BestEval,BestLayers,BestHasCluster);

				if (Better){
					HasBest = true;
					BestEval = LocalResult.Eval;
					BestLayers = LocalResult.Layers;
					BestItems = std::move(LocalResult.Items);
					BestMetaItems = std::move(LocalResult.MetaItems);
					BestHasCluster = LocalResult.HasCluster;

					std::cout << "[POLYGON][GLOBAL BEST UPDATE] HasCluster = "
						<< BestHasCluster
						<< ", count = " << BestEval.FirstBinCount
						<< ", area = " << BestEval.FirstBinArea
						<< ", layers = " << BestEval.Layers
						<< ", packedItems = " << BestItems.size()
						<< std::endl;
				}
			}

			if (!HasBest){
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
				else {
					std::cout << "[POLYGON][FINAL] no valid best result." << std::endl;
					return 0;
				}
			}

			if (!BestHasCluster){
				std::cout << "[POLYGON][FINAL BEST] Restore normal item order." << std::endl;
			}
			else {
				std::cout << "[POLYGON][FINAL BEST] Use cluster expand." << std::endl;
			}
			const long double CoordinateScale = NestUtils::NestScale();
			std::cout << "[POLYGON][FINAL REMNANT] MetricsAvailable=" << BestEval.HasRemnantMetrics
				<< ", AreaMm2=" << static_cast<double>(BestEval.ReusableRemnantArea / (CoordinateScale * CoordinateScale))
				<< ", ShortSideMm=" << static_cast<double>(BestEval.ReusableRemnantShortSide / CoordinateScale)
				<< ", UsedDepthMm=" << static_cast<double>(BestEval.UsedDepth / CoordinateScale)
				<< ", Direction=" << (BestEval.RemnantIsTopStrip ? "Top" : "Right")
				<< std::endl;
			// Sorting strategies reorder packed items, so metadata restoration is required for singles and clusters.
			Nest2DUtils->Nest2DCluster->ExpandClusterResultToOriginalItems(OriginalItems,BestItems,BestMetaItems,ANestItems);
			
			double BoardBinWidth = AOptions.BinWidth;
			double BoardBinHeight = AOptions.BinHeight;
			CetPolygonImpl BinPoly = Nest2DUtils->Nest2DBord->BuildBinPolygonFromOptions(AOptions,BoardBinWidth,BoardBinHeight);

			if (_RepairAndEvacuate(ANestItems, AOptions, BinPoly, BoardBinWidth, BoardBinHeight, BestLayers)) {
				_TryBoardFeedbackNest(ANestItems, AOptions, ATracker, BestLayers);
			}
			BestEval = Nest2DUtils->Nest2DStrategy->EvaluateNestResult(ANestItems, BestLayers);
			std::cout << "================ POLYGON BEST NEST RESULT ================" << std::endl;
			std::cout << "[POLYGON BEST] bin0 count = " << BestEval.FirstBinCount << ", bin0 area = " << BestEval.FirstBinArea << ", layers = " << BestLayers << std::endl;

			Nest2DUtils->Nest2DStrategy->PrintBinCount(ANestItems);
			std::cout << "===========================================================" << std::endl;

			return BestLayers;
		}

		std::size_t CetNest2DEngine::RunPolygonNestOnce(CetTNestItemVector& ATestItems, const TetNestOptions& AOptions, TetNestProgressTracker& ATracker)
		{
			double BoardBinWidth = AOptions.BinWidth;
			double BoardBinHeight = AOptions.BinHeight;

			CetPolygonImpl BinPoly = Nest2DUtils->Nest2DBord->BuildBinPolygonFromOptions(AOptions,BoardBinWidth,BoardBinHeight);

			using CetMyPlacer = placers::_NofitPolyPlacer<CetPolygonImpl, CetPolygonImpl>;
			// Keep the primary ordering stable; expanded items are backfilled after nesting.
			using CetMySelector = selections::_FirstFitSelection<CetPolygonImpl>;

			NestConfig<CetMyPlacer, CetMySelector> cfg;

			cfg.placer_config.alignment =
				placers::NfpPConfig<CetPolygonImpl>::Alignment::DONT_ALIGN;

			cfg.placer_config.starting_point =
				placers::NfpPConfig<CetPolygonImpl>::Alignment::BOTTOM_LEFT;

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

			std::size_t Layers = nest(ATestItems,BinPoly,NestUtils::ToNestCoord(AOptions.Spacing),cfg,ProgressFunction{ ATracker });

			std::cout << "[POLYGON ONCE] before repair, Layers = " << Layers << std::endl;

			Nest2DUtils->Nest2DPolygonBord->SetContext(ATestItems,AOptions,BinPoly,BoardBinWidth,BoardBinHeight);

			Nest2DUtils->Nest2DPolygonBord->Repair(Layers);

			std::cout << "[POLYGON ONCE] after repair, Layers = " << Layers << std::endl;

			Nest2DUtils->Nest2DStrategy->PrintBinCount(ATestItems);

			return Layers;
		}

		std::size_t CetNest2DEngine::RunRectangleBoardNesting(CetTNestItemVector& ANestItems, const TetNestOptions& AOptions, TetNestProgressTracker& ATracker)
		{
			std::cout << "[NEST] use original rectangle BIN" << std::endl;
			CetTNestItemVector OriginalItems = ANestItems;
			//CetShapeAnalyzer ShapeAnalyzer;
			const std::vector<TetShapeFeature> Features =Nest2DUtils->Nest2DShape->AnalyzeALL(OriginalItems);
			std::cout << "[SHAPE ANALYZER][DONE]" << " ItemCount = " << OriginalItems.size() << ", FeatureCount = " << Features.size() << std::endl;
			
			bool HasBest = false;
			CetTNestItemVector BestItems;
			TetTNestEvalResult BestEval{};
			std::size_t BestLayers = 0;
			std::vector<TetMetaItem> BestMetaItems;
			bool BestHasCluster = false;
			
			const std::vector<MetClusterStrategy> ClusterStrategies = BuildClusterStrategies(Features);
			for (auto ClusterStrategy : ClusterStrategies){
				
			//	TetClusterBuildResult ClusterResult = Nest2DUtils->Nest2DCluster->BuildClusterItems(OriginalItems, AOptions, ClusterStrategy);
				TetClusterBuildResult ClusterResult = Nest2DUtils->Nest2DCluster->BuildClusterItemsWithFeatures(OriginalItems,Features, AOptions, ClusterStrategy);
				
				int ClusterCount = 0;
				for (const auto& Meta : ClusterResult.MetaItems){
					if (Meta.IsCluster) ClusterCount++;
				}
				std::cout << "[CLUSTER][BUILD] Strategy = " << static_cast<int>(ClusterStrategy)
					<< ", OriginalItems = " << OriginalItems.size()
					<< ", PackedItems = " << ClusterResult.NestItems.size()
					<< ", MetaItems = " << ClusterResult.MetaItems.size()
					<< ", ClusterCount = " << ClusterCount << std::endl;  

				
				TetExpandedSpacingFailure SpacingFailure;
				TetLocalBestResult LocalResult = EvaluateSortingStrategies(ClusterResult, OriginalItems, AOptions, ATracker, &SpacingFailure);
				if (!LocalResult.HasBest && SpacingFailure.Valid) {
					LocalResult = _TryLocalClusterSpacingFallback(ClusterResult, OriginalItems, AOptions, ATracker, SpacingFailure);
				}
				bool Better = ShoouldUpdateGlobalBest(LocalResult, HasBest, BestEval, BestLayers, BestHasCluster);
				
				if (Better){
					HasBest = true;
					BestEval = LocalResult.Eval;
					BestLayers = LocalResult.Layers;
					
					BestItems = std::move(LocalResult.Items);
					BestMetaItems = std::move(LocalResult.MetaItems);
					BestHasCluster = LocalResult.HasCluster;
					std::cout << "[NEST][GLOBAL BEST UPDATE] HasCluster = " << BestHasCluster
						<< ", count = " << BestEval.FirstBinCount
						<< ", area = " << BestEval.FirstBinArea
						<< ", layers = " << BestEval.Layers
						<< ", packedItems = " << BestItems.size() << std::endl;
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

			if (HasBest){
				std::cout << "[NEST][FINAL BEST] BestHasCluster = " << BestHasCluster << ", BestItems.size = " << BestItems.size() << ", BestMetaItems.size = " << BestMetaItems.size() << std::endl;

				if (!BestHasCluster){
					std::cout << "[NEST][FINAL BEST] Restore normal item order." << std::endl;
				}
				else {
					std::cout << "[NEST][FINAL BEST] Use cluster expand." << std::endl;
				}
				const long double CoordinateScale = NestUtils::NestScale();
				std::cout << "[NEST][FINAL REMNANT] MetricsAvailable=" << BestEval.HasRemnantMetrics
					<< ", AreaMm2=" << static_cast<double>(BestEval.ReusableRemnantArea / (CoordinateScale * CoordinateScale))
					<< ", ShortSideMm=" << static_cast<double>(BestEval.ReusableRemnantShortSide / CoordinateScale)
					<< ", SkylineWasteMm2=" << static_cast<double>(BestEval.SkylineWasteArea / (CoordinateScale * CoordinateScale))
					<< ", UsedDepthMm=" << static_cast<double>(BestEval.UsedDepth / CoordinateScale)
					<< ", Direction=" << (BestEval.RemnantIsTopStrip ? "Top" : "Right")
					<< std::endl;
				// Sorting strategies reorder packed items, so metadata restoration is required for singles and clusters.
				const CetTNestItemVector ItemsBeforeBackfill = BestItems;
				const std::size_t LayersBeforeBackfill = BestLayers;
				BestLayers = BackfillClusterSheets(BestItems, AOptions, BestLayers);
				if (!Nest2DUtils->Nest2DCluster->ValidatePackedResultSpacing(OriginalItems, BestItems, BestMetaItems, AOptions)){
					std::cout << "[NEST][CLUSTER BACKFILL][ROLLBACK] Expanded validation failed." << std::endl;
					BestItems = ItemsBeforeBackfill;
					BestLayers = LayersBeforeBackfill;
				}
				else {
					BestEval = Nest2DUtils->Nest2DStrategy->EvaluatePackedResultWithMeta(BestItems, BestMetaItems, OriginalItems, AOptions, BestLayers);
					std::cout << "[NEST][CLUSTER BACKFILL][VALID] FirstBinCount=" << BestEval.FirstBinCount
						<< ", Layers=" << BestEval.Layers << std::endl;
				}
				Nest2DUtils->Nest2DCluster->ExpandClusterResultToOriginalItems(OriginalItems, BestItems, BestMetaItems, ANestItems);
				std::cout << "[NEST][REPAIR PREP] ExpandedItems=" << ANestItems.size() << " Layers=" << BestLayers << std::endl;
				CetPolygonImpl RectBinPoly = Nest2DUtils->Nest2DBord->BuildRectangleBinPolygon(AOptions.BinWidth, AOptions.BinHeight);
				std::cout << "[NEST][REPAIR PREP] RectangleBinReady Contour=" << RectBinPoly.Contour.size() << std::endl;
				if (_RepairAndEvacuate(ANestItems, AOptions, RectBinPoly, AOptions.BinWidth, AOptions.BinHeight, BestLayers)) {
					_TryBoardFeedbackNest(ANestItems, AOptions, ATracker, BestLayers);
				}
				BestEval = Nest2DUtils->Nest2DStrategy->EvaluateNestResult(ANestItems, BestLayers);
			}
			std::cout << "================ BEST NEST RESULT ================" << std::endl;
				std::cout << "[NEST BEST] bin0 count = " << BestEval.FirstBinCount << ", bin0 area = " << BestEval.FirstBinArea << ", layers = " << BestEval.Layers << std::endl;
			Nest2DUtils->Nest2DStrategy->PrintBinCount(ANestItems);
			std::cout << "==================================================" << std::endl;

			return BestLayers;
		}

		std::size_t CetNest2DEngine::RunRectangleNestOnce(CetTNestItemVector& ATestItems, const TetNestOptions& AOptions, TetNestProgressTracker& ATracker)
		{
			double BinWidth = AOptions.BinWidth;
			double BinHeight = AOptions.BinHeight;

			auto Width = NestUtils::ToNestCoord(BinWidth);
			auto Height = NestUtils::ToNestCoord(BinHeight);

			Box Bin(Width, Height, { Width / 2, Height / 2 });
			//Box Bin(width, height);

			//using CetMyPlacer = placers::_NofitPolyPlacer<CetPolygonImpl, Box>;
			using CetMyPlacer = placers::_BottomLeftPlacer<CetPolygonImpl>;
			// Keep the primary ordering stable; expanded items are backfilled after nesting.
			using CetMySelector = selections::_FirstFitSelection<CetPolygonImpl>;
			//using CetMySelector = selections::_FillerSelection<CetPolygonImpl>;
			//using CetMySelector = selections::_DJDHeuristic<CetPolygonImpl>;

			NestConfig<CetMyPlacer, CetMySelector> cfg;
			
			//cfg.placer_config.accuracy = AOptions.Placer.Accuracy;
			////cfg.placer_config.alignment = placers::NfpPConfig<CetPolygonImpl>::Alignment::DONT_ALIGN;
			//cfg.placer_config.alignment = ToLibNestAlignment(AOptions.Placer.Alignment);
			//cfg.placer_config.starting_point = ToLibNestAlignment(AOptions.Placer.StartingPoint);
			//cfg.placer_config.parallel = AOptions.Placer.Parallel;
			//cfg.placer_config.explore_holes = AOptions.Placer.Parallel;
			//cfg.placer_config.rotations.clear();
			//FillRotations(cfg.placer_config.rotations, AOptions.Rotations);

			
			cfg.placer_config.min_obj_distance = NestUtils::ToNestCoord(AOptions.Spacing);
			cfg.placer_config.epsilon = 1;

			
			cfg.placer_config.allow_rotations = CetRotationUtils::IsAllowedRotation(CET_CLUSTER_HALF_PI, AOptions.Rotations, 1e-9);

			
			//cfg.selector_config.try_pairs = true;
			//cfg.selector_config.try_triplets = false;
			//cfg.selector_config.try_reverse_order = true;
			//cfg.selector_config.initial_fill_proportion = 0.2f;
			//cfg.selector_config.waste_increment = 0.1f;
			//cfg.selector_config.allow_parallel = true;
			//cfg.selector_config.force_parallel = false;

			std::cout << "================ DEBUG INFO ================" << std::endl;
			std::cout << "UsePolygonBoard: false" << std::endl;
			std::cout << "Bin Width: " << Bin.width() << ", Height: " << Bin.height() << std::endl;
			std::cout << "Spacing: " << NestUtils::ToNestCoord(AOptions.Spacing) << std::endl;
			std::cout << "============================================" << std::endl;

			std::size_t Layers = nest(ATestItems,Bin,NestUtils::ToNestCoord(AOptions.Spacing),cfg,ProgressFunction{ ATracker });

			std::cout << "[NEST] Layers = " << Layers << std::endl;
		/*	if (Layers > 0) {
				CetPolygonImpl RectBinPoly = Nest2DUtils->Nest2DBord->BuildRectangleBinPolygon(BinWidth, BinHeight);

				Nest2DUtils->Nest2DPolygonBord->SetContext(ATestItems, AOptions, RectBinPoly, BinWidth, BinHeight);
				Nest2DUtils->Nest2DPolygonBord->Repair(Layers);
			}*/

			Nest2DUtils->Nest2DStrategy->PrintBinCount(ATestItems);

			return Layers;
		}

		bool CetNest2DEngine::_HasClusterItems(const std::vector<TetMetaItem>& AMetaItems) const
		{
			return std::any_of(AMetaItems.begin(), AMetaItems.end(), [](const TetMetaItem& AMeta) { return AMeta.IsCluster; });
		}

		std::vector<std::size_t> CetNest2DEngine::_BuildPriorityOrder(CetTNestItemVector& AItems, const TetNestOptions& AOptions, MetENestOrderStrategy AStrategy) const
		{
			Nest2DUtils->Nest2DStrategy->ApplyNestPriorityStrategy(AItems, AOptions, AStrategy);
			std::vector<std::size_t> Indices(AItems.size());
			std::iota(Indices.begin(), Indices.end(), 0);
			std::stable_sort(Indices.begin(), Indices.end(), [&](std::size_t A, std::size_t AB) {
				const int PriorityA = AItems[A].priority();
				const int PriorityB = AItems[AB].priority();
				if (PriorityA != PriorityB) return PriorityA > PriorityB;
				const double AreaA = std::abs(static_cast<double>(AItems[A].area()));
				const double AreaB = std::abs(static_cast<double>(AItems[AB].area()));
				return std::abs(AreaA - AreaB) > 1e-6 ? AreaA > AreaB : A < AB;
			});
			return Indices;
		}

		void CetNest2DEngine::_BuildSortedTestData(CetTNestItemVector& APriorityItems, const std::vector<TetMetaItem>& AMetaItems, const std::vector<std::size_t>& ASortedIndices, CetTNestItemVector& AOutItems, std::vector<TetMetaItem>& AOutMetaItems) const
		{
			AOutItems.reserve(APriorityItems.size());
			AOutMetaItems.reserve(AMetaItems.size());
			for (std::size_t Index : ASortedIndices){
				AOutItems.push_back(std::move(APriorityItems[Index]));
				AOutMetaItems.push_back(AMetaItems[Index]);
				AOutMetaItems.back().PackedItemIndex = static_cast<int>(AOutMetaItems.size() - 1);
			}
		}

		void CetNest2DEngine::_UpdateLocalBest(TetLocalBestResult& ALocalBest, TetTNestEvalResult AEvaluation, std::size_t ALayers, CetTNestItemVector& AItems, std::vector<TetMetaItem>& AMetaItems, bool AHasCluster) const
		{
			bool Better = !ALocalBest.HasBest;
			if (!Better && Nest2DUtils->Nest2DStrategy->IsBetterNestResult(AEvaluation, ALocalBest.Eval)) Better = true;
			if (!Better && AHasCluster && !ALocalBest.HasCluster) Better = !Nest2DUtils->Nest2DStrategy->IsBetterNestResult(ALocalBest.Eval, AEvaluation);
			if (!Better) return;
			ALocalBest.HasBest = true;
			ALocalBest.Eval = AEvaluation;
			ALocalBest.Layers = ALayers;
			ALocalBest.Items = std::move(AItems);
			ALocalBest.MetaItems = std::move(AMetaItems);
			ALocalBest.HasCluster = AHasCluster;
			std::cout << "[NEST][LOCAL BEST UPDATE] HasCluster = " << ALocalBest.HasCluster << ", count = " << ALocalBest.Eval.FirstBinCount << ", area = " << ALocalBest.Eval.FirstBinArea << ", layers = " << ALocalBest.Eval.Layers << ", packedItems = " << ALocalBest.Items.size() << std::endl;
		}

		TetLocalBestResult CetNest2DEngine::_EvaluateSingleSortingStrategy(const TetClusterBuildResult& AClusterResult, const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, TetNestProgressTracker& ATracker, MetENestOrderStrategy AStrategy, TetExpandedSpacingFailure* AOutSpacingFailure)
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
			_UpdateLocalBest(LocalBest, Eval, Layers, TestItems, TestMetaItems, HasCluster);
			return LocalBest;
		}

		TetLocalBestResult CetNest2DEngine::_TryLocalClusterSpacingFallback(const TetClusterBuildResult& AClusterResult, const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, TetNestProgressTracker& ATracker, const TetExpandedSpacingFailure& AInitialFailure)
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
					const TetMetaItem& Meta = AClusterResult.MetaItems[PackedIndex];
					if (!Meta.IsCluster) {
						continue;
					}
					for (const TetItemTransform& Transform : Meta.TransformData) {
						if (Transform.OriginalId == Failure.FirstOriginalIndex || Transform.OriginalId == Failure.SecondOriginalIndex) {
							PackedIndices.insert(static_cast<int>(PackedIndex));
							break;
						}
					}
				}
				if (PackedIndices.size() == ClusterCountBefore) {
					std::cout << "[NEST][SPACING FALLBACK][LOCAL][STOP] Attempt=" << Attempt + 1
						<< ", reason=no additional conflicting cluster." << std::endl;
					break;
				}

				std::cout << "[NEST][SPACING FALLBACK][LOCAL] Attempt=" << Attempt + 1
					<< ", OriginalPair=" << Failure.FirstOriginalIndex << "," << Failure.SecondOriginalIndex
					<< ", DissolvedClusters=";
				for (int PackedIndex : PackedIndices) std::cout << PackedIndex << " ";
				std::cout << ", RawOverlap=" << (Failure.RawContoursIntersect ? 1 : 0) << std::endl;

				const TetClusterBuildResult DissolvedResult = DissolvePackedClusters(AOriginalItems, AClusterResult, PackedIndices);
				TetExpandedSpacingFailure RetryFailure;
				TetLocalBestResult LocalResult = _EvaluateSingleSortingStrategy(DissolvedResult, AOriginalItems, AOptions, ATracker, MetENestOrderStrategy::LargeFirst, &RetryFailure);
				if (LocalResult.HasBest) {
					std::cout << "[NEST][SPACING FALLBACK][LOCAL][VALID] Attempt=" << Attempt + 1
						<< ", Layers=" << LocalResult.Layers << ", PackedItems=" << LocalResult.Items.size() << std::endl;
					return LocalResult;
				}
				Failure = RetryFailure;
			}

			std::cout << "[NEST][SPACING FALLBACK][LOCAL][FAILED] Attempts=" << MaxLocalRetries << std::endl;
			return NoResult;
		}

		TetLocalBestResult CetNest2DEngine::EvaluateSortingStrategies(const TetClusterBuildResult& AClusterResult, const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, TetNestProgressTracker& ATracker, TetExpandedSpacingFailure* AOutSpacingFailure)
		{
			TetLocalBestResult LocalBest;
			if (AOutSpacingFailure != nullptr) {
				*AOutSpacingFailure = TetExpandedSpacingFailure{};
			}
			if (AClusterResult.NestItems.size() != AClusterResult.MetaItems.size()){
				std::cout << "[NEST][EVAL][ERROR] Cluster NestItems size != MetaItems size. NestItems = " << AClusterResult.NestItems.size() << ", MetaItems = " << AClusterResult.MetaItems.size() << std::endl;
				return LocalBest;
			}
			const bool UsePolygonBoard = AOptions.Board.Enabled && AOptions.Board.Vertices.size() >= 3;
			const bool CurrentHasCluster = _HasClusterItems(AClusterResult.MetaItems);
			std::vector<MetENestOrderStrategy> Strategies;
			if (AClusterResult.NestItems.size() > CET_NEST_REDUCED_STRATEGY_ITEM_LIMIT){
				Strategies = { MetENestOrderStrategy::LargeFirst, MetENestOrderStrategy::AreaDensityFirst };
			}
			else if (AClusterResult.NestItems.size() > CET_NEST_FULL_STRATEGY_ITEM_LIMIT){
				Strategies = { MetENestOrderStrategy::LargeFirst, MetENestOrderStrategy::AreaDensityFirst, MetENestOrderStrategy::LongSideFirst };
			}
			else {
				Strategies = { MetENestOrderStrategy::LargeFirst, MetENestOrderStrategy::AreaDensityFirst, MetENestOrderStrategy::SmallFirst, MetENestOrderStrategy::LongSideFirst, MetENestOrderStrategy::ThinFirst };
			}
			std::set<std::vector<std::size_t>> EvaluatedOrders;
			for (MetENestOrderStrategy Strategy : Strategies){
				CetTNestItemVector PriorityItems = AClusterResult.NestItems;
				const std::vector<std::size_t> SortedIndices = _BuildPriorityOrder(PriorityItems, AOptions, Strategy);
				if (!EvaluatedOrders.insert(SortedIndices).second){
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
				if (CurrentHasCluster && !Nest2DUtils->Nest2DCluster->ValidatePackedResultSpacing(AOriginalItems, TestItems, TestMetaItems, AOptions, &SpacingFailure)){
					if (AOutSpacingFailure != nullptr && !AOutSpacingFailure->Valid) {
						*AOutSpacingFailure = SpacingFailure;
					}
					std::cout << "[NEST][EVAL][SKIP] Strategy = " << static_cast<int>(Strategy) << ", reason = expanded cluster spacing violation" << std::endl;
					continue;
				}
				TetTNestEvalResult Eval = Nest2DUtils->Nest2DStrategy->EvaluatePackedResultWithMeta(TestItems, TestMetaItems, AOriginalItems, AOptions, Layers);
				std::cout << "[NEST][EVAL] Strategy = " << static_cast<int>(Strategy) << ", HasCluster = " << CurrentHasCluster << ", Eval.FirstBinCount = " << Eval.FirstBinCount << ", Eval.FirstBinArea = " << Eval.FirstBinArea << ", Eval.Layers = " << Eval.Layers << ", Eval.RemnantArea = " << Eval.ReusableRemnantArea << ", Eval.RemnantShortSide = " << Eval.ReusableRemnantShortSide << ", Eval.SkylineWaste = " << Eval.SkylineWasteArea << ", Eval.RemnantDirection = " << (Eval.RemnantIsTopStrip ? "Top" : "Right") << ", LocalBest.FirstBinCount = " << LocalBest.Eval.FirstBinCount << ", LocalBest.FirstBinArea = " << LocalBest.Eval.FirstBinArea << ", LocalBest.Layers = " << LocalBest.Eval.Layers << std::endl;
				_UpdateLocalBest(LocalBest, Eval, Layers, TestItems, TestMetaItems, CurrentHasCluster);
				if (AOriginalItems.size() > CET_NEST_FULL_STRATEGY_ITEM_LIMIT && LocalBest.HasBest && LocalBest.Layers == 1){
					// One sheet is already the minimum possible. On large orders a
					// second full NFP pass can cost minutes for only a secondary
					// remnant-shape comparison; the clustered strategy is still
					// evaluated separately and can replace this result.
					std::cout << "[NEST][EVAL][SKIP REMAINING] OriginalCount=" << AOriginalItems.size() << ", PackedCount=" << AClusterResult.NestItems.size() << ", reason=one-sheet optimum at large-order limit" << std::endl;
					break;
				}
			}
			return LocalBest;
		}
		bool CetNest2DEngine::ShoouldUpdateGlobalBest(const TetLocalBestResult& ALocalResult, bool AHasBest, const TetTNestEvalResult& ABestEval, std::size_t ABestLayers, bool ABestHasCluster)
		{
			if (!ALocalResult.HasBest){
				return false;
			}

			
			if (!AHasBest){
				return true;
			}

			
			if (Nest2DUtils->Nest2DStrategy->IsBetterNestResult(ALocalResult.Eval, ABestEval)){
				return true;
			}

			
			const bool Equivalent = !Nest2DUtils->Nest2DStrategy->IsBetterNestResult(ABestEval,ALocalResult.Eval);

			if (Equivalent){
				
				if (ALocalResult.HasCluster && !ABestHasCluster){
					return true;
				}
			}

			return false;
		}

	}
}
