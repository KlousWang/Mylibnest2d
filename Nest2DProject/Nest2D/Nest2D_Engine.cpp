
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

		static std::vector<MetClusterStrategy> BuildClusterStrategies(const std::vector<TetShapeFeature>& AFeatures)
		{
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

				TetLocalBestResult LocalResult =EvaluateSortingStrategies(ClusterResult,OriginalItems,AOptions,ATracker);

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
				std::cout << "[POLYGON][FINAL] no valid best result." << std::endl;
				return 0;
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

			Nest2DUtils->Nest2DPolygonBord->SetContext(ANestItems,AOptions,BinPoly,BoardBinWidth,BoardBinHeight);
			Nest2DUtils->Nest2DPolygonBord->Repair(BestLayers);
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

				
				TetLocalBestResult LocalResult = EvaluateSortingStrategies(ClusterResult, OriginalItems, AOptions, ATracker);
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
				Nest2DUtils->Nest2DCluster->ExpandClusterResultToOriginalItems(OriginalItems, BestItems, BestMetaItems, ANestItems);
			}
			//if(BestLayers > 0) {
			//	CetPolygonImpl RectBinPoly = Nest2DUtils->Nest2DBord->BuildRectangleBinPolygon(AOptions.BinWidth, AOptions.BinHeight);
			
			//	Nest2DUtils->Nest2DPolygonBord->SetContext(ANestItems, AOptions, RectBinPoly, AOptions.BinWidth, AOptions.BinHeight);
			//	Nest2DUtils->Nest2DPolygonBord->Repair(BestLayers);
			//}
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

		std::vector<std::size_t> CetNest2DEngine::_BuildPriorityOrder(CetTNestItemVector& AItems, MetENestOrderStrategy AStrategy) const
		{
			Nest2DUtils->Nest2DStrategy->ApplyNestPriorityStrategy(AItems, AStrategy);
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

		TetLocalBestResult CetNest2DEngine::EvaluateSortingStrategies(const TetClusterBuildResult& AClusterResult, const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, TetNestProgressTracker& ATracker)
		{
			TetLocalBestResult LocalBest;
			if (AClusterResult.NestItems.size() != AClusterResult.MetaItems.size()){
				std::cout << "[NEST][EVAL][ERROR] Cluster NestItems size != MetaItems size. NestItems = " << AClusterResult.NestItems.size() << ", MetaItems = " << AClusterResult.MetaItems.size() << std::endl;
				return LocalBest;
			}
			const bool UsePolygonBoard = AOptions.Board.Enabled && AOptions.Board.Vertices.size() >= 3;
			const bool CurrentHasCluster = _HasClusterItems(AClusterResult.MetaItems);
			const std::vector<MetENestOrderStrategy> Strategies = { MetENestOrderStrategy::LargeFirst, MetENestOrderStrategy::SmallFirst, MetENestOrderStrategy::LongSideFirst, MetENestOrderStrategy::ThinFirst };
			std::set<std::vector<std::size_t>> EvaluatedOrders;
			for (MetENestOrderStrategy Strategy : Strategies){
				CetTNestItemVector PriorityItems = AClusterResult.NestItems;
				const std::vector<std::size_t> SortedIndices = _BuildPriorityOrder(PriorityItems, Strategy);
				if (!EvaluatedOrders.insert(SortedIndices).second){
					std::cout << "[NEST][EVAL][SKIP] Strategy = " << static_cast<int>(Strategy) << ", reason = duplicate item order" << std::endl;
					continue;
				}
				CetTNestItemVector TestItems;
				std::vector<TetMetaItem> TestMetaItems;
				_BuildSortedTestData(PriorityItems, AClusterResult.MetaItems, SortedIndices, TestItems, TestMetaItems);
				const std::size_t Layers = UsePolygonBoard ? RunPolygonNestOnce(TestItems, AOptions, ATracker) : RunRectangleNestOnce(TestItems, AOptions, ATracker);
				if (CurrentHasCluster && !Nest2DUtils->Nest2DCluster->ValidatePackedResultNoOverlap(AOriginalItems, TestItems, TestMetaItems)){
					std::cout << "[NEST][EVAL][SKIP] Strategy = " << static_cast<int>(Strategy) << ", reason = expanded cluster overlap" << std::endl;
					continue;
				}
				TetTNestEvalResult Eval = Nest2DUtils->Nest2DStrategy->EvaluatePackedResultWithMeta(TestItems, TestMetaItems, AOriginalItems, AOptions, Layers);
				std::cout << "[NEST][EVAL] Strategy = " << static_cast<int>(Strategy) << ", HasCluster = " << CurrentHasCluster << ", Eval.FirstBinCount = " << Eval.FirstBinCount << ", Eval.FirstBinArea = " << Eval.FirstBinArea << ", Eval.Layers = " << Eval.Layers << ", Eval.RemnantArea = " << Eval.ReusableRemnantArea << ", Eval.RemnantShortSide = " << Eval.ReusableRemnantShortSide << ", Eval.SkylineWaste = " << Eval.SkylineWasteArea << ", Eval.RemnantDirection = " << (Eval.RemnantIsTopStrip ? "Top" : "Right") << ", LocalBest.FirstBinCount = " << LocalBest.Eval.FirstBinCount << ", LocalBest.FirstBinArea = " << LocalBest.Eval.FirstBinArea << ", LocalBest.Layers = " << LocalBest.Eval.Layers << std::endl;
				_UpdateLocalBest(LocalBest, Eval, Layers, TestItems, TestMetaItems, CurrentHasCluster);
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
