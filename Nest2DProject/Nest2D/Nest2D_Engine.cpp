
#include "pch.h"
#include "Nest2D_Engine.h"
#include "Nest2D_DataConst.h"
#include "Nest2D_PolygonBoardRepairer.h"
#include "Nest2D_PrivateDataType.h"
#include "NestUtils.h"
#include "Nest2D_SelfFunction.h"
#include"Nest2D_ClusterManager.h"
#include <map>
#include<vector>
#include<algorithm>
#include<limits>
#include<cmath>

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
			ARotations.clear();

			if (ARotationCount > 0) {
				const double PI = 3.14159265358979323846;
				double AngleStep = (2.0 * PI) / ARotationCount;

				for (int i = 0; i < ARotationCount; ++i) {
					ARotations.push_back(libnest2d::Radians(i * AngleStep));
				}
			}
			else {
				ARotations.push_back(libnest2d::Radians(0.0));
			}
		}
		static placers::NfpPConfig<PolygonImpl>::Alignment ToLibNestAlignment(MetNestAlignment AAlignment)
		{
			using CetAlignment = placers::NfpPConfig<PolygonImpl>::Alignment;

			switch (AAlignment)
			{
			case MetNestAlignment::DontAlign:
				return CetAlignment::DONT_ALIGN;

			case MetNestAlignment::BottomLeft:
			default:
				return CetAlignment::BOTTOM_LEFT;
			}
		}	
		


		int CetNest2DEngine::RunNesting_Impl(CetTNestItemVector& ANestItems, const TetNestOptions& AOptions, std::size_t* AUsedBins)
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
			}
			else {
				Layers = RunRectangleBoardNesting(ANestItems, AOptions, Tracker);
			}
			std::cout << "[NEST] after polygon nest, Layers = " << Layers << std::endl;

			if (Layers == 0) {
				return NEST2D_ERR_CORE_NESTING_FAILED;
			}

			if (AUsedBins != nullptr) {
				*AUsedBins = Layers;
			}

			return Nest2D_Success;
		}

		std::size_t CetNest2DEngine::RunPolygonBoardNesting(CetTNestItemVector& ANestItems, const TetNestOptions& AOptions, TetNestProgressTracker& Tracker)
		{
			std::cout << "[NEST] use custom polygon board with strategy loop"
				<< std::endl;

			CetTNestItemVector OriginalItems = ANestItems;

			bool HasBest = false;
			CetTNestItemVector BestItems;
			TetTNestEvalResult BestEval{};
			std::size_t BestLayers = 0;
			std::vector<TetMetaItem> BestMetaItems;
			bool BestHasCluster = false;

			std::vector<MetClusterStrategy> ClusterStrategies = {
				MetClusterStrategy::None,
				MetClusterStrategy::RightTrianglePair
			};

			for (auto ClusterStrategy : ClusterStrategies) {
				TetClusterBuildResult ClusterResult =Nest2DUtils->BuildClusterItems(OriginalItems,AOptions,ClusterStrategy);
				int ClusterCount = 0;
				for (const auto& Meta : ClusterResult.MetaItems) {
					if (Meta.IsCluster) {
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

				TetLocalBestResult LocalResult =EvaluateSortingStrategies(ClusterResult,OriginalItems,AOptions,Tracker);

				bool Better = ShoouldUpdateGlobalBest(LocalResult,HasBest,BestEval,BestLayers,BestHasCluster);

				if (Better) {
					HasBest = true;
					BestEval = LocalResult.Eval;
					BestLayers = LocalResult.Layers;
					BestItems = std::move(LocalResult.Items);
					BestMetaItems = ClusterResult.MetaItems;
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

			if (!HasBest) {
				std::cout << "[POLYGON][FINAL] no valid best result."
					<< std::endl;
				return 0;
			}

			if (!BestHasCluster) {
				std::cout << "[POLYGON][FINAL BEST] Use normal items."
					<< std::endl;
				ANestItems = std::move(BestItems);
			}
			else {
				std::cout << "[POLYGON][FINAL BEST] Use cluster expand."<< std::endl;
				Nest2DUtils->ExpandClusterResultToOriginalItems(OriginalItems,BestItems,BestMetaItems,ANestItems);
			}
			// Cluster 展开后，必须再做一次不规则板材合法性修复。
			double BoardBinWidth = AOptions.BinWidth;
			double BoardBinHeight = AOptions.BinHeight;
			PolygonImpl BinPoly = Nest2DUtils->BuildBinPolygonFromOptions(AOptions,BoardBinWidth,BoardBinHeight);

			Nest2DUtils->SetPolygonBoardRepairContext(ANestItems,AOptions,BinPoly,BoardBinWidth,BoardBinHeight);
			Nest2DUtils->RepairPolygonBoard(BestLayers);
			std::cout << "================ POLYGON BEST NEST RESULT ================"<< std::endl;
			std::cout << "[POLYGON BEST] bin0 count = "
				<< BestEval.FirstBinCount
				<< ", bin0 area = " << BestEval.FirstBinArea
				<< ", layers = " << BestLayers
				<< std::endl;

			Nest2DUtils->PrintBinCount(ANestItems);
			std::cout << "==========================================================="<< std::endl;

			return BestLayers;
		}

		std::size_t CetNest2DEngine::RunPolygonNestOnce(CetTNestItemVector& ATestItems, const TetNestOptions& AOptions, TetNestProgressTracker& ATracker)
		{
			double BoardBinWidth = AOptions.BinWidth;
			double BoardBinHeight = AOptions.BinHeight;

			PolygonImpl BinPoly = Nest2DUtils->BuildBinPolygonFromOptions(
				AOptions,
				BoardBinWidth,
				BoardBinHeight
			);

			using CetMyPlacer = placers::_NofitPolyPlacer<PolygonImpl, PolygonImpl>;
			using CetMySelector = selections::_FirstFitSelection<PolygonImpl>;

			NestConfig<CetMyPlacer, CetMySelector> cfg;

			cfg.placer_config.alignment =
				placers::NfpPConfig<PolygonImpl>::Alignment::DONT_ALIGN;

			cfg.placer_config.starting_point =
				placers::NfpPConfig<PolygonImpl>::Alignment::BOTTOM_LEFT;

			cfg.placer_config.accuracy = 1.0f;
			cfg.placer_config.parallel = true;
			cfg.placer_config.explore_holes = false;

			FillRotations(cfg.placer_config.rotations, AOptions.Rotations);

			std::cout << "================ POLYGON ONCE DEBUG ================" << std::endl;
			std::cout << "UsePolygonBoard: true" << std::endl;
			std::cout << "BoardBinWidth: " << BoardBinWidth
				<< ", BoardBinHeight: " << BoardBinHeight << std::endl;
			std::cout << "Spacing: "
				<< NestUtils::ToNestCoord(AOptions.Spacing) << std::endl;
			std::cout << "Board.Vertices.size: "
				<< AOptions.Board.Vertices.size() << std::endl;
			std::cout << "====================================================" << std::endl;

			std::size_t Layers = nest(
				ATestItems,
				BinPoly,
				NestUtils::ToNestCoord(AOptions.Spacing),
				cfg,
				ProgressFunction{ ATracker }
			);

			std::cout << "[POLYGON ONCE] before repair, Layers = "
				<< Layers << std::endl;

			Nest2DUtils->SetPolygonBoardRepairContext(
				ATestItems,
				AOptions,
				BinPoly,
				BoardBinWidth,
				BoardBinHeight
			);

			Nest2DUtils->RepairPolygonBoard(Layers);

			std::cout << "[POLYGON ONCE] after repair, Layers = "
				<< Layers << std::endl;

			Nest2DUtils->PrintBinCount(ATestItems);

			return Layers;
		}

		std::size_t CetNest2DEngine::RunRectangleBoardNesting(CetTNestItemVector& ANestItems, const TetNestOptions& AOptions, TetNestProgressTracker& Tracker)
		{
			std::cout << "[NEST] use original rectangle BIN" << std::endl;
			CetTNestItemVector OriginalItems = ANestItems;
			// 全局最优解的状态记录
			bool HasBest = false;
			CetTNestItemVector BestItems;
			TetTNestEvalResult BestEval{};
			std::size_t BestLayers = 0;
			std::vector<TetMetaItem> BestMetaItems;
			bool BestHasCluster = false;
			// 外层：组合件/聚类策略
			std::vector<MetClusterStrategy> ClusterStrategies = {
				 MetClusterStrategy::None,
				 MetClusterStrategy::RightTrianglePair
			};
			for (auto ClusterStrategy : ClusterStrategies) {
				//  构建当前策略下的 Cluster 数据
				TetClusterBuildResult ClusterResult = Nest2DUtils->BuildClusterItems(OriginalItems, AOptions, ClusterStrategy);
				// 打印调试信息
				int ClusterCount = 0;
				for (const auto& Meta : ClusterResult.MetaItems) {
					if (Meta.IsCluster) ClusterCount++;
				}
				std::cout << "[CLUSTER][BUILD] Strategy = " << static_cast<int>(ClusterStrategy)
					<< ", OriginalItems = " << OriginalItems.size()
					<< ", PackedItems = " << ClusterResult.NestItems.size()
					<< ", MetaItems = " << ClusterResult.MetaItems.size()
					<< ", ClusterCount = " << ClusterCount << std::endl;

				// 调用抽离的内层函数，获取该 Cluster 策略下的局部最优解
				TetLocalBestResult LocalResult = EvaluateSortingStrategies(ClusterResult, OriginalItems, AOptions, Tracker);
				bool Better = ShoouldUpdateGlobalBest(LocalResult, HasBest, BestEval, BestLayers, BestHasCluster);
				//  如果更好，更新全局最优解状态
				if (Better) {
					HasBest = true;
					BestEval = LocalResult.Eval;
					BestLayers = LocalResult.Layers;
					// 将局部最优数据转移给全局最优
					BestItems = std::move(LocalResult.Items);
					BestMetaItems = ClusterResult.MetaItems;
					BestHasCluster = LocalResult.HasCluster;
					std::cout << "[NEST][GLOBAL BEST UPDATE] HasCluster = " << BestHasCluster
						<< ", count = " << BestEval.FirstBinCount
						<< ", area = " << BestEval.FirstBinArea
						<< ", layers = " << BestEval.Layers
						<< ", packedItems = " << BestItems.size() << std::endl;
				}
			}
			//  最终结果处理与还原展开
			if (HasBest) {
				std::cout << "[NEST][FINAL BEST] BestHasCluster = " << BestHasCluster
					<< ", BestItems.size = " << BestItems.size()
					<< ", BestMetaItems.size = " << BestMetaItems.size() << std::endl;

				if (!BestHasCluster) {
					std::cout << "[NEST][FINAL BEST] Use normal items." << std::endl;
					ANestItems = std::move(BestItems); // 最终交接给外部
				}
				else {
					std::cout << "[NEST][FINAL BEST] Use cluster expand." << std::endl;
					Nest2DUtils->ExpandClusterResultToOriginalItems(OriginalItems, BestItems, BestMetaItems, ANestItems);
				}
			}
			std::cout << "================ BEST NEST RESULT ================" << std::endl;
			std::cout << "[NEST BEST] bin0 count = " << BestEval.FirstBinCount
				<< ", bin0 area = " << BestEval.FirstBinArea
				<< ", layers = " << BestEval.Layers << std::endl;
			Nest2DUtils->PrintBinCount(ANestItems);
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

			//using CetMyPlacer = placers::_NofitPolyPlacer<PolygonImpl, Box>;
			using CetMyPlacer = placers::_BottomLeftPlacer<PolygonImpl>;
			using CetMySelector = selections::_FirstFitSelection<PolygonImpl>;
			//using CetMySelector = selections::_FillerSelection<PolygonImpl>;
			//using CetMySelector = selections::_DJDHeuristic<PolygonImpl>;

			NestConfig<CetMyPlacer, CetMySelector> cfg;
			////nfp配置
			//cfg.placer_config.accuracy = AOptions.Placer.Accuracy;
			////cfg.placer_config.alignment = placers::NfpPConfig<PolygonImpl>::Alignment::DONT_ALIGN;
			//cfg.placer_config.alignment = ToLibNestAlignment(AOptions.Placer.Alignment);
			//cfg.placer_config.starting_point = ToLibNestAlignment(AOptions.Placer.StartingPoint);
			//cfg.placer_config.parallel = AOptions.Placer.Parallel;
			//cfg.placer_config.explore_holes = AOptions.Placer.Parallel;
			//cfg.placer_config.rotations.clear();
			//FillRotations(cfg.placer_config.rotations, AOptions.Rotations);

			// BottomLeftPlacer 的配置
			cfg.placer_config.min_obj_distance = NestUtils::ToNestCoord(AOptions.Spacing);
			cfg.placer_config.epsilon = 1;

			// BottomLeftPlacer 只支持“不旋转 / 失败后尝试 90 度”这种简单旋转
			cfg.placer_config.allow_rotations = (AOptions.Rotations > 1);

			////DJD配置
			//cfg.selector_config.try_pairs = true;
			//cfg.selector_config.try_triplets = false;
			//cfg.selector_config.try_reverse_order = true;
			//cfg.selector_config.initial_fill_proportion = 0.2f;
			//cfg.selector_config.waste_increment = 0.1f;
			//cfg.selector_config.allow_parallel = true;
			//cfg.selector_config.force_parallel = false;

			std::cout << "================ DEBUG INFO ================" << std::endl;
			std::cout << "UsePolygonBoard: false" << std::endl;
			std::cout << "Bin Width: " << Bin.width()
				<< ", Height: " << Bin.height() << std::endl;
			std::cout << "Spacing: "
				<< NestUtils::ToNestCoord(AOptions.Spacing) << std::endl;
			std::cout << "============================================" << std::endl;

			std::size_t Layers = nest(
				ATestItems,
				Bin,
				NestUtils::ToNestCoord(AOptions.Spacing),
				cfg,
				ProgressFunction{ ATracker }
			);

			std::cout << "[NEST] Layers = " << Layers << std::endl;

			Nest2DUtils->PrintBinCount(ATestItems);

			return Layers;
		}

		TetLocalBestResult CetNest2DEngine::EvaluateSortingStrategies(const TetClusterBuildResult& AClusterResult, const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, TetNestProgressTracker& ATracker)
		{
			// 初始化局部最优解状态
			TetLocalBestResult LocalBest;

			const bool UsePolygonBoard = AOptions.Board.Enabled && AOptions.Board.Vertices.size() >= 3;
			// 定义要测试的所有排序策略
			std::vector<MetENestOrderStrategy> Strategies = {
				MetENestOrderStrategy::LargeFirst,
				MetENestOrderStrategy::SmallFirst,
				MetENestOrderStrategy::LongSideFirst,
				MetENestOrderStrategy::ThinFirst
			};

			// 检查当前的 Cluster 结果中是否真的包含组合件
			bool CurrentHasCluster = false;
			for (const auto& Meta : AClusterResult.MetaItems) {
				if (Meta.IsCluster) {
					CurrentHasCluster = true;
					break;
				}
			}
			// 遍历所有排序策略打擂台
			for (MetENestOrderStrategy Strategy : Strategies) {
				// 准备测试数据（拷贝一份，避免相互污染）
				CetTNestItemVector TestItems = AClusterResult.NestItems;

				//  应用排序策略
				Nest2DUtils->ApplyNestPriorityStrategy(TestItems, Strategy);
				// 执行单次排版（调用底层引擎）
				std::size_t Layers = 0;
				if (UsePolygonBoard) {
					 Layers = RunPolygonNestOnce(TestItems, AOptions, ATracker);
				}
				else{
					 Layers = RunRectangleNestOnce(TestItems, AOptions, ATracker);
				}	

				//  评估本次排版结果
				TetTNestEvalResult Eval = Nest2DUtils->EvaluatePackedResultWithMeta(TestItems,AClusterResult.MetaItems,AOriginalItems,Layers);

				std::cout << "[NEST][EVAL] Strategy = " << static_cast<int>(Strategy)
					<< ", HasCluster = " << CurrentHasCluster
					<< ", Eval.FirstBinCount = " << Eval.FirstBinCount
					<< ", Eval.FirstBinArea = " << Eval.FirstBinArea
					<< ", Eval.Layers = " << Eval.Layers
					<< ", LocalBest.FirstBinCount = " << LocalBest.Eval.FirstBinCount
					<< ", LocalBest.FirstBinArea = " << LocalBest.Eval.FirstBinArea
					<< ", LocalBest.Layers = " << LocalBest.Eval.Layers
					<< std::endl;

				//  比较是否是更好的结果
				bool Better = false;
				if (!LocalBest.HasBest) {
					// 如果这是第一个跑出来的结果，直接当擂主
					Better = true;
				}
				else if (Nest2DUtils->IsBetterNestResult(Eval, LocalBest.Eval)) {
					// 如果按照评估标准，当前分数更高，踢馆成功
					Better = true;
				}
				else {
					// 如果主要分数相同，进入“抢七”断路器规则（优先选择带组合件的方案）
					bool SameCount = (Eval.FirstBinCount == LocalBest.Eval.FirstBinCount);
					bool SameLayers = (Eval.Layers == LocalBest.Eval.Layers);
					bool SameArea = std::abs(Eval.FirstBinArea - LocalBest.Eval.FirstBinArea) <= 1e-6;

					if (SameCount && SameLayers && SameArea) {
						if (CurrentHasCluster && !LocalBest.HasCluster) {
							Better = true;
						}
					}
				}

				//  如果更好，更新局部最优解状态
				if (Better) {
					LocalBest.HasBest = true;
					LocalBest.Eval = Eval;
					LocalBest.Layers = Layers;

					// 安全使用 std::move，因为 TestItems 的生命周期仅在当前单次循环内
					LocalBest.Items = std::move(TestItems);
					LocalBest.HasCluster = CurrentHasCluster;

					std::cout << "[NEST][LOCAL BEST UPDATE] HasCluster = " << LocalBest.HasCluster
						<< ", count = " << LocalBest.Eval.FirstBinCount
						<< ", area = " << LocalBest.Eval.FirstBinArea
						<< ", layers = " << LocalBest.Eval.Layers
						<< ", packedItems = " << LocalBest.Items.size()
						<< std::endl;
				}
			}

			// 将这一批测试中的最高分返回给外层主函数
			return LocalBest;
		}

		bool CetNest2DEngine::ShoouldUpdateGlobalBest(const TetLocalBestResult& ALocalResult, bool AHasBest, const TetTNestEvalResult& ABestEval, std::size_t ABestLayers, bool ABestHasCluster)
		{
			// 还没有全局最优，当前局部结果直接成为全局最优
			if (!AHasBest) {
				return true;
			}

			// 按原有评分逻辑比较，当前结果更优
			if (Nest2DUtils->IsBetterNestResult(ALocalResult.Eval, ABestEval)) {
				return true;
			}

			// 分数没有更优时，进入同等结果下的断路器规则
			bool SameCount = (ALocalResult.Eval.FirstBinCount == ABestEval.FirstBinCount);
			bool SameLayers = (ALocalResult.Layers == ABestLayers);
			bool SameArea = std::abs(ALocalResult.Eval.FirstBinArea - ABestEval.FirstBinArea) <= 1e-6;

			if (SameCount && SameLayers && SameArea) {
				// 同等结果下，优先选择带 cluster 的方案
				if (ALocalResult.HasCluster && !ABestHasCluster) {
					return true;
				}
			}

			return false;
		}

		//std::size_t CetNest2DEngine::RunRectangleBoardNestingFill(CetTNestItemVector& ANestItems, const TetNestOptions& AOptions, TetNestProgressTracker& Tracker)
		//{
		//	std::cout << "[NEST] use original rectangle BIN" << std::endl;

		//	double BinWidth = AOptions.BinWidth;
		//	double BinHeight = AOptions.BinHeight;

		//	auto width = NestUtils::ToNestCoord(BinWidth);
		//	auto height = NestUtils::ToNestCoord(BinHeight);

		//	Box Bin(width, height, { width / 2, height / 2 });
		//	//Box Bin(width, height);

		//	//using CetMyPlacer = placers::_NofitPolyPlacer<PolygonImpl, Box>;
		//	using CetMyPlacer = placers::_BottomLeftPlacer<PolygonImpl>;
		//	//using CetMySelector = selections::_FirstFitSelection<PolygonImpl>;
		//	using CetMySelector = selections::_FillerSelection<PolygonImpl>;
		//	//using CetMySelector = selections::_DJDHeuristic<PolygonImpl>;

		//	NestConfig<CetMyPlacer, CetMySelector> cfg;
		//	//nfp配置
		//	//cfg.placer_config.accuracy = AOptions.Placer.Accuracy;
		//	////cfg.placer_config.alignment = placers::NfpPConfig<PolygonImpl>::Alignment::DONT_ALIGN;
		//	//cfg.placer_config.alignment = ToLibNestAlignment(AOptions.Placer.Alignment);
		//	//cfg.placer_config.starting_point = ToLibNestAlignment(AOptions.Placer.StartingPoint);
		//	//cfg.placer_config.parallel = AOptions.Placer.Parallel;
		//	//cfg.placer_config.explore_holes = AOptions.Placer.Parallel;
		//	//cfg.placer_config.rotations.clear();
		//	//FillRotations(cfg.placer_config.rotations, AOptions.Rotations);

		//	// BottomLeftPlacer 的配置
		//	cfg.placer_config.min_obj_distance = NestUtils::ToNestCoord(AOptions.Spacing);
		//	cfg.placer_config.epsilon = 1;

		//	// BottomLeftPlacer 只支持“不旋转 / 失败后尝试 90 度”这种简单旋转
		//	cfg.placer_config.allow_rotations = (AOptions.Rotations > 1);

		//	std::cout << "================ DEBUG INFO ================" << std::endl;
		//	std::cout << "UsePolygonBoard: false" << std::endl;
		//	std::cout << "Bin Width: " << Bin.width()
		//		<< ", Height: " << Bin.height() << std::endl;
		//	std::cout << "Spacing: "
		//		<< NestUtils::ToNestCoord(AOptions.Spacing) << std::endl;
		//	std::cout << "============================================" << std::endl;

		//	using CetNester = libnest2d::_Nester<CetMyPlacer, CetMySelector>;

		//	CetNester Nester(
		//		Bin,
		//		NestUtils::ToNestCoord(AOptions.Spacing),
		//		cfg.placer_config,
		//		cfg.selector_config
		//	);

		//	Nester.progressIndicator(ProgressFunction{ Tracker });

		//	std::size_t Layers = Nester.execute(
		//		ANestItems.begin(),
		//		ANestItems.end()
		//	);
		//	const auto& PackResult = Nester.lastResult();
		//	Nest2DUtils->ExportSvgPackGroup(PackResult, AOptions);
		//	/*	if (Layers == 1)
		//		{
		//			for (auto& Item : ANestItems)
		//			{
		//				if (Item.binId() < 0)
		//				{
		//					Item.binId(0);
		//				}
		//			}

		//			std::cout << "[NEST][Filler Patch] Layers == 1, patch all unset binId to 0"
		//				<< std::endl;
		//		}*/
		//	std::cout << "[NEST] Layers = " << Layers << std::endl;

		//	std::map<int, int> BinCount;

		//	for (const auto& Item : ANestItems)
		//	{
		//		BinCount[Item.binId()]++;
		//	}

		//	for (const auto& Pair : BinCount)
		//	{
		//		std::cout << "[NEST] binId = " << Pair.first
		//			<< ", count = " << Pair.second << std::endl;
		//	}

		//	return Layers;
		//}

	}
}