
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
			std::cout << "[NEST] use custom polygon board" << std::endl;

			double BoardBinWidth = AOptions.BinWidth;
			double BoardBinHeight = AOptions.BinHeight;

			PolygonImpl binPoly =Nest2DUtils->BuildBinPolygonFromOptions(AOptions,BoardBinWidth,BoardBinHeight);
			using CetMyPlacer =placers::_NofitPolyPlacer<PolygonImpl, PolygonImpl>;
			using CetMySelector =selections::_FirstFitSelection<PolygonImpl>;

			NestConfig<CetMyPlacer, CetMySelector> cfg;
			cfg.placer_config.alignment =placers::NfpPConfig<PolygonImpl>::Alignment::DONT_ALIGN;
			cfg.placer_config.starting_point =placers::NfpPConfig<PolygonImpl>::Alignment::BOTTOM_LEFT;

			cfg.placer_config.accuracy = 1.0f;
			cfg.placer_config.parallel = true;
			cfg.placer_config.explore_holes = false;

			FillRotations(cfg.placer_config.rotations, AOptions.Rotations);

			std::cout << "================ DEBUG INFO ================" << std::endl;
			std::cout << "UsePolygonBoard: true" << std::endl;
			std::cout << "BoardBinWidth: " << BoardBinWidth
				<< ", BoardBinHeight: " << BoardBinHeight << std::endl;
			std::cout << "Spacing: "
				<< NestUtils::ToNestCoord(AOptions.Spacing) << std::endl;
			std::cout << "Board.Vertices.size: "
				<< AOptions.Board.Vertices.size() << std::endl;
			std::cout << "============================================" << std::endl;

			std::size_t Layers = nest(
				ANestItems,
				binPoly,
				NestUtils::ToNestCoord(AOptions.Spacing),
				cfg,
				ProgressFunction{ Tracker }
			);

			std::cout << "[NEST] before repair, Layers = " << Layers << std::endl;

			Nest2DUtils->SetPolygonBoardRepairContext(
				ANestItems,
				AOptions,
				binPoly,
				BoardBinWidth,
				BoardBinHeight
			);

			Nest2DUtils->RepairPolygonBoard(Layers);

			std::cout << "[NEST] after repair, Layers = " << Layers << std::endl;

			return Layers;
		}

		std::size_t CetNest2DEngine::RunRectangleBoardNesting(CetTNestItemVector& ANestItems, const TetNestOptions& AOptions, TetNestProgressTracker& Tracker)
		{
			std::cout << "[NEST] use original rectangle BIN" << std::endl;

			CetTNestItemVector OriginalItems = ANestItems;
			std::vector<MetENestOrderStrategy> Strategies = {
				MetENestOrderStrategy::LargeFirst,
				MetENestOrderStrategy::SmallFirst,
				MetENestOrderStrategy::LongSideFirst,
				MetENestOrderStrategy::ThinFirst
			};
			bool HasBest = false;
			CetTNestItemVector BestItems;
			TetTetTNestEvalResult BestEval;
			std::size_t BestLayers = 0;

			// 保存最佳结果对应的 cluster 元信息
			std::vector<TetMetaItem> BestMetaItems;

			// 标记最佳结果里面是否真的包含组合件
			bool BestHasCluster = false;

			auto RunOnce = [&](CetTNestItemVector& TestItems)->std::size_t {
				double BinWidth = AOptions.BinWidth;
				double BinHeight = AOptions.BinHeight;

				auto width = NestUtils::ToNestCoord(BinWidth);
				auto height = NestUtils::ToNestCoord(BinHeight);
				Box Bin(width, height, { width / 2, height / 2 });
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
					TestItems,
					Bin,
					NestUtils::ToNestCoord(AOptions.Spacing),
					cfg,
					ProgressFunction{ Tracker }
				);

				std::cout << "[NEST] Layers = " << Layers << std::endl;

				std::map<int, int> BinCount;
				for (const auto& Item : TestItems){
					BinCount[Item.binId()]++;
				}
				for (const auto& Pair : BinCount){
					std::cout << "[NEST] binId = " << Pair.first
						<< ", count = " << Pair.second << std::endl;
				}
				return Layers;
				};
			auto EvaluatePackedResultWithMeta =[&](const CetTNestItemVector& PackedItems,const std::vector<TetMetaItem>& MetaItems,std::size_t Layers) -> TetTetTNestEvalResult{
					TetTetTNestEvalResult Result;
					Result.Layers = Layers;

					if (PackedItems.size() != MetaItems.size()){
						std::cout << "[NEST][EVAL][ERROR] PackedItems size != MetaItems size. "
							<< "PackedItems = " << PackedItems.size()
							<< ", MetaItems = " << MetaItems.size()
							<< std::endl;
						return Result;
					}
					for (std::size_t PackedIndex = 0; PackedIndex < PackedItems.size(); ++PackedIndex){
						const auto& PackedItem = PackedItems[PackedIndex];
						const auto& Meta = MetaItems[PackedIndex];
						if (PackedItem.binId() != 0){
							continue;
						}
						// 这里统计原始零件数量，不是 packed item 数量
						for (const auto& Transform : Meta.TransformData){
							int OriginalId = Transform.OriginalId;
							if (OriginalId < 0 ||OriginalId >= static_cast<int>(OriginalItems.size())){
								continue;
							}
							Result.FirstBinCount++;

							// 面积也用原始零件面积，而不是 cluster 矩形面积
							Result.FirstBinArea +=std::abs(static_cast<double>(OriginalItems[OriginalId].area()));
						}
					}
					return Result;
				};
			std::vector<MetClusterStrategy> ClusterStrategies = {
				 MetClusterStrategy::None,
				 MetClusterStrategy::RightTrianglePair
			};
			for(auto ClusterStrategy : ClusterStrategies){
				TetClusterBuildResult ClusterResult =Nest2DUtils->BuildClusterItems(OriginalItems, AOptions, ClusterStrategy);
				int ClusterCount = 0;
				for (const auto& Meta : ClusterResult.MetaItems){
					if (Meta.IsCluster){
						ClusterCount++;
					}
				}
				std::cout << "[CLUSTER][BUILD] Strategy = "
					<< static_cast<int>(ClusterStrategy)
					<< ", OriginalItems = " << OriginalItems.size()
					<< ", PackedItems = " << ClusterResult.NestItems.size()
					<< ", MetaItems = " << ClusterResult.MetaItems.size()
					<< ", ClusterCount = " << ClusterCount
					<< std::endl;
				bool CurrentHasCluster = false;
				for (const auto& Meta : ClusterResult.MetaItems){
					if (Meta.IsCluster){
						CurrentHasCluster = true;
						break;
					}
				}

				for (MetENestOrderStrategy Strategy : Strategies){
					CetTNestItemVector TestItems = ClusterResult.NestItems;
					Nest2DUtils->ApplyNestPriorityStrategy(TestItems, Strategy);
	   		     	std::size_t Layers = RunOnce(TestItems);

					//TetTetTNestEvalResult Eval = Nest2DUtils->EvaluateNestResult(TestItems, Layers);
					TetTetTNestEvalResult Eval =EvaluatePackedResultWithMeta(TestItems,ClusterResult.MetaItems,Layers);
					bool Better = false;

					if (!HasBest){
						Better = true;
					}
					else if (Nest2DUtils->IsBetterNestResult(Eval, BestEval)){
						Better = true;
					}
					else{
						bool SameCount = (Eval.FirstBinCount == BestEval.FirstBinCount);
						bool SameLayers = (Eval.Layers == BestEval.Layers);
						bool SameArea =std::abs(Eval.FirstBinArea - BestEval.FirstBinArea) <= 1e-6;
						// 同等结果时，优先选择带 cluster 的方案
						if (SameCount && SameLayers && SameArea){
							if (CurrentHasCluster && !BestHasCluster){
								Better = true;
							}
						}
					}
					if (Better){
						HasBest = true;
						BestEval = Eval;
						BestLayers = Layers;
						BestItems = std::move(TestItems);
						BestMetaItems = ClusterResult.MetaItems;
						BestHasCluster = CurrentHasCluster;
						std::cout << "[NEST][BEST UPDATE] HasCluster = "<< BestHasCluster<< ", count = " << BestEval.FirstBinCount<< ", area = " << BestEval.FirstBinArea
							<< ", layers = " << BestEval.Layers<< ", packedItems = " << BestItems.size()<< std::endl;
					}
				}
			}
			if (HasBest){
				std::cout << "[NEST][FINAL BEST] BestHasCluster = "<< BestHasCluster<< ", BestItems.size = " << BestItems.size()<< ", BestMetaItems.size = " << BestMetaItems.size()<< std::endl;

				if (!BestHasCluster){
					std::cout << "[NEST][FINAL BEST] Use normal items." << std::endl;
					ANestItems = BestItems;
				}
				else{
					std::cout << "[NEST][FINAL BEST] Use cluster expand." << std::endl;

					Nest2DUtils->ExpandClusterResultToOriginalItems(OriginalItems,BestItems,BestMetaItems,ANestItems
					);
				}
			}
			std::cout << "================ BEST NEST RESULT ================"<< std::endl;

			std::cout << "[NEST BEST] bin0 count = "<< BestEval.FirstBinCount<< ", bin0 area = "<< BestEval.FirstBinArea
				<< ", layers = "<< BestEval.Layers<< std::endl;
			Nest2DUtils-> PrintBinCount(ANestItems);

			std::cout << "=================================================="<< std::endl;

			return BestLayers;
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