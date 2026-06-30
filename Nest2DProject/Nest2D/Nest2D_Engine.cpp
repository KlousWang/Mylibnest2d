
#include "pch.h"
#include "Nest2D_Engine.h"
#include "Nest2D_DataConst.h"
#include "Nest2D_PolygonBoardRepairer.h"
#include "NestUtils.h"
#include "Nest2D_SelfFunction.h"
#include <map>

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
				Layers = RunRectangleBoardNestingFill(ANestItems, AOptions, Tracker);
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

			double BinWidth = AOptions.BinWidth;
			double BinHeight = AOptions.BinHeight;

			auto width = NestUtils::ToNestCoord(BinWidth);
			auto height = NestUtils::ToNestCoord(BinHeight);

			Box Bin(width, height, { width / 2, height / 2 });
			//Box Bin(width, height);

			using CetMyPlacer = placers::_NofitPolyPlacer<PolygonImpl, Box>;
			//using CetMyPlacer = placers::_BottomLeftPlacer<PolygonImpl>;
			using CetMySelector = selections::_FirstFitSelection<PolygonImpl>;
			//using CetMySelector = selections::_FillerSelection<PolygonImpl>;
			//using CetMySelector = selections::_DJDHeuristic<PolygonImpl>;

			NestConfig<CetMyPlacer, CetMySelector> cfg;
			//nfp配置
			cfg.placer_config.accuracy = AOptions.Placer.Accuracy;
			//cfg.placer_config.alignment = placers::NfpPConfig<PolygonImpl>::Alignment::DONT_ALIGN;
			cfg.placer_config.alignment = ToLibNestAlignment(AOptions.Placer.Alignment);
			cfg.placer_config.starting_point = ToLibNestAlignment(AOptions.Placer.StartingPoint);
			cfg.placer_config.parallel = AOptions.Placer.Parallel;
			cfg.placer_config.explore_holes = AOptions.Placer.Parallel;
			cfg.placer_config.rotations.clear();
			FillRotations(cfg.placer_config.rotations, AOptions.Rotations);

			//// BottomLeftPlacer 的配置
			//cfg.placer_config.min_obj_distance = NestUtils::ToNestCoord(AOptions.Spacing);
			//cfg.placer_config.epsilon = 1;

			//// BottomLeftPlacer 只支持“不旋转 / 失败后尝试 90 度”这种简单旋转
			//cfg.placer_config.allow_rotations = (AOptions.Rotations > 1);

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
				ANestItems,
				Bin,
				NestUtils::ToNestCoord(AOptions.Spacing),
				cfg,
				ProgressFunction{ Tracker }
			);
		
			std::cout << "[NEST] Layers = " << Layers << std::endl;

			std::map<int, int> BinCount;

			for (const auto& Item : ANestItems)
			{
				BinCount[Item.binId()]++;
			}

			for (const auto& Pair : BinCount)
			{
				std::cout << "[NEST] binId = " << Pair.first
					<< ", count = " << Pair.second << std::endl;
			}

			return Layers;
		}

		std::size_t CetNest2DEngine::RunRectangleBoardNestingFill(CetTNestItemVector& ANestItems, const TetNestOptions& AOptions, TetNestProgressTracker& Tracker)
		{
			std::cout << "[NEST] use original rectangle BIN" << std::endl;

			double BinWidth = AOptions.BinWidth;
			double BinHeight = AOptions.BinHeight;

			auto width = NestUtils::ToNestCoord(BinWidth);
			auto height = NestUtils::ToNestCoord(BinHeight);

			Box Bin(width, height, { width / 2, height / 2 });
			//Box Bin(width, height);

			//using CetMyPlacer = placers::_NofitPolyPlacer<PolygonImpl, Box>;
			using CetMyPlacer = placers::_BottomLeftPlacer<PolygonImpl>;
			//using CetMySelector = selections::_FirstFitSelection<PolygonImpl>;
			using CetMySelector = selections::_FillerSelection<PolygonImpl>;
			//using CetMySelector = selections::_DJDHeuristic<PolygonImpl>;

			NestConfig<CetMyPlacer, CetMySelector> cfg;
			//nfp配置
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

			std::cout << "================ DEBUG INFO ================" << std::endl;
			std::cout << "UsePolygonBoard: false" << std::endl;
			std::cout << "Bin Width: " << Bin.width()
				<< ", Height: " << Bin.height() << std::endl;
			std::cout << "Spacing: "
				<< NestUtils::ToNestCoord(AOptions.Spacing) << std::endl;
			std::cout << "============================================" << std::endl;

			using CetNester = libnest2d::_Nester<CetMyPlacer, CetMySelector>;

			CetNester Nester(
				Bin,
				NestUtils::ToNestCoord(AOptions.Spacing),
				cfg.placer_config,
				cfg.selector_config
			);

			Nester.progressIndicator(ProgressFunction{ Tracker });

			std::size_t Layers = Nester.execute(
				ANestItems.begin(),
				ANestItems.end()
			);
			const auto& PackResult = Nester.lastResult();
			Nest2DUtils->ExportSvgPackGroup(PackResult, AOptions);
			/*	if (Layers == 1)
				{
					for (auto& Item : ANestItems)
					{
						if (Item.binId() < 0)
						{
							Item.binId(0);
						}
					}

					std::cout << "[NEST][Filler Patch] Layers == 1, patch all unset binId to 0"
						<< std::endl;
				}*/
			std::cout << "[NEST] Layers = " << Layers << std::endl;

			std::map<int, int> BinCount;

			for (const auto& Item : ANestItems)
			{
				BinCount[Item.binId()]++;
			}

			for (const auto& Pair : BinCount)
			{
				std::cout << "[NEST] binId = " << Pair.first
					<< ", count = " << Pair.second << std::endl;
			}

			return Layers;
		}

	}
}