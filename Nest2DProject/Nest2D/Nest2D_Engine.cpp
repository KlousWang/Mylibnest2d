
#include "pch.h"
#include "Nest2D_Engine.h"
#include "Nest2D_DataConst.h"
#include "NestUtils.h"
#include "Nest2D_SelfFunction.h"
#include <map>
#include "Nest2D_PolygonBoardRepairer.h"

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
		static void FillRotations(std::vector<libnest2d::Radians>& ARotations,int ARotationCount)
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
		struct TetNestProgressTracker {
			int totalItems;
			NestProgressCallback callback;

			TetNestProgressTracker(int total, NestProgressCallback cb)
				: totalItems(total), callback(cb) {
			}
			void operator()(unsigned cnt) const {
				if (callback != nullptr) {
					int finished = totalItems - static_cast<int>(cnt);
					callback(finished, totalItems);
				}
			}
		};
		int CetNest2DEngine::RunNesting_Impl(CetTNestItemVector& ANestItems, const TetNestOptions& AOptions, std::size_t* AUsedBins)
		{
			std::cout << "[DLL]this is running nesting" << std::endl;
			if (AUsedBins != nullptr) {
				*AUsedBins = 0;
			}

			if (ANestItems.empty()) {
				return NEST2D_ERR_CORE_EMPTY_INPUT;
			}		
			const bool UsePolygonBoard =AOptions.Board.Enabled &&AOptions.Board.Vertices.size() >= 3;

			int TotalItems = static_cast<int>(ANestItems.size());
			TetNestProgressTracker Tracker(TotalItems, AOptions.ProgressCallback);

			std::size_t Layers = 0;

			if (UsePolygonBoard) {
				std::cout << "[NEST] use custom polygon board" << std::endl;

				double BoardBinWidth = AOptions.BinWidth;
				double BoardBinHeight = AOptions.BinHeight;

				PolygonImpl binPoly = Nest2DUtils->BuildBinPolygonFromOptions(
					AOptions,
					BoardBinWidth,
					BoardBinHeight
				);

				using CetMyPlacer =
					placers::_NofitPolyPlacer<PolygonImpl, PolygonImpl>;

				using CetMySelector =
					selections::_FirstFitSelection<PolygonImpl>;

				NestConfig<CetMyPlacer, CetMySelector> cfg;

				cfg.placer_config.alignment =placers::NfpPConfig<PolygonImpl>::Alignment::DONT_ALIGN;
				cfg.placer_config.starting_point = placers::NfpPConfig<PolygonImpl>::Alignment::BOTTOM_LEFT;
				cfg.placer_config.accuracy = 1.0f;
				cfg.placer_config.parallel = false;
				cfg.placer_config.explore_holes = false;

				FillRotations(cfg.placer_config.rotations,AOptions.Rotations);

				std::cout << "================ DEBUG INFO ================" << std::endl;
				std::cout << "UsePolygonBoard: true" << std::endl;
				std::cout << "BoardBinWidth: " << BoardBinWidth<< ", BoardBinHeight: " << BoardBinHeight<< std::endl;
				std::cout << "Spacing: "<< NestUtils::ToNestCoord(AOptions.Spacing)<< std::endl;
				std::cout << "Board.Vertices.size: "<< AOptions.Board.Vertices.size()<< std::endl;
				std::cout << "============================================" << std::endl;

				Layers = nest(
					ANestItems,
					binPoly,
					NestUtils::ToNestCoord(AOptions.Spacing),
					cfg,
					ProgressFunction{ Tracker }
				);
				std::cout << "[NEST] before repair, Layers = "<< Layers<< std::endl;

				Nest2DUtils->SetPolygonBoardRepairContext(
					ANestItems,
					AOptions,
					binPoly,
					BoardBinWidth,
					BoardBinHeight
				);
				Nest2DUtils->RepairPolygonBoard(Layers);
				std::cout << "[NEST] after repair, Layers = "
					<< Layers
					<< std::endl;
			}
			else {
				std::cout << "[NEST] use original rectangle BIN" << std::endl;

				double BinWidth = AOptions.BinWidth;
				double BinHeight = AOptions.BinHeight;

				auto width = NestUtils::ToNestCoord(BinWidth);
				auto height = NestUtils::ToNestCoord(BinHeight);

				Box Bin(width, height, { width / 2, height / 2 });

				using CetMyPlacer =
					placers::_NofitPolyPlacer<PolygonImpl, Box>;

				using CetMySelector =
					selections::_FirstFitSelection<PolygonImpl>;

				NestConfig<CetMyPlacer, CetMySelector> cfg;

				cfg.placer_config.alignment =
					placers::NfpPConfig<PolygonImpl>::Alignment::BOTTOM_LEFT;

				cfg.placer_config.parallel = false;
				cfg.placer_config.explore_holes = true;

				FillRotations(cfg.placer_config.rotations,AOptions.Rotations);

				std::cout << "================ DEBUG INFO ================" << std::endl;
				std::cout << "UsePolygonBoard: false" << std::endl;
				std::cout << "Bin Width: " << Bin.width()<< ", Height: " << Bin.height()<< std::endl;
				std::cout << "Spacing: "<< NestUtils::ToNestCoord(AOptions.Spacing)<< std::endl;
				std::cout << "============================================" << std::endl;

				Layers = nest(
					ANestItems,
					Bin,
					NestUtils::ToNestCoord(AOptions.Spacing),
					cfg,
					ProgressFunction{ Tracker }
				);
			}
			std::cout << "[NEST] after polygon nest, Layers = "<< Layers<< std::endl;
			
			if (Layers == 0) {
				return NEST2D_ERR_CORE_NESTING_FAILED;
			}

			if (AUsedBins != nullptr) {
				*AUsedBins = Layers;
			}

			return Nest2D_Success;
		}

	}
}