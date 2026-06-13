#include "pch.h"
#include "Nest2D_Engine.h"
#include "Nest2D_DataConst.h"
//#include <libnest2d/backends/clipper/geometries.hpp>
//#include <libnest2d/libnest2d.hpp>
#include "NestUtils.h"

using namespace libnest2d;
namespace ET {
	namespace NEST2DMANAGERLIB {

		CetNest2DEngine::CetNest2DEngine() :CetCoreObject()
		{
		}

		CetNest2DEngine::~CetNest2DEngine()
		{
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

			if (AOptions.BinWidth <= 0.0 || AOptions.BinHeight <= 0.0) {
				return NEST2D_ERR_CORE_INVALID_SIZE;
			}
			auto width = NestUtils::ToNestCoord(AOptions.BinWidth);
			auto height = NestUtils::ToNestCoord(AOptions.BinHeight);

			Box Bin(width, height, { width / 2, height / 2 });

			using CetMyPlacer = placers::_NofitPolyPlacer<PolygonImpl>;
			using CetMySelector = selections::_FirstFitSelection<PolygonImpl>;

			int TotalItems = static_cast<int>(ANestItems.size());

			TetNestProgressTracker Tracker(TotalItems, AOptions.ProgressCallback);
			NestConfig<CetMyPlacer, CetMySelector> cfg;
			cfg.placer_config.alignment = placers::NfpPConfig<PolygonImpl>::Alignment::BOTTOM_LEFT;
			cfg.placer_config.parallel = true;
			cfg.placer_config.rotations.clear();
			if (AOptions.Rotations > 0) {
				const double PI = 3.14159265358979323846;
				double angleStep = (2.0 * PI) / AOptions.Rotations;
				for (int i = 0; i < AOptions.Rotations; ++i) {
					cfg.placer_config.rotations.push_back(i * angleStep);
				}
			}
			else
			{
				cfg.placer_config.rotations.push_back(0.0);
			}
			// --- 终极数据核对打印代码 开始 ---
			std::cout << "================ DEBUG INFO ================" << std::endl;

			// 1. 打印传入到底层的 Bin (画布) 尺寸
			std::cout << "Bin Width: " << Bin.width() << ", Height: " << Bin.height() << std::endl;

			// 2. 打印底层真正接收到的间距大小
			std::cout << "Spacing: " << NestUtils::ToNestCoord(AOptions.Spacing) << std::endl;

			// 3. 核心测试：测试一下你的缩放函数！
			// 我们假定原始输入的三角形边长是 30，画布是 100
			std::cout << "Test Scale (30.0 mm) -> " << NestUtils::ToNestCoord(30.0) << std::endl;
			std::cout << "Test Scale (100.0 mm) -> " << NestUtils::ToNestCoord(100.0) << std::endl;

			std::cout << "============================================" << std::endl;
			// --- 终极数据核对打印代码 结束 ---
			//cfg.placer_config.allow_rotations = true;

			/*std::vector<Item> input1(3,
				{
					{-5000000, 8954050},
					{5000000, 8954050},
					{5000000, -45949},
					{4972609, -568550},
					{3500000, -8954050},
					{-3500000, -8954050},
					{-4972609, -568550},
					{-5000000, -45949},
					{-5000000, 8954050},
				});
			std::vector<Item> input;
			input.insert(input.end(), input1.begin(), input1.end());*/
			std::size_t Layers = nest(
				ANestItems,
				Bin,
				//0,
				NestUtils::ToNestCoord(AOptions.Spacing),
				cfg,
				ProgressFunction{ Tracker }
			);

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