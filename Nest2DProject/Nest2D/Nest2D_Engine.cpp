
#include "pch.h"
#include "Nest2D_Engine.h"
#include "Nest2D_DataConst.h"
#include "NestUtils.h"
#include <map>

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
		static TetBoardBounds CalcBoardBoundsLocal(const TetNestBoard& ABoard)
		{
			TetBoardBounds B;

			if (!ABoard.Enabled || ABoard.Vertices.size() < 3) {
				return B;
			}

			B.MinX = B.MaxX = ABoard.Vertices[0].X;
			B.MinY = B.MaxY = ABoard.Vertices[0].Y;

			for (const auto& P : ABoard.Vertices) {
				B.MinX = std::min(B.MinX, P.X);
				B.MaxX = std::max(B.MaxX, P.X);
				B.MinY = std::min(B.MinY, P.Y);
				B.MaxY = std::max(B.MaxY, P.Y);
			}

			B.Width = B.MaxX - B.MinX;
			B.Height = B.MaxY - B.MinY;
			B.Valid = B.Width > 0.0 && B.Height > 0.0;

			return B;
		}
		static Path BuildPathFromPoints(const std::vector<TetNestPoint>& APoints,double AOffsetX,double AOffsetY,bool AWantOuter)
		{
			Path Result;
			Result.reserve(APoints.size());

			for (const auto& P : APoints) {
				Result.push_back(Point(
					NestUtils::ToNestCoord(P.X - AOffsetX),
					NestUtils::ToNestCoord(P.Y - AOffsetY)
				));
			}

			if (Result.size() >= 3) {
				bool IsCCW = ClipperLib::Orientation(Result);

				// 外轮廓保持 CCW
				if (AWantOuter && !IsCCW) {
					std::reverse(Result.begin(), Result.end());
				}

				// 洞保持 CW
				if (!AWantOuter && IsCCW) {
					std::reverse(Result.begin(), Result.end());
				}
			}

			return Result;
		}
		static PolygonImpl BuildBinPolygonFromOptions(const TetNestOptions& AOptions,double& AOutBinWidth,double& AOutBinHeight)
		{
			if (AOptions.Board.Enabled && AOptions.Board.Vertices.size() >= 3) {
				TetBoardBounds Bounds = CalcBoardBoundsLocal(AOptions.Board);

				if (!Bounds.Valid) {
					throw std::runtime_error("Invalid custom board bounds.");
				}
				AOutBinWidth = Bounds.Width;
				AOutBinHeight = Bounds.Height;
				Path Outer = BuildPathFromPoints(
					AOptions.Board.Vertices,
					Bounds.MinX,
					Bounds.MinY,
					true
				);
				Paths Holes;
				Holes.reserve(AOptions.Board.Holes.size());

				for (const auto& Hole : AOptions.Board.Holes) {
					if (Hole.size() < 3) {
						continue;
					}

					Holes.push_back(BuildPathFromPoints(Hole,Bounds.MinX,Bounds.MinY,false));
				}

				return PolygonImpl(std::move(Outer), std::move(Holes));
			}

			AOutBinWidth = AOptions.BinWidth;
			AOutBinHeight = AOptions.BinHeight;

			Path Outer;

			Outer.push_back(Point(NestUtils::ToNestCoord(0.0),NestUtils::ToNestCoord(0.0)));

			Outer.push_back(Point(NestUtils::ToNestCoord(AOptions.BinWidth),NestUtils::ToNestCoord(0.0)));

			Outer.push_back(Point(NestUtils::ToNestCoord(AOptions.BinWidth),NestUtils::ToNestCoord(AOptions.BinHeight)));

			Outer.push_back(Point(NestUtils::ToNestCoord(0.0),NestUtils::ToNestCoord(AOptions.BinHeight)));

			if (ClipperLib::Orientation(Outer) == false) {
				std::reverse(Outer.begin(), Outer.end());
			}

			Paths Holes;

			return PolygonImpl(std::move(Outer), std::move(Holes));
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
		static std::size_t CompactNestItemBins(CetTNestItemVector& ANestItems)
		{
			std::map<int, int> Remap;
			int NextBin = 0;

			for (auto& Item : ANestItems) {
				int OldBin = static_cast<int>(Item.binId());

				if (OldBin < 0) {
					continue;
				}

				auto It = Remap.find(OldBin);

				if (It == Remap.end()) {
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
		static bool CanPlaceNestItemAt(CetTNestItemVector& ANestItems,std::size_t AItemIndex,int ATargetBin,const PolygonImpl& ABinPoly,const Point& ATranslation,libnest2d::Radians ARotation,long long ASpacingCoord)
		{
			using NestItemType = CetTNestItemVector::value_type;

			if (AItemIndex >= ANestItems.size()) {
				return false;
			}

			auto& Candidate = ANestItems[AItemIndex];

			Point OldTranslation = Candidate.translation();
			libnest2d::Radians OldRotation = Candidate.rotation();
			int OldBin = static_cast<int>(Candidate.binId());
			auto OldInflation = Candidate.inflation();

			Candidate.translation(ATranslation);
			Candidate.rotation(ARotation);
			Candidate.binId(ATargetBin);
			Candidate.inflation(0);

			bool CanPlace = true;

			auto BB = Candidate.boundingBox();

			if (getX(BB.minCorner()) < 0 ||
				getY(BB.minCorner()) < 0) {
				CanPlace = false;
			}

			// 1. 原始轮廓必须在板材内，不带膨胀
			if (CanPlace && !Candidate.isInside(ABinPoly)) {
				CanPlace = false;
			}

			// 2. 间距判断：只在和其他零件相交检测时膨胀
			if (CanPlace && ASpacingCoord > 0) {
				Candidate.inflation(static_cast<decltype(OldInflation)>(ASpacingCoord));
			}

			if (CanPlace) {
				for (std::size_t i = 0; i < ANestItems.size(); ++i) {
					if (i == AItemIndex) {
						continue;
					}

					const auto& Other = ANestItems[i];

					if (Other.binId() != ATargetBin || Other.binId() < 0) {
						continue;
					}

					if (NestItemType::intersects(Candidate, Other)) {
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
		static Point CalcTranslationForBBoxMin(CetTNestItemVector& ANestItems,std::size_t AItemIndex,libnest2d::Radians AAngle,double ATargetMinX,double ATargetMinY)
		{
			auto& Item = ANestItems[AItemIndex];

			Point OldTranslation = Item.translation();
			libnest2d::Radians OldRotation = Item.rotation();

			// 先把零件放回原点，只设置旋转
			Item.translation(Point(0, 0));
			Item.rotation(AAngle);

			auto BB = Item.boundingBox();

			Point DesiredMin(NestUtils::ToNestCoord(ATargetMinX),NestUtils::ToNestCoord(ATargetMinY));

			// 关键：translation 要补偿旋转后包围盒的 minCorner
			Point Result = DesiredMin - BB.minCorner();

			Item.translation(OldTranslation);
			Item.rotation(OldRotation);

			return Result;
		}
		static bool TryPlaceNestItemInBinByGrid(CetTNestItemVector& ANestItems,std::size_t AItemIndex,int ATargetBin,const PolygonImpl& ABinPoly,double ABoardBinWidth,double ABoardBinHeight,const TetNestOptions& AOptions,double AStepMm)
		{
			if (AItemIndex >= ANestItems.size()) {
				return false;
			}

			std::vector<libnest2d::Radians> Rotations;
			FillRotations(Rotations, AOptions.Rotations);

			long long SpacingCoord = NestUtils::ToNestCoord(AOptions.Spacing);

			for (auto Angle : Rotations) {
				for (double Y = 0.0; Y <= ABoardBinHeight; Y += AStepMm) {
					for (double X = 0.0; X <= ABoardBinWidth; X += AStepMm) {

						Point Translation = CalcTranslationForBBoxMin(ANestItems,AItemIndex,Angle,X,Y);

						if (CanPlaceNestItemAt(ANestItems,AItemIndex,ATargetBin,ABinPoly,Translation,Angle,SpacingCoord)) {
							ANestItems[AItemIndex].translation(Translation);
							ANestItems[AItemIndex].rotation(Angle);
							ANestItems[AItemIndex].binId(ATargetBin);

							std::cout << "[REPAIR] item "
								<< AItemIndex
								<< " moved to bin "
								<< ATargetBin
								<< ", x = "
								<< X
								<< ", y = "
								<< Y
								<< std::endl;

							return true;
						}
					}
				}
			}

			return false;
		}
		static void RepairPolygonBoardPacking(CetTNestItemVector& ANestItems,const TetNestOptions& AOptions,const PolygonImpl& ABinPoly,double ABoardBinWidth,double ABoardBinHeight,std::size_t& ALayers)
		{
			if (ALayers <= 1) {
				return;
			}

			if (!AOptions.Board.Enabled || AOptions.Board.Vertices.size() < 3) {
				return;
			}

			// 第一版先用比较粗的网格，避免太慢。
			// 你这个 100x100 的测试，Step = 1 或 2 都可以。
			double StepMm = std::max(1.0, AOptions.Spacing);

			std::cout << "[REPAIR] start polygon board repair. Layers = "
				<< ALayers
				<< ", StepMm = "
				<< StepMm
				<< std::endl;

			bool Changed = true;

			while (Changed) {
				Changed = false;

				for (std::size_t i = 0; i < ANestItems.size(); ++i) {
					int OriginalBin = static_cast<int>(ANestItems[i].binId());

					if (OriginalBin <= 0) {
						continue;
					}

					Point OldTranslation = ANestItems[i].translation();
					libnest2d::Radians OldRotation = ANestItems[i].rotation();

					bool Placed = false;

					for (int TargetBin = 0; TargetBin < OriginalBin; ++TargetBin) {
						if (TryPlaceNestItemInBinByGrid(ANestItems,i,TargetBin,ABinPoly,ABoardBinWidth,ABoardBinHeight,AOptions,StepMm
						)) {
							Placed = true;
							Changed = true;
							break;
						}
					}

					if (!Placed) {
						ANestItems[i].translation(OldTranslation);
						ANestItems[i].rotation(OldRotation);
						ANestItems[i].binId(OriginalBin);
					}
				}
			}

			ALayers = CompactNestItemBins(ANestItems);

			std::cout << "[REPAIR] finish polygon board repair. Layers = "
				<< ALayers
				<< std::endl;
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
		
			const bool UsePolygonBoard =AOptions.Board.Enabled &&AOptions.Board.Vertices.size() >= 3;

			int TotalItems = static_cast<int>(ANestItems.size());
			TetNestProgressTracker Tracker(TotalItems, AOptions.ProgressCallback);

			std::size_t Layers = 0;

			if (UsePolygonBoard) {
				std::cout << "[NEST] use custom polygon board" << std::endl;

				double BoardBinWidth = AOptions.BinWidth;
				double BoardBinHeight = AOptions.BinHeight;

				PolygonImpl binPoly = BuildBinPolygonFromOptions(
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
				std::cout << "[NEST] before repair, Layers = "
					<< Layers
					<< std::endl;

				RepairPolygonBoardPacking(
					ANestItems,
					AOptions,
					binPoly,
					BoardBinWidth,
					BoardBinHeight,
					Layers
				);

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