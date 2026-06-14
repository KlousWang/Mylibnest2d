#include "pch.h"
#include "Nest2DAPI.h"
#include"NestUtils.h"
#include"NestDataMapper.h"
#include"Nest2D_DataConst.h"

#include"Nest2D_SelfFunction.h"

#include <limits>
#include <stdexcept>
#include <algorithm>


using namespace ClipperLib;
using namespace libnest2d;

namespace ET {
	namespace NEST2DMANAGERLIB {

		//using TNestItemVector = std::vector<libnest2d::Item>;

		static CetTNestItemVector* AsNestItems(TetLib2DItemDataType* AData)
		{
			return reinterpret_cast<CetTNestItemVector*>(AData);
		}

		static const CetTNestItemVector* AsNestItems(const TetLib2DItemDataType* AData)
		{
			return reinterpret_cast<const CetTNestItemVector*>(AData);
		}
		CetNest2DManager::CetNest2DManager():CetCoreObject()
		{
			//_Lib2DItemDataType = reinterpret_cast<TetLib2DItemDataType*>(new CetTNestItemVector());
			_Lib2DItemDataType = (TetLib2DItemDataType*)(new CetTNestItemVector());
			
			std::cout << "CetNest2DManager Constructor" << std::endl;
		}

		CetNest2DManager::~CetNest2DManager()
		{
			delete AsNestItems(_Lib2DItemDataType);
			_Lib2DItemDataType = nullptr;
		}
        static TetNestPoint TransformPoint(const TetNestPoint& P,double X,double Y,double Angle)
        {
            TetNestPoint R;

            double C = std::cos(Angle);
            double S = std::sin(Angle);

            R.X = P.X * C - P.Y * S + X;
            R.Y = P.X * S + P.Y * C + Y;

            return R;
        }

        static bool PointInPolygon(const TetNestPoint& P,const std::vector<TetNestPoint>& Polygon)
        {
            bool Inside = false;
            size_t Count = Polygon.size();

            if (Count < 3) {
                return false;
            }

            for (size_t i = 0, j = Count - 1; i < Count; j = i++) {
                const auto& Pi = Polygon[i];
                const auto& Pj = Polygon[j];

                bool Intersect =
                    ((Pi.Y > P.Y) != (Pj.Y > P.Y)) &&
                    (P.X < (Pj.X - Pi.X) * (P.Y - Pi.Y) / (Pj.Y - Pi.Y + 1e-12) + Pi.X);

                if (Intersect) {
                    Inside = !Inside;
                }
            }

            return Inside;
        }

        static bool IsPointInsideBoard(const TetNestPoint& P,const TetNestBoard& Board)
        {
            if (!PointInPolygon(P, Board.Vertices)) {
                return false;
            }

            for (const auto& Hole : Board.Holes) {
                if (PointInPolygon(P, Hole)) {
                    return false;
                }
            }

            return true;
        }

        static void ValidateItemsInsideBoard(std::vector<TetNestPolygon>& AItems,const TetNestBoard& Board)
        {
            if (!Board.Enabled || Board.Vertices.size() < 3) {
                return;
            }

            for (auto& Item : AItems) {
                if (Item.Out_bin < 0) {
                    continue;
                }

                bool Valid = true;

                for (const auto& P : Item.Vertices) {
                    TetNestPoint TP = TransformPoint(
                        P,
                        Item.Out_x,
                        Item.Out_y,
                        Item.Out_angle
                    );

                    if (!IsPointInsideBoard(TP, Board)) {
                        Valid = false;
                        break;
                    }
                }

                if (!Valid) {
                    Item.Out_bin = -1;
                    Item.Out_x = 0.0;
                    Item.Out_y = 0.0;
                    Item.Out_angle = 0.0;
                }
            }
        }

        static std::size_t RecalcUsedBinsFromItems(  std::vector<TetNestPolygon>& AItems )
        {
            std::map<int, int> Remap;
            int NextBin = 0;

            for (auto& Item : AItems) {
                if (Item.Out_bin < 0) {
                    continue;
                }

                auto It = Remap.find(Item.Out_bin);
                if (It == Remap.end()) {
                    Remap[Item.Out_bin] = NextBin;
                    Item.Out_bin = NextBin;
                    ++NextBin;
                }
                else {
                    Item.Out_bin = It->second;
                }
            }

            return static_cast<std::size_t>(NextBin);
        }
        static TetBoardBounds CalcBoardBounds(const TetNestBoard& Board)
        {
            TetBoardBounds B;

            if (!Board.Enabled || Board.Vertices.size() < 3) {
                return B;
            }

            B.MinX = B.MaxX = Board.Vertices[0].X;
            B.MinY = B.MaxY = Board.Vertices[0].Y;

            for (const auto& P : Board.Vertices) {
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
		int CetNest2DManager::PerformNestingEx(std::vector<TetNestPolygon>& AItems, const TetNestOptions& AOptions, TetNestResult* AResult)
		{
			//Nest2DUtils->WWFunct1(1);
			//Nest2DLibConfig->CreateCoreObj();
			if (AResult) {
				AResult->Code = 0;
				AResult->UsedBins = 0;
				AResult->Message.clear();
			}
			
			if (AItems.empty()) return  NEST2D_ERR_CORE_EMPTY_INPUT;
			if (AOptions.BinHeight <= 0 || AOptions.BinWidth <= 0) return  NEST2D_ERR_CORE_INVALID_SIZE;
			//Box bin(NestUtils::ToNestCoord(AOptions.BinWidth), NestUtils::ToNestCoord(AOptions.BinHeight));
			//std::vector<Item> nestItems = CetNestDataMapper::BuildNestItems(AItems);
			CetTNestItemVector* NestItemsPtr = AsNestItems(_Lib2DItemDataType);
			if (NestItemsPtr == nullptr)return NEST2D_ERR_CORE_NESTING_FAILED;
			CetTNestItemVector& NestItems = *NestItemsPtr;
			NestItems.clear();
			Nest2DUtils->BuildNestms(AItems,NestItems);
            std::cout << "[DEBUG] AItems.size = " << AItems.size()
                << ", NestItems.size = " << NestItems.size()
                << std::endl;

            if (NestItems.size() != AItems.size()) {
                if (AResult) {
                    AResult->Code = NEST2D_ERR_CORE_NESTING_FAILED;
                    AResult->Message = "BuildNestItems size mismatch.";
                }
                return NEST2D_ERR_CORE_NESTING_FAILED;
            }
			 //int i  =Nest2DUtils->BuildNestms.GetResult();
			 //std::cout << i << std::endl;
			std::size_t UsedBins = 0;
			int NestCode = Nest2DUtils->RunNestingFunctor(NestItems, AOptions, &UsedBins);

			if (NestCode != Nest2D_Success)return NestCode;

			Nest2DUtils->ApplyResults(NestItems, AItems);

			if (AOptions.Board.Enabled) {
                TetBoardBounds BoardBounds = CalcBoardBounds(AOptions.Board);
                if (!BoardBounds.Valid) {
                    if (AResult) {
                        AResult->Code = NEST2D_ERR_CORE_INVALID_SIZE;
                        AResult->Message = "Invalid custom board.";
                    }
                    return NEST2D_ERR_CORE_INVALID_SIZE;
                }
                // 如果底板不是从 0,0 开始，把排版结果平移到底板真实坐标系
                if (BoardBounds.MinX != 0.0 || BoardBounds.MinY != 0.0) {
                    for (auto& Item : AItems) {
                        if (Item.Out_bin >= 0) {
                            Item.Out_x += BoardBounds.MinX;
                            Item.Out_y += BoardBounds.MinY;
                        }
                    }
                }
				ValidateItemsInsideBoard(AItems, AOptions.Board);
                UsedBins = RecalcUsedBinsFromItems(AItems);
			}
			if (AOptions.ExportSvg) {
				
				//CetExportPhoto::ExportSvg(AItems, AOptions, static_cast<int>(layers)); // 调用我们之前拆出来的函数
				Nest2DUtils->ExportSvgbd(AItems, AOptions, static_cast<int>(UsedBins));
			}

			if (AResult) {
				AResult->UsedBins = UsedBins;
				AResult->Message = "Nesting success.";
			}
			return 0;
		}
		
	}
}
