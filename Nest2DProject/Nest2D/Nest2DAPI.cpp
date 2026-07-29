#include "pch.h"
#include "Nest2DAPI.h"
#include"NestUtils.h"
#include"NestDataMapper.h"
#include"Nest2D_DataConst.h"

#include"Nest2D_SelfFunction.h"
#include"Nest2D_AreaUsageCalculator.h"

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

		int CetNest2DManager::PerformNestingEx(std::vector<TetNestPolygon>& AItems, const TetNestOptions& AOptions, TetNestResult* AResult)
		{
		
			if (AResult) {
				AResult->Code = 0;
				AResult->UsedBins = 0;
                AResult->BoardUsages.clear();
				AResult->Message.clear();
			}
			
			if (AItems.empty()) return  NEST2D_ERR_CORE_EMPTY_INPUT;
			if (AOptions.BinHeight <= 0 || AOptions.BinWidth <= 0) return  NEST2D_ERR_CORE_INVALID_SIZE;
			//Box bin(NestUtils::ToNestCoord(AOptions.BinWidth), NestUtils::ToNestCoord(AOptions.BinHeight));
			//std::vector<Item> nestItems = CetNestDataMapper::BuildNestItems(AItems);
			CetTNestItemVector* NestItemsPtr = AsNestItems(_Lib2DItemDataType);//准备内部数据
			if (NestItemsPtr == nullptr)return NEST2D_ERR_CORE_NESTING_FAILED;
			CetTNestItemVector& NestItems = *NestItemsPtr;
			NestItems.clear();
			// ==================== 对源数据进行排序 ====================
			std::cout << "[NEST] Sorting working copy by Bounding Box Area (Descending)..." << std::endl;
			std::vector<std::pair<std::size_t, TetNestPolygon>> SortedPairs;
			SortedPairs.reserve(AItems.size());
			for (std::size_t ItemIndex = 0; ItemIndex < AItems.size(); ++ItemIndex) {
				SortedPairs.push_back({ ItemIndex, AItems[ItemIndex] });
			}
			std::stable_sort(SortedPairs.begin(), SortedPairs.end(), [&](const auto& Left, const auto& Right) {
				const bool LeftBeforeRight = Nest2DUtils->Nest2DGeometryUtils->ComparePolygonAreaDesc(Left.second, Right.second);
				const bool RightBeforeLeft = Nest2DUtils->Nest2DGeometryUtils->ComparePolygonAreaDesc(Right.second, Left.second);
				if (LeftBeforeRight != RightBeforeLeft) {
					return LeftBeforeRight;
				}
				return Left.first < Right.first;
				});

			std::vector<TetNestPolygon> SortedItems;
			std::vector<std::size_t> SortedToOriginal;
			SortedItems.reserve(SortedPairs.size());
			SortedToOriginal.reserve(SortedPairs.size());
			for (const auto& Entry : SortedPairs) {
				SortedToOriginal.push_back(Entry.first);
				SortedItems.push_back(Entry.second);
			}
			// ==========================================================================

			Nest2DUtils->NestDataMapperIns->BuildNestItems(SortedItems,NestItems);
            std::cout << "[DEBUG] SortedItems.size = " << SortedItems.size()
                << ", NestItems.size = " << NestItems.size()
                << std::endl;

            if (NestItems.size() != SortedItems.size()) {
                if (AResult) {
                    AResult->Code = NEST2D_ERR_CORE_NESTING_FAILED;
                    AResult->Message = "BuildNestItems size mismatch.";
                }
                return NEST2D_ERR_CORE_NESTING_FAILED;
            }
			 //int i  =Nest2DUtils->BuildNestms.GetResult();
			 //std::cout << i << std::endl;
			std::size_t UsedBins = 0;
			//int NestCode = Nest2DUtils->Nest2DEngineIns->RunNesting_Impl(NestItems, AOptions, &UsedBins);
			int NestCode = Nest2DUtils->PerformNestingEx(NestItems, AOptions, &UsedBins);
			
			//int NestCode = Nest2DUtils->RunNestingFunctor(NestItems, AOptions, &UsedBins);
			if (NestCode != Nest2D_Success)return NestCode;

			Nest2DUtils->NestDataMapperIns->ApplyResults(NestItems, SortedItems);
			for (std::size_t SortedIndex = 0; SortedIndex < SortedItems.size() && SortedIndex < SortedToOriginal.size(); ++SortedIndex) {
				const std::size_t OriginalIndex = SortedToOriginal[SortedIndex];
				if (OriginalIndex >= AItems.size()) {
					continue;
				}
				AItems[OriginalIndex].Out_bin = SortedItems[SortedIndex].Out_bin;
				AItems[OriginalIndex].Out_x = SortedItems[SortedIndex].Out_x;
				AItems[OriginalIndex].Out_y = SortedItems[SortedIndex].Out_y;
				AItems[OriginalIndex].Out_angle = SortedItems[SortedIndex].Out_angle;
			}
            for (const auto& Item : AItems) {  
                std::cout << "[RESULT] item id = " << Item.Id
                    << ", bin = " << Item.Out_bin
                    << ", x = " << Item.Out_x
                    << ", y = " << Item.Out_y
                    << ", angle = " << Item.Out_angle
                    << std::endl;
            }
			if (AOptions.Board.Enabled) {
                TetBoardBounds BoardBounds = Nest2DUtils->Nest2DBord->CalcBoardBoundsLocal(AOptions.Board);
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
			   Nest2DUtils->Nest2DGeometryUtils->ValidateItemsInsideBoard(AItems, AOptions.Board);
                UsedBins = RecalcUsedBinsFromItems(AItems);
			}
            CetAreaUsageCalculator LocalUsageCalculator;
            CetAreaUsageCalculator* UsageCalculator = Nest2DUtils->Nest2DAreaUsage != nullptr ? Nest2DUtils->Nest2DAreaUsage : &LocalUsageCalculator;
			//Nest2DUtils->Nest2DAreaUsage->CalculateBoardUsages(AItems, AOptions, static_cast<int>(UsedBins));
            std::vector<TetBoardUsageResult> BoardUsages = UsageCalculator->CalculateBoardUsages(AItems, AOptions, static_cast<int>(UsedBins));

			if (AOptions.ExportSvg) {
				
				//CetExportPhoto::ExportSvg(AItems, AOptions, static_cast<int>(layers)); // 调用我们之前拆出来的函数
				Nest2DUtils->Nest2DExportPhoto->ExportSvg(AItems, AOptions, static_cast<int>(UsedBins));
			}

			if (AResult) {
				AResult->UsedBins = UsedBins;
                AResult->BoardUsages = BoardUsages;
				AResult->Message = "Nesting success.";
			}
			return 0;
		}

		void CetNest2DManager::SortItemsByAreaDesc(std::vector<TetNestPolygon>& AItems)
		{
			Nest2DUtils->Nest2DSortIns->Sort(AItems);
			/*std::sort(AItems.begin(), AItems.end(), [&](const TetNestPolygon& ADataa, const TetNestPolygon& ADatab) {
				return Nest2DUtils->ComparePolygonAreaDesc(ADataa, ADatab);
				});*/
		}
		
	}
}
