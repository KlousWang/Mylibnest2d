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
			 //int i  =Nest2DUtils->BuildNestms.GetResult();
			 //std::cout << i << std::endl;
			std::size_t UsedBins = 0;
			int NestCode = Nest2DUtils->RunNestingFunctor(NestItems, AOptions, &UsedBins);

			if (NestCode != Nest2D_Success)return NestCode;

			Nest2DUtils->ApplyResults(NestItems, AItems);
			if (AOptions.ExportSvg) {
				
				//CetExportPhoto::ExportSvg(AItems, AOptions, static_cast<int>(layers)); // 调用我们之前拆出来的函数
				Nest2DUtils->ExportSvg(NestItems, AOptions, static_cast<int>(UsedBins));
			}

			if (AResult) {
				AResult->UsedBins = UsedBins;
				AResult->Message = "Nesting success.";
			}
			return 0;
		}
		
	}
}
