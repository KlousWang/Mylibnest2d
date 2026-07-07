#pragma once
#include "EtTechCore_Object.h"
#include "NestUtils.h"
#include "Nest2D_PrivateDataType.h"
#include "Nest2D_DataType.h"

#include <vector>
#include <string>
#include <map>

namespace ET {
	namespace NEST2DMANAGERLIB {
		class CetClusterManager :public ET::CORE::CetCoreObject
		{
			Inherit_Invoke_Hook(CetClusterManager)

		protected:
			int _Init() override {
				CetCoreObject::_Init();
				return 0;
			}
			void _WrapFuncs() override {
				CetCoreObject::_WrapFuncs();
				_WrapFunc("BuildClusterItems", Type_Class_Func(BuildClusterItems));
				_WrapFunc("ExpandClusterResultToOriginalItems", Type_Class_Func(ExpandClusterResultToOriginalItems));

			}

		public:
			CetClusterManager();
			~CetClusterManager();

		public:
			TetClusterBuildResult BuildClusterItems(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, MetClusterStrategy AStrategy);
			void ExpandClusterResultToOriginalItems(const CetTNestItemVector& AOriginalItems,const CetTNestItemVector& APackedItems,const std::vector<TetMetaItem>& AMetaItems,CetTNestItemVector& AOutOriginalItems);

		protected:
			void AddSingleItem(const CetTNestItemVector& AOriginalItems, int AOriginalIndex, TetClusterBuildResult& AResult);
			bool TryMakeRightTrianglePair(const CetTNestItemVector& AOriginalItems, int AIndex, int BIndex, const TetNestOptions& AOptions, TetClusterBuildResult& AResult);
			bool NearlyEqual(double A, double B, double RelTol);
			double GetItemWidth(const CetNestItem& AItem);
			double GetItemHeight(const CetNestItem& AItem);
			bool IsRightTriangleLike(const CetNestItem& AItem);
			bool IsSameSizeTrianglePair(const CetNestItem& AItem, const CetNestItem& BItem);
			CetNestItem MakeRectangleNestItemByNestCoord(double AW, double AH);
			double CalcTrianglePairAxisGap(double AW, double AH, double ASpacing);
			//void ExpandOnePackedItemToOriginalItems(std::size_t APackedIndex,const CetNestItem& APackedItem,const TetMetaItem& AMeta,CetTNestItemVector& AOutOriginalItems);
			void ExpandClusterChildren(const CetNestItem& PackedItem,const TetMetaItem& Meta,CetTNestItemVector& AOutOriginalItems);
		};
	}
}
