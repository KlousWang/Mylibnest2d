#pragma once
#include"EtTechCore_Object.h"
#include"Nest2D_PrivateDataType.h"
#include"Nest2D_DataType.h"

namespace ET {
	namespace NEST2DMANAGERLIB {
		class CetStrategyManager :public ET::CORE::CetCoreObject
		{
			Inherit_Invoke_Hook(CetStrategyManager)

		protected:
			int _Init() override {
				CetCoreObject::_Init();
				return 0;
			}
			void _WrapFuncs() override {
				CetCoreObject::_WrapFuncs();
				_WrapFunc("EvaluateNestResult", Type_Class_Func(EvaluateNestResult));
				_WrapFunc("IsBetterNestResult", Type_Class_Func(IsBetterNestResult));
				_WrapFunc("ApplyNestPriorityStrategy", Type_Class_Func(ApplyNestPriorityStrategy));
				_WrapFunc("PrintBinCount", Type_Class_Func(PrintBinCount));
				_WrapFunc("EvaluatePackedResultWithMeta", Type_Class_Func(EvaluatePackedResultWithMeta));
			}
		public:
			CetStrategyManager();
			~CetStrategyManager();
		public:
			TetTNestEvalResult EvaluateNestResult(const CetTNestItemVector& AItems, std::size_t ALayers);
			bool IsBetterNestResult(const TetTNestEvalResult& A, const TetTNestEvalResult& AB);
			void ApplyNestPriorityStrategy(CetTNestItemVector& AItems, const TetNestOptions& AOptions, MetENestOrderStrategy AStrategy);
			void PrintBinCount(const CetTNestItemVector& AItems);
			TetTNestEvalResult EvaluatePackedResultWithMeta(const CetTNestItemVector& AItems, const std::vector<TetMetaItem>& AMetaItems, const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, std::size_t ALayers);
	
		protected:
			// 1. 展开 Meta 信息，提取最后一块板上所有零件的包围盒，并统计基础面积
			std::vector<TetRemnantPartBounds> _ExtractLastBinBounds(const CetTNestItemVector& AItems,const std::vector<TetMetaItem>& AMetaItems,const CetTNestItemVector& AOriginalItems,int ALastBinId,TetTNestEvalResult& AOutResult) const;

			// 2. 基于天际线算法计算余料的各项指标
			void _CalculateRemnantMetrics(const std::vector<TetRemnantPartBounds>& ALastBinBounds,const TetNestOptions& AOptions,TetTNestEvalResult& AOutResult) const;
		};
	}
}

