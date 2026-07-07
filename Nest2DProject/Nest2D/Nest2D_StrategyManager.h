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
			TetTNestEvalResult EvaluateNestResult(const CetTNestItemVector& Items, std::size_t Layers);
			bool IsBetterNestResult(const TetTNestEvalResult& A, const TetTNestEvalResult& B);
			void ApplyNestPriorityStrategy(CetTNestItemVector& AItems, MetENestOrderStrategy AStrategy);
			void PrintBinCount(const CetTNestItemVector& AItems);
			TetTNestEvalResult EvaluatePackedResultWithMeta(const CetTNestItemVector& AItems, const std::vector<TetMetaItem>& AMetaItems, const CetTNestItemVector& AOriginalItems, std::size_t ALayers);
		};
	}
}

