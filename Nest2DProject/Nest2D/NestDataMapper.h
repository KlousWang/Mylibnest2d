#pragma once
#include"Nest2D_DataType.h"
#include"Nest2D_PrivateDataType.h"
#include"EtTechCore_Object.h"
#include<vector>


namespace ET {
	namespace NEST2DMANAGERLIB {
		class CetNestDataMapper : public ET::CORE::CetCoreObject
		{
			Inherit_Invoke_Hook(CetNestDataMapper)

		protected:
			int _Init() override {
				CetCoreObject::_Init();
				return 0;
			}
			void _WrapFuncs() override {
				CetCoreObject::_WrapFuncs();
				_WrapFunc("BuildNestItems", Type_Class_Func(BuildNestItems));
				_WrapFunc("ApplyResults", Type_Class_Func(ApplyResults));
			}
		public:
			CetNestDataMapper();
			virtual ~CetNestDataMapper();

		public:
			 void BuildNestItems(const std::vector<TetNestPolygon>& AItems,CetTNestItemVector& ANestItems);
			 void ApplyResults(const CetTNestItemVector& nestItems, std::vector<TetNestPolygon>& AItems);
		};

	}
}