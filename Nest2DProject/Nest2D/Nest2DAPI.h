#pragma once
#include"Nest2D_Def.h"
#include"Nest2D_DataType.h"
#include"Nest2D_PrivateDataType.h"
#include"Nest2D_Engine.h"
#include"EtTechCore_Object.h"
//#include"EtTechCore_Component.h"
#include<vector>
#include<string>
//#include <libnest2d/libnest2d.hpp>
//#include "Nest2DAPI.cpp"
namespace ET {
	namespace NEST2DMANAGERLIB {
		class NEST2D_API CetNest2DManager : public ET::CORE::CetCoreObject
		{
			Inherit_Invoke_Hook(CetNest2DManager)

		protected:
			int _Init()override { CetCoreObject::_Init(); return 0; }
			void _WrapFuncs()override {
				CetCoreObject::_WrapFuncs();
				_WrapFunc("PerformNestingEx", Type_Class_Func(PerformNestingEx));
			}

		public:
			CetNest2DManager();
			~CetNest2DManager();
		public:
			int PerformNestingEx(std::vector<TetNestPolygon>& AItems, const TetNestOptions& AOptions, TetNestResult* AResult = nullptr);

		protected:
			TetLib2DItemDataType* _Lib2DItemDataType = nullptr;
		};
	}
}