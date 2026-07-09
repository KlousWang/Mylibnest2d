#pragma once
#include"EtTechCore_Object.h"
#include"Nest2D_DataType.h"
#include<iostream>
#include<vector>
namespace ET {
	namespace NEST2DMANAGERLIB {
		class CetSort:public ET::CORE::CetCoreObject
		{
            Inherit_Invoke_Hook(CetSort)

        protected:
            int _Init() override {
                CetCoreObject::_Init();
                return 0;
            }
            void _WrapFuncs() override {
                CetCoreObject::_WrapFuncs();
              // _WrapFunc("RunNesting", Type_Class_Func(RunNesting_Impl));
            }
        public:
            CetSort();
            ~CetSort();

        public:
            void Sort(std::vector<TetNestPolygon>& AItems);
		};

	}
}

