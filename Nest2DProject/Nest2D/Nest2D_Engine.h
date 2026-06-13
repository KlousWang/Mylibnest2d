#pragma once
#include "Nest2D_DataType.h" // °üº¬ TNestItemVector
#include "Nest2D_PrivateDataType.h"
#include "EtTechCore_Object.h"

namespace ET {
    namespace NEST2DMANAGERLIB {
       
        class CetNest2DEngine : public ET::CORE::CetCoreObject
        {
            Inherit_Invoke_Hook(CetNest2DEngine)

        protected:
            int _Init() override { 
                CetCoreObject::_Init(); 
                return 0; 
            }
            void _WrapFuncs() override {
                CetCoreObject::_WrapFuncs(); 
                _WrapFunc("RunNesting", Type_Class_Func(RunNesting_Impl));
            }

        public:
            CetNest2DEngine();
            ~CetNest2DEngine();

            int RunNesting_Impl(CetTNestItemVector& ANestItems, const TetNestOptions& AOptions, std::size_t* AUsedBins);
        };
    }
}