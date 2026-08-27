#pragma once
#include "EtTechCore_InvokeFunctorMacro.h"
#include "EtTechCore_LibConfig.h"
#include "Nest2DTest_InvokeFunctor.h"
namespace ET {
    namespace NEST2DTESTAPP {

        Def_InvokeFunctor_Builder(CetNest2DTestUtils, "EtCore_LibConfig", "Nest2DTest_InvokeFunctor", "Libnest2DTest.json")
#define Nest2DTestUtils Def_Invoker_Functor(CetNest2DTestUtils, CetNest2DTestInvokeFunctor)
#define Nest2DTestLibConfig Def_Invoker_LibConfig(CetNest2DTestUtils)

    } // namespace NEST2DTESTAPP
} // namespace ET
