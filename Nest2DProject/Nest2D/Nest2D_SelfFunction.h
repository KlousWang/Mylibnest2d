#pragma once
#include "EtTechCore_InvokeFunctorMacro.h"
#include "Nest2D_InvokeFunctor.h"
// 2. 引入你刚刚写好的“遥控器”头文件

namespace ET {
	namespace NEST2DMANAGERLIB {
		Def_InvokeFunctor_Builder(CetNest2DUtils, "EtCore_LibConfig", "Nest2D_InvokeFunctor", "D:/Work/Project/libnest2d/KlousProject/Nest2DProject/Nest2DDLL.json")

#define Nest2DUtils         Def_Invoker_Functor(CetNest2DUtils, CetNest2DInvokeFunctor)
#define Nest2DLibConfig     Def_Invoker_LibConfig(CetNest2DUtils)
	}
}
