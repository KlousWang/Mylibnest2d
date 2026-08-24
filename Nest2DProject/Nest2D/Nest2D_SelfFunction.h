#pragma once
#include "EtTechCore_InvokeFunctorMacro.h"
#include "Nest2D_InvokeFunctor.h"
//#include"Nest2D_ClusterInvokeFunctor.h"
#include"EtTechCore_LibConfig.h"


namespace ET {
	namespace NEST2DMANAGERLIB {
		Def_InvokeFunctor_Builder(CetNest2DUtils, "EtCore_LibConfig", "Nest2D_InvokeFunctor", "Nest2DDLL.json")

#define Nest2DUtils         Def_Invoker_Functor(CetNest2DUtils, CetNest2DInvokeFunctor)
#define Nest2DLibConfig     Def_Invoker_LibConfig(CetNest2DUtils)

//#define Nest2DClusterUtils         Def_Invoker_Functor(CetNest2DClusterUtils, CetNest2DClusterInvokeFunctor)
//#define Nest2DCluesterLibConfig     Def_Invoker_LibConfig(CetNest2DClusterUtils) 
	}
}
//namespace ET {
//	namespace NEST2DMANAGERLIB {
//		Def_InvokeFunctor_Builder(CetNest2DClusterUtils, "EtCore_LibConfig", "Nest2D_ClusterInvokeFunctor", "Nest2DDLL.json")
//
//#define Nest2DClusterUtils         Def_Invoker_Functor(CetNest2DClusterUtils, CetNest2DClusterInvokeFunctor)
//#define Nest2DCluesterLibConfig     Def_Invoker_LibConfig(CetNest2DClusterUtils)
//	}
//}