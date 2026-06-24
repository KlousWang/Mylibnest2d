#include "Nest2DTest_InvokeFunctor.h"
#include"Nest2DTestApp.h"
#include "MainMenuRunner.h"
#include "BoardMenuRunner.h"
#include "ShapeMenuRunner.h"

#include "NestExportManager.h"
#include "SvgExporter.h"
#include "CoordinateFileExporter.h"
namespace ET {
	namespace NEST2DTESTAPP {
		using namespace ET::CORE;
		Reg_EtCore_Obj_Str(Nest2DTest_InvokeFunctor, CetNest2DTestInvokeFunctor)
		Reg_EtCore_Obj_Str(Nest2DTestApp, CetTestApp)
			//Reg_EtCore_Obj_Str(Nest2DTest_InvokeFunctor, CetNest2DTestInvokeFunctor);
		Reg_EtCore_Obj_Str(BoardMenuRunner, CetBoardMenuRunner)
		Reg_EtCore_Obj_Str(ShapeMenuRunner, CetShapeMenuRunner)
		Reg_EtCore_Obj_Str(CoordinateFileExporter, CetCoordinateFileExporter)
	}
}
using namespace ET::CORE;
Reg_EtCore_Obj_Str(MainMenuRunner, CetMainMenuRunner)
Reg_EtCore_Obj_Str(NestExportManager, CetNestExportManager)
Reg_EtCore_Obj_Str(SvgExporter, CetSvgExporter)