#include "EtTechCore_AppConfig.h"
#include "EtTechCore_SelfFunction.h"
#include "EtTechCore_Functor.h"
#include "Nest2D_DataType.h"
#include"NestTestData_DataType.h"
#include"Nest2DTest_SelfFunction.h"
#include"Nest2DTestApp.h"
#include"MainMenuRunner.h"
#include"MenuRunnerBase.h"
using namespace ET::CORE;

int main() {
	CetCoreAppConfig* AppConfig = (decltype(AppConfig))CetCoreObject::CreateIns("EtCore_AppConfig");
	//std::cout << "AppConfig ptr: " << (void*)AppConfig << std::endl;
	int loadResult = AppConfig->LoadAll("../Libnest2d.json");
	//std::cout << "config " << config << std::endl;
	int globalconfig= AppConfig->CreateGlobalVars();
	//std::cout << "global config " << globalconfig << std::endl;
	auto tmpObj1 = CetCoreObjStorage::GetClassIns("gCreateTestData");
	auto tmpObj2 = CetCoreObjStorage::GetClassIns("gFile");
	auto tmpObj3 = CetCoreObjStorage::GetClassIns("gNest2D");
	CetMainMenuRunner  menuRunner;
	menuRunner.Run();
 
	return 0;
	//================
}

