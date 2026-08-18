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

int main()
{
	CetCoreAppConfig* AppConfig = (decltype(AppConfig))CetCoreObject::CreateIns("EtCore_AppConfig");
	if (!AppConfig) {
		std::cout << "Create application configuration failed." << std::endl;
		return -1;
	}

	int loadResult = AppConfig->LoadAll("../Libnest2d.json");
	if (loadResult < 0) {
		std::cout << "Load library configuration failed: " << loadResult << std::endl;
		return loadResult;
	}

	int globalconfig = AppConfig->CreateGlobalVars();
	if (globalconfig < 0) {
		std::cout << "Create global instances failed: " << globalconfig << std::endl;
		return globalconfig;
	}

	auto tmpObj1 = CetCoreObjStorage::GetClassIns("gCreateTestData");
	auto tmpObj2 = CetCoreObjStorage::GetClassIns("gFile");
	auto tmpObj3 = CetCoreObjStorage::GetClassIns("gNest2D");
	if (!tmpObj2) {
		tmpObj2 = CetCoreObject::CreateIns("File_Load");
		if (tmpObj2) {
			CetCoreObjStorage::SaveClassIns("gFile", tmpObj2);
		}
	}
	if (!tmpObj3) {
		tmpObj3 = CetCoreObject::CreateIns("Nest_2D");
		if (tmpObj3) {
			CetCoreObjStorage::SaveClassIns("gNest2D", tmpObj3);
		}
	}
	if (!tmpObj1 || !tmpObj2 || !tmpObj3) {
		std::cout << "Required global object is unavailable. Check Libnest2d.json and Nest2DDLL.json." << std::endl;
		return -1;
	}
    auto* TestApp =static_cast<ET::NEST2DTESTAPP::CetTestApp*>(CetCoreObject::CreateIns("Nest2DTestApp"));
    if (!TestApp) {
        std::cout << "Create Nest2DTestApp failed." << std::endl;
        return -1;
    }
    auto* MenuRunner =static_cast<CetMainMenuRunner*>(CetCoreObject::CreateIns("MainMenuRunner"));
    if (!MenuRunner) {
        std::cout << "Create MainMenuRunner failed." << std::endl;
        return -1;
    }
   // ET::NEST2DTESTAPP::Nest2DLibConfig->GetClassFuncName("","");
    if (MenuRunner->SetTestApp(TestApp) != 0) {
        return -1;
    }

    MenuRunner->Run();

    return 0;
	//================
}

