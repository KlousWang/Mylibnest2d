#include "EtTechCore_AppConfig.h"
#include "EtTechCore_SelfFunction.h"
#include "EtTechCore_Functor.h"
#include "Nest2D_DataType.h"
#include"NestTestData_DataType.h"
#include"Nest2DTest_SelfFunction.h"
#include"Nest2DTestApp.h"
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <fstream>

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

    while (true) {
        int mode = ET::NEST2DTESTAPP::ReadMainMode();
        if (mode == 0) {
            std::cout << "Exiting program." << std::endl;
            break;
        }
        std::string inputFile;

        if (mode == 1) {
            if (!ET::NEST2DTESTAPP::GenerateNestFile(inputFile)) {
                std::cout << "Generate nest file failed." << std::endl;
                continue;
            }
        }
        else if (mode == 2) {
            if (!ET::NEST2DTESTAPP::ReadNestFileName(inputFile)) {
                std::cout << "Input file is empty." << std::endl;
                continue;
            }
        }
        else {
            std::cout << "Invalid mode." << std::endl;
            continue;
        }
        int result = ET::NEST2DTESTAPP::RunNestProcess(inputFile);
        std::cout << "RunNestProcess result = " << result << std::endl;
    }
	return 0;
	//================
}

