#include "EtTechCore_AppConfig.h"
#include "EtTechCore_SelfFunction.h"
#include "EtTechCore_Functor.h"
#include "Nest2D_DataType.h"

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <fstream>

using namespace ET::CORE;

void ShowTitle() {
	std::cout << "Please select model" << std::endl;
	std::cout << "0、Exit App" << std::endl;
	std::cout << "1、Generate and format test graphics" << std::endl;
	std::cout << "2、Read and format an existing file" << std::endl;
	std::cout << "Please enter" << std::endl;
}
void ShowAddShapeTitle() {
	std::cout << std::endl;
	std::cout << "Please select shape type:" << std::endl;
	std::cout << "1. Triangle" << std::endl;
	std::cout << "2. Rectangle" << std::endl;
	std::cout << "3. Circle" << std::endl;
	std::cout << "4.With Holes" << std::endl;
	std::cout << "0. Finish adding shapes" << std::endl;
	std::cout << "Please enter: ";
}
int main() {
	CetCoreAppConfig* AppConfig = (decltype(AppConfig))CetCoreObject::CreateIns("EtCore_AppConfig");
	//std::cout << "AppConfig ptr: " << (void*)AppConfig << std::endl;
	int loadResult = AppConfig->LoadAll("../Libnest2d.json");
	//std::cout << "load  " << loadResult << std::endl;
	int config =AppConfig->CreateGlobalVars();
	//std::cout << "config " << config << std::endl;
	int globalconfig= AppConfig->CreateGlobalVars(nullptr);
	//std::cout << "global config " << globalconfig << std::endl;
	auto tmpObj1 = CetCoreObjStorage::GetClassIns("gCreateTestData");
	auto tmpObj2 = CetCoreObjStorage::GetClassIns("gFile");
	auto tmpObj3 = CetCoreObjStorage::GetClassIns("gNest2D");
	std::cout << "polyIns (g_PolygonIns) ptr: " << tmpObj1 << std::endl;
	CetCoreObjFunctor<int(double, double, double, int)>Init;
	CetCoreObjFunctor<void(int, double, double, bool) > AddTrigle;
	CetCoreObjFunctor<void(int, double, double, double) > AddCustomTrigle;
	CetCoreObjFunctor<void(int, double, double) > AddRectangle;
	CetCoreObjFunctor<void(int, double, bool, double, double) > AddCircle;
	CetCoreObjFunctor<void() > ClearData;
	CetCoreObjFunctor<void(int) > AddCustomShapeWithHolesByInput;
	CetCoreObjFunctor<bool(const std::string&)>SaveToFile;
	CetCoreObjFunctor<int(const std::string&, TetNestOptions&, std::vector<TetNestPolygon>&, std::string*)>LoadFile;
	CetCoreObjFunctor<int(const std::string&, const TetNestOptions& , const std::vector<TetNestPolygon>&, int)>SaveCoordinatesFile;
	CetCoreObjFunctor<int(std::vector<TetNestPolygon>&, const TetNestOptions&, TetNestResult*)>PerformNest;

	Init.Reload(tmpObj1, "Init");
	AddTrigle.Reload(tmpObj1, "AddTriangle");
	AddCustomTrigle.Reload(tmpObj1, "AddTriangleBySides");
	AddRectangle.Reload(tmpObj1, "AddRectangle");
	AddCircle.Reload(tmpObj1, "AddCircle");
	ClearData.Reload(tmpObj1, "ClearPolygons");
	AddCustomShapeWithHolesByInput.Reload(tmpObj1, "AddCustomShapeWithHolesByInput");
	SaveToFile.Reload(tmpObj1, "SaveToFile");
	LoadFile.Reload(tmpObj2, "LoadNestCaseFromFile");
	SaveCoordinatesFile.Reload(tmpObj2, "SaveNestResultToFile");
	PerformNest.Reload(tmpObj3, "PerformNestingEx");
	while (true) {
		TetNestOptions Options;
		TetNestResult result;
		std::vector<TetNestPolygon> Items;
		std::string ErrorMessage;

		int mode = 0;
		std::string saveFile;
		std::string inputFile;
		std::string svgPath;
		std::string FilePath;

		ShowTitle();
		std::cin >> mode;
		if (mode == 0) {
			std::cout << "Exiting program." << std::endl;
			break;
		}
		else if (mode == 1) {
			double binWidth = 0;
			double binHeight = 0;
			double spacing = 0;
			int rotations = 0;
			int triangleCount = 0;
			double triangleWidth = 0;
			double TriangleHeight = 0;
			bool randomPosition = true;
			std::cout << "Please enter bin Width：";
			std::cin >> binWidth;
			std::cout << "Please enter bin height：";
			std::cin >> binHeight;
			std::cout << "Please enter spacing：";
			std::cin >> spacing;
			std::cout << "Please enter rotation number,for example 4：";
			std::cin >> rotations;
			double minOtherItemSize = 0;
			bool hasOtherItems = false;
			int initCode = Init(binWidth, binHeight, spacing, rotations);
			std::cout << "init result = " << initCode << std::endl;
			ClearData();
			while (true) {
				int shapeType = 0;
				ShowAddShapeTitle();
				std::cin >> shapeType;
				if (shapeType == 0) {
					break;
				}
				int count = 0;
				double width = 0.0;
				double height = 0.0;
				double line1 = 0.0;
				double line2 = 0.0;
				double line3 = 0.0;
				double radius = 0.0;
				double ratio = 0.0;
				std::cout << "Please enter shape count: ";
				std::cin >> count;
		
				if (shapeType == 1) {
					int typeTri = 0;
					std::cout << "If you want to generate a custom triangle, enter 1; for anything else, enter 2:" << std::endl;
					std::cin >> typeTri;
					if (typeTri == 1) {
						std::cout << "Please enter shape width: ";
						std::cin >> width;
						std::cout << "Please enter shape height: ";
						std::cin >> height;	
						double curMin = std::min(width, height);
						if (curMin > minOtherItemSize) {

							minOtherItemSize = curMin;
						}
						for (int i = 0; i < count; ++i) {
							AddTrigle(i + 1, width, height, randomPosition);
						}
						std::cout << "Added triangle count = " << count << std::endl;	
					}
					else
					{
						std::cout << "Please enter shape line1: ";
						std::cin >> line1;
						std::cout << "Please enter shape line2: ";
						std::cin >> line2;
						std::cout << "Please enter shape line3: ";
						std::cin >> line3;
						
						double curMin = std::min({ line1, line2, line3 });

						if (curMin > minOtherItemSize) {
							minOtherItemSize = curMin;
						}
						for (int i = 0; i < count; ++i) {
							AddCustomTrigle(i + 1, line1, line2, line3);
						}
						std::cout << "Added triangle count = " << count << std::endl;
					
					}
					
				}
				else if (shapeType == 2) {
					std::cout << "Please enter shape width: ";
					std::cin >> width;
					std::cout << "Please enter shape height: ";
					std::cin >> height;
				
					double curMin = std::min(width, height);
					if (curMin > minOtherItemSize) {

						minOtherItemSize = curMin;
					}
					for (int i = 0; i < count; ++i) {
						AddRectangle(i + 1, width, height);
					}
					std::cout << "Added rectangle count = " << count << std::endl;
					
				}
				else if (shapeType == 3) {
					std::cout << "Please enter shape radius: ";
					std::cin >> radius;
					std::cout << "Please enter shape ratio: ";
					std::cin >> ratio;
					
					if (minOtherItemSize == 0) {
						hasOtherItems = false;
					}
					else {
						hasOtherItems = true;
					}											
					for (int i = 0; i < count; ++i) {
						AddCircle(i + 1, radius, hasOtherItems, minOtherItemSize,ratio);
					}
				
				}
				else if (shapeType==4) {
					
					for (int i = 0; i < count; ++i) {
						AddCustomShapeWithHolesByInput(i + 1);
					}
					std::cout << "Added Shape count = " << count << std::endl;
					
				}
				else {
					std::cout << "Invalid shape type." << std::endl;
				}
			}
			std::cout << "Please enter need save file name, for example testoutput.nest: ";
			std::cin >> saveFile;

			bool saveResult = SaveToFile(saveFile);
			std::cout << "SaveToFile result = " << saveResult << std::endl;

			inputFile = saveFile;
		}
		else if (mode == 2) {
			std::cout << "please enter need read nest file name ,for example case5.nest:";
			std::cin >> inputFile;
		}
		else {
			std::cout << "Invalid mode." << std::endl;
			return -1;
		}
		int loadCode = LoadFile(inputFile, Options, Items, &ErrorMessage);
		std::cout << "LoadFile result = " << loadCode << std::endl;
		if (loadCode != 0) {
			std::cout << "LoadFile failed: " << ErrorMessage << std::endl;
			return loadCode;
		}
		std::cout << "Read Items number: " << Items.size() << std::endl;

		char exportSvg = 'y';
		std::cout << "Export SVG? y/n: ";
		std::cin >> exportSvg;
		if (exportSvg == 'y' || exportSvg == 'Y') {
			Options.ExportSvg = true;
			std::cout << "Please enter SVG out path，for example Result.svg: ";
			std::cin >> svgPath;
			Options.SvgPath = svgPath;
		}
		else {
			Options.ExportSvg = false;
		}

		int nestCode = PerformNest(Items, Options, &result);
		std::cout << "PerformNest result = " << nestCode << std::endl;
		std::cout << "UsedBins = " << result.UsedBins << std::endl;
		std::cout << "Message = " << result.Message << std::endl;

		char exportFile = 'y';
		std::cout << "Export File? y/n: ";
		std::cin >> exportFile;
		if (exportFile == 'y' || exportFile == 'Y') {

			std::cout << "Please enter File out path，for example Result.txt: ";
			std::cin >> FilePath;
			int saveCoordCode = SaveCoordinatesFile(FilePath, Options, Items, result.UsedBins);

			std::cout << "SaveCoordinatesFile result = " << saveCoordCode << std::endl;
		}
		//return nestCode;
	}
	return 0;
	//================
}

