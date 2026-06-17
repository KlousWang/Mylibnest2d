#include "Nest2DTestApp.h"
#include"Nest2D_DataType.h"

#include"Nest2DTest_SelfFunction.h"
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <fstream>

namespace ET {
	namespace NEST2DTESTAPP {
		void ShowTitle() {
			std::cout << "Please select model" << std::endl;
			std::cout << "0¡¢Exit App" << std::endl;
			std::cout << "1¡¢Generate and format test graphics" << std::endl;
			std::cout << "2¡¢Read and format an existing file" << std::endl;
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
		int ReadMainMode()
		{
			ShowTitle();

			int mode = 0;
			std::cin >> mode;

			return mode;
		}
        void InputBoardIfNeeded()
        {
            char useIrregularBoard = 'n';
            std::cout << "Use irregular board? y/n: ";
            std::cin >> useIrregularBoard;

            if (useIrregularBoard != 'y' && useIrregularBoard != 'Y') {
                return;
            }

            int boardType = 0;
            std::cout << "Please select board type:" << std::endl;
            std::cout << "1. L shape board" << std::endl;
            std::cout << "2. Custom polygon board" << std::endl;
            std::cout << "Please enter: ";
            std::cin >> boardType;

            if (boardType == 1) {
                double W = 0.0;
                double H = 0.0;
                double CutW = 0.0;
                double CutH = 0.0;

                std::cout << "Please enter board width: ";
                std::cin >> W;

                std::cout << "Please enter board height: ";
                std::cin >> H;

                std::cout << "Please enter cut width: ";
                std::cin >> CutW;

                std::cout << "Please enter cut height: ";
                std::cin >> CutH;

                CetVertices Board = {
                    {0.0, 0.0},
                    {W, 0.0},
                    {W, H - CutH},
                    {W - CutW, H - CutH},
                    {W - CutW, H},
                    {0.0, H}
                };

                Nest2DUtils->AddBoard(std::move(Board));
            }
            else if (boardType == 2) {
                int pointCount = 0;
                std::cout << "Please enter board point count: ";
                std::cin >> pointCount;

                CetVertices Board;
                Board.reserve(pointCount);

                for (int i = 0; i < pointCount; ++i) {
                    double X = 0.0;
                    double Y = 0.0;

                    std::cout << "Board point " << i + 1 << " X: ";
                    std::cin >> X;

                    std::cout << "Board point " << i + 1 << " Y: ";
                    std::cin >> Y;

                    Board.emplace_back(X, Y);
                }

                Nest2DUtils->AddBoard(std::move(Board));
            }
        }
        void InputShapes()
        {
            double minOtherItemSize = 0.0;
            bool hasOtherItems = false;
            bool randomPosition = true;

            while (true) {
                int shapeType = 0;
                ShowAddShapeTitle();
                std::cin >> shapeType;

                if (shapeType == 0) {
                    break;
                }

                int count = 0;
                std::cout << "Please enter shape count: ";
                std::cin >> count;

                if (shapeType == 1) {
                    int typeTri = 0;
                    std::cout << "If you want to generate a custom triangle, enter 1; for anything else, enter 2:" << std::endl;
                    std::cin >> typeTri;

                    if (typeTri == 1) {
                        double width = 0.0;
                        double height = 0.0;

                        std::cout << "Please enter shape width: ";
                        std::cin >> width;

                        std::cout << "Please enter shape height: ";
                        std::cin >> height;

                        double curMin = std::min(width, height);
                        if (curMin > minOtherItemSize) {
                            minOtherItemSize = curMin;
                        }

                        for (int i = 0; i < count; ++i) {
                            Nest2DUtils->AddTrigle(i + 1, width, height, randomPosition);
                        }

                        std::cout << "Added triangle count = " << count << std::endl;
                    }
                    else {
                        double line1 = 0.0;
                        double line2 = 0.0;
                        double line3 = 0.0;

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
                            Nest2DUtils->AddCustomTrigle(i + 1, line1, line2, line3);
                        }

                        std::cout << "Added triangle count = " << count << std::endl;
                    }
                }
                else if (shapeType == 2) {
                    double width = 0.0;
                    double height = 0.0;

                    std::cout << "Please enter shape width: ";
                    std::cin >> width;

                    std::cout << "Please enter shape height: ";
                    std::cin >> height;

                    double curMin = std::min(width, height);
                    if (curMin > minOtherItemSize) {
                        minOtherItemSize = curMin;
                    }

                    for (int i = 0; i < count; ++i) {
                        Nest2DUtils->AddRectangle(i + 1, width, height);
                    }

                    std::cout << "Added rectangle count = " << count << std::endl;
                }
                else if (shapeType == 3) {
                    double radius = 0.0;
                    double ratio = 0.0;

                    std::cout << "Please enter shape radius: ";
                    std::cin >> radius;

                    std::cout << "Please enter shape ratio: ";
                    std::cin >> ratio;

                    hasOtherItems = minOtherItemSize != 0.0;

                    for (int i = 0; i < count; ++i) {
                        Nest2DUtils->AddCircle(i + 1, radius, hasOtherItems, minOtherItemSize, ratio);
                    }
                }
                else if (shapeType == 4) {
                    for (int i = 0; i < count; ++i) {
                        Nest2DUtils->AddCustomShapeWithHolesByInput(i + 1);
                    }

                    std::cout << "Added Shape count = " << count << std::endl;
                }
                else {
                    std::cout << "Invalid shape type." << std::endl;
                }
            }
        }
        bool GenerateNestFile(std::string& AInputFile)
        {
            double binWidth = 0.0;
            double binHeight = 0.0;
            double spacing = 0.0;
            int rotations = 0;

            std::cout << "Please enter bin Width£º";
            std::cin >> binWidth;

            std::cout << "Please enter bin height£º";
            std::cin >> binHeight;

            std::cout << "Please enter spacing£º";
            std::cin >> spacing;

            std::cout << "Please enter rotation number,for example 4£º";
            std::cin >> rotations;

            int initCode = Nest2DUtils->Init(binWidth, binHeight, spacing, rotations);
            std::cout << "init result = " << initCode << std::endl;

            if (initCode != 0) {
                return false;
            }

            Nest2DUtils->ClearData();

            InputBoardIfNeeded();
            InputShapes();

            std::string saveFile;
            std::cout << "Please enter need save file name, for example testoutput.nest: ";
            std::cin >> saveFile;

            bool saveResult = Nest2DUtils->SaveToFile(saveFile);
            std::cout << "SaveToFile result = " << saveResult << std::endl;

            if (!saveResult) {
                return false;
            }

            AInputFile = saveFile;
            return true;
        }
        bool ReadNestFileName(std::string& AInputFile)
        {
            std::cout << "please enter need read nest file name ,for example case5.nest:";
            std::cin >> AInputFile;

            return !AInputFile.empty();
        }
        int RunNestProcess(const std::string& AInputFile)
        {
            TetNestOptions Options;
            TetNestResult result;
            std::vector<TetNestPolygon> Items;
            std::string ErrorMessage;

            int loadCode = Nest2DUtils->LoadFile(AInputFile, Options, Items, &ErrorMessage);
            std::cout << "LoadFile result = " << loadCode << std::endl;

            if (loadCode != 0) {
                std::cout << "LoadFile failed: " << ErrorMessage << std::endl;
                return loadCode;
            }

            std::cout << "Read Items number: " << Items.size() << std::endl;

            char exportSvg = 'y';
            std::string svgPath;

            std::cout << "Export SVG? y/n: ";
            std::cin >> exportSvg;

            if (exportSvg == 'y' || exportSvg == 'Y') {
                Options.ExportSvg = true;

                std::cout << "Please enter SVG out path£¬for example Result.svg: ";
                std::cin >> svgPath;

                Options.SvgPath = svgPath;
            }
            else {
                Options.ExportSvg = false;
            }

            int nestCode = Nest2DUtils->PerformNest(Items, Options, &result);
            std::cout << "PerformNest result = " << nestCode << std::endl;
            std::cout << "UsedBins = " << result.UsedBins << std::endl;
            std::cout << "Message = " << result.Message << std::endl;

            char exportFile = 'y';
            std::cout << "Export File? y/n: ";
            std::cin >> exportFile;

            if (exportFile == 'y' || exportFile == 'Y') {
                std::string FilePath;

                std::cout << "Please enter File out path£¬for example Result.txt: ";
                std::cin >> FilePath;

                int saveCoordCode =
                    Nest2DUtils->SaveCoordinatesFile(FilePath, Options, Items, result.UsedBins);

                std::cout << "SaveCoordinatesFile result = " << saveCoordCode << std::endl;
            }

            return nestCode;
        }
	}
}