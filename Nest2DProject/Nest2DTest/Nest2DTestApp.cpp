#include "Nest2DTestApp.h"
#include "ShapeMenuRunner.h"
#include "BoardMenuRunner.h"
#include"NestExportManager.h"
#include "Nest2D_DataType.h"
#include "Nest2DTest_SelfFunction.h"
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <fstream>

namespace ET {
	namespace NEST2DTESTAPP {
    
        void CetTestApp::InputBoardIfNeeded()
        {
            CetBoardMenuRunner BoardRunner(*this);
            BoardRunner.Run();
        }
        void CetTestApp::InputShapes()
        {
            m_MinOtherItemSize = 0.0;
            m_HasOtherItems = false;
            m_RandomPosition = true;

            CetShapeMenuRunner ShapeRunner(*this);
            ShapeRunner.Run();
        }
        bool CetTestApp::GenerateNestFile(std::string& AInputFile)
        {
            TetNestInitInput InitInput;

            if (!_InputNestInitOptions(InitInput)) {
                return false;
            }
            if (!_InitNestSystem(InitInput)) {
                return false;
            }
            Nest2DUtils->ClearData();
            InputBoardIfNeeded();
            InputShapes();
            std::string SaveFile;
            if (!_InputSaveFileName(SaveFile)) {
                return false;
            }
            return _SaveNestFile(SaveFile, AInputFile);
        }
        bool CetTestApp::ReadNestFileName(std::string& AInputFile)
        {
            std::cout << "please enter need read nest file name ,for example case5.nest:";
            std::cin >> AInputFile;

            return !AInputFile.empty();
        }
        int CetTestApp::RunNestProcess(const std::string& AInputFile)
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

            CetNestExportManager ExportManager;

            if (!ExportManager.PrepareAll(Options)) {
                return -1;
            }
            int nestCode = Nest2DUtils->PerformNest(Items, Options, &result);
            std::cout << "PerformNest result = " << nestCode << std::endl;
            std::cout << "UsedBins = " << result.UsedBins << std::endl;
            std::cout << "Message = " << result.Message << std::endl;
            ExportManager.ExportAll(Options, Items, result);

            return nestCode;
        }
        int CetTestApp::_UseNormalBoard()
        {
            std::cout << "Use normal rectangle board." << std::endl;
            return 0;
        }
        int CetTestApp::_InputLShapeBoard()
        {
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

            return 0;
        }
        int CetTestApp::_InputCustomPolygonBoard()
        {
            int pointCount = 0;

            std::cout << "Please enter board point count: ";
            std::cin >> pointCount;

            if (pointCount < 3) {
                std::cout << "Board point count must be >= 3." << std::endl;
                return -1;
            }

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

            return 0;
        }
        bool CetTestApp::_InputNestInitOptions(TetNestInitInput& AInput) const
        {
            std::cout << "Please enter bin Width£º";
            std::cin >> AInput.BinWidth;

            std::cout << "Please enter bin height£º";
            std::cin >> AInput.BinHeight;

            std::cout << "Please enter spacing£º";
            std::cin >> AInput.Spacing;

            std::cout << "Please enter rotation number,for example 4£º";
            std::cin >> AInput.Rotations;

            if (AInput.BinWidth <= 0.0 || AInput.BinHeight <= 0.0) {
                std::cout << "Invalid bin size." << std::endl;
                return false;
            }

            if (AInput.Spacing < 0.0) {
                std::cout << "Invalid spacing." << std::endl;
                return false;
            }

            if (AInput.Rotations <= 0) {
                std::cout << "Invalid rotations." << std::endl;
                return false;
            }

            return true;
        }
        bool CetTestApp::_InitNestSystem(const TetNestInitInput& AInput) const
        {
            int initCode = Nest2DUtils->Init(AInput.BinWidth,AInput.BinHeight,AInput.Spacing,AInput.Rotations);
            std::cout << "init result = "<< initCode<< std::endl;

            return initCode == 0;
        }
        bool CetTestApp::_InputSaveFileName(std::string& AFileName) const
        {
            std::cout << "Please enter need save file name, for example testoutput.nest: ";
            std::cin >> AFileName;

            if (AFileName.empty()) {
                std::cout << "Save file name is empty." << std::endl;
                return false;
            }

            return true;
        }
        bool CetTestApp::_SaveNestFile(const std::string& ASaveFile, std::string& AInputFile) const
        {
            bool saveResult = Nest2DUtils->SaveToFile(ASaveFile);

            std::cout << "SaveToFile result = "<< saveResult<< std::endl;

            if (!saveResult) {
                return false;
            }

            AInputFile = ASaveFile;
            return true;
        }
        int CetTestApp::_InputTriangle()
        {
            int count = 0;
            std::cout << "Please enter shape count: ";
            std::cin >> count;
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
                if (curMin > m_MinOtherItemSize) {
                    m_MinOtherItemSize = curMin;
                }
                for (int i = 0; i < count; ++i) {
                    Nest2DUtils->AddTrigle(
                        i + 1,
                        width,
                        height,
                        m_RandomPosition
                    );
                }
                std::cout << "Added triangle count = "
                    << count
                    << std::endl;
                return 0;
            }
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
            if (curMin > m_MinOtherItemSize) {
                m_MinOtherItemSize = curMin;
            }

            for (int i = 0; i < count; ++i) {
                Nest2DUtils->AddCustomTrigle(i + 1, line1, line2, line3);
            }
            std::cout << "Added triangle count = " << count << std::endl;

            return 0;
        }
        int CetTestApp::_InputRectangle()
        {
            int count = 0;
            std::cout << "Please enter shape count: ";
            std::cin >> count;

            double width = 0.0;
            double height = 0.0;

            std::cout << "Please enter shape width: ";
            std::cin >> width;

            std::cout << "Please enter shape height: ";
            std::cin >> height;

            double curMin = std::min(width, height);

            if (curMin > m_MinOtherItemSize) {
                m_MinOtherItemSize = curMin;
            }

            for (int i = 0; i < count; ++i) {
                Nest2DUtils->AddRectangle( i + 1,width,height );
            }

            std::cout << "Added rectangle count = "
                << count
                << std::endl;

            return 0;
        }
        int CetTestApp::_InputCircle()
        {
            int count = 0;
            std::cout << "Please enter shape count: ";
            std::cin >> count;

            double radius = 0.0;
            double ratio = 0.0;

            std::cout << "Please enter shape radius: ";
            std::cin >> radius;

            std::cout << "Please enter shape ratio: ";
            std::cin >> ratio;

            m_HasOtherItems = m_MinOtherItemSize != 0.0;

            for (int i = 0; i < count; ++i) {
                Nest2DUtils->AddCircle( i + 1, radius,m_HasOtherItems, m_MinOtherItemSize, ratio );
            }

            return 0;
        }
        int CetTestApp::_InputShapeWithHoles()
        {
            int count = 0;
            std::cout << "Please enter shape count: ";
            std::cin >> count;

            for (int i = 0; i < count; ++i) {
                Nest2DUtils->AddCustomShapeWithHolesByInput(i + 1);
            }
            std::cout << "Added Shape count = " << count << std::endl;
            return 0;
        }

}
}