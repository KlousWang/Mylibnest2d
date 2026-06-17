#pragma once
#include"string.h"
#include "EtTechCore_Object.h"
#include<iostream>
#include <string>
#include <map>

namespace ET {
	namespace NEST2DTESTAPP {	
		class CetTestApp {
		public:
			void InputBoardIfNeeded();
			void InputShapes();
			bool GenerateNestFile(std::string& AInputFile);
			bool ReadNestFileName(std::string& AInputFile);
			int RunNestProcess(const std::string& AInputFile);

        protected:
            struct TetNestInitInput
            {
                double BinWidth = 0.0;
                double BinHeight = 0.0;
                double Spacing = 0.0;
                int Rotations = 0;
            };
        protected:
            using TetShapeFunc = int (CetTestApp::*)();
            struct TetShapeMenuItem
            {
                std::string Description;
                TetShapeFunc Func = nullptr;
            };
            using CetShapeMenuMap = std::map<int, TetShapeMenuItem>;

            using CetBoardFunc = int (CetTestApp::*)();

            struct TetBoardMenuItem
            {
                std::string Description;
                CetBoardFunc Func = nullptr;
            };

            using TetBoardMenuMap = std::map<int, TetBoardMenuItem>;

        protected:
            void _PrintShapeMenu(const CetShapeMenuMap& AMenuItems) const;
            int _ReadChoice() const;
            int _ExecuteShapeMenuItem(int AChoice, const CetShapeMenuMap& AMenuItems);

            int _FinishInputShapes();
            int _InputTriangle();
            int _InputRectangle();
            int _InputCircle();
            int _InputShapeWithHoles();

            void _PrintBoardMenu(const TetBoardMenuMap& AMenuItems) const;

            int _ExecuteBoardMenuItem(int AChoice, const TetBoardMenuMap& AMenuItems);

            int _UseNormalBoard();
            int _InputLShapeBoard();
            int _InputCustomPolygonBoard();

            bool _InputNestInitOptions(TetNestInitInput& AInput) const;
            bool _InitNestSystem(const TetNestInitInput& AInput) const;
            bool _InputSaveFileName(std::string& AFileName) const;
            bool _SaveNestFile(const std::string& ASaveFile, std::string& AInputFile) const;
        protected:
            bool m_StopInputShapes = false;
            double m_MinOtherItemSize = 0.0;
            bool m_HasOtherItems = false;
            bool m_RandomPosition = true;
        
		};

		
	}
}