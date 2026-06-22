#pragma once
#include "EtTechCore_Object.h"
#include <iostream>
#include <string>

namespace ET {
    namespace NEST2DTESTAPP {

        class CetShapeMenuRunner;
        class CetBoardMenuRunner;        
        class CetTestApp : public ET::CORE::CetCoreObject {
        public :
            CetTestApp();
            virtual ~CetTestApp();
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
            friend class CetShapeMenuRunner;
            friend class CetBoardMenuRunner;

            int _InputTriangle();
            int _InputRectangle();
            int _InputCircle();
            int _InputShapeWithHoles();

            int _UseNormalBoard();
            int _InputLShapeBoard();
            int _InputCustomPolygonBoard();

            bool _InputNestInitOptions(TetNestInitInput& AInput) const;
            bool _InitNestSystem(const TetNestInitInput& AInput) const;
            bool _InputSaveFileName(std::string& AFileName) const;
            bool _SaveNestFile(const std::string& ASaveFile, std::string& AInputFile) const;

        protected:
            double m_MinOtherItemSize = 0.0;
            bool m_HasOtherItems = false;
            bool m_RandomPosition = true;
        };
    }
}