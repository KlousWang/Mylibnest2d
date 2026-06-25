#pragma once

#include "NestTestDataAPI_Def.h"
#include "NestTestData_DataType.h"
#include "EtTechCore_Object.h"
//#include "EtTechCore_Component.h"
#include"NestGeometryUtils.h"
#include"NestProfileInputUtils.h"
#include <vector>
#include <string>
#include <utility>
#include <random>

namespace ET {
    namespace NESTTESTDATALIB {

        class NEST_API CetNestTestDataAPI : public ET::CORE::CetCoreObject
        {
            Inherit_Invoke_Hook(CetNestTestDataAPI)

        protected:
            int _Init() override {
               // std::cout << "[DLL] CetNestTestDataAPI::_Init called" << std::endl;

                CetCoreObject::_Init();
                return 0;
            }
            void _WrapFuncs() override {
                CetCoreObject::_WrapFuncs();
                _WrapFunc("ClearPolygons", Type_Class_Func(ClearPolygons));
                _WrapFunc("AddTriangle", Type_Class_Func(AddTriangle));
                _WrapFunc("AddTriangleBySides", Type_Class_Func(AddTriangleBySides));
                _WrapFunc("Init", Type_Class_Func(Init));          
                _WrapFunc("AddRectangle", Type_Class_Func(AddRectangle));
                _WrapFunc("AddCircle", Type_Class_Func(AddCircle));
                _WrapFunc("AddLShape", Type_Class_Func(AddLShape));
                _WrapFunc("GenerateRandomConvexPolygons", Type_Class_Func(GenerateRandomConvexPolygons));
                _WrapFunc("AddCustomShapeWithHolesByInput", Type_Class_Func(AddCustomShapeWithHolesByInput));
                _WrapFunc("AddBoard", Type_Class_Func(AddBoard));
                _WrapFunc("AddBoardWithHoles", Type_Class_Func(AddBoardWithHoles));
                _WrapFunc("GenerateRandomTriangles", Type_Class_Func(GenerateRandomTriangles));
                _WrapFunc("PolygonCount", Type_Class_Func(PolygonCountWrap));
                _WrapFunc("SaveToFile", Type_Class_Func(SaveToFileWrap));
                _WrapFunc("PolygonCountWrap", Type_Class_Func(PolygonCountWrap));
                _WrapFunc("SaveToFileWrap", Type_Class_Func(SaveToFileWrap));
               // _WrapFunc("ReadProfileVertices", Type_Class_Func(ReadProfileVertices));
            }

        public:
            CetNestTestDataAPI();
             ~CetNestTestDataAPI();

        public:
            int Init( const TetNestDataOptions& opt);
            void AddPolygon(const TetPolygonData& APoly );
            void AddPolygon(int AId,const std::string& AName, CetVertices&& AVertices);
            void AddRectangle( int AId, double AW,double AH);
            void AddTriangle(int AId,double ABase,double AHeight, bool ARightAngle = true );
            void AddTriangleBySides(int AId, double ASideA, double ASideB, double ASideC);
			void AddCircle(int AId, double ARadius, bool AHasOtherItems, double AMinOtherItemSize, double AToleranceRatio = 0.1);
            void AddLShape(int AId,double AW,double AH,double ACutW, double ACutH);
            void AddPolygonWithHoles( int AId,const std::string& AName,CetVertices&& AOuter, std::vector<CetVertices>&& AHoles);
            void AddCustomShapeWithHolesByInput(int AId);
            void AddBoard(CetVertices&& AVertices);
            void AddBoardWithHoles(CetVertices&& AOuter, std::vector<CetVertices>&& AHoles);
            void ClearBoard();
            void GenerateRandomConvexPolygons(int ACount, int AMinVertices, int AMaxVertices,double AMaxWidth, double AMaxHeight, unsigned ASeed = std::random_device{}());
            void GenerateRandomTriangles(int ACount, double AMaxWidth,double AMaxHeight, unsigned ASeed = std::random_device{}() );
           // CetVertices ReadProfileVertices(bool AIsHole, std::string* AProfileName);
            std::string ToString() const;
            size_t PolygonCount() const;
            void ClearPolygons();
            bool SaveToFile(
                const std::string& AFilePath
            ) const;

        public:
            // 给 EtTechCore Functor 调用的非 const 包装函数
            size_t PolygonCountWrap() {
                return PolygonCount();
            }

            bool SaveToFileWrap( const std::string& AFilePath ){
                return SaveToFile(AFilePath);
            }

        protected:
			double m_BinWidth = 0;
			double m_BinHeight = 0;
			double m_Bpacing = 0;
			int m_Rotations = 0;

            float m_PlacerAccuracy =0;
            int m_PlacerAlignment =0;
            int m_PlacerStartingPoint =0;
            bool m_PlacerParallel =true;
            bool m_PlacerExploreHoles =false;
         
			std::vector<TetPolygonData> m_Polygons;
            TetBoardData m_Board;
            //std::vector<std::string>
            CetNestGeometryUtils m_GeometryUtils;
            CetNestProfileInputUtils m_ProfileInputUtils;
			/*static std::vector<std::pair<double, double>> _ConvexHull(
				std::vector<std::pair<double, double>> APoints
            );*/
        };

    }
}