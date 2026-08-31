#pragma once
#include"Nest2D_DataType.h"
#include"Nest2D_PrivateDataType.h"
#include "EtTechCore_Object.h"

using namespace ClipperLib;
using namespace libnest2d;
namespace ET {
	namespace NEST2DMANAGERLIB {
		class CetNest2DBoardUtils :public ET::CORE::CetCoreObject
		{
			Inherit_Invoke_Hook(CetNest2DBoardUtils)
		protected:
			int _Init() override {
				CetCoreObject::_Init();
				return 0;
			}
			void _WrapFuncs() override {
				CetCoreObject::_WrapFuncs();
                _WrapFunc("FitsBin", Type_Class_Func(FitsBin));
				_WrapFunc("CalcBoardBoundsLocal", Type_Class_Func(CalcBoardBoundsLocal));
				_WrapFunc("BuildPathFromPoints", Type_Class_Func(BuildPathFromPoints));
				_WrapFunc("BuildBinPolygonFromOptions", Type_Class_Func(BuildBinPolygonFromOptions));
                _WrapFunc("BuildRectangleBinPolygon", Type_Class_Func(BuildRectangleBinPolygon));
                _WrapFunc("BuildBoardPath", Type_Class_Func(BuildBoardPath));
                _WrapFunc("BuildBoardSubjectContours", Type_Class_Func(BuildBoardSubjectContours));
                _WrapFunc("BuildPlacedReservedContours", Type_Class_Func(BuildPlacedReservedContours));
			}

		public:
			CetNest2DBoardUtils();
			~CetNest2DBoardUtils();

		public:
			// Checks whether a layout fits the configured bin, including an allowed
			// quarter-turn of the completed layout.
		    bool FitsBin(double AWidth, double AHeight, const TetNestOptions& AOptions);
			TetBoardBounds CalcBoardBoundsLocal(const TetNestBoard& ABoard);
			CetPath BuildPathFromPoints(const std::vector<TetNestPoint>& APoints, double AOffsetX, double AOffsetY, bool AWantOuter);
            CetPolygonImpl BuildBinPolygonFromOptions(const TetNestOptions& AOptions, double& AOutBinWidth, double& AOutBinHeight);
            CetPolygonImpl BuildRectangleBinPolygon(double ABinWidth, double ABinHeight);
            bool BuildBoardPath(const std::vector<TetNestPoint>& AVertices, bool AOuter, CetPath& AOutPath);
            bool BuildBoardSubjectContours(const TetNestOptions& AOptions, ClipperLib::Paths& AOutContours);
            bool BuildPlacedReservedContours(const CetTNestItemVector& AItems, int ABinId, double ASpacing, ClipperLib::Paths& AOutContours, libnest2d::Coord AExtraInflation = 0);
        };
    }
}
