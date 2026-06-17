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
				_WrapFunc("CalcBoardBoundsLocal", Type_Class_Func(CalcBoardBoundsLocal));
				_WrapFunc("BuildPathFromPoints", Type_Class_Func(BuildPathFromPoints));
				_WrapFunc("BuildBinPolygonFromOptions", Type_Class_Func(BuildBinPolygonFromOptions));
			}

		public:
			CetNest2DBoardUtils();
			~CetNest2DBoardUtils();

		public:
			TetBoardBounds CalcBoardBoundsLocal(const TetNestBoard& ABoard);
			Path BuildPathFromPoints(const std::vector<TetNestPoint>& APoints, double AOffsetX, double AOffsetY, bool AWantOuter);
            PolygonImpl BuildBinPolygonFromOptions(const TetNestOptions& AOptions, double& AOutBinWidth, double& AOutBinHeight);
        };
    }
}
