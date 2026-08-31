#pragma once
#include "EtTechCore_Object.h"
#include "Nest2D_DataType.h"
#include "Nest2D_PrivateDataType.h"
#include <iostream>

namespace ET {
	namespace NEST2DMANAGERLIB {
		class CetGeometryUtils :public ET::CORE::CetCoreObject
		{
			Inherit_Invoke_Hook(CetGeometryUtils)
		protected:
			int _Init() override {
				CetCoreObject::_Init();
				return 0;
			}
			void _WrapFuncs() override {
				CetCoreObject::_WrapFuncs();
				_WrapFunc("TransformPoint", Type_Class_Func(TransformPoint));
				_WrapFunc("RadToDeg", Type_Class_Func(RadToDeg));
				_WrapFunc("PointInPolygon", Type_Class_Func(PointInPolygon));
				_WrapFunc("IsPointInsideBoard", Type_Class_Func(IsPointInsideBoard));
				_WrapFunc("ValidateItemsInsideBoard", Type_Class_Func(ValidateItemsInsideBoard));
				_WrapFunc("CalcPolygonBoundingBoxArea", Type_Class_Func(CalcPolygonBoundingBoxArea));
			_WrapFunc("ComparePolygonAreaDesc", Type_Class_Func(ComparePolygonAreaDesc));
				_WrapFunc("AccumulateFreeRegions", Type_Class_Func(AccumulateFreeRegions));
				_WrapFunc("GetMinimumPassableWidth", Type_Class_Func(GetMinimumPassableWidth));
				_WrapFunc("BuildInsetBoardContours", Type_Class_Func(BuildInsetBoardContours));
			}

		public:
			CetGeometryUtils();
			~CetGeometryUtils();
		public:
			TetNestPoint TransformPoint(const TetNestPoint& AP, double AX, double AY, double AAngle);
			double RadToDeg(double ARad);
			bool PointInPolygon(const TetNestPoint& AP, const std::vector<TetNestPoint>& APolygon);
			bool IsPointInsideBoard(const TetNestPoint& AP, const TetNestBoard& ABoard);
			void ValidateItemsInsideBoard(std::vector<TetNestPolygon>& AItems,const TetNestBoard& ABoard);

			double CalcPolygonBoundingBoxArea(const TetNestPolygon& APoly);
			bool ComparePolygonAreaDesc(const TetNestPolygon&ADataa, const TetNestPolygon& ADAtab);
			void AccumulateFreeRegions(const ClipperLib::PolyNode&, std::size_t&, double&, double&);
			double GetMinimumPassableWidth(const CetTNestItemVector&, const TetNestOptions&);
			bool BuildInsetBoardContours(const ClipperLib::Paths&, libnest2d::Coord, ClipperLib::Paths&);
		};
	}

}
