#pragma once
#include"EtTechCore_Object.h"
#include"Nest2D_PrivateDataType.h"
#include<vector>

namespace ET {
	namespace NEST2DMANAGERLIB {
		class CetShapeAnalyzer:public ET::CORE::CetCoreObject
		{
			Inherit_Invoke_Hook(CetShapeAnalyzer)
		protected:
			int _Init() override {
				CetCoreObject::_Init();
				return 0;
			}
			void _WrapFuncs() override {
				CetCoreObject::_WrapFuncs();
				_WrapFunc("AnalyzeALL", Type_Class_Func(AnalyzeALL));
			/*	_WrapFunc("ExportSvg", Type_Class_Func(ExportSvgItems));
				_WrapFunc("ExportSvgbd", Type_Class_Func(ExportSvg));
				_WrapFunc("ExportSvgPackGroup", Type_Class_Func(ExportSvgPackGroup));*/
			}
		public:
			CetShapeAnalyzer();
			~CetShapeAnalyzer();

		public:
			std::vector<TetShapeFeature> AnalyzeALL(const CetTNestItemVector& AItems);

		protected:
			TetShapeFeature _AnalyzeOne(const CetNestItem& AItem, int AOriginalIndex);
			double _CalculatePerimeter(const CetPath& ACounter);
			bool _IsConvex(const CetPath& ACounter);
		
			//analyzeone¸¨Öúº¯Êý
			void _NormalizePath(CetPath& APath);
			MetShapeType _ClassifyShape(const TetShapeFeature& AFeature,bool AHasHoles);
		};
	}
}


