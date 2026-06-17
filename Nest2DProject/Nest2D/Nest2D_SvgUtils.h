#pragma once
#include "EtTechCore_Object.h"
#include"Nest2D_DataType.h"
#include<iostream>
#include<vector>
namespace ET {
	namespace NEST2DMANAGERLIB {
		class CetSvgUtils :public ET::CORE::CetCoreObject
		{
			Inherit_Invoke_Hook(CetSvgUtils)
		protected:
			int _Init() override {
				CetCoreObject::_Init();
				return 0;
			}
			void _WrapFuncs() override {
				CetCoreObject::_WrapFuncs();
				_WrapFunc("MakeBoardSvgPath", Type_Class_Func(MakeBoardSvgPath));
				_WrapFunc("InsertTextBeforeSvgEnd", Type_Class_Func(InsertTextBeforeSvgEnd));
			}
		public:
			CetSvgUtils();
			~CetSvgUtils();

		public:
			std::string MakeBoardSvgPath(const TetNestBoard& ABoard, double ASvgHeight);
			void InsertTextBeforeSvgEnd(const std::string& AFilePath, const std::string& AText);
		};

	}
}