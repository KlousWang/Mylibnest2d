#pragma once
#include"Nest2D_DataType.h"
#include"Nest2D_PrivateDataType.h"
#include"EtTechCore_Object.h"
#include<vector>
//#include<libnest2d/backends/clipper/geometries.hpp>
//#include<libnest2d/libnest2d.hpp>
namespace ET {
	namespace NEST2DMANAGERLIB {
		class CetExportPhoto :public ET::CORE::CetCoreObject
		{
			Inherit_Invoke_Hook(CetExportPhoto)

		protected:
			int _Init() override {
				CetCoreObject::_Init();
				return 0;
			}
			void _WrapFuncs() override {
				CetCoreObject::_WrapFuncs();
				_WrapFunc("ExportSvg", Type_Class_Func(ExportSvgItems));
			}
		public:
			CetExportPhoto();
			virtual ~CetExportPhoto();
		public:
			static int ExportSvg(const std::vector<TetNestPolygon>& AItems, const TetNestOptions& AOptions, int AUsedBins);

			 int ExportSvgItems(CetTNestItemVector& ANestItems, const TetNestOptions& AOptions, int AUsedBins);
		};

	}
}