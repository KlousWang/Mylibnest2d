#pragma once
#include"Nest2D_Def.h"
#include"Nest2D_DataType.h"
#include "EtTechCore_Object.h"
#include "EtTechCore_Component.h"
#include<string>
#include<vector>

namespace ET {
	namespace NEST2DMANAGERLIB {
		//class CetNest2DManager;
		class NEST2D_API CetFile :public ET::CORE::CetCoreObject
		{
			Inherit_Invoke_Hook(CetFile)

		protected:
			int _Init()override {
				CetCoreObject::_Init();
				return 0;
			}
			void _WrapFuncs()override {
				CetCoreObject::_WrapFuncs();
				_WrapFunc("LoadNestCaseFromFile", Type_Class_Func(LoadNestCaseFromFile));
				_WrapFunc("SaveNestResultToFile", Type_Class_Func(SaveNestResultToFile));
			}

		public:
			CetFile();
			~CetFile();

		public:
			int LoadNestCaseFromFile(const std::string& AFilePath, TetNestOptions& AOptions, std::vector<TetNestPolygon>& AItems, std::string* AErrorMessage = nullptr);
			int SaveNestResultToFile(const std::string& AFilePath, const TetNestOptions& AOptions, const std::vector<TetNestPolygon>& AItems, int AUsedBins);
	
		};
	}
}
