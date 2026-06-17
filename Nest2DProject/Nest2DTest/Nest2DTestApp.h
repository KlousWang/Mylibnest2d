#pragma once
#include<iostream>
#include"string.h"
namespace ET {
	namespace NEST2DTESTAPP {	
		int ReadMainMode();
		void InputBoardIfNeeded();
		void InputShapes();
		bool GenerateNestFile(std::string& AInputFile);
		bool ReadNestFileName(std::string& AInputFile);
		int RunNestProcess(const std::string& AInputFile);
	}
}