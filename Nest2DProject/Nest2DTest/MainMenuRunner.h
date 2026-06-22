#pragma once
#include"Nest2DTestApp.h"
#include"MenuRunnerBase.h"
#include<iostream>
#include<map>

class CetMainMenuRunner :public CetMenuRunnerBase
{
public:
	CetMainMenuRunner();
	virtual ~CetMainMenuRunner();

	int SetTestApp(ET::NEST2DTESTAPP::CetTestApp* ATestApp);
protected:
	std::string _GetMenuTitle() const override;

protected:
	int _ExitAction();
	int _GenerateAndRunAction();
	int _ReadAndRunAction();
protected:
	enum MetMainMenuChoice
	{
		MENU_EXIT = 0,
		MENU_GENERATE_AND_RUN = 1,
		MENU_READ_AND_RUN = 2
	};

protected:
    ET::NEST2DTESTAPP::CetTestApp* m_pTestApp = nullptr;
};

