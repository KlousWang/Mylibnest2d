#pragma once
#include"EtTechCore_Object.h"
#include<iostream>
#include<map>
#include<string>
#include<functional>
#include<limits>

class CetMenuRunnerBase : public ET::CORE::CetCoreObject
{
public:
	using CetMenuFunc = std::function<int()>;
	struct TetMenuItem
	{
		std::string Description;
		CetMenuFunc Func = nullptr;
	};
public:
	CetMenuRunnerBase();
	virtual ~CetMenuRunnerBase();

	virtual int Run();
protected:
	virtual std::string _GetMenuTitle()const;
	virtual void _AddMenuItem(int AChoice, const std::string& ADescription, CetMenuFunc AFunc);
	virtual void _Stop();
	virtual void _PrintMenu() const;
	virtual int _ReadChoice() const;
	virtual int _ExecuteMenuItem(int AChoice);
protected:
	bool m_Running = true;
	std::map<int, TetMenuItem> m_MenuItems;
};

