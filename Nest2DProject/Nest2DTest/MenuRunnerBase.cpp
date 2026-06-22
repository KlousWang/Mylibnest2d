#include "MenuRunnerBase.h"

CetMenuRunnerBase::CetMenuRunnerBase()
{
}

CetMenuRunnerBase::~CetMenuRunnerBase()
{
}
int CetMenuRunnerBase::Run()
{
	while (m_Running) {
		_PrintMenu();
		int Choice = _ReadChoice();
		_ExecuteMenuItem(Choice);
	}
	return 0;
}
std::string CetMenuRunnerBase::_GetMenuTitle() const
{
	return "Please select Menu";
}

void CetMenuRunnerBase::_AddMenuItem(int AChoice, const std::string& ADescription, CetMenuFunc AFunc)
{
	m_MenuItems[AChoice] = { ADescription, AFunc };
}

void CetMenuRunnerBase::_Stop()
{
	m_Running = false;
}

void CetMenuRunnerBase::_PrintMenu() const
{
	std::cout << _GetMenuTitle() << std::endl;
	for (const auto& Item : m_MenuItems) {
		std::cout << Item.first<< ". "<< Item.second.Description<< std::endl;
	}
	std::cout << "Please enter: ";
}

int CetMenuRunnerBase::_ReadChoice() const
{
	int Choice = 0;
	std::cin >> Choice;
	return Choice;
}

int CetMenuRunnerBase::_ExecuteMenuItem(int AChoice)
{
	auto It = m_MenuItems.find(AChoice);
	if (It == m_MenuItems.end()) {
		std::cout << "Invalid choice." << std::endl;
		return -1;
	}
	if (It->second.Func == nullptr) {
		std::cout << "Function is empty." << std::endl;
		return -1;
	}
	return It->second.Func();
}