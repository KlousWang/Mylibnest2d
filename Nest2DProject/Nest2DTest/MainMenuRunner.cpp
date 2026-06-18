#include "MainMenuRunner.h"

CetMainMenuRunner::CetMainMenuRunner()
{
    
    _AddMenuItem(
        MENU_EXIT,
        "Exit App",
        std::bind(&CetMainMenuRunner::_ExitAction, this)
    );

    _AddMenuItem(
        MENU_GENERATE_AND_RUN,
        "Generate and format test graphics",
        std::bind(&CetMainMenuRunner::_GenerateAndRunAction, this)
    );

    _AddMenuItem(
        MENU_READ_AND_RUN,
        "Read and format an existing file",
        std::bind(&CetMainMenuRunner::_ReadAndRunAction, this)
    );
}

CetMainMenuRunner::~CetMainMenuRunner()
{
}

std::string CetMainMenuRunner::_GetMenuTitle() const
{
    return "Please select mode:";
}

int CetMainMenuRunner::_ExitAction()
{
    std::cout << "Exiting program." << std::endl;

    _Stop();

    return 0;
}

int CetMainMenuRunner::_GenerateAndRunAction()
{
    std::string InputFile;

    if (!m_TestApp.GenerateNestFile(InputFile)) {
        std::cout << "Generate nest file failed." << std::endl;
        return -1;
    }

    int Result = m_TestApp.RunNestProcess(InputFile);

    std::cout << "RunNestProcess result = "
        << Result
        << std::endl;

    return Result;
}

int CetMainMenuRunner::_ReadAndRunAction()
{
    std::string InputFile;

    if (!m_TestApp.ReadNestFileName(InputFile)) {
        std::cout << "Input file is empty." << std::endl;
        return -1;
    }

    int Result = m_TestApp.RunNestProcess(InputFile);

    std::cout << "RunNestProcess result = "
        << Result
        << std::endl;

    return Result;
}