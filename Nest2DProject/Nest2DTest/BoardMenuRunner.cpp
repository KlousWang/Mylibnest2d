#include "BoardMenuRunner.h"
#include <functional>

namespace ET {
    namespace NEST2DTESTAPP {

        CetBoardMenuRunner::CetBoardMenuRunner()
        {
        }
        CetBoardMenuRunner::~CetBoardMenuRunner()
        {
        }

        int CetBoardMenuRunner::SetTestApp(CetTestApp* ATestApp)
        {
            if (!ATestApp) {
                std::cout << "TestApp is null." << std::endl;
                return -1;
            }

            m_pTestApp = ATestApp;
            _BindMenuItems();

            return 0;
        }

        void CetBoardMenuRunner::_BindMenuItems()
        {
            m_MenuItems.clear();

            _AddMenuItem(
                BOARD_NORMAL,
                "Normal rectangle board",
                std::bind(&CetTestApp::_UseNormalBoard, m_pTestApp)
            );

            _AddMenuItem(
                BOARD_L_SHAPE,
                "L shape board",
                std::bind(&CetTestApp::_InputLShapeBoard, m_pTestApp)
            );

            _AddMenuItem(
                BOARD_CUSTOM_POLYGON,
                "Custom polygon board",
                std::bind(&CetTestApp::_InputCustomPolygonBoard, m_pTestApp)
            );
        }

        std::string CetBoardMenuRunner::_GetMenuTitle() const
        {
            return "Please select board type:";
        }

        int CetBoardMenuRunner::Run()
        {
            if (!m_pTestApp) {
                std::cout << "TestApp is null. Please call SetTestApp first." << std::endl;
                return -1;
            }

            _PrintMenu();

            int Choice = _ReadChoice();

            return _ExecuteMenuItem(Choice);
        }

    }
}