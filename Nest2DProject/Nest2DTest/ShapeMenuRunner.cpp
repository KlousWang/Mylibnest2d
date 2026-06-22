#include "ShapeMenuRunner.h"
#include <functional>

namespace ET {
    namespace NEST2DTESTAPP {

        CetShapeMenuRunner::CetShapeMenuRunner()
        {
        }

        CetShapeMenuRunner::~CetShapeMenuRunner()
        {
        }

        int CetShapeMenuRunner::SetTestApp(CetTestApp* ATestApp)
        {
            if (!ATestApp) {
                std::cout << "TestApp is null." << std::endl;
                return -1;
            }
            m_pTestApp = ATestApp;
            _BindMenuItems();
            return 0;
        }

        void CetShapeMenuRunner::_BindMenuItems()
        {
			m_MenuItems.clear();
			_AddMenuItem(SHAPE_FINISH, "Finish adding shapes", std::bind(&CetShapeMenuRunner::_FinishAction, this));
			_AddMenuItem(SHAPE_TRIANGLE, "Triangle", std::bind(&CetTestApp::_InputTriangle, m_pTestApp));
			_AddMenuItem(SHAPE_RECTANGLE, "Rectangle", std::bind(&CetTestApp::_InputRectangle, m_pTestApp));
			_AddMenuItem(SHAPE_CIRCLE, "Circle", std::bind(&CetTestApp::_InputCircle, m_pTestApp));
			_AddMenuItem(SHAPE_WITH_HOLES,"With Holes",std::bind(&CetTestApp::_InputShapeWithHoles, m_pTestApp));
        }

        std::string CetShapeMenuRunner::_GetMenuTitle() const
        {
            return "Please select shape type:";
        }

        int CetShapeMenuRunner::_FinishAction()
        {
            _Stop();
            return 0;
        }
    }
}