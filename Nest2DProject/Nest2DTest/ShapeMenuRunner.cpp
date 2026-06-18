#include "ShapeMenuRunner.h"
namespace ET {
	namespace NEST2DTESTAPP {
		CetShapeMenuRunner::CetShapeMenuRunner(CetTestApp& AtestApp):m_TestApp(AtestApp)
		{
            _AddMenuItem(SHAPE_FINISH,"Finish adding shapes",std::bind(&CetShapeMenuRunner::_FinishAction, this));
            _AddMenuItem(SHAPE_TRIANGLE,"Triangle",std::bind(&CetTestApp::_InputTriangle, &m_TestApp));
            _AddMenuItem(SHAPE_RECTANGLE,"Rectangle",std::bind(&CetTestApp::_InputRectangle, &m_TestApp));
            _AddMenuItem(SHAPE_CIRCLE,"Circle",std::bind(&CetTestApp::_InputCircle, &m_TestApp));
            _AddMenuItem(SHAPE_WITH_HOLES,"With Holes",std::bind(&CetTestApp::_InputShapeWithHoles, &m_TestApp));
		}
        CetShapeMenuRunner::~CetShapeMenuRunner()
        {
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

