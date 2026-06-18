#pragma once
#include"MenuRunnerBase.h"
#include"Nest2DTestApp.h"
namespace ET {
	namespace NEST2DTESTAPP {
		class CetShapeMenuRunner :public CetMenuRunnerBase
		{

		public:
			CetShapeMenuRunner(CetTestApp& AtestApp);
			virtual ~CetShapeMenuRunner();
		protected:
			std::string _GetMenuTitle()const override;

		protected:
			int _FinishAction();
		protected:
			CetTestApp& m_TestApp;
			enum MetShapeMenuChoice {
				SHAPE_FINISH = 0,
				SHAPE_TRIANGLE = 1,
				SHAPE_RECTANGLE = 2,
				SHAPE_CIRCLE = 3,
				SHAPE_WITH_HOLES = 4
			};
 		};
	}
}
