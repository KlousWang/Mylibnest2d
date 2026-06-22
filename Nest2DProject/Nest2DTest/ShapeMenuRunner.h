#pragma once
#include "MenuRunnerBase.h"
#include "Nest2DTestApp.h"

namespace ET {
    namespace NEST2DTESTAPP {

        class CetShapeMenuRunner : public CetMenuRunnerBase
        {
        public:
            CetShapeMenuRunner();
            virtual ~CetShapeMenuRunner();
            int SetTestApp(CetTestApp* ATestApp);
        protected:
            std::string _GetMenuTitle() const override;
        protected:
            int _FinishAction();
        private:
            void _BindMenuItems();
        protected:
            CetTestApp* m_pTestApp = nullptr;

            enum MetShapeMenuChoice
            {
                SHAPE_FINISH = 0,
                SHAPE_TRIANGLE = 1,
                SHAPE_RECTANGLE = 2,
                SHAPE_CIRCLE = 3,
                SHAPE_WITH_HOLES = 4
            };
        };
    }
}