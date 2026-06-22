#pragma once
#include "MenuRunnerBase.h"
#include "Nest2DTestApp.h"

namespace ET {
    namespace NEST2DTESTAPP {

        class CetBoardMenuRunner : public CetMenuRunnerBase
        {
        public:
            explicit CetBoardMenuRunner();
            virtual ~CetBoardMenuRunner();
            int SetTestApp(CetTestApp* ATestApp);
            int Run() override;

        protected:
            std::string _GetMenuTitle() const override;
            void _BindMenuItems();

        private:
            CetTestApp* m_pTestApp = nullptr;

            enum MetBoardMenuChoice
            {
                BOARD_NORMAL = 0,
                BOARD_L_SHAPE = 1,
                BOARD_CUSTOM_POLYGON = 2
            };
        };

    }
}