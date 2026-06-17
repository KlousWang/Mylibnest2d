#pragma once

//#include "Nest2D_Def.h"
#include "Nest2D_DataType.h"
#include"NestTestData_DataType.h"
//#include "Nest2D_PrivateDataType.h"

#include "EtTechCore_InvokeFunctor.h"
#include "EtTechCore_Functor.h"

namespace ET {
    namespace NEST2DTESTAPP {

        class CetNest2DTestInvokeFunctor : public ET::CORE::CetCoreInvokeFunctor
        {
            Inherit_Invoke_Hook(CetNest2DTestInvokeFunctor)

        public:
            CetNest2DTestInvokeFunctor();
             ~CetNest2DTestInvokeFunctor();

        protected:
            int _EnableFunctor() override;

        private:
            ET::CORE::CetCoreObject* m_pCreateTestData = nullptr;
            ET::CORE::CetCoreObject* m_pFile = nullptr;
            ET::CORE::CetCoreObject* m_pNest2D = nullptr;

        public:
            ET::CORE::CetCoreObjFunctor<int(double, double, double, int)>Init;
            ET::CORE::CetCoreObjFunctor<void(int, double, double, bool) > AddTrigle;
            ET::CORE::CetCoreObjFunctor<void(int, double, double, double) > AddCustomTrigle;
            ET::CORE::CetCoreObjFunctor<void(int, double, double) > AddRectangle;
            ET::CORE::CetCoreObjFunctor<void(int, double, bool, double, double) > AddCircle;
            ET::CORE::CetCoreObjFunctor<void() > ClearData;
            ET::CORE::CetCoreObjFunctor<void(int) > AddCustomShapeWithHolesByInput;
            ET::CORE::CetCoreObjFunctor<void(CetVertices&&, std::vector<CetVertices>&&) > AddBoardWithHoles;
            ET::CORE::CetCoreObjFunctor<void(CetVertices&&)> AddBoard;
            ET::CORE::CetCoreObjFunctor<bool(const std::string&)>SaveToFile;
            ET::CORE::CetCoreObjFunctor<int(const std::string&, TetNestOptions&, std::vector<TetNestPolygon>&, std::string*)>LoadFile;
            ET::CORE::CetCoreObjFunctor<int(const std::string&, const TetNestOptions&, const std::vector<TetNestPolygon>&, int)>SaveCoordinatesFile;
            ET::CORE::CetCoreObjFunctor<int(std::vector<TetNestPolygon>&, const TetNestOptions&, TetNestResult*)>PerformNest;
        };

    }
}