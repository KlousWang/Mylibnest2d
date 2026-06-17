#include "Nest2DTest_InvokeFunctor.h"
namespace ET {
    namespace NEST2DTESTAPP {
        CetNest2DTestInvokeFunctor::CetNest2DTestInvokeFunctor () :  ET::CORE::CetCoreInvokeFunctor()
        {
        }

        CetNest2DTestInvokeFunctor::~CetNest2DTestInvokeFunctor()
        {
        }

        int CetNest2DTestInvokeFunctor::_EnableFunctor()
        {
            CetCoreInvokeFunctor::_EnableFunctor();
            m_pCreateTestData =ET::CORE::CetCoreObjStorage::GetClassIns("gCreateTestData");
            if (!m_pCreateTestData)return -1;
            m_pFile =ET::CORE::CetCoreObjStorage::GetClassIns("gFile");
            if (!m_pFile)return -1;
            m_pNest2D =ET::CORE::CetCoreObjStorage::GetClassIns("gNest2D");
            if (!m_pNest2D)return -1;

            CetNest2DTestInvokeFunctor::Init.Reload(m_pCreateTestData, "Init");
            CetNest2DTestInvokeFunctor::AddTrigle.Reload(m_pCreateTestData, "AddTriangle");
            CetNest2DTestInvokeFunctor::AddCustomTrigle.Reload(m_pCreateTestData, "AddTriangleBySides");
            CetNest2DTestInvokeFunctor::AddRectangle.Reload(m_pCreateTestData, "AddRectangle");
            CetNest2DTestInvokeFunctor::AddCircle.Reload(m_pCreateTestData, "AddCircle");
            CetNest2DTestInvokeFunctor::ClearData.Reload(m_pCreateTestData, "ClearPolygons");
            CetNest2DTestInvokeFunctor::AddCustomShapeWithHolesByInput.Reload(m_pCreateTestData, "AddCustomShapeWithHolesByInput");
            CetNest2DTestInvokeFunctor::AddBoardWithHoles.Reload(m_pCreateTestData, "AddBoardWithHoles");
            CetNest2DTestInvokeFunctor::AddBoard.Reload(m_pCreateTestData, "AddBoard");
            CetNest2DTestInvokeFunctor::SaveToFile.Reload(m_pCreateTestData, "SaveToFile");
            CetNest2DTestInvokeFunctor::LoadFile.Reload(m_pFile, "LoadNestCaseFromFile");
            CetNest2DTestInvokeFunctor::SaveCoordinatesFile.Reload(m_pFile, "SaveNestResultToFile");
            CetNest2DTestInvokeFunctor::PerformNest.Reload(m_pNest2D, "PerformNestingEx");
            return 0;
        }
    }
}