#include"pch.h"
#include "Nest2D_InvokeFunctor.h"

namespace ET {
    namespace NEST2DMANAGERLIB {

        CetNest2DInvokeFunctor::CetNest2DInvokeFunctor() :ET::CORE::CetCoreInvokeFunctor() {}
        CetNest2DInvokeFunctor::~CetNest2DInvokeFunctor() {}

        int CetNest2DInvokeFunctor::_EnableFunctor()
        {
            // 1. 必须先调用基类的初始化 [cite: 150]
            CetCoreInvokeFunctor::_EnableFunctor();
            //WWFunct1.Reload(_);
            Nest2DExportPhoto = (decltype(Nest2DExportPhoto))m_LibConfig->GetLocalCoreObjIns("l_ExportPhoto");
            //if (!m_ExportPhoto)return -1;
            NestDataMapperIns = (decltype(NestDataMapperIns))m_LibConfig->GetLocalCoreObjIns("l_NestDataMapper");
           /* if (!m_NestDataMapper)return -1;*/
            // 2. 通过框架基类自带的 m_LibConfig，根据变量名抓取到底层的 Object 实例 [cite: 128, 149]
            // 注意：这里的 "g_Nest2DEngine" 是你在 JSON 中配置的 GlobalObjIns 变量名
			Nest2DEngineIns = (decltype(Nest2DEngineIns))m_LibConfig->GetLocalCoreObjIns("l_Nest2DEngine");
            m_Engine =m_LibConfig->CreateCoreObj("Nest2D_Engine");
           // if (!Nest2DEngineIns) return -1;

            Nest2DBord =(decltype(Nest2DBord)) m_LibConfig->GetLocalCoreObjIns("l_BoardUtils");
           // if (!m_BoardUtils) return -1;

            Nest2DGeometryUtils =(decltype(Nest2DGeometryUtils)) m_LibConfig->GetLocalCoreObjIns("l_GeometryUtils");
            //if (!m_GeometryUtils) return -1;

            Nest2DSvgUtils =(decltype(Nest2DSvgUtils)) m_LibConfig->GetLocalCoreObjIns("l_SvgUtils");
           // if (!m_SvgUtils) return -1;

            Nest2DPolygonBord =(decltype(Nest2DPolygonBord)) m_LibConfig->GetLocalCoreObjIns("l_PolygonBoardRepairer");
            //if (!m_PolygonBoardRepairer) return -1;

			Nest2DCluster =(decltype(Nest2DCluster)) m_LibConfig->GetLocalCoreObjIns("l_ClusterManager");
			//if (!m_ClusterManager) return -1;

			Nest2DStrategy =(decltype(Nest2DStrategy)) m_LibConfig->GetLocalCoreObjIns("l_StrategyManager");
		    
            Nest2DSortIns = (decltype(Nest2DSortIns))m_LibConfig->GetLocalCoreObjIns("l_Sort");


            const char* FuncName = m_LibConfig->GetClassFuncName("Nest2D_Engine", "localNest2D_Engine");
      
			PerformNestingEx.Reload(m_Engine, FuncName);
           /* CetNest2DInvokeFunctor::MakeBoardSvgPath.Reload(m_SvgUtils, "MakeBoardSvgPath");
            CetNest2DInvokeFunctor::InsertTextBeforeSvgEnd.Reload(m_SvgUtils, "InsertTextBeforeSvgEnd");*/

           /* CetNest2DInvokeFunctor::SetPolygonBoardRepairContext.Reload(m_PolygonBoardRepairer, "SetContext");
            CetNest2DInvokeFunctor::RepairPolygonBoard.Reload(m_PolygonBoardRepairer, "Repair");*/

		/*	CetNest2DInvokeFunctor::EvaluateNestResult.Reload(m_StartegManager, "EvaluateNestResult");
			CetNest2DInvokeFunctor::IsBetterNestResult.Reload(m_StartegManager, "IsBetterNestResult");
			CetNest2DInvokeFunctor::ApplyNestPriorityStrategy.Reload(m_StartegManager, "ApplyNestPriorityStrategy");
			CetNest2DInvokeFunctor::PrintBinCount.Reload(m_StartegManager, "PrintBinCount");
			CetNest2DInvokeFunctor::EvaluatePackedResultWithMeta.Reload(m_StartegManager, "EvaluatePackedResultWithMeta");*/

		/*	CetNest2DInvokeFunctor::BuildClusterItems.Reload(m_ClusterManager, "BuildClusterItems");
			CetNest2DInvokeFunctor::ExpandClusterResultToOriginalItems.Reload(m_ClusterManager, "ExpandClusterResultToOriginalItems");*/

            return 0;
        }

    }
}