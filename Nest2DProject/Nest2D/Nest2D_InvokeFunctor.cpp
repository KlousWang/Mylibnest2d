#include "pch.h"
#include "Nest2D_InvokeFunctor.h"

namespace ET {
    namespace NEST2DMANAGERLIB {

        CetNest2DInvokeFunctor::CetNest2DInvokeFunctor() : ET::CORE::CetCoreInvokeFunctor() {}
        CetNest2DInvokeFunctor::~CetNest2DInvokeFunctor() {}

        int CetNest2DInvokeFunctor::_EnableFunctor()
        {

            CetCoreInvokeFunctor::_EnableFunctor();
            // WWFunct1.Reload(_);
            Nest2DExportPhoto = (decltype(Nest2DExportPhoto))m_LibConfig->GetLocalCoreObjIns("l_ExportPhoto");
            // if (!m_ExportPhoto)return -1;
            NestDataMapperIns = (decltype(NestDataMapperIns))m_LibConfig->GetLocalCoreObjIns("l_NestDataMapper");
            /* if (!m_NestDataMapper)return -1;*/

            Nest2DEngineIns = (decltype(Nest2DEngineIns))m_LibConfig->GetLocalCoreObjIns("l_Nest2DEngine");
            m_Engine = m_LibConfig->CreateCoreObj("Nest2D_Engine");
            // if (!Nest2DEngineIns) return -1;

            Nest2DBord = (decltype(Nest2DBord))m_LibConfig->GetLocalCoreObjIns("l_BoardUtils");
            // if (!m_BoardUtils) return -1;

            Nest2DGeometryUtils = (decltype(Nest2DGeometryUtils))m_LibConfig->GetLocalCoreObjIns("l_GeometryUtils");
            // if (!m_GeometryUtils) return -1;

            Nest2DSvgUtils = (decltype(Nest2DSvgUtils))m_LibConfig->GetLocalCoreObjIns("l_SvgUtils");
            // if (!m_SvgUtils) return -1;

            Nest2DPolygonBord = (decltype(Nest2DPolygonBord))m_LibConfig->GetLocalCoreObjIns("l_PolygonBoardRepairer");
            // if (!m_PolygonBoardRepairer) return -1;

            Nest2DCluster = (decltype(Nest2DCluster))m_LibConfig->GetLocalCoreObjIns("l_ClusterManager");
            // if (!m_ClusterManager) return -1;

            Nest2DStrategy = (decltype(Nest2DStrategy))m_LibConfig->GetLocalCoreObjIns("l_StrategyManager");

            Nest2DAreaUsage = (decltype(Nest2DAreaUsage))m_LibConfig->GetLocalCoreObjIns("l_AreaUsageCalculator");

            Nest2DSortIns = (decltype(Nest2DSortIns))m_LibConfig->GetLocalCoreObjIns("l_Sort");

            Nest2DShape = (decltype(Nest2DShape))m_LibConfig->GetLocalCoreObjIns("l_ShapeAnalyzer");
            Nest2dClusterTri = (decltype(Nest2dClusterTri))m_LibConfig->GetLocalCoreObjIns("l_TriangleClusterBuilder");
            Nest2dClusterCircle = (decltype(Nest2dClusterCircle))m_LibConfig->GetLocalCoreObjIns("l_CircleClusterBuilder");
            Nest2dLocalCompactor = (decltype(Nest2dLocalCompactor))m_LibConfig->GetLocalCoreObjIns("l_LocalCompactor");
            Nest2dQuarterTurnOptimizer = (decltype(Nest2dQuarterTurnOptimizer))m_LibConfig->GetLocalCoreObjIns("l_QuarterTurnOptimizer");
            Nest2dRectangleGridOptimizer = (decltype(Nest2dRectangleGridOptimizer))m_LibConfig->GetLocalCoreObjIns("l_RectangleGridOptimizer");
            Nest2dFreeSpaceEvaluator = (decltype(Nest2dFreeSpaceEvaluator))m_LibConfig->GetLocalCoreObjIns("l_FreeSpaceEvaluator");
            Nest2dRotationUtils = (decltype(Nest2dRotationUtils))m_LibConfig->GetLocalCoreObjIns("l_RotationUtils");
            Nest2dAutoPairClusterBuilder = (decltype(Nest2dAutoPairClusterBuilder))m_LibConfig->GetLocalCoreObjIns("l_AutoPairClusterBuilder");
            Nest2dClusterTemplateFillOptimizer = (decltype(Nest2dClusterTemplateFillOptimizer))m_LibConfig->GetLocalCoreObjIns("l_ClusterTemplateFillOptimizer");
            Nest2dCircleGapFiller = (decltype(Nest2dCircleGapFiller))m_LibConfig->GetLocalCoreObjIns("l_CircleGapFiller");
            Nest2dEllipseGapFiller = (decltype(Nest2dEllipseGapFiller))m_LibConfig->GetLocalCoreObjIns("l_EllipseGapFiller");
            PerformNestingEx.Reload(m_Engine, "RunNesting");
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

    } // namespace NEST2DMANAGERLIB
} // namespace ET
