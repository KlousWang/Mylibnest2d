#pragma once
#include "Nest2D_Def.h"
#include "Nest2D_DataType.h"
#include "Nest2D_PrivateDataType.h" // 包含你定义的 TNestItemVector 别名
#include "EtTechCore_InvokeFunctor.h"
#include "EtTechCore_Functor.h"
#include"Nest2D_Engine.h"
#include"Nest2D_Sort.h"
#include"NestDataMapper.h"
#include"ExportPhoto.h"
#include"Nest2D_BoardUtils.h"
#include"Nest2D_GeometryUtils.h"
#include"Nest2D_SvgUtils.h"
#include"Nest2D_PolygonBoardRepairer.h"
#include"Nest2D_StrategyManager.h"
#include"Nest2D_ClusterManager.h"

#include"Nest2D_ShapeAnalyzer.h"
#include"Nest2D_TriangleClusterBuilder.h"
#include"Nest2D_CircleClusterBuilder.h"

namespace ET {
    namespace NEST2DMANAGERLIB {

        // 继承自框架专属的“调用器”基类
        class CetNest2DInvokeFunctor : public ET::CORE::CetCoreInvokeFunctor
        {
            Inherit_Invoke_Hook(CetNest2DInvokeFunctor)

        protected:
            // 必须重写的生命周期函数，用于绑定所有的暗号
            int _EnableFunctor() override;

        public:
            CetNest2DInvokeFunctor();
            ~CetNest2DInvokeFunctor();    
        public:
			
            CetNest2DEngine* Nest2DEngineIns = nullptr;
            CetSort* Nest2DSortIns = nullptr;
            CetNestDataMapper* NestDataMapperIns = nullptr;
            CetExportPhoto* Nest2DExportPhoto = nullptr;
            CetNest2DBoardUtils* Nest2DBord = nullptr;
            CetGeometryUtils* Nest2DGeometryUtils = nullptr;
            CetSvgUtils* Nest2DSvgUtils = nullptr;
            CetPolygonBoardRepairer* Nest2DPolygonBord = nullptr;
            CetStrategyManager* Nest2DStrategy = nullptr;
            CetClusterManager* Nest2DCluster = nullptr;

            CetShapeAnalyzer* Nest2DShape = nullptr;
            CetTriangleClusterBuilder* Nest2dClusterTri = nullptr;
            CetCircleClusterBuilder* Nest2dClusterCircle =nullptr;
        protected:
            // 缓存底层引擎的指针
			ET::CORE::CetCoreObject* m_Engine = nullptr;
          //  ET::CORE::CetCoreObject* m_SvgUtils = nullptr;
           // ET::CORE::CetCoreObject* m_PolygonBoardRepairer = nullptr;

           // ET::CORE::CetCoreObject* m_StartegManager = nullptr;
           // ET::CORE::CetCoreObject* m_ClusterManager = nullptr;
            //CetSort* Nest2DSortIns = nullptr;
        public:   
			ET::CORE::CetCoreObjFunctor<int(CetTNestItemVector& , const TetNestOptions& , size_t*)> PerformNestingEx;
           /* ET::CORE::CetCoreObjFunctor<std::string(const TetNestBoard&,double)> MakeBoardSvgPath;
            ET::CORE::CetCoreObjFunctor<void(const std::string&,const std::string&)> InsertTextBeforeSvgEnd;*/

         /*   ET::CORE::CetCoreObjFunctor<void(CetTNestItemVector&,const TetNestOptions&,const libnest2d::PolygonImpl&,double,double)> SetPolygonBoardRepairContext;
            ET::CORE::CetCoreObjFunctor<void(std::size_t&)> RepairPolygonBoard;*/

         /*   ET::CORE::CetCoreObjFunctor<TetTNestEvalResult (const CetTNestItemVector& , std::size_t )> EvaluateNestResult;
            ET::CORE::CetCoreObjFunctor<bool (const TetTNestEvalResult& , const TetTNestEvalResult& ) > IsBetterNestResult;
            ET::CORE::CetCoreObjFunctor<void (CetTNestItemVector& , MetENestOrderStrategy )> ApplyNestPriorityStrategy;
            ET::CORE::CetCoreObjFunctor<void (const CetTNestItemVector& )> PrintBinCount;
            ET::CORE::CetCoreObjFunctor< TetTNestEvalResult (const CetTNestItemVector& , const std::vector<TetMetaItem>& , const CetTNestItemVector& , std::size_t )> EvaluatePackedResultWithMeta;*/

            //ET::CORE::CetCoreObjFunctor< TetClusterBuildResult (const CetTNestItemVector& , const TetNestOptions& , MetClusterStrategy )> BuildClusterItems;
            //ET::CORE::CetCoreObjFunctor< void (const CetTNestItemVector& , const CetTNestItemVector& , const std::vector<TetMetaItem>& ,CetTNestItemVector& )> ExpandClusterResultToOriginalItems;

        };
    }
}