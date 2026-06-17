#pragma once
#include "Nest2D_Def.h"
#include "Nest2D_DataType.h"
#include "Nest2D_PrivateDataType.h" // 包含你定义的 TNestItemVector 别名
#include "EtTechCore_InvokeFunctor.h"
#include "EtTechCore_Functor.h"

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

        protected:
            // 缓存底层引擎的指针
            ET::CORE::CetCoreObject* m_pNest2DEngine = nullptr;
            ET::CORE::CetCoreObject* m_NestDataMapper = nullptr;
            ET::CORE::CetCoreObject* m_ExportPhoto = nullptr;

            ET::CORE::CetCoreObject* m_BoardUtils = nullptr;
            ET::CORE::CetCoreObject* m_GeometryUtils = nullptr;
            ET::CORE::CetCoreObject* m_SvgUtils = nullptr;
            ET::CORE::CetCoreObject* m_PolygonBoardRepairer = nullptr;
        public:   
            //ET::CORE::CetCoreObjFunctor<int(int, int)> WWFunct1;
            ET::CORE::CetCoreObjFunctor<int(CetTNestItemVector&, const TetNestOptions&, std::size_t*)> RunNestingFunctor;
            ET::CORE::CetCoreObjFunctor<void(const std::vector< TetNestPolygon>&, CetTNestItemVector&)>BuildNestms;
            ET::CORE::CetCoreObjFunctor<void(const CetTNestItemVector&, std::vector<TetNestPolygon>&)>ApplyResults;
            ET::CORE::CetCoreObjFunctor<int(CetTNestItemVector&, const TetNestOptions&, int)>ExportSvg;
            ET::CORE::CetCoreObjFunctor<int(const std::vector<TetNestPolygon>&, const TetNestOptions&, int)>ExportSvgbd;

            ET::CORE::CetCoreObjFunctor<TetBoardBounds(const TetNestBoard&)> CalcBoardBoundsLocal;
            ET::CORE::CetCoreObjFunctor<ClipperLib::Path(const std::vector<TetNestPoint>&,double,double,bool)> BuildPathFromPoints;
            ET::CORE::CetCoreObjFunctor<libnest2d::PolygonImpl(const TetNestOptions&,double&,double&)> BuildBinPolygonFromOptions;
            ET::CORE::CetCoreObjFunctor<TetNestPoint(const TetNestPoint&,double,double,double)> TransformPoint;
            ET::CORE::CetCoreObjFunctor<double(double)> RadToDeg;
            ET::CORE::CetCoreObjFunctor<bool(const TetNestPoint&,const std::vector<TetNestPoint>&)> PointInPolygon;
            ET::CORE::CetCoreObjFunctor<bool(const TetNestPoint&,const TetNestBoard&)> IsPointInsideBoard;
            ET::CORE::CetCoreObjFunctor<void(std::vector<TetNestPolygon>&,const TetNestBoard&)> ValidateItemsInsideBoard;
            ET::CORE::CetCoreObjFunctor<std::string(const TetNestBoard&,double)> MakeBoardSvgPath;
            ET::CORE::CetCoreObjFunctor<void(const std::string&,const std::string&)> InsertTextBeforeSvgEnd;
            ET::CORE::CetCoreObjFunctor<void(CetTNestItemVector&,const TetNestOptions&,const libnest2d::PolygonImpl&,double,double)> SetPolygonBoardRepairContext;
            ET::CORE::CetCoreObjFunctor<void(std::size_t&)> RepairPolygonBoard;
        };
    }
}