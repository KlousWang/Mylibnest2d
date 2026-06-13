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
            m_ExportPhoto = m_LibConfig->GetLocalCoreObjIns("l_ExportPhoto");
            if (!m_ExportPhoto)return -1;
            m_NestDataMapper = m_LibConfig->GetLocalCoreObjIns("l_NestDataMapper");
            if (!m_NestDataMapper)return -1;
            // 2. 通过框架基类自带的 m_LibConfig，根据变量名抓取到底层的 Object 实例 [cite: 128, 149]
            // 注意：这里的 "g_Nest2DEngine" 是你在 JSON 中配置的 GlobalObjIns 变量名
            m_pNest2DEngine = m_LibConfig->GetLocalCoreObjIns("l_Nest2DEngine");
            if (!m_pNest2DEngine) return -1;
            //WWFunct1.Reload(m_pNest2DEngine, "func1");
            // 3. 将 Functor 变量与底层的真实字符串暗号 "RunNesting" 绑定起来 [cite: 84]
            CetNest2DInvokeFunctor::RunNestingFunctor.Reload(m_pNest2DEngine, "RunNesting");

            CetNest2DInvokeFunctor::BuildNestms.Reload(m_NestDataMapper, "BuildNestItems");
            CetNest2DInvokeFunctor::ApplyResults.Reload(m_NestDataMapper, "ApplyResults");

           
            CetNest2DInvokeFunctor::ExportSvg.Reload(m_ExportPhoto, "ExportSvg");

            return 0;
        }

        // 外部包裹函数的实现
        //int CetNest2DInvokeFunctor::RunNesting(CetTNestItemVector& ANestItems, const TetNestOptions& AOptions, std::size_t* AUsedBins)
        //{
        //    // 如果暗号绑定成功 (即底层引擎确实公布了这个函数) [cite: 74]
        //    if (m_RunNestingFunctor.GetResult()) {

        //        return m_RunNestingFunctor(ANestItems, AOptions, AUsedBins);
        //    }

        //    return -1; 
        //}

    }
}