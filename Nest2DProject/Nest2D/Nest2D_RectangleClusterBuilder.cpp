#include "pch.h"
#include "Nest2D_RectangleClusterBuilder.h"

namespace ET {
    namespace NEST2DMANAGERLIB {

        CetRectangleClusterBuilder::CetRectangleClusterBuilder() : CetCoreObject() {}
        CetRectangleClusterBuilder::~CetRectangleClusterBuilder() {}
        void CetRectangleClusterBuilder::BuildCandidates(const CetTNestItemVector& AItems,const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices,const TetNestOptions& AOptions, std::vector<TetClusterCandidate>& AOut) {
            // 第一阶段只建立统一入口。矩形不生成组合候选，自动退化为单件。
        }

    }
}