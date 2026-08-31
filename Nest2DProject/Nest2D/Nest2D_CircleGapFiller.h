#pragma once

#include "EtTechCore_Object.h"
#include "Nest2D_PrivateDataType.h"
#include <map>
#include <string>
#include <vector>

namespace ET { namespace NEST2DMANAGERLIB {

class CetCircleGapFiller : public ET::CORE::CetCoreObject {
    Inherit_Invoke_Hook(CetCircleGapFiller)

protected:
    int _Init() override { CetCoreObject::_Init(); return 0; }
    void _WrapFuncs() override {
        CetCoreObject::_WrapFuncs();
        _WrapFunc("BuildCircleGapCandidate", Type_Class_Func(BuildCircleGapCandidate));
        _WrapFunc("CollectCircleGapTemplateAnchors", Type_Class_Func(CollectCircleGapTemplateAnchors));
        _WrapFunc("CopyCircleGapTemplate", Type_Class_Func(CopyCircleGapTemplate));
        _WrapFunc("BuildFreeRegionTemplateCandidate", Type_Class_Func(BuildFreeRegionTemplateCandidate));
    }

public:
    CetCircleGapFiller();
    ~CetCircleGapFiller();

    bool BuildCircleGapCandidate(const TetClusterFillContext&, std::map<std::string, TetCircleGapTemplate>&, TetClusterCandidate&);
    std::vector<TetCircleGapTemplateAnchor> CollectCircleGapTemplateAnchors(const CetTNestItemVector&, const std::vector<TetShapeFeature>&, const TetClusterCandidate&);
    bool CopyCircleGapTemplate(const TetClusterFillContext&, const std::vector<TetCircleGapTemplateAnchor>&, const TetClusterCandidate&, std::size_t, TetClusterCandidate&, std::size_t&);
    bool BuildFreeRegionTemplateCandidate(const TetClusterFillContext&, const TetClusterCandidate&, TetClusterCandidate&);
};

}}
