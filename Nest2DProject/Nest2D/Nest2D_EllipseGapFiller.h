#pragma once

#include "EtTechCore_Object.h"
#include "Nest2D_PrivateDataType.h"

namespace ET { namespace NEST2DMANAGERLIB {

class CetEllipseGapFiller : public ET::CORE::CetCoreObject {
    Inherit_Invoke_Hook(CetEllipseGapFiller)

protected:
    int _Init() override { CetCoreObject::_Init(); return 0; }
    void _WrapFuncs() override {
        CetCoreObject::_WrapFuncs();
        _WrapFunc("BuildEllipseGapCandidate", Type_Class_Func(BuildEllipseGapCandidate));
        _WrapFunc("TryBuildCachedEllipseTemplateVariant", Type_Class_Func(TryBuildCachedEllipseTemplateVariant));
        _WrapFunc("CacheEllipseTemplateVariant", Type_Class_Func(CacheEllipseTemplateVariant));
    }

public:
    CetEllipseGapFiller();
    ~CetEllipseGapFiller();

    bool BuildEllipseGapCandidate(const TetClusterFillContext&, const TetClusterFillSearchConfig&, TetClusterCandidate&);
    bool TryBuildCachedEllipseTemplateVariant(const TetClusterFillContext&, const TetEllipseGapTemplateCache&, TetClusterCandidate&);
    void CacheEllipseTemplateVariant(const TetClusterFillContext&, const std::vector<TetClusterFillSearchState>&, bool, TetEllipseGapTemplateCache&);
};

}}
