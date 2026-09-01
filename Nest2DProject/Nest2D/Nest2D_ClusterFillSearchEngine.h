#pragma once

#include "EtTechCore_Object.h"
#include "Nest2D_PrivateDataType.h"

#include <cstdint>
#include <cstddef>
#include <map>
#include <string>
#include <vector>

namespace ET { namespace NEST2DMANAGERLIB {

class CetClusterFillSearchEngine : public ET::CORE::CetCoreObject {
    Inherit_Invoke_Hook(CetClusterFillSearchEngine)

protected:
    int _Init() override { CetCoreObject::_Init(); return 0; }
    void _WrapFuncs() override { CetCoreObject::_WrapFuncs(); }

public:
    CetClusterFillSearchEngine();
    ~CetClusterFillSearchEngine();

    static TetClusterFillSearchConfig GetClusterFillSearchConfig(std::size_t);
    static TetClusterFillSearchConfig GetClusterEnvelopeFillSearchConfig(std::size_t);
    static std::uint64_t MakeFillerFamilyKey(const TetShapeFeature &);
    static bool BuildRectangleEnvelopeCandidate(const CetTNestItemVector &, const TetNestOptions &, const TetClusterCandidate &, TetClusterCandidate &);
    void BuildFilledVariantsForBase(const TetClusterFillContext &, const TetClusterFillSearchConfig &, std::vector<TetClusterCandidate> &, TetClusterFillSearchStats &);
    void BuildEnvelopeFilledVariantsForBase(const TetEnvelopeFillVariantRequest &);

protected:
    bool _IsFillMetricLess(double, double) const;
    bool _IsFilledVariantBetter(const TetClusterFillSearchState &, const TetClusterFillSearchState &) const;
    bool _IsFilledVariantWorthKeeping(const TetClusterCandidate &, const TetClusterCandidate &) const;
    bool _IsEnvelopeFillStateWorthExpanding(const TetClusterCandidate &, const TetClusterCandidate &) const;
    bool _RebuildEnvelopeFillWithTrueContour(const CetTNestItemVector &, const TetNestOptions &, const TetClusterCandidate &, const TetClusterCandidate &, TetClusterCandidate &);
    std::string _MakeFilledVariantKey(const TetClusterCandidate &) const;
    void _DeduplicateFilledStates(std::vector<TetClusterFillSearchState> &);
    void _TrimFillBeam(std::vector<TetClusterFillSearchState> &, std::size_t);
    bool _IsEnvelopeStateBetter(const TetClusterFillSearchState &, const TetClusterFillSearchState &) const;
    std::uint64_t _MakeFillerFamilyKey(const TetShapeFeature &) const;
    std::uint64_t _GetEnvelopeSeedFamilyKey(const TetClusterFillSearchState &, const std::vector<TetShapeFeature> &, std::size_t) const;
    void _TrimEnvelopeBeam(std::vector<TetClusterFillSearchState> &, std::size_t, const std::vector<TetShapeFeature> &, std::size_t, bool = false);
    void _FinalizeEnvelopeFilledStates(const TetClusterFillContext &, std::vector<TetClusterFillSearchState> &, std::vector<TetClusterCandidate> &, TetClusterFillSearchStats &);
    std::vector<int> _CollectCompatibleFillers(const std::vector<TetShapeFeature> &, const TetClusterCandidate &, const std::vector<TetClusterFreeRegion> &, const TetClusterFillSearchConfig &, bool = false) const;
    TetClusterFillSearchState _BuildEnvelopeFillSeed(const TetClusterFillContext &, std::map<std::string, TetCircleGapTemplate> &, const TetClusterFillSearchState *, bool, bool, const TetClusterFillSearchConfig &, std::vector<TetClusterFillSearchState> &);
    void _SearchEnvelopeFillVariants(const TetClusterFillContext &, const TetClusterFillSearchConfig &, std::map<std::string, TetCircleGapTemplate> &, const TetClusterFillSearchState *, std::vector<TetClusterFillSearchState> &, TetClusterFillSearchStats &);
    bool _TryBuildCachedEllipseTemplateSeed(const TetClusterFillContext &, const TetEllipseGapTemplateCache &, TetClusterFillSearchState &);
};

}}
