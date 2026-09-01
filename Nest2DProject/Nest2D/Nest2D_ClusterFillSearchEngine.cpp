#include "pch.h"
#include "Nest2D_ClusterFillSearchEngine.h"
#include "Nest2D_CircleGapFiller.h"
#include "Nest2D_EllipseGapFiller.h"
#include "Nest2D_ClusterGeometryHelper.h"
#include "Nest2D_RectangleFillClusterBuilder.h"
#include "Nest2D_SelfFunction.h"
#include "NestUtils.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>

using namespace ClipperLib;
using namespace libnest2d;

namespace ET { namespace NEST2DMANAGERLIB {
    CetNest2DInvokeFunctor *_GetNest2DInvokeFunctor();
}}

namespace {
    bool CircleGapSearchTimeReached(const std::chrono::steady_clock::time_point &AStart, long long ALimitMs)
    {
        return ALimitMs > 0 && std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - AStart).count() >= ALimitMs;
    }
    bool ContainsOriginalIndex(const TetClusterCandidate &ACandidate, int AOriginalIndex)
    {
        return std::find(ACandidate.OriginalIndices.begin(), ACandidate.OriginalIndices.end(), AOriginalIndex) != ACandidate.OriginalIndices.end();
    }
    /* Legacy geometry helper moved to CetClusterGeometryHelper.
    bool HasFullRectangleProxy(const TetClusterCandidate &ACandidate)
    {
        const double BoundingArea = ACandidate.ClusterWidth * ACandidate.ClusterHeight;
        const double AreaTolerance = std::max(1.0, BoundingArea * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE);
        return ACandidate.Valid && ACandidate.ClusterWidth > 0.0 && ACandidate.ClusterHeight > 0.0 && std::isfinite(BoundingArea) && std::abs(ACandidate.ProxyArea - BoundingArea) <= AreaTolerance;
    }
    */
    bool IsFixedCircleEnvelopeBase(const TetClusterCandidate &ACandidate, const std::vector<TetShapeFeature> &AFeatures)
    {
        if (!ACandidate.Valid || ACandidate.BuilderName != "CircleBuilder" || ACandidate.OriginalIndices.size() < 2)
            return false;
        for (int Index : ACandidate.OriginalIndices) {
            if (Index < 0 || Index >= static_cast<int>(AFeatures.size()) || AFeatures[Index].ShapeType != MetShapeType::CircleLike)
                return false;
        }
        return true;
    }
    bool IsFixedEllipseEnvelopeBase(const TetClusterCandidate &ACandidate, const std::vector<TetShapeFeature> &AFeatures)
    {
        if (!ACandidate.Valid || ACandidate.BuilderName != "EllipseBuilder" || ACandidate.OriginalIndices.size() < 2)
            return false;
        for (int Index : ACandidate.OriginalIndices) {
            if (Index < 0 || Index >= static_cast<int>(AFeatures.size()) || AFeatures[Index].ShapeType != MetShapeType::EllipseLike)
                return false;
        }
        return true;
    }
}

namespace ET { namespace NEST2DMANAGERLIB {
    CetClusterFillSearchEngine::CetClusterFillSearchEngine() : CetCoreObject() {}
    CetClusterFillSearchEngine::~CetClusterFillSearchEngine() {}

    TetClusterFillSearchConfig CetClusterFillSearchEngine::GetClusterFillSearchConfig(std::size_t AItemCount)
    {
        if (AItemCount > CET_NEST_REDUCED_STRATEGY_ITEM_LIMIT)
            return {CET_CLUSTER_FILL_REDUCED_ORDER_BEAM_WIDTH, std::max<std::size_t>(1, AItemCount), std::max<std::size_t>(1, AItemCount), 0};
        if (AItemCount > CET_NEST_FULL_STRATEGY_ITEM_LIMIT)
            return {CET_CLUSTER_FILL_REDUCED_ORDER_BEAM_WIDTH, std::max<std::size_t>(1, AItemCount), std::max<std::size_t>(1, AItemCount), 0};
        return {};
    }
    TetClusterFillSearchConfig CetClusterFillSearchEngine::GetClusterEnvelopeFillSearchConfig(std::size_t AItemCount)
    {
        const std::size_t InventoryDepth = std::max<std::size_t>(1, AItemCount);
        if (AItemCount > CET_NEST_FULL_STRATEGY_ITEM_LIMIT)
            return {CET_CLUSTER_ENVELOPE_FILL_LARGE_ORDER_BEAM_WIDTH, std::min(InventoryDepth, CET_CLUSTER_ENVELOPE_FILL_LARGE_ORDER_MAX_DEPTH), CET_CLUSTER_ENVELOPE_FILL_LARGE_ORDER_MAX_CANDIDATE_FILLERS, CET_CLUSTER_ENVELOPE_FILL_LARGE_ORDER_MAX_PLACEMENT_ATTEMPTS, CET_CLUSTER_ENVELOPE_FILL_LARGE_ORDER_MIN_DEPTH_BEFORE_TIMEOUT, CET_CLUSTER_ENVELOPE_FILL_LARGE_ORDER_MAX_SEARCH_TIME_MS};
        return {CET_CLUSTER_ENVELOPE_FILL_BEAM_WIDTH, std::min(InventoryDepth, CET_CLUSTER_ENVELOPE_FILL_MAX_DEPTH), CET_CLUSTER_ENVELOPE_FILL_MAX_CANDIDATE_FILLERS, CET_CLUSTER_ENVELOPE_FILL_MAX_PLACEMENT_ATTEMPTS, CET_CLUSTER_ENVELOPE_FILL_MIN_DEPTH_BEFORE_TIMEOUT, CET_CLUSTER_ENVELOPE_FILL_MAX_SEARCH_TIME_MS};
    }
    bool CetClusterFillSearchEngine::_IsFillMetricLess(double ALeft, double ARight) const
    {
        return ALeft < ARight - CET_CLUSTER_FILL_VARIANT_POSITION_TOLERANCE;
    }
    bool CetClusterFillSearchEngine::_IsFilledVariantBetter(const TetClusterFillSearchState &AFirst, const TetClusterFillSearchState &ASecond) const
    {
        const TetClusterCandidate &First = AFirst.Candidate;
        const TetClusterCandidate &Second = ASecond.Candidate;
        if (_IsFillMetricLess(First.ProxyWasteArea, Second.ProxyWasteArea)) return true;
        if (_IsFillMetricLess(Second.ProxyWasteArea, First.ProxyWasteArea)) return false;
        if (First.FillRatio > Second.FillRatio + 1e-9) return true;
        if (Second.FillRatio > First.FillRatio + 1e-9) return false;
        if (First.ProxyWasteRatio < Second.ProxyWasteRatio - 1e-9) return true;
        if (Second.ProxyWasteRatio < First.ProxyWasteRatio - 1e-9) return false;
        if (_IsFillMetricLess(First.ProxyArea, Second.ProxyArea)) return true;
        if (_IsFillMetricLess(Second.ProxyArea, First.ProxyArea)) return false;
        if (First.FragmentationRisk < Second.FragmentationRisk - 1e-9) return true;
        if (Second.FragmentationRisk < First.FragmentationRisk - 1e-9) return false;
        if (AFirst.FillerCount != ASecond.FillerCount) return AFirst.FillerCount < ASecond.FillerCount;
        return First.OriginalIndices < Second.OriginalIndices;
    }
    bool CetClusterFillSearchEngine::_IsFilledVariantWorthKeeping(const TetClusterCandidate &ABaseCandidate, const TetClusterCandidate &ACandidate) const
    {
        const double AreaTolerance = std::max(1.0, ABaseCandidate.ProxyArea * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE);
        return ACandidate.Valid && ACandidate.ProxyWasteArea < ABaseCandidate.ProxyWasteArea - AreaTolerance && ACandidate.FillRatio > ABaseCandidate.FillRatio + 1e-9;
    }
    bool CetClusterFillSearchEngine::BuildRectangleEnvelopeCandidate(const CetTNestItemVector &AOriginalItems, const TetNestOptions &AOptions, const TetClusterCandidate &ABaseCandidate, TetClusterCandidate &AOutEnvelopeCandidate)
    {
        AOutEnvelopeCandidate = TetClusterCandidate{};
        if (!ABaseCandidate.Valid || ABaseCandidate.OriginalIndices.size() < 2 || CetClusterGeometryHelper::HasFullRectangleProxy(ABaseCandidate)) {
            return false;
        }
        AOutEnvelopeCandidate = ABaseCandidate;
        ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
        if (!Geometry.FinalizeCandidateInRectangle(AOriginalItems, AOptions, AOutEnvelopeCandidate, ABaseCandidate.ClusterWidth, ABaseCandidate.ClusterHeight))
            return false;
        AOutEnvelopeCandidate.BuilderName = ABaseCandidate.BuilderName;
        AOutEnvelopeCandidate.ClusterType = ABaseCandidate.ClusterType;
        return AOutEnvelopeCandidate.ProxyArea > ABaseCandidate.ProxyArea + std::max(1.0, ABaseCandidate.ProxyArea * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE);
    }
    bool CetClusterFillSearchEngine::_IsEnvelopeFillStateWorthExpanding(const TetClusterCandidate &AEnvelopeCandidate, const TetClusterCandidate &ACandidate) const
    {
        const double EnvelopeTolerance = std::max(1.0, AEnvelopeCandidate.ProxyArea * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE);
        return ACandidate.Valid && std::abs(ACandidate.ProxyArea - AEnvelopeCandidate.ProxyArea) <= EnvelopeTolerance && (ACandidate.Transforms.size() > AEnvelopeCandidate.Transforms.size() || ACandidate.ProxyWasteArea < AEnvelopeCandidate.ProxyWasteArea - EnvelopeTolerance);
    }
    /* Legacy transform-preservation helper moved to CetClusterGeometryHelper.
    bool CetClusterFillSearchEngine::_PreservesBaseTransforms(const TetClusterCandidate &ABaseCandidate, const TetClusterCandidate &ACandidate) const
    {
        if (ABaseCandidate.Transforms.size() > ACandidate.Transforms.size())
            return false;
        for (std::size_t Index = 0; Index < ABaseCandidate.Transforms.size(); ++Index) {
            const TetItemTransform &Base = ABaseCandidate.Transforms[Index];
            const TetItemTransform &Current = ACandidate.Transforms[Index];
            if (Base.OriginalId != Current.OriginalId || std::abs(Base.RelativeX - Current.RelativeX) > CET_RECTANGLE_FILL_POSITION_TOLERANCE || std::abs(Base.RelativeY - Current.RelativeY) > CET_RECTANGLE_FILL_POSITION_TOLERANCE || std::abs(Base.RelativeRotation - Current.RelativeRotation) > CET_CLUSTER_FILL_VARIANT_ROTATION_TOLERANCE)
                return false;
        }
        return true;
    }
    */
    bool CetClusterFillSearchEngine::_RebuildEnvelopeFillWithTrueContour(const CetTNestItemVector &AOriginalItems, const TetNestOptions &AOptions, const TetClusterCandidate &ABaseCandidate, const TetClusterCandidate &AEnvelopeFilledCandidate, TetClusterCandidate &AOutCandidate)
    {
        AOutCandidate = TetClusterCandidate{};
        if (!AEnvelopeFilledCandidate.Valid || AEnvelopeFilledCandidate.OriginalIndices.size() <= ABaseCandidate.OriginalIndices.size())
            return false;
        AOutCandidate = AEnvelopeFilledCandidate;
        const bool PreserveBaseOutline = AEnvelopeFilledCandidate.BuilderName == "FixedOutlineGapFill";
        ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
        if (!Geometry.FinalizeCandidate(AOriginalItems, AOptions, AOutCandidate))
            return false;
        const TetClusterCandidate &ProxySource = PreserveBaseOutline ? ABaseCandidate : AEnvelopeFilledCandidate;
        // Internal fills retain the original outline; other envelope variants retain
        // the rectangle selected by the pre-existing envelope search.
        AOutCandidate.ProxyContour = ProxySource.ProxyContour;
        AOutCandidate.ProxyContourNormalized = ProxySource.ProxyContourNormalized;
        AOutCandidate.ProxyArea = ProxySource.ProxyArea;
        AOutCandidate.ProxyMode = ProxySource.ProxyMode;
        AOutCandidate.ClusterWidth = ProxySource.ClusterWidth;
        AOutCandidate.ClusterHeight = ProxySource.ClusterHeight;
        AOutCandidate.BoundingBoxArea = ProxySource.BoundingBoxArea;
        AOutCandidate.ReservedArea = std::min(AOutCandidate.ProxyArea, AOutCandidate.RealArea);
        AOutCandidate.ProxyWasteArea = std::max(0.0, AOutCandidate.ProxyArea - AOutCandidate.ReservedArea);
        AOutCandidate.ProxyWasteRatio = AOutCandidate.ProxyArea > 0.0 ? AOutCandidate.ProxyWasteArea / AOutCandidate.ProxyArea : 1.0;
        AOutCandidate.FillRatio = AOutCandidate.ProxyArea > 0.0 ? std::clamp(AOutCandidate.RealArea / AOutCandidate.ProxyArea, 0.0, 1.0) : 0.0;
        AOutCandidate.BoundingFillRatio = AOutCandidate.FillRatio;
        const std::size_t FillerCount = AOutCandidate.OriginalIndices.size() - ABaseCandidate.OriginalIndices.size();
        const double TrueDensityGain = AOutCandidate.FillRatio - ABaseCandidate.FillRatio;
        AOutCandidate.BuilderName = PreserveBaseOutline ? "FixedOutlineGapFill" : "EnvelopeFillSearch";
        AOutCandidate.ClusterType = ABaseCandidate.ClusterType + (PreserveBaseOutline ? "_InnerFill" : "_EnvelopeFill");
        AOutCandidate.Score = std::max(AOutCandidate.Score, ABaseCandidate.Score + static_cast<double>(FillerCount) * CET_CLUSTER_ENVELOPE_FILL_CHILD_SCORE + TrueDensityGain * CET_CLUSTER_ENVELOPE_FILL_TRUE_DENSITY_SCORE);
        return true;
    }
    std::string CetClusterFillSearchEngine::_MakeFilledVariantKey(const TetClusterCandidate &ACandidate) const
    {
        std::vector<TetItemTransform> Transforms = ACandidate.Transforms;
        std::sort(Transforms.begin(), Transforms.end(), [](const TetItemTransform &AFirst, const TetItemTransform &ASecond) { return AFirst.OriginalId < ASecond.OriginalId; });
        std::vector<int> Indices = ACandidate.OriginalIndices;
        std::sort(Indices.begin(), Indices.end());
        std::ostringstream Stream;
        for (int Index : Indices)
            Stream << Index << ',';
        Stream << '|';
        for (const TetItemTransform &Transform : Transforms) {
            Stream << Transform.OriginalId << ':' << std::llround(Transform.RelativeX / CET_CLUSTER_FILL_VARIANT_POSITION_TOLERANCE) << ':' << std::llround(Transform.RelativeY / CET_CLUSTER_FILL_VARIANT_POSITION_TOLERANCE) << ':' << std::llround(Transform.RelativeRotation / CET_CLUSTER_FILL_VARIANT_ROTATION_TOLERANCE) << ';';
        }
        return Stream.str();
    }
    void CetClusterFillSearchEngine::_DeduplicateFilledStates(std::vector<TetClusterFillSearchState> &AStates)
    {
        std::map<std::string, TetClusterFillSearchState> UniqueStates;
        for (const TetClusterFillSearchState &State : AStates) {
            const std::string Key = _MakeFilledVariantKey(State.Candidate);
            auto It = UniqueStates.find(Key);
            if (It == UniqueStates.end() || _IsFilledVariantBetter(State, It->second))
                UniqueStates[Key] = State;
        }
        AStates.clear();
        for (const auto &Entry : UniqueStates)
            AStates.push_back(Entry.second);
    }
    void CetClusterFillSearchEngine::_TrimFillBeam(std::vector<TetClusterFillSearchState> &AStates, std::size_t AMaxCount)
    {
        std::stable_sort(AStates.begin(), AStates.end(), [this](const TetClusterFillSearchState &AFirst, const TetClusterFillSearchState &ASecond) { return _IsFilledVariantBetter(AFirst, ASecond); });
        if (AStates.size() > AMaxCount)
            AStates.resize(AMaxCount);
    }
    bool CetClusterFillSearchEngine::_IsEnvelopeStateBetter(const TetClusterFillSearchState &AFirst, const TetClusterFillSearchState &ASecond) const
    {
        const TetClusterCandidate &First = AFirst.Candidate;
        const TetClusterCandidate &Second = ASecond.Candidate;
        // The envelope dimensions are fixed.  Rank states by the material they
        // actually recover from that envelope, so a locally verified set of small
        // gap fillers is not discarded in favour of a higher temporary probe score.
        if (_IsFillMetricLess(First.ProxyWasteArea, Second.ProxyWasteArea))
            return true;
        if (_IsFillMetricLess(Second.ProxyWasteArea, First.ProxyWasteArea))
            return false;
        if (std::abs(First.RealArea - Second.RealArea) > 1.0)
            return First.RealArea > Second.RealArea;
        if (std::abs(Second.RealArea - First.RealArea) > 1.0)
            return false;
        if (First.FillRatio > Second.FillRatio + 1e-9)
            return true;
        if (Second.FillRatio > First.FillRatio + 1e-9)
            return false;
        if (AFirst.FillerCount != ASecond.FillerCount)
            return AFirst.FillerCount > ASecond.FillerCount;
        if (std::abs(First.Score - Second.Score) > 1e-9)
            return First.Score > Second.Score;
        return First.OriginalIndices < Second.OriginalIndices;
    }
    std::uint64_t CetClusterFillSearchEngine::_GetEnvelopeSeedFamilyKey(const TetClusterFillSearchState &AState, const std::vector<TetShapeFeature> &AFeatures, std::size_t ASkeletonChildCount) const
    {
        if (AState.Candidate.Transforms.size() <= ASkeletonChildCount)
            return 0;
        const int OriginalId = AState.Candidate.Transforms[ASkeletonChildCount].OriginalId;
        if (OriginalId < 0 || OriginalId >= static_cast<int>(AFeatures.size()))
            return 0;
        return _MakeFillerFamilyKey(AFeatures[OriginalId]);
    }
    void CetClusterFillSearchEngine::_TrimEnvelopeBeam(std::vector<TetClusterFillSearchState> &AStates, std::size_t AMaxCount, const std::vector<TetShapeFeature> &AFeatures, std::size_t ASkeletonChildCount, bool APreferFillerCount)
    {
        std::stable_sort(AStates.begin(), AStates.end(), [&](const TetClusterFillSearchState &AFirst, const TetClusterFillSearchState &ASecond) {
            if (APreferFillerCount && AFirst.FillerCount != ASecond.FillerCount) {
                return AFirst.FillerCount > ASecond.FillerCount;
            }
            return _IsEnvelopeStateBetter(AFirst, ASecond);
        });
        if (AStates.size() <= AMaxCount)
            return;
        std::vector<TetClusterFillSearchState> Selected;
        std::set<std::uint64_t> SeedFamilies;
        for (const TetClusterFillSearchState &State : AStates) {
            const std::uint64_t Family = _GetEnvelopeSeedFamilyKey(State, AFeatures, ASkeletonChildCount);
            if (Family != 0 && SeedFamilies.insert(Family).second)
                Selected.push_back(State);
        }
        for (const TetClusterFillSearchState &State : AStates) {
            if (Selected.size() >= AMaxCount)
                break;
            if (std::find_if(Selected.begin(), Selected.end(), [&](const TetClusterFillSearchState &Existing) { return _MakeFilledVariantKey(Existing.Candidate) == _MakeFilledVariantKey(State.Candidate); }) == Selected.end())
                Selected.push_back(State);
        }
        if (Selected.size() > AMaxCount)
            Selected.resize(AMaxCount);
        AStates = std::move(Selected);
    }
    std::uint64_t CetClusterFillSearchEngine::MakeFillerFamilyKey(const TetShapeFeature &AFeature)
    {
        std::uint64_t Hash = 1469598103934665603ULL;
        auto Mix = [&](std::uint64_t AValue) {
            Hash ^= AValue;
            Hash *= 1099511628211ULL;
        };
        Mix(static_cast<std::uint64_t>(AFeature.ShapeType));
        Mix(static_cast<std::uint64_t>(AFeature.HoleCount));
        Mix(static_cast<std::uint64_t>(AFeature.NormalizedContour.size()));
        for (const ClipperLib::IntPoint &Point : AFeature.NormalizedContour) {
            Mix(static_cast<std::uint64_t>(Point.X));
            Mix(static_cast<std::uint64_t>(Point.Y));
        }
        return Hash;
    }
    std::uint64_t CetClusterFillSearchEngine::_MakeFillerFamilyKey(const TetShapeFeature &AFeature) const
    {
        return MakeFillerFamilyKey(AFeature);
    }
    void CetClusterFillSearchEngine::_FinalizeEnvelopeFilledStates(const TetClusterFillContext &AContext, std::vector<TetClusterFillSearchState> &AStates, std::vector<TetClusterCandidate> &AOutVariants, TetClusterFillSearchStats &AStats)
    {
        const auto &AOriginalItems = AContext.OriginalItems; const auto &AFeatures = AContext.Features; const auto &AOptions = AContext.Options; const auto &ABaseCandidate = AContext.BaseCandidate;
        _DeduplicateFilledStates(AStates);
        _TrimEnvelopeBeam(AStates, CET_CLUSTER_ENVELOPE_FILL_MAX_TRUE_CONTOUR_STATES, AFeatures, ABaseCandidate.SkeletonChildCount);
        const auto Start = std::chrono::steady_clock::now();
        for (const TetClusterFillSearchState &State : AStates) {
            if (!CetClusterGeometryHelper::PreservesBaseTransforms(ABaseCandidate, State.Candidate))
                continue;
            TetClusterCandidate TrueContourCandidate;
            if (!_RebuildEnvelopeFillWithTrueContour(AOriginalItems, AOptions, ABaseCandidate, State.Candidate, TrueContourCandidate))
                continue;
            AStats.BestEnvelopeFillRatioGain = std::max(AStats.BestEnvelopeFillRatioGain, State.Candidate.FillRatio - ABaseCandidate.BoundingFillRatio);
            AStats.BestEnvelopeRectangleFillRatio = std::max(AStats.BestEnvelopeRectangleFillRatio, State.Candidate.FillRatio);
            AStats.EnvelopeBestFillerCount = std::max(AStats.EnvelopeBestFillerCount, State.FillerCount);
            AOutVariants.push_back(std::move(TrueContourCandidate));
        }
        AStats.EnvelopeTrueContourMs += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - Start).count();
        AStats.EnvelopeDeduplicatedVariantCount += AOutVariants.size();
    }
    std::vector<int> CetClusterFillSearchEngine::_CollectCompatibleFillers(const std::vector<TetShapeFeature> &AFeatures, const TetClusterCandidate &ABaseCandidate, const std::vector<TetClusterFreeRegion> &AFreeRegions, const TetClusterFillSearchConfig &AConfig, bool ADeduplicateFamilies) const
    {
        std::vector<int> Fillers;
        const double AvailableArea = std::max(0.0, ABaseCandidate.ProxyWasteArea);
        const double AreaTolerance = std::max(1.0, ABaseCandidate.ProxyArea * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE);
        for (int Index = 0; Index < static_cast<int>(AFeatures.size()); ++Index) {
            const TetShapeFeature &Feature = AFeatures[Index];
            if (!ContainsOriginalIndex(ABaseCandidate, Index) && std::isfinite(Feature.Area) && Feature.Area > 0.0 && Feature.Area <= AvailableArea + AreaTolerance && CetClusterGeometryHelper::FitsAnyFreeRegion(Feature, AFreeRegions))
                Fillers.push_back(Index);
        }
        std::stable_sort(Fillers.begin(), Fillers.end(), [&](int AFirst, int ASecond) {
            if (std::abs(AFeatures[AFirst].Area - AFeatures[ASecond].Area) > 1.0)
                return AFeatures[AFirst].Area > AFeatures[ASecond].Area;
            if (std::abs(AFeatures[AFirst].OrientedFillRatio - AFeatures[ASecond].OrientedFillRatio) > 1e-9)
                return AFeatures[AFirst].OrientedFillRatio > AFeatures[ASecond].OrientedFillRatio;
            return AFirst < ASecond;
        });
        if (ADeduplicateFamilies) {
            std::set<std::uint64_t> SeenFamilies;
            Fillers.erase(std::remove_if(Fillers.begin(), Fillers.end(), [&](int Index) { return !SeenFamilies.insert(_MakeFillerFamilyKey(AFeatures[Index])).second; }), Fillers.end());
        }
        if (Fillers.size() > AConfig.MaxCandidateFillers) {
            const std::size_t LargestCount = (AConfig.MaxCandidateFillers + 1) / 2;
            const std::size_t SmallestCount = AConfig.MaxCandidateFillers - LargestCount;
            std::vector<int> BoundedFillers;
            BoundedFillers.reserve(AConfig.MaxCandidateFillers);
            BoundedFillers.insert(BoundedFillers.end(), Fillers.begin(), Fillers.begin() + static_cast<std::vector<int>::difference_type>(LargestCount));
            BoundedFillers.insert(BoundedFillers.end(), Fillers.end() - static_cast<std::vector<int>::difference_type>(SmallestCount), Fillers.end());
            Fillers = std::move(BoundedFillers);
        }
        return Fillers;
    }
    void CetClusterFillSearchEngine::BuildFilledVariantsForBase(const TetClusterFillContext &AContext, const TetClusterFillSearchConfig &AConfig, std::vector<TetClusterCandidate> &AOutVariants, TetClusterFillSearchStats &AStats)
    {
        const auto &AOriginalItems = AContext.OriginalItems; 
        const auto &AFeatures = AContext.Features; 
        const auto &AOptions = AContext.Options; 
        const auto &ABaseCandidate = AContext.BaseCandidate;
        AOutVariants.clear();
        if (!ABaseCandidate.Valid || ABaseCandidate.OriginalIndices.size() < 2 || ABaseCandidate.ProxyWasteArea <= 0.0)
            return;
        ET::NEST2DMANAGERLIB::CetRectangleFillClusterBuilder Builder;
        ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
        std::vector<TetClusterFillSearchState> Beam{{ABaseCandidate, 0}};
        std::vector<TetClusterFillSearchState> AllVariants;
        for (std::size_t Depth = 0; Depth < AConfig.MaxDepth && !Beam.empty(); ++Depth) {
            std::vector<TetClusterFillSearchState> NextBeam;
            for (const TetClusterFillSearchState &State : Beam) {
                if (State.FillerCount >= AConfig.MaxDepth)
                    continue;
                std::vector<TetClusterFreeRegion> FreeRegions;
                if (!Geometry.ExtractCandidateFreeRegions(AOriginalItems, AOptions, State.Candidate, FreeRegions) || FreeRegions.empty())
                    continue;
                AStats.FreeRegionCount += FreeRegions.size();
                const std::vector<int> Fillers = _CollectCompatibleFillers(AFeatures, State.Candidate, FreeRegions, AConfig);
                for (int FillerIndex : Fillers) {
                    if (AConfig.MaxPlacementAttempts > 0 && AStats.SearchAttempts >= AConfig.MaxPlacementAttempts)
                        break;
                    if (ContainsOriginalIndex(State.Candidate, FillerIndex))
                        continue;
                    ++AStats.SearchAttempts;
                   /* TetClusterCandidate Candidate;
                    if (!Builder.TryAppendFillerInFreeRegions({AOriginalItems, AFeatures, AOptions, ABaseCandidate, ABaseCandidate, State.Candidate, &FreeRegions, FillerIndex, ABaseCandidate.ClusterWidth, ABaseCandidate.ClusterHeight}, Candidate))
                        continue;
                    if (!_IsFilledVariantWorthKeeping(ABaseCandidate, Candidate))
                        continue;
                    NextBeam.push_back({std::move(Candidate), State.FillerCount + 1});
                    ++AStats.GeneratedVariantCount;*/
                    std::vector<TetClusterCandidate> PlacementCandidates;
                    Builder.BuildFillerVariantsInFreeRegions({AOriginalItems, AFeatures, AOptions, ABaseCandidate, ABaseCandidate, State.Candidate, &FreeRegions, FillerIndex, ABaseCandidate.ClusterWidth, ABaseCandidate.ClusterHeight}, CET_CLUSTER_FILL_TOP_PLACEMENTS_PER_FILLER, PlacementCandidates);
                    for (TetClusterCandidate &Candidate : PlacementCandidates) {
                        if (!_IsFilledVariantWorthKeeping(ABaseCandidate, Candidate)) {
                            continue;
                        }
                        NextBeam.push_back({std::move(Candidate), State.FillerCount + 1});
                        ++AStats.GeneratedVariantCount;
                    }
                }
            }
            if (AConfig.MaxPlacementAttempts > 0 && AStats.SearchAttempts >= AConfig.MaxPlacementAttempts)
                break;
            _DeduplicateFilledStates(NextBeam);
            _TrimFillBeam(NextBeam, AConfig.BeamWidth);
            AllVariants.insert(AllVariants.end(), NextBeam.begin(), NextBeam.end());
            Beam = std::move(NextBeam);
        }
        _DeduplicateFilledStates(AllVariants);
        _TrimFillBeam(AllVariants, CET_CLUSTER_FILL_MAX_VARIANTS_PER_BASE);
        AStats.DeduplicatedVariantCount += AllVariants.size();
        for (const TetClusterFillSearchState &State : AllVariants) {
            AStats.FilledFillRatioSum += State.Candidate.FillRatio;
            AStats.BestFillRatioGain = std::max(AStats.BestFillRatioGain, State.Candidate.FillRatio - ABaseCandidate.FillRatio);
            AOutVariants.push_back(State.Candidate);
        }
    }
    TetClusterFillSearchState CetClusterFillSearchEngine::_BuildEnvelopeFillSeed(const TetClusterFillContext &AContext, std::map<std::string, TetCircleGapTemplate> &AGapTemplates, const TetClusterFillSearchState *ASeedState, bool AIsCircleBase, bool AIsEllipseBase, const TetClusterFillSearchConfig &AConfig, std::vector<TetClusterFillSearchState> &AOutStates)
    {
        const auto &AOriginalItems = AContext.OriginalItems; const auto &AFeatures = AContext.Features; const auto &AOptions = AContext.Options; const auto &ABaseCandidate = AContext.BaseCandidate; const auto &AEnvelopeCandidate = AContext.EnvelopeCandidate;
        AOutStates.clear();
        if (ASeedState != nullptr && CetClusterGeometryHelper::PreservesBaseTransforms(ABaseCandidate, ASeedState->Candidate) && _IsEnvelopeFillStateWorthExpanding(AEnvelopeCandidate, ASeedState->Candidate)) {
            AOutStates.push_back(*ASeedState);
        }
        TetClusterCandidate LocalGap;
        const TetClusterFillContext FillContext = AContext;
        if (AIsCircleBase && ET::NEST2DMANAGERLIB::_GetNest2DInvokeFunctor()->Nest2dCircleGapFiller->BuildCircleGapCandidate(FillContext, AGapTemplates, LocalGap) && CetClusterGeometryHelper::PreservesBaseTransforms(ABaseCandidate, LocalGap) && _IsEnvelopeFillStateWorthExpanding(AEnvelopeCandidate, LocalGap)) {
            AOutStates.push_back({LocalGap, LocalGap.Transforms.size() - ABaseCandidate.Transforms.size()});
            std::cout << "[TEMPLATE][LOCAL GAP] Fillers=" << AOutStates.back().FillerCount << std::endl;
        }
        TetClusterCandidate EllipseGap;
        if (AIsEllipseBase && ET::NEST2DMANAGERLIB::_GetNest2DInvokeFunctor()->Nest2dEllipseGapFiller->BuildEllipseGapCandidate(FillContext, AConfig, EllipseGap) && CetClusterGeometryHelper::PreservesBaseTransforms(ABaseCandidate, EllipseGap) && _IsEnvelopeFillStateWorthExpanding(AEnvelopeCandidate, EllipseGap)) {
            const std::size_t FillerCount = EllipseGap.Transforms.size() > ABaseCandidate.Transforms.size() ? EllipseGap.Transforms.size() - ABaseCandidate.Transforms.size() : 0;
            AOutStates.push_back({std::move(EllipseGap), FillerCount});
            std::cout << "[TEMPLATE][ELLIPSE LOCAL GAP] Fillers=" << AOutStates.back().FillerCount << std::endl;
        }
        const TetClusterCandidate *FreeRegionSeed = &AEnvelopeCandidate;
        if (!AOutStates.empty()) {
            FreeRegionSeed = &std::max_element(AOutStates.begin(), AOutStates.end(), [this](const TetClusterFillSearchState &AFirst, const TetClusterFillSearchState &ASecond) { return _IsEnvelopeStateBetter(ASecond, AFirst); })->Candidate;
        }
        TetClusterCandidate FreeRegionGap;
        if (ABaseCandidate.SkeletonChildCount >= 3 && ET::NEST2DMANAGERLIB::_GetNest2DInvokeFunctor()->Nest2dCircleGapFiller->BuildFreeRegionTemplateCandidate(FillContext, *FreeRegionSeed, FreeRegionGap) && CetClusterGeometryHelper::PreservesBaseTransforms(ABaseCandidate, FreeRegionGap) && _IsEnvelopeFillStateWorthExpanding(AEnvelopeCandidate, FreeRegionGap)) {
            const std::size_t FillerCount = FreeRegionGap.Transforms.size() > ABaseCandidate.Transforms.size() ? FreeRegionGap.Transforms.size() - ABaseCandidate.Transforms.size() : 0;
            AOutStates.push_back({std::move(FreeRegionGap), FillerCount});
            std::cout << "[TEMPLATE][FREE REGION GAP] Fillers=" << AOutStates.back().FillerCount << std::endl;
        }
        TetClusterFillSearchState Initial{AEnvelopeCandidate, 0};
        if (!AOutStates.empty()) {
            Initial = *std::max_element(AOutStates.begin(), AOutStates.end(), [this](const TetClusterFillSearchState &AFirst, const TetClusterFillSearchState &ASecond) { return _IsEnvelopeStateBetter(ASecond, AFirst); });
        }
        std::cout << "[TEMPLATE][ELLIPSE SEARCH SEED] IsEllipse=" << AIsEllipseBase << ", States=" << AOutStates.size() << ", Fillers=" << Initial.FillerCount << std::endl;
        return Initial;
    }
    void CetClusterFillSearchEngine::_SearchEnvelopeFillVariants(const TetClusterFillContext &AContext, const TetClusterFillSearchConfig &AConfig, std::map<std::string, TetCircleGapTemplate> &AGapTemplates, const TetClusterFillSearchState *ASeedState, std::vector<TetClusterFillSearchState> &AOutStates, TetClusterFillSearchStats &AStats)
    {
        const auto &AOriginalItems = AContext.OriginalItems; const auto &AFeatures = AContext.Features; const auto &AOptions = AContext.Options; const auto &ABaseCandidate = AContext.BaseCandidate; const auto &AEnvelopeCandidate = AContext.EnvelopeCandidate;
        std::size_t LocalAttempts = 0;
        const bool IsCircleBase = IsFixedCircleEnvelopeBase(ABaseCandidate, AFeatures);
        const std::vector<TetCircleGapTemplateAnchor> Anchors = IsCircleBase ? ET::NEST2DMANAGERLIB::_GetNest2DInvokeFunctor()->Nest2dCircleGapFiller->CollectCircleGapTemplateAnchors(AOriginalItems, AFeatures, ABaseCandidate) : std::vector<TetCircleGapTemplateAnchor>{};
        ET::NEST2DMANAGERLIB::CetRectangleFillClusterBuilder Builder;
        ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
        AOutStates.clear();
        const bool IsEllipseBase = IsFixedEllipseEnvelopeBase(ABaseCandidate, AFeatures);
        const TetClusterFillSearchState Initial = _BuildEnvelopeFillSeed(AContext, AGapTemplates, ASeedState, IsCircleBase, IsEllipseBase, AConfig, AOutStates);
        const auto SearchStart = std::chrono::steady_clock::now();
        auto TimeLimitReached = [&]() {
            const long long Limit = IsEllipseBase && AConfig.MaxElapsedMs > 0 ? std::min(AConfig.MaxElapsedMs, CET_ELLIPSE_GAP_FILL_GENERIC_TIME_MS) : AConfig.MaxElapsedMs;
            return CircleGapSearchTimeReached(SearchStart, Limit);
        };
        // Keep every distinct seed long enough for the fixed-envelope search to
        // compare a locally dense layout with a less dense layout that may leave a
        // better-shaped pocket for the remaining inventory.
        std::vector<TetClusterFillSearchState> Beam = AOutStates;
        Beam.push_back(Initial);
        _DeduplicateFilledStates(Beam);
        _TrimEnvelopeBeam(Beam, AConfig.BeamWidth, AFeatures, ABaseCandidate.SkeletonChildCount, IsEllipseBase);
        const std::size_t MaxFillers = ABaseCandidate.Transforms.size() + AConfig.MaxDepth + CET_ELLIPSE_GAP_FILL_MAX_COMPOSITE_DEPTH;
        AOutStates.insert(AOutStates.end(), Beam.begin(), Beam.end());
        for (std::size_t Depth = 0; Depth < AConfig.MaxDepth && !Beam.empty(); ++Depth) {
            if (TimeLimitReached()) {
                ++AStats.EnvelopeTimeLimitHits;
                break;
            }
            std::vector<TetClusterFillSearchState> Next;
            for (const TetClusterFillSearchState &State : Beam) {
                if (State.FillerCount >= MaxFillers)
                    continue;
                std::vector<TetClusterFreeRegion> StateFreeRegions;
                if (!Geometry.ExtractCandidateFreeRegions(AOriginalItems, AOptions, State.Candidate, StateFreeRegions) || StateFreeRegions.empty())
                    continue;
                AStats.EnvelopeFreeRegionCount += StateFreeRegions.size();
                const std::vector<int> Fillers = _CollectCompatibleFillers(AFeatures, State.Candidate, StateFreeRegions, AConfig, true);
                for (int Filler : Fillers) {
                    if ((AConfig.MaxPlacementAttempts > 0 && LocalAttempts >= AConfig.MaxPlacementAttempts) || TimeLimitReached())
                        break;
                    if (ContainsOriginalIndex(State.Candidate, Filler))
                        continue;
                    ++LocalAttempts;
                    ++AStats.EnvelopeSearchAttempts;
                    TetClusterCandidate Candidate;
                    if (!Builder.TryAppendFillerInRectangleEnvelope({AOriginalItems, AFeatures, AOptions, ABaseCandidate, AEnvelopeCandidate, State.Candidate, &StateFreeRegions, Filler, AEnvelopeCandidate.ClusterWidth, AEnvelopeCandidate.ClusterHeight}, Candidate))
                        continue;
                    std::size_t Copies = 0;
                    TetClusterCandidate Copied;
                    const std::size_t Remaining = MaxFillers > State.FillerCount + 1 ? MaxFillers - State.FillerCount - 1 : 0;
                    const TetClusterFillContext FillContext = AContext;
                    if (IsCircleBase && ET::NEST2DMANAGERLIB::_GetNest2DInvokeFunctor()->Nest2dCircleGapFiller->CopyCircleGapTemplate(FillContext, Anchors, Candidate, Remaining, Copied, Copies))
                        Candidate = std::move(Copied);
                    if (!CetClusterGeometryHelper::PreservesBaseTransforms(ABaseCandidate, Candidate) || !_IsEnvelopeFillStateWorthExpanding(AEnvelopeCandidate, Candidate))
                        continue;
                    Next.push_back({std::move(Candidate), State.FillerCount + 1 + Copies});
                    ++AStats.EnvelopeGeneratedVariantCount;
                }
            }
            _DeduplicateFilledStates(Next);
            _TrimEnvelopeBeam(Next, AConfig.BeamWidth, AFeatures, ABaseCandidate.SkeletonChildCount, IsEllipseBase);
            AOutStates.insert(AOutStates.end(), Next.begin(), Next.end());
            Beam = std::move(Next);
            AStats.EnvelopeMaxDepthReached = std::max(AStats.EnvelopeMaxDepthReached, Depth + 1);
            if (AConfig.MaxPlacementAttempts > 0 && LocalAttempts >= AConfig.MaxPlacementAttempts || TimeLimitReached()) {
                if (TimeLimitReached())
                    ++AStats.EnvelopeTimeLimitHits;
                break;
            }
        }
        AStats.EnvelopeSearchMs += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - SearchStart).count();
    }
    bool CetClusterFillSearchEngine::_TryBuildCachedEllipseTemplateSeed(const TetClusterFillContext &AContext, const TetEllipseGapTemplateCache &ATemplates, TetClusterFillSearchState &AOutSeed)
    {
        const auto &ABaseCandidate = AContext.BaseCandidate;
        AOutSeed = TetClusterFillSearchState{};
        TetClusterCandidate Candidate;
        if (!ET::NEST2DMANAGERLIB::_GetNest2DInvokeFunctor()->Nest2dEllipseGapFiller->TryBuildCachedEllipseTemplateVariant(AContext, ATemplates, Candidate) || !CetClusterGeometryHelper::PreservesBaseTransforms(ABaseCandidate, Candidate) || !_IsEnvelopeFillStateWorthExpanding(AContext.EnvelopeCandidate, Candidate))
            return false;
        AOutSeed.Candidate = std::move(Candidate);
        AOutSeed.FillerCount = AOutSeed.Candidate.Transforms.size() - ABaseCandidate.Transforms.size();
        return true;
    }
    void CetClusterFillSearchEngine::BuildEnvelopeFilledVariantsForBase(const TetEnvelopeFillVariantRequest &ARequest)
    {
        const auto &AOriginalItems = ARequest.OriginalItems; const auto &AFeatures = ARequest.Features; const auto &AOptions = ARequest.Options; const auto &ABaseCandidate = ARequest.BaseCandidate; const auto &AConfig = ARequest.Config; auto &AGapTemplates = ARequest.GapTemplates; auto &AEllipseTemplates = ARequest.EllipseTemplates; auto &AOutVariants = ARequest.OutVariants; auto &AStats = ARequest.Stats;
        AOutVariants.clear();
        TetClusterCandidate EnvelopeCandidate = ABaseCandidate;
        if (!CetClusterGeometryHelper::HasFullRectangleProxy(ABaseCandidate) && !BuildRectangleEnvelopeCandidate(AOriginalItems, AOptions, ABaseCandidate, EnvelopeCandidate))
            return;
        const bool IsEllipseBase = IsFixedEllipseEnvelopeBase(ABaseCandidate, AFeatures);
        TetClusterFillSearchState CachedSeed;
        const TetClusterFillContext FillContext{AOriginalItems, AFeatures, AOptions, ABaseCandidate, EnvelopeCandidate};
        if (IsEllipseBase && _TryBuildCachedEllipseTemplateSeed(FillContext, AEllipseTemplates, CachedSeed)) {
            std::cout << "[TEMPLATE][ELLIPSE GAP CACHE SEED] Fillers=" << CachedSeed.FillerCount << std::endl;
        }
        std::vector<TetClusterFillSearchState> States;
        _SearchEnvelopeFillVariants(FillContext, AConfig, AGapTemplates, CachedSeed.Candidate.Valid ? &CachedSeed : nullptr, States, AStats);
        _FinalizeEnvelopeFilledStates({AOriginalItems, AFeatures, AOptions, ABaseCandidate, EnvelopeCandidate}, States, AOutVariants, AStats);
        if (IsEllipseBase)
            ET::NEST2DMANAGERLIB::_GetNest2DInvokeFunctor()->Nest2dEllipseGapFiller->CacheEllipseTemplateVariant(FillContext, States, !AOutVariants.empty(), AEllipseTemplates);
    }

}}
