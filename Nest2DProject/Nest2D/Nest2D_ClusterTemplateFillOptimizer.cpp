#include "pch.h"
#include "Nest2D_ClusterTemplateFillOptimizer.h"
#include "Nest2D_CircleGapFiller.h"
#include "Nest2D_ArcClusterBuilder.h"
#include "Nest2D_CircleClusterBuilder.h"
#include "Nest2D_ClusterBoundary.h"
#include "Nest2D_ClusterGeometryHelper.h"
#include "Nest2D_CustomClusterBuilder.h"
#include "Nest2D_EllipseClusterBuilder.h"
#include "Nest2D_RectangleClusterBuilder.h"
#include "Nest2D_RectangleFillClusterBuilder.h"
#include "Nest2D_RotationUtils.h"
#include "Nest2D_SelfFunction.h"
#include "Nest2D_TriangleClusterBuilder.h"
#include "Nest2D_EllipseGapFiller.h"
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
#include <unordered_map>
#include <utility>

using namespace ClipperLib;
using namespace libnest2d;

namespace ET { namespace NEST2DMANAGERLIB {
    CetNest2DInvokeFunctor *_GetNest2DInvokeFunctor()
    {
        return Nest2DUtils;
    }
}}

namespace {
    bool CircleGapSearchTimeReached(const std::chrono::steady_clock::time_point &AStart, long long ALimitMs)
    {
        return ALimitMs > 0 && std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - AStart).count() >= ALimitMs;
    }
    TetClusterFillSearchConfig GetClusterFillSearchConfig(std::size_t AItemCount)
    {
        if (AItemCount > CET_NEST_REDUCED_STRATEGY_ITEM_LIMIT) {
            return {CET_CLUSTER_FILL_REDUCED_ORDER_BEAM_WIDTH, std::max<std::size_t>(1, AItemCount), std::max<std::size_t>(1, AItemCount), 0};
        }
        if (AItemCount > CET_NEST_FULL_STRATEGY_ITEM_LIMIT) {
            return {CET_CLUSTER_FILL_REDUCED_ORDER_BEAM_WIDTH, std::max<std::size_t>(1, AItemCount), std::max<std::size_t>(1, AItemCount), 0};
        }
        return {};
    }
    TetClusterFillSearchConfig GetClusterEnvelopeFillSearchConfig(std::size_t AItemCount)
    {
        const std::size_t InventoryDepth = std::max<std::size_t>(1, AItemCount);
        if (AItemCount > CET_NEST_FULL_STRATEGY_ITEM_LIMIT) {
            return {CET_CLUSTER_ENVELOPE_FILL_LARGE_ORDER_BEAM_WIDTH, std::min(InventoryDepth, CET_CLUSTER_ENVELOPE_FILL_LARGE_ORDER_MAX_DEPTH), CET_CLUSTER_ENVELOPE_FILL_LARGE_ORDER_MAX_CANDIDATE_FILLERS, CET_CLUSTER_ENVELOPE_FILL_LARGE_ORDER_MAX_PLACEMENT_ATTEMPTS, CET_CLUSTER_ENVELOPE_FILL_LARGE_ORDER_MIN_DEPTH_BEFORE_TIMEOUT, CET_CLUSTER_ENVELOPE_FILL_LARGE_ORDER_MAX_SEARCH_TIME_MS};
        }
        return {CET_CLUSTER_ENVELOPE_FILL_BEAM_WIDTH, std::min(InventoryDepth, CET_CLUSTER_ENVELOPE_FILL_MAX_DEPTH), CET_CLUSTER_ENVELOPE_FILL_MAX_CANDIDATE_FILLERS, CET_CLUSTER_ENVELOPE_FILL_MAX_PLACEMENT_ATTEMPTS, CET_CLUSTER_ENVELOPE_FILL_MIN_DEPTH_BEFORE_TIMEOUT, CET_CLUSTER_ENVELOPE_FILL_MAX_SEARCH_TIME_MS};
    }
    bool ContainsOriginalIndex(const TetClusterCandidate &ACandidate, int AOriginalIndex) { return std::find(ACandidate.OriginalIndices.begin(), ACandidate.OriginalIndices.end(), AOriginalIndex) != ACandidate.OriginalIndices.end(); }
    bool IsFillMetricLess(double ALeft, double ARight) { return ALeft < ARight - CET_CLUSTER_FILL_VARIANT_POSITION_TOLERANCE; }
    bool IsFilledVariantBetter(const TetClusterFillSearchState &AFirst, const TetClusterFillSearchState &ASecond)
    {
        const TetClusterCandidate &First = AFirst.Candidate;
        const TetClusterCandidate &Second = ASecond.Candidate;
        if (IsFillMetricLess(First.ProxyWasteArea, Second.ProxyWasteArea))
            return true;
        if (IsFillMetricLess(Second.ProxyWasteArea, First.ProxyWasteArea))
            return false;
        if (First.FillRatio > Second.FillRatio + 1e-9)
            return true;
        if (Second.FillRatio > First.FillRatio + 1e-9)
            return false;
        if (First.ProxyWasteRatio < Second.ProxyWasteRatio - 1e-9)
            return true;
        if (Second.ProxyWasteRatio < First.ProxyWasteRatio - 1e-9)
            return false;
        if (IsFillMetricLess(First.ProxyArea, Second.ProxyArea))
            return true;
        if (IsFillMetricLess(Second.ProxyArea, First.ProxyArea))
            return false;
        if (First.FragmentationRisk < Second.FragmentationRisk - 1e-9)
            return true;
        if (Second.FragmentationRisk < First.FragmentationRisk - 1e-9)
            return false;
        if (AFirst.FillerCount != ASecond.FillerCount)
            return AFirst.FillerCount < ASecond.FillerCount;
        return First.OriginalIndices < Second.OriginalIndices;
    }
    bool IsFilledVariantWorthKeeping(const TetClusterCandidate &ABaseCandidate, const TetClusterCandidate &ACandidate)
    {
        const double AreaTolerance = std::max(1.0, ABaseCandidate.ProxyArea * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE);
        return ACandidate.Valid && ACandidate.ProxyWasteArea < ABaseCandidate.ProxyWasteArea - AreaTolerance && ACandidate.FillRatio > ABaseCandidate.FillRatio + 1e-9;
    }
    bool HasFullRectangleProxy(const TetClusterCandidate &ACandidate)
    {
        const double BoundingArea = ACandidate.ClusterWidth * ACandidate.ClusterHeight;
        const double AreaTolerance = std::max(1.0, BoundingArea * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE);
        return ACandidate.Valid && ACandidate.ClusterWidth > 0.0 && ACandidate.ClusterHeight > 0.0 && std::isfinite(BoundingArea) && std::abs(ACandidate.ProxyArea - BoundingArea) <= AreaTolerance;
    }
    bool SkipsGenericTemplateFill(const TetClusterCandidate &ACandidate) { return ACandidate.BuilderName == "CircleBuilder" || ACandidate.BuilderName == "EllipseBuilder" || ACandidate.BuilderName == "ArcBuilder"; }
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
    bool IsCompletedEnvelopeFill(const TetClusterCandidate &ACandidate) { return ACandidate.BuilderName == "EnvelopeFillSearch" && ACandidate.ProxyMode != MetClusterProxyMode::Unknown && ACandidate.OriginalIndices.size() >= 3; }
    bool IsCircleSkeletonCandidate(const TetClusterCandidate &ACandidate) { return ACandidate.SkeletonChildCount >= 2 && (ACandidate.BuilderName == "CircleBuilder" || ACandidate.ClusterType.find("Circle") == 0); }
    bool IsEllipseSkeletonCandidate(const TetClusterCandidate &ACandidate) { return ACandidate.SkeletonChildCount >= 2 && (ACandidate.BuilderName == "EllipseBuilder" || ACandidate.ClusterType.find("Ellipse") == 0); }
    bool IsInventorySkeletonCandidate(const TetClusterCandidate &ACandidate) { return IsCircleSkeletonCandidate(ACandidate) || IsEllipseSkeletonCandidate(ACandidate); }
    bool IsCompletedEnvelopeFillBetter(const TetClusterCandidate &AFirst, const TetClusterCandidate &ASecond)
    {
        // A circle framework is the virtual board for its fillers.  Keep the
        // largest complete framework intact before comparing local filler gains;
        // otherwise an 8-circle frame with one extra small part can consume the
        // inventory ahead of a 12-circle frame and fragment the intended layout.
        if (AFirst.SkeletonChildCount != ASecond.SkeletonChildCount) {
            return AFirst.SkeletonChildCount > ASecond.SkeletonChildCount;
        }
        const std::size_t FirstFillers = AFirst.OriginalIndices.size() - AFirst.SkeletonChildCount;
        const std::size_t SecondFillers = ASecond.OriginalIndices.size() - ASecond.SkeletonChildCount;
        if (FirstFillers != SecondFillers)
            return FirstFillers > SecondFillers;
        if (std::abs(AFirst.FillRatio - ASecond.FillRatio) > 1e-9) {
            return AFirst.FillRatio > ASecond.FillRatio;
        }
        if (std::abs(AFirst.FragmentationRisk - ASecond.FragmentationRisk) > 1e-9) {
            return AFirst.FragmentationRisk < ASecond.FragmentationRisk;
        }
        if (std::abs(AFirst.SheetReuseScore - ASecond.SheetReuseScore) > 1e-9) {
            return AFirst.SheetReuseScore > ASecond.SheetReuseScore;
        }
        if (std::abs(AFirst.Score - ASecond.Score) > 1e-9)
            return AFirst.Score > ASecond.Score;
        return AFirst.ProxyArea < ASecond.ProxyArea;
    }
    std::string BuildCircleGapTemplateCacheKey(const TetClusterCandidate &ABaseCandidate)
    {
        std::ostringstream Stream;
        Stream << ABaseCandidate.BuilderName << '|' << ABaseCandidate.ClusterType << '|' << ABaseCandidate.SkeletonChildCount << '|' << std::llround(ABaseCandidate.ClusterWidth / CET_RECTANGLE_FILL_POSITION_TOLERANCE) << '|' << std::llround(ABaseCandidate.ClusterHeight / CET_RECTANGLE_FILL_POSITION_TOLERANCE);
        return Stream.str();
    }
    bool BuildRectangleEnvelopeCandidate(const CetTNestItemVector &AOriginalItems, const TetNestOptions &AOptions, const TetClusterCandidate &ABaseCandidate, TetClusterCandidate &AOutEnvelopeCandidate)
    {
        AOutEnvelopeCandidate = TetClusterCandidate{};
        if (!ABaseCandidate.Valid || ABaseCandidate.OriginalIndices.size() < 2 || HasFullRectangleProxy(ABaseCandidate)) {
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
    bool IsEnvelopeFillStateWorthExpanding(const TetClusterCandidate &AEnvelopeCandidate, const TetClusterCandidate &ACandidate)
    {
        const double EnvelopeTolerance = std::max(1.0, AEnvelopeCandidate.ProxyArea * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE);
        return ACandidate.Valid && std::abs(ACandidate.ProxyArea - AEnvelopeCandidate.ProxyArea) <= EnvelopeTolerance && (ACandidate.Transforms.size() > AEnvelopeCandidate.Transforms.size() || ACandidate.ProxyWasteArea < AEnvelopeCandidate.ProxyWasteArea - EnvelopeTolerance);
    }
    bool PreservesBaseTransforms(const TetClusterCandidate &ABaseCandidate, const TetClusterCandidate &ACandidate)
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
    bool RebuildEnvelopeFillWithTrueContour(const CetTNestItemVector &AOriginalItems, const TetNestOptions &AOptions, const TetClusterCandidate &ABaseCandidate, const TetClusterCandidate &AEnvelopeFilledCandidate, TetClusterCandidate &AOutCandidate)
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
    std::string MakeFilledVariantKey(const TetClusterCandidate &ACandidate)
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
    void DeduplicateFilledStates(std::vector<TetClusterFillSearchState> &AStates)
    {
        std::map<std::string, TetClusterFillSearchState> UniqueStates;
        for (const TetClusterFillSearchState &State : AStates) {
            const std::string Key = MakeFilledVariantKey(State.Candidate);
            auto It = UniqueStates.find(Key);
            if (It == UniqueStates.end() || IsFilledVariantBetter(State, It->second))
                UniqueStates[Key] = State;
        }
        AStates.clear();
        for (const auto &Entry : UniqueStates)
            AStates.push_back(Entry.second);
    }
    void TrimFillBeam(std::vector<TetClusterFillSearchState> &AStates, std::size_t AMaxCount)
    {
        std::stable_sort(AStates.begin(), AStates.end(), IsFilledVariantBetter);
        if (AStates.size() > AMaxCount)
            AStates.resize(AMaxCount);
    }
    bool IsEnvelopeStateBetter(const TetClusterFillSearchState &AFirst, const TetClusterFillSearchState &ASecond)
    {
        const TetClusterCandidate &First = AFirst.Candidate;
        const TetClusterCandidate &Second = ASecond.Candidate;
        // The envelope dimensions are fixed.  Rank states by the material they
        // actually recover from that envelope, so a locally verified set of small
        // gap fillers is not discarded in favour of a higher temporary probe score.
        if (IsFillMetricLess(First.ProxyWasteArea, Second.ProxyWasteArea))
            return true;
        if (IsFillMetricLess(Second.ProxyWasteArea, First.ProxyWasteArea))
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
    std::uint64_t MakeFillerFamilyKey(const TetShapeFeature &AFeature);
    std::uint64_t GetEnvelopeSeedFamilyKey(const TetClusterFillSearchState &AState, const std::vector<TetShapeFeature> &AFeatures, std::size_t ASkeletonChildCount)
    {
        if (AState.Candidate.Transforms.size() <= ASkeletonChildCount)
            return 0;
        const int OriginalId = AState.Candidate.Transforms[ASkeletonChildCount].OriginalId;
        if (OriginalId < 0 || OriginalId >= static_cast<int>(AFeatures.size()))
            return 0;
        return MakeFillerFamilyKey(AFeatures[OriginalId]);
    }
    void TrimEnvelopeBeam(std::vector<TetClusterFillSearchState> &AStates, std::size_t AMaxCount, const std::vector<TetShapeFeature> &AFeatures, std::size_t ASkeletonChildCount, bool APreferFillerCount = false)
    {
        std::stable_sort(AStates.begin(), AStates.end(), [&](const TetClusterFillSearchState &AFirst, const TetClusterFillSearchState &ASecond) {
            if (APreferFillerCount && AFirst.FillerCount != ASecond.FillerCount) {
                return AFirst.FillerCount > ASecond.FillerCount;
            }
            return IsEnvelopeStateBetter(AFirst, ASecond);
        });
        if (AStates.size() <= AMaxCount)
            return;
        std::vector<TetClusterFillSearchState> Selected;
        std::set<std::uint64_t> SeedFamilies;
        for (const TetClusterFillSearchState &State : AStates) {
            const std::uint64_t Family = GetEnvelopeSeedFamilyKey(State, AFeatures, ASkeletonChildCount);
            if (Family != 0 && SeedFamilies.insert(Family).second)
                Selected.push_back(State);
        }
        for (const TetClusterFillSearchState &State : AStates) {
            if (Selected.size() >= AMaxCount)
                break;
            if (std::find_if(Selected.begin(), Selected.end(), [&](const TetClusterFillSearchState &Existing) { return MakeFilledVariantKey(Existing.Candidate) == MakeFilledVariantKey(State.Candidate); }) == Selected.end())
                Selected.push_back(State);
        }
        if (Selected.size() > AMaxCount)
            Selected.resize(AMaxCount);
        AStates = std::move(Selected);
    }
    std::uint64_t MakeFillerFamilyKey(const TetShapeFeature &AFeature)
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
    void FinalizeEnvelopeFilledStates(const TetClusterFillContext &AContext, std::vector<TetClusterFillSearchState> &AStates, std::vector<TetClusterCandidate> &AOutVariants, TetClusterFillSearchStats &AStats)
    {
        const auto &AOriginalItems = AContext.OriginalItems; const auto &AFeatures = AContext.Features; const auto &AOptions = AContext.Options; const auto &ABaseCandidate = AContext.BaseCandidate;
        DeduplicateFilledStates(AStates);
        TrimEnvelopeBeam(AStates, CET_CLUSTER_ENVELOPE_FILL_MAX_TRUE_CONTOUR_STATES, AFeatures, ABaseCandidate.SkeletonChildCount);
        const auto Start = std::chrono::steady_clock::now();
        for (const TetClusterFillSearchState &State : AStates) {
            if (!PreservesBaseTransforms(ABaseCandidate, State.Candidate))
                continue;
            TetClusterCandidate TrueContourCandidate;
            if (!RebuildEnvelopeFillWithTrueContour(AOriginalItems, AOptions, ABaseCandidate, State.Candidate, TrueContourCandidate))
                continue;
            AStats.BestEnvelopeFillRatioGain = std::max(AStats.BestEnvelopeFillRatioGain, State.Candidate.FillRatio - ABaseCandidate.BoundingFillRatio);
            AStats.BestEnvelopeRectangleFillRatio = std::max(AStats.BestEnvelopeRectangleFillRatio, State.Candidate.FillRatio);
            AStats.EnvelopeBestFillerCount = std::max(AStats.EnvelopeBestFillerCount, State.FillerCount);
            AOutVariants.push_back(std::move(TrueContourCandidate));
        }
        AStats.EnvelopeTrueContourMs += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - Start).count();
        AStats.EnvelopeDeduplicatedVariantCount += AOutVariants.size();
    }
    bool HasValidCandidateInventory(const TetClusterCandidate &ACandidate, const std::vector<TetShapeFeature> &AFeatures, const std::vector<bool> &AUsed) { return ACandidate.SkeletonChildCount <= ACandidate.Transforms.size() && AFeatures.size() == AUsed.size() && ACandidate.OriginalIndices.size() == ACandidate.Transforms.size(); }
    int FindAvailableFamilyItem(const std::vector<TetShapeFeature> &AFeatures, const std::vector<bool> &AUsed, const std::set<int> &AReserved, int APrototypeId)
    {
        if (APrototypeId < 0 || APrototypeId >= static_cast<int>(AFeatures.size()))
            return -1;
        const std::uint64_t FamilyKey = MakeFillerFamilyKey(AFeatures[APrototypeId]);
        for (int Index = 0; Index < static_cast<int>(AFeatures.size()); ++Index) {
            if (!AUsed[Index] && AReserved.find(Index) == AReserved.end() && MakeFillerFamilyKey(AFeatures[Index]) == FamilyKey)
                return Index;
        }
        return -1;
    }
    bool TryBindCandidateInventory(const TetClusterCandidate &ACandidate, const std::vector<TetShapeFeature> &AFeatures, const std::vector<bool> &AUsed, TetClusterCandidate &AOutCandidate)
    {
        AOutCandidate = ACandidate;
        if (!HasValidCandidateInventory(ACandidate, AFeatures, AUsed))
            return false;
        if (ACandidate.SkeletonChildCount == 0)
            return true;
        std::set<int> Reserved;
        for (std::size_t Index = 0; Index < ACandidate.SkeletonChildCount; ++Index) {
            const int OriginalId = ACandidate.Transforms[Index].OriginalId;
            if (OriginalId < 0 || OriginalId >= static_cast<int>(AUsed.size()) || AUsed[OriginalId] || !Reserved.insert(OriginalId).second)
                return false;
        }
        for (std::size_t Index = ACandidate.SkeletonChildCount; Index < ACandidate.Transforms.size(); ++Index) {
            const int PrototypeId = ACandidate.Transforms[Index].OriginalId;
            const int BoundId = FindAvailableFamilyItem(AFeatures, AUsed, Reserved, PrototypeId);
            if (BoundId < 0)
                return false;
            AOutCandidate.Transforms[Index].OriginalId = BoundId;
            Reserved.insert(BoundId);
        }
        AOutCandidate.OriginalIndices.clear();
        AOutCandidate.OriginalIndices.reserve(AOutCandidate.Transforms.size());
        for (const TetItemTransform &Transform : AOutCandidate.Transforms) {
            AOutCandidate.OriginalIndices.push_back(Transform.OriginalId);
        }
        return true;
    }
    bool FitsAnyFreeRegion(const TetShapeFeature &AFeature, const std::vector<TetClusterFreeRegion> &AFreeRegions)
    {
        for (const TetClusterFreeRegion &Region : AFreeRegions) {
            const bool FitsNormal = AFeature.Width <= Region.Width && AFeature.Height <= Region.Height;
            const bool FitsRotated = AFeature.Height <= Region.Width && AFeature.Width <= Region.Height;
            if (AFeature.Area <= Region.Area && (FitsNormal || FitsRotated))
                return true;
        }
        return false;
    }
    bool IsInventoryRebalanceCandidate(const TetClusterCandidate &ACandidate)
    {
        return ACandidate.Valid &&
               ACandidate.SkeletonChildCount >= 2
               // Inventory rebalance preserves the existing proxy.  It is therefore
               // valid only for candidates that were already finalized as a fixed
               // envelope-fill virtual board; a raw skeleton may have an irregular
               // proxy whose outer boundary changes when a child is appended.
               && (ACandidate.BuilderName == "EnvelopeFillSearch" || ACandidate.BuilderName == "GlobalInventoryRebalance" || (IsInventorySkeletonCandidate(ACandidate) && HasFullRectangleProxy(ACandidate))) && ACandidate.SkeletonChildCount <= ACandidate.Transforms.size() && ACandidate.OriginalIndices.size() == ACandidate.Transforms.size() && ACandidate.ProxyContour.size() >= 3 && ACandidate.ProxyArea > 0.0;
    }
    bool IsRebalanceOrderBetter(const TetClusterCandidate &AFirst, const TetClusterCandidate &ASecond)
    {
        if (AFirst.SkeletonChildCount != ASecond.SkeletonChildCount) {
            return AFirst.SkeletonChildCount > ASecond.SkeletonChildCount;
        }
        if (std::abs(AFirst.ProxyWasteArea - ASecond.ProxyWasteArea) > 1.0) {
            return AFirst.ProxyWasteArea > ASecond.ProxyWasteArea;
        }
        return AFirst.ClusterType < ASecond.ClusterType;
    }
    void PreserveInventoryProxy(const TetClusterCandidate &AProxySource, TetClusterCandidate &AInOutCandidate)
    {
        AInOutCandidate.ProxyContour = AProxySource.ProxyContour;
        AInOutCandidate.ProxyContourNormalized = AProxySource.ProxyContourNormalized;
        AInOutCandidate.ProxyArea = AProxySource.ProxyArea;
        AInOutCandidate.ProxyMode = AProxySource.ProxyMode;
        AInOutCandidate.ClusterWidth = AProxySource.ClusterWidth;
        AInOutCandidate.ClusterHeight = AProxySource.ClusterHeight;
        AInOutCandidate.BoundingBoxArea = AProxySource.BoundingBoxArea;
        AInOutCandidate.ReservedArea = std::min(AProxySource.ProxyArea, AInOutCandidate.RealArea);
        AInOutCandidate.ProxyWasteArea = std::max(0.0, AProxySource.ProxyArea - AInOutCandidate.ReservedArea);
        AInOutCandidate.ProxyWasteRatio = AProxySource.ProxyArea > 0.0 ? AInOutCandidate.ProxyWasteArea / AProxySource.ProxyArea : 1.0;
        AInOutCandidate.FillRatio = AProxySource.ProxyArea > 0.0 ? std::clamp(AInOutCandidate.RealArea / AProxySource.ProxyArea, 0.0, 1.0) : 0.0;
        AInOutCandidate.BoundingFillRatio = AInOutCandidate.BoundingBoxArea > 0.0 ? std::clamp(AInOutCandidate.RealArea / AInOutCandidate.BoundingBoxArea, 0.0, 1.0) : 0.0;
        AInOutCandidate.SheetReuseScore = AProxySource.SheetReuseScore;
        AInOutCandidate.FragmentationRisk = AProxySource.FragmentationRisk;
        AInOutCandidate.Score = AProxySource.Score;
    }
    bool TryAppendInventoryFiller(const CetTNestItemVector &AOriginalItems, const std::vector<TetShapeFeature> &AFeatures, const TetNestOptions &AOptions, const TetClusterCandidate &ACurrentCandidate, int AFillerIndex, TetClusterCandidate &AOutCandidate)
    {
        AOutCandidate = TetClusterCandidate{};
        ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
        std::vector<TetClusterFreeRegion> FreeRegions;
        if (!Geometry.ExtractCandidateFreeRegions(AOriginalItems, AOptions, ACurrentCandidate, FreeRegions)) {
            std::cout << "[TEMPLATE][GLOBAL FILL REJECT] Filler=" << AFillerIndex << " Reason=FreeRegionExtract" << std::endl;
            return false;
        }
        if (!FitsAnyFreeRegion(AFeatures[AFillerIndex], FreeRegions)) {
            std::cout << "[TEMPLATE][GLOBAL FILL REJECT] Filler=" << AFillerIndex << " Reason=FreeRegionBounds RegionCount=" << FreeRegions.size() << std::endl;
            return false;
        }
        ET::NEST2DMANAGERLIB::CetRectangleFillClusterBuilder Builder;
        if (!Builder.TryAppendFillerInFreeRegions({AOriginalItems, AFeatures, AOptions, ACurrentCandidate, ACurrentCandidate, ACurrentCandidate, &FreeRegions, AFillerIndex, ACurrentCandidate.ClusterWidth, ACurrentCandidate.ClusterHeight}, AOutCandidate)) {
            std::cout << "[TEMPLATE][GLOBAL FILL REJECT] Filler=" << AFillerIndex << " Reason=ContourOrSpacing" << std::endl;
            return false;
        }
        PreserveInventoryProxy(ACurrentCandidate, AOutCandidate);
        AOutCandidate.BuilderName = "GlobalInventoryRebalance";
        AOutCandidate.ClusterType = ACurrentCandidate.ClusterType;
        return true;
    }
    bool TryRemoveInventoryFiller(const CetTNestItemVector &AOriginalItems, const TetNestOptions &AOptions, const TetClusterCandidate &ACandidate, std::size_t ATransformIndex, TetClusterCandidate &AOutCandidate)
    {
        AOutCandidate = TetClusterCandidate{};
        if (!IsInventoryRebalanceCandidate(ACandidate) || ATransformIndex < ACandidate.SkeletonChildCount || ATransformIndex >= ACandidate.Transforms.size())
            return false;
        AOutCandidate = ACandidate;
        const int OriginalId = AOutCandidate.Transforms[ATransformIndex].OriginalId;
        AOutCandidate.Transforms.erase(AOutCandidate.Transforms.begin() + static_cast<std::ptrdiff_t>(ATransformIndex));
        auto IndexIt = std::find(AOutCandidate.OriginalIndices.begin(), AOutCandidate.OriginalIndices.end(), OriginalId);
        if (IndexIt == AOutCandidate.OriginalIndices.end())
            return false;
        AOutCandidate.OriginalIndices.erase(IndexIt);
        ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
        if (!Geometry.FinalizeCandidate(AOriginalItems, AOptions, AOutCandidate))
            return false;
        PreserveInventoryProxy(ACandidate, AOutCandidate);
        AOutCandidate.BuilderName = "GlobalInventoryRebalance";
        AOutCandidate.ClusterType = ACandidate.ClusterType;
        return true;
    }
    bool IsTriangleBuilderCandidate(const TetClusterCandidate &ACandidate) { return ACandidate.Valid && ACandidate.BuilderName == "TriangleBuilder" && ACandidate.OriginalIndices.size() == ACandidate.Transforms.size() && ACandidate.OriginalIndices.size() >= 4; }
    bool IsDeferredTriangleCandidate(const TetClusterCandidate &ACandidate) { return ACandidate.BuilderName == "TriangleBuilder"; }
    bool HasFilledEllipseCandidate(const std::vector<TetClusterCandidate> &ACandidates)
    {
        for (const TetClusterCandidate &Candidate : ACandidates) {
            if (Candidate.ClusterType.find("Ellipse") != std::string::npos && Candidate.OriginalIndices.size() > Candidate.SkeletonChildCount)
                return true;
        }
        return false;
    }
    bool HasExactCandidateInventory(const TetClusterCandidate &ACandidate, const std::vector<int> &AExpectedIndices)
    {
        if (!ACandidate.Valid || ACandidate.OriginalIndices.size() != AExpectedIndices.size() || ACandidate.Transforms.size() != AExpectedIndices.size())
            return false;
        std::set<int> Actual(ACandidate.OriginalIndices.begin(), ACandidate.OriginalIndices.end());
        std::set<int> Expected(AExpectedIndices.begin(), AExpectedIndices.end());
        return Actual == Expected && Actual.size() == AExpectedIndices.size();
    }
    bool IsInventoryTransferWorthKeeping(const TetClusterCandidate &ATargetBefore, const TetClusterCandidate &ATargetAfter, const TetClusterCandidate &ASourceBefore, const TetClusterCandidate &ASourceAfter);
    bool TryBuildReducedTriangleCandidate(const CetTNestItemVector &AOriginalItems, const std::vector<TetShapeFeature> &AFeatures, const TetNestOptions &AOptions, const TetClusterCandidate &ASourceCandidate, const std::set<int> &ARemovedIds, TetClusterCandidate &AOutCandidate)
    {
        AOutCandidate = TetClusterCandidate{};
        std::vector<int> Remaining;
        for (int OriginalId : ASourceCandidate.OriginalIndices) {
            if (ARemovedIds.find(OriginalId) == ARemovedIds.end())
                Remaining.push_back(OriginalId);
        }
        if (Remaining.size() < 2)
            return false;
        ET::NEST2DMANAGERLIB::CetTriangleClusterBuilder Builder;
        std::vector<TetClusterCandidate> Candidates;
        Builder.BuildCandidates(AOriginalItems, AFeatures, Remaining, AOptions, Candidates);
        for (const TetClusterCandidate &Candidate : Candidates) {
            if (HasExactCandidateInventory(Candidate, Remaining)) {
                AOutCandidate = Candidate;
                PreserveInventoryProxy(ASourceCandidate, AOutCandidate);
                AOutCandidate.BuilderName = "GlobalInventoryRebalance";
                AOutCandidate.ClusterType = ASourceCandidate.ClusterType + "_Reduced";
                return true;
            }
        }
        return false;
    }
    bool IsTriangleTransferWorthKeeping(const TetClusterCandidate &ATargetBefore, const TetClusterCandidate &ATargetAfter, const TetClusterCandidate &ASourceBefore, const TetClusterCandidate &ASourceAfter);
    bool TryTransferTrianglePair(const TetClusterFillContext &AContext, TetClusterCandidate &AOutTarget, TetClusterCandidate &AOutSource, std::pair<int, int> &AOutPair)
    {
        const auto &AOriginalItems = AContext.OriginalItems; const auto &AFeatures = AContext.Features; const auto &AOptions = AContext.Options; const auto &ATargetBefore = AContext.BaseCandidate; const auto &ASourceBefore = AContext.EnvelopeCandidate;
        const std::size_t ItemCount = ASourceBefore.OriginalIndices.size();
        for (std::size_t First = ItemCount - 2;; --First) {
            for (std::size_t Second = ItemCount - 1; Second > First; --Second) {
                const int FirstId = ASourceBefore.OriginalIndices[First];
                const int SecondId = ASourceBefore.OriginalIndices[Second];
                TetClusterCandidate ExpandedTarget;
                if (!TryAppendInventoryFiller(AOriginalItems, AFeatures, AOptions, ATargetBefore, FirstId, ExpandedTarget)) {
                    continue;
                }
                TetClusterCandidate ExpandedTargetPair;
                if (!TryAppendInventoryFiller(AOriginalItems, AFeatures, AOptions, ExpandedTarget, SecondId, ExpandedTargetPair)) {
                    std::cout << "[TEMPLATE][TRIANGLE TRANSFER PROBE] Pair=" << FirstId << "," << SecondId << " SecondAppend=reject" << std::endl;
                    continue;
                }
                const std::set<int> RemovedIds = {FirstId, SecondId};
                TetClusterCandidate ReducedSource;
                if (!TryBuildReducedTriangleCandidate(AOriginalItems, AFeatures, AOptions, ASourceBefore, RemovedIds, ReducedSource)) {
                    std::cout << "[TEMPLATE][TRIANGLE TRANSFER PROBE] Pair=" << FirstId << "," << SecondId << " ReducedSource=reject" << std::endl;
                    continue;
                }
                if (!IsTriangleTransferWorthKeeping(ATargetBefore, ExpandedTargetPair, ASourceBefore, ReducedSource)) {
                    std::cout << "[TEMPLATE][TRIANGLE TRANSFER PROBE] Pair=" << FirstId << "," << SecondId << " Gain=reject" << std::endl;
                    continue;
                }
                AOutTarget = std::move(ExpandedTargetPair);
                AOutSource = std::move(ReducedSource);
                AOutPair = {FirstId, SecondId};
                return true;
            }
            if (First == 0)
                break;
        }
        return false;
    }
    bool IsTransferPairGloballyUnique(const std::vector<TetClusterCandidate> &AAcceptedCandidates, std::size_t ASourceIndex, const std::pair<int, int> &APair)
    {
        for (std::size_t CandidateIndex = 0; CandidateIndex < AAcceptedCandidates.size(); ++CandidateIndex) {
            if (CandidateIndex == ASourceIndex)
                continue;
            for (int OriginalId : AAcceptedCandidates[CandidateIndex].OriginalIndices) {
                if (OriginalId == APair.first || OriginalId == APair.second)
                    return false;
            }
        }
        return APair.first >= 0 && APair.second >= 0 && APair.first != APair.second;
    }
    bool IsTriangleTransferWorthKeeping(const TetClusterCandidate &ATargetBefore, const TetClusterCandidate &ATargetAfter, const TetClusterCandidate &ASourceBefore, const TetClusterCandidate &ASourceAfter)
    {
        if (ATargetAfter.RealArea <= ATargetBefore.RealArea)
            return false;
        const double TotalBefore = ATargetBefore.ProxyWasteArea + ASourceBefore.ProxyWasteArea;
        const double TotalAfter = ATargetAfter.ProxyWasteArea + ASourceAfter.ProxyWasteArea;
        const double Tolerance = std::max(1.0, ATargetBefore.ProxyArea * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE);
        std::cout << "[TEMPLATE][TRIANGLE TRANSFER METRIC] TargetArea=" << ATargetBefore.RealArea << "->" << ATargetAfter.RealArea << " Waste=" << TotalBefore << "->" << TotalAfter << std::endl;
        return TotalAfter <= TotalBefore + Tolerance;
    }
    bool IsInventoryTransferWorthKeeping(const TetClusterCandidate &ATargetBefore, const TetClusterCandidate &ATargetAfter, const TetClusterCandidate &ASourceBefore, const TetClusterCandidate &ASourceAfter)
    {
        const double TargetTolerance = std::max(1.0, ATargetBefore.ProxyArea * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE);
        const double TotalBefore = ATargetBefore.ProxyWasteArea + ASourceBefore.ProxyWasteArea;
        const double TotalAfter = ATargetAfter.ProxyWasteArea + ASourceAfter.ProxyWasteArea;
        return ATargetAfter.ProxyWasteArea < ATargetBefore.ProxyWasteArea - TargetTolerance && TotalAfter <= TotalBefore + TargetTolerance;
    }
    bool TryBuildInventorySkeletonEnvelope(const CetTNestItemVector &AOriginalItems, const TetNestOptions &AOptions, const TetClusterCandidate &ACandidate, TetClusterCandidate &AOutCandidate)
    {
        AOutCandidate = TetClusterCandidate{};
        if (!IsInventorySkeletonCandidate(ACandidate) || !BuildRectangleEnvelopeCandidate(AOriginalItems, AOptions, ACandidate, AOutCandidate))
            return false;
        AOutCandidate.BuilderName = "EnvelopeFillSearch";
        AOutCandidate.ClusterType = ACandidate.ClusterType + "_InventoryEnvelope";
        AOutCandidate.SkeletonChildCount = ACandidate.SkeletonChildCount;
        AOutCandidate.Score = ACandidate.Score;
        return true;
    }
    void RebalanceAcceptedClusterInventory(const CetTNestItemVector &AOriginalItems, const std::vector<TetShapeFeature> &AFeatures, const TetNestOptions &AOptions, std::vector<TetClusterCandidate> &AAcceptedCandidates, std::vector<bool> &AUsed)
    {
        const auto Start = std::chrono::steady_clock::now();
        std::vector<std::size_t> Targets;
        for (std::size_t Index = 0; Index < AAcceptedCandidates.size(); ++Index) {
            if (IsInventoryRebalanceCandidate(AAcceptedCandidates[Index]) || IsInventorySkeletonCandidate(AAcceptedCandidates[Index]))
                Targets.push_back(Index);
        }
        std::stable_sort(Targets.begin(), Targets.end(), [&](std::size_t A, std::size_t B) { return IsRebalanceOrderBetter(AAcceptedCandidates[A], AAcceptedCandidates[B]); });
        std::size_t Attempts = 0;
        for (std::size_t TargetIndex : Targets) {
            if (Attempts >= CET_CLUSTER_GLOBAL_REBALANCE_MAX_ATTEMPTS || CircleGapSearchTimeReached(Start, CET_CLUSTER_GLOBAL_REBALANCE_MAX_SEARCH_TIME_MS))
                break;
            std::vector<int> Available;
            for (int Index = 0; Index < static_cast<int>(AFeatures.size()) && Available.size() < CET_CLUSTER_GLOBAL_REBALANCE_MAX_UNASSIGNED_FILLERS; ++Index) {
                if (!AUsed[Index] && AFeatures[Index].Area > 0.0)
                    Available.push_back(Index);
            }
            std::stable_sort(Available.begin(), Available.end(), [&](int A, int B) { return AFeatures[A].Area > AFeatures[B].Area; });
            TetClusterCandidate TargetCandidate = AAcceptedCandidates[TargetIndex];
            if (!IsInventoryRebalanceCandidate(TargetCandidate)) {
                TetClusterCandidate EnvelopeCandidate;
                if (!TryBuildInventorySkeletonEnvelope(AOriginalItems, AOptions, TargetCandidate, EnvelopeCandidate))
                    continue;
                TargetCandidate = std::move(EnvelopeCandidate);
            }
            for (int FillerIndex : Available) {
                if (Attempts++ >= CET_CLUSTER_GLOBAL_REBALANCE_MAX_ATTEMPTS)
                    break;
                TetClusterCandidate Expanded;
                if (!TryAppendInventoryFiller(AOriginalItems, AFeatures, AOptions, TargetCandidate, FillerIndex, Expanded))
                    continue;
                AAcceptedCandidates[TargetIndex] = std::move(Expanded);
                TargetCandidate = AAcceptedCandidates[TargetIndex];
                AUsed[FillerIndex] = true;
                std::cout << "[TEMPLATE][GLOBAL FILL] Target=" << TargetIndex << " Filler=" << FillerIndex << std::endl;
            }
        }
        std::size_t Transfers = 0;
        for (std::size_t TargetIndex : Targets) {
            if (Transfers >= CET_CLUSTER_GLOBAL_REBALANCE_MAX_TRANSFERS || CircleGapSearchTimeReached(Start, CET_CLUSTER_GLOBAL_REBALANCE_MAX_SEARCH_TIME_MS))
                break;
            if (!IsInventoryRebalanceCandidate(AAcceptedCandidates[TargetIndex]))
                continue;
            for (std::size_t SourceIndex : Targets) {
                if (TargetIndex == SourceIndex || AAcceptedCandidates[TargetIndex].SkeletonChildCount <= AAcceptedCandidates[SourceIndex].SkeletonChildCount)
                    continue;
                for (std::size_t FillerIndex = AAcceptedCandidates[SourceIndex].SkeletonChildCount; FillerIndex < AAcceptedCandidates[SourceIndex].Transforms.size(); ++FillerIndex) {
                    if (Transfers >= CET_CLUSTER_GLOBAL_REBALANCE_MAX_TRANSFERS || CircleGapSearchTimeReached(Start, CET_CLUSTER_GLOBAL_REBALANCE_MAX_SEARCH_TIME_MS))
                        return;
                    const int OriginalId = AAcceptedCandidates[SourceIndex].Transforms[FillerIndex].OriginalId;
                    TetClusterCandidate ReducedSource;
                    TetClusterCandidate ExpandedTarget;
                    if (!TryRemoveInventoryFiller(AOriginalItems, AOptions, AAcceptedCandidates[SourceIndex], FillerIndex, ReducedSource) || !TryAppendInventoryFiller(AOriginalItems, AFeatures, AOptions, AAcceptedCandidates[TargetIndex], OriginalId, ExpandedTarget) || !IsInventoryTransferWorthKeeping(AAcceptedCandidates[TargetIndex], ExpandedTarget, AAcceptedCandidates[SourceIndex], ReducedSource))
                        continue;
                    AAcceptedCandidates[TargetIndex] = std::move(ExpandedTarget);
                    AAcceptedCandidates[SourceIndex] = std::move(ReducedSource);
                    ++Transfers;
                    std::cout << "[TEMPLATE][GLOBAL TRANSFER] Source=" << SourceIndex << " Target=" << TargetIndex << " Filler=" << OriginalId << std::endl;
                    break;
                }
            }
        }
        const auto TriangleTransferStart = std::chrono::steady_clock::now();
        for (std::size_t TargetIndex : Targets) {
            if (Transfers >= CET_CLUSTER_GLOBAL_REBALANCE_MAX_TRANSFERS || CircleGapSearchTimeReached(TriangleTransferStart, CET_CLUSTER_GLOBAL_REBALANCE_MAX_SEARCH_TIME_MS))
                break;
            if (!IsInventoryRebalanceCandidate(AAcceptedCandidates[TargetIndex]))
                continue;
            for (std::size_t SourceIndex = 0; SourceIndex < AAcceptedCandidates.size(); ++SourceIndex) {
                if (TargetIndex == SourceIndex || !IsTriangleBuilderCandidate(AAcceptedCandidates[SourceIndex]))
                    continue;
                TetClusterCandidate ExpandedTarget;
                TetClusterCandidate ReducedSource;
                std::pair<int, int> MovedPair = {-1, -1};
                if (!TryTransferTrianglePair({AOriginalItems, AFeatures, AOptions, AAcceptedCandidates[TargetIndex], AAcceptedCandidates[SourceIndex]}, ExpandedTarget, ReducedSource, MovedPair))
                    continue;
                if (!IsTransferPairGloballyUnique(AAcceptedCandidates, SourceIndex, MovedPair)) {
                    std::cout << "[TEMPLATE][TRIANGLE TRANSFER] Pair=" << MovedPair.first << "," << MovedPair.second << " DuplicateInventory=reject" << std::endl;
                    continue;
                }
                AAcceptedCandidates[TargetIndex] = std::move(ExpandedTarget);
                AAcceptedCandidates[SourceIndex] = std::move(ReducedSource);
                ++Transfers;
                std::cout << "[TEMPLATE][TRIANGLE TRANSFER] Source=" << SourceIndex << " Target=" << TargetIndex << " Fillers=" << MovedPair.first << "," << MovedPair.second << std::endl;
                break;
            }
        }
    }
    std::vector<int> CollectCompatibleFillers(const std::vector<TetShapeFeature> &AFeatures, const TetClusterCandidate &ABaseCandidate, const std::vector<TetClusterFreeRegion> &AFreeRegions, const TetClusterFillSearchConfig &AConfig, bool ADeduplicateFamilies = false)
    {
        std::vector<int> Fillers;
        const double AvailableArea = std::max(0.0, ABaseCandidate.ProxyWasteArea);
        const double AreaTolerance = std::max(1.0, ABaseCandidate.ProxyArea * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE);
        for (int Index = 0; Index < static_cast<int>(AFeatures.size()); ++Index) {
            const TetShapeFeature &Feature = AFeatures[Index];
            if (!ContainsOriginalIndex(ABaseCandidate, Index) && std::isfinite(Feature.Area) && Feature.Area > 0.0 && Feature.Area <= AvailableArea + AreaTolerance && FitsAnyFreeRegion(Feature, AFreeRegions))
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
            Fillers.erase(std::remove_if(Fillers.begin(), Fillers.end(), [&](int Index) { return !SeenFamilies.insert(MakeFillerFamilyKey(AFeatures[Index])).second; }), Fillers.end());
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
    void BuildFilledVariantsForBase(const TetClusterFillContext &AContext, const TetClusterFillSearchConfig &AConfig, std::vector<TetClusterCandidate> &AOutVariants, TetClusterFillSearchStats &AStats)
    {
        const auto &AOriginalItems = AContext.OriginalItems; const auto &AFeatures = AContext.Features; const auto &AOptions = AContext.Options; const auto &ABaseCandidate = AContext.BaseCandidate;
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
                const std::vector<int> Fillers = CollectCompatibleFillers(AFeatures, State.Candidate, FreeRegions, AConfig);
                for (int FillerIndex : Fillers) {
                    if (AConfig.MaxPlacementAttempts > 0 && AStats.SearchAttempts >= AConfig.MaxPlacementAttempts)
                        break;
                    if (ContainsOriginalIndex(State.Candidate, FillerIndex))
                        continue;
                    ++AStats.SearchAttempts;
                    TetClusterCandidate Candidate;
                    if (!Builder.TryAppendFillerInFreeRegions({AOriginalItems, AFeatures, AOptions, ABaseCandidate, ABaseCandidate, State.Candidate, &FreeRegions, FillerIndex, ABaseCandidate.ClusterWidth, ABaseCandidate.ClusterHeight}, Candidate))
                        continue;
                    if (!IsFilledVariantWorthKeeping(ABaseCandidate, Candidate))
                        continue;
                    NextBeam.push_back({std::move(Candidate), State.FillerCount + 1});
                    ++AStats.GeneratedVariantCount;
                }
            }
            if (AConfig.MaxPlacementAttempts > 0 && AStats.SearchAttempts >= AConfig.MaxPlacementAttempts)
                break;
            DeduplicateFilledStates(NextBeam);
            TrimFillBeam(NextBeam, AConfig.BeamWidth);
            AllVariants.insert(AllVariants.end(), NextBeam.begin(), NextBeam.end());
            Beam = std::move(NextBeam);
        }
        DeduplicateFilledStates(AllVariants);
        TrimFillBeam(AllVariants, CET_CLUSTER_FILL_MAX_VARIANTS_PER_BASE);
        AStats.DeduplicatedVariantCount += AllVariants.size();
        for (const TetClusterFillSearchState &State : AllVariants) {
            AStats.FilledFillRatioSum += State.Candidate.FillRatio;
            AStats.BestFillRatioGain = std::max(AStats.BestFillRatioGain, State.Candidate.FillRatio - ABaseCandidate.FillRatio);
            AOutVariants.push_back(State.Candidate);
        }
    }
    TetClusterFillSearchState BuildEnvelopeFillSeed(const TetClusterFillContext &AContext, std::map<std::string, TetCircleGapTemplate> &AGapTemplates, const TetClusterFillSearchState *ASeedState, bool AIsCircleBase, bool AIsEllipseBase, const TetClusterFillSearchConfig &AConfig, std::vector<TetClusterFillSearchState> &AOutStates)
    {
        const auto &AOriginalItems = AContext.OriginalItems; const auto &AFeatures = AContext.Features; const auto &AOptions = AContext.Options; const auto &ABaseCandidate = AContext.BaseCandidate; const auto &AEnvelopeCandidate = AContext.EnvelopeCandidate;
        AOutStates.clear();
        if (ASeedState != nullptr && PreservesBaseTransforms(ABaseCandidate, ASeedState->Candidate) && IsEnvelopeFillStateWorthExpanding(AEnvelopeCandidate, ASeedState->Candidate)) {
            AOutStates.push_back(*ASeedState);
        }
        TetClusterCandidate LocalGap;
        const TetClusterFillContext FillContext = AContext;
        if (AIsCircleBase && ET::NEST2DMANAGERLIB::_GetNest2DInvokeFunctor()->Nest2dCircleGapFiller->BuildCircleGapCandidate(FillContext, AGapTemplates, LocalGap) && PreservesBaseTransforms(ABaseCandidate, LocalGap) && IsEnvelopeFillStateWorthExpanding(AEnvelopeCandidate, LocalGap)) {
            AOutStates.push_back({LocalGap, LocalGap.Transforms.size() - ABaseCandidate.Transforms.size()});
            std::cout << "[TEMPLATE][LOCAL GAP] Fillers=" << AOutStates.back().FillerCount << std::endl;
        }
        TetClusterCandidate EllipseGap;
        if (AIsEllipseBase && ET::NEST2DMANAGERLIB::_GetNest2DInvokeFunctor()->Nest2dEllipseGapFiller->BuildEllipseGapCandidate(FillContext, AConfig, EllipseGap) && PreservesBaseTransforms(ABaseCandidate, EllipseGap) && IsEnvelopeFillStateWorthExpanding(AEnvelopeCandidate, EllipseGap)) {
            const std::size_t FillerCount = EllipseGap.Transforms.size() > ABaseCandidate.Transforms.size() ? EllipseGap.Transforms.size() - ABaseCandidate.Transforms.size() : 0;
            AOutStates.push_back({std::move(EllipseGap), FillerCount});
            std::cout << "[TEMPLATE][ELLIPSE LOCAL GAP] Fillers=" << AOutStates.back().FillerCount << std::endl;
        }
        const TetClusterCandidate *FreeRegionSeed = &AEnvelopeCandidate;
        if (!AOutStates.empty()) {
            FreeRegionSeed = &std::max_element(AOutStates.begin(), AOutStates.end(), [](const TetClusterFillSearchState &AFirst, const TetClusterFillSearchState &ASecond) { return IsEnvelopeStateBetter(ASecond, AFirst); })->Candidate;
        }
        TetClusterCandidate FreeRegionGap;
        if (ABaseCandidate.SkeletonChildCount >= 3 && ET::NEST2DMANAGERLIB::_GetNest2DInvokeFunctor()->Nest2dCircleGapFiller->BuildFreeRegionTemplateCandidate(FillContext, *FreeRegionSeed, FreeRegionGap) && PreservesBaseTransforms(ABaseCandidate, FreeRegionGap) && IsEnvelopeFillStateWorthExpanding(AEnvelopeCandidate, FreeRegionGap)) {
            const std::size_t FillerCount = FreeRegionGap.Transforms.size() > ABaseCandidate.Transforms.size() ? FreeRegionGap.Transforms.size() - ABaseCandidate.Transforms.size() : 0;
            AOutStates.push_back({std::move(FreeRegionGap), FillerCount});
            std::cout << "[TEMPLATE][FREE REGION GAP] Fillers=" << AOutStates.back().FillerCount << std::endl;
        }
        TetClusterFillSearchState Initial{AEnvelopeCandidate, 0};
        if (!AOutStates.empty()) {
            Initial = *std::max_element(AOutStates.begin(), AOutStates.end(), [](const TetClusterFillSearchState &AFirst, const TetClusterFillSearchState &ASecond) { return IsEnvelopeStateBetter(ASecond, AFirst); });
        }
        std::cout << "[TEMPLATE][ELLIPSE SEARCH SEED] IsEllipse=" << AIsEllipseBase << ", States=" << AOutStates.size() << ", Fillers=" << Initial.FillerCount << std::endl;
        return Initial;
    }
    void SearchEnvelopeFillVariants(const TetClusterFillContext &AContext, const TetClusterFillSearchConfig &AConfig, std::map<std::string, TetCircleGapTemplate> &AGapTemplates, const TetClusterFillSearchState *ASeedState, std::vector<TetClusterFillSearchState> &AOutStates, TetClusterFillSearchStats &AStats)
    {
        const auto &AOriginalItems = AContext.OriginalItems; const auto &AFeatures = AContext.Features; const auto &AOptions = AContext.Options; const auto &ABaseCandidate = AContext.BaseCandidate; const auto &AEnvelopeCandidate = AContext.EnvelopeCandidate;
        std::size_t LocalAttempts = 0;
        const bool IsCircleBase = IsFixedCircleEnvelopeBase(ABaseCandidate, AFeatures);
        const std::vector<TetCircleGapTemplateAnchor> Anchors = IsCircleBase ? ET::NEST2DMANAGERLIB::_GetNest2DInvokeFunctor()->Nest2dCircleGapFiller->CollectCircleGapTemplateAnchors(AOriginalItems, AFeatures, ABaseCandidate) : std::vector<TetCircleGapTemplateAnchor>{};
        ET::NEST2DMANAGERLIB::CetRectangleFillClusterBuilder Builder;
        ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
        AOutStates.clear();
        const bool IsEllipseBase = IsFixedEllipseEnvelopeBase(ABaseCandidate, AFeatures);
        const TetClusterFillSearchState Initial = BuildEnvelopeFillSeed(AContext, AGapTemplates, ASeedState, IsCircleBase, IsEllipseBase, AConfig, AOutStates);
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
        DeduplicateFilledStates(Beam);
        TrimEnvelopeBeam(Beam, AConfig.BeamWidth, AFeatures, ABaseCandidate.SkeletonChildCount, IsEllipseBase);
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
                const std::vector<int> Fillers = CollectCompatibleFillers(AFeatures, State.Candidate, StateFreeRegions, AConfig, true);
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
                    if (!PreservesBaseTransforms(ABaseCandidate, Candidate) || !IsEnvelopeFillStateWorthExpanding(AEnvelopeCandidate, Candidate))
                        continue;
                    Next.push_back({std::move(Candidate), State.FillerCount + 1 + Copies});
                    ++AStats.EnvelopeGeneratedVariantCount;
                }
            }
            DeduplicateFilledStates(Next);
            TrimEnvelopeBeam(Next, AConfig.BeamWidth, AFeatures, ABaseCandidate.SkeletonChildCount, IsEllipseBase);
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
    bool TryBuildCachedEllipseTemplateSeed(const TetClusterFillContext &AContext, const TetEllipseGapTemplateCache &ATemplates, TetClusterFillSearchState &AOutSeed)
    {
        const auto &ABaseCandidate = AContext.BaseCandidate;
        AOutSeed = TetClusterFillSearchState{};
        TetClusterCandidate Candidate;
        if (!ET::NEST2DMANAGERLIB::_GetNest2DInvokeFunctor()->Nest2dEllipseGapFiller->TryBuildCachedEllipseTemplateVariant(AContext, ATemplates, Candidate) || !PreservesBaseTransforms(ABaseCandidate, Candidate) || !IsEnvelopeFillStateWorthExpanding(AContext.EnvelopeCandidate, Candidate))
            return false;
        AOutSeed.Candidate = std::move(Candidate);
        AOutSeed.FillerCount = AOutSeed.Candidate.Transforms.size() - ABaseCandidate.Transforms.size();
        return true;
    }
    void BuildEnvelopeFilledVariantsForBase(const TetEnvelopeFillVariantRequest &ARequest)
    {
        const auto &AOriginalItems = ARequest.OriginalItems; const auto &AFeatures = ARequest.Features; const auto &AOptions = ARequest.Options; const auto &ABaseCandidate = ARequest.BaseCandidate; const auto &AConfig = ARequest.Config; auto &AGapTemplates = ARequest.GapTemplates; auto &AEllipseTemplates = ARequest.EllipseTemplates; auto &AOutVariants = ARequest.OutVariants; auto &AStats = ARequest.Stats;
        AOutVariants.clear();
        TetClusterCandidate EnvelopeCandidate = ABaseCandidate;
        if (!HasFullRectangleProxy(ABaseCandidate) && !BuildRectangleEnvelopeCandidate(AOriginalItems, AOptions, ABaseCandidate, EnvelopeCandidate))
            return;
        const bool IsEllipseBase = IsFixedEllipseEnvelopeBase(ABaseCandidate, AFeatures);
        TetClusterFillSearchState CachedSeed;
        const TetClusterFillContext FillContext{AOriginalItems, AFeatures, AOptions, ABaseCandidate, EnvelopeCandidate};
        if (IsEllipseBase && TryBuildCachedEllipseTemplateSeed(FillContext, AEllipseTemplates, CachedSeed)) {
            std::cout << "[TEMPLATE][ELLIPSE GAP CACHE SEED] Fillers=" << CachedSeed.FillerCount << std::endl;
        }
        std::vector<TetClusterFillSearchState> States;
        SearchEnvelopeFillVariants(FillContext, AConfig, AGapTemplates, CachedSeed.Candidate.Valid ? &CachedSeed : nullptr, States, AStats);
        FinalizeEnvelopeFilledStates({AOriginalItems, AFeatures, AOptions, ABaseCandidate, EnvelopeCandidate}, States, AOutVariants, AStats);
        if (IsEllipseBase)
            ET::NEST2DMANAGERLIB::_GetNest2DInvokeFunctor()->Nest2dEllipseGapFiller->CacheEllipseTemplateVariant(FillContext, States, !AOutVariants.empty(), AEllipseTemplates);
    }
    TetPairCandidateKey MakePairCandidateKey(int AFirst, int ASecond)
    {
        if (AFirst > ASecond) {
            std::swap(AFirst, ASecond);
        }
        return {AFirst, ASecond};
    }
    using TPairCandidateLookup = std::unordered_map<TetPairCandidateKey, const TetClusterCandidate *, TetPairCandidateKeyHash>;
    bool IsPairCandidateUsable(const TetClusterCandidate &ACandidate, int AOriginalItemCount)
    {
        if (!ACandidate.Valid || ACandidate.OriginalIndices.size() != 2 || ACandidate.Transforms.size() != 2 || ACandidate.ProxyContour.size() < 3 || ACandidate.ClusterWidth <= 0.0 || ACandidate.ClusterHeight <= 0.0 || ACandidate.ProxyArea <= 0.0 || !std::isfinite(ACandidate.Score)) {
            return false;
        }
        const int FirstIndex = ACandidate.OriginalIndices[0];
        const int SecondIndex = ACandidate.OriginalIndices[1];
        return FirstIndex >= 0 && SecondIndex >= 0 && FirstIndex < AOriginalItemCount && SecondIndex < AOriginalItemCount && FirstIndex != SecondIndex;
    }
    void BuildPairCandidateLookup(const std::vector<TetClusterCandidate> &ACandidates, int AOriginalItemCount, TPairCandidateLookup &AOutLookup)
    {
        AOutLookup.clear();
        AOutLookup.reserve(ACandidates.size());
        for (const TetClusterCandidate &Candidate : ACandidates) {
            if (!IsPairCandidateUsable(Candidate, AOriginalItemCount)) {
                continue;
            }
            const TetPairCandidateKey Key = MakePairCandidateKey(Candidate.OriginalIndices[0], Candidate.OriginalIndices[1]);
            auto It = AOutLookup.find(Key);
            if (It == AOutLookup.end() || Candidate.Score > It->second->Score) {
                AOutLookup[Key] = &Candidate;
            }
        }
    }
    std::vector<std::size_t> CollectPairCandidatePositions(const std::vector<TetClusterCandidate> &ACandidates)
    {
        std::vector<std::size_t> PairPositions;
        PairPositions.reserve(ACandidates.size());
        for (std::size_t CandidateIndex = 0; CandidateIndex < ACandidates.size(); ++CandidateIndex) {
            const TetClusterCandidate &Candidate = ACandidates[CandidateIndex];
            if (Candidate.Valid && Candidate.OriginalIndices.size() == 2 && std::isfinite(Candidate.Score)) {
                PairPositions.push_back(CandidateIndex);
            }
        }
        std::stable_sort(PairPositions.begin(), PairPositions.end(), [&](std::size_t AFirstPosition, std::size_t ASecondPosition) { return ACandidates[AFirstPosition].Score > ACandidates[ASecondPosition].Score; });
        if (PairPositions.size() > static_cast<std::size_t>(CET_CLUSTER_FILL_MAX_SWAP_CLUSTERS)) {
            PairPositions.resize(CET_CLUSTER_FILL_MAX_SWAP_CLUSTERS);
        }
        return PairPositions;
    }
    bool TryFindBetterPairSwap(const TPairCandidateLookup &APairCandidateLookup, const TetClusterCandidate &AFirstCandidate, const TetClusterCandidate &ASecondCandidate, const TetClusterCandidate *&AOutFirstCandidate, const TetClusterCandidate *&AOutSecondCandidate)
    {
        AOutFirstCandidate = nullptr;
        AOutSecondCandidate = nullptr;
        if (AFirstCandidate.OriginalIndices.size() != 2 || ASecondCandidate.OriginalIndices.size() != 2) {
            return false;
        }
        const int A = AFirstCandidate.OriginalIndices[0];
        const int B = AFirstCandidate.OriginalIndices[1];
        const int C = ASecondCandidate.OriginalIndices[0];
        const int D = ASecondCandidate.OriginalIndices[1];
        if (A == B || A == C || A == D || B == C || B == D || C == D) {
            return false;
        }
        double BestScore = AFirstCandidate.Score + ASecondCandidate.Score;
        const auto TrySelectSwap = [&](int AFirstIndex, int ASecondIndex, int BFirstIndex, int BSecondIndex) {
            const auto FirstIt = APairCandidateLookup.find(MakePairCandidateKey(AFirstIndex, ASecondIndex));
            const auto SecondIt = APairCandidateLookup.find(MakePairCandidateKey(BFirstIndex, BSecondIndex));
            if (FirstIt == APairCandidateLookup.end() || SecondIt == APairCandidateLookup.end() || FirstIt->second == SecondIt->second) {
                return;
            }
            const double NewScore = FirstIt->second->Score + SecondIt->second->Score;
            const double GainRatio = (NewScore - BestScore) / std::max(std::abs(BestScore), 1.0);
            if (GainRatio >= CET_CLUSTER_FILL_MIN_SWAP_GAIN_RATIO) {
                BestScore = NewScore;
                AOutFirstCandidate = FirstIt->second;
                AOutSecondCandidate = SecondIt->second;
            }
        };
        TrySelectSwap(A, C, B, D);
        TrySelectSwap(A, D, B, C);
        return AOutFirstCandidate != nullptr && AOutSecondCandidate != nullptr;
    }
} // namespace

namespace ET {
    namespace NEST2DMANAGERLIB {
        CetClusterTemplateFillOptimizer::CetClusterTemplateFillOptimizer() : CetCoreObject() {}
        CetClusterTemplateFillOptimizer::~CetClusterTemplateFillOptimizer() {}

        TetClusterBuildResult CetClusterTemplateFillOptimizer::BuildTemplateClusters(const CetTNestItemVector &AOriginalItems, const std::vector<TetShapeFeature> &AFeatures, const TetNestOptions &AOptions)
        {
            const int Count = static_cast<int>(AOriginalItems.size());
            if (Count <= 0) {
                return TetClusterBuildResult{};
            }
            if (AFeatures.size() != AOriginalItems.size()) {
                std::cout << "[TEMPLATE][ERROR] Feature count mismatch. OriginalItems=" << AOriginalItems.size() << ", Features=" << AFeatures.size() << std::endl;
                return Nest2DUtils->Nest2DCluster->BuildClusterResultFromCandidates(AOriginalItems, {}, AOptions);
            }
            std::vector<bool> Used(Count, false);
            std::map<MetShapeType, std::vector<int>> IndicesByType;
            //  褰㈢姸鍒嗙被
            _CollectTemplateShapeIndices(AFeatures, IndicesByType);
            // Collect all template candidates.
            std::vector<TetClusterCandidate> BaseCandidates;
            _BuildTemplateCandidates(AOriginalItems, AFeatures, AOptions, IndicesByType, BaseCandidates);
            // Keep skeletons and their bounded fill variants together for global selection.
            std::vector<TetClusterCandidate> ExpandedCandidates;
            _BuildFilledTemplateCandidateVariants(AOriginalItems, AFeatures, AOptions, BaseCandidates, ExpandedCandidates);
            std::vector<TetClusterCandidate> AcceptedCandidates = _SelectAndOptimizeTemplateCandidates(AOriginalItems, AFeatures, AOptions, ExpandedCandidates, Used, Count);
            return Nest2DUtils->Nest2DCluster->BuildClusterResultFromCandidates(AOriginalItems, AcceptedCandidates, AOptions);
        }
        void CetClusterTemplateFillOptimizer::_CollectTemplateShapeIndices(const std::vector<TetShapeFeature> &AFeatures, std::map<MetShapeType, std::vector<int>> &AIndicesByType)
        {
            const int Count = static_cast<int>(AFeatures.size());
            for (int i = 0; i < Count; ++i) {
                const TetShapeFeature &Feature = AFeatures[i];
                if (Feature.Width <= 0.0 || Feature.Height <= 0.0) {
                    continue;
                }
                AIndicesByType[Feature.ShapeType].push_back(i);
            }
            std::cout << "[TEMPLATE][SHAPE COUNTS] Triangle=" << AIndicesByType[MetShapeType::TriangleLike].size() << " Circle=" << AIndicesByType[MetShapeType::CircleLike].size() << " Ellipse=" << AIndicesByType[MetShapeType::EllipseLike].size() << " Rectangle=" << AIndicesByType[MetShapeType::RectangleLike].size() << " Arc=" << AIndicesByType[MetShapeType::ArcLike].size() << " Convex=" << AIndicesByType[MetShapeType::ConvexPolygon].size() << " Concave=" << AIndicesByType[MetShapeType::ConcavePolygon].size() << std::endl;
        }
        void CetClusterTemplateFillOptimizer::_BuildTemplateCandidates(const CetTNestItemVector &AOriginalItems, const std::vector<TetShapeFeature> &AFeatures, const TetNestOptions &AOptions, const std::map<MetShapeType, std::vector<int>> &AIndicesByType, std::vector<TetClusterCandidate> &ABaseCandidates)
        {
            auto AppendBuilderLog = [&](const char *ABuilderName, std::size_t AOldCount) { std::cout << "[TEMPLATE][BUILDER] " << ABuilderName << " NewCandidates=" << ABaseCandidates.size() - AOldCount << std::endl; };
            {
                const std::size_t OldCount = ABaseCandidates.size();
                CetTriangleClusterBuilder Builder;
                Builder.BuildCandidates(AOriginalItems, AFeatures, AIndicesByType.at(MetShapeType::TriangleLike), AOptions, ABaseCandidates);
                AppendBuilderLog("TriangleBuilder", OldCount);
            }
            {
                const std::size_t OldCount = ABaseCandidates.size();
                CetCircleClusterBuilder Builder;
                Builder.BuildCandidates(AOriginalItems, AFeatures, AIndicesByType.at(MetShapeType::CircleLike), AOptions, ABaseCandidates);
                AppendBuilderLog("CircleBuilder", OldCount);
            }
            {
                const std::size_t OldCount = ABaseCandidates.size();
                CetEllipseClusterBuilder Builder;
                Builder.BuildCandidates(AOriginalItems, AFeatures, AIndicesByType.at(MetShapeType::EllipseLike), AOptions, ABaseCandidates);
                AppendBuilderLog("EllipseBuilder", OldCount);
            }
            {
                const std::size_t OldCount = ABaseCandidates.size();
                CetRectangleClusterBuilder Builder;
                Builder.BuildCandidates(AOriginalItems, AFeatures, AIndicesByType.at(MetShapeType::RectangleLike), AOptions, ABaseCandidates);
                AppendBuilderLog("RectangleBuilder", OldCount);
            }
            {
                const std::size_t OldCount = ABaseCandidates.size();
                CetArcClusterBuilder Builder;
                Builder.BuildCandidates(AOriginalItems, AFeatures, AIndicesByType.at(MetShapeType::ArcLike), AOptions, ABaseCandidates);
                AppendBuilderLog("ArcBuilder", OldCount);
            }
            {
                std::vector<int> CustomIndices;
                const auto AppendCustomIndices = [&](MetShapeType AShapeType) {
                    auto It = AIndicesByType.find(AShapeType);
                    if (It != AIndicesByType.end()) {
                        const std::vector<int> &TypeIndices = It->second;
                        CustomIndices.insert(CustomIndices.end(), TypeIndices.begin(), TypeIndices.end());
                    }
                };
                AppendCustomIndices(MetShapeType::QuadrilateralLike);
                AppendCustomIndices(MetShapeType::ConvexPolygon);
                AppendCustomIndices(MetShapeType::ConcavePolygon);
                const std::size_t OldCount = ABaseCandidates.size();
                CetCustomClusterBuilder Builder;
                Builder.BuildCandidates(AOriginalItems, AFeatures, CustomIndices, AOptions, ABaseCandidates);
                AppendBuilderLog("CustomBuilder", OldCount);
            }
        }
        void CetClusterTemplateFillOptimizer::_BuildFilledTemplateCandidateVariants(const CetTNestItemVector &AOriginalItems, const std::vector<TetShapeFeature> &AFeatures, const TetNestOptions &AOptions, const std::vector<TetClusterCandidate> &ABaseCandidates, std::vector<TetClusterCandidate> &AOutCandidates)
        {
            AOutCandidates = ABaseCandidates;
            TetClusterFillSearchStats Stats;
            const TetClusterFillSearchConfig Config = GetClusterFillSearchConfig(AOriginalItems.size());
            std::size_t ValidBaseCandidateCount = 0;
            std::size_t FilledVariantCount = 0;
            const bool HasFixedEnvelopeBase = std::any_of(ABaseCandidates.begin(), ABaseCandidates.end(), [&](const TetClusterCandidate &Candidate) { return IsFixedCircleEnvelopeBase(Candidate, AFeatures) || IsFixedEllipseEnvelopeBase(Candidate, AFeatures); });
            for (const TetClusterCandidate &BaseCandidate : ABaseCandidates) {
                if (!BaseCandidate.Valid)
                    continue;
                ++ValidBaseCandidateCount;
                Stats.BaseFillRatioSum += BaseCandidate.FillRatio;
                if (HasFixedEnvelopeBase || SkipsGenericTemplateFill(BaseCandidate))
                    continue;
                std::vector<TetClusterCandidate> Variants;
                BuildFilledVariantsForBase({AOriginalItems, AFeatures, AOptions, BaseCandidate, BaseCandidate}, Config, Variants, Stats);
                if (!Variants.empty()) {
                    FilledVariantCount += Variants.size();
                    std::cout << "[TEMPLATE][FILL VARIANT] BaseType=" << BaseCandidate.ClusterType << " Generated=" << Variants.size() << " BaseFillRatio=" << BaseCandidate.FillRatio << " BestFillRatio=" << Variants.front().FillRatio << std::endl;
                    AOutCandidates.insert(AOutCandidates.end(), Variants.begin(), Variants.end());
                }
            }
            const TetClusterFillSearchConfig EnvelopeConfig = GetClusterEnvelopeFillSearchConfig(AOriginalItems.size());
            TetCircleGapTemplateCache CircleGapTemplateCache;
            TetEllipseGapTemplateCache EllipseGapTemplateCache;
            std::vector<const TetClusterCandidate *> EnvelopeBaseCandidates;
            EnvelopeBaseCandidates.reserve(ABaseCandidates.size());
            for (const TetClusterCandidate &BaseCandidate : ABaseCandidates) {
                if (IsFixedCircleEnvelopeBase(BaseCandidate, AFeatures) || IsFixedEllipseEnvelopeBase(BaseCandidate, AFeatures)) {
                    EnvelopeBaseCandidates.push_back(&BaseCandidate);
                }
            }
            std::stable_sort(EnvelopeBaseCandidates.begin(), EnvelopeBaseCandidates.end(), [](const TetClusterCandidate *AFirst, const TetClusterCandidate *ASecond) {
                const bool FirstIsEllipse = AFirst->BuilderName == "EllipseBuilder";
                const bool SecondIsEllipse = ASecond->BuilderName == "EllipseBuilder";
                if (FirstIsEllipse != SecondIsEllipse)
                    return FirstIsEllipse;
                const double FirstAvailableArea = std::max(0.0, AFirst->BoundingBoxArea - AFirst->ReservedArea);
                const double SecondAvailableArea = std::max(0.0, ASecond->BoundingBoxArea - ASecond->ReservedArea);
                if (std::abs(FirstAvailableArea - SecondAvailableArea) > 1.0)
                    return FirstAvailableArea > SecondAvailableArea;
                return AFirst->ClusterType < ASecond->ClusterType;
            });
            for (const TetClusterCandidate *BaseCandidate : EnvelopeBaseCandidates) {
                std::vector<TetClusterCandidate> Variants;
                std::map<std::string, TetCircleGapTemplate> &GapTemplates = CircleGapTemplateCache[BuildCircleGapTemplateCacheKey(*BaseCandidate)];
                BuildEnvelopeFilledVariantsForBase({AOriginalItems, AFeatures, AOptions, *BaseCandidate, EnvelopeConfig, GapTemplates, EllipseGapTemplateCache, Variants, Stats});
                if (Variants.empty())
                    continue;
                const TetClusterCandidate &BestVariant = Variants.front();
                std::cout << "[TEMPLATE][ENVELOPE FILL] BaseType=" << BaseCandidate->ClusterType << " Generated=" << Variants.size() << " BaseEnvelopeFill=" << BaseCandidate->BoundingFillRatio << " BestFillRatio=" << BestVariant.FillRatio << " EnvelopeFillGain=" << (BestVariant.FillRatio - BaseCandidate->BoundingFillRatio) << " ProxyMode=" << ToString(BestVariant.ProxyMode) << " ProxyArea=" << BestVariant.ProxyArea << " FillerCount=" << BestVariant.OriginalIndices.size() - BaseCandidate->OriginalIndices.size() << " Width=" << BestVariant.ClusterWidth << " Height=" << BestVariant.ClusterHeight << std::endl;
                AOutCandidates.insert(AOutCandidates.end(), Variants.begin(), Variants.end());
            }
            const double BaseAverage = ValidBaseCandidateCount == 0 ? 0.0 : Stats.BaseFillRatioSum / static_cast<double>(ValidBaseCandidateCount);
            const double FilledAverage = FilledVariantCount == 0 ? 0.0 : Stats.FilledFillRatioSum / static_cast<double>(FilledVariantCount);
            std::cout << "[TEMPLATE][FILL SUMMARY] BaseCandidateCount=" << ABaseCandidates.size() << " GeneratedVariantCount=" << Stats.GeneratedVariantCount << " DeduplicatedVariantCount=" << Stats.DeduplicatedVariantCount << " FilledVariantCount=" << FilledVariantCount << " AverageBaseFillRatio=" << BaseAverage << " AverageFilledFillRatio=" << FilledAverage << " BestFillRatioGain=" << Stats.BestFillRatioGain << " FreeRegionCount=" << Stats.FreeRegionCount << " SearchAttempts=" << Stats.SearchAttempts << " EnvelopeGeneratedVariantCount=" << Stats.EnvelopeGeneratedVariantCount << " EnvelopeDeduplicatedVariantCount=" << Stats.EnvelopeDeduplicatedVariantCount << " EnvelopeFreeRegionCount=" << Stats.EnvelopeFreeRegionCount << " EnvelopeSearchAttempts=" << Stats.EnvelopeSearchAttempts << " EnvelopeTimeLimitHits=" << Stats.EnvelopeTimeLimitHits << " EnvelopeMaxDepthReached=" << Stats.EnvelopeMaxDepthReached << " EnvelopeBestFillerCount=" << Stats.EnvelopeBestFillerCount << " EnvelopeSearchMs=" << Stats.EnvelopeSearchMs << " EnvelopeTrueContourMs=" << Stats.EnvelopeTrueContourMs << " BestEnvelopeFillRatioGain=" << Stats.BestEnvelopeFillRatioGain << " BestEnvelopeRectangleFillRatio=" << Stats.BestEnvelopeRectangleFillRatio << " EnvelopeBeamWidth=" << EnvelopeConfig.BeamWidth << " EnvelopeMaxDepth=" << EnvelopeConfig.MaxDepth << " EnvelopeMaxFillers=" << EnvelopeConfig.MaxCandidateFillers << " EnvelopeMaxPlacementAttempts=" << EnvelopeConfig.MaxPlacementAttempts << " BeamWidth=" << Config.BeamWidth << " MaxDepth=" << Config.MaxDepth << " MaxFillers=" << Config.MaxCandidateFillers << " MaxPlacementAttempts=" << Config.MaxPlacementAttempts << std::endl;
        }
        std::vector<TetClusterCandidate> CetClusterTemplateFillOptimizer::_SelectTemplateCandidates(const CetTNestItemVector &AOriginalItems, const std::vector<TetShapeFeature> &AFeatures, const TetNestOptions &AOptions, const std::vector<TetClusterCandidate> &ABaseCandidates, std::vector<bool> &AUsed)
        {
            const int Count = static_cast<int>(AOriginalItems.size());
            std::vector<TetClusterCandidate> SortedCandidates = ABaseCandidates;
            auto CountFillerFamilies = [&](const TetClusterCandidate &Candidate) {
                std::set<std::uint64_t> Families;
                const std::size_t Start = std::min(Candidate.SkeletonChildCount, Candidate.OriginalIndices.size());
                for (std::size_t Index = Start; Index < Candidate.OriginalIndices.size(); ++Index) {
                    const int OriginalIndex = Candidate.OriginalIndices[Index];
                    if (OriginalIndex >= 0 && OriginalIndex < static_cast<int>(AFeatures.size())) {
                        Families.insert(MakeFillerFamilyKey(AFeatures[OriginalIndex]));
                    }
                }
                return Families.size();
            };
            std::stable_sort(SortedCandidates.begin(), SortedCandidates.end(), [&](const TetClusterCandidate &A, const TetClusterCandidate &AB) {
                const bool ACompletedEnvelopeFill = IsCompletedEnvelopeFill(A);
                const bool BCompletedEnvelopeFill = IsCompletedEnvelopeFill(AB);
                if (ACompletedEnvelopeFill != BCompletedEnvelopeFill) {
                    return ACompletedEnvelopeFill;
                }
                if (ACompletedEnvelopeFill && BCompletedEnvelopeFill) {
                    const std::size_t AFillCount = A.OriginalIndices.size() - std::min(A.SkeletonChildCount, A.OriginalIndices.size());
                    const std::size_t BFillCount = AB.OriginalIndices.size() - std::min(AB.SkeletonChildCount, AB.OriginalIndices.size());
                    if (A.SkeletonChildCount == AB.SkeletonChildCount && AFillCount == BFillCount) {
                        const std::size_t AFamilyCount = CountFillerFamilies(A);
                        const std::size_t BFamilyCount = CountFillerFamilies(AB);
                        if (AFamilyCount != BFamilyCount)
                            return AFamilyCount > BFamilyCount;
                    }
                    return IsCompletedEnvelopeFillBetter(A, AB);
                }
                const bool ARegularSkeleton = IsInventorySkeletonCandidate(A);
                const bool BRegularSkeleton = IsInventorySkeletonCandidate(AB);
                if (ARegularSkeleton != BRegularSkeleton) {
                    return ARegularSkeleton;
                }
                const bool ACircleSkeleton = IsCircleSkeletonCandidate(A);
                const bool BCircleSkeleton = IsCircleSkeletonCandidate(AB);
                if (ACircleSkeleton != BCircleSkeleton) {
                    return ACircleSkeleton;
                }
                if (ARegularSkeleton && BRegularSkeleton && A.SkeletonChildCount != AB.SkeletonChildCount) {
                    return A.SkeletonChildCount > AB.SkeletonChildCount;
                }
                if (std::abs(A.Score - AB.Score) > 1e-9) {
                    return A.Score > AB.Score;
                }
                if (std::abs(A.SheetReuseScore - AB.SheetReuseScore) > 1e-9) {
                    return A.SheetReuseScore > AB.SheetReuseScore;
                }
                if (A.OriginalIndices.size() != AB.OriginalIndices.size()) {
                    return A.OriginalIndices.size() > AB.OriginalIndices.size();
                }
                if (std::abs(A.ProxyArea - AB.ProxyArea) > 1e-9) {
                    return A.ProxyArea < AB.ProxyArea;
                }
                return A.ClusterType < AB.ClusterType;
            });
            std::cout << "[TEMPLATE][BASE CANDIDATE TOTAL] " << SortedCandidates.size() << std::endl;
            std::vector<TetClusterCandidate> AcceptedCandidates;
            AcceptedCandidates.reserve(SortedCandidates.size());
            std::vector<TetClusterCandidate> DeferredTriangles;
            for (const TetClusterCandidate &Candidate : SortedCandidates) {
                if (IsDeferredTriangleCandidate(Candidate)) {
                    DeferredTriangles.push_back(Candidate);
                    continue;
                }
                TetClusterCandidate BoundCandidate;
                if (!TryBindCandidateInventory(Candidate, AFeatures, AUsed, BoundCandidate) || !_CanAcceptClusterCandidate(AOriginalItems, AOptions, BoundCandidate, AUsed, Count)) {
                    std::cout << "[TEMPLATE][REJECT] Builder=" << Candidate.BuilderName << " Type=" << Candidate.ClusterType << " Score=" << Candidate.Score << std::endl;
                    continue;
                }
                AcceptedCandidates.push_back(std::move(BoundCandidate));
                for (int OriginalIndex : AcceptedCandidates.back().OriginalIndices) {
                    AUsed[OriginalIndex] = true;
                }
                std::cout << "[TEMPLATE][BASE ACCEPT] Builder=" << Candidate.BuilderName << " Type=" << Candidate.ClusterType << " ChildCount=" << Candidate.OriginalIndices.size() << " Score=" << Candidate.Score << std::endl;
            }
            RebalanceAcceptedClusterInventory(AOriginalItems, AFeatures, AOptions, AcceptedCandidates, AUsed);
            const bool DeferRemainingTriangles = HasFilledEllipseCandidate(AcceptedCandidates);
            for (const TetClusterCandidate &Candidate : DeferredTriangles) {
                if (DeferRemainingTriangles) {
                    std::cout << "[TEMPLATE][DEFERRED TRIANGLE SINGLE] Type=" << Candidate.ClusterType << std::endl;
                    continue;
                }
                TetClusterCandidate BoundCandidate;
                if (!TryBindCandidateInventory(Candidate, AFeatures, AUsed, BoundCandidate) || !_CanAcceptClusterCandidate(AOriginalItems, AOptions, BoundCandidate, AUsed, Count)) {
                    std::cout << "[TEMPLATE][DEFERRED TRIANGLE REJECT] Type=" << Candidate.ClusterType << std::endl;
                    continue;
                }
                AcceptedCandidates.push_back(std::move(BoundCandidate));
                for (int OriginalIndex : AcceptedCandidates.back().OriginalIndices) {
                    AUsed[OriginalIndex] = true;
                }
                std::cout << "[TEMPLATE][DEFERRED TRIANGLE ACCEPT] Type=" << Candidate.ClusterType << " ChildCount=" << Candidate.OriginalIndices.size() << std::endl;
            }
            return AcceptedCandidates;
        }
        std::vector<TetClusterCandidate> CetClusterTemplateFillOptimizer::_SelectAndOptimizeTemplateCandidates(const CetTNestItemVector &AOriginalItems, const std::vector<TetShapeFeature> &AFeatures, const TetNestOptions &AOptions, const std::vector<TetClusterCandidate> &ABaseCandidates, std::vector<bool> &AUsed, int AOriginalItemCount)
        {
#ifdef _DEBUG
            const auto GreedyStartTime = std::chrono::steady_clock::now();
#endif
            std::vector<TetClusterCandidate> AcceptedCandidates = _SelectTemplateCandidates(AOriginalItems, AFeatures, AOptions, ABaseCandidates, AUsed);
#ifdef _DEBUG
            const auto GreedyEndTime = std::chrono::steady_clock::now();
            const std::vector<TetClusterCandidate> GreedyResult = AcceptedCandidates;
            const double GreedyScore = _CalculateCandidateSelectionScore(GreedyResult);
            const auto OptimizeStartTime = std::chrono::steady_clock::now();
#endif
            const int SwapCount = _OptimizePairClusterSelection(ABaseCandidates, AcceptedCandidates, AOriginalItemCount);
#ifndef _DEBUG
            (void)SwapCount;
#endif
#ifdef _DEBUG
            const auto OptimizeEndTime = std::chrono::steady_clock::now();
            const std::size_t OptimizedCandidateCount = AcceptedCandidates.size();
            const double OptimizedScore = _CalculateCandidateSelectionScore(AcceptedCandidates);
#endif
            std::fill(AUsed.begin(), AUsed.end(), false);
            for (const TetClusterCandidate &Candidate : AcceptedCandidates) {
                for (int OriginalIndex : Candidate.OriginalIndices) {
                    if (OriginalIndex >= 0 && OriginalIndex < AOriginalItemCount) {
                        AUsed[OriginalIndex] = true;
                    }
                }
            }
#ifdef _DEBUG
            const auto ToMilliseconds = [](const auto &AStart, const auto &AEnd) { return std::chrono::duration<double, std::milli>(AEnd - AStart).count(); };
            std::cout << "[ClusterSelection] Greedy Score = " << GreedyScore << ", Optimized Score = " << OptimizedScore << ", Improvement = " << OptimizedScore - GreedyScore << ", Swap Count = " << SwapCount << std::endl;
            std::cout << "[ClusterPerf] Candidates: " << ABaseCandidates.size() << ", AcceptedGreedy: " << GreedyResult.size() << ", AcceptedOptimized: " << OptimizedCandidateCount << ", SwapCount: " << SwapCount << ", GreedyMs: " << ToMilliseconds(GreedyStartTime, GreedyEndTime) << ", OptimizeMs: " << ToMilliseconds(OptimizeStartTime, OptimizeEndTime) << std::endl;
#endif
            return AcceptedCandidates;
        }
        int CetClusterTemplateFillOptimizer::_OptimizePairClusterSelection(const std::vector<TetClusterCandidate> &AAllCandidates, std::vector<TetClusterCandidate> &AAcceptedCandidates, int AOriginalItemCount)
        {
            if (AAcceptedCandidates.size() < 2 || AOriginalItemCount <= 0) {
                return 0;
            }
            TPairCandidateLookup PairCandidateLookup;
            BuildPairCandidateLookup(AAllCandidates, AOriginalItemCount, PairCandidateLookup);
            if (PairCandidateLookup.empty()) {
                return 0;
            }
            const std::vector<TetClusterCandidate> GreedyResult = AAcceptedCandidates;
            const double GreedyScore = _CalculateCandidateSelectionScore(GreedyResult);
            if (!_ValidateClusterSelection(GreedyResult, AOriginalItemCount)) {
                return 0;
            }
            int SwapCount = 0;
            for (int Round = 0; Round < CET_CLUSTER_FILL_MAX_SWAP_ROUNDS; ++Round) {
                const std::vector<std::size_t> PairPositions = CollectPairCandidatePositions(AAcceptedCandidates);
                if (PairPositions.size() < 2) {
                    break;
                }
                bool Changed = false;
                for (std::size_t FirstPairIndex = 0; FirstPairIndex + 1 < PairPositions.size(); ++FirstPairIndex) {
                    for (std::size_t SecondPairIndex = FirstPairIndex + 1; SecondPairIndex < PairPositions.size(); ++SecondPairIndex) {
                        const std::size_t FirstPosition = PairPositions[FirstPairIndex];
                        const std::size_t SecondPosition = PairPositions[SecondPairIndex];
                        const TetClusterCandidate *FirstReplacement = nullptr;
                        const TetClusterCandidate *SecondReplacement = nullptr;
                        if (!TryFindBetterPairSwap(PairCandidateLookup, AAcceptedCandidates[FirstPosition], AAcceptedCandidates[SecondPosition], FirstReplacement, SecondReplacement)) {
                            continue;
                        }
                        std::vector<TetClusterCandidate> TrialSelection = AAcceptedCandidates;
                        TrialSelection[FirstPosition] = *FirstReplacement;
                        TrialSelection[SecondPosition] = *SecondReplacement;
                        if (!_ValidateClusterSelection(TrialSelection, AOriginalItemCount)) {
                            continue;
                        }
                        AAcceptedCandidates = std::move(TrialSelection);
                        ++SwapCount;
                        Changed = true;
                    }
                }
                if (!Changed) {
                    break;
                }
            }
            const bool SelectionValid = _ValidateClusterSelection(AAcceptedCandidates, AOriginalItemCount);
            const double OptimizedScore = _CalculateCandidateSelectionScore(AAcceptedCandidates);
            if (!SelectionValid) {
                std::cout << "[ClusterSelection][ROLLBACK] Invalid pair-swap selection." << std::endl;
                AAcceptedCandidates = GreedyResult;
                return 0;
            }
            const double GainRatio = (OptimizedScore - GreedyScore) / std::max(std::abs(GreedyScore), 1.0);
            if (SwapCount > 0 && GainRatio < CET_CLUSTER_FILL_MIN_SWAP_GAIN_RATIO) {
                AAcceptedCandidates = GreedyResult;
                return 0;
            }
            return SwapCount;
        }
        double CetClusterTemplateFillOptimizer::_CalculateCandidateSelectionScore(const std::vector<TetClusterCandidate> &ACandidates)
        {
            double TotalScore = 0.0;
            for (const TetClusterCandidate &Candidate : ACandidates) {
                TotalScore += Candidate.Score;
            }
            return TotalScore;
        }
        bool CetClusterTemplateFillOptimizer::_ValidateClusterSelection(const std::vector<TetClusterCandidate> &ACandidates, int AOriginalItemCount)
        {
            if (AOriginalItemCount < 0) {
                return false;
            }
            std::vector<bool> Used(static_cast<std::size_t>(AOriginalItemCount), false);
            for (const TetClusterCandidate &Candidate : ACandidates) {
                if (!Candidate.Valid || Candidate.OriginalIndices.empty() || Candidate.OriginalIndices.size() != Candidate.Transforms.size() || !std::isfinite(Candidate.Score)) {
                    return false;
                }
                std::set<int> CandidateIds;
                std::set<int> TransformIds;
                for (int OriginalIndex : Candidate.OriginalIndices) {
                    if (OriginalIndex < 0 || OriginalIndex >= AOriginalItemCount || Used[OriginalIndex] || !CandidateIds.insert(OriginalIndex).second) {
                        return false;
                    }
                    Used[OriginalIndex] = true;
                }
                for (const TetItemTransform &Transform : Candidate.Transforms) {
                    if (Transform.OriginalId < 0 || Transform.OriginalId >= AOriginalItemCount || !std::isfinite(Transform.RelativeX) || !std::isfinite(Transform.RelativeY) || !std::isfinite(Transform.RelativeRotation) || !TransformIds.insert(Transform.OriginalId).second) {
                        return false;
                    }
                }
                if (CandidateIds != TransformIds) {
                    return false;
                }
            }
            return true;
        }


        bool CetClusterTemplateFillOptimizer::_CanAcceptClusterCandidate(const CetTNestItemVector &AOriginalItems, const TetNestOptions &AOptions, const TetClusterCandidate &ACandidate, const std::vector<bool> &AUsed, int AOriginalCount)
        {
            if (AOriginalCount < 0 || AOriginalItems.size() != static_cast<std::size_t>(AOriginalCount)) return false;
            if (!ACandidate.Valid || ACandidate.OriginalIndices.empty() || ACandidate.OriginalIndices.size() != ACandidate.Transforms.size() || ACandidate.ProxyContour.size() < 3) return false;
            if (ACandidate.ClusterWidth <= 0.0 || ACandidate.ClusterHeight <= 0.0 || ACandidate.ProxyArea <= 0.0) return false;
            std::set<int> CandidateIds;
            std::set<int> TransformIds;
            for (int OriginalIndex : ACandidate.OriginalIndices) {
                if (OriginalIndex < 0 || OriginalIndex >= AOriginalCount || OriginalIndex >= static_cast<int>(AUsed.size()) || AUsed[OriginalIndex] || !CandidateIds.insert(OriginalIndex).second) return false;
            }
            for (const TetItemTransform &Transform : ACandidate.Transforms) {
                if (Transform.OriginalId < 0 || Transform.OriginalId >= AOriginalCount || !std::isfinite(Transform.RelativeX) || !std::isfinite(Transform.RelativeY) || !std::isfinite(Transform.RelativeRotation) || !TransformIds.insert(Transform.OriginalId).second) return false;
            }
            return CandidateIds == TransformIds;
        }

    }
}
