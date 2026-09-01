#include "pch.h"
#include "Nest2D_ClusterInventoryRebalancer.h"
#include "Nest2D_ClusterGeometryHelper.h"
#include "Nest2D_RectangleFillClusterBuilder.h"
#include "Nest2D_TriangleClusterBuilder.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <set>
#include <utility>

using namespace ClipperLib;
using namespace libnest2d;

namespace {
    bool CircleGapSearchTimeReached(const std::chrono::steady_clock::time_point &AStart, long long ALimitMs)
    {
        return ALimitMs > 0 && std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - AStart).count() >= ALimitMs;
    }
    bool IsCircleSkeletonCandidate(const TetClusterCandidate &ACandidate)
    {
        return ACandidate.SkeletonChildCount >= 2 && (ACandidate.BuilderName == "CircleBuilder" || ACandidate.ClusterType.find("Circle") == 0);
    }
    bool IsEllipseSkeletonCandidate(const TetClusterCandidate &ACandidate)
    {
        return ACandidate.SkeletonChildCount >= 2 && (ACandidate.BuilderName == "EllipseBuilder" || ACandidate.ClusterType.find("Ellipse") == 0);
    }
    bool IsInventorySkeletonCandidate(const TetClusterCandidate &ACandidate)
    {
        return IsCircleSkeletonCandidate(ACandidate) || IsEllipseSkeletonCandidate(ACandidate);
    }
    /* Legacy helper moved to CetClusterGeometryHelper.
    bool HasFullRectangleProxy(const TetClusterCandidate &ACandidate)
    {
        const double BoundingArea = ACandidate.ClusterWidth * ACandidate.ClusterHeight;
        const double AreaTolerance = std::max(1.0, BoundingArea * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE);
        return ACandidate.Valid && ACandidate.ClusterWidth > 0.0 && ACandidate.ClusterHeight > 0.0 && std::isfinite(BoundingArea) && std::abs(ACandidate.ProxyArea - BoundingArea) <= AreaTolerance;
    }
    */
    bool BuildRectangleEnvelopeCandidate(const CetTNestItemVector &AOriginalItems, const TetNestOptions &AOptions, const TetClusterCandidate &ABaseCandidate, TetClusterCandidate &AOutEnvelopeCandidate)
    {
        AOutEnvelopeCandidate = TetClusterCandidate{};
        if (!ABaseCandidate.Valid || ABaseCandidate.OriginalIndices.size() < 2 || ET::NEST2DMANAGERLIB::CetClusterGeometryHelper::HasFullRectangleProxy(ABaseCandidate))
            return false;
        AOutEnvelopeCandidate = ABaseCandidate;
        ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
        if (!Geometry.FinalizeCandidateInRectangle(AOriginalItems, AOptions, AOutEnvelopeCandidate, ABaseCandidate.ClusterWidth, ABaseCandidate.ClusterHeight))
            return false;
        AOutEnvelopeCandidate.BuilderName = ABaseCandidate.BuilderName;
        AOutEnvelopeCandidate.ClusterType = ABaseCandidate.ClusterType;
        return AOutEnvelopeCandidate.ProxyArea > ABaseCandidate.ProxyArea + std::max(1.0, ABaseCandidate.ProxyArea * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE);
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
}

namespace ET { namespace NEST2DMANAGERLIB {
    CetClusterInventoryRebalancer::CetClusterInventoryRebalancer() : CetCoreObject() {}
    CetClusterInventoryRebalancer::~CetClusterInventoryRebalancer() {}

    bool CetClusterInventoryRebalancer::_HasValidCandidateInventory(const TetClusterCandidate &ACandidate, const std::vector<TetShapeFeature> &AFeatures, const std::vector<bool> &AUsed) { return ACandidate.SkeletonChildCount <= ACandidate.Transforms.size() && AFeatures.size() == AUsed.size() && ACandidate.OriginalIndices.size() == ACandidate.Transforms.size(); }
    int CetClusterInventoryRebalancer::_FindAvailableFamilyItem(const std::vector<TetShapeFeature> &AFeatures, const std::vector<bool> &AUsed, const std::set<int> &AReserved, int APrototypeId)
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
    bool CetClusterInventoryRebalancer::TryBindCandidateInventory(const TetClusterCandidate &ACandidate, const std::vector<TetShapeFeature> &AFeatures, const std::vector<bool> &AUsed, TetClusterCandidate &AOutCandidate)
    {
        AOutCandidate = ACandidate;
        if (!_HasValidCandidateInventory(ACandidate, AFeatures, AUsed))
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
            const int BoundId = _FindAvailableFamilyItem(AFeatures, AUsed, Reserved, PrototypeId);
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
    /* Legacy helper moved to CetClusterGeometryHelper.
    bool CetClusterInventoryRebalancer::FitsAnyFreeRegion(const TetShapeFeature &AFeature, const std::vector<TetClusterFreeRegion> &AFreeRegions)
    {
        for (const TetClusterFreeRegion &Region : AFreeRegions) {
            const bool FitsNormal = AFeature.Width <= Region.Width && AFeature.Height <= Region.Height;
            const bool FitsRotated = AFeature.Height <= Region.Width && AFeature.Width <= Region.Height;
            if (AFeature.Area <= Region.Area && (FitsNormal || FitsRotated))
                return true;
        }
        return false;
    }
    */
    bool CetClusterInventoryRebalancer::_IsInventoryRebalanceCandidate(const TetClusterCandidate &ACandidate)
    {
        return ACandidate.Valid &&
               ACandidate.SkeletonChildCount >= 2
               // Inventory rebalance preserves the existing proxy.  It is therefore
               // valid only for candidates that were already finalized as a fixed
               // envelope-fill virtual board; a raw skeleton may have an irregular
               // proxy whose outer boundary changes when a child is appended.
               && (ACandidate.BuilderName == "EnvelopeFillSearch" || ACandidate.BuilderName == "GlobalInventoryRebalance" || (IsInventorySkeletonCandidate(ACandidate) && CetClusterGeometryHelper::HasFullRectangleProxy(ACandidate))) && ACandidate.SkeletonChildCount <= ACandidate.Transforms.size() && ACandidate.OriginalIndices.size() == ACandidate.Transforms.size() && ACandidate.ProxyContour.size() >= 3 && ACandidate.ProxyArea > 0.0;
    }
    bool CetClusterInventoryRebalancer::_IsRebalanceOrderBetter(const TetClusterCandidate &AFirst, const TetClusterCandidate &ASecond)
    {
        if (AFirst.SkeletonChildCount != ASecond.SkeletonChildCount) {
            return AFirst.SkeletonChildCount > ASecond.SkeletonChildCount;
        }
        if (std::abs(AFirst.ProxyWasteArea - ASecond.ProxyWasteArea) > 1.0) {
            return AFirst.ProxyWasteArea > ASecond.ProxyWasteArea;
        }
        return AFirst.ClusterType < ASecond.ClusterType;
    }
    void CetClusterInventoryRebalancer::_PreserveInventoryProxy(const TetClusterCandidate &AProxySource, TetClusterCandidate &AInOutCandidate)
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
    bool CetClusterInventoryRebalancer::_TryAppendInventoryFiller(const CetTNestItemVector &AOriginalItems, const std::vector<TetShapeFeature> &AFeatures, const TetNestOptions &AOptions, const TetClusterCandidate &ACurrentCandidate, int AFillerIndex, TetClusterCandidate &AOutCandidate)
    {
        AOutCandidate = TetClusterCandidate{};
        ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
        std::vector<TetClusterFreeRegion> FreeRegions;
        if (!Geometry.ExtractCandidateFreeRegions(AOriginalItems, AOptions, ACurrentCandidate, FreeRegions)) {
            std::cout << "[TEMPLATE][GLOBAL FILL REJECT] Filler=" << AFillerIndex << " Reason=FreeRegionExtract" << std::endl;
            return false;
        }
        if (!CetClusterGeometryHelper::FitsAnyFreeRegion(AFeatures[AFillerIndex], FreeRegions)) {
            std::cout << "[TEMPLATE][GLOBAL FILL REJECT] Filler=" << AFillerIndex << " Reason=FreeRegionBounds RegionCount=" << FreeRegions.size() << std::endl;
            return false;
        }
        ET::NEST2DMANAGERLIB::CetRectangleFillClusterBuilder Builder;
        if (!Builder.TryAppendFillerInFreeRegions({AOriginalItems, AFeatures, AOptions, ACurrentCandidate, ACurrentCandidate, ACurrentCandidate, &FreeRegions, AFillerIndex, ACurrentCandidate.ClusterWidth, ACurrentCandidate.ClusterHeight}, AOutCandidate)) {
            std::cout << "[TEMPLATE][GLOBAL FILL REJECT] Filler=" << AFillerIndex << " Reason=ContourOrSpacing" << std::endl;
            return false;
        }
        _PreserveInventoryProxy(ACurrentCandidate, AOutCandidate);
        AOutCandidate.BuilderName = "GlobalInventoryRebalance";
        AOutCandidate.ClusterType = ACurrentCandidate.ClusterType;
        return true;
    }
    bool CetClusterInventoryRebalancer::_TryRemoveInventoryFiller(const CetTNestItemVector &AOriginalItems, const TetNestOptions &AOptions, const TetClusterCandidate &ACandidate, std::size_t ATransformIndex, TetClusterCandidate &AOutCandidate)
    {
        AOutCandidate = TetClusterCandidate{};
        if (!_IsInventoryRebalanceCandidate(ACandidate) || ATransformIndex < ACandidate.SkeletonChildCount || ATransformIndex >= ACandidate.Transforms.size())
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
        _PreserveInventoryProxy(ACandidate, AOutCandidate);
        AOutCandidate.BuilderName = "GlobalInventoryRebalance";
        AOutCandidate.ClusterType = ACandidate.ClusterType;
        return true;
    }
    bool CetClusterInventoryRebalancer::_IsTriangleBuilderCandidate(const TetClusterCandidate &ACandidate) { return ACandidate.Valid && ACandidate.BuilderName == "TriangleBuilder" && ACandidate.OriginalIndices.size() == ACandidate.Transforms.size() && ACandidate.OriginalIndices.size() >= 4; }
    bool CetClusterInventoryRebalancer::_HasExactCandidateInventory(const TetClusterCandidate &ACandidate, const std::vector<int> &AExpectedIndices)
    {
        if (!ACandidate.Valid || ACandidate.OriginalIndices.size() != AExpectedIndices.size() || ACandidate.Transforms.size() != AExpectedIndices.size())
            return false;
        std::set<int> Actual(ACandidate.OriginalIndices.begin(), ACandidate.OriginalIndices.end());
        std::set<int> Expected(AExpectedIndices.begin(), AExpectedIndices.end());
        return Actual == Expected && Actual.size() == AExpectedIndices.size();
    }
    bool CetClusterInventoryRebalancer::_TryBuildReducedTriangleCandidate(const CetTNestItemVector &AOriginalItems, const std::vector<TetShapeFeature> &AFeatures, const TetNestOptions &AOptions, const TetClusterCandidate &ASourceCandidate, const std::set<int> &ARemovedIds, TetClusterCandidate &AOutCandidate)
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
            if (_HasExactCandidateInventory(Candidate, Remaining)) {
                AOutCandidate = Candidate;
                _PreserveInventoryProxy(ASourceCandidate, AOutCandidate);
                AOutCandidate.BuilderName = "GlobalInventoryRebalance";
                AOutCandidate.ClusterType = ASourceCandidate.ClusterType + "_Reduced";
                return true;
            }
        }
        return false;
    }
    bool CetClusterInventoryRebalancer::_TryTransferTrianglePair(const TetClusterFillContext &AContext, TetClusterCandidate &AOutTarget, TetClusterCandidate &AOutSource, std::pair<int, int> &AOutPair)
    {
        const auto &AOriginalItems = AContext.OriginalItems; const auto &AFeatures = AContext.Features; const auto &AOptions = AContext.Options; const auto &ATargetBefore = AContext.BaseCandidate; const auto &ASourceBefore = AContext.EnvelopeCandidate;
        const std::size_t ItemCount = ASourceBefore.OriginalIndices.size();
        for (std::size_t First = ItemCount - 2;; --First) {
            for (std::size_t Second = ItemCount - 1; Second > First; --Second) {
                const int FirstId = ASourceBefore.OriginalIndices[First];
                const int SecondId = ASourceBefore.OriginalIndices[Second];
                TetClusterCandidate ExpandedTarget;
                if (!_TryAppendInventoryFiller(AOriginalItems, AFeatures, AOptions, ATargetBefore, FirstId, ExpandedTarget)) {
                    continue;
                }
                TetClusterCandidate ExpandedTargetPair;
                if (!_TryAppendInventoryFiller(AOriginalItems, AFeatures, AOptions, ExpandedTarget, SecondId, ExpandedTargetPair)) {
                    std::cout << "[TEMPLATE][TRIANGLE TRANSFER PROBE] Pair=" << FirstId << "," << SecondId << " SecondAppend=reject" << std::endl;
                    continue;
                }
                const std::set<int> RemovedIds = {FirstId, SecondId};
                TetClusterCandidate ReducedSource;
                if (!_TryBuildReducedTriangleCandidate(AOriginalItems, AFeatures, AOptions, ASourceBefore, RemovedIds, ReducedSource)) {
                    std::cout << "[TEMPLATE][TRIANGLE TRANSFER PROBE] Pair=" << FirstId << "," << SecondId << " ReducedSource=reject" << std::endl;
                    continue;
                }
                if (!_IsTriangleTransferWorthKeeping(ATargetBefore, ExpandedTargetPair, ASourceBefore, ReducedSource)) {
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
    bool CetClusterInventoryRebalancer::_IsTransferPairGloballyUnique(const std::vector<TetClusterCandidate> &AAcceptedCandidates, std::size_t ASourceIndex, const std::pair<int, int> &APair)
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
    bool CetClusterInventoryRebalancer::_IsTriangleTransferWorthKeeping(const TetClusterCandidate &ATargetBefore, const TetClusterCandidate &ATargetAfter, const TetClusterCandidate &ASourceBefore, const TetClusterCandidate &ASourceAfter)
    {
        if (ATargetAfter.RealArea <= ATargetBefore.RealArea)
            return false;
        const double TotalBefore = ATargetBefore.ProxyWasteArea + ASourceBefore.ProxyWasteArea;
        const double TotalAfter = ATargetAfter.ProxyWasteArea + ASourceAfter.ProxyWasteArea;
        const double Tolerance = std::max(1.0, ATargetBefore.ProxyArea * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE);
        std::cout << "[TEMPLATE][TRIANGLE TRANSFER METRIC] TargetArea=" << ATargetBefore.RealArea << "->" << ATargetAfter.RealArea << " Waste=" << TotalBefore << "->" << TotalAfter << std::endl;
        return TotalAfter <= TotalBefore + Tolerance;
    }
    bool CetClusterInventoryRebalancer::_IsInventoryTransferWorthKeeping(const TetClusterCandidate &ATargetBefore, const TetClusterCandidate &ATargetAfter, const TetClusterCandidate &ASourceBefore, const TetClusterCandidate &ASourceAfter)
    {
        const double TargetTolerance = std::max(1.0, ATargetBefore.ProxyArea * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE);
        const double TotalBefore = ATargetBefore.ProxyWasteArea + ASourceBefore.ProxyWasteArea;
        const double TotalAfter = ATargetAfter.ProxyWasteArea + ASourceAfter.ProxyWasteArea;
        return ATargetAfter.ProxyWasteArea < ATargetBefore.ProxyWasteArea - TargetTolerance && TotalAfter <= TotalBefore + TargetTolerance;
    }
    bool CetClusterInventoryRebalancer::_TryBuildInventorySkeletonEnvelope(const CetTNestItemVector &AOriginalItems, const TetNestOptions &AOptions, const TetClusterCandidate &ACandidate, TetClusterCandidate &AOutCandidate)
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
        void CetClusterInventoryRebalancer::RebalanceAcceptedClusterInventory(const CetTNestItemVector &AOriginalItems, const std::vector<TetShapeFeature> &AFeatures, const TetNestOptions &AOptions, std::vector<TetClusterCandidate> &AAcceptedCandidates, std::vector<bool> &AUsed)
    {
        const auto Start = std::chrono::steady_clock::now();
        std::vector<std::size_t> Targets;
        for (std::size_t Index = 0; Index < AAcceptedCandidates.size(); ++Index) {
        if (_IsInventoryRebalanceCandidate(AAcceptedCandidates[Index]) || IsInventorySkeletonCandidate(AAcceptedCandidates[Index]))
                Targets.push_back(Index);
        }
        std::stable_sort(Targets.begin(), Targets.end(), [&](std::size_t A, std::size_t B) { return _IsRebalanceOrderBetter(AAcceptedCandidates[A], AAcceptedCandidates[B]); });
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
            if (!_IsInventoryRebalanceCandidate(TargetCandidate)) {
                TetClusterCandidate EnvelopeCandidate;
                if (!_TryBuildInventorySkeletonEnvelope(AOriginalItems, AOptions, TargetCandidate, EnvelopeCandidate))
                    continue;
                TargetCandidate = std::move(EnvelopeCandidate);
            }
            for (int FillerIndex : Available) {
                if (Attempts++ >= CET_CLUSTER_GLOBAL_REBALANCE_MAX_ATTEMPTS)
                    break;
                TetClusterCandidate Expanded;
                if (!_TryAppendInventoryFiller(AOriginalItems, AFeatures, AOptions, TargetCandidate, FillerIndex, Expanded))
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
            if (!_IsInventoryRebalanceCandidate(AAcceptedCandidates[TargetIndex]))
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
                    if (!_TryRemoveInventoryFiller(AOriginalItems, AOptions, AAcceptedCandidates[SourceIndex], FillerIndex, ReducedSource) || !_TryAppendInventoryFiller(AOriginalItems, AFeatures, AOptions, AAcceptedCandidates[TargetIndex], OriginalId, ExpandedTarget) || !_IsInventoryTransferWorthKeeping(AAcceptedCandidates[TargetIndex], ExpandedTarget, AAcceptedCandidates[SourceIndex], ReducedSource))
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
            if (!_IsInventoryRebalanceCandidate(AAcceptedCandidates[TargetIndex]))
                continue;
            for (std::size_t SourceIndex = 0; SourceIndex < AAcceptedCandidates.size(); ++SourceIndex) {
                if (TargetIndex == SourceIndex || !_IsTriangleBuilderCandidate(AAcceptedCandidates[SourceIndex]))
                    continue;
                TetClusterCandidate ExpandedTarget;
                TetClusterCandidate ReducedSource;
                std::pair<int, int> MovedPair = {-1, -1};
                if (!_TryTransferTrianglePair({AOriginalItems, AFeatures, AOptions, AAcceptedCandidates[TargetIndex], AAcceptedCandidates[SourceIndex]}, ExpandedTarget, ReducedSource, MovedPair))
                    continue;
                if (!_IsTransferPairGloballyUnique(AAcceptedCandidates, SourceIndex, MovedPair)) {
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

}}
