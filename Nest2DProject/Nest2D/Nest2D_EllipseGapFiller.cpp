#include "pch.h"
#include "Nest2D_EllipseGapFiller.h"
#include "Nest2D_ClusterGeometryHelper.h"
#include "Nest2D_RectangleFillClusterBuilder.h"
#include "Nest2D_RotationUtils.h"
#include "Nest2D_SelfFunction.h"
#include "NestUtils.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>

using namespace ClipperLib;
using namespace libnest2d;

namespace ET { namespace NEST2DMANAGERLIB {
namespace {
bool AppendLocalFreeRegion(const ClipperLib::PolyNode &ANode, TetClusterFreeRegion &AOutRegion)
{
    if (ANode.IsHole() || ANode.Contour.size() < 3)
        return false;
    AOutRegion = TetClusterFreeRegion{};
    AOutRegion.Contour = ANode.Contour;
    AOutRegion.IsClosed = true;
    AOutRegion.Area = std::abs(static_cast<double>(ClipperLib::Area(ANode.Contour)));
    ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
    if (AOutRegion.Area <= 0.0 || !Geometry.GetBounds(AOutRegion.Contour, AOutRegion.MinX, AOutRegion.MinY, AOutRegion.MaxX, AOutRegion.MaxY))
        return false;
    for (const ClipperLib::PolyNode *Child : ANode.Childs) {
        if (Child == nullptr || !Child->IsHole() || Child->Contour.size() < 3)
            continue;
        AOutRegion.Holes.push_back(Child->Contour);
        AOutRegion.Area -= std::abs(static_cast<double>(ClipperLib::Area(Child->Contour)));
    }
    AOutRegion.Width = AOutRegion.MaxX - AOutRegion.MinX;
    AOutRegion.Height = AOutRegion.MaxY - AOutRegion.MinY;
    return AOutRegion.Area > 0.0;
}
bool ClipFreeRegionsToGapWindow(const std::vector<TetClusterFreeRegion> &AFreeRegions, const TetCircleGapWindow &AWindow, std::vector<TetClusterFreeRegion> &AOutRegions)
{
    AOutRegions.clear();
    const double MinX = AWindow.CenterX - AWindow.HalfWidth, MinY = AWindow.CenterY - AWindow.HalfHeight;
    const double MaxX = AWindow.CenterX + AWindow.HalfWidth, MaxY = AWindow.CenterY + AWindow.HalfHeight;
    if (MaxX <= MinX || MaxY <= MinY)
        return false;
    const CetPath Window{{static_cast<ClipperLib::cInt>(std::llround(MinX)), static_cast<ClipperLib::cInt>(std::llround(MinY))}, {static_cast<ClipperLib::cInt>(std::llround(MaxX)), static_cast<ClipperLib::cInt>(std::llround(MinY))}, {static_cast<ClipperLib::cInt>(std::llround(MaxX)), static_cast<ClipperLib::cInt>(std::llround(MaxY))}, {static_cast<ClipperLib::cInt>(std::llround(MinX)), static_cast<ClipperLib::cInt>(std::llround(MaxY))}};
    for (const TetClusterFreeRegion &Region : AFreeRegions) {
        ClipperLib::Clipper Clipper;
        if (!Region.IsClosed || !Clipper.AddPath(Region.Contour, ClipperLib::ptSubject, true) || (!Region.Holes.empty() && !Clipper.AddPaths(Region.Holes, ClipperLib::ptSubject, true)) || !Clipper.AddPath(Window, ClipperLib::ptClip, true))
            continue;
        ClipperLib::PolyTree Tree;
        if (!Clipper.Execute(ClipperLib::ctIntersection, Tree, ClipperLib::pftNonZero, ClipperLib::pftNonZero))
            continue;
        for (const ClipperLib::PolyNode *Node : Tree.Childs) {
            TetClusterFreeRegion Local;
            if (Node != nullptr && AppendLocalFreeRegion(*Node, Local))
                AOutRegions.push_back(std::move(Local));
        }
    }
    return !AOutRegions.empty();
}
bool BuildEllipseGapRegionSignature(const CetTNestItemVector &AOriginalItems, const TetNestOptions &AOptions, const TetClusterCandidate &ACandidate, const TetCircleGapWindow &AWindow, std::string &AOutSignature)
{
    AOutSignature.clear();
    ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
    std::vector<TetClusterFreeRegion> FreeRegions, LocalRegions;
    if (!Geometry.ExtractCandidateFreeRegions(AOriginalItems, AOptions, ACandidate, FreeRegions) || !ClipFreeRegionsToGapWindow(FreeRegions, AWindow, LocalRegions))
        return false;
    std::stable_sort(LocalRegions.begin(), LocalRegions.end(), [](const TetClusterFreeRegion &A, const TetClusterFreeRegion &B) {
        if (std::abs(A.Area - B.Area) > 1.0)
            return A.Area > B.Area;
        if (std::abs(A.Width - B.Width) > 1.0)
            return A.Width > B.Width;
        return A.Height > B.Height;
    });
    std::ostringstream Stream;
    Stream << AWindow.ClassKey;
    for (const TetClusterFreeRegion &Region : LocalRegions)
        Stream << '|' << std::llround(Region.Area / 1000.0) << ':' << std::llround(Region.Width / 1000.0) << ':' << std::llround(Region.Height / 1000.0) << ':' << Region.Contour.size() << ':' << Region.Holes.size();
    AOutSignature = Stream.str();
    return true;
}
long long QuantizeCircleGapValue(double AValue) { return std::llround(std::max(0.0, AValue) / CET_RECTANGLE_FILL_POSITION_TOLERANCE); }
std::string BuildCircleGapClassKey(const char *AKind, double AHalfWidth, double AHalfHeight, double AScale)
{
    long long FirstSize = QuantizeCircleGapValue(AHalfWidth);
    long long SecondSize = QuantizeCircleGapValue(AHalfHeight);
    if (SecondSize < FirstSize)
        std::swap(FirstSize, SecondSize);
    return std::string(AKind) + "-" + std::to_string(FirstSize) + "-" + std::to_string(SecondSize) + "-" + std::to_string(QuantizeCircleGapValue(AScale));
}
int GetCircleGapPriority(const std::string &AClassKey)
{
    if (AClassKey.rfind("quad-", 0) == 0)
        return 0;
    if (AClassKey.rfind("triple-", 0) == 0)
        return 1;
    if (AClassKey.rfind("pair-", 0) == 0)
        return 2;
    return 3;
}
bool CircleGapSearchTimeReached(const std::chrono::steady_clock::time_point &AStart, long long ALimitMs)
{
    return ALimitMs > 0 && std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - AStart).count() >= ALimitMs;
}
double FindNearestCircleDistance(const std::vector<TetCircleCenter> &ACenters)
{
    double Nearest = std::numeric_limits<double>::infinity();
    for (std::size_t First = 0; First < ACenters.size(); ++First)
        for (std::size_t Second = First + 1; Second < ACenters.size(); ++Second)
            Nearest = std::min(Nearest, std::hypot(ACenters[Second].X - ACenters[First].X, ACenters[Second].Y - ACenters[First].Y));
    return Nearest;
}
std::vector<std::vector<std::size_t>> BuildCircleNeighborLists(const std::vector<TetCircleCenter> &ACenters, double ALimit)
{
    std::vector<std::vector<std::pair<double, std::size_t>>> Ranked(ACenters.size());
    for (std::size_t First = 0; First < ACenters.size(); ++First) {
        for (std::size_t Second = First + 1; Second < ACenters.size(); ++Second) {
            const double Distance = std::hypot(ACenters[Second].X - ACenters[First].X, ACenters[Second].Y - ACenters[First].Y);
            if (Distance <= CET_RECTANGLE_FILL_POSITION_TOLERANCE || Distance > ALimit)
                continue;
            Ranked[First].push_back({Distance, Second});
            Ranked[Second].push_back({Distance, First});
        }
    }
    std::vector<std::vector<std::size_t>> Neighbors(ACenters.size());
    for (std::size_t Index = 0; Index < Ranked.size(); ++Index) {
        std::stable_sort(Ranked[Index].begin(), Ranked[Index].end());
        if (Ranked[Index].size() > CET_CIRCLE_GAP_MAX_NEIGHBORS)
            Ranked[Index].resize(CET_CIRCLE_GAP_MAX_NEIGHBORS);
        for (const auto &Entry : Ranked[Index])
            Neighbors[Index].push_back(Entry.second);
    }
    return Neighbors;
}
bool AreCircleNeighbors(const std::vector<std::vector<std::size_t>> &ANeighbors, std::size_t AFirst, std::size_t ASecond)
{
    if (AFirst >= ANeighbors.size() || ASecond >= ANeighbors.size())
        return false;
    const auto Contains = [&](std::size_t AIndex, std::size_t ATarget) { return std::find(ANeighbors[AIndex].begin(), ANeighbors[AIndex].end(), ATarget) != ANeighbors[AIndex].end(); };
    return Contains(AFirst, ASecond) && Contains(ASecond, AFirst);
}
bool ContainsOriginalIndex(const TetClusterCandidate &ACandidate, int AOriginalIndex) { return std::find(ACandidate.OriginalIndices.begin(), ACandidate.OriginalIndices.end(), AOriginalIndex) != ACandidate.OriginalIndices.end(); }
bool IsFillMetricLess(double ALeft, double ARight) { return ALeft < ARight - CET_CLUSTER_FILL_VARIANT_POSITION_TOLERANCE; }
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
/* Legacy geometry helper moved to CetClusterGeometryHelper.
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
*/
std::vector<int> CollectCompatibleFillers(const std::vector<TetShapeFeature> &AFeatures, const TetClusterCandidate &ABaseCandidate, const std::vector<TetClusterFreeRegion> &AFreeRegions, const TetClusterFillSearchConfig &AConfig, bool ADeduplicateFamilies = false)
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
double NormalizeClusterAngle(double AAngle)
{
    while (AAngle > CET_CLUSTER_PI)
        AAngle -= CET_CLUSTER_TWO_PI;
    while (AAngle < -CET_CLUSTER_PI)
        AAngle += CET_CLUSTER_TWO_PI;
    return AAngle;
}
std::vector<TetEllipseCenter> CollectEllipseGapCenters(const CetTNestItemVector &AOriginalItems, const std::vector<TetShapeFeature> &AFeatures, const TetClusterCandidate &ACandidate)
{
    std::vector<TetEllipseCenter> Centers;
    ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
    for (const TetItemTransform &Transform : ACandidate.Transforms) {
        if (Transform.OriginalId < 0 || Transform.OriginalId >= static_cast<int>(AFeatures.size()) || AFeatures[Transform.OriginalId].ShapeType != MetShapeType::EllipseLike)
            continue;
        const CetPath Contour = Geometry.TransformContour(Geometry.GetIdentityContour(AOriginalItems[Transform.OriginalId]), Transform.RelativeRotation, Transform.RelativeX, Transform.RelativeY);
        double MinX = 0.0, MinY = 0.0, MaxX = 0.0, MaxY = 0.0;
        if (Geometry.GetBounds(Contour, MinX, MinY, MaxX, MaxY)) {
            Centers.push_back({(MinX + MaxX) * 0.5, (MinY + MaxY) * 0.5, (MaxX - MinX) * 0.5, (MaxY - MinY) * 0.5, Transform.RelativeRotation});
        }
    }
    return Centers;
}
void AppendEllipseGapWindow(std::vector<TetCircleGapWindow> &AWindows, const TetGapWindowPlacement &APlacement)
{
    const double AX = APlacement.CenterX; const double AY = APlacement.CenterY; const double AAngle = APlacement.Angle; const double AHalfWidth = APlacement.HalfWidth; const double AHalfHeight = APlacement.HalfHeight; const char *AKind = APlacement.Kind; const double AScale = APlacement.Scale;
    if (AHalfWidth <= CET_RECTANGLE_FILL_POSITION_TOLERANCE || AHalfHeight <= CET_RECTANGLE_FILL_POSITION_TOLERANCE)
        return;
    const long long AngleKey = std::llround(NormalizeClusterAngle(AAngle) / CET_ELLIPSE_GAP_TEMPLATE_ANGLE_TOLERANCE);
    AWindows.push_back({AX, AY, AAngle, AHalfWidth, AHalfHeight, BuildCircleGapClassKey(AKind, AHalfWidth, AHalfHeight, AScale) + "|a=" + std::to_string(AngleKey)});
}
void AppendEllipseQuadGapWindows(const std::vector<TetEllipseCenter> &ACenters, const std::vector<std::vector<std::size_t>> &ANeighbors, std::vector<TetCircleGapWindow> &AWindows)
{
    std::set<std::string> Seen;
    for (std::size_t First = 0; First < ANeighbors.size(); ++First) {
        const std::vector<std::size_t> &Local = ANeighbors[First];
        for (std::size_t Left = 0; Left < Local.size(); ++Left)
            for (std::size_t Right = Left + 1; Right < Local.size(); ++Right) {
                const std::size_t Second = Local[Left], Third = Local[Right];
                const double Cross = (ACenters[Second].X - ACenters[First].X) * (ACenters[Third].Y - ACenters[First].Y) - (ACenters[Second].Y - ACenters[First].Y) * (ACenters[Third].X - ACenters[First].X);
                if (std::abs(Cross) <= CET_RECTANGLE_FILL_POSITION_TOLERANCE)
                    continue;
                for (std::size_t Fourth : ANeighbors[Second]) {
                    if (Fourth == First || Fourth == Second || Fourth == Third || !AreCircleNeighbors(ANeighbors, Third, Fourth))
                        continue;
                    std::vector<std::size_t> Indices{First, Second, Third, Fourth};
                    std::sort(Indices.begin(), Indices.end());
                    std::ostringstream Key;
                    for (std::size_t Index : Indices)
                        Key << Index << ',';
                    if (!Seen.insert(Key.str()).second)
                        continue;
                    double MinX = ACenters[First].X, MaxX = MinX, MinY = ACenters[First].Y, MaxY = MinY;
                    double Scale = std::min(ACenters[First].HalfWidth, ACenters[First].HalfHeight);
                    for (std::size_t Index : Indices) {
                        MinX = std::min(MinX, ACenters[Index].X);
                        MaxX = std::max(MaxX, ACenters[Index].X);
                        MinY = std::min(MinY, ACenters[Index].Y);
                        MaxY = std::max(MaxY, ACenters[Index].Y);
                        Scale = std::min(Scale, std::min(ACenters[Index].HalfWidth, ACenters[Index].HalfHeight));
                    }
                    AppendEllipseGapWindow(AWindows, {(MinX + MaxX) * 0.5, (MinY + MaxY) * 0.5, 0.0, std::max(CET_RECTANGLE_FILL_POSITION_TOLERANCE * 2.0, Scale * 0.40), std::max(CET_RECTANGLE_FILL_POSITION_TOLERANCE * 2.0, Scale * 0.40), 0.0, "quad-ellipse", Scale});
                }
            }
    }
}
std::vector<TetCircleGapWindow> CollectEllipseGapWindows(const CetTNestItemVector &AOriginalItems, const std::vector<TetShapeFeature> &AFeatures, const TetClusterCandidate &ABaseCandidate, const TetClusterCandidate &AEnvelopeCandidate)
{
    const std::vector<TetEllipseCenter> Centers = CollectEllipseGapCenters(AOriginalItems, AFeatures, ABaseCandidate);
    std::vector<TetCircleCenter> Proxies;
    for (const TetEllipseCenter &Center : Centers) {
        Proxies.push_back({Center.X, Center.Y, std::max(Center.HalfWidth, Center.HalfHeight)});
    }
    const double Nearest = FindNearestCircleDistance(Proxies);
    double MaxExtent = 0.0;
    for (const TetEllipseCenter &Center : Centers) {
        MaxExtent = std::max(MaxExtent, std::max(Center.HalfWidth, Center.HalfHeight));
    }
    const double NeighborLimit = std::isfinite(Nearest) ? std::max(Nearest * 1.35, MaxExtent * 2.2) + CET_RECTANGLE_FILL_POSITION_TOLERANCE : 0.0;
    const std::vector<std::vector<std::size_t>> Neighbors = NeighborLimit > 0.0 ? BuildCircleNeighborLists(Proxies, NeighborLimit) : std::vector<std::vector<std::size_t>>{};
    std::vector<TetCircleGapWindow> Windows;
    AppendEllipseQuadGapWindows(Centers, Neighbors, Windows);
    for (std::size_t First = 0; First < Centers.size(); ++First) {
        for (std::size_t Second : Neighbors[First]) {
            if (Second <= First)
                continue;
            const double DX = Centers[Second].X - Centers[First].X;
            const double DY = Centers[Second].Y - Centers[First].Y;
            const double Distance = std::hypot(DX, DY);
            if (Distance <= CET_RECTANGLE_FILL_POSITION_TOLERANCE)
                continue;
            const double Scale = std::min({Centers[First].HalfWidth, Centers[First].HalfHeight, Centers[Second].HalfWidth, Centers[Second].HalfHeight});
            const double HalfSize = std::max(CET_RECTANGLE_FILL_POSITION_TOLERANCE * 2.0, std::min(Scale * 0.35, Distance * 0.30));
            AppendEllipseGapWindow(Windows, {(Centers[First].X + Centers[Second].X) * 0.5, (Centers[First].Y + Centers[Second].Y) * 0.5, std::atan2(DY, DX), HalfSize, HalfSize, 0.0, "pair-ellipse", Distance});
        }
    }
    for (std::size_t First = 0; First < Centers.size(); ++First) {
        const std::vector<std::size_t> &Local = Neighbors[First];
        for (std::size_t Left = 0; Left < Local.size(); ++Left)
            for (std::size_t Right = Left + 1; Right < Local.size(); ++Right) {
                const std::size_t Second = Local[Left], Third = Local[Right];
                if (Second <= First || Third <= First || !AreCircleNeighbors(Neighbors, Second, Third))
                    continue;
                const double Scale = std::min({Centers[First].HalfWidth, Centers[First].HalfHeight, Centers[Second].HalfWidth, Centers[Second].HalfHeight, Centers[Third].HalfWidth, Centers[Third].HalfHeight});
                const double HalfSize = std::max(CET_RECTANGLE_FILL_POSITION_TOLERANCE * 2.0, Scale * 0.28);
                AppendEllipseGapWindow(Windows, {(Centers[First].X + Centers[Second].X + Centers[Third].X) / 3.0, (Centers[First].Y + Centers[Second].Y + Centers[Third].Y) / 3.0, 0.0, HalfSize, HalfSize, 0.0, "triple-ellipse", Scale});
            }
    }
    for (const TetEllipseCenter &Center : Centers) {
        const double Top = Center.Y - Center.HalfHeight;
        const double Bottom = AEnvelopeCandidate.ClusterHeight - Center.Y - Center.HalfHeight;
        const double Left = Center.X - Center.HalfWidth;
        const double Right = AEnvelopeCandidate.ClusterWidth - Center.X - Center.HalfWidth;
        if (Top > CET_RECTANGLE_FILL_POSITION_TOLERANCE)
            AppendEllipseGapWindow(Windows, {Center.X, Top * 0.5, 0.0, std::min(Center.HalfWidth, AEnvelopeCandidate.ClusterWidth * 0.5), Top * 0.5, 0.0, "edge-ellipse", Top});
        if (Bottom > CET_RECTANGLE_FILL_POSITION_TOLERANCE)
            AppendEllipseGapWindow(Windows, {Center.X, AEnvelopeCandidate.ClusterHeight - Bottom * 0.5, CET_CLUSTER_PI, std::min(Center.HalfWidth, AEnvelopeCandidate.ClusterWidth * 0.5), Bottom * 0.5, 0.0, "edge-ellipse", Bottom});
        if (Left > CET_RECTANGLE_FILL_POSITION_TOLERANCE)
            AppendEllipseGapWindow(Windows, {Left * 0.5, Center.Y, CET_CLUSTER_HALF_PI, Left * 0.5, std::min(Center.HalfHeight, AEnvelopeCandidate.ClusterHeight * 0.5), 0.0, "edge-ellipse", Left});
        if (Right > CET_RECTANGLE_FILL_POSITION_TOLERANCE)
            AppendEllipseGapWindow(Windows, {AEnvelopeCandidate.ClusterWidth - Right * 0.5, Center.Y, -CET_CLUSTER_HALF_PI, Right * 0.5, std::min(Center.HalfHeight, AEnvelopeCandidate.ClusterHeight * 0.5), 0.0, "edge-ellipse", Right});
    }
    std::stable_sort(Windows.begin(), Windows.end(), [](const TetCircleGapWindow &A, const TetCircleGapWindow &B) {
        const int APriority = GetCircleGapPriority(A.ClassKey), BPriority = GetCircleGapPriority(B.ClassKey);
        if (APriority != BPriority)
            return APriority < BPriority;
        const double AArea = A.HalfWidth * A.HalfHeight;
        const double BArea = B.HalfWidth * B.HalfHeight;
        if (std::abs(AArea - BArea) > 1.0)
            return AArea > BArea;
        return A.ClassKey < B.ClassKey;
    });
    if (Windows.size() > CET_ELLIPSE_GAP_FILL_MAX_WINDOWS)
        Windows.resize(CET_ELLIPSE_GAP_FILL_MAX_WINDOWS);
    return Windows;
}
bool SearchEllipseGapWindow(const TetClusterFillContext &AContext, const TetCircleGapWindow &AWindow, const TetClusterFillSearchConfig &AConfig, const TetClusterFillSearchState &AInitial, TetClusterCandidate &AOutCandidate)
{
    const auto &AOriginalItems = AContext.OriginalItems; const auto &AFeatures = AContext.Features; const auto &AOptions = AContext.Options; const auto &ABaseCandidate = AContext.BaseCandidate; const auto &AEnvelopeCandidate = AContext.EnvelopeCandidate;
    const auto SearchStart = std::chrono::steady_clock::now();
    std::vector<TetClusterFillSearchState> Beam{AInitial}, Next;
    TetClusterFillSearchState Best = AInitial;
    ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
    ET::NEST2DMANAGERLIB::CetRectangleFillClusterBuilder Builder;
    std::size_t Attempts = 0;
    const std::size_t MaxDepth = std::min(CET_ELLIPSE_GAP_FILL_MAX_TEMPLATE_DEPTH, AConfig.MaxDepth);
    for (std::size_t Depth = 0; Depth < MaxDepth && !Beam.empty(); ++Depth) {
        Next.clear();
        for (const TetClusterFillSearchState &State : Beam) {
            std::vector<TetClusterFreeRegion> FreeRegions, LocalRegions;
            if (!Geometry.ExtractCandidateFreeRegions(AOriginalItems, AOptions, State.Candidate, FreeRegions))
                continue;
            if (!ClipFreeRegionsToGapWindow(FreeRegions, AWindow, LocalRegions))
                continue;
            if (LocalRegions.empty())
                continue;
            // One representative per family keeps the local beam compositional while
            // avoiding identical contour probes for every remaining source instance.
            for (int Filler : CollectCompatibleFillers(AFeatures, State.Candidate, LocalRegions, AConfig, true)) {
                if (Attempts++ >= CET_ELLIPSE_GAP_FILL_MAX_ATTEMPTS)
                    break;
                TetClusterCandidate Candidate;
                if (!Builder.TryAppendFillerInRectangleEnvelope({AOriginalItems, AFeatures, AOptions, ABaseCandidate, AEnvelopeCandidate, State.Candidate, &LocalRegions, Filler, AEnvelopeCandidate.ClusterWidth, AEnvelopeCandidate.ClusterHeight}, Candidate))
                    continue;
                TetClusterFillSearchState StateCandidate{std::move(Candidate), State.FillerCount + 1};
                if (IsEnvelopeStateBetter(StateCandidate, Best))
                    Best = StateCandidate;
                Next.push_back(std::move(StateCandidate));
            }
        }
        TrimEnvelopeBeam(Next, CET_CIRCLE_GAP_SEARCH_BEAM_WIDTH, AFeatures, ABaseCandidate.SkeletonChildCount);
        Beam = std::move(Next);
        if (Attempts >= CET_ELLIPSE_GAP_FILL_MAX_ATTEMPTS || CircleGapSearchTimeReached(SearchStart, CET_ELLIPSE_GAP_FILL_MAX_TIME_MS))
            break;
    }
    std::cout << "[TEMPLATE][ELLIPSE GAP SEARCH] Class=" << AWindow.ClassKey << " Attempts=" << Attempts << " Fillers=" << Best.FillerCount << " Ms=" << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - SearchStart).count() << std::endl;
    AOutCandidate = std::move(Best.Candidate);
    return Best.FillerCount > AInitial.FillerCount;
}
bool TryCopyEllipseGapWindowTemplate(const TetClusterFillContext &AContext, const TetEllipseGapWindowTemplate &ATemplate, const TetCircleGapWindow &ATarget, const TetClusterCandidate &ACurrent, TetClusterCandidate &AOutCandidate)
{
    const auto &AOriginalItems = AContext.OriginalItems; const auto &AFeatures = AContext.Features; const auto &AOptions = AContext.Options; const auto &ABaseCandidate = AContext.BaseCandidate; const auto &AEnvelopeCandidate = AContext.EnvelopeCandidate;
    AOutCandidate = TetClusterCandidate{};
    if (ATemplate.Transforms.empty() || ATemplate.Source.ClassKey != ATarget.ClassKey)
        return false;
    if (std::abs(NormalizeClusterAngle(ATarget.Angle - ATemplate.Source.Angle)) > CET_ELLIPSE_GAP_TEMPLATE_ANGLE_TOLERANCE)
        return false;
    ET::NEST2DMANAGERLIB::CetRectangleFillClusterBuilder Builder;
    TetClusterCandidate Current = ACurrent;
    std::set<int> Reserved(Current.OriginalIndices.begin(), Current.OriginalIndices.end());
    for (const TetItemTransform &Source : ATemplate.Transforms) {
        const int TargetId = FindAvailableFamilyItem(AFeatures, std::vector<bool>(AFeatures.size(), false), Reserved, Source.OriginalId);
        if (TargetId < 0)
            return false;
        TetItemTransform Transform = Source;
        Transform.OriginalId = TargetId;
        Transform.RelativeX += ATarget.CenterX - ATemplate.Source.CenterX;
        Transform.RelativeY += ATarget.CenterY - ATemplate.Source.CenterY;
        TetClusterCandidate Next;
        if (!Builder.TryAppendFillerTemplateInRectangleEnvelope({AOriginalItems, AFeatures, AOptions, ABaseCandidate, AEnvelopeCandidate, Current, nullptr, Transform.OriginalId, AEnvelopeCandidate.ClusterWidth, AEnvelopeCandidate.ClusterHeight}, Transform, Next))
            return false;
        Current = std::move(Next);
        Reserved.insert(TargetId);
    }
    AOutCandidate = std::move(Current);
    return true;
}
void CollectEllipseGapGroups(const CetTNestItemVector &AOriginalItems, const TetNestOptions &AOptions, const TetClusterCandidate &ACandidate, const std::vector<TetCircleGapWindow> &AWindows, std::vector<std::vector<TetCircleGapWindow>> &AOutGroups)
{
    AOutGroups.clear();
    std::map<std::string, std::size_t> GroupIndices;
    for (const TetCircleGapWindow &Window : AWindows) {
        std::string Signature;
        if (!BuildEllipseGapRegionSignature(AOriginalItems, AOptions, ACandidate, Window, Signature))
            continue;
        const auto Result = GroupIndices.emplace(Signature, AOutGroups.size());
        if (Result.second)
            AOutGroups.push_back({});
        AOutGroups[Result.first->second].push_back(Window);
    }
}
bool BuildCompleteEllipseGapTemplate(const TetClusterFillContext &AContext, const TetClusterFillSearchConfig &AConfig, const TetClusterFillSearchState &ACurrent, const TetCircleGapWindow &AWindow, TetClusterCandidate &AOutCandidate, TetEllipseGapWindowTemplate &AOutTemplate)
{
    const auto &AOriginalItems = AContext.OriginalItems;
    AOutCandidate = TetClusterCandidate{};
    AOutTemplate = TetEllipseGapWindowTemplate{};
    const std::size_t Remaining = AOriginalItems.size() > ACurrent.Candidate.Transforms.size() ? AOriginalItems.size() - ACurrent.Candidate.Transforms.size() : 0;
    if (Remaining == 0)
        return false;
    TetClusterFillSearchConfig TemplateConfig = AConfig;
    TemplateConfig.MaxDepth = std::min(Remaining, CET_ELLIPSE_GAP_FILL_MAX_TEMPLATE_DEPTH);
    const std::size_t Start = ACurrent.Candidate.Transforms.size();
    if (!SearchEllipseGapWindow(AContext, AWindow, TemplateConfig, ACurrent, AOutCandidate))
        return false;
    AOutTemplate.Source = AWindow;
    AOutTemplate.Transforms.assign(AOutCandidate.Transforms.begin() + static_cast<std::ptrdiff_t>(Start), AOutCandidate.Transforms.end());
    return !AOutTemplate.Transforms.empty();
}
bool BuildLocalEllipseGapFilledCandidate(const TetClusterFillContext &AContext, const TetClusterFillSearchConfig &AConfig, TetClusterCandidate &AOutCandidate)
{
    const auto &AOriginalItems = AContext.OriginalItems; const auto &AFeatures = AContext.Features; const auto &AOptions = AContext.Options; const auto &ABaseCandidate = AContext.BaseCandidate; const auto &AEnvelopeCandidate = AContext.EnvelopeCandidate;
    AOutCandidate = AEnvelopeCandidate;
    const std::vector<TetCircleGapWindow> Windows = CollectEllipseGapWindows(AOriginalItems, AFeatures, ABaseCandidate, AEnvelopeCandidate);
    const auto SearchStart = std::chrono::steady_clock::now();
    TetClusterFillSearchState Current{AOutCandidate, 0};
    while (!CircleGapSearchTimeReached(SearchStart, CET_ELLIPSE_GAP_FILL_TOTAL_TIME_MS)) {
        std::vector<std::vector<TetCircleGapWindow>> Groups;
        CollectEllipseGapGroups(AOriginalItems, AOptions, Current.Candidate, Windows, Groups);
        bool FilledBatch = false;
        for (const std::vector<TetCircleGapWindow> &Group : Groups) {
            if (Group.empty() || CircleGapSearchTimeReached(SearchStart, CET_ELLIPSE_GAP_FILL_TOTAL_TIME_MS))
                break;
            TetClusterCandidate Candidate;
            TetEllipseGapWindowTemplate Template;
            if (!BuildCompleteEllipseGapTemplate(AContext, AConfig, Current, Group.front(), Candidate, Template))
                continue;
            const std::size_t TemplateFillerCount = Candidate.Transforms.size() - ABaseCandidate.Transforms.size();
            Current = {std::move(Candidate), TemplateFillerCount};
            std::size_t Copies = 0;
            for (std::size_t Index = 1; Index < Group.size(); ++Index) {
                TetClusterCandidate Copied;
                if (!TryCopyEllipseGapWindowTemplate(AContext, Template, Group[Index], Current.Candidate, Copied))
                    break;
                const std::size_t CopiedFillerCount = Copied.Transforms.size() - ABaseCandidate.Transforms.size();
                Current = {std::move(Copied), CopiedFillerCount};
                ++Copies;
            }
            std::cout << "[TEMPLATE][ELLIPSE COMPLETE TEMPLATE] Class=" << Template.Source.ClassKey << " Fillers=" << Template.Transforms.size() << " Copies=" << Copies << std::endl;
            FilledBatch = true;
        }
        if (!FilledBatch)
            break;
    }
    AOutCandidate = std::move(Current.Candidate);
    return AOutCandidate.Transforms.size() > ABaseCandidate.Transforms.size();
}
long long QuantizeEllipseTemplateRatio(double AValue)
{
    return std::llround(std::max(0.0, AValue) / CET_ELLIPSE_GAP_TEMPLATE_SIZE_TOLERANCE);
}
std::string BuildEllipseGapTemplateCacheKey(const TetClusterCandidate &ABaseCandidate, const std::vector<TetShapeFeature> &AFeatures)
{
    std::vector<long long> AspectRatios;
    for (std::size_t Index = 0; Index < ABaseCandidate.SkeletonChildCount && Index < ABaseCandidate.OriginalIndices.size(); ++Index) {
        const int OriginalIndex = ABaseCandidate.OriginalIndices[Index];
        if (OriginalIndex < 0 || OriginalIndex >= static_cast<int>(AFeatures.size()))
            continue;
        const TetShapeFeature &Feature = AFeatures[OriginalIndex];
        AspectRatios.push_back(QuantizeEllipseTemplateRatio(std::min(Feature.Width, Feature.Height) / std::max(1.0, std::max(Feature.Width, Feature.Height))));
    }
    std::sort(AspectRatios.begin(), AspectRatios.end());
    const double ShortSide = std::max(1.0, std::min(ABaseCandidate.ClusterWidth, ABaseCandidate.ClusterHeight));
    const double LongSide = std::max(ABaseCandidate.ClusterWidth, ABaseCandidate.ClusterHeight);
    std::vector<long long> SkeletonDistances;
    const double Normalizer = std::max(1.0, std::hypot(ABaseCandidate.ClusterWidth, ABaseCandidate.ClusterHeight));
    for (std::size_t First = 0; First < ABaseCandidate.SkeletonChildCount && First < ABaseCandidate.Transforms.size(); ++First) {
        for (std::size_t Second = First + 1; Second < ABaseCandidate.SkeletonChildCount && Second < ABaseCandidate.Transforms.size(); ++Second) {
            const TetItemTransform &FirstTransform = ABaseCandidate.Transforms[First];
            const TetItemTransform &SecondTransform = ABaseCandidate.Transforms[Second];
            SkeletonDistances.push_back(QuantizeEllipseTemplateRatio(std::hypot(SecondTransform.RelativeX - FirstTransform.RelativeX, SecondTransform.RelativeY - FirstTransform.RelativeY) / Normalizer));
        }
    }
    std::sort(SkeletonDistances.begin(), SkeletonDistances.end());
    std::ostringstream Stream;
    Stream << ABaseCandidate.ClusterType << '|' << ABaseCandidate.SkeletonChildCount << '|' << QuantizeEllipseTemplateRatio(ShortSide / std::max(1.0, LongSide));
    for (long long Ratio : AspectRatios)
        Stream << '|' << Ratio;
    for (long long Distance : SkeletonDistances)
        Stream << '|' << Distance;
    return Stream.str();
}
double GetEllipseTemplateFrameAngle(const CetTNestItemVector &AOriginalItems, const TetClusterCandidate &ACandidate)
{
    if (ACandidate.SkeletonChildCount < 2 || ACandidate.Transforms.size() < 2)
        return 0.0;
    ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
    auto GetCenter = [&](const TetItemTransform &Transform, double &AX, double &AY) {
        if (Transform.OriginalId < 0 || Transform.OriginalId >= static_cast<int>(AOriginalItems.size()))
            return false;
        const CetPath Contour = Geometry.TransformContour(Geometry.GetIdentityContour(AOriginalItems[Transform.OriginalId]), Transform.RelativeRotation, Transform.RelativeX, Transform.RelativeY);
        double MinX = 0.0, MinY = 0.0, MaxX = 0.0, MaxY = 0.0;
        if (!Geometry.GetBounds(Contour, MinX, MinY, MaxX, MaxY))
            return false;
        AX = (MinX + MaxX) * 0.5;
        AY = (MinY + MaxY) * 0.5;
        return true;
    };
    double FirstX = 0.0, FirstY = 0.0, SecondX = 0.0, SecondY = 0.0;
    if (!GetCenter(ACandidate.Transforms[0], FirstX, FirstY) || !GetCenter(ACandidate.Transforms[1], SecondX, SecondY))
        return 0.0;
    return std::atan2(SecondY - FirstY, SecondX - FirstX);
}
bool BuildEllipseTemplateTransform(const TetEllipseTemplateTransformRequest &ARequest, TetItemTransform &AOutTransform)
{
    const auto &AOriginalItems = ARequest.OriginalItems; const auto &ATemplate = ARequest.Template; const auto &AEnvelopeCandidate = ARequest.EnvelopeCandidate; const auto &ASourceTransform = ARequest.SourceTransform; const int ATargetId = ARequest.TargetId; const double ATargetAngle = ARequest.TargetAngle; const bool AMirror = ARequest.Mirror; const double ARotation = ARequest.Rotation;
    ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
    if (ASourceTransform.OriginalId < 0 || ASourceTransform.OriginalId >= static_cast<int>(AOriginalItems.size()) || ATargetId < 0 || ATargetId >= static_cast<int>(AOriginalItems.size()))
        return false;
    const CetPath SourceContour = Geometry.TransformContour(Geometry.GetIdentityContour(AOriginalItems[ASourceTransform.OriginalId]), ASourceTransform.RelativeRotation, ASourceTransform.RelativeX, ASourceTransform.RelativeY);
    double SourceMinX = 0.0, SourceMinY = 0.0, SourceMaxX = 0.0, SourceMaxY = 0.0;
    if (!Geometry.GetBounds(SourceContour, SourceMinX, SourceMinY, SourceMaxX, SourceMaxY))
        return false;
    const double Delta = NormalizeClusterAngle(ATargetAngle - ATemplate.SourceAngle);
    const bool SwapsEnvelopeAxes = std::abs(std::sin(Delta)) > std::abs(std::cos(Delta));
    const double ScaleX = (SwapsEnvelopeAxes ? AEnvelopeCandidate.ClusterHeight : AEnvelopeCandidate.ClusterWidth) / ATemplate.EnvelopeWidth;
    const double ScaleY = (SwapsEnvelopeAxes ? AEnvelopeCandidate.ClusterWidth : AEnvelopeCandidate.ClusterHeight) / ATemplate.EnvelopeHeight;
    double OffsetX = ((SourceMinX + SourceMaxX) - ATemplate.EnvelopeWidth) * 0.5;
    double OffsetY = ((SourceMinY + SourceMaxY) - ATemplate.EnvelopeHeight) * 0.5;
    if (AMirror)
        OffsetX = -OffsetX;
    OffsetX *= ScaleX;
    OffsetY *= ScaleY;
    const double Cosine = std::cos(Delta), Sine = std::sin(Delta);
    const double CenterX = AEnvelopeCandidate.ClusterWidth * 0.5 + OffsetX * Cosine - OffsetY * Sine;
    const double CenterY = AEnvelopeCandidate.ClusterHeight * 0.5 + OffsetX * Sine + OffsetY * Cosine;
    const CetPath Rotated = Geometry.TransformContour(Geometry.GetIdentityContour(AOriginalItems[ATargetId]), ARotation, 0.0, 0.0);
    double MinX = 0.0, MinY = 0.0, MaxX = 0.0, MaxY = 0.0;
    if (!Geometry.GetBounds(Rotated, MinX, MinY, MaxX, MaxY))
        return false;
    AOutTransform.OriginalId = ATargetId;
    AOutTransform.RelativeRotation = ARotation;
    AOutTransform.RelativeX = CenterX - (MinX + MaxX) * 0.5;
    AOutTransform.RelativeY = CenterY - (MinY + MaxY) * 0.5;
    return true;
}
bool TryBuildEllipseTemplateVariant(const TetClusterFillContext &AContext, const TetEllipseGapTemplate &ATemplate, bool AMirror, TetClusterCandidate &AOutCandidate)
{
    const auto &AOriginalItems = AContext.OriginalItems; const auto &AFeatures = AContext.Features; const auto &AOptions = AContext.Options; const auto &ABaseCandidate = AContext.BaseCandidate; const auto &AEnvelopeCandidate = AContext.EnvelopeCandidate;
    AOutCandidate = TetClusterCandidate{};
    if (ATemplate.Transforms.size() <= ATemplate.SkeletonChildCount || ATemplate.SkeletonChildCount != ABaseCandidate.SkeletonChildCount)
        return false;
    const double TargetAngle = GetEllipseTemplateFrameAngle(AOriginalItems, ABaseCandidate);
    const double Delta = NormalizeClusterAngle(TargetAngle - ATemplate.SourceAngle);
    const std::vector<double> Allowed = ET::NEST2DMANAGERLIB::CetRotationUtils::BuildAllowedRotations(AOptions.Rotations);
    if (Allowed.empty())
        return false;
    ET::NEST2DMANAGERLIB::CetRectangleFillClusterBuilder Builder;
    TetClusterCandidate Current = AEnvelopeCandidate;
    std::set<int> Reserved(Current.OriginalIndices.begin(), Current.OriginalIndices.end());
    for (std::size_t Index = ATemplate.SkeletonChildCount; Index < ATemplate.Transforms.size(); ++Index) {
        const TetItemTransform &Source = ATemplate.Transforms[Index];
        const int TargetId = FindAvailableFamilyItem(AFeatures, std::vector<bool>(AFeatures.size(), false), Reserved, Source.OriginalId);
        if (TargetId < 0)
            return false;
        const double Desired = AMirror ? CET_CLUSTER_PI - Source.RelativeRotation + Delta : Source.RelativeRotation + Delta;
        std::vector<double> Rotations = Allowed;
        std::stable_sort(Rotations.begin(), Rotations.end(), [&](double A, double B) {
            const bool AAligned = std::abs(NormalizeClusterAngle(A - Desired)) <= CET_ELLIPSE_GAP_TEMPLATE_ANGLE_TOLERANCE;
            const bool BAligned = std::abs(NormalizeClusterAngle(B - Desired)) <= CET_ELLIPSE_GAP_TEMPLATE_ANGLE_TOLERANCE;
            if (AAligned != BAligned)
                return AAligned;
            return std::abs(NormalizeClusterAngle(A - Desired)) < std::abs(NormalizeClusterAngle(B - Desired));
        });
        bool Appended = false;
        for (double Rotation : Rotations) {
            TetItemTransform Transform;
            if (!BuildEllipseTemplateTransform({AOriginalItems, ATemplate, AEnvelopeCandidate, Source, TargetId, TargetAngle, AMirror, Rotation}, Transform))
                continue;
            TetClusterCandidate Next;
            if (Builder.TryAppendFillerTemplateInRectangleEnvelope({AOriginalItems, AFeatures, AOptions, ABaseCandidate, AEnvelopeCandidate, Current, nullptr, Transform.OriginalId, AEnvelopeCandidate.ClusterWidth, AEnvelopeCandidate.ClusterHeight}, Transform, Next)) {
                Current = std::move(Next);
                Reserved.insert(TargetId);
                Appended = true;
                break;
            }
        }
        if (!Appended)
            return false;
    }
    AOutCandidate = std::move(Current);
    return AOutCandidate.Valid;
}
bool BuildCachedEllipseTemplateVariant(const TetClusterFillContext &AContext, const TetEllipseGapTemplateCache &ATemplates, TetClusterCandidate &AOutCandidate)
{
    const auto &AOriginalItems = AContext.OriginalItems; const auto &AFeatures = AContext.Features; const auto &ABaseCandidate = AContext.BaseCandidate; const auto &AEnvelopeCandidate = AContext.EnvelopeCandidate;
    AOutCandidate = TetClusterCandidate{};
    const auto It = ATemplates.find(BuildEllipseGapTemplateCacheKey(ABaseCandidate, AFeatures));
    if (It == ATemplates.end())
        return false;
    const TetEllipseGapTemplate &Template = It->second;
    const double TargetAngle = GetEllipseTemplateFrameAngle(AOriginalItems, ABaseCandidate);
    const double Delta = NormalizeClusterAngle(TargetAngle - Template.SourceAngle);
    const bool SwapsEnvelopeAxes = std::abs(std::sin(Delta)) > std::abs(std::cos(Delta));
    const double WidthRatio = (SwapsEnvelopeAxes ? AEnvelopeCandidate.ClusterHeight : AEnvelopeCandidate.ClusterWidth) / std::max(1.0, Template.EnvelopeWidth);
    const double HeightRatio = (SwapsEnvelopeAxes ? AEnvelopeCandidate.ClusterWidth : AEnvelopeCandidate.ClusterHeight) / std::max(1.0, Template.EnvelopeHeight);
    if (std::abs(WidthRatio - 1.0) > CET_ELLIPSE_GAP_TEMPLATE_SIZE_TOLERANCE || std::abs(HeightRatio - 1.0) > CET_ELLIPSE_GAP_TEMPLATE_SIZE_TOLERANCE)
        return false;
    if (TryBuildEllipseTemplateVariant(AContext, Template, false, AOutCandidate))
        return true;
    return TryBuildEllipseTemplateVariant(AContext, Template, true, AOutCandidate);
}
void StoreEllipseTemplateVariant(const TetClusterFillContext &AContext, const std::vector<TetClusterFillSearchState> &AStates, bool AHasValidatedVariant, TetEllipseGapTemplateCache &ATemplates)
{
    const auto &AOriginalItems = AContext.OriginalItems; const auto &AFeatures = AContext.Features; const auto &ABaseCandidate = AContext.BaseCandidate; const auto &AEnvelopeCandidate = AContext.EnvelopeCandidate;
    if (!AHasValidatedVariant || AStates.empty())
        return;
    const TetClusterFillSearchState *Best = &AStates.front();
    for (const TetClusterFillSearchState &State : AStates) {
        if (IsEnvelopeStateBetter(State, *Best))
            Best = &State;
    }
    if (Best->Candidate.Transforms.size() <= ABaseCandidate.SkeletonChildCount)
        return;
    const std::string Key = BuildEllipseGapTemplateCacheKey(ABaseCandidate, AFeatures);
    if (ATemplates.find(Key) != ATemplates.end())
        return;
    TetEllipseGapTemplate Template;
    Template.SourceAngle = GetEllipseTemplateFrameAngle(AOriginalItems, ABaseCandidate);
    Template.EnvelopeWidth = AEnvelopeCandidate.ClusterWidth;
    Template.EnvelopeHeight = AEnvelopeCandidate.ClusterHeight;
    Template.SkeletonChildCount = ABaseCandidate.SkeletonChildCount;
    const std::size_t TransformCount = std::min(Best->Candidate.Transforms.size(), Template.SkeletonChildCount + CET_ELLIPSE_GAP_TEMPLATE_MAX_COPIES);
    Template.Transforms.assign(Best->Candidate.Transforms.begin(), Best->Candidate.Transforms.begin() + TransformCount);
    ATemplates.emplace(Key, std::move(Template));
    std::cout << "[TEMPLATE][ELLIPSE GAP CACHE STORE] Fillers=" << TransformCount - ABaseCandidate.SkeletonChildCount << std::endl;
}
} // anonymous namespace

CetEllipseGapFiller::CetEllipseGapFiller() : CetCoreObject() {}
CetEllipseGapFiller::~CetEllipseGapFiller() {}

bool CetEllipseGapFiller::BuildEllipseGapCandidate(const TetClusterFillContext& AContext, const TetClusterFillSearchConfig& AConfig, TetClusterCandidate& AOutCandidate)
{
    return BuildLocalEllipseGapFilledCandidate(AContext, AConfig, AOutCandidate);
}

bool CetEllipseGapFiller::TryBuildCachedEllipseTemplateVariant(const TetClusterFillContext& AContext, const TetEllipseGapTemplateCache& ATemplates, TetClusterCandidate& AOutCandidate)
{
    return BuildCachedEllipseTemplateVariant(AContext, ATemplates, AOutCandidate);
}

void CetEllipseGapFiller::CacheEllipseTemplateVariant(const TetClusterFillContext& AContext, const std::vector<TetClusterFillSearchState>& AStates, bool AHasValidatedVariant, TetEllipseGapTemplateCache& ATemplates)
{
    StoreEllipseTemplateVariant(AContext, AStates, AHasValidatedVariant, ATemplates);
}

}}
