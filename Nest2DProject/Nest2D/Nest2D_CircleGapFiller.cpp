#include "pch.h"
#include "Nest2D_CircleGapFiller.h"
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
    bool ContainsOriginalIndex(const TetClusterCandidate &ACandidate, int AOriginalIndex)
    {
        return std::find(ACandidate.OriginalIndices.begin(), ACandidate.OriginalIndices.end(), AOriginalIndex) != ACandidate.OriginalIndices.end();
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
        for (const TetItemTransform &Transform : Transforms)
            Stream << Transform.OriginalId << ':' << std::llround(Transform.RelativeX / CET_CLUSTER_FILL_VARIANT_POSITION_TOLERANCE) << ':' << std::llround(Transform.RelativeY / CET_CLUSTER_FILL_VARIANT_POSITION_TOLERANCE) << ':' << std::llround(Transform.RelativeRotation / CET_CLUSTER_FILL_VARIANT_ROTATION_TOLERANCE) << ';';
        return Stream.str();
    }
std::vector<TetCircleCenter> CollectCircleCenters(const CetTNestItemVector &AOriginalItems, const std::vector<TetShapeFeature> &AFeatures, const TetClusterCandidate &ACandidate)
{
    std::vector<TetCircleCenter> Centers;
    ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
    for (const TetItemTransform &Transform : ACandidate.Transforms) {
        if (Transform.OriginalId < 0 || Transform.OriginalId >= static_cast<int>(AFeatures.size()) || AFeatures[Transform.OriginalId].ShapeType != MetShapeType::CircleLike)
            continue;
        const CetPath Contour = Geometry.TransformContour(Geometry.GetIdentityContour(AOriginalItems[Transform.OriginalId]), Transform.RelativeRotation, Transform.RelativeX, Transform.RelativeY);
        double MinX = 0.0, MinY = 0.0, MaxX = 0.0, MaxY = 0.0;
        if (Geometry.GetBounds(Contour, MinX, MinY, MaxX, MaxY)) {
            Centers.push_back({(MinX + MaxX) * 0.5, (MinY + MaxY) * 0.5, std::min(MaxX - MinX, MaxY - MinY) * 0.5});
        }
    }
    return Centers;
}
double FindNearestCircleDistance(const std::vector<TetCircleCenter> &ACenters)
{
    double Nearest = std::numeric_limits<double>::infinity();
    for (std::size_t First = 0; First < ACenters.size(); ++First) {
        for (std::size_t Second = First + 1; Second < ACenters.size(); ++Second) {
            Nearest = std::min(Nearest, std::hypot(ACenters[Second].X - ACenters[First].X, ACenters[Second].Y - ACenters[First].Y));
        }
    }
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
void AppendPairGapAnchors(const std::vector<TetCircleCenter> &ACenters, const std::vector<std::vector<std::size_t>> &ANeighbors, std::vector<TetCircleGapTemplateAnchor> &AAnchors)
{
    for (std::size_t First = 0; First < ANeighbors.size(); ++First) {
        for (std::size_t Second : ANeighbors[First]) {
            if (Second <= First || !AreCircleNeighbors(ANeighbors, First, Second))
                continue;
            const double DeltaX = ACenters[Second].X - ACenters[First].X;
            const double DeltaY = ACenters[Second].Y - ACenters[First].Y;
            AAnchors.push_back({(ACenters[First].X + ACenters[Second].X) * 0.5, (ACenters[First].Y + ACenters[Second].Y) * 0.5, std::atan2(DeltaY, DeltaX), std::hypot(DeltaX, DeltaY), 2});
        }
    }
}
bool TryBuildTripleGapAnchor(const TetCircleCenter &AFirst, const TetCircleCenter &ASecond, const TetCircleCenter &AThird, double ANeighborLimit, TetCircleGapTemplateAnchor &AAnchor)
{
    const double AB = std::hypot(ASecond.X - AFirst.X, ASecond.Y - AFirst.Y);
    const double AC = std::hypot(AThird.X - AFirst.X, AThird.Y - AFirst.Y);
    const double BC = std::hypot(AThird.X - ASecond.X, AThird.Y - ASecond.Y);
    const double MinSide = std::min({AB, AC, BC});
    const double MaxSide = std::max({AB, AC, BC});
    const double Cross = (ASecond.X - AFirst.X) * (AThird.Y - AFirst.Y) - (ASecond.Y - AFirst.Y) * (AThird.X - AFirst.X);
    if (MinSide <= CET_RECTANGLE_FILL_POSITION_TOLERANCE || MaxSide > MinSide * 1.15 || MaxSide > ANeighborLimit || std::abs(Cross) <= CET_RECTANGLE_FILL_POSITION_TOLERANCE)
        return false;
    const double FirstSq = AFirst.X * AFirst.X + AFirst.Y * AFirst.Y;
    const double SecondSq = ASecond.X * ASecond.X + ASecond.Y * ASecond.Y;
    const double ThirdSq = AThird.X * AThird.X + AThird.Y * AThird.Y;
    AAnchor.CenterX = (FirstSq * (ASecond.Y - AThird.Y) + SecondSq * (AThird.Y - AFirst.Y) + ThirdSq * (AFirst.Y - ASecond.Y)) / (2.0 * Cross);
    AAnchor.CenterY = (FirstSq * (AThird.X - ASecond.X) + SecondSq * (AFirst.X - AThird.X) + ThirdSq * (ASecond.X - AFirst.X)) / (2.0 * Cross);
    AAnchor.Angle = std::atan2(ASecond.Y - AFirst.Y, ASecond.X - AFirst.X);
    AAnchor.Distance = std::hypot(AAnchor.CenterX - AFirst.X, AAnchor.CenterY - AFirst.Y);
    AAnchor.NeighborCount = 3;
    return true;
}
void AppendTripleGapAnchors(const std::vector<TetCircleCenter> &ACenters, const std::vector<std::vector<std::size_t>> &ANeighbors, double ALimit, std::vector<TetCircleGapTemplateAnchor> &AAnchors)
{
    for (std::size_t First = 0; First < ANeighbors.size(); ++First) {
        const std::vector<std::size_t> &Local = ANeighbors[First];
        for (std::size_t Left = 0; Left < Local.size(); ++Left) {
            for (std::size_t Right = Left + 1; Right < Local.size(); ++Right) {
                const std::size_t Second = Local[Left], Third = Local[Right];
                if (Second <= First || Third <= First || !AreCircleNeighbors(ANeighbors, Second, Third))
                    continue;
                TetCircleGapTemplateAnchor Anchor;
                if (TryBuildTripleGapAnchor(ACenters[First], ACenters[Second], ACenters[Third], ALimit, Anchor)) {
                    AAnchors.push_back(Anchor);
                }
            }
        }
    }
}
std::vector<TetCircleGapTemplateAnchor> _CollectCircleGapTemplateAnchors(const CetTNestItemVector &AOriginalItems, const std::vector<TetShapeFeature> &AFeatures, const TetClusterCandidate &ABaseCandidate)
{
    const std::vector<TetCircleCenter> Centers = CollectCircleCenters(AOriginalItems, AFeatures, ABaseCandidate);
    const double NearestCenterDistance = FindNearestCircleDistance(Centers);
    if (!std::isfinite(NearestCenterDistance))
        return {};
    const double NeighborLimit = NearestCenterDistance * 1.15 + CET_RECTANGLE_FILL_POSITION_TOLERANCE;
    const std::vector<std::vector<std::size_t>> Neighbors = BuildCircleNeighborLists(Centers, NeighborLimit);
    std::vector<TetCircleGapTemplateAnchor> Anchors;
    AppendPairGapAnchors(Centers, Neighbors, Anchors);
    AppendTripleGapAnchors(Centers, Neighbors, NeighborLimit, Anchors);
    return Anchors;
}
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
    for (const TetClusterFreeRegion &Region : LocalRegions) {
        Stream << '|' << std::llround(Region.Area / 1000.0) << ':' << std::llround(Region.Width / 1000.0) << ':' << std::llround(Region.Height / 1000.0) << ':' << Region.Contour.size() << ':' << Region.Holes.size();
    }
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
void AppendBoundaryGapWindow(std::vector<TetCircleGapWindow> &AWindows, const TetGapWindowPlacement &APlacement)
{
    const double ACenterX = APlacement.CenterX; const double ACenterY = APlacement.CenterY; const double AAngle = APlacement.Angle; const double AHalfWidth = APlacement.HalfWidth; const double AHalfHeight = APlacement.HalfHeight; const double ARadius = APlacement.Radius;
    if (AHalfWidth <= CET_RECTANGLE_FILL_POSITION_TOLERANCE || AHalfHeight <= CET_RECTANGLE_FILL_POSITION_TOLERANCE)
        return;
    AWindows.push_back({ACenterX, ACenterY, AAngle, AHalfWidth, AHalfHeight, BuildCircleGapClassKey("edge", AHalfWidth, AHalfHeight, ARadius)});
}
void AppendBoundaryGapWindows(const std::vector<TetCircleCenter> &ACenters, const TetClusterCandidate &AEnvelope, std::vector<TetCircleGapWindow> &AWindows)
{
    double MaxTop = 0.0, MaxBottom = 0.0, MaxLeft = 0.0, MaxRight = 0.0;
    for (const TetCircleCenter &Circle : ACenters) {
        const double Top = Circle.Y - Circle.Radius, Bottom = AEnvelope.ClusterHeight - Circle.Y - Circle.Radius;
        const double Left = Circle.X - Circle.Radius, Right = AEnvelope.ClusterWidth - Circle.X - Circle.Radius;
        MaxTop = std::max(MaxTop, Top);
        MaxBottom = std::max(MaxBottom, Bottom);
        MaxLeft = std::max(MaxLeft, Left);
        MaxRight = std::max(MaxRight, Right);
        AppendBoundaryGapWindow(AWindows, {Circle.X, Top * 0.5, -CET_CLUSTER_HALF_PI, Circle.Radius, Top * 0.5, Circle.Radius});
        AppendBoundaryGapWindow(AWindows, {Circle.X, AEnvelope.ClusterHeight - Bottom * 0.5, CET_CLUSTER_HALF_PI, Circle.Radius, Bottom * 0.5, Circle.Radius});
        AppendBoundaryGapWindow(AWindows, {Left * 0.5, Circle.Y, CET_CLUSTER_PI, Left * 0.5, Circle.Radius, Circle.Radius});
        AppendBoundaryGapWindow(AWindows, {AEnvelope.ClusterWidth - Right * 0.5, Circle.Y, 0.0, Right * 0.5, Circle.Radius, Circle.Radius});
    }
    if (MaxTop > CET_RECTANGLE_FILL_POSITION_TOLERANCE) {
        AppendBoundaryGapWindow(AWindows, {AEnvelope.ClusterWidth * 0.5, MaxTop * 0.5, -CET_CLUSTER_HALF_PI, AEnvelope.ClusterWidth * 0.5, MaxTop * 0.5, MaxTop});
    }
    if (MaxBottom > CET_RECTANGLE_FILL_POSITION_TOLERANCE) {
        AppendBoundaryGapWindow(AWindows, {AEnvelope.ClusterWidth * 0.5, AEnvelope.ClusterHeight - MaxBottom * 0.5, CET_CLUSTER_HALF_PI, AEnvelope.ClusterWidth * 0.5, MaxBottom * 0.5, MaxBottom});
    }
    if (MaxLeft > CET_RECTANGLE_FILL_POSITION_TOLERANCE) {
        AppendBoundaryGapWindow(AWindows, {MaxLeft * 0.5, AEnvelope.ClusterHeight * 0.5, CET_CLUSTER_PI, MaxLeft * 0.5, AEnvelope.ClusterHeight * 0.5, MaxLeft});
    }
    if (MaxRight > CET_RECTANGLE_FILL_POSITION_TOLERANCE) {
        AppendBoundaryGapWindow(AWindows, {AEnvelope.ClusterWidth - MaxRight * 0.5, AEnvelope.ClusterHeight * 0.5, 0.0, MaxRight * 0.5, AEnvelope.ClusterHeight * 0.5, MaxRight});
    }
}
void AppendCircleQuadGapWindows(const std::vector<TetCircleCenter> &ACenters, const std::vector<std::vector<std::size_t>> &ANeighbors, std::vector<TetCircleGapWindow> &AWindows);
std::vector<TetCircleGapWindow> CollectCircleGapWindows(const CetTNestItemVector &AOriginalItems, const std::vector<TetShapeFeature> &AFeatures, const TetClusterCandidate &ABase, const TetClusterCandidate &AEnvelope)
{
    const std::vector<TetCircleCenter> Centers = CollectCircleCenters(AOriginalItems, AFeatures, ABase);
    const std::vector<TetCircleGapTemplateAnchor> Anchors = _CollectCircleGapTemplateAnchors(AOriginalItems, AFeatures, ABase);
    std::vector<TetCircleGapWindow> Windows;
    AppendCircleQuadGapWindows(Centers, BuildCircleNeighborLists(Centers, FindNearestCircleDistance(Centers) * 1.15 + CET_RECTANGLE_FILL_POSITION_TOLERANCE), Windows);
    for (const TetCircleGapTemplateAnchor &Anchor : Anchors) {
        const double Span = std::max(Anchor.Distance, CET_RECTANGLE_FILL_POSITION_TOLERANCE * 4.0);
        if (Anchor.NeighborCount == 3) {
            const double HalfSize = Span * 0.30;
            Windows.push_back({Anchor.CenterX, Anchor.CenterY, Anchor.Angle, HalfSize, HalfSize, BuildCircleGapClassKey("triple", HalfSize, HalfSize, Span)});
            continue;
        }
        const double NX = -std::sin(Anchor.Angle), NY = std::cos(Anchor.Angle);
        const double HalfWidth = Span * 0.60, HalfHeight = Span * 0.46;
        const std::string ClassKey = BuildCircleGapClassKey("pair", HalfWidth, HalfHeight, Span);
        for (int Side : {-1, 1})
            Windows.push_back({Anchor.CenterX + NX * Span * 0.42 * Side, Anchor.CenterY + NY * Span * 0.42 * Side, Anchor.Angle + Side * CET_CLUSTER_HALF_PI, HalfWidth, HalfHeight, ClassKey});
    }
    AppendBoundaryGapWindows(Centers, AEnvelope, Windows);
    std::map<std::string, double> AreaByClass;
    for (const TetCircleGapWindow &Window : Windows) {
        AreaByClass[Window.ClassKey] += Window.HalfWidth * Window.HalfHeight * 4.0;
    }
    std::stable_sort(Windows.begin(), Windows.end(), [&](const TetCircleGapWindow &A, const TetCircleGapWindow &B) {
        const int APriority = GetCircleGapPriority(A.ClassKey);
        const int BPriority = GetCircleGapPriority(B.ClassKey);
        if (APriority != BPriority)
            return APriority < BPriority;
        if (std::abs(AreaByClass[A.ClassKey] - AreaByClass[B.ClassKey]) > 1.0)
            return AreaByClass[A.ClassKey] > AreaByClass[B.ClassKey];
        if (A.ClassKey != B.ClassKey)
            return A.ClassKey < B.ClassKey;
        if (A.CenterY != B.CenterY)
            return A.CenterY < B.CenterY;
        return A.CenterX < B.CenterX;
    });
    return Windows;
}
bool TryAppendAlternativeCircleGapFiller(const TetClusterFillContext &AContext, const TetClusterCandidate &ACurrentCandidate, TetClusterCandidate &AOutCandidate)
{
    const auto &AOriginalItems = AContext.OriginalItems; const auto &AFeatures = AContext.Features; const auto &AOptions = AContext.Options; const auto &ABaseCandidate = AContext.BaseCandidate; const auto &AEnvelopeCandidate = AContext.EnvelopeCandidate;
    AOutCandidate = TetClusterCandidate{};
    ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
    std::vector<TetClusterFreeRegion> FreeRegions;
    if (!Geometry.ExtractCandidateFreeRegions(AOriginalItems, AOptions, ACurrentCandidate, FreeRegions) || FreeRegions.empty())
        return false;
    std::vector<int> Alternatives;
    for (int Index = 0; Index < static_cast<int>(AFeatures.size()); ++Index) {
        const TetShapeFeature &Feature = AFeatures[Index];
        if (!ContainsOriginalIndex(ACurrentCandidate, Index) && Feature.Area > 0.0 && Feature.Area <= ACurrentCandidate.ProxyWasteArea + 1.0)
            Alternatives.push_back(Index);
    }
    std::stable_sort(Alternatives.begin(), Alternatives.end(), [&](int A, int B) { return AFeatures[A].Area > AFeatures[B].Area; });
    ET::NEST2DMANAGERLIB::CetRectangleFillClusterBuilder Builder;
    for (int Index : Alternatives) {
        TetClusterCandidate Candidate;
        if (Builder.TryAppendFillerInRectangleEnvelope({AOriginalItems, AFeatures, AOptions, ABaseCandidate, AEnvelopeCandidate, ACurrentCandidate, &FreeRegions, Index, AEnvelopeCandidate.ClusterWidth, AEnvelopeCandidate.ClusterHeight}, Candidate)) {
            AOutCandidate = std::move(Candidate);
            return true;
        }
    }
    return false;
}
bool TryCopyCircleGapTemplate(const TetClusterFillContext &AContext, const std::vector<TetCircleGapTemplateAnchor> &AAnchors, const TetClusterCandidate &ASeedCandidate, std::size_t AMaxCopies, TetClusterCandidate &AOutCandidate, std::size_t &AOutCopies)
{
    const auto &AOriginalItems = AContext.OriginalItems; const auto &AFeatures = AContext.Features; const auto &AOptions = AContext.Options; const auto &ABaseCandidate = AContext.BaseCandidate; const auto &AEnvelopeCandidate = AContext.EnvelopeCandidate;
    AOutCandidate = ASeedCandidate;
    AOutCopies = 0;
    if (AMaxCopies == 0 || ASeedCandidate.Transforms.size() <= ABaseCandidate.Transforms.size())
        return false;
    const TetItemTransform &Prototype = ASeedCandidate.Transforms.back();
    if (Prototype.OriginalId < 0 || Prototype.OriginalId >= static_cast<int>(AFeatures.size()))
        return false;
    if (AAnchors.size() < 2)
        return false;
    ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
    const CetPath PrototypeContour = Geometry.TransformContour(Geometry.GetIdentityContour(AOriginalItems[Prototype.OriginalId]), Prototype.RelativeRotation, Prototype.RelativeX, Prototype.RelativeY);
    double MinX = 0.0, MinY = 0.0, MaxX = 0.0, MaxY = 0.0;
    if (!Geometry.GetBounds(PrototypeContour, MinX, MinY, MaxX, MaxY))
        return false;
    const double FillerX = (MinX + MaxX) * 0.5, FillerY = (MinY + MaxY) * 0.5;
    auto SourceIt = std::min_element(AAnchors.begin(), AAnchors.end(), [&](const TetCircleGapTemplateAnchor &A, const TetCircleGapTemplateAnchor &B) { return std::hypot(A.CenterX - FillerX, A.CenterY - FillerY) < std::hypot(B.CenterX - FillerX, B.CenterY - FillerY); });
    if (SourceIt == AAnchors.end())
        return false;
    const std::uint64_t Family = MakeFillerFamilyKey(AFeatures[Prototype.OriginalId]);
    const std::vector<double> AllowedRotations = ET::NEST2DMANAGERLIB::CetRotationUtils::BuildAllowedRotations(AOptions.Rotations);
    if (AllowedRotations.empty())
        return false;
    const double SourceOffsetX = FillerX - SourceIt->CenterX;
    const double SourceOffsetY = FillerY - SourceIt->CenterY;
    auto NormalizeAngle = [](double AAngle) {
        while (AAngle > CET_CLUSTER_PI)
            AAngle -= 2.0 * CET_CLUSTER_PI;
        while (AAngle < -CET_CLUSTER_PI)
            AAngle += 2.0 * CET_CLUSTER_PI;
        return AAngle;
    };
    ET::NEST2DMANAGERLIB::CetRectangleFillClusterBuilder Builder;
    for (const TetCircleGapTemplateAnchor &Target : AAnchors) {
        if (&Target == &(*SourceIt) || AOutCopies >= AMaxCopies || Target.NeighborCount != SourceIt->NeighborCount || std::abs(Target.Distance - SourceIt->Distance) > CET_RECTANGLE_FILL_POSITION_TOLERANCE)
            continue;
        int CopyIndex = -1;
        for (int Index = 0; Index < static_cast<int>(AFeatures.size()); ++Index) {
            if (!ContainsOriginalIndex(AOutCandidate, Index) && MakeFillerFamilyKey(AFeatures[Index]) == Family) {
                CopyIndex = Index;
                break;
            }
        }
        if (CopyIndex < 0) {
            TetClusterCandidate Alternative;
            if (TryAppendAlternativeCircleGapFiller({AOriginalItems, AFeatures, AOptions, ABaseCandidate, AEnvelopeCandidate}, AOutCandidate, Alternative)) {
                AOutCandidate = std::move(Alternative);
                ++AOutCopies;
                std::cout << "[TEMPLATE][GAP ALTERNATIVE] Template family exhausted, Filler=" << AOutCandidate.Transforms.back().OriginalId << std::endl;
            }
            continue;
        }
        const double DesiredRotation = Prototype.RelativeRotation + Target.Angle - SourceIt->Angle;
        std::vector<double> CandidateRotations = AllowedRotations;
        std::stable_sort(CandidateRotations.begin(), CandidateRotations.end(), [&](double A, double B) { return std::abs(NormalizeAngle(A - DesiredRotation)) < std::abs(NormalizeAngle(B - DesiredRotation)); });
        for (double Rotation : CandidateRotations) {
            TetItemTransform Copy;
            Copy.OriginalId = CopyIndex;
            Copy.RelativeRotation = Rotation;
            const CetPath Rotated = Geometry.TransformContour(Geometry.GetIdentityContour(AOriginalItems[CopyIndex]), Rotation, 0.0, 0.0);
            if (!Geometry.GetBounds(Rotated, MinX, MinY, MaxX, MaxY))
                continue;
            const double DeltaAngle = Target.Angle - SourceIt->Angle;
            const double Cosine = std::cos(DeltaAngle);
            const double Sine = std::sin(DeltaAngle);
            const double TargetCenterX = Target.CenterX + SourceOffsetX * Cosine - SourceOffsetY * Sine;
            const double TargetCenterY = Target.CenterY + SourceOffsetX * Sine + SourceOffsetY * Cosine;
            Copy.RelativeX = TargetCenterX - (MinX + MaxX) * 0.5;
            Copy.RelativeY = TargetCenterY - (MinY + MaxY) * 0.5;
            TetClusterCandidate Next;
            if (Builder.TryAppendFillerTemplateInRectangleEnvelope({AOriginalItems, AFeatures, AOptions, ABaseCandidate, AEnvelopeCandidate, AOutCandidate, nullptr, Copy.OriginalId, AEnvelopeCandidate.ClusterWidth, AEnvelopeCandidate.ClusterHeight}, Copy, Next)) {
                AOutCandidate = std::move(Next);
                ++AOutCopies;
                break;
            }
        }
    }
    return AOutCopies > 0;
}
bool GetCircleGapFreeRegions(const CetTNestItemVector &AOriginalItems, const TetNestOptions &AOptions, const TetClusterCandidate &ACandidate, const TetCircleGapWindow &AWindow, std::vector<TetClusterFreeRegion> &AOutRegions)
{
    ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
    std::vector<TetClusterFreeRegion> FreeRegions;
    return Geometry.ExtractCandidateFreeRegions(AOriginalItems, AOptions, ACandidate, FreeRegions) && ClipFreeRegionsToGapWindow(FreeRegions, AWindow, AOutRegions);
}
std::string BuildFreeRegionClassKey(const TetClusterFreeRegion &ARegion)
{
    const double LongSide = std::max(ARegion.Width, ARegion.Height);
    const double ShortSide = std::min(ARegion.Width, ARegion.Height);
    const long long AspectBucket = LongSide > CET_RECTANGLE_FILL_POSITION_TOLERANCE ? std::llround(ShortSide / LongSide * 20.0) : 0;
    const std::size_t VertexBucket = std::min<std::size_t>(16, std::max<std::size_t>(1, ARegion.Contour.size() / 4));
    const std::size_t HoleBucket = std::min<std::size_t>(3, ARegion.Holes.size());
    return std::string("free-") + (ARegion.IsClosed ? "closed" : "open") + "-h" + std::to_string(HoleBucket) + "-v" + std::to_string(VertexBucket) + "-r" + std::to_string(AspectBucket);
}
std::vector<TetCircleGapWindow> CollectFreeRegionTemplateWindows(const CetTNestItemVector &AOriginalItems, const TetNestOptions &AOptions, const TetClusterCandidate &ACandidate)
{
    ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
    std::vector<TetClusterFreeRegion> Regions;
    if (!Geometry.ExtractCandidateFreeRegions(AOriginalItems, AOptions, ACandidate, Regions))
        return {};
    std::stable_sort(Regions.begin(), Regions.end(), [](const TetClusterFreeRegion &A, const TetClusterFreeRegion &B) {
        if (A.IsClosed != B.IsClosed)
            return A.IsClosed;
        if (A.Holes.size() != B.Holes.size())
            return A.Holes.size() > B.Holes.size();
        if (std::abs(A.Area - B.Area) > 1.0)
            return A.Area > B.Area;
        if (std::abs(A.Width - B.Width) > 1.0)
            return A.Width > B.Width;
        return A.Height > B.Height;
    });
    std::vector<TetCircleGapWindow> Windows;
    Windows.reserve(std::min(Regions.size(), CET_FREE_REGION_TEMPLATE_MAX_WINDOWS));
    for (const TetClusterFreeRegion &Region : Regions) {
        if (Region.Area <= 0.0 || Region.Width <= CET_RECTANGLE_FILL_POSITION_TOLERANCE * 2.0 || Region.Height <= CET_RECTANGLE_FILL_POSITION_TOLERANCE * 2.0)
            continue;
        Windows.push_back({(Region.MinX + Region.MaxX) * 0.5, (Region.MinY + Region.MaxY) * 0.5, 0.0, Region.Width * 0.5, Region.Height * 0.5, BuildFreeRegionClassKey(Region)});
        if (Windows.size() == CET_FREE_REGION_TEMPLATE_MAX_WINDOWS)
            break;
    }
    return Windows;
}
bool FitsCircleGapRegion(const TetShapeFeature &AFeature, const std::vector<TetClusterFreeRegion> &ARegions)
{
    for (const TetClusterFreeRegion &Region : ARegions) {
        const bool FitsBounds = (AFeature.Width <= Region.Width && AFeature.Height <= Region.Height) || (AFeature.Height <= Region.Width && AFeature.Width <= Region.Height);
        if (AFeature.Area <= Region.Area + 1.0 && FitsBounds)
            return true;
    }
    return false;
}
std::vector<int> CollectCircleGapFillers(const std::vector<TetShapeFeature> &AFeatures, const TetClusterCandidate &ACandidate, const std::vector<TetClusterFreeRegion> &ARegions)
{
    std::vector<int> Fillers;
    for (int Index = 0; Index < static_cast<int>(AFeatures.size()); ++Index) {
        if (!ContainsOriginalIndex(ACandidate, Index) && AFeatures[Index].Area > 0.0 && FitsCircleGapRegion(AFeatures[Index], ARegions))
            Fillers.push_back(Index);
    }
    std::stable_sort(Fillers.begin(), Fillers.end(), [&](int A, int B) {
        const bool AIsCircle = AFeatures[A].ShapeType == MetShapeType::CircleLike;
        const bool BIsCircle = AFeatures[B].ShapeType == MetShapeType::CircleLike;
        // A circular frame's most constrained pocket is normally the one shared
        // by several of its members. Try its smallest circular inventory first;
        // larger or non-circular parts can still use the remaining edge regions.
        if (AIsCircle != BIsCircle)
            return AIsCircle;
        if (AIsCircle && std::abs(AFeatures[A].Area - AFeatures[B].Area) > 1.0) {
            return AFeatures[A].Area < AFeatures[B].Area;
        }
        return AFeatures[A].Area > AFeatures[B].Area;
    });
    if (Fillers.size() <= CET_CIRCLE_GAP_SEARCH_MAX_CANDIDATES)
        return Fillers;
    std::vector<int> Bounded;
    Bounded.reserve(CET_CIRCLE_GAP_SEARCH_MAX_CANDIDATES);
    std::set<std::uint64_t> Families;
    for (int Index : Fillers) {
        if (Families.insert(MakeFillerFamilyKey(AFeatures[Index])).second) {
            Bounded.push_back(Index);
            if (Bounded.size() == CET_CIRCLE_GAP_SEARCH_MAX_CANDIDATES)
                return Bounded;
        }
    }
    for (int Index : Fillers) {
        if (Bounded.size() == CET_CIRCLE_GAP_SEARCH_MAX_CANDIDATES)
            break;
        if (std::find(Bounded.begin(), Bounded.end(), Index) == Bounded.end())
            Bounded.push_back(Index);
    }
    return Bounded;
}
bool IsCircleGapStateBetter(const TetClusterFillSearchState &AFirst, const TetClusterFillSearchState &ASecond, const std::vector<TetShapeFeature> &AFeatures)
{
    if (AFirst.FillerCount > 0 && ASecond.FillerCount > 0 && !AFirst.Candidate.Transforms.empty() && !ASecond.Candidate.Transforms.empty()) {
        const int FirstIndex = AFirst.Candidate.Transforms.back().OriginalId;
        const int SecondIndex = ASecond.Candidate.Transforms.back().OriginalId;
        if (FirstIndex >= 0 && FirstIndex < static_cast<int>(AFeatures.size()) && SecondIndex >= 0 && SecondIndex < static_cast<int>(AFeatures.size())) {
            const bool FirstIsCircle = AFeatures[FirstIndex].ShapeType == MetShapeType::CircleLike;
            const bool SecondIsCircle = AFeatures[SecondIndex].ShapeType == MetShapeType::CircleLike;
            if (FirstIsCircle != SecondIsCircle)
                return FirstIsCircle;
        }
    }
    if (AFirst.FillerCount != ASecond.FillerCount)
        return AFirst.FillerCount > ASecond.FillerCount;
    if (AFirst.FillerCount > 0 && ASecond.FillerCount > 0 && std::abs(AFirst.Candidate.Score - ASecond.Candidate.Score) > 1e-9) {
        return AFirst.Candidate.Score > ASecond.Candidate.Score;
    }
    if (std::abs(AFirst.Candidate.RealArea - ASecond.Candidate.RealArea) > 1.0) {
        return AFirst.Candidate.RealArea > ASecond.Candidate.RealArea;
    }
    if (std::abs(AFirst.Candidate.ProxyWasteArea - ASecond.Candidate.ProxyWasteArea) > 1.0) {
        return AFirst.Candidate.ProxyWasteArea < ASecond.Candidate.ProxyWasteArea;
    }
    if (std::abs(AFirst.Candidate.Score - ASecond.Candidate.Score) > 1e-9) {
        return AFirst.Candidate.Score > ASecond.Candidate.Score;
    }
    return AFirst.Candidate.OriginalIndices < ASecond.Candidate.OriginalIndices;
}
void TrimCircleGapBeam(std::vector<TetClusterFillSearchState> &AStates, const std::vector<TetShapeFeature> &AFeatures)
{
    std::map<std::string, TetClusterFillSearchState> Unique;
    for (const TetClusterFillSearchState &State : AStates) {
        const std::string Key = MakeFilledVariantKey(State.Candidate);
        auto It = Unique.find(Key);
        if (It == Unique.end() || IsCircleGapStateBetter(State, It->second, AFeatures))
            Unique[Key] = State;
    }
    AStates.clear();
    for (const auto &Entry : Unique)
        AStates.push_back(Entry.second);
    std::stable_sort(AStates.begin(), AStates.end(), [&](const TetClusterFillSearchState &A, const TetClusterFillSearchState &B) { return IsCircleGapStateBetter(A, B, AFeatures); });
    if (AStates.size() > CET_CIRCLE_GAP_SEARCH_BEAM_WIDTH)
        AStates.resize(CET_CIRCLE_GAP_SEARCH_BEAM_WIDTH);
}
bool CircleGapSearchTimeReached(const std::chrono::steady_clock::time_point &AStart, long long ALimitMs) { return ALimitMs > 0 && std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - AStart).count() >= ALimitMs; }
bool AreLocalGapFillersInsideBaseOutline(const CetTNestItemVector &AOriginalItems, const TetClusterCandidate &ABaseCandidate, const TetClusterCandidate &ACandidate);
bool SearchCircleGapTemplate(const TetClusterFillContext &AContext, const TetCircleGapWindow &AWindow, const TetClusterCandidate &ACurrentCandidate, std::size_t AMaxDepth, TetClusterCandidate &AOutCandidate)
{
    const auto &AOriginalItems = AContext.OriginalItems; const auto &AFeatures = AContext.Features; const auto &AOptions = AContext.Options; const auto &ABaseCandidate = AContext.BaseCandidate; const auto &AEnvelopeCandidate = AContext.EnvelopeCandidate;
    const auto SearchStart = std::chrono::steady_clock::now();
    std::size_t Attempts = 0;
    std::size_t RegionCount = 0;
    std::size_t CandidateCount = 0;
    std::size_t ExactRejectCount = 0;
    std::set<std::uint64_t> TriedFamilies;
    double LargestRegionArea = 0.0;
    std::vector<TetClusterFillSearchState> Beam{{ACurrentCandidate, 0}};
    TetClusterFillSearchState Best{ACurrentCandidate, 0};
    ET::NEST2DMANAGERLIB::CetRectangleFillClusterBuilder Builder;
    for (std::size_t Depth = 0; Depth < AMaxDepth && !Beam.empty(); ++Depth) {
        std::vector<TetClusterFillSearchState> NextBeam;
        for (const TetClusterFillSearchState &State : Beam) {
            std::vector<TetClusterFreeRegion> LocalRegions;
            if (!GetCircleGapFreeRegions(AOriginalItems, AOptions, State.Candidate, AWindow, LocalRegions))
                continue;
            RegionCount += LocalRegions.size();
            for (const TetClusterFreeRegion &Region : LocalRegions)
                LargestRegionArea = std::max(LargestRegionArea, Region.Area);
            const std::vector<int> Fillers = CollectCircleGapFillers(AFeatures, State.Candidate, LocalRegions);
            CandidateCount = std::max(CandidateCount, Fillers.size());
            for (int Index : Fillers) {
                const std::uint64_t Family = MakeFillerFamilyKey(AFeatures[Index]);
                const bool MustTryFamily = TriedFamilies.size() < CET_CIRCLE_GAP_SEARCH_MIN_FAMILY_ATTEMPTS && TriedFamilies.find(Family) == TriedFamilies.end();
                if (Attempts >= CET_CIRCLE_GAP_SEARCH_MAX_ATTEMPTS || (CircleGapSearchTimeReached(SearchStart, CET_CIRCLE_GAP_SEARCH_MAX_TIME_MS) && !MustTryFamily))
                    break;
                ++Attempts;
                TriedFamilies.insert(Family);
                std::vector<TetClusterCandidate> PlacementCandidates;
                const TetRectangleFillRequest FillRequest{AOriginalItems, AFeatures, AOptions, ABaseCandidate, AEnvelopeCandidate, State.Candidate, &LocalRegions, Index, AEnvelopeCandidate.ClusterWidth, AEnvelopeCandidate.ClusterHeight};
                Builder.BuildFillerVariantsInRectangleEnvelope(FillRequest, CET_CIRCLE_GAP_SEARCH_BEAM_WIDTH, PlacementCandidates);
                if (PlacementCandidates.empty()) {
                    ++ExactRejectCount;
                    continue;
                }
                const char *Shape = AFeatures[Index].ShapeType == MetShapeType::CircleLike ? "Circle" : (AFeatures[Index].ShapeType == MetShapeType::EllipseLike ? "Ellipse" : (AFeatures[Index].ShapeType == MetShapeType::TriangleLike ? "Triangle" : "Other"));
                for (TetClusterCandidate &Candidate : PlacementCandidates) {
                    if (!AreLocalGapFillersInsideBaseOutline(AOriginalItems, ABaseCandidate, Candidate)) {
                        ++ExactRejectCount;
                        continue;
                    }
                    TetClusterFillSearchState Next{std::move(Candidate), State.FillerCount + 1};
                    std::cout << "[GAP][PLACE] Class=" << AWindow.ClassKey << " Id=" << Index << " Shape=" << Shape << " Area=" << AFeatures[Index].Area << " Depth=" << Next.FillerCount << std::endl;
                    if (IsCircleGapStateBetter(Next, Best, AFeatures))
                        Best = Next;
                    NextBeam.push_back(std::move(Next));
                }
            }
        }
        TrimCircleGapBeam(NextBeam, AFeatures);
        Beam = std::move(NextBeam);
        if (Attempts >= CET_CIRCLE_GAP_SEARCH_MAX_ATTEMPTS || (CircleGapSearchTimeReached(SearchStart, CET_CIRCLE_GAP_SEARCH_MAX_TIME_MS) && TriedFamilies.size() >= CET_CIRCLE_GAP_SEARCH_MIN_FAMILY_ATTEMPTS))
            break;
    }
    const bool TimeReached = CircleGapSearchTimeReached(SearchStart, CET_CIRCLE_GAP_SEARCH_MAX_TIME_MS);
    const char *StopReason = Best.FillerCount > 0 ? "filled" : (RegionCount == 0 ? "no-free-region" : (CandidateCount == 0 ? "no-compatible-filler" : (TimeReached ? "time-limit" : "no-exact-position")));
    std::cout << "[GAP][FRAME SEARCH] Class=" << AWindow.ClassKey << " Regions=" << RegionCount << " LargestArea=" << LargestRegionArea << " Candidates=" << CandidateCount << " Families=" << TriedFamilies.size() << " Attempts=" << Attempts << " ExactRejects=" << ExactRejectCount << " Fillers=" << Best.FillerCount << " Stop=" << StopReason << " Ms=" << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - SearchStart).count() << std::endl;
    AOutCandidate = std::move(Best.Candidate);
    return Best.FillerCount > 0;
}
bool IsContourInsideGapRegions(const CetPath &AContour, const std::vector<TetClusterFreeRegion> &ARegions)
{
    ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
    for (const TetClusterFreeRegion &Region : ARegions) {
        if (Geometry.IsContourInsideFreeRegion(AContour, Region, std::max(1.0, Region.Area * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE)))
            return true;
    }
    return false;
}
bool TryCopyCircleGapTemplateAtAngle(const TetClusterFillContext &AContext, const TetCircleGapTemplate &ATemplate, const TetCircleGapWindow &ATarget, double ADelta, TetClusterCandidate &AInOutCandidate)
{
    const auto &AOriginalItems = AContext.OriginalItems; const auto &AFeatures = AContext.Features; const auto &AOptions = AContext.Options; const auto &ABaseCandidate = AContext.BaseCandidate; const auto &AEnvelopeCandidate = AContext.EnvelopeCandidate;
    ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
    ET::NEST2DMANAGERLIB::CetRectangleFillClusterBuilder Builder;
    const std::vector<double> Allowed = ET::NEST2DMANAGERLIB::CetRotationUtils::BuildAllowedRotations(AOptions.Rotations);
    if (ATemplate.Transforms.empty() || Allowed.empty())
        return false;
    const double Cosine = std::cos(ADelta), Sine = std::sin(ADelta);
    for (const TetItemTransform &Source : ATemplate.Transforms) {
        int CopyIndex = -1;
        const std::uint64_t Family = MakeFillerFamilyKey(AFeatures[Source.OriginalId]);
        for (int Index = 0; Index < static_cast<int>(AFeatures.size()); ++Index) {
            if (!ContainsOriginalIndex(AInOutCandidate, Index) && MakeFillerFamilyKey(AFeatures[Index]) == Family) {
                CopyIndex = Index;
                break;
            }
        }
        if (CopyIndex < 0)
            return false;
        const CetPath SourceContour = Geometry.TransformContour(Geometry.GetIdentityContour(AOriginalItems[Source.OriginalId]), Source.RelativeRotation, Source.RelativeX, Source.RelativeY);
        double MinX = 0.0, MinY = 0.0, MaxX = 0.0, MaxY = 0.0;
        if (!Geometry.GetBounds(SourceContour, MinX, MinY, MaxX, MaxY))
            return false;
        const double OffsetX = (MinX + MaxX) * 0.5 - ATemplate.Source.CenterX;
        const double OffsetY = (MinY + MaxY) * 0.5 - ATemplate.Source.CenterY;
        std::vector<double> Rotations = Allowed;
        const double Desired = Source.RelativeRotation + ADelta;
        std::stable_sort(Rotations.begin(), Rotations.end(), [&](double A, double B) { return std::abs(A - Desired) < std::abs(B - Desired); });
        bool Copied = false;
        for (double Rotation : Rotations) {
            const CetPath Rotated = Geometry.TransformContour(Geometry.GetIdentityContour(AOriginalItems[CopyIndex]), Rotation, 0.0, 0.0);
            if (!Geometry.GetBounds(Rotated, MinX, MinY, MaxX, MaxY))
                continue;
            TetItemTransform Copy;
            Copy.OriginalId = CopyIndex;
            Copy.RelativeRotation = Rotation;
            Copy.RelativeX = ATarget.CenterX + OffsetX * Cosine - OffsetY * Sine - (MinX + MaxX) * 0.5;
            Copy.RelativeY = ATarget.CenterY + OffsetX * Sine + OffsetY * Cosine - (MinY + MaxY) * 0.5;
            std::vector<TetClusterFreeRegion> LocalRegions;
            const CetPath Contour = Geometry.TransformContour(Rotated, 0.0, Copy.RelativeX, Copy.RelativeY);
            TetClusterCandidate Next;
            if (GetCircleGapFreeRegions(AOriginalItems, AOptions, AInOutCandidate, ATarget, LocalRegions) && IsContourInsideGapRegions(Contour, LocalRegions) && Builder.TryAppendFillerTemplateInRectangleEnvelope({AOriginalItems, AFeatures, AOptions, ABaseCandidate, AEnvelopeCandidate, AInOutCandidate, &LocalRegions, Copy.OriginalId, AEnvelopeCandidate.ClusterWidth, AEnvelopeCandidate.ClusterHeight}, Copy, Next)) {
                AInOutCandidate = std::move(Next);
                Copied = true;
                break;
            }
        }
        if (!Copied)
            return false;
    }
    return true;
}
bool CopyCircleGapTemplate(const TetClusterFillContext &AContext, const TetCircleGapTemplate &ATemplate, const TetCircleGapWindow &ATarget, TetClusterCandidate &AInOutCandidate)
{
    const auto &AOriginalItems = AContext.OriginalItems; const auto &AFeatures = AContext.Features; const auto &AOptions = AContext.Options; const auto &ABaseCandidate = AContext.BaseCandidate; const auto &AEnvelopeCandidate = AContext.EnvelopeCandidate;
    if (ATemplate.Transforms.empty() || ATemplate.Source.ClassKey != ATarget.ClassKey)
        return false;
    std::vector<double> Deltas{ATarget.Angle - ATemplate.Source.Angle};
    if (ATemplate.Source.ClassKey.rfind("triple-", 0) == 0) {
        Deltas.push_back(Deltas.front() + CET_CLUSTER_PI);
    }
    for (double Delta : Deltas) {
        TetClusterCandidate Candidate = AInOutCandidate;
        if (TryCopyCircleGapTemplateAtAngle(AContext, ATemplate, ATarget, Delta, Candidate)) {
            AInOutCandidate = std::move(Candidate);
            return true;
        }
    }
    return false;
}
void CollectCircleGapGroups(const CetTNestItemVector &AOriginalItems, const TetNestOptions &AOptions, const TetClusterCandidate &ACandidate, const std::vector<TetCircleGapWindow> &AWindows, std::vector<std::vector<TetCircleGapWindow>> &AOutGroups)
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
void AppendCircleQuadGapWindows(const std::vector<TetCircleCenter> &ACenters, const std::vector<std::vector<std::size_t>> &ANeighbors, std::vector<TetCircleGapWindow> &AWindows)
{
    std::set<std::string> Seen;
    for (std::size_t First = 0; First < ANeighbors.size(); ++First) {
        const std::vector<std::size_t> &Local = ANeighbors[First];
        for (std::size_t Left = 0; Left < Local.size(); ++Left) {
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
                    double MinX = ACenters[First].X, MaxX = MinX;
                    double MinY = ACenters[First].Y, MaxY = MinY;
                    double Radius = ACenters[First].Radius;
                    for (std::size_t Index : Indices) {
                        MinX = std::min(MinX, ACenters[Index].X);
                        MaxX = std::max(MaxX, ACenters[Index].X);
                        MinY = std::min(MinY, ACenters[Index].Y);
                        MaxY = std::max(MaxY, ACenters[Index].Y);
                        Radius = std::min(Radius, ACenters[Index].Radius);
                    }
                    const double HalfWidth = std::max(CET_RECTANGLE_FILL_POSITION_TOLERANCE * 2.0, (MaxX - MinX) * CET_CIRCLE_GAP_INTERIOR_WINDOW_SPAN_RATIO);
                    const double HalfHeight = std::max(CET_RECTANGLE_FILL_POSITION_TOLERANCE * 2.0, (MaxY - MinY) * CET_CIRCLE_GAP_INTERIOR_WINDOW_SPAN_RATIO);
                    if (HalfWidth > CET_RECTANGLE_FILL_POSITION_TOLERANCE && HalfHeight > CET_RECTANGLE_FILL_POSITION_TOLERANCE) {
                        AWindows.push_back({(MinX + MaxX) * 0.5, (MinY + MaxY) * 0.5, 0.0, HalfWidth, HalfHeight, BuildCircleGapClassKey("quad-circle", HalfWidth, HalfHeight, Radius)});
                    }
                }
            }
        }
    }
}
bool AreLocalGapFillersInsideBaseOutline(const CetTNestItemVector &AOriginalItems, const TetClusterCandidate &ABaseCandidate, const TetClusterCandidate &ACandidate)
{
    if (ACandidate.Transforms.size() <= ABaseCandidate.Transforms.size() || ABaseCandidate.ProxyContour.size() < 3)
        return false;
    ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
    const double Tolerance = std::max(1.0, ABaseCandidate.ProxyArea * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE);
    for (std::size_t Index = ABaseCandidate.Transforms.size(); Index < ACandidate.Transforms.size(); ++Index) {
        const TetItemTransform &Transform = ACandidate.Transforms[Index];
        if (Transform.OriginalId < 0 || Transform.OriginalId >= static_cast<int>(AOriginalItems.size()))
            return false;
        const CetPath Contour = Geometry.TransformContour(Geometry.GetIdentityContour(AOriginalItems[Transform.OriginalId]), Transform.RelativeRotation, Transform.RelativeX, Transform.RelativeY);
        if (!Geometry.IsContourFullyContained(Contour, ABaseCandidate.ProxyContour, Tolerance))
            return false;
    }
    return true;
}
bool BuildCompleteCircleGapTemplate(const TetClusterFillContext &AContext, const TetClusterFillSearchState &ACurrent, const TetCircleGapWindow &AWindow, TetClusterCandidate &AOutCandidate, TetCircleGapTemplate &AOutTemplate)
{
    const auto &AOriginalItems = AContext.OriginalItems; const auto &ABaseCandidate = AContext.BaseCandidate;
    AOutCandidate = TetClusterCandidate{};
    AOutTemplate = TetCircleGapTemplate{};
    const std::size_t Remaining = AOriginalItems.size() > ACurrent.Candidate.Transforms.size() ? AOriginalItems.size() - ACurrent.Candidate.Transforms.size() : 0;
    if (Remaining == 0)
        return false;
    const std::size_t Start = ACurrent.Candidate.Transforms.size();
    const std::size_t MaxDepth = std::min(Remaining, CET_CIRCLE_GAP_FILL_MAX_TEMPLATE_DEPTH);
    if (!SearchCircleGapTemplate(AContext, AWindow, ACurrent.Candidate, MaxDepth, AOutCandidate))
        return false;
    AOutTemplate.Source = AWindow;
    AOutTemplate.Transforms.assign(AOutCandidate.Transforms.begin() + static_cast<std::ptrdiff_t>(Start), AOutCandidate.Transforms.end());
    return !AOutTemplate.Transforms.empty();
}
bool BuildLocalCircleGapFilledCandidate(const TetClusterFillContext &AContext, std::map<std::string, TetCircleGapTemplate> &ATemplates, TetClusterCandidate &AOutCandidate)
{
    const auto &AOriginalItems = AContext.OriginalItems; const auto &AFeatures = AContext.Features; const auto &AOptions = AContext.Options; const auto &ABaseCandidate = AContext.BaseCandidate; const auto &AEnvelopeCandidate = AContext.EnvelopeCandidate;
    AOutCandidate = AEnvelopeCandidate;
    const auto SearchStart = std::chrono::steady_clock::now();
    const std::vector<TetCircleGapWindow> Windows = CollectCircleGapWindows(AOriginalItems, AFeatures, ABaseCandidate, AEnvelopeCandidate);
    ATemplates.clear();
    TetClusterFillSearchState Current{AOutCandidate, 0};
    while (!CircleGapSearchTimeReached(SearchStart, CET_CIRCLE_GAP_TOTAL_SEARCH_MAX_TIME_MS)) {
        std::vector<std::vector<TetCircleGapWindow>> Groups;
        CollectCircleGapGroups(AOriginalItems, AOptions, Current.Candidate, Windows, Groups);
        bool FilledBatch = false;
        for (const std::vector<TetCircleGapWindow> &Group : Groups) {
            if (Group.empty() || CircleGapSearchTimeReached(SearchStart, CET_CIRCLE_GAP_TOTAL_SEARCH_MAX_TIME_MS))
                break;
            TetClusterCandidate Candidate;
            TetCircleGapTemplate Template;
            if (!BuildCompleteCircleGapTemplate(AContext, Current, Group.front(), Candidate, Template))
                continue;
            const std::size_t TemplateFillerCount = Candidate.Transforms.size() - ABaseCandidate.Transforms.size();
            Current = {std::move(Candidate), TemplateFillerCount};
            std::size_t Copies = 0;
            for (std::size_t Index = 1; Index < Group.size(); ++Index) {
                TetClusterCandidate Copied = Current.Candidate;
                if (!CopyCircleGapTemplate(AContext, Template, Group[Index], Copied))
                    break;
                const std::size_t CopiedFillerCount = Copied.Transforms.size() - ABaseCandidate.Transforms.size();
                Current = {std::move(Copied), CopiedFillerCount};
                ++Copies;
            }
            std::cout << "[TEMPLATE][CIRCLE COMPLETE TEMPLATE] Class=" << Template.Source.ClassKey << " Fillers=" << Template.Transforms.size() << " Copies=" << Copies << std::endl;
            FilledBatch = true;
        }
        if (!FilledBatch)
            break;
    }
    if (!AreLocalGapFillersInsideBaseOutline(AOriginalItems, ABaseCandidate, Current.Candidate)) {
        std::cout << "[TEMPLATE][LOCAL GAP REJECT] Reason=OuterOutline" << std::endl;
        return false;
    }
    Current.Candidate.BuilderName = "FixedOutlineGapFill";
    Current.Candidate.ClusterType = ABaseCandidate.ClusterType + "_InnerFill";
    AOutCandidate = std::move(Current.Candidate);
    return true;
}
bool BuildLocalFreeRegionTemplateCandidate(const TetClusterFillContext &AContext, const TetClusterCandidate &ASeedCandidate, TetClusterCandidate &AOutCandidate)
{
    const auto &AOriginalItems = AContext.OriginalItems; const auto &AFeatures = AContext.Features; const auto &AOptions = AContext.Options; const auto &ABaseCandidate = AContext.BaseCandidate; const auto &AEnvelopeCandidate = AContext.EnvelopeCandidate;
    AOutCandidate = TetClusterCandidate{};
    if (!ASeedCandidate.Valid || !PreservesBaseTransforms(ABaseCandidate, ASeedCandidate))
        return false;
    const auto SearchStart = std::chrono::steady_clock::now();
    TetClusterFillSearchState Current{ASeedCandidate, 0};
    std::size_t TemplateCount = 0;
    std::size_t CopyCount = 0;
    while (!CircleGapSearchTimeReached(SearchStart, CET_FREE_REGION_TEMPLATE_TOTAL_SEARCH_MAX_MS)) {
        const std::vector<TetCircleGapWindow> Windows = CollectFreeRegionTemplateWindows(AOriginalItems, AOptions, Current.Candidate);
        if (Windows.empty())
            break;
        std::vector<std::vector<TetCircleGapWindow>> Groups;
        CollectCircleGapGroups(AOriginalItems, AOptions, Current.Candidate, Windows, Groups);
        bool FilledBatch = false;
        for (const std::vector<TetCircleGapWindow> &Group : Groups) {
            if (Group.empty() || CircleGapSearchTimeReached(SearchStart, CET_FREE_REGION_TEMPLATE_TOTAL_SEARCH_MAX_MS))
                break;
            TetClusterCandidate Candidate;
            TetCircleGapTemplate Template;
            if (!BuildCompleteCircleGapTemplate(AContext, Current, Group.front(), Candidate, Template))
                continue;
            if (!AreLocalGapFillersInsideBaseOutline(AOriginalItems, ABaseCandidate, Candidate))
                continue;
            Current = {std::move(Candidate), Current.FillerCount + Template.Transforms.size()};
            ++TemplateCount;
            for (std::size_t Index = 1; Index < Group.size(); ++Index) {
                TetClusterCandidate Copied = Current.Candidate;
                if (!CopyCircleGapTemplate(AContext, Template, Group[Index], Copied) || !AreLocalGapFillersInsideBaseOutline(AOriginalItems, ABaseCandidate, Copied))
                    break;
                Current = {std::move(Copied), Current.FillerCount + Template.Transforms.size()};
                ++CopyCount;
            }
            std::cout << "[GAP][FREE REGION TEMPLATE] Class=" << Template.Source.ClassKey << " Fillers=" << Template.Transforms.size() << " Copies=" << CopyCount << std::endl;
            FilledBatch = true;
        }
        if (!FilledBatch)
            break;
    }
    const std::size_t Added = Current.Candidate.Transforms.size() > ASeedCandidate.Transforms.size() ? Current.Candidate.Transforms.size() - ASeedCandidate.Transforms.size() : 0;
    const char *StopReason = Added > 0 ? "filled" : "no-compatible-template";
    std::cout << "[GAP][FREE REGION SUMMARY] Windows=" << CET_FREE_REGION_TEMPLATE_MAX_WINDOWS << " Templates=" << TemplateCount << " Copies=" << CopyCount << " Added=" << Added << " Stop=" << StopReason << " Ms=" << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - SearchStart).count() << std::endl;
    if (Added == 0 || !AreLocalGapFillersInsideBaseOutline(AOriginalItems, ABaseCandidate, Current.Candidate))
        return false;
    Current.Candidate.BuilderName = "FixedOutlineGapFill";
    Current.Candidate.ClusterType = ABaseCandidate.ClusterType + "_FreeRegionFill";
    AOutCandidate = std::move(Current.Candidate);
    return true;
}

} // anonymous namespace

CetCircleGapFiller::CetCircleGapFiller() : CetCoreObject() {}
CetCircleGapFiller::~CetCircleGapFiller() {}

bool CetCircleGapFiller::BuildCircleGapCandidate(const TetClusterFillContext& AContext, std::map<std::string, TetCircleGapTemplate>& ATemplates, TetClusterCandidate& AOutCandidate)
{
    return BuildLocalCircleGapFilledCandidate(AContext, ATemplates, AOutCandidate);
}

std::vector<TetCircleGapTemplateAnchor> CetCircleGapFiller::CollectCircleGapTemplateAnchors(const CetTNestItemVector& AItems, const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ACandidate)
{
    return _CollectCircleGapTemplateAnchors(AItems, AFeatures, ACandidate);
}

bool CetCircleGapFiller::CopyCircleGapTemplate(const TetClusterFillContext& AContext, const std::vector<TetCircleGapTemplateAnchor>& AAnchors, const TetClusterCandidate& ASeedCandidate, std::size_t AMaxCopies, TetClusterCandidate& AOutCandidate, std::size_t& AOutCopies)
{
    return TryCopyCircleGapTemplate(AContext, AAnchors, ASeedCandidate, AMaxCopies, AOutCandidate, AOutCopies);
}

bool CetCircleGapFiller::BuildFreeRegionTemplateCandidate(const TetClusterFillContext& AContext, const TetClusterCandidate& ASeedCandidate, TetClusterCandidate& AOutCandidate)
{
    return BuildLocalFreeRegionTemplateCandidate(AContext, ASeedCandidate, AOutCandidate);
}

}}
