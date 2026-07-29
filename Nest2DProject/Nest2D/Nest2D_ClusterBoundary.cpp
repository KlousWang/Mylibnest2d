#include "pch.h"
#include "Nest2D_ClusterBoundary.h"
#include "Nest2D_ClusterGeometryHelper.h"
#include "NestUtils.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace ET {
    namespace NEST2DMANAGERLIB {

        namespace {
            constexpr int CET_CLUSTER_BOUNDARY_OFFSET_ATTEMPTS = 5;
            constexpr double CET_CLUSTER_BOUNDARY_AREA_TOLERANCE = 16.0;
            constexpr double CET_CLUSTER_BOUNDARY_RELATIVE_AREA_TOLERANCE = 1e-10;
        }

        CetClusterBoundary::CetClusterBoundary() : CetCoreObject()
        {
        }

        CetClusterBoundary::~CetClusterBoundary()
        {
        }

        bool CetClusterBoundary::BuildBoundary(const CetTNestItemVector& AOriginalItems, const std::vector<TetItemTransform>& ATransforms, CetPath& AOutBoundary)
        {
            AOutBoundary.clear();

            TetNestOptions DefaultOptions;
            TetClusterBoundaryResult Result;
            if (!BuildBoundaryWithResult(AOriginalItems, ATransforms, DefaultOptions, Result)){
                return false;
            }

            AOutBoundary = std::move(Result.Boundary);
            return true;
        }

        bool CetClusterBoundary::BuildBoundaryWithResult(const CetTNestItemVector& AOriginalItems, const std::vector<TetItemTransform>& ATransforms, const TetNestOptions& AOptions, TetClusterBoundaryResult& AOutResult)
        {
            AOutResult = TetClusterBoundaryResult{};

            ClipperLib::Paths Contours;
            if (!_CollectTransformedContours(AOriginalItems, ATransforms, Contours)){
                return false;
            }

            CetPath Boundary;
            if (_BuildExactUnionBoundary(Contours, Boundary) && _BoundaryContainsAllContours(Contours, Boundary, _GetContainmentAreaTolerance(Boundary))){
                AOutResult.Success = true;
                AOutResult.Boundary = std::move(Boundary);
                AOutResult.Mode = MetClusterProxyMode::ExactUnion;
                AOutResult.BoundaryArea = _CalculateBoundaryArea(AOutResult.Boundary);
                return true;
            }

            Boundary.clear();
            if (_BuildOffsetUnionBoundary(Contours, AOptions, Boundary) && _BoundaryContainsAllContours(Contours, Boundary, _GetContainmentAreaTolerance(Boundary))){
                AOutResult.Success = true;
                AOutResult.Boundary = std::move(Boundary);
                AOutResult.Mode = MetClusterProxyMode::OffsetUnion;
                AOutResult.BoundaryArea = _CalculateBoundaryArea(AOutResult.Boundary);
                return true;
            }

            Boundary.clear();
            if (_BuildConvexHullBoundary(Contours, Boundary) && _BoundaryContainsAllContours(Contours, Boundary, _GetContainmentAreaTolerance(Boundary))){
                AOutResult.Success = true;
                AOutResult.Boundary = std::move(Boundary);
                AOutResult.Mode = MetClusterProxyMode::ConvexHull;
                AOutResult.BoundaryArea = _CalculateBoundaryArea(AOutResult.Boundary);
                return true;
            }

            return false;
        }

        bool CetClusterBoundary::_CollectTransformedContours(const CetTNestItemVector& AOriginalItems, const std::vector<TetItemTransform>& ATransforms, ClipperLib::Paths& AOutContours)
        {
            AOutContours.clear();
            if (AOriginalItems.empty() || ATransforms.empty()){
                return false;
            }

            AOutContours.reserve(ATransforms.size());
            CetClusterGeometryHelper Geometry;

            for (const TetItemTransform& Transform : ATransforms){
                if (Transform.OriginalId < 0 || Transform.OriginalId >= static_cast<int>(AOriginalItems.size())){
                    return false;
                }

                if (!std::isfinite(Transform.RelativeX) || !std::isfinite(Transform.RelativeY) || !std::isfinite(Transform.RelativeRotation)){
                    return false;
                }

                CetPath Contour = Geometry.TransformContour(Geometry.GetIdentityContour(AOriginalItems[Transform.OriginalId]), Transform.RelativeRotation, Transform.RelativeX, Transform.RelativeY);

                const double ContourArea = std::abs(static_cast<double>(ClipperLib::Area(Contour)));
                if (Contour.size() < 3 || !std::isfinite(ContourArea) || ContourArea <= 0.0){
                    continue;
                }

                if (!ClipperLib::Orientation(Contour)){
                    std::reverse(Contour.begin(), Contour.end());
                }

                AOutContours.push_back(std::move(Contour));
            }

            return !AOutContours.empty();
        }

        bool CetClusterBoundary::_BuildExactUnionBoundary(const ClipperLib::Paths& AContours, CetPath& AOutBoundary)
        {
            return _BuildSingleOuterBoundary(AContours, AOutBoundary);
        }

        bool CetClusterBoundary::_BuildOffsetUnionBoundary(const ClipperLib::Paths& AContours, const TetNestOptions& AOptions, CetPath& AOutBoundary)
        {
            AOutBoundary.clear();
            if (AContours.empty()){
                return false;
            }

            double MinX = 0.0;
            double MinY = 0.0;
            double MaxX = 0.0;
            double MaxY = 0.0;
            if (!_GetContourBounds(AContours, MinX, MinY, MaxX, MaxY)){
                return false;
            }

            const double ClusterWidth = MaxX - MinX;
            const double ClusterHeight = MaxY - MinY;
            const double ClusterShortSide = std::max(1.0, std::min(ClusterWidth, ClusterHeight));
            const double SpacingCoord = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));
            const double InitialOffset = std::max(1.0, SpacingCoord * 0.5);
            const double SpacingLimitedOffset = SpacingCoord > 0.0 ? SpacingCoord * 3.0 : InitialOffset * 6.0;
            const double ShapeLimitedOffset = ClusterShortSide * 0.35;
            const double MaxOffset = std::max(InitialOffset, std::min(std::max(InitialOffset * 6.0, SpacingLimitedOffset), std::max(InitialOffset, ShapeLimitedOffset)));

            double OffsetDistance = InitialOffset;
            for (int AttemptIndex = 0; AttemptIndex < CET_CLUSTER_BOUNDARY_OFFSET_ATTEMPTS && OffsetDistance <= MaxOffset + 1.0; ++AttemptIndex){
                ClipperLib::Paths GrownContours;
                ClipperLib::ClipperOffset GrowOffset(2.0, std::max(1.0, OffsetDistance * 0.02));
                GrowOffset.AddPaths(AContours, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
                GrowOffset.Execute(GrownContours, OffsetDistance);
                ClipperLib::CleanPolygons(GrownContours, 1.0);
                if (GrownContours.empty()){
                    OffsetDistance *= 1.6;
                    continue;
                }

                CetPath GrownBoundary;
                if (!_BuildSingleOuterBoundary(GrownContours, GrownBoundary)){
                    OffsetDistance *= 1.6;
                    continue;
                }

                ClipperLib::Paths ShrunkContours;
                ClipperLib::ClipperOffset ShrinkOffset(2.0, std::max(1.0, OffsetDistance * 0.02));
                ShrinkOffset.AddPath(GrownBoundary, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
                ShrinkOffset.Execute(ShrunkContours, -OffsetDistance);
                ClipperLib::CleanPolygons(ShrunkContours, 1.0);

                CetPath CandidateBoundary;
                if (_BuildSingleOuterBoundary(ShrunkContours, CandidateBoundary) && _BoundaryContainsAllContours(AContours, CandidateBoundary, _GetContainmentAreaTolerance(CandidateBoundary))){
                    AOutBoundary = std::move(CandidateBoundary);
                    return true;
                }

                OffsetDistance *= 1.6;
            }

            return false;
        }

        bool CetClusterBoundary::_BuildSingleOuterBoundary(const ClipperLib::Paths& AContours, CetPath& AOutBoundary)
        {
            AOutBoundary.clear();
            if (AContours.empty()){
                return false;
            }

            ClipperLib::Paths UnionContours;
            ClipperLib::Clipper UnionClipper;
            if (!UnionClipper.AddPaths(AContours, ClipperLib::ptSubject, true)){
                return false;
            }

            if (!UnionClipper.Execute(ClipperLib::ctUnion, UnionContours, ClipperLib::pftNonZero, ClipperLib::pftNonZero)){
                return false;
            }

            std::vector<CetPath> OuterContours;
            for (CetPath UnionContour : UnionContours){
                if (UnionContour.size() < 3 || std::abs(static_cast<double>(ClipperLib::Area(UnionContour))) <= 0.0){
                    continue;
                }

                if (!ClipperLib::Orientation(UnionContour)){
                    continue;
                }

                OuterContours.push_back(std::move(UnionContour));
            }

            if (OuterContours.size() != 1){
                return false;
            }

            AOutBoundary = std::move(OuterContours.front());
            return _NormalizeBoundary(AOutBoundary);
        }

        bool CetClusterBoundary::_BuildConvexHullBoundary(const ClipperLib::Paths& AContours, CetPath& AOutBoundary)
        {
            AOutBoundary.clear();

            std::vector<CetInpoint> Points = _CollectContourPoints(AContours);
            if (!_SortUniquePoints(Points)){
                return false;
            }

            return _BuildConvexHullFromPoints(Points, AOutBoundary);
        }

        bool CetClusterBoundary::_BoundaryContainsAllContours(const ClipperLib::Paths& AContours, const CetPath& ABoundary, double AAreaTolerance)
        {
            if (AContours.empty() || ABoundary.size() < 3 || !std::isfinite(AAreaTolerance)){
                return false;
            }

            CetClusterGeometryHelper Geometry;
            for (const CetPath& Contour : AContours){
                if (!Geometry.IsContourFullyContained(Contour, ABoundary, AAreaTolerance)){
                    return false;
                }
            }

            return true;
        }

        bool CetClusterBoundary::_GetContourBounds(const ClipperLib::Paths& AContours, double& AOutMinX, double& AOutMinY, double& AOutMaxX, double& AOutMaxY)
        {
            if (AContours.empty()){
                return false;
            }

            bool HasPoint = false;
            AOutMinX = 0.0;
            AOutMinY = 0.0;
            AOutMaxX = 0.0;
            AOutMaxY = 0.0;

            for (const CetPath& Contour : AContours){
                for (const CetInpoint& Point : Contour){
                    const double PointX = static_cast<double>(Point.X);
                    const double PointY = static_cast<double>(Point.Y);
                    if (!HasPoint){
                        AOutMinX = AOutMaxX = PointX;
                        AOutMinY = AOutMaxY = PointY;
                        HasPoint = true;
                        continue;
                    }

                    AOutMinX = std::min(AOutMinX, PointX);
                    AOutMinY = std::min(AOutMinY, PointY);
                    AOutMaxX = std::max(AOutMaxX, PointX);
                    AOutMaxY = std::max(AOutMaxY, PointY);
                }
            }

            return HasPoint && AOutMaxX > AOutMinX && AOutMaxY > AOutMinY;
        }

        double CetClusterBoundary::_CalculateBoundaryArea(const CetPath& ABoundary)
        {
            if (ABoundary.size() < 3){
                return 0.0;
            }

            const double BoundaryArea = std::abs(static_cast<double>(ClipperLib::Area(ABoundary)));
            return std::isfinite(BoundaryArea) ? BoundaryArea : 0.0;
        }

        double CetClusterBoundary::_GetContainmentAreaTolerance(const CetPath& ABoundary)
        {
            const double BoundaryArea = _CalculateBoundaryArea(ABoundary);
            return std::max(CET_CLUSTER_BOUNDARY_AREA_TOLERANCE, BoundaryArea * CET_CLUSTER_BOUNDARY_RELATIVE_AREA_TOLERANCE);
        }

        std::vector<CetInpoint> CetClusterBoundary::_CollectContourPoints(const ClipperLib::Paths& AContours)
        {
            std::vector<CetInpoint> Points;
            for (const CetPath& Contour : AContours){
                for (const CetInpoint& Point : Contour){
                    Points.push_back(Point);
                }
            }
            return Points;
        }

        bool CetClusterBoundary::_SortUniquePoints(std::vector<CetInpoint>& APoints)
        {
            if (APoints.size() < 3){
                return false;
            }

            std::sort(APoints.begin(),APoints.end(),[](const CetInpoint& AFirstPoint, const CetInpoint& ASecondPoint){
                    if (AFirstPoint.X != ASecondPoint.X){
                        return AFirstPoint.X < ASecondPoint.X;
                    }
                    return AFirstPoint.Y < ASecondPoint.Y;
                });

            APoints.erase(std::unique(APoints.begin(),APoints.end(),[](const CetInpoint& AFirstPoint, const CetInpoint& ASecondPoint){
                        return AFirstPoint.X == ASecondPoint.X && AFirstPoint.Y == ASecondPoint.Y;
                    }),
                APoints.end());

            return APoints.size() >= 3;
        }

        bool CetClusterBoundary::_BuildConvexHullFromPoints(const std::vector<CetInpoint>& APoints, CetPath& AOutBoundary)
        {
            if (APoints.size() < 3){
                return false;
            }

            std::vector<CetInpoint> Hull(APoints.size() * 2);
            std::size_t HullSize = 0;

            for (const CetInpoint& Point : APoints){
                while (HullSize >= 2 && _CrossProduct(Hull[HullSize - 2], Hull[HullSize - 1], Point) <= 0.0L){
                    --HullSize;
                }
                Hull[HullSize] = Point;
                ++HullSize;
            }

            const std::size_t LowerHullSize = HullSize;
            for (std::size_t ReverseOffset = APoints.size(); ReverseOffset > 0; --ReverseOffset){
                const CetInpoint& Point = APoints[ReverseOffset - 1];
                while (HullSize > LowerHullSize && _CrossProduct(Hull[HullSize - 2], Hull[HullSize - 1], Point) <= 0.0L){
                    --HullSize;
                }
                Hull[HullSize] = Point;
                ++HullSize;
            }

            if (HullSize > 1){
                --HullSize;
            }

            Hull.resize(HullSize);
            if (Hull.size() < 3){
                return false;
            }

            AOutBoundary.assign(Hull.begin(), Hull.end());
            return _NormalizeBoundary(AOutBoundary);
        }

        bool CetClusterBoundary::_NormalizeBoundary(CetPath& ABoundary)
        {
            if (ABoundary.size() < 3){
                return false;
            }

            ClipperLib::CleanPolygon(ABoundary, 1.0);
            const double BoundaryArea = std::abs(static_cast<double>(ClipperLib::Area(ABoundary)));
            if (ABoundary.size() < 3 || !std::isfinite(BoundaryArea) || BoundaryArea <= 0.0){
                return false;
            }

            if (!ClipperLib::Orientation(ABoundary)){
                std::reverse(ABoundary.begin(), ABoundary.end());
            }

            return true;
        }

        long double CetClusterBoundary::_CrossProduct(const CetInpoint& AOrigin, const CetInpoint& AFirstPoint, const CetInpoint& ASecondPoint)
        {
            const long double FirstDeltaX = static_cast<long double>(AFirstPoint.X) - static_cast<long double>(AOrigin.X);
            const long double FirstDeltaY = static_cast<long double>(AFirstPoint.Y) - static_cast<long double>(AOrigin.Y);
            const long double SecondDeltaX = static_cast<long double>(ASecondPoint.X) - static_cast<long double>(AOrigin.X);
            const long double SecondDeltaY = static_cast<long double>(ASecondPoint.Y) - static_cast<long double>(AOrigin.Y);
            return FirstDeltaX * SecondDeltaY - FirstDeltaY * SecondDeltaX;
        }

    }
}
