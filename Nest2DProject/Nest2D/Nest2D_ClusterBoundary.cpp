#include "pch.h"
#include "Nest2D_ClusterBoundary.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace ET {
    namespace NEST2DMANAGERLIB {

        CetClusterBoundary::CetClusterBoundary() : CetCoreObject()
        {
        }

        CetClusterBoundary::~CetClusterBoundary()
        {
        }

        bool CetClusterBoundary::BuildBoundary(const CetTNestItemVector& AOriginalItems, const std::vector<TetItemTransform>& ATransforms, CetPath& AOutBoundary)
        {
            AOutBoundary.clear();

            ClipperLib::Paths Contours;
            if (!_CollectTransformedContours(AOriginalItems, ATransforms, Contours)) {
                return false;
            }

            if (_BuildUnionBoundary(Contours, AOutBoundary)) {
                return true;
            }

            return _BuildConvexHullBoundary(Contours, AOutBoundary);
        }

        CetPath CetClusterBoundary::_GetIdentityContour(const CetNestItem& AItem)
        {
            CetNestItem TempItem = AItem;
            TempItem.translation(libnest2d::Point(0, 0));
            TempItem.rotation(libnest2d::Radians(0.0));
            TempItem.inflation(0);
            return TempItem.transformedShape().Contour;
        }

        CetPath CetClusterBoundary::_TransformContour(const CetPath& AContour, double ARotation, double ATranslationX, double ATranslationY)
        {
            CetPath Result;
            Result.reserve(AContour.size());

            const double CosValue = std::cos(ARotation);
            const double SinValue = std::sin(ARotation);

            for (const CetInpoint& Point : AContour) {
                const double SourceX = static_cast<double>(Point.X);
                const double SourceY = static_cast<double>(Point.Y);
                Result.emplace_back(
                    static_cast<ClipperLib::cInt>(std::llround(SourceX * CosValue - SourceY * SinValue + ATranslationX)),
                    static_cast<ClipperLib::cInt>(std::llround(SourceX * SinValue + SourceY * CosValue + ATranslationY)));
            }

            return Result;
        }

        bool CetClusterBoundary::_CollectTransformedContours(const CetTNestItemVector& AOriginalItems, const std::vector<TetItemTransform>& ATransforms, ClipperLib::Paths& AOutContours)
        {
            AOutContours.clear();
            if (AOriginalItems.empty() || ATransforms.empty()) {
                return false;
            }

            AOutContours.reserve(ATransforms.size());

            for (const TetItemTransform& Transform : ATransforms) {
                if (Transform.OriginalId < 0 || Transform.OriginalId >= static_cast<int>(AOriginalItems.size())) {
                    return false;
                }

                if (!std::isfinite(Transform.RelativeX) || !std::isfinite(Transform.RelativeY) || !std::isfinite(Transform.RelativeRotation)) {
                    return false;
                }

                CetPath Contour = _TransformContour(
                    _GetIdentityContour(AOriginalItems[Transform.OriginalId]),
                    Transform.RelativeRotation,
                    Transform.RelativeX,
                    Transform.RelativeY);

                if (Contour.size() < 3 || std::abs(static_cast<double>(ClipperLib::Area(Contour))) <= 0.0) {
                    continue;
                }

                if (!ClipperLib::Orientation(Contour)) {
                    std::reverse(Contour.begin(), Contour.end());
                }

                AOutContours.push_back(std::move(Contour));
            }

            return !AOutContours.empty();
        }

        bool CetClusterBoundary::_BuildUnionBoundary(const ClipperLib::Paths& AContours, CetPath& AOutBoundary)
        {
            AOutBoundary.clear();
            if (AContours.empty()) {
                return false;
            }

            ClipperLib::Paths UnionContours;
            ClipperLib::Clipper UnionClipper;
            if (!UnionClipper.AddPaths(AContours, ClipperLib::ptSubject, true)) {
                return false;
            }

            if (!UnionClipper.Execute(ClipperLib::ctUnion, UnionContours, ClipperLib::pftNonZero, ClipperLib::pftNonZero)) {
                return false;
            }

            std::vector<CetPath> OuterContours;
            for (CetPath UnionContour : UnionContours) {
                if (UnionContour.size() < 3 || std::abs(static_cast<double>(ClipperLib::Area(UnionContour))) <= 0.0) {
                    continue;
                }

                if (!ClipperLib::Orientation(UnionContour)) {
                    continue;
                }

                OuterContours.push_back(std::move(UnionContour));
            }

            if (OuterContours.size() != 1) {
                return false;
            }

            AOutBoundary = std::move(OuterContours.front());
            return _NormalizeBoundary(AOutBoundary);
        }

        bool CetClusterBoundary::_BuildConvexHullBoundary(const ClipperLib::Paths& AContours, CetPath& AOutBoundary)
        {
            AOutBoundary.clear();

            std::vector<CetInpoint> Points = _CollectContourPoints(AContours);
            if (!_SortUniquePoints(Points)) {
                return false;
            }

            return _BuildConvexHullFromPoints(Points, AOutBoundary);
        }

        std::vector<CetInpoint> CetClusterBoundary::_CollectContourPoints(const ClipperLib::Paths& AContours)
        {
            std::vector<CetInpoint> Points;
            for (const CetPath& Contour : AContours) {
                for (const CetInpoint& Point : Contour) {
                    Points.push_back(Point);
                }
            }
            return Points;
        }

        bool CetClusterBoundary::_SortUniquePoints(std::vector<CetInpoint>& APoints)
        {
            if (APoints.size() < 3) {
                return false;
            }

            std::sort(APoints.begin(),APoints.end(),[](const CetInpoint& FirstPoint, const CetInpoint& SecondPoint){
                    if (FirstPoint.X != SecondPoint.X) {
                        return FirstPoint.X < SecondPoint.X;
                    }
                    return FirstPoint.Y < SecondPoint.Y;
                });

            APoints.erase(std::unique(APoints.begin(),APoints.end(),[](const CetInpoint& FirstPoint, const CetInpoint& SecondPoint){
                        return FirstPoint.X == SecondPoint.X && FirstPoint.Y == SecondPoint.Y;
                    }),
                APoints.end());

            return APoints.size() >= 3;
        }

        bool CetClusterBoundary::_BuildConvexHullFromPoints(const std::vector<CetInpoint>& APoints, CetPath& AOutBoundary)
        {
            if (APoints.size() < 3) {
                return false;
            }

            std::vector<CetInpoint> Hull(APoints.size() * 2);
            std::size_t HullSize = 0;

            for (const CetInpoint& Point : APoints) {
                while (HullSize >= 2 && _CrossProduct(Hull[HullSize - 2], Hull[HullSize - 1], Point) <= 0.0L) {
                    --HullSize;
                }
                Hull[HullSize] = Point;
                ++HullSize;
            }

            const std::size_t LowerHullSize = HullSize;
            for (std::size_t ReverseOffset = APoints.size(); ReverseOffset > 0; --ReverseOffset) {
                const CetInpoint& Point = APoints[ReverseOffset - 1];
                while (HullSize > LowerHullSize && _CrossProduct(Hull[HullSize - 2], Hull[HullSize - 1], Point) <= 0.0L) {
                    --HullSize;
                }
                Hull[HullSize] = Point;
                ++HullSize;
            }

            if (HullSize > 1) {
                --HullSize;
            }

            Hull.resize(HullSize);
            if (Hull.size() < 3) {
                return false;
            }

            AOutBoundary.assign(Hull.begin(), Hull.end());
            return _NormalizeBoundary(AOutBoundary);
        }

        bool CetClusterBoundary::_NormalizeBoundary(CetPath& ABoundary)
        {
            if (ABoundary.size() < 3) {
                return false;
            }

            ClipperLib::CleanPolygon(ABoundary, 1.0);
            if (ABoundary.size() < 3 || std::abs(static_cast<double>(ClipperLib::Area(ABoundary))) <= 0.0) {
                return false;
            }

            if (!ClipperLib::Orientation(ABoundary)) {
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
