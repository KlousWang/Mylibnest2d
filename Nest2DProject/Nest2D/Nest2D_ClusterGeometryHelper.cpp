#include "pch.h"
#include "Nest2D_ClusterBoundary.h"
#include "Nest2D_ClusterGeometryHelper.h"
#include "Nest2D_RotationUtils.h"
#include "NestUtils.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace ET {
    namespace NEST2DMANAGERLIB {

        namespace {
        }
        CetClusterGeometryHelper::CetClusterGeometryHelper() : CetCoreObject() {}
        CetClusterGeometryHelper::~CetClusterGeometryHelper() {}

        CetPath CetClusterGeometryHelper::GetIdentityContour(const CetNestItem& AItem) const 
        {
            CetNestItem Temp = AItem;
            Temp.translation(libnest2d::Point(0, 0));
            Temp.rotation(libnest2d::Radians(0.0));
            Temp.inflation(0);
            return Temp.transformedShape().Contour;
        }

        CetPath CetClusterGeometryHelper::TransformContour(const CetPath& AContour, double ARotation, double ATranslationX, double ATranslationY) const
        {
            CetPath Result;
            Result.reserve(AContour.size());
            const double CosVal = std::cos(ARotation);
            const double SinVal = std::sin(ARotation);
            for (const auto& Pt : AContour){
                const double PosX = static_cast<double>(Pt.X);
                const double PosY = static_cast<double>(Pt.Y);
                Result.emplace_back(static_cast<ClipperLib::cInt>(std::llround(PosX * CosVal - PosY * SinVal + ATranslationX)),static_cast<ClipperLib::cInt>(std::llround(PosX * SinVal + PosY * CosVal + ATranslationY)));
            }
            return Result;
        }

        bool CetClusterGeometryHelper::GetBounds(const CetPath& AContour, double& AOutMinX, double& AOutMinY, double& AOutMaxX, double& AOutMaxY) const
        {
            if (AContour.size() < 3) return false;
            AOutMinX = AOutMaxX = static_cast<double>(AContour.front().X);
            AOutMinY = AOutMaxY = static_cast<double>(AContour.front().Y);
            for (const auto& Pt : AContour){
                AOutMinX = std::min(AOutMinX, static_cast<double>(Pt.X));
                AOutMinY = std::min(AOutMinY, static_cast<double>(Pt.Y));
                AOutMaxX = std::max(AOutMaxX, static_cast<double>(Pt.X));
                AOutMaxY = std::max(AOutMaxY, static_cast<double>(Pt.Y));
            }
            return AOutMaxX > AOutMinX && AOutMaxY > AOutMinY;
        }

        CetPath CetClusterGeometryHelper::MakeRectangleContour(double AWidth, double AHeight) const 
        {
            const auto RectWidth = static_cast<ClipperLib::cInt>(std::ceil(std::max(0.0, AWidth)));
            const auto RectHeight = static_cast<ClipperLib::cInt>(std::ceil(std::max(0.0, AHeight)));
            CetPath Path;
            if (RectWidth <= 0 || RectHeight <= 0) return Path;
            Path.emplace_back(0, 0);
            Path.emplace_back(RectWidth, 0);
            Path.emplace_back(RectWidth, RectHeight);
            Path.emplace_back(0, RectHeight);
            if (!ClipperLib::Orientation(Path)) std::reverse(Path.begin(), Path.end());
            return Path;
        }

		CetNestItem CetClusterGeometryHelper::MakeNestItemFromProxyContour(const CetPath& AProxyContour) const
		{
			CetPath Contour = AProxyContour;

			if (Contour.size() < 3){
				Contour = MakeRectangleContour(1.0, 1.0);
			}

			if (!ClipperLib::Orientation(Contour)){
				std::reverse(Contour.begin(), Contour.end());
			}

			ClipperLib::Paths Holes;
			CetPolygonImpl Polygon(std::move(Contour), std::move(Holes));
			return CetNestItem(std::move(Polygon));
		}

        bool CetClusterGeometryHelper::IsContourFullyContained(const CetPath& AChildContour, const CetPath& AProxyContour, double AAreaTolerance) const
        {
            if (!std::isfinite(AAreaTolerance) || AAreaTolerance < 0.0){
                return false;
            }

            CetPath ChildContour;
            CetPath ProxyContour;
            if (!_NormalizeContourForClipper(AChildContour, ChildContour, 0.0) || !_NormalizeContourForClipper(AProxyContour, ProxyContour, 0.0)){
                return false;
            }

            ClipperLib::Clipper DifferenceClipper;
            if (!DifferenceClipper.AddPath(ChildContour, ClipperLib::ptSubject, true)){
                return false;
            }
            if (!DifferenceClipper.AddPath(ProxyContour, ClipperLib::ptClip, true)){
                return false;
            }

            ClipperLib::Paths DifferenceContours;
            if (!DifferenceClipper.Execute(ClipperLib::ctDifference, DifferenceContours, ClipperLib::pftNonZero, ClipperLib::pftNonZero)){
                return false;
            }

            double DifferenceArea = 0.0;
            for (const CetPath& DifferenceContour : DifferenceContours){
                const double CurrentArea = std::abs(static_cast<double>(ClipperLib::Area(DifferenceContour)));
                if (!std::isfinite(CurrentArea)){
                    return false;
                }
                DifferenceArea += CurrentArea;
            }

            return DifferenceArea <= AAreaTolerance;
        }

        bool CetClusterGeometryHelper::ExtractCandidateFreeRegions(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const TetClusterCandidate& ACandidate, std::vector<TetClusterFreeRegion>& AOutRegions) const
        {
            AOutRegions.clear();
            if (!ACandidate.Valid || ACandidate.ProxyContour.size() < 3 || ACandidate.ProxyArea <= 0.0) return false;
            CetPath ProxyContour;
            if (!_NormalizeContourForClipper(ACandidate.ProxyContour, ProxyContour, 0.0)) return false;
            ClipperLib::Paths ReservedContours;
            if (!_BuildReservedChildContours(AOriginalItems, AOptions, ACandidate, ReservedContours)) return false;
            ClipperLib::Clipper DifferenceClipper;
            if (!DifferenceClipper.AddPath(ProxyContour, ClipperLib::ptSubject, true)) return false;
            if (!ReservedContours.empty() && !DifferenceClipper.AddPaths(ReservedContours, ClipperLib::ptClip, true)) return false;
            ClipperLib::PolyTree Tree;
            if (!DifferenceClipper.Execute(ClipperLib::ctDifference, Tree, ClipperLib::pftNonZero, ClipperLib::pftNonZero)) return false;
            for (const ClipperLib::PolyNode* Node : Tree.Childs) {
                if (Node != nullptr && !_AppendFreeRegion(*Node, AOutRegions)) return false;
            }
            std::stable_sort(AOutRegions.begin(), AOutRegions.end(), [](const TetClusterFreeRegion& AFirst, const TetClusterFreeRegion& ASecond) {
                if (std::abs(AFirst.Area - ASecond.Area) > CET_CLUSTER_GEOMETRY_AREA_TOLERANCE) return AFirst.Area > ASecond.Area;
                if (AFirst.MinY != ASecond.MinY) return AFirst.MinY < ASecond.MinY;
                return AFirst.MinX < ASecond.MinX;
            });
            if (AOutRegions.size() > CET_CLUSTER_FILL_MAX_FREE_REGIONS) AOutRegions.resize(CET_CLUSTER_FILL_MAX_FREE_REGIONS);
            return true;
        }

        bool CetClusterGeometryHelper::IsContourInsideFreeRegion(const CetPath& AContour, const TetClusterFreeRegion& AFreeRegion, double AAreaTolerance) const
        {
            if (!AFreeRegion.IsClosed || !IsContourFullyContained(AContour, AFreeRegion.Contour, AAreaTolerance)) return false;
            for (const CetPath& Hole : AFreeRegion.Holes) {
                ClipperLib::Clipper IntersectionClipper;
                if (!IntersectionClipper.AddPath(AContour, ClipperLib::ptSubject, true) || !IntersectionClipper.AddPath(Hole, ClipperLib::ptClip, true)) return false;
                ClipperLib::Paths Intersections;
                if (!IntersectionClipper.Execute(ClipperLib::ctIntersection, Intersections, ClipperLib::pftNonZero, ClipperLib::pftNonZero)) return false;
                if (_CalculateUnionArea(Intersections) > AAreaTolerance) return false;
            }
            return true;
        }

        bool CetClusterGeometryHelper::_BuildReservedChildContours(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const TetClusterCandidate& ACandidate, ClipperLib::Paths& AOutContours) const
        {
            AOutContours.clear();
            if (ACandidate.Transforms.empty()) return false;
            for (const TetItemTransform& Transform : ACandidate.Transforms) {
                if (Transform.OriginalId < 0 || Transform.OriginalId >= static_cast<int>(AOriginalItems.size())) return false;
                CetNestItem Item = AOriginalItems[Transform.OriginalId];
                Item.translation(libnest2d::Point(static_cast<ClipperLib::cInt>(std::llround(Transform.RelativeX)), static_cast<ClipperLib::cInt>(std::llround(Transform.RelativeY))));
                Item.rotation(libnest2d::Radians(Transform.RelativeRotation));
                Item.inflation(0);
                const CetPolygonImpl& Shape = Item.transformedShape();
                if (Shape.Contour.size() < 3) return false;
                AOutContours.push_back(Shape.Contour);
                for (const CetPath& Hole : Shape.Holes) if (Hole.size() >= 3) AOutContours.push_back(Hole);
            }
            const double SpacingCoord = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));
            if (SpacingCoord <= 0.0) return true;
            ClipperLib::Paths OffsetContours;
            ClipperLib::ClipperOffset OffsetBuilder(2.0, std::max(1.0, SpacingCoord * 0.02));
            OffsetBuilder.AddPaths(AOutContours, ClipperLib::jtRound, ClipperLib::etClosedPolygon);
            OffsetBuilder.Execute(OffsetContours, SpacingCoord);
            if (!OffsetContours.empty()) AOutContours = std::move(OffsetContours);
            return true;
        }

        bool CetClusterGeometryHelper::_AppendFreeRegion(const ClipperLib::PolyNode& ANode, std::vector<TetClusterFreeRegion>& AOutRegions) const
        {
            if (!ANode.IsHole() && ANode.Contour.size() >= 3) {
                TetClusterFreeRegion Region;
                Region.Contour = ANode.Contour;
                Region.IsClosed = true;
                Region.Area = std::abs(static_cast<double>(ClipperLib::Area(Region.Contour)));
                if (!std::isfinite(Region.Area) || Region.Area <= 0.0 || !GetBounds(Region.Contour, Region.MinX, Region.MinY, Region.MaxX, Region.MaxY)) return false;
                Region.Width = Region.MaxX - Region.MinX;
                Region.Height = Region.MaxY - Region.MinY;
                for (const ClipperLib::PolyNode* Child : ANode.Childs) {
                    if (Child == nullptr || !Child->IsHole()) continue;
                    const double HoleArea = std::abs(static_cast<double>(ClipperLib::Area(Child->Contour)));
                    if (!std::isfinite(HoleArea) || HoleArea >= Region.Area) return false;
                    Region.Area -= HoleArea;
                    Region.Holes.push_back(Child->Contour);
                }
                if (Region.Area <= 0.0) return false;
                AOutRegions.push_back(std::move(Region));
            }
            for (const ClipperLib::PolyNode* Child : ANode.Childs) if (Child != nullptr && !_AppendFreeRegion(*Child, AOutRegions)) return false;
            return true;
        }

        bool CetClusterGeometryHelper::_NormalizeContourForClipper(const CetPath& AInputContour, CetPath& AOutContour, double AAreaTolerance) const
        {
            AOutContour = AInputContour;
            if (AOutContour.size() < 3 || !std::isfinite(AAreaTolerance) || AAreaTolerance < 0.0){
                return false;
            }

            ClipperLib::CleanPolygon(AOutContour, 1.0);
            const double ContourArea = std::abs(static_cast<double>(ClipperLib::Area(AOutContour)));
            if (AOutContour.size() < 3 || !std::isfinite(ContourArea) || ContourArea <= AAreaTolerance){
                return false;
            }

            if (!ClipperLib::Orientation(AOutContour)){
                std::reverse(AOutContour.begin(), AOutContour.end());
            }

            return true;
        }

        bool CetClusterGeometryHelper::_BuildTransformedChildContours(const CetTNestItemVector& AOriginalItems, const TetClusterCandidate& ACandidate, ClipperLib::Paths& AOutContours) const
        {
            AOutContours.clear();
            if (ACandidate.Transforms.empty()){
                return false;
            }

            AOutContours.reserve(ACandidate.Transforms.size());
            for (const TetItemTransform& Transform : ACandidate.Transforms){
                if (Transform.OriginalId < 0 || Transform.OriginalId >= static_cast<int>(AOriginalItems.size())){
                    return false;
                }
                if (!std::isfinite(Transform.RelativeX) || !std::isfinite(Transform.RelativeY) || !std::isfinite(Transform.RelativeRotation)){
                    return false;
                }

                const CetPath ChildContour = TransformContour(GetIdentityContour(AOriginalItems[Transform.OriginalId]), Transform.RelativeRotation, Transform.RelativeX, Transform.RelativeY);
                CetPath NormalizedContour;
                if (!_NormalizeContourForClipper(ChildContour, NormalizedContour, 0.0)){
                    return false;
                }
                AOutContours.push_back(std::move(NormalizedContour));
            }

            return !AOutContours.empty();
        }

        double CetClusterGeometryHelper::_CalculateUnionArea(const ClipperLib::Paths& AContours) const
        {
            if (AContours.empty()){
                return 0.0;
            }
            ClipperLib::Clipper UnionClipper;
            if (!UnionClipper.AddPaths(AContours,ClipperLib::ptSubject,true)){
                return 0.0;
            }
            ClipperLib::Paths UnionContours;
            if (!UnionClipper.Execute(ClipperLib::ctUnion,UnionContours,ClipperLib::pftNonZero,ClipperLib::pftNonZero)){
                return 0.0;
            }
            double SignedUnionArea = 0.0;
            for (const CetPath& UnionContour : UnionContours){
                const double CurrentArea =static_cast<double>(ClipperLib::Area(UnionContour));
                if (!std::isfinite(CurrentArea)){
                    return 0.0;
                }
                SignedUnionArea += CurrentArea;
            }
            const double UnionArea =std::abs(SignedUnionArea);
            return std::isfinite(UnionArea)? UnionArea: 0.0;
        }

        double CetClusterGeometryHelper::_CalculateReservedArea(const ClipperLib::Paths& AChildContours, const TetNestOptions& AOptions, double AOccupiedArea) const
        {
            if (!std::isfinite(AOccupiedArea) || AOccupiedArea <= 0.0){
                return 0.0;
            }

            const double SpacingCoord = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));
            if (SpacingCoord <= 0.0 || AChildContours.empty()){
                return AOccupiedArea;
            }

            ClipperLib::Paths OffsetContours;
            ClipperLib::ClipperOffset OffsetBuilder(2.0, std::max(1.0, SpacingCoord * 0.02));
            OffsetBuilder.AddPaths(AChildContours, ClipperLib::jtRound, ClipperLib::etClosedPolygon);
            OffsetBuilder.Execute(OffsetContours, SpacingCoord * 0.5);
            ClipperLib::CleanPolygons(OffsetContours, 1.0);
            if (OffsetContours.empty()){
                return AOccupiedArea;
            }

            const double ReservedArea = _CalculateUnionArea(OffsetContours);
            if (!std::isfinite(ReservedArea) || ReservedArea <= 0.0){
                return AOccupiedArea;
            }

            return std::max(AOccupiedArea, ReservedArea);
        }

        double CetClusterGeometryHelper::_GetAreaTolerance(double AReferenceArea) const
        {
            const double SafeReferenceArea = std::isfinite(AReferenceArea) ? std::abs(AReferenceArea) : 0.0;
            return std::max(CET_CLUSTER_GEOMETRY_AREA_TOLERANCE, SafeReferenceArea * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE);
        }
        bool CetClusterGeometryHelper::_ValidateIndexAndTransforms(const CetTNestItemVector& AOriginalItems, const TetClusterCandidate& ACandidate) const
        {
            if (ACandidate.OriginalIndices.empty()) return false;
            if (ACandidate.OriginalIndices.size() != ACandidate.Transforms.size()) return false;
            std::set<int> Indices;
            std::set<int> TransformIds;
            for (int Index : ACandidate.OriginalIndices){
                if (Index < 0 || Index >= static_cast<int>(AOriginalItems.size())) return false;
                if (!Indices.insert(Index).second) return false;
            }
            for (const auto& Transform : ACandidate.Transforms){
                if (Transform.OriginalId < 0 || Transform.OriginalId >= static_cast<int>(AOriginalItems.size())) return false;
                if (!std::isfinite(Transform.RelativeX) || !std::isfinite(Transform.RelativeY) || !std::isfinite(Transform.RelativeRotation)) return false;
                if (!TransformIds.insert(Transform.OriginalId).second) return false;
            }
            return Indices == TransformIds;
        }

        bool CetClusterGeometryHelper::_FitsBoardBounds(const TetClusterCandidate& ACandidate, const TetNestOptions& AOptions) const {
            if (ACandidate.ClusterWidth <= 0.0 || ACandidate.ClusterHeight <= 0.0) return false;
            const double BinWidth = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth));
            const double BinHeight = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinHeight));
            if (BinWidth <= 0.0 || BinHeight <= 0.0) return false;
            const bool Normal = ACandidate.ClusterWidth <= BinWidth && ACandidate.ClusterHeight <= BinHeight;
            const bool QuarterTurnAllowed = CetRotationUtils::IsAllowedRotation(CET_CLUSTER_HALF_PI, AOptions.Rotations, 1e-9);
            const bool Rotated = QuarterTurnAllowed && ACandidate.ClusterHeight <= BinWidth && ACandidate.ClusterWidth <= BinHeight;
            return Normal || Rotated;
        }

        bool CetClusterGeometryHelper::CanPlaceCandidateCopiesOnBoard(const TetClusterCandidate& ACandidate, const TetNestOptions& AOptions, std::size_t ARequiredCopies) const
        {
            if (ARequiredCopies <= 1){
                return true;
            }
            if (!ACandidate.Valid || ACandidate.ClusterWidth <= 0.0 || ACandidate.ClusterHeight <= 0.0){
                return false;
            }

            const double BinWidth = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth));
            const double BinHeight = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinHeight));
            const double Gap = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));
            if (BinWidth <= 0.0 || BinHeight <= 0.0){
                return false;
            }

            const double Width = ACandidate.ClusterWidth;
            const double Height = ACandidate.ClusterHeight;
            auto GetAxisCapacity = [Gap](double ABinSize, double AItemSize) -> std::size_t {
                if (ABinSize <= 0.0 || AItemSize <= 0.0 || AItemSize > ABinSize){
                    return 0;
                }
                return static_cast<std::size_t>(std::floor((ABinSize + Gap) / (AItemSize + Gap)));
                };
            auto GetGridCapacity = [&](double AItemWidth, double AItemHeight) -> std::size_t {
                const std::size_t Columns = GetAxisCapacity(BinWidth, AItemWidth);
                const std::size_t Rows = GetAxisCapacity(BinHeight, AItemHeight);
                if (Columns == 0 || Rows == 0 || Columns > ARequiredCopies / Rows){
                    return Columns > 0 && Rows > 0 ? ARequiredCopies : 0;
                }
                return Columns * Rows;
                };

            if (GetGridCapacity(Width, Height) >= ARequiredCopies){
                return true;
            }

            const bool QuarterTurnAllowed = CetRotationUtils::IsAllowedRotation(CET_CLUSTER_HALF_PI, AOptions.Rotations, 1e-9);
            return QuarterTurnAllowed && GetGridCapacity(Height, Width) >= ARequiredCopies;
        }

        bool CetClusterGeometryHelper::_ValidateChildContainment(const CetTNestItemVector& AOriginalItems, const TetClusterCandidate& ACandidate) const
        {
            if (ACandidate.ProxyContour.size() < 3 || ACandidate.ProxyArea <= 0.0 || !std::isfinite(ACandidate.ProxyArea)){
                return false;
            }

            const double AreaTolerance = _GetAreaTolerance(ACandidate.ProxyArea);
            for (const TetItemTransform& Transform : ACandidate.Transforms){
                if (Transform.OriginalId < 0 || Transform.OriginalId >= static_cast<int>(AOriginalItems.size())){
                    return false;
                }

                const CetPath ChildContour = TransformContour(GetIdentityContour(AOriginalItems[Transform.OriginalId]), Transform.RelativeRotation, Transform.RelativeX, Transform.RelativeY);
                if (!IsContourFullyContained(ChildContour, ACandidate.ProxyContour, AreaTolerance)){
                    std::cout << "[GEOMETRY][REJECT] Child outside proxy. OriginalId=" << Transform.OriginalId << ", AreaTolerance=" << AreaTolerance << std::endl;
                    return false;
                }
            }

            return true;
        }

        bool CetClusterGeometryHelper::_HaveRequiredSpacing(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const TetItemTransform& AFirstTransform, const TetItemTransform& ASecondTransform) const
        {
            if (AFirstTransform.OriginalId < 0 || AFirstTransform.OriginalId >= static_cast<int>(AOriginalItems.size()) ||ASecondTransform.OriginalId < 0 || ASecondTransform.OriginalId >= static_cast<int>(AOriginalItems.size())){
                return false;
            }

            const double SpacingCoord = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));
            CetNestItem FirstItem = AOriginalItems[AFirstTransform.OriginalId];
            FirstItem.translation(libnest2d::Point(static_cast<ClipperLib::cInt>(std::llround(AFirstTransform.RelativeX)),static_cast<ClipperLib::cInt>(std::llround(AFirstTransform.RelativeY))));
            FirstItem.rotation(libnest2d::Radians(AFirstTransform.RelativeRotation));
            FirstItem.inflation(0);

            CetNestItem SecondItem = AOriginalItems[ASecondTransform.OriginalId];
            SecondItem.translation(libnest2d::Point(static_cast<ClipperLib::cInt>(std::llround(ASecondTransform.RelativeX)),static_cast<ClipperLib::cInt>(std::llround(ASecondTransform.RelativeY))));
            SecondItem.rotation(libnest2d::Radians(ASecondTransform.RelativeRotation));
            SecondItem.inflation(0);

            if (SpacingCoord > 0.0){
                CetNestItem InflatedFirstItem = FirstItem;
                const auto OriginalInflation = InflatedFirstItem.inflation();
                InflatedFirstItem.inflation(static_cast<decltype(OriginalInflation)>(SpacingCoord));
                return !CetNestItem::intersects(InflatedFirstItem,SecondItem);
            }

            return !CetNestItem::intersects(FirstItem,SecondItem);
        }

        bool CetClusterGeometryHelper::_ValidateChildSpacing(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const TetClusterCandidate& ACandidate, bool ALogRejection) const
        {
            const double SpacingCoord = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));

            for (std::size_t i = 0; i < ACandidate.Transforms.size(); ++i){
                for (std::size_t j = i + 1; j < ACandidate.Transforms.size(); ++j){
                    const TetItemTransform& TransformA = ACandidate.Transforms[i];
                    const TetItemTransform& TransformB = ACandidate.Transforms[j];
                    if (!_HaveRequiredSpacing(AOriginalItems,AOptions,TransformA,TransformB)){
                        if (ALogRejection){
                            if (SpacingCoord > 0.0){
                                std::cout << "[GEOMETRY][REJECT] Child spacing violation. A=" << TransformA.OriginalId << ", B=" << TransformB.OriginalId << ", RequiredSpacing=" << AOptions.Spacing << ", SpacingCoord=" << SpacingCoord << std::endl;
                            }
                            else {
                                std::cout << "[GEOMETRY][REJECT] Child intersects. A=" << TransformA.OriginalId << ", B=" << TransformB.OriginalId << std::endl;
                            }
                        }
                        return false;
                    }
                }
            }

            return true;
        }

        bool CetClusterGeometryHelper::ValidateCandidateGeometry(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const TetClusterCandidate& ACandidate) const 
        {
            if (!_ValidateIndexAndTransforms(AOriginalItems, ACandidate)) return false;
            if (ACandidate.ProxyMode == MetClusterProxyMode::Unknown) return false;
            if (ACandidate.ProxyContour.size() < 3 || ACandidate.ProxyArea <= 0.0 || !std::isfinite(ACandidate.ProxyArea)) return false;
            if (!std::isfinite(ACandidate.ProxyWasteRatio) || ACandidate.ProxyWasteRatio < 0.0 || ACandidate.ProxyWasteRatio > 1.0) return false;
            if (!_ValidateChildContainment(AOriginalItems, ACandidate)) return false;
            if (!_ValidateChildSpacing(AOriginalItems, AOptions, ACandidate, true)) return false;
            return true;
        }

        bool CetClusterGeometryHelper::HasValidTransformSpacing(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const std::vector<TetItemTransform>& ATransforms) const
        {
            TetClusterCandidate Candidate;
            Candidate.Transforms = ATransforms;
            Candidate.OriginalIndices.reserve(ATransforms.size());
            for (const TetItemTransform& Transform : ATransforms){
                Candidate.OriginalIndices.push_back(Transform.OriginalId);
            }

            return _ValidateIndexAndTransforms(AOriginalItems, Candidate) && _ValidateChildSpacing(AOriginalItems, AOptions, Candidate, false);
        }

        bool CetClusterGeometryHelper::CanAppendTransformWithSpacing(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const std::vector<TetItemTransform>& AExistingTransforms, const TetItemTransform& ANewTransform) const
        {
            if (ANewTransform.OriginalId < 0 || ANewTransform.OriginalId >= static_cast<int>(AOriginalItems.size()) || !std::isfinite(ANewTransform.RelativeX) || !std::isfinite(ANewTransform.RelativeY) || !std::isfinite(ANewTransform.RelativeRotation)){
                return false;
            }
            for (const TetItemTransform& ExistingTransform : AExistingTransforms){
                if (ExistingTransform.OriginalId < 0 || ExistingTransform.OriginalId >= static_cast<int>(AOriginalItems.size()) || ExistingTransform.OriginalId == ANewTransform.OriginalId){
                    return false;
                }
                if (!_HaveRequiredSpacing(AOriginalItems, AOptions, ExistingTransform, ANewTransform)){
                    return false;
                }
            }
            return true;
        }

        bool CetClusterGeometryHelper::FinalizeCandidate(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, TetClusterCandidate& ACandidate) const
        {
            return FinalizeCandidate(AOriginalItems, AOptions, ACandidate, false);
        }

        void CetClusterGeometryHelper::_ResetCandidate(TetClusterCandidate& ACandidate) const
        {
            ACandidate.Valid = false;
            ACandidate.ProxyContour.clear();
            ACandidate.ProxyContourNormalized = false;
            ACandidate.ProxyMode = MetClusterProxyMode::Unknown;
            ACandidate.RealArea = 0.0;
            ACandidate.ProxyArea = 0.0;
            ACandidate.FillRatio = 0.0;
            ACandidate.OccupiedArea = 0.0;
            ACandidate.ReservedArea = 0.0;
            ACandidate.ProxyWasteArea = 0.0;
            ACandidate.ProxyWasteRatio = 1.0;
            ACandidate.BoundingBoxArea = 0.0;
            ACandidate.BoundingFillRatio = 0.0;
            ACandidate.CompactnessRatio = 0.0;
            ACandidate.BoardSpanRatio = 0.0;
            ACandidate.SheetReuseScore = 0.0;
            ACandidate.FragmentationRisk = 1.0;
            ACandidate.BaselineArea = 0.0;
            ACandidate.AreaSavingRatio = 0.0;
        }

        bool CetClusterGeometryHelper::_CalculateCandidateGeometry(const CetTNestItemVector& AOriginalItems, const TetClusterCandidate& ACandidate, TetCandidateGeometryStats& AOutStats) const
        {
            for (const TetItemTransform& Transform : ACandidate.Transforms){
                const CetNestItem& Original = AOriginalItems[Transform.OriginalId];
                const CetPath Child = TransformContour(GetIdentityContour(Original), Transform.RelativeRotation, Transform.RelativeX, Transform.RelativeY);
                double ChildMinX = 0.0;
                double ChildMinY = 0.0;
                double ChildMaxX = 0.0;
                double ChildMaxY = 0.0;
                if (!GetBounds(Child, ChildMinX, ChildMinY, ChildMaxX, ChildMaxY)) return false;

                AOutStats.MinX = std::min(AOutStats.MinX, ChildMinX);
                AOutStats.MinY = std::min(AOutStats.MinY, ChildMinY);
                AOutStats.MaxX = std::max(AOutStats.MaxX, ChildMaxX);
                AOutStats.MaxY = std::max(AOutStats.MaxY, ChildMaxY);

                const double OriginalArea = std::abs(static_cast<double>(Original.area()));
                const double ChildBoxArea = (ChildMaxX - ChildMinX) * (ChildMaxY - ChildMinY);
                if (!std::isfinite(OriginalArea) || !std::isfinite(ChildBoxArea) || OriginalArea <= 0.0 || ChildBoxArea <= 0.0) return false;
                AOutStats.RealArea += OriginalArea;
                AOutStats.BaselineArea += ChildBoxArea;
            }

            return std::isfinite(AOutStats.RealArea) && AOutStats.RealArea > 0.0 && std::isfinite(AOutStats.BaselineArea);
        }

        void CetClusterGeometryHelper::_BuildCandidateProxy(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, TetClusterCandidate& ACandidate, bool AForceRectangleProxy) const
        {
            if (!AForceRectangleProxy){
                CetClusterBoundary BoundaryBuilder;
                TetClusterBoundaryResult BoundaryResult;
                if (BoundaryBuilder.BuildBoundaryWithResult(AOriginalItems, ACandidate.Transforms, AOptions, BoundaryResult) && BoundaryResult.Mode != MetClusterProxyMode::Unknown){
                    ACandidate.ProxyContour = std::move(BoundaryResult.Boundary);
                    ACandidate.ProxyMode = BoundaryResult.Mode;
                    ACandidate.ProxyContourNormalized = false;
                }
            }
        }

        bool CetClusterGeometryHelper::_NormalizeCandidateProxy(TetClusterCandidate& ACandidate, const TetCandidateGeometryStats& AStats) const
        {
            for (TetItemTransform& Transform : ACandidate.Transforms){
                Transform.RelativeX -= AStats.MinX;
                Transform.RelativeY -= AStats.MinY;
            }

            const double Width = AStats.MaxX - AStats.MinX;
            const double Height = AStats.MaxY - AStats.MinY;
            if (!std::isfinite(Width) || !std::isfinite(Height) || Width <= 0.0 || Height <= 0.0) return false;

            if (ACandidate.ProxyContour.size() < 3){
                ACandidate.ProxyContour = MakeRectangleContour(Width, Height);
                ACandidate.ProxyMode = MetClusterProxyMode::RectangleFallback;
            }
            else {
                for (CetInpoint& Point : ACandidate.ProxyContour){
                    Point.X -= static_cast<ClipperLib::cInt>(std::llround(AStats.MinX));
                    Point.Y -= static_cast<ClipperLib::cInt>(std::llround(AStats.MinY));
                }
            }

            double ProxyMinX = 0.0;
            double ProxyMinY = 0.0;
            double ProxyMaxX = 0.0;
            double ProxyMaxY = 0.0;
            if (!GetBounds(ACandidate.ProxyContour, ProxyMinX, ProxyMinY, ProxyMaxX, ProxyMaxY)) return false;

            ACandidate.ClusterWidth = ProxyMaxX - ProxyMinX;
            ACandidate.ClusterHeight = ProxyMaxY - ProxyMinY;
            ACandidate.ProxyArea = std::abs(static_cast<double>(ClipperLib::Area(ACandidate.ProxyContour)));
            if (!std::isfinite(ACandidate.ProxyArea) || ACandidate.ProxyArea <= 0.0) return false;
            return true;
        }

        bool CetClusterGeometryHelper::_CalculateCandidateMetrics(TetClusterCandidate& ACandidate, const TetNestOptions& AOptions) const
        {
            const double AreaTolerance = _GetAreaTolerance(ACandidate.ProxyArea);
            ACandidate.FillRatio = ACandidate.ProxyArea > AreaTolerance ? std::clamp(ACandidate.RealArea / ACandidate.ProxyArea, 0.0, 1.0) : 0.0;
            if (!std::isfinite(ACandidate.FillRatio)) ACandidate.FillRatio = 0.0;

            ACandidate.AreaSavingRatio = ACandidate.BaselineArea > AreaTolerance ? 1.0 - ACandidate.ProxyArea / ACandidate.BaselineArea : 0.0;
            if (!std::isfinite(ACandidate.AreaSavingRatio)) ACandidate.AreaSavingRatio = 0.0;

            ACandidate.ProxyWasteArea = std::max(0.0, ACandidate.ProxyArea - ACandidate.ReservedArea);
            if (!std::isfinite(ACandidate.ProxyWasteArea) || ACandidate.ProxyWasteArea < 0.0) ACandidate.ProxyWasteArea = 0.0;
            ACandidate.ProxyWasteRatio = ACandidate.ProxyArea > AreaTolerance ? ACandidate.ProxyWasteArea / ACandidate.ProxyArea : 1.0;
            if (!std::isfinite(ACandidate.ProxyWasteRatio)) ACandidate.ProxyWasteRatio = 1.0;
            ACandidate.ProxyWasteRatio = std::clamp(ACandidate.ProxyWasteRatio, 0.0, 1.0);

            ACandidate.BoundingBoxArea = ACandidate.ClusterWidth * ACandidate.ClusterHeight;
            if (!std::isfinite(ACandidate.BoundingBoxArea) || ACandidate.BoundingBoxArea <= 0.0){
                return false;
            }

            ACandidate.BoundingFillRatio = std::clamp(ACandidate.RealArea / ACandidate.BoundingBoxArea, 0.0, 1.0);
            const double LongSide = std::max(ACandidate.ClusterWidth, ACandidate.ClusterHeight);
            const double ShortSide = std::min(ACandidate.ClusterWidth, ACandidate.ClusterHeight);
            ACandidate.CompactnessRatio = LongSide > 0.0 ? std::clamp(ShortSide / LongSide, 0.0, 1.0) : 0.0;

            const double BinWidth = std::max(1.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth)));
            const double BinHeight = std::max(1.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.BinHeight)));
            ACandidate.BoardSpanRatio = std::clamp(
                std::max(ACandidate.ClusterWidth / BinWidth, ACandidate.ClusterHeight / BinHeight),
                0.0,
                1.0);

            // A sparse or slender cluster is more likely to leave disconnected scraps
            // even when its exact proxy area looks efficient.
            const double HollowRisk = 1.0 - ACandidate.BoundingFillRatio;
            const double SlenderRisk = 1.0 - ACandidate.CompactnessRatio;
            const double ExcessiveSpanRisk = std::clamp((ACandidate.BoardSpanRatio - 0.55) / 0.45, 0.0, 1.0);
            ACandidate.FragmentationRisk = std::clamp(
                HollowRisk * 0.55 + SlenderRisk * 0.30 + ExcessiveSpanRisk * 0.15,
                0.0,
                1.0);
            ACandidate.SheetReuseScore = 1.0 - ACandidate.FragmentationRisk;
            return true;
        }

        bool CetClusterGeometryHelper::FinalizeCandidate(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, TetClusterCandidate& ACandidate, bool AForceRectangleProxy) const
        {
            _ResetCandidate(ACandidate);
            if (!_ValidateIndexAndTransforms(AOriginalItems, ACandidate)) return false;

            TetCandidateGeometryStats GeometryStats;
            if (!_CalculateCandidateGeometry(AOriginalItems, ACandidate, GeometryStats)) return false;
            ACandidate.RealArea = GeometryStats.RealArea;
            ACandidate.BaselineArea = GeometryStats.BaselineArea;

            ClipperLib::Paths TransformedChildContours;
            if (!_BuildTransformedChildContours(AOriginalItems, ACandidate, TransformedChildContours)) return false;
            ACandidate.OccupiedArea = _CalculateUnionArea(TransformedChildContours);
            if (!std::isfinite(ACandidate.OccupiedArea) || ACandidate.OccupiedArea <= 0.0) return false;
            ACandidate.ReservedArea = _CalculateReservedArea(TransformedChildContours, AOptions, ACandidate.OccupiedArea);
            if (!std::isfinite(ACandidate.ReservedArea) || ACandidate.ReservedArea <= 0.0) ACandidate.ReservedArea = ACandidate.OccupiedArea;

            _BuildCandidateProxy(AOriginalItems, AOptions, ACandidate, AForceRectangleProxy);
            if (!_NormalizeCandidateProxy(ACandidate, GeometryStats)) return false;
            if (!_CalculateCandidateMetrics(ACandidate, AOptions)) return false;
            ACandidate.ProxyContourNormalized = true;
            if (!_FitsBoardBounds(ACandidate, AOptions)) return false;
            if (!ValidateCandidateGeometry(AOriginalItems, AOptions, ACandidate)) return false;

            const double WasteSafetyPenalty = ACandidate.ProxyWasteRatio > 0.60 ? (ACandidate.ProxyWasteRatio - 0.60) * 50.0 : 0.0;
            ACandidate.Score = ACandidate.AreaSavingRatio * 1000.0 + ACandidate.FillRatio * 100.0 + static_cast<double>(ACandidate.OriginalIndices.size()) * 10.0 + ACandidate.Confidence - WasteSafetyPenalty;
            ACandidate.Valid = true;
            return true;
        }

        bool CetClusterGeometryHelper::FinalizeCandidateInRectangle(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, TetClusterCandidate& ACandidate, double AEnvelopeWidth, double AEnvelopeHeight) const
        {
            if (!std::isfinite(AEnvelopeWidth) || !std::isfinite(AEnvelopeHeight) || AEnvelopeWidth <= 0.0 || AEnvelopeHeight <= 0.0){
                return false;
            }

            if (!FinalizeCandidate(AOriginalItems, AOptions, ACandidate, true)){
                return false;
            }

            const double DimensionTolerance = std::max(1.0, std::max(AEnvelopeWidth, AEnvelopeHeight) * 1e-9);
            if (ACandidate.ClusterWidth > AEnvelopeWidth + DimensionTolerance || ACandidate.ClusterHeight > AEnvelopeHeight + DimensionTolerance){
                return false;
            }

            ACandidate.ProxyContour = MakeRectangleContour(AEnvelopeWidth, AEnvelopeHeight);
            if (ACandidate.ProxyContour.size() < 4){
                return false;
            }
            ACandidate.ProxyMode = MetClusterProxyMode::RectangleFallback;
            ACandidate.ProxyContourNormalized = true;

            double ProxyMinX = 0.0;
            double ProxyMinY = 0.0;
            double ProxyMaxX = 0.0;
            double ProxyMaxY = 0.0;
            if (!GetBounds(ACandidate.ProxyContour, ProxyMinX, ProxyMinY, ProxyMaxX, ProxyMaxY)){
                return false;
            }

            ACandidate.ClusterWidth = ProxyMaxX - ProxyMinX;
            ACandidate.ClusterHeight = ProxyMaxY - ProxyMinY;
            ACandidate.ProxyArea = std::abs(static_cast<double>(ClipperLib::Area(ACandidate.ProxyContour)));
            if (ACandidate.ClusterWidth <= 0.0 || ACandidate.ClusterHeight <= 0.0 || ACandidate.ProxyArea <= 0.0 || !std::isfinite(ACandidate.ProxyArea)){
                return false;
            }

            const double AreaTolerance = _GetAreaTolerance(ACandidate.ProxyArea);
            ACandidate.FillRatio = ACandidate.ProxyArea > AreaTolerance ? std::clamp(ACandidate.RealArea / ACandidate.ProxyArea, 0.0, 1.0) : 0.0;
            ACandidate.AreaSavingRatio = ACandidate.BaselineArea > AreaTolerance ? 1.0 - ACandidate.ProxyArea / ACandidate.BaselineArea : 0.0;
            ACandidate.ProxyWasteArea = std::max(0.0, ACandidate.ProxyArea - ACandidate.ReservedArea);
            ACandidate.ProxyWasteRatio = ACandidate.ProxyArea > AreaTolerance ? ACandidate.ProxyWasteArea / ACandidate.ProxyArea : 1.0;
            ACandidate.ProxyWasteRatio = std::clamp(ACandidate.ProxyWasteRatio, 0.0, 1.0);

            ACandidate.BoundingBoxArea = ACandidate.ClusterWidth * ACandidate.ClusterHeight;
            ACandidate.BoundingFillRatio = ACandidate.BoundingBoxArea > AreaTolerance ? std::clamp(ACandidate.RealArea / ACandidate.BoundingBoxArea, 0.0, 1.0) : 0.0;
            const double LongSide = std::max(ACandidate.ClusterWidth, ACandidate.ClusterHeight);
            const double ShortSide = std::min(ACandidate.ClusterWidth, ACandidate.ClusterHeight);
            ACandidate.CompactnessRatio = LongSide > 0.0 ? std::clamp(ShortSide / LongSide, 0.0, 1.0) : 0.0;

            const double BinWidth = std::max(1.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth)));
            const double BinHeight = std::max(1.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.BinHeight)));
            ACandidate.BoardSpanRatio = std::clamp(std::max(ACandidate.ClusterWidth / BinWidth, ACandidate.ClusterHeight / BinHeight), 0.0, 1.0);
            const double HollowRisk = 1.0 - ACandidate.BoundingFillRatio;
            const double SlenderRisk = 1.0 - ACandidate.CompactnessRatio;
            const double ExcessiveSpanRisk = std::clamp((ACandidate.BoardSpanRatio - 0.55) / 0.45, 0.0, 1.0);
            ACandidate.FragmentationRisk = std::clamp(HollowRisk * 0.55 + SlenderRisk * 0.30 + ExcessiveSpanRisk * 0.15, 0.0, 1.0);
            ACandidate.SheetReuseScore = 1.0 - ACandidate.FragmentationRisk;

            if (!_FitsBoardBounds(ACandidate, AOptions) || !ValidateCandidateGeometry(AOriginalItems, AOptions, ACandidate)){
                return false;
            }

            const double WasteSafetyPenalty = ACandidate.ProxyWasteRatio > 0.60 ? (ACandidate.ProxyWasteRatio - 0.60) * 50.0 : 0.0;
            ACandidate.Score = ACandidate.AreaSavingRatio * 1000.0 + ACandidate.FillRatio * 100.0 + static_cast<double>(ACandidate.OriginalIndices.size()) * 10.0 + ACandidate.Confidence - WasteSafetyPenalty;
            ACandidate.Valid = true;
            return true;
        }

    }
}
