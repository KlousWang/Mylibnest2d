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
            constexpr double CET_CLUSTER_GEOMETRY_AREA_TOLERANCE = 16.0;
            constexpr double CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE = 1e-10;
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
            for (const auto& Pt : AContour) {
                const double PosX = static_cast<double>(Pt.X);
                const double PosY = static_cast<double>(Pt.Y);
                Result.emplace_back(
                    static_cast<ClipperLib::cInt>(std::llround(PosX * CosVal - PosY * SinVal + ATranslationX)),
                    static_cast<ClipperLib::cInt>(std::llround(PosX * SinVal + PosY * CosVal + ATranslationY)));
            }
            return Result;
        }

        bool CetClusterGeometryHelper::GetBounds(const CetPath& AContour, double& AOutMinX, double& AOutMinY, double& AOutMaxX, double& AOutMaxY) const
        {
            if (AContour.size() < 3) return false;
            AOutMinX = AOutMaxX = static_cast<double>(AContour.front().X);
            AOutMinY = AOutMaxY = static_cast<double>(AContour.front().Y);
            for (const auto& Pt : AContour) {
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

			if (Contour.size() < 3) {
				Contour = MakeRectangleContour(1.0, 1.0);
			}

			if (!ClipperLib::Orientation(Contour)) {
				std::reverse(Contour.begin(), Contour.end());
			}

			ClipperLib::Paths Holes;
			CetPolygonImpl Polygon(std::move(Contour), std::move(Holes));
			return CetNestItem(std::move(Polygon));
		}

        bool CetClusterGeometryHelper::IsContourFullyContained(const CetPath& AChildContour, const CetPath& AProxyContour, double AAreaTolerance) const
        {
            if (!std::isfinite(AAreaTolerance) || AAreaTolerance < 0.0) {
                return false;
            }

            CetPath ChildContour;
            CetPath ProxyContour;
            if (!_NormalizeContourForClipper(AChildContour, ChildContour, 0.0) || !_NormalizeContourForClipper(AProxyContour, ProxyContour, 0.0)) {
                return false;
            }

            ClipperLib::Clipper DifferenceClipper;
            if (!DifferenceClipper.AddPath(ChildContour, ClipperLib::ptSubject, true)) {
                return false;
            }
            if (!DifferenceClipper.AddPath(ProxyContour, ClipperLib::ptClip, true)) {
                return false;
            }

            ClipperLib::Paths DifferenceContours;
            if (!DifferenceClipper.Execute(ClipperLib::ctDifference, DifferenceContours, ClipperLib::pftNonZero, ClipperLib::pftNonZero)) {
                return false;
            }

            double DifferenceArea = 0.0;
            for (const CetPath& DifferenceContour : DifferenceContours) {
                const double CurrentArea = std::abs(static_cast<double>(ClipperLib::Area(DifferenceContour)));
                if (!std::isfinite(CurrentArea)) {
                    return false;
                }
                DifferenceArea += CurrentArea;
            }

            return DifferenceArea <= AAreaTolerance;
        }

        bool CetClusterGeometryHelper::_NormalizeContourForClipper(const CetPath& AInputContour, CetPath& AOutContour, double AAreaTolerance) const
        {
            AOutContour = AInputContour;
            if (AOutContour.size() < 3 || !std::isfinite(AAreaTolerance) || AAreaTolerance < 0.0) {
                return false;
            }

            ClipperLib::CleanPolygon(AOutContour, 1.0);
            const double ContourArea = std::abs(static_cast<double>(ClipperLib::Area(AOutContour)));
            if (AOutContour.size() < 3 || !std::isfinite(ContourArea) || ContourArea <= AAreaTolerance) {
                return false;
            }

            if (!ClipperLib::Orientation(AOutContour)) {
                std::reverse(AOutContour.begin(), AOutContour.end());
            }

            return true;
        }

        bool CetClusterGeometryHelper::_BuildTransformedChildContours(const CetTNestItemVector& AOriginalItems, const TetClusterCandidate& ACandidate, ClipperLib::Paths& AOutContours) const
        {
            AOutContours.clear();
            if (ACandidate.Transforms.empty()) {
                return false;
            }

            AOutContours.reserve(ACandidate.Transforms.size());
            for (const TetItemTransform& Transform : ACandidate.Transforms) {
                if (Transform.OriginalId < 0 || Transform.OriginalId >= static_cast<int>(AOriginalItems.size())) {
                    return false;
                }
                if (!std::isfinite(Transform.RelativeX) || !std::isfinite(Transform.RelativeY) || !std::isfinite(Transform.RelativeRotation)) {
                    return false;
                }

                const CetPath ChildContour = TransformContour(GetIdentityContour(AOriginalItems[Transform.OriginalId]), Transform.RelativeRotation, Transform.RelativeX, Transform.RelativeY);
                CetPath NormalizedContour;
                if (!_NormalizeContourForClipper(ChildContour, NormalizedContour, 0.0)) {
                    return false;
                }
                AOutContours.push_back(std::move(NormalizedContour));
            }

            return !AOutContours.empty();
        }

        double CetClusterGeometryHelper::_CalculateUnionArea(const ClipperLib::Paths& AContours) const
        {
            if (AContours.empty()) {
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
            for (const CetPath& UnionContour : UnionContours)
            {
                const double CurrentArea =static_cast<double>(ClipperLib::Area(UnionContour));
                if (!std::isfinite(CurrentArea)) {
                    return 0.0;
                }
                SignedUnionArea += CurrentArea;
            }
            const double UnionArea =std::abs(SignedUnionArea);
            return std::isfinite(UnionArea)? UnionArea: 0.0;
        }

        double CetClusterGeometryHelper::_CalculateReservedArea(const ClipperLib::Paths& AChildContours, const TetNestOptions& AOptions, double AOccupiedArea) const
        {
            if (!std::isfinite(AOccupiedArea) || AOccupiedArea <= 0.0) {
                return 0.0;
            }

            const double SpacingCoord = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));
            if (SpacingCoord <= 0.0 || AChildContours.empty()) {
                return AOccupiedArea;
            }

            ClipperLib::Paths OffsetContours;
            ClipperLib::ClipperOffset OffsetBuilder(2.0, std::max(1.0, SpacingCoord * 0.02));
            OffsetBuilder.AddPaths(AChildContours, ClipperLib::jtRound, ClipperLib::etClosedPolygon);
            OffsetBuilder.Execute(OffsetContours, SpacingCoord * 0.5);
            ClipperLib::CleanPolygons(OffsetContours, 1.0);
            if (OffsetContours.empty()) {
                return AOccupiedArea;
            }

            const double ReservedArea = _CalculateUnionArea(OffsetContours);
            if (!std::isfinite(ReservedArea) || ReservedArea <= 0.0) {
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
            for (int Index : ACandidate.OriginalIndices) {
                if (Index < 0 || Index >= static_cast<int>(AOriginalItems.size())) return false;
                if (!Indices.insert(Index).second) return false;
            }
            for (const auto& Transform : ACandidate.Transforms) {
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
            if (ARequiredCopies <= 1) {
                return true;
            }
            if (!ACandidate.Valid || ACandidate.ClusterWidth <= 0.0 || ACandidate.ClusterHeight <= 0.0) {
                return false;
            }

            const double BinWidth = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth));
            const double BinHeight = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinHeight));
            const double Gap = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));
            if (BinWidth <= 0.0 || BinHeight <= 0.0) {
                return false;
            }

            const double Width = ACandidate.ClusterWidth;
            const double Height = ACandidate.ClusterHeight;
            auto GetAxisCapacity = [Gap](double BinSize, double ItemSize) -> std::size_t {
                if (BinSize <= 0.0 || ItemSize <= 0.0 || ItemSize > BinSize) {
                    return 0;
                }
                return static_cast<std::size_t>(std::floor((BinSize + Gap) / (ItemSize + Gap)));
                };
            auto GetGridCapacity = [&](double ItemWidth, double ItemHeight) -> std::size_t {
                const std::size_t Columns = GetAxisCapacity(BinWidth, ItemWidth);
                const std::size_t Rows = GetAxisCapacity(BinHeight, ItemHeight);
                if (Columns == 0 || Rows == 0 || Columns > ARequiredCopies / Rows) {
                    return Columns > 0 && Rows > 0 ? ARequiredCopies : 0;
                }
                return Columns * Rows;
                };

            if (GetGridCapacity(Width, Height) >= ARequiredCopies) {
                return true;
            }

            const bool QuarterTurnAllowed = CetRotationUtils::IsAllowedRotation(CET_CLUSTER_HALF_PI, AOptions.Rotations, 1e-9);
            return QuarterTurnAllowed && GetGridCapacity(Height, Width) >= ARequiredCopies;
        }

        bool CetClusterGeometryHelper::_ValidateChildContainment(const CetTNestItemVector& AOriginalItems, const TetClusterCandidate& ACandidate) const
        {
            if (ACandidate.ProxyContour.size() < 3 || ACandidate.ProxyArea <= 0.0 || !std::isfinite(ACandidate.ProxyArea)) {
                return false;
            }

            const double AreaTolerance = _GetAreaTolerance(ACandidate.ProxyArea);
            for (const TetItemTransform& Transform : ACandidate.Transforms) {
                if (Transform.OriginalId < 0 || Transform.OriginalId >= static_cast<int>(AOriginalItems.size())) {
                    return false;
                }

                const CetPath ChildContour = TransformContour(GetIdentityContour(AOriginalItems[Transform.OriginalId]), Transform.RelativeRotation, Transform.RelativeX, Transform.RelativeY);
                if (!IsContourFullyContained(ChildContour, ACandidate.ProxyContour, AreaTolerance)) {
                    std::cout << "[GEOMETRY][REJECT] Child outside proxy. OriginalId=" << Transform.OriginalId << ", AreaTolerance=" << AreaTolerance << std::endl;
                    return false;
                }
            }

            return true;
        }

        bool CetClusterGeometryHelper::_ValidateChildSpacing(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const TetClusterCandidate& ACandidate) const 
        {
            /*
               * 将用户输入的实际单位转换为 libnest2d 内部坐标。
               *
               * 例如：
               * AOptions.Spacing = 1 mm
               * 转换后可能为 1000000 个内部坐标单位。
               */
            const double SpacingCoord = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));

            for (std::size_t i = 0; i < ACandidate.Transforms.size(); ++i) {
                const auto& TransformA = ACandidate.Transforms[i];
                CetNestItem ItemA = AOriginalItems[TransformA.OriginalId];

                /*
                 * 恢复子件 A 在组合件局部坐标系中的位置和旋转。
                 */
                ItemA.translation(libnest2d::Point(static_cast<ClipperLib::cInt>(std::llround(TransformA.RelativeX)), static_cast<ClipperLib::cInt>(std::llround(TransformA.RelativeY))));
                ItemA.rotation(libnest2d::Radians(TransformA.RelativeRotation));

                /*
                 * 原始碰撞检查必须先清除零件原有 inflation，
                 * 防止外部排样配置影响组合件内部校验。
                 */
                ItemA.inflation(0);

                for (std::size_t j = i + 1; j < ACandidate.Transforms.size(); ++j) {
                    const auto& TransformB = ACandidate.Transforms[j];
                    CetNestItem ItemB = AOriginalItems[TransformB.OriginalId];

                    /*
                     * 恢复子件 B 在组合件局部坐标系中的位置和旋转。
                     */
                    ItemB.translation(libnest2d::Point(static_cast<ClipperLib::cInt>(std::llround(TransformB.RelativeX)), static_cast<ClipperLib::cInt>(std::llround(TransformB.RelativeY))));
                    ItemB.rotation(libnest2d::Radians(TransformB.RelativeRotation));
                    ItemB.inflation(0);

                    if (SpacingCoord > 0.0) {
                        /*
                         * 将 A 向外膨胀完整的 Spacing。
                         *
                         * 如果膨胀后的 A 与 B 相交，
                         * 说明原始 A、B 之间的距离小于 Spacing。
                         *
                         * 只膨胀 A 即可，不需要 A、B 各膨胀一半。
                         */
                        CetNestItem InflatedItemA = ItemA;
                        const auto OriginalInflation = InflatedItemA.inflation();
                        InflatedItemA.inflation(static_cast<decltype(OriginalInflation)>(SpacingCoord));

                        if (CetNestItem::intersects(InflatedItemA, ItemB)) {
                            std::cout << "[GEOMETRY][REJECT] Child spacing violation. A=" << TransformA.OriginalId << ", B=" << TransformB.OriginalId << ", RequiredSpacing=" << AOptions.Spacing << ", SpacingCoord=" << SpacingCoord << std::endl;
                            return false;
                        }
                    }
                    else {
                        /*
                         * Spacing == 0 时不进行轮廓膨胀，
                         * 只禁止两个零件发生实体重叠。
                         */
                        if (CetNestItem::intersects(ItemA, ItemB)) {
                            std::cout << "[GEOMETRY][REJECT] Child intersects. A=" << TransformA.OriginalId << ", B=" << TransformB.OriginalId << std::endl;
                            return false;
                        }
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
            if (!_ValidateChildSpacing(AOriginalItems, AOptions, ACandidate)) return false;
            return true;
        }

        bool CetClusterGeometryHelper::FinalizeCandidate(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, TetClusterCandidate& ACandidate) const
        {
            return FinalizeCandidate(AOriginalItems, AOptions, ACandidate, false);
        }

        bool CetClusterGeometryHelper::FinalizeCandidate(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, TetClusterCandidate& ACandidate, bool AForceRectangleProxy) const
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
            ACandidate.BaselineArea = 0.0;
            ACandidate.AreaSavingRatio = 0.0;

            if (!_ValidateIndexAndTransforms(AOriginalItems, ACandidate)) return false;

            double MinX = std::numeric_limits<double>::max();
            double MinY = std::numeric_limits<double>::max();
            double MaxX = std::numeric_limits<double>::lowest();
            double MaxY = std::numeric_limits<double>::lowest();

            for (const TetItemTransform& Transform : ACandidate.Transforms) {
                const CetNestItem& Original = AOriginalItems[Transform.OriginalId];
                const CetPath Child = TransformContour(GetIdentityContour(Original), Transform.RelativeRotation, Transform.RelativeX, Transform.RelativeY);
                double ChildMinX = 0.0;
                double ChildMinY = 0.0;
                double ChildMaxX = 0.0;
                double ChildMaxY = 0.0;
                if (!GetBounds(Child, ChildMinX, ChildMinY, ChildMaxX, ChildMaxY)) return false;

                MinX = std::min(MinX, ChildMinX);
                MinY = std::min(MinY, ChildMinY);
                MaxX = std::max(MaxX, ChildMaxX);
                MaxY = std::max(MaxY, ChildMaxY);

                const double OriginalArea = std::abs(static_cast<double>(Original.area()));
                const double ChildBoxArea = (ChildMaxX - ChildMinX) * (ChildMaxY - ChildMinY);
                if (!std::isfinite(OriginalArea) || !std::isfinite(ChildBoxArea) || OriginalArea <= 0.0 || ChildBoxArea <= 0.0) return false;
                ACandidate.RealArea += OriginalArea;
                ACandidate.BaselineArea += ChildBoxArea;
            }

            if (!std::isfinite(ACandidate.RealArea) || ACandidate.RealArea <= 0.0 || !std::isfinite(ACandidate.BaselineArea)) return false;

            ClipperLib::Paths TransformedChildContours;
            if (!_BuildTransformedChildContours(AOriginalItems, ACandidate, TransformedChildContours)) return false;

            ACandidate.OccupiedArea = _CalculateUnionArea(TransformedChildContours);
            if (!std::isfinite(ACandidate.OccupiedArea) || ACandidate.OccupiedArea <= 0.0) return false;

            ACandidate.ReservedArea = _CalculateReservedArea(TransformedChildContours, AOptions, ACandidate.OccupiedArea);
            if (!std::isfinite(ACandidate.ReservedArea) || ACandidate.ReservedArea <= 0.0) {
                ACandidate.ReservedArea = ACandidate.OccupiedArea;
            }

            if (!AForceRectangleProxy) {
                CetClusterBoundary BoundaryBuilder;
                TetClusterBoundaryResult BoundaryResult;
                if (BoundaryBuilder.BuildBoundaryWithResult(AOriginalItems, ACandidate.Transforms, AOptions, BoundaryResult) && BoundaryResult.Mode != MetClusterProxyMode::Unknown) {
                    ACandidate.ProxyContour = std::move(BoundaryResult.Boundary);
                    ACandidate.ProxyMode = BoundaryResult.Mode;
                    ACandidate.ProxyContourNormalized = false;
                }
            }

            for (TetItemTransform& Transform : ACandidate.Transforms) {
                Transform.RelativeX -= MinX;
                Transform.RelativeY -= MinY;
            }

            const double Width = MaxX - MinX;
            const double Height = MaxY - MinY;
            if (!std::isfinite(Width) || !std::isfinite(Height) || Width <= 0.0 || Height <= 0.0) return false;

            if (ACandidate.ProxyContour.size() < 3) {
                ACandidate.ProxyContour = MakeRectangleContour(Width, Height);
                ACandidate.ProxyMode = MetClusterProxyMode::RectangleFallback;
            }
            else {
                for (CetInpoint& Point : ACandidate.ProxyContour) {
                    Point.X -= static_cast<ClipperLib::cInt>(std::llround(MinX));
                    Point.Y -= static_cast<ClipperLib::cInt>(std::llround(MinY));
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

            ACandidate.ProxyContourNormalized = true;
            if (!_FitsBoardBounds(ACandidate, AOptions)) return false;
            if (!ValidateCandidateGeometry(AOriginalItems, AOptions, ACandidate)) return false;

            const double WasteSafetyPenalty = ACandidate.ProxyWasteRatio > 0.60 ? (ACandidate.ProxyWasteRatio - 0.60) * 50.0 : 0.0;
            ACandidate.Score = ACandidate.AreaSavingRatio * 1000.0 + ACandidate.FillRatio * 100.0 + static_cast<double>(ACandidate.OriginalIndices.size()) * 10.0 + ACandidate.Confidence - WasteSafetyPenalty;
            ACandidate.Valid = true;
            return true;
        }

    }
}
