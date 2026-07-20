#include "pch.h"
#include "Nest2D_ClusterGeometryHelper.h"
#include "NestUtils.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace ET {
    namespace NEST2DMANAGERLIB {

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
            const bool Rotated = AOptions.Rotations > 1 && ACandidate.ClusterHeight <= BinWidth && ACandidate.ClusterWidth <= BinHeight;
            return Normal || Rotated;
        }

        bool CetClusterGeometryHelper::_ValidateChildContainment(const CetTNestItemVector& AOriginalItems, const TetClusterCandidate& ACandidate) const {
            if (ACandidate.ProxyContour.size() < 3) return false;
            for (const auto& Transform : ACandidate.Transforms) {
                CetPath Child = TransformContour(GetIdentityContour(AOriginalItems[Transform.OriginalId]), Transform.RelativeRotation, Transform.RelativeX, Transform.RelativeY);
                if (Child.size() < 3) return false;
                for (const auto& Pt : Child) {
                    const int Position = ClipperLib::PointInPolygon(Pt, ACandidate.ProxyContour);
                    if (Position == 0) return false;
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
            if (ACandidate.ProxyContour.size() < 3 || ACandidate.ProxyArea <= 0.0) return false;
            if (!_ValidateChildContainment(AOriginalItems, ACandidate)) return false;
            if (!_ValidateChildSpacing(AOriginalItems, AOptions, ACandidate)) return false;
            return true;
        }

        bool CetClusterGeometryHelper::FinalizeCandidate(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, TetClusterCandidate& ACandidate) const
        {
            ACandidate.Valid = false;
            if (!_ValidateIndexAndTransforms(AOriginalItems, ACandidate)) return false;

            double MinX = std::numeric_limits<double>::max();
            double MinY = std::numeric_limits<double>::max();
            double MaxX = std::numeric_limits<double>::lowest();
            double MaxY = std::numeric_limits<double>::lowest();
            ACandidate.RealArea = 0.0;
            ACandidate.BaselineArea = 0.0;

            for (const auto& Transform : ACandidate.Transforms) {
                const CetNestItem& Original = AOriginalItems[Transform.OriginalId];
                const CetPath Child = TransformContour(GetIdentityContour(Original), Transform.RelativeRotation, Transform.RelativeX, Transform.RelativeY);
                double ChildMinX, ChildMinY, ChildMaxX, ChildMaxY;
                if (!GetBounds(Child, ChildMinX, ChildMinY, ChildMaxX, ChildMaxY)) return false;
                MinX = std::min(MinX, ChildMinX); MinY = std::min(MinY, ChildMinY);
                MaxX = std::max(MaxX, ChildMaxX); MaxY = std::max(MaxY, ChildMaxY);
                ACandidate.RealArea += std::abs(static_cast<double>(Original.area()));
                ACandidate.BaselineArea += (ChildMaxX - ChildMinX) * (ChildMaxY - ChildMinY);
            }

            for (auto& Transform : ACandidate.Transforms) {
                Transform.RelativeX -= MinX;
                Transform.RelativeY -= MinY;
            }

            const double Width = MaxX - MinX;
            const double Height = MaxY - MinY;
            if (Width <= 0.0 || Height <= 0.0) return false;

            if (ACandidate.ProxyContour.size() < 3) {
                ACandidate.ProxyContour = MakeRectangleContour(Width, Height);
            }
            else {
                for (auto& Pt : ACandidate.ProxyContour) {
                    Pt.X -= static_cast<ClipperLib::cInt>(std::llround(MinX));
                    Pt.Y -= static_cast<ClipperLib::cInt>(std::llround(MinY));
                }
            }

            double ProxyMinX, ProxyMinY, ProxyMaxX, ProxyMaxY;
            if (!GetBounds(ACandidate.ProxyContour, ProxyMinX, ProxyMinY, ProxyMaxX, ProxyMaxY)) return false;
            ACandidate.ClusterWidth = ProxyMaxX - ProxyMinX;
            ACandidate.ClusterHeight = ProxyMaxY - ProxyMinY;
            ACandidate.ProxyArea = std::abs(static_cast<double>(ClipperLib::Area(ACandidate.ProxyContour)));
            if (ACandidate.ProxyArea <= 0.0) return false;
            ACandidate.FillRatio = std::clamp(ACandidate.RealArea / ACandidate.ProxyArea, 0.0, 1.0);
            ACandidate.AreaSavingRatio = ACandidate.BaselineArea > 0.0 ? 1.0 - ACandidate.ProxyArea / ACandidate.BaselineArea : 0.0;
            ACandidate.ProxyContourNormalized = true;
            if (!_FitsBoardBounds(ACandidate, AOptions)) return false;
            if (!ValidateCandidateGeometry(AOriginalItems, AOptions, ACandidate)) return false;

            ACandidate.Score = ACandidate.AreaSavingRatio * 1000.0 + ACandidate.FillRatio * 100.0 + static_cast<double>(ACandidate.OriginalIndices.size()) * 10.0 + ACandidate.Confidence;
            ACandidate.Valid = true;
            return true;
        }

    }
}
