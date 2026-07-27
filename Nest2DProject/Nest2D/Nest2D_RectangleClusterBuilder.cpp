#include "pch.h"
#include "Nest2D_RectangleClusterBuilder.h"
#include "Nest2D_ClusterGeometryHelper.h"
#include "Nest2D_RotationUtils.h"
#include "NestUtils.h"
#include "Nest2D_PrivateDataType.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

namespace ET {
    namespace NEST2DMANAGERLIB {

        // 两个矩形的长边和短边允许有 5% 的相对误差。
        constexpr double CET_RECT_SIZE_TOLERANCE = 0.05;

        // 判断正方形时使用更严格的 1% 误差。
        constexpr double CET_RECT_SQUARE_TOLERANCE = 0.01;

        // 第一阶段最多允许组合代理面积比基准面积扩大 10%。
        // 超过该值就认为组合明显变差。
        constexpr double CET_RECT_MAX_AREA_LOSS_RATIO = 0.10;
        namespace {

            bool NearlyEqual(double A, double B, double ARelativeTolerance)
            {
                const double Denominator = std::max(1.0, std::max(std::abs(A), std::abs(B)));
                return std::abs(A - B) <= Denominator * ARelativeTolerance;
            }

            void GetCanonicalSides(const TetShapeFeature& AFeature, double& AOutShortSide, double& AOutLongSide)
            {
                AOutShortSide = std::min(AFeature.OrientedWidth, AFeature.OrientedHeight);
                AOutLongSide = std::max(AFeature.OrientedWidth, AFeature.OrientedHeight);
            }
            bool _MakePose(const CetNestItem& AItem, const TetShapeFeature& AFeature, bool ARotate90, const TetNestOptions& AOptions, const CetClusterGeometryHelper& AGeometry, TetRectanglePose& AOutPose)
            {
                AOutPose = TetRectanglePose{};

                /*
                 * 先把矩形自身的倾斜角度消除，
                 * 使其中一条边与 X 轴平行。
                 *
                 * 如果 ARotate90 为 true，
                 * 再额外旋转 90°。
                 */
                const double TargetRotation = -AFeature.OrientedAngle + (ARotate90 ? CET_CLUSTER_HALF_PI : 0.0);
                if (!CetRotationUtils::SnapToNearestAllowedRotation(TargetRotation, AOptions.Rotations, AOutPose.Rotation)) {
                    return false;
                }
                const CetPath Contour = AGeometry.TransformContour(AGeometry.GetIdentityContour(AItem), AOutPose.Rotation, 0.0, 0.0);
                double MaxX = 0.0;
                double MaxY = 0.0;
                if (!AGeometry.GetBounds(Contour, AOutPose.MinX, AOutPose.MinY, MaxX, MaxY)) { return false; }

                AOutPose.Width = MaxX - AOutPose.MinX;
                AOutPose.Height = MaxY - AOutPose.MinY;

                return AOutPose.Width > 0.0 && AOutPose.Height > 0.0;
            }

        }

        CetRectangleClusterBuilder::CetRectangleClusterBuilder() : CetCoreObject() {}
        CetRectangleClusterBuilder::~CetRectangleClusterBuilder() {}

        void CetRectangleClusterBuilder::BuildCandidates(const CetTNestItemVector& AItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, std::vector<TetClusterCandidate>& AOut)
        {
            if (AItems.size() != AFeatures.size()) {
                std::cout << "[RECTANGLE][ERROR] " << "Feature count mismatch. Items=" << AItems.size() << ", Features=" << AFeatures.size() << std::endl;
                return;
            }

            if (AIndices.size() < 2) { return; }

            std::vector<int> ValidIndices;
            ValidIndices.reserve(AIndices.size());
            for (int Index : AIndices) {
                if (Index < 0 || Index >= static_cast<int>(AFeatures.size())) { continue; }
                if (_IsValidRectangle(AFeatures[Index])) {
                    ValidIndices.push_back(Index);
                }
            }

            std::sort(ValidIndices.begin(), ValidIndices.end());
            ValidIndices.erase(std::unique(ValidIndices.begin(), ValidIndices.end()), ValidIndices.end());
            if (ValidIndices.size() < 2) { return; }

            const std::size_t OldCandidateCount = AOut.size();
            const bool AllowQuarterTurn = _IsQuarterTurnAllowed(AOptions);
            std::vector<int> Remaining = ValidIndices;

            while (Remaining.size() >= 2) {
                const int IndexA = Remaining.front();
                auto PairIt = std::find_if(Remaining.begin() + 1, Remaining.end(), [&](int IndexB) {
                    return _AreCompatible(AFeatures[IndexA], AFeatures[IndexB]);
                    });

                if (PairIt == Remaining.end()) {
                    Remaining.erase(Remaining.begin());
                    continue;
                }

                const int IndexB = *PairIt;
                bool HasBestCandidate = false;
                TetClusterCandidate BestCandidate;
                auto TryLayout = [&](bool AHorizontal, bool ARotateB90) {
                    TetClusterCandidate Candidate;
                    if (!_MakePairCandidate(AItems, AFeatures, IndexA, IndexB, AHorizontal, ARotateB90, AOptions, Candidate)) {
                        return;
                    }
                    if (!HasBestCandidate || Candidate.Score > BestCandidate.Score) {
                        HasBestCandidate = true;
                        BestCandidate = std::move(Candidate);
                    }
                    };

                TryLayout(true, false);
                TryLayout(false, false);
                if (AllowQuarterTurn && !_IsSquareLike(AFeatures[IndexB])) {
                    TryLayout(true, true);
                    TryLayout(false, true);
                }

                const auto PairOffset = static_cast<std::vector<int>::difference_type>(std::distance(Remaining.begin(), PairIt));
                if (HasBestCandidate) {
                    AOut.push_back(std::move(BestCandidate));
                    Remaining.erase(Remaining.begin() + PairOffset);
                    Remaining.erase(Remaining.begin());
                }
                else {
                    Remaining.erase(Remaining.begin());
                }
            }

            std::cout << "[RECTANGLE][BUILD CANDIDATES] " << "IndexCount=" << ValidIndices.size() << ", NewCandidateCount=" << AOut.size() - OldCandidateCount << std::endl;
        }
        bool CetRectangleClusterBuilder::_IsValidRectangle(const TetShapeFeature& AFeature)
        {
            return AFeature.ShapeType == MetShapeType::RectangleLike && AFeature.IsRotatedRectangle && !AFeature.HasHoles && AFeature.Area > 0.0 && AFeature.OrientedWidth > 0.0 && AFeature.OrientedHeight > 0.0;
        }

        bool CetRectangleClusterBuilder::_AreCompatible(const TetShapeFeature& AFeatureA, const TetShapeFeature& AFeatureB)
        {
            double ShortA = 0.0;
            double LongA = 0.0;
            double ShortB = 0.0;
            double LongB = 0.0;

            GetCanonicalSides(AFeatureA, ShortA, LongA);
            GetCanonicalSides(AFeatureB, ShortB, LongB);

            return NearlyEqual(ShortA, ShortB, CET_RECT_SIZE_TOLERANCE) && NearlyEqual(LongA, LongB, CET_RECT_SIZE_TOLERANCE);
        }

        bool CetRectangleClusterBuilder::_IsSquareLike(const TetShapeFeature& AFeature)
        {
            return NearlyEqual(AFeature.OrientedWidth, AFeature.OrientedHeight, CET_RECT_SQUARE_TOLERANCE);
        }

        bool CetRectangleClusterBuilder::_IsQuarterTurnAllowed(const TetNestOptions& AOptions)
        {
            constexpr double RotationTolerance = 1e-9;
            return CetRotationUtils::IsAllowedRotation(CET_CLUSTER_HALF_PI, AOptions.Rotations, RotationTolerance);
        }

        bool CetRectangleClusterBuilder::_MakePairCandidate(const CetTNestItemVector& AItems, const std::vector<TetShapeFeature>& AFeatures, int AIndexA, int AIndexB, bool AHorizontal, bool ARotateB90, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate)
        {
            AOutCandidate = TetClusterCandidate{};

            if (AIndexA < 0 || AIndexB < 0 || AIndexA == AIndexB || AIndexA >= static_cast<int>(AItems.size()) || AIndexB >= static_cast<int>(AItems.size()) || AIndexA >= static_cast<int>(AFeatures.size()) || AIndexB >= static_cast<int>(AFeatures.size())) { return false; }

            const TetShapeFeature& FeatureA = AFeatures[AIndexA];
            const TetShapeFeature& FeatureB = AFeatures[AIndexB];

            if (!_IsValidRectangle(FeatureA) || !_IsValidRectangle(FeatureB) || !_AreCompatible(FeatureA, FeatureB)) { return false; }

            CetClusterGeometryHelper Geometry;

            TetRectanglePose PoseA;
            TetRectanglePose PoseB;

            if (!_MakePose(AItems[AIndexA], FeatureA, false, AOptions, Geometry, PoseA) || !_MakePose(AItems[AIndexB], FeatureB, ARotateB90, AOptions, Geometry, PoseB)) { return false; }

            /*
             * Spacing 转换为排样内部坐标。
             *
             * 横排时：
             * B.MinX = A.Width + Gap
             *
             * 竖排时：
             * B.MinY = A.Height + Gap
             *
             * 因此两个矩形包围盒之间会严格保留 Gap。
             */
            const double Gap = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));

            TetItemTransform TransformA;
            TransformA.OriginalId = AIndexA;
            TransformA.RelativeRotation = PoseA.Rotation;
            TransformA.RelativeX = -PoseA.MinX;
            TransformA.RelativeY = -PoseA.MinY;

            TetItemTransform TransformB;
            TransformB.OriginalId = AIndexB;
            TransformB.RelativeRotation = PoseB.Rotation;

            if (AHorizontal) {
                /*
                 * 横向排列：
                 *
                 * ┌─────────┐ Gap ┌─────────┐
                 * │    A    │     │    B    │
                 * └─────────┘     └─────────┘
                 */
                TransformB.RelativeX = PoseA.Width + Gap - PoseB.MinX;
                TransformB.RelativeY = -PoseB.MinY;
            }
            else {
                /*
                 * 纵向排列：
                 *
                 * ┌─────────┐
                 * │    A    │
                 * └─────────┘
                 *     Gap
                 * ┌─────────┐
                 * │    B    │
                 * └─────────┘
                 */
                TransformB.RelativeX = -PoseB.MinX;
                TransformB.RelativeY = PoseA.Height + Gap - PoseB.MinY;
            }

            AOutCandidate.BuilderName = "RectangleBuilder";
            AOutCandidate.ClusterType = AHorizontal ? "RectanglePairHorizontal" : "RectanglePairVertical";

            if (ARotateB90) {
                AOutCandidate.ClusterType += "RotatedB90";
            }

            AOutCandidate.OriginalIndices = { AIndexA, AIndexB };
            AOutCandidate.Transforms = { TransformA, TransformB };

            /*
             * 简单矩形并排主要用于完成第一阶段闭环，
             * 暂时不给它高于三角形、圆弧嵌套等组合的置信度。
             */
            AOutCandidate.Confidence = 0.60;

            /*
             * 统一交给 GeometryHelper：
             *
             * 1. 归一化局部坐标；
             * 2. 创建代理轮廓；
             * 3. 计算宽高和面积；
             * 4. 检查子件包含关系；
             * 5. 检查子件相交；
             * 6. 检查能否放入板材；
             * 7. 计算候选评分。
             */
            if (!Geometry.FinalizeCandidate(AItems, AOptions, AOutCandidate)) { return false; }

            /*
             * 当前 ClusterManager 会继续接收所有合法且
             * 原始零件尚未使用的候选。
             *
             * 因此 Builder 内先拦截明显扩大代理面积的组合，
             * 避免两个差异较大的矩形被强制组合。
             */
            if (AOutCandidate.AreaSavingRatio < -CET_RECT_MAX_AREA_LOSS_RATIO) { return false; }

            return true;
        }

    }
}
