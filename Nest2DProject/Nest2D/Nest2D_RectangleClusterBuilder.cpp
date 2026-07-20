#include "pch.h"
#include "Nest2D_RectangleClusterBuilder.h"
#include "Nest2D_ClusterGeometryHelper.h"
#include "NestUtils.h"
#include "Nest2D_PrivateDataType.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

namespace ET {
    namespace NEST2DMANAGERLIB {
        constexpr double CET_RECT_PI = 3.14159265358979323846;
        constexpr double CET_RECT_HALF_PI = CET_RECT_PI * 0.5;

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
            bool _MakePose(const CetNestItem& AItem, const TetShapeFeature& AFeature, bool ARotate90, const CetClusterGeometryHelper& AGeometry, TetRectanglePose& AOutPose)
            {
                AOutPose = TetRectanglePose{};

                /*
                 * 先把矩形自身的倾斜角度消除，
                 * 使其中一条边与 X 轴平行。
                 *
                 * 如果 ARotate90 为 true，
                 * 再额外旋转 90°。
                 */
                AOutPose.Rotation = -AFeature.OrientedAngle + (ARotate90 ? CET_RECT_HALF_PI : 0.0);
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

            /*
             * 先过滤掉：
             *
             * 1. 越界索引；
             * 2. 非矩形；
             * 3. 带孔矩形；
             * 4. 未成功识别方向的矩形；
             * 5. 面积或尺寸无效的矩形。
             */
            std::vector<int> ValidIndices;
            ValidIndices.reserve(AIndices.size());
            for (int Index : AIndices) {
                if (Index < 0 || Index >= static_cast<int>(AFeatures.size())) { continue; }
                if (_IsValidRectangle(AFeatures[Index])) {
                    ValidIndices.push_back(Index);
                }
            }
            /*
             * 保证同一个原始零件索引只出现一次。
             */
            std::sort(ValidIndices.begin(), ValidIndices.end());
            ValidIndices.erase(std::unique(ValidIndices.begin(), ValidIndices.end()), ValidIndices.end());
            if (ValidIndices.size() < 2) { return; }
            const std::size_t OldCandidateCount = AOut.size();
            const bool AllowQuarterTurn = _IsQuarterTurnAllowed(AOptions);
            /*
             * 第一阶段采用两两组合。
             *
             * 后续如果实现四矩形块、多层组合或组合件再组合，
             * 可以在这里继续添加新的候选生成入口。
             */
            for (std::size_t i = 0; i < ValidIndices.size(); ++i) {
                for (std::size_t j = i + 1; j < ValidIndices.size(); ++j) {
                    const int IndexA = ValidIndices[i];
                    const int IndexB = ValidIndices[j];
                    /*
                     * 第一阶段只处理长短边尺寸相近的矩形。
                     * 不直接组合任意大小矩形，避免产生大量低质量候选。
                     */
                    if (!_AreCompatible(AFeatures[IndexA], AFeatures[IndexB])) { continue; }
                    /*
                     * LayoutMode == 0：横排
                     * LayoutMode == 1：竖排
                     */
                    for (int LayoutMode = 0; LayoutMode < 2; ++LayoutMode) {
                        const bool Horizontal = LayoutMode == 0;
                        TetClusterCandidate Candidate;

                        /*
                         * 候选一：
                         * 两个矩形均按照自身方向校正后排列。
                         */
                        if (_MakePairCandidate(AItems, AFeatures, IndexA, IndexB, Horizontal, false, AOptions, Candidate)) {
                            AOut.push_back(std::move(Candidate));
                        }
                        /*
                         * 如果旋转设置包含 90°，继续生成：
                         *
                         * A 保持校正方向；
                         * B 在校正方向基础上额外旋转 90°。
                         *
                         * 正方形旋转 90° 不会形成新布局，因此跳过。
                         */
                        if (!AllowQuarterTurn || _IsSquareLike(AFeatures[IndexB])) { continue; }
                        TetClusterCandidate RotatedCandidate;
                        if (_MakePairCandidate(AItems, AFeatures, IndexA, IndexB, Horizontal, true, AOptions, RotatedCandidate)) {
                            AOut.push_back(std::move(RotatedCandidate));
                        }
                    }
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
            /*
                 * Rotations 表示整圆离散角度数量。
                 *
                 * Rotations = 1：
                 * 只有 0°
                 *
                 * Rotations = 2：
                 * 0°、180°
                 *
                 * Rotations = 4：
                 * 0°、90°、180°、270°
                 *
                 * 因此只有 Rotations 是 4 的倍数时，
                 * 离散旋转集合中才精确包含 90°。
                 */
            return AOptions.Rotations >= 4 && AOptions.Rotations % 4 == 0;
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

            if (!_MakePose(AItems[AIndexA], FeatureA, false, Geometry, PoseA) || !_MakePose(AItems[AIndexB], FeatureB, ARotateB90, Geometry, PoseB)) { return false; }

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
