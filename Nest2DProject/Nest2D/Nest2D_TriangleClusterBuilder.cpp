#include "pch.h"
#include "Nest2D_TriangleClusterBuilder.h"
#include"Nest2D_ClusterGeometryHelper.h"
#include"NestUtils.h"

#include <algorithm>
#include <cmath>
#include <iostream>

using namespace ClipperLib;
using namespace libnest2d;
namespace ET {
    namespace NEST2DMANAGERLIB {
        CetTriangleClusterBuilder::CetTriangleClusterBuilder() : CetCoreObject()
        {
        }
        CetTriangleClusterBuilder::~CetTriangleClusterBuilder()
        {
        }
        bool CetTriangleClusterBuilder::TryMakeRightTrianglePair(const CetTNestItemVector& AOriginalItems, int AAIndex, int ABIndex, const TetNestOptions& AOptions, TetClusterBuildResult& AResult)
        {
            const auto& ItemA = AOriginalItems[AAIndex];
            const auto& ItemB = AOriginalItems[ABIndex];

            double WA = _GetItemWidth(ItemA);
            double HA = _GetItemHeight(ItemA);
            double WB = _GetItemWidth(ItemB);
            double HB = _GetItemHeight(ItemB);

            double AreaA = std::abs(static_cast<double>(ItemA.area()));
            double AreaB = std::abs(static_cast<double>(ItemB.area()));

            bool RightA = _IsRightTriangleLike(ItemA);
            bool RightB = _IsRightTriangleLike(ItemB);

            if (!RightA || !RightB) {
                std::cout << "[CLUSTER][REJECT] not right triangle: "
                    << AAIndex << " + " << ABIndex
                    << ", A(W,H,Area)=(" << WA << "," << HA << "," << AreaA << ")"
                    << ", B(W,H,Area)=(" << WB << "," << HB << "," << AreaB << ")"
                    << ", A ratio=" << (WA * HA > 0.0 ? AreaA * 2.0 / (WA * HA) : 0.0)
                    << ", B ratio=" << (WB * HB > 0.0 ? AreaB * 2.0 / (WB * HB) : 0.0)
                    << std::endl;

                return false;
            }

            if (!_IsSameSizeTrianglePair(ItemA, ItemB)) {
                std::cout << "[CLUSTER][REJECT] size mismatch: "
                    << AAIndex << " + " << ABIndex
                    << ", A(W,H)=(" << WA << "," << HA << ")"
                    << ", B(W,H)=(" << WB << "," << HB << ")"
                    << std::endl;

                return false;
            }
            double W = WA;
            double H = HA;
            if (W <= 0.0 || H <= 0.0) {
                return false;
            }

            double InternalSpacing = static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing));
            double AxisGap = _CalcTrianglePairAxisGap(W, H, InternalSpacing);
            double ClusterW = W + AxisGap;
            double ClusterH = H + AxisGap;

            double BinW = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth));
            double BinH = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinHeight));
            if (ClusterW > BinW || ClusterH > BinH) {
                std::cout << "[CLUSTER][REJECT] cluster bigger than bin: "
                    << AAIndex << " + " << ABIndex
                    << ", W = " << W
                    << ", H = " << H
                    << ", InternalSpacing = " << InternalSpacing
                    << ", AxisGap = " << AxisGap
                    << ", ClusterW = " << ClusterW
                    << ", ClusterH = " << ClusterH
                    << ", BinW = " << BinW
                    << ", BinH = " << BinH
                    << std::endl;

                return false;
            }
            auto ClusterItem = _MakeRectangleNestItem(ClusterW, ClusterH);
            const int PackedIndex = static_cast<int>(AResult.NestItems.size());
            AResult.NestItems.push_back(std::move(ClusterItem));

            TetMetaItem Meta;
            Meta.PackedItemIndex = PackedIndex;
            Meta.IsCluster = true;
            Meta.ClusterType = "RightTrianglePair";

            TetItemTransform TransformA;
            TransformA.OriginalId = AAIndex;
            TransformA.RelativeX = 0.0;
            TransformA.RelativeY = 0.0;
            TransformA.RelativeRotation = 0.0;
            Meta.TransformData.push_back(TransformA);

            TetItemTransform TransformB;
            TransformB.OriginalId = ABIndex;
            TransformB.RelativeX = ClusterW;
            TransformB.RelativeY = ClusterH;
            TransformB.RelativeRotation = CET_CLUSTER_PI;
            Meta.TransformData.push_back(TransformB);

            AResult.MetaItems.push_back(Meta);

            std::cout << "[CLUSTER] RightTrianglePair created: "
                << AAIndex << " + " << ABIndex
                << ", W = " << W
                << ", H = " << H
                << ", InternalSpacing = " << InternalSpacing
                << ", AxisGap = " << AxisGap
                << ", ClusterW = " << ClusterW
                << ", ClusterH = " << ClusterH
                << ", PackedIndex = " << PackedIndex
                << std::endl;

            return true;
        }
        void CetTriangleClusterBuilder::BuildCandidates(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, std::vector<TetClusterCandidate>& AOutCandidates) 
        {
            if (AOriginalItems.size() != AFeatures.size()) {
                std::cout << "[TRIANGLE][ERROR] Feature count mismatch." << std::endl;
                return;
            }

            if (AIndices.size() < 2) { return; }

            const std::size_t OldCandidateCount = AOutCandidates.size();

            for (std::size_t i = 0; i < AIndices.size(); ++i) {
                for (std::size_t j = i + 1; j < AIndices.size(); ++j) {
                    const int AIndex = AIndices[i];
                    const int BIndex = AIndices[j];

                    if (AIndex < 0 || BIndex < 0 || AIndex >= static_cast<int>(AFeatures.size()) || BIndex >= static_cast<int>(AFeatures.size())) { continue; }

                    const TetShapeFeature& FeatureA = AFeatures[AIndex];
                    const TetShapeFeature& FeatureB = AFeatures[BIndex];

                    if (FeatureA.ShapeType != MetShapeType::TriangleLike || FeatureB.ShapeType != MetShapeType::TriangleLike) { continue; }

                    
                     //第一版先只处理全等或近似全等三角形。
                     //注意：这里不再要求 RightTriangle。
                     
                    if (!_AreCongruentTriangles(FeatureA, FeatureB)) {
                        std::cout << "[TRIANGLE][REJECT] not congruent: " << AIndex << " + " << BIndex << std::endl;
                        continue;
                    }

                    TetClusterCandidate BestCandidate;
                    bool HasBestCandidate = false;
                    /*
                     * 旧直角三角形模板必须保留。
                     * 这样 AnyTrianglePair 第一版不稳定时，
                     * 不会破坏原本已经跑通的直角三角形组合能力。
                     */
                    if (FeatureA.TriangleAngleType == MetTriangleAngleType::Right && FeatureB.TriangleAngleType == MetTriangleAngleType::Right) {
                        TetClusterCandidate RightCandidate;
                        if (_BuildRightTrianglePairCandidate(AOriginalItems, AFeatures, AIndex, BIndex, AOptions, RightCandidate)) {
                            HasBestCandidate = true;
                            BestCandidate = std::move(RightCandidate);

                            std::cout << "[TRIANGLE][CANDIDATE] " << AIndex << " + " << BIndex << " Type=RightTrianglePair" << std::endl;
                        }
                    }

                    // 新的任意三角形边匹配。
                   
                    {
                        TetClusterCandidate AnyCandidate;

                        if (_BuildAnyTrianglePairCandidate(AOriginalItems, AFeatures, AIndex, BIndex, AOptions, AnyCandidate)) {
                            if (!HasBestCandidate || AnyCandidate.Score > BestCandidate.Score) {
                                HasBestCandidate = true;
                                BestCandidate = std::move(AnyCandidate);
                            }

                            std::cout << "[TRIANGLE][CANDIDATE] " << AIndex << " + " << BIndex << " Type=AnyTrianglePair" << std::endl;
                        }
                        else {
                            std::cout << "[TRIANGLE][WARN] any triangle build failed: " << AIndex << " + " << BIndex << std::endl;
                        }
                    }

                    if (!HasBestCandidate) {
                        std::cout << "[TRIANGLE][REJECT] all triangle build failed: " << AIndex << " + " << BIndex << std::endl;
                        continue;
                    }
                    std::cout<< "[TRIANGLE][CANDIDATE][FINAL] "<< AIndex << " + " << BIndex<< " Type=" << BestCandidate.ClusterType<< ", Score=" << BestCandidate.Score<< std::endl;
                    AOutCandidates.push_back(std::move(BestCandidate));
                    //std::cout << "[TRIANGLE][CANDIDATE] " << AIndex << " + " << BIndex << " Type=AnyTrianglePair" << std::endl;
                }
            }
            std::cout << "[TRIANGLE][BUILD CANDIDATES] IndexCount=" << AIndices.size() << ", NewCandidateCount=" << AOutCandidates.size() - OldCandidateCount << std::endl;

        }

        bool CetTriangleClusterBuilder::_IsRightTriangleLike(const CetNestItem& AItem)
        {
            double W = _GetItemWidth(AItem);
            double H = _GetItemHeight(AItem);
            if (W <= 0.0 || H <= 0.0) {
                return false;
            }
            double BoxArea = std::abs(W * H);
            double ItemArea = std::abs(static_cast<double>(AItem.area()));
            if (ItemArea <= 0.0 || BoxArea <= 0.0) {
                return false;
            }
            double Ratio = ItemArea * 2.0 / BoxArea;
            // 直角三角形：Area ≈ W * H / 2，所以 Ratio ≈ 1
            return std::abs(Ratio - 1.0) <= 0.08;
        }
        bool CetTriangleClusterBuilder::_IsSameSizeTrianglePair(const CetNestItem& AItem, const CetNestItem& BItem)
        {
            double WA = _GetItemWidth(AItem);
            double HA = _GetItemHeight(AItem);
            double WB = _GetItemWidth(BItem);
            double HB = _GetItemHeight(BItem);
            bool SameDirection = _NearlyEqual(WA, WB, 0.05) && _NearlyEqual(HA, HB, 0.05);
            bool SwappedDirection = _NearlyEqual(WA, HB, 0.05) && _NearlyEqual(HA, WB, 0.05);
            return SameDirection || SwappedDirection;
        }
        bool CetTriangleClusterBuilder::_NearlyEqual(double A, double B, double RelTol)
        {
            double Den = std::max(1.0, std::max(std::abs(A), std::abs(B)));
            return std::abs(A - B) <= Den * RelTol;
        }  
        double CetTriangleClusterBuilder::_GetItemWidth(const CetNestItem& AItem)
        {
            return static_cast<double>(AItem.boundingBox().width());
        }
        double CetTriangleClusterBuilder::_GetItemHeight(const CetNestItem& AItem)
        {
            return static_cast<double>(AItem.boundingBox().height());
        }
        double CetTriangleClusterBuilder::_CalcTrianglePairAxisGap(double AW, double AH, double ASpacing)
        {
            if (AW <= 0.0 || AH <= 0.0 || ASpacing <= 0.0) {
                return 0.0;
            }
            double AxisGap = ASpacing * std::sqrt(AW * AW + AH * AH) / (AW + AH);
            // 因为后面会转成整数坐标，向上取整，避免实际间隙被截断变小
            return std::ceil(AxisGap);
        }
        CetNestItem CetTriangleClusterBuilder::_MakeRectangleNestItem(double AWidth, double AHeight)
        {
            using namespace libnest2d;
            Path outerPoints;
            outerPoints.reserve(4);
            outerPoints.push_back(Point(0, 0));
            outerPoints.push_back(Point(static_cast<ClipperLib::cInt>(AWidth), 0));
            outerPoints.push_back(Point(static_cast<ClipperLib::cInt>(AWidth), static_cast<ClipperLib::cInt>(AHeight)));
            outerPoints.push_back(Point(0, static_cast<ClipperLib::cInt>(AHeight)));
            if (ClipperLib::Orientation(outerPoints) == false) {
                std::reverse(outerPoints.begin(), outerPoints.end());
            }
            Paths holes;
            PolygonImpl poly(std::move(outerPoints), std::move(holes));
            return CetTNestItemVector::value_type(std::move(poly));
        }
        bool CetTriangleClusterBuilder::_BuildRightTrianglePairCandidate(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, int AAIndex, int ABIndex, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate)
        {
            if (AAIndex < 0 || ABIndex < 0 ||
                AAIndex >= static_cast<int>(AFeatures.size()) ||
                ABIndex >= static_cast<int>(AFeatures.size()) ||
                AAIndex == ABIndex)
            {
                return false;
            }

            const TetShapeFeature& FA = AFeatures[AAIndex];
            const TetShapeFeature& FB = AFeatures[ABIndex];

            if (FA.TriangleAngleType != MetTriangleAngleType::Right ||
                FB.TriangleAngleType != MetTriangleAngleType::Right)
            {
                return false;
            }

            if (!_AreCongruentTriangles(FA, FB))
            {
                return false;
            }

            const double Gap = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));

            AOutCandidate = TetClusterCandidate{};
            AOutCandidate.BuilderName = "TriangleBuilder";
            AOutCandidate.ClusterType = "RightTrianglePair";
            AOutCandidate.OriginalIndices = { AAIndex, ABIndex };

            TetItemTransform TA;
            TA.OriginalId = AAIndex;
            TA.RelativeX = -FA.MinX;
            TA.RelativeY = -FA.MinY;

            TetItemTransform TB;
            TB.OriginalId = ABIndex;
            TB.RelativeRotation = CET_CLUSTER_PI;
            // 旋转180度后放到右上角，随后由 GeometryHelper 精确归一化和校验。
            TB.RelativeX = FA.Width + Gap - FB.MinX;
            TB.RelativeY = FA.Height + Gap - FB.MinY;

            AOutCandidate.Transforms = { TA, TB };
            AOutCandidate.Confidence = 1.0;

            CetClusterGeometryHelper Geometry;
            return Geometry.FinalizeCandidate(AOriginalItems, AOptions, AOutCandidate);
           
        }
        bool CetTriangleClusterBuilder::_AreCongruentTriangles(const TetShapeFeature& AA, const TetShapeFeature& AB)
        {
            if (AA.ShapeType != MetShapeType::TriangleLike ||AB.ShapeType != MetShapeType::TriangleLike){
                return false;
            }
            for (int i = 0; i < 3; ++i) if (!_NearlyEqual(AA.TriangleSides[i], AB.TriangleSides[i], 0.03)) return false;
            return true;
        }
        bool CetTriangleClusterBuilder::_BuildAnyTrianglePairCandidate(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, int AAIndex, int ABIndex, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate)
        {
            bool HasBest = false;
            TetClusterCandidate BestCandidate;
            int EdgeTryCount = 0;
            int EdgeSuccessCount = 0;
            /*
             * 三角形有 3 条边。
             * A 的每条边都尝试和 B 的每条边匹配。
             * ANormalSide = 1 / -1 表示把 B 放在 A 边的两侧分别尝试。
             */
            for (int AEdgeIndex = 0; AEdgeIndex < 3; ++AEdgeIndex) {
                for (int BEdgeIndex = 0; BEdgeIndex < 3; ++BEdgeIndex) {
                    for (int NormalSide : { 1, -1 }) {
                        ++EdgeTryCount;
                        TetClusterCandidate Candidate;

                        if (!_TryBuildTriangleEdgePairCandidate(AOriginalItems, AFeatures, AAIndex, ABIndex, AEdgeIndex, BEdgeIndex, NormalSide, AOptions, Candidate)) {
                            continue;
                        }
                        ++EdgeSuccessCount;
                        if (!HasBest || Candidate.Score > BestCandidate.Score) {
                            HasBest = true;
                            BestCandidate = std::move(Candidate);
                        }
                    }
                }
            }
            std::cout<< "[TRIANGLE][ANY][EDGE SUMMARY] "<< "A=" << AAIndex<< ", B=" << ABIndex<< ", EdgeTryCount=" << EdgeTryCount<< ", EdgeSuccessCount=" << EdgeSuccessCount<< std::endl;
            if (!HasBest)
            {
                TetClusterCandidate OppositeCandidate;
                if (_BuildOppositeTrianglePairCandidate(AOriginalItems, AFeatures, AAIndex, ABIndex, AOptions, OppositeCandidate))
                {
                    std::cout << "[TRIANGLE][ANY][FALLBACK] Opposite triangle pair accepted. "
                        << "A=" << AAIndex << ", B=" << ABIndex
                        << ", Score=" << OppositeCandidate.Score << std::endl;
                    AOutCandidate = std::move(OppositeCandidate);
                    return true;
                }

                std::cout << "[TRIANGLE][ANY][REJECT] no valid edge-pair candidate. "
                    << "A=" << AAIndex << ", B=" << ABIndex << std::endl;
                return false;
            }

            AOutCandidate = std::move(BestCandidate);
            return true;
        }
        bool CetTriangleClusterBuilder::_TryBuildTriangleEdgePairCandidate(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, int AAIndex, int ABIndex, int AEdgeIndex, int BEdgeIndex, int ANormalSide, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate)
        {
            (void)ANormalSide;

            if (AAIndex < 0 || ABIndex < 0 || AAIndex == ABIndex ||
                AAIndex >= static_cast<int>(AOriginalItems.size()) || ABIndex >= static_cast<int>(AOriginalItems.size()) ||
                AAIndex >= static_cast<int>(AFeatures.size()) || ABIndex >= static_cast<int>(AFeatures.size())) {
                return false;
            }
            const TetShapeFeature& FeatureA = AFeatures[AAIndex];
            const TetShapeFeature& FeatureB = AFeatures[ABIndex];
            if (FeatureA.ShapeType != MetShapeType::TriangleLike || FeatureB.ShapeType != MetShapeType::TriangleLike) {
                return false;
            }
            if (!_AreCongruentTriangles(FeatureA, FeatureB)) {
                return false;
            }

            CetClusterGeometryHelper Geometry;
            CetPath ContourA = Geometry.GetIdentityContour(AOriginalItems[AAIndex]);
            CetPath ContourB = Geometry.GetIdentityContour(AOriginalItems[ABIndex]);
            // 去除首尾重合的点
            auto removeDuplicateStartEnd = [](CetPath& path) {
                if (path.size() > 3 && path.front().X == path.back().X && path.front().Y == path.back().Y) {
                    path.pop_back();
                }
                };
            removeDuplicateStartEnd(ContourA);
            removeDuplicateStartEnd(ContourB);

            if (ContourA.size() != 3 || ContourB.size() != 3) {
                std::cout << "Failed at Contour Size. A=" << ContourA.size() << ", B=" << ContourB.size() << std::endl;
                return false;
            }
            TetTriangleEdgePose EdgeA;
            TetTriangleEdgePose EdgeB;
            if (!_GetTriangleEdgePose(ContourA, AEdgeIndex, EdgeA) || !_GetTriangleEdgePose(ContourB, BEdgeIndex, EdgeB)) {
                return false;
            }
            if (!_NearlyEqual(EdgeA.Length, EdgeB.Length, 0.03)) {
                std::cout << "Failed at Edge Length. A=" << EdgeA.Length << ", B=" << EdgeB.Length << std::endl;
                return false;
            }
            CetInpoint AThird;
            CetInpoint BThird;
            if (!_GetTriangleThirdPoint(ContourA, AEdgeIndex, AThird) || !_GetTriangleThirdPoint(ContourB, BEdgeIndex, BThird)) {
                return false;
            }

            const double ASide = _Cross(EdgeA.Start, EdgeA.End, AThird);
            if (std::abs(ASide) <= 1.0) {
                return false;
            }

            /* 让 B 的匹配边和 A 的匹配边反向平行。 */
            const double RotationB = _NormalizeAngle(EdgeA.Angle + CET_CLUSTER_PI - EdgeB.Angle);

            double AMinX = 0.0, AMinY = 0.0, AMaxX = 0.0, AMaxY = 0.0;
            if (!Geometry.GetBounds(ContourA, AMinX, AMinY, AMaxX, AMaxY)) {
                return false;
            }

            const double ATranslationX = -AMinX;
            const double ATranslationY = -AMinY;
            const double AStartX = static_cast<double>(EdgeA.Start.X) + ATranslationX;
            const double AStartY = static_cast<double>(EdgeA.Start.Y) + ATranslationY;
            const double AEndX = static_cast<double>(EdgeA.End.X) + ATranslationX;
            const double AEndY = static_cast<double>(EdgeA.End.Y) + ATranslationY;

            const double EdgeDX = AEndX - AStartX;
            const double EdgeDY = AEndY - AStartY;
            const double EdgeLen = std::sqrt(EdgeDX * EdgeDX + EdgeDY * EdgeDY);

            if (EdgeLen <= 0.0) {
                return false;
            }

            const double UnitX = EdgeDX / EdgeLen;
            const double UnitY = EdgeDY / EdgeLen;
            const double RequiredGap = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));
            const double SafetyGap = RequiredGap > 0.0 ? std::max(10.0, RequiredGap * 0.001) : 0.0;
            const double Gap = RequiredGap + SafetyGap;

            const CetInpoint RotatedBStart = _RotatePoint(EdgeB.Start, RotationB);
            const CetInpoint RotatedBEnd = _RotatePoint(EdgeB.End, RotationB);
            const CetInpoint RotatedBThird = _RotatePoint(BThird, RotationB);

            const double AMidX = (AStartX + AEndX) * 0.5;
            const double AMidY = (AStartY + AEndY) * 0.5;
            const double BMidX = (static_cast<double>(RotatedBStart.X) + static_cast<double>(RotatedBEnd.X)) * 0.5;
            const double BMidY = (static_cast<double>(RotatedBStart.Y) + static_cast<double>(RotatedBEnd.Y)) * 0.5;

            std::vector<TetBaseOffset> BaseOffsets;

            /* 1. 中点对中点。 */
            BaseOffsets.push_back({ AMidX - BMidX, AMidY - BMidY });
            /* 2. A.Start 对 B.End。 */
            BaseOffsets.push_back({ AStartX - static_cast<double>(RotatedBEnd.X), AStartY - static_cast<double>(RotatedBEnd.Y) });
            /* 3. A.End 对 B.Start。 */
            BaseOffsets.push_back({ AEndX - static_cast<double>(RotatedBStart.X), AEndY - static_cast<double>(RotatedBStart.Y) });
            /* 4. A.Start 对 B.Start。对部分顶点顺序不同的三角形更稳。 */
            BaseOffsets.push_back({ AStartX - static_cast<double>(RotatedBStart.X), AStartY - static_cast<double>(RotatedBStart.Y) });
            /* 5. A.End 对 B.End。 */
            BaseOffsets.push_back({ AEndX - static_cast<double>(RotatedBEnd.X), AEndY - static_cast<double>(RotatedBEnd.Y) });

            bool HasBest = false;
            TetClusterCandidate BestCandidate;
            int SideRejectCount = 0;
            int FinalizeRejectCount = 0;

            for (const auto& BaseOffset : BaseOffsets) {
                /* 先不加 Gap，判断 B 第三个点在基础对齐位置下位于 A 匹配边的哪一侧。 */
                const double BThirdBaseX = static_cast<double>(RotatedBThird.X) + BaseOffset.X;
                const double BThirdBaseY = static_cast<double>(RotatedBThird.Y) + BaseOffset.Y;
                const double BaseAPX = BThirdBaseX - AStartX;
                const double BaseAPY = BThirdBaseY - AStartY;
                const double BSideBase = EdgeDX * BaseAPY - EdgeDY * BaseAPX;

                /* A 和 B 的三角形主体必须在匹配边两侧。 */
                if (ASide * BSideBase >= 0.0) {
                    ++SideRejectCount;
                    continue;
                }

                /*
                 * BSideBase > 0 代表 B 在 A 边左侧；BSideBase < 0 代表 B 在 A 边右侧。
                 * Gap 应该继续沿 B 所在侧推出去。
                 */
                const double OffsetSign = BSideBase > 0.0 ? 1.0 : -1.0;
                const double BTranslationX = BaseOffset.X + (-UnitY) * OffsetSign * Gap;
                const double BTranslationY = BaseOffset.Y + UnitX * OffsetSign * Gap;

                /* 加 Gap 后，再确认 B 仍在 A 的另一侧。 */
                const double BThirdX = static_cast<double>(RotatedBThird.X) + BTranslationX;
                const double BThirdY = static_cast<double>(RotatedBThird.Y) + BTranslationY;
                const double APX = BThirdX - AStartX;
                const double APY = BThirdY - AStartY;
                const double BSide = EdgeDX * APY - EdgeDY * APX;

                if (ASide * BSide >= 0.0) {
                    ++SideRejectCount;
                    continue;
                }

                TetClusterCandidate Candidate;
                Candidate.BuilderName = "TriangleBuilder";
                Candidate.ClusterType = "AnyTriangleEdgePair";
                Candidate.OriginalIndices = { AAIndex, ABIndex };

                TetItemTransform TransformA;
                TransformA.OriginalId = AAIndex;
                TransformA.RelativeX = ATranslationX;
                TransformA.RelativeY = ATranslationY;
                TransformA.RelativeRotation = 0.0;

                TetItemTransform TransformB;
                TransformB.OriginalId = ABIndex;
                TransformB.RelativeX = BTranslationX;
                TransformB.RelativeY = BTranslationY;
                TransformB.RelativeRotation = RotationB;

                Candidate.Transforms = { TransformA, TransformB };
                Candidate.Confidence = 0.90;

                if (!Geometry.FinalizeCandidate(AOriginalItems, AOptions, Candidate)) {
                    ++FinalizeRejectCount;
                    continue;
                }

                const double LengthMatchRatio = std::min(EdgeA.Length, EdgeB.Length) / std::max(EdgeA.Length, EdgeB.Length);
                Candidate.Score += LengthMatchRatio * 50.0;

                if (!HasBest || Candidate.Score > BestCandidate.Score) {
                    HasBest = true;
                    BestCandidate = std::move(Candidate);
                }
            }

            if (!HasBest) {
                std::cout << "[TRIANGLE][EDGE][REJECT] A=" << AAIndex << ", B=" << ABIndex
                    << ", AEdge=" << AEdgeIndex << ", BEdge=" << BEdgeIndex
                    << ", SideReject=" << SideRejectCount << ", FinalizeReject=" << FinalizeRejectCount << std::endl;
                return false;
            }

            AOutCandidate = std::move(BestCandidate);
            return true;
        }
        bool CetTriangleClusterBuilder::_GetTriangleEdgePose(const CetPath& AContour, int AEdgeIndex, TetTriangleEdgePose& AOutEdge)
        {
            if (AContour.size() != 3) { return false; }
            if (AEdgeIndex < 0 || AEdgeIndex >= 3) { return false; }

            const int NextIndex = (AEdgeIndex + 1) % 3;

            AOutEdge = TetTriangleEdgePose{};
            AOutEdge.Start = AContour[AEdgeIndex];
            AOutEdge.End = AContour[NextIndex];

            const double DX = static_cast<double>(AOutEdge.End.X - AOutEdge.Start.X);
            const double DY = static_cast<double>(AOutEdge.End.Y - AOutEdge.Start.Y);
            AOutEdge.Length = std::sqrt(DX * DX + DY * DY);

            if (AOutEdge.Length <= 0.0) { return false; }

            AOutEdge.Angle = std::atan2(DY, DX);

            return true;
        }
        double CetTriangleClusterBuilder::_NormalizeAngle(double AAngle)
        {
            while (AAngle > CET_CLUSTER_PI) {
                AAngle -= CET_CLUSTER_TWO_PI;
            }
            while (AAngle <= -CET_CLUSTER_PI) {
                AAngle += CET_CLUSTER_TWO_PI;
            }
            return AAngle;
        }
        CetInpoint CetTriangleClusterBuilder::_RotatePoint(const CetInpoint& APoint, double ARotation)
        {
            const double CosVal = std::cos(ARotation);
            const double SinVal = std::sin(ARotation);

            const double X = static_cast<double>(APoint.X);
            const double Y = static_cast<double>(APoint.Y);

            return CetInpoint(
                static_cast<ClipperLib::cInt>(std::llround(X * CosVal - Y * SinVal)),
                static_cast<ClipperLib::cInt>(std::llround(X * SinVal + Y * CosVal))
            );

        }
        bool CetTriangleClusterBuilder::_BuildOppositeTrianglePairCandidate(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, int AAIndex, int ABIndex, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate)
        {
            if (AAIndex < 0 || ABIndex < 0 || AAIndex == ABIndex ||
                AAIndex >= static_cast<int>(AOriginalItems.size()) ||
                ABIndex >= static_cast<int>(AOriginalItems.size()) ||
                AAIndex >= static_cast<int>(AFeatures.size()) ||
                ABIndex >= static_cast<int>(AFeatures.size()))
            {
                return false;
            }

            const TetShapeFeature& FeatureA = AFeatures[AAIndex];
            const TetShapeFeature& FeatureB = AFeatures[ABIndex];

            if (FeatureA.ShapeType != MetShapeType::TriangleLike ||
                FeatureB.ShapeType != MetShapeType::TriangleLike)
            {
                return false;
            }

            if (!_AreCongruentTriangles(FeatureA, FeatureB))
            {
                return false;
            }

            CetClusterGeometryHelper Geometry;
            CetPath PathA = Geometry.GetIdentityContour(AOriginalItems[AAIndex]);
            CetPath PathB = Geometry.TransformContour(Geometry.GetIdentityContour(AOriginalItems[ABIndex]), CET_CLUSTER_PI, 0.0, 0.0);

            double AMinX = 0.0, AMinY = 0.0, AMaxX = 0.0, AMaxY = 0.0;
            double BMinX = 0.0, BMinY = 0.0, BMaxX = 0.0, BMaxY = 0.0;

            if (!Geometry.GetBounds(PathA, AMinX, AMinY, AMaxX, AMaxY)) return false;
            if (!Geometry.GetBounds(PathB, BMinX, BMinY, BMaxX, BMaxY)) return false;

            const double RequiredGap = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));
            const double SafetyGap = RequiredGap > 0.0 ? std::max(10.0, RequiredGap * 0.001) : 0.0;
            const double Gap = RequiredGap + SafetyGap;

            // A 正放，先归一化到局部原点
            TetItemTransform TransformA;
            TransformA.OriginalId = AAIndex;
            TransformA.RelativeRotation = 0.0;
            TransformA.RelativeX = -AMinX;
            TransformA.RelativeY = -AMinY;

            // B 旋转 180 度，放到 A 上方：A 的最高点 + Gap 对齐 B 旋转后的最低点
            TetItemTransform TransformB;
            TransformB.OriginalId = ABIndex;
            TransformB.RelativeRotation = CET_CLUSTER_PI;

            // X 方向居中对齐
            const double ACenterX = (AMaxX - AMinX) * 0.5;
            const double BWidth = BMaxX - BMinX;
            TransformB.RelativeX = ACenterX - BWidth * 0.5 - BMinX;
            TransformB.RelativeY = (AMaxY - AMinY) + Gap - BMinY;

            AOutCandidate = TetClusterCandidate{};
            AOutCandidate.BuilderName = "TriangleBuilder";
            AOutCandidate.ClusterType = "TriangleOppositePair";
            AOutCandidate.OriginalIndices = { AAIndex, ABIndex };
            AOutCandidate.Transforms = { TransformA, TransformB };
            AOutCandidate.Confidence = 0.75;

            if (!Geometry.FinalizeCandidate(AOriginalItems, AOptions, AOutCandidate))
            {
                return false;
            }

            return true;
        }
        //判断点在哪一侧
        double CetTriangleClusterBuilder::_Cross(const CetInpoint& AA, const CetInpoint& AB, CetInpoint& AP)
        {
            const double ABX = static_cast<double>(AB.X - AA.X);
            const double ABY = static_cast<double>(AB.Y - AA.Y);

            const double APX = static_cast<double>(AP.X - AA.X);
            const double APY = static_cast<double>(AP.Y - AA.Y);

            return ABX * APY - ABY * APX;
        }
        //获取第三个点
        bool CetTriangleClusterBuilder::_GetTriangleThirdPoint(const CetPath& AContour, int AEdgeIndex, CetInpoint& AOutPoint)
        {
            if (AContour.size() != 3) {return false;}
            if (AEdgeIndex < 0 || AEdgeIndex >= 3) {return false;}
            const int ThirdIndex = (AEdgeIndex + 2) % 3;
            AOutPoint = AContour[ThirdIndex];
            return true;
        }
    }
}