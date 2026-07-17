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
        constexpr double CET_TRIANGLE_PI = 3.14159265358979323846;
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
            TransformB.RelativeRotation = CET_TRIANGLE_PI;
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
            if (AIndices.size() < 2) {
                return;
            }
            const std::size_t OldCandidateCount = AOutCandidates.size();
            for (std::size_t i = 0; i < AIndices.size(); ++i) {
                for (std::size_t j = i + 1; j < AIndices.size(); ++j) {
                    const int AIndex = AIndices[i];
                    const int BIndex = AIndices[j];
                    if (AIndex < 0 || BIndex < 0 ||AIndex >= static_cast<int>(AFeatures.size()) ||BIndex >= static_cast<int>(AFeatures.size())){
                        continue;
                    }

                    const TetShapeFeature& FeatureA = AFeatures[AIndex];
                    const TetShapeFeature& FeatureB = AFeatures[BIndex];
                    if (FeatureA.ShapeType != MetShapeType::TriangleLike ||FeatureB.ShapeType != MetShapeType::TriangleLike){
                        continue;
                    }

                    if (FeatureA.TriangleAngleType != MetTriangleAngleType::Right ||FeatureB.TriangleAngleType != MetTriangleAngleType::Right){
                        continue;
                    }

                    if (!_AreCongruentTriangles(FeatureA, FeatureB)) {
                        std::cout << "[TRIANGLE][REJECT] not congruent: "
                            << AIndex << " + " << BIndex << std::endl;
                        continue;
                    }
                    TetClusterCandidate Candidate;
                    if (!_BuildRightTrianglePairCandidate(AOriginalItems,AFeatures,AIndex,BIndex,AOptions,Candidate)){
                        std::cout << "[TRIANGLE][REJECT] build failed: "
                            << AIndex << " + " << BIndex << std::endl;
                        continue;
                    }
                    AOutCandidates.push_back(std::move(Candidate));
                    std::cout << "[TRIANGLE][CANDIDATE] "
                        << AIndex << " + " << BIndex
                        << " Type=RightTrianglePair"
                        << std::endl;
                }
            }
            std::cout << "[TRIANGLE][BUILD CANDIDATES] IndexCount="
                << AIndices.size()
                << ", NewCandidateCount="
                << AOutCandidates.size() - OldCandidateCount
                << std::endl;
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
            TB.RelativeRotation = CET_TRIANGLE_PI;
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
    }
}