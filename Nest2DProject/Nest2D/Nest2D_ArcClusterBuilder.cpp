#include "pch.h"
#include "Nest2D_ArcClusterBuilder.h"
#include "Nest2D_ClusterGeometryHelper.h"
#include "NestUtils.h"
#include <algorithm>
#include <cmath>

namespace ET {
    namespace NEST2DMANAGERLIB {

        namespace { constexpr double PI = 3.14159265358979323846; }

        CetArcClusterBuilder::CetArcClusterBuilder() : CetCoreObject() {}
        CetArcClusterBuilder::~CetArcClusterBuilder() {}

        void CetArcClusterBuilder::BuildCandidates(const CetTNestItemVector& AItems,const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices,const TetNestOptions& AOptions, std::vector<TetClusterCandidate>& AOut) 
        {

            if (AItems.size() != AFeatures.size()) return;

            CetClusterGeometryHelper Geometry;
            const double Gap = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));

            // lambda 表达式参数去除单字母
            auto Nearly = [](double ValA, double ValB, double Tolerance) {
                return std::abs(ValA - ValB) <= std::max(1.0, std::max(std::abs(ValA), std::abs(ValB))) * Tolerance;
                };

            for (std::size_t i = 0; i < AIndices.size(); ++i) {
                for (std::size_t j = i + 1; j < AIndices.size(); ++j) {

                    // A, B 改为 IndexA, IndexB
                    const int IndexA = AIndices[i];
                    const int IndexB = AIndices[j];

                    if (IndexA < 0 || IndexB < 0 || IndexA >= static_cast<int>(AFeatures.size()) || IndexB >= static_cast<int>(AFeatures.size())) continue;

                    const auto& FeatureA = AFeatures[IndexA];
                    const auto& FeatureB = AFeatures[IndexB];

                    if (FeatureA.ArcType != MetArcType::SemiCircleLike || FeatureB.ArcType != MetArcType::SemiCircleLike) continue;
                    if (!Nearly(FeatureA.ArcRadius, FeatureB.ArcRadius, 0.03) || !Nearly(FeatureA.ArcChordLength, FeatureB.ArcChordLength, 0.03)) continue;

                    // C 改为 Candidate
                    TetClusterCandidate Candidate;
                    Candidate.BuilderName = "ArcBuilder";
                    Candidate.ClusterType = "SemiCirclePair";
                    Candidate.OriginalIndices = { IndexA, IndexB };

                    // TA, TB 改为 TransformA, TransformB
                    TetItemTransform TransformA;
                    TransformA.OriginalId = IndexA;
                    TransformA.RelativeRotation = -FeatureA.ArcChordAngle;

                    TetItemTransform TransformB;
                    TransformB.OriginalId = IndexB;
                    TransformB.RelativeRotation = PI - FeatureB.ArcChordAngle;

                    // 先将两个半圆旋转到水平弦，再把第二个放到第一件右侧；FinalizeCandidate 负责归一化。
                    CetPath PathA = Geometry.TransformContour(Geometry.GetIdentityContour(AItems[IndexA]), TransformA.RelativeRotation, 0, 0);
                    double PathAMinX, PathAMinY, PathAMaxX, PathAMaxY;
                    if (!Geometry.GetBounds(PathA, PathAMinX, PathAMinY, PathAMaxX, PathAMaxY)) continue;

                    CetPath PathB = Geometry.TransformContour(Geometry.GetIdentityContour(AItems[IndexB]), TransformB.RelativeRotation, 0, 0);
                    double PathBMinX, PathBMinY, PathBMaxX, PathBMaxY;
                    if (!Geometry.GetBounds(PathB, PathBMinX, PathBMinY, PathBMaxX, PathBMaxY)) continue;

                    TransformA.RelativeX = -PathAMinX;
                    TransformA.RelativeY = -PathAMinY;

                    TransformB.RelativeX = -PathBMinX;
                    TransformB.RelativeY = (PathAMaxY - PathAMinY) + Gap - PathBMinY;

                    Candidate.Transforms = { TransformA, TransformB };
                    Candidate.Confidence = 0.7;

                    if (Geometry.FinalizeCandidate(AItems, AOptions, Candidate)) {
                        AOut.push_back(std::move(Candidate));
                    }
                }
            }
        }

    }
}