#include "pch.h"
#include "Nest2D_EllipseClusterBuilder.h"
#include "Nest2D_ClusterGeometryHelper.h"
#include "NestUtils.h"
#include <algorithm>
#include <cmath>

namespace ET {
    namespace NEST2DMANAGERLIB {

        CetEllipseClusterBuilder::CetEllipseClusterBuilder() : CetCoreObject() {}
        CetEllipseClusterBuilder::~CetEllipseClusterBuilder() {}

        void CetEllipseClusterBuilder::BuildCandidates(const CetTNestItemVector& AItems,const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices,const TetNestOptions& AOptions, std::vector<TetClusterCandidate>& AOut)
        {
            if (AItems.size() != AFeatures.size()) return;
            CetClusterGeometryHelper Geometry;
            const double Gap = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));
            // 为了避免和传参 A 混淆，将 lambda 表达式参数改为 ValA 和 ValB
            auto Nearly = [](double ValA, double ValB) {
                return std::abs(ValA - ValB) <= std::max(1.0, std::max(std::abs(ValA), std::abs(ValB))) * 0.05;
                };

            for (std::size_t i = 0; i < AIndices.size(); ++i) {
                for (std::size_t j = i + 1; j < AIndices.size(); ++j) {
                    // 将原来的局部变量 A, B 改为 IndexA, IndexB 以提升可读性
                    const int IndexA = AIndices[i];
                    const int IndexB = AIndices[j];

                    if (IndexA < 0 || IndexB < 0 || IndexA >= static_cast<int>(AFeatures.size()) || IndexB >= static_cast<int>(AFeatures.size())) continue;

                    const auto& FA = AFeatures[IndexA];
                    const auto& FB = AFeatures[IndexB];

                    if (FA.ShapeType != MetShapeType::EllipseLike || FB.ShapeType != MetShapeType::EllipseLike) continue;
                    if (!Nearly(FA.EllipseMajorAxis, FB.EllipseMajorAxis) || !Nearly(FA.EllipseMinorAxis, FB.EllipseMinorAxis)) continue;

                    for (int Mode = 0; Mode < 2; ++Mode) {
                        TetClusterCandidate C;
                        C.BuilderName = "EllipseBuilder";
                        C.ClusterType = Mode == 0 ? "EllipsePairHorizontal" : "EllipsePairVertical";
                        C.OriginalIndices = { IndexA, IndexB };

                        TetItemTransform TA;
                        TA.OriginalId = IndexA;
                        TA.RelativeX = -FA.MinX;
                        TA.RelativeY = -FA.MinY;

                        TetItemTransform TB;
                        TB.OriginalId = IndexB;
                        TB.RelativeX = (Mode == 0 ? FA.Width + Gap : 0.0) - FB.MinX;
                        TB.RelativeY = (Mode == 1 ? FA.Height + Gap : 0.0) - FB.MinY;

                        C.Transforms = { TA, TB };
                        C.Confidence = 0.8;

                        if (Geometry.FinalizeCandidate(AItems, AOptions, C)) {
                            AOut.push_back(std::move(C));
                        }
                    }
                }
            }
        }

    }
}