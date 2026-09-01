#include "pch.h"
#include "Nest2D_TemplateCandidateGenerator.h"
#include "Nest2D_ArcClusterBuilder.h"
#include "Nest2D_CircleClusterBuilder.h"
#include "Nest2D_CustomClusterBuilder.h"
#include "Nest2D_EllipseClusterBuilder.h"
#include "Nest2D_RectangleClusterBuilder.h"
#include "Nest2D_TriangleClusterBuilder.h"

#include <iostream>

using namespace libnest2d;

namespace ET { namespace NEST2DMANAGERLIB {

CetTemplateCandidateGenerator::CetTemplateCandidateGenerator() : CetCoreObject() {}
CetTemplateCandidateGenerator::~CetTemplateCandidateGenerator() {}

void CetTemplateCandidateGenerator::CollectTemplateShapeIndices(const std::vector<TetShapeFeature> &AFeatures, std::map<MetShapeType, std::vector<int>> &AIndicesByType)
{
    const int Count = static_cast<int>(AFeatures.size());
    for (int i = 0; i < Count; ++i) {
        const TetShapeFeature &Feature = AFeatures[i];
        if (Feature.Width <= 0.0 || Feature.Height <= 0.0)
            continue;
        AIndicesByType[Feature.ShapeType].push_back(i);
    }
    std::cout << "[TEMPLATE][SHAPE COUNTS] Triangle=" << AIndicesByType[MetShapeType::TriangleLike].size() << " Circle=" << AIndicesByType[MetShapeType::CircleLike].size() << " Ellipse=" << AIndicesByType[MetShapeType::EllipseLike].size() << " Rectangle=" << AIndicesByType[MetShapeType::RectangleLike].size() << " Arc=" << AIndicesByType[MetShapeType::ArcLike].size() << " Convex=" << AIndicesByType[MetShapeType::ConvexPolygon].size() << " Concave=" << AIndicesByType[MetShapeType::ConcavePolygon].size() << std::endl;
}

void CetTemplateCandidateGenerator::BuildTemplateCandidates(const CetTNestItemVector &AOriginalItems, const std::vector<TetShapeFeature> &AFeatures, const TetNestOptions &AOptions, const std::map<MetShapeType, std::vector<int>> &AIndicesByType, std::vector<TetClusterCandidate> &ABaseCandidates)
{
    auto AppendBuilderLog = [&](const char *ABuilderName, std::size_t AOldCount) {
        std::cout << "[TEMPLATE][BUILDER] " << ABuilderName << " NewCandidates=" << ABaseCandidates.size() - AOldCount << std::endl;
    };
    {
        const std::size_t OldCount = ABaseCandidates.size();
        CetTriangleClusterBuilder Builder;
        Builder.BuildCandidates(AOriginalItems, AFeatures, AIndicesByType.at(MetShapeType::TriangleLike), AOptions, ABaseCandidates);
        AppendBuilderLog("TriangleBuilder", OldCount);
    }
    {
        const std::size_t OldCount = ABaseCandidates.size();
        CetCircleClusterBuilder Builder;
        Builder.BuildCandidates(AOriginalItems, AFeatures, AIndicesByType.at(MetShapeType::CircleLike), AOptions, ABaseCandidates);
        AppendBuilderLog("CircleBuilder", OldCount);
    }
    {
        const std::size_t OldCount = ABaseCandidates.size();
        CetEllipseClusterBuilder Builder;
        Builder.BuildCandidates(AOriginalItems, AFeatures, AIndicesByType.at(MetShapeType::EllipseLike), AOptions, ABaseCandidates);
        AppendBuilderLog("EllipseBuilder", OldCount);
    }
    {
        const std::size_t OldCount = ABaseCandidates.size();
        CetRectangleClusterBuilder Builder;
        Builder.BuildCandidates(AOriginalItems, AFeatures, AIndicesByType.at(MetShapeType::RectangleLike), AOptions, ABaseCandidates);
        AppendBuilderLog("RectangleBuilder", OldCount);
    }
    {
        const std::size_t OldCount = ABaseCandidates.size();
        CetArcClusterBuilder Builder;
        Builder.BuildCandidates(AOriginalItems, AFeatures, AIndicesByType.at(MetShapeType::ArcLike), AOptions, ABaseCandidates);
        AppendBuilderLog("ArcBuilder", OldCount);
    }
    {
        std::vector<int> CustomIndices;
        const auto AppendCustomIndices = [&](MetShapeType AShapeType) {
            auto It = AIndicesByType.find(AShapeType);
            if (It != AIndicesByType.end()) {
                const std::vector<int> &TypeIndices = It->second;
                CustomIndices.insert(CustomIndices.end(), TypeIndices.begin(), TypeIndices.end());
            }
        };
        AppendCustomIndices(MetShapeType::QuadrilateralLike);
        AppendCustomIndices(MetShapeType::ConvexPolygon);
        AppendCustomIndices(MetShapeType::ConcavePolygon);
        const std::size_t OldCount = ABaseCandidates.size();
        CetCustomClusterBuilder Builder;
        Builder.BuildCandidates(AOriginalItems, AFeatures, CustomIndices, AOptions, ABaseCandidates);
        AppendBuilderLog("CustomBuilder", OldCount);
    }
}

}}
