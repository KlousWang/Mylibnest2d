#include "pch.h"
#include "Nest2D_CircleClusterBuilder.h"
#include "Nest2D_ClusterGeometryHelper.h"
#include "NestUtils.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>

namespace ET {
    namespace NEST2DMANAGERLIB {

        CetCircleClusterBuilder::CetCircleClusterBuilder() : CetCoreObject() {}

        CetCircleClusterBuilder::~CetCircleClusterBuilder() {}

        void CetCircleClusterBuilder::BuildCandidates(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, std::vector<TetClusterCandidate>& AOutCandidates)
        {
            if (AOriginalItems.empty()) { return; }

            if (AFeatures.size() != AOriginalItems.size()) {
                std::cout << "[CIRCLE][ERROR] Feature count mismatch. OriginalItems = " << AOriginalItems.size() << ", Features = " << AFeatures.size() << std::endl;
                return;
            }

            if (AIndices.size() < 2) { return; }

            std::vector<int> ValidIndices;
            ValidIndices.reserve(AIndices.size());

            for (int Index : AIndices) {
                if (Index < 0 || Index >= static_cast<int>(AOriginalItems.size())) { continue; }
                const TetShapeFeature& Feature = AFeatures[Index];
                if (Feature.ShapeType != MetShapeType::CircleLike) { continue; }
                if (Feature.Width <= 0.0 || Feature.Height <= 0.0 || Feature.Area <= 0.0) { continue; }
                ValidIndices.push_back(Index);
            }

            std::sort(ValidIndices.begin(), ValidIndices.end());
            ValidIndices.erase(std::unique(ValidIndices.begin(), ValidIndices.end()), ValidIndices.end());

            if (ValidIndices.size() < 2) { return; }

            const std::size_t OldCandidateCount = AOutCandidates.size();

            _BuildBlock4Candidates(AOriginalItems, AFeatures, ValidIndices, AOptions, AOutCandidates);
            _BuildPairCandidates(AOriginalItems, AFeatures, ValidIndices, AOptions, AOutCandidates);

            std::cout << "[CIRCLE][BUILD CANDIDATES] IndexCount = " << ValidIndices.size() << ", NewCandidateCount = " << AOutCandidates.size() - OldCandidateCount << std::endl;
        }

        void CetCircleClusterBuilder::_BuildPairCandidates(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, std::vector<TetClusterCandidate>& AOutCandidates)
        {
            if (AFeatures.size() != AOriginalItems.size() || AIndices.size() < 2) { return; }

            for (std::size_t i = 0; i < AIndices.size(); ++i) {
                for (std::size_t j = i + 1; j < AIndices.size(); ++j) {
                    const int IndexA = AIndices[i];
                    const int IndexB = AIndices[j];

                    if (IndexA < 0 || IndexB < 0 || IndexA >= static_cast<int>(AOriginalItems.size()) || IndexB >= static_cast<int>(AOriginalItems.size()) || IndexA == IndexB) { continue; }

                    TetClusterCandidate Candidate;
                    if (!_BuildPairCandidate(AOriginalItems, AFeatures, IndexA, IndexB, AOptions, Candidate)) {
                        continue;
                    }

                    AOutCandidates.push_back(std::move(Candidate));
                    std::cout << "[CIRCLE][PAIR CANDIDATE] " << IndexA << " + " << IndexB << std::endl;
                }
            }
        }

        void CetCircleClusterBuilder::_BuildBlock4Candidates(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, std::vector<TetClusterCandidate>& AOutCandidates)
        {
            if (AFeatures.size() != AOriginalItems.size() || AIndices.size() < 4) { return; }

            for (std::size_t i = 0; i + 3 < AIndices.size(); i += 4) {
                const int Index0 = AIndices[i];
                const int Index1 = AIndices[i + 1];
                const int Index2 = AIndices[i + 2];
                const int Index3 = AIndices[i + 3];

                if (Index0 < 0 || Index1 < 0 || Index2 < 0 || Index3 < 0 || Index0 >= static_cast<int>(AOriginalItems.size()) || Index1 >= static_cast<int>(AOriginalItems.size()) || Index2 >= static_cast<int>(AOriginalItems.size()) || Index3 >= static_cast<int>(AOriginalItems.size())) { continue; }

                TetClusterCandidate Candidate;
                if (!_BuildBlock4Candidate(AOriginalItems,AFeatures,Index0,Index1,Index2,Index3,AOptions,Candidate)){
                    continue;
                }

                AOutCandidates.push_back(std::move(Candidate));
                std::cout << "[CIRCLE][BLOCK4 CANDIDATE] " << Index0 << ", " << Index1 << ", " << Index2 << ", " << Index3 << std::endl;
            }
        }

        bool CetCircleClusterBuilder::_BuildPairCandidate(
            const CetTNestItemVector& AOriginalItems,
            const std::vector<TetShapeFeature>& AFeatures,
            int AIndexA,
            int AIndexB,
            const TetNestOptions& AOptions,
            TetClusterCandidate& AOutCandidate)
        {
            AOutCandidate = TetClusterCandidate{};

            if (AIndexA < 0 || AIndexB < 0 ||
                AIndexA >= static_cast<int>(AFeatures.size()) ||
                AIndexB >= static_cast<int>(AFeatures.size()) ||
                AIndexA >= static_cast<int>(AOriginalItems.size()) ||
                AIndexB >= static_cast<int>(AOriginalItems.size()) ||
                AIndexA == AIndexB)
            {
                return false;
            }

            const TetShapeFeature& FeatureA = AFeatures[AIndexA];
            const TetShapeFeature& FeatureB = AFeatures[AIndexB];

            if (FeatureA.ShapeType != MetShapeType::CircleLike ||
                FeatureB.ShapeType != MetShapeType::CircleLike)
            {
                return false;
            }

            if (FeatureA.Width <= 0.0 || FeatureA.Height <= 0.0 ||
                FeatureB.Width <= 0.0 || FeatureB.Height <= 0.0 ||
                FeatureA.Area <= 0.0 || FeatureB.Area <= 0.0)
            {
                return false;
            }

            auto NearlyEqual = [](double ValA, double ValB, double RelativeTolerance) {
                const double Denominator =
                    std::max(1.0, std::max(std::abs(ValA), std::abs(ValB)));

                return std::abs(ValA - ValB) <=
                    Denominator * RelativeTolerance;
                };

            constexpr double SizeTolerance = 0.03;

            if (!NearlyEqual(FeatureA.Width, FeatureB.Width, SizeTolerance) ||
                !NearlyEqual(FeatureA.Height, FeatureB.Height, SizeTolerance))
            {
                return false;
            }

            const double CellWidth =
                std::max(FeatureA.Width, FeatureB.Width);

            const double CellHeight =
                std::max(FeatureA.Height, FeatureB.Height);

            const double Gap =
                std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));

            const double ClusterWidth =
                CellWidth * 2.0 + Gap;

            const double ClusterHeight =
                CellHeight;

            if (!_FitsBin(ClusterWidth, ClusterHeight, AOptions)) {
                return false;
            }

            TetItemTransform TransformA;
            TransformA.OriginalId = AIndexA;
            TransformA.RelativeX = 0.0 - FeatureA.MinX;
            TransformA.RelativeY = 0.0 - FeatureA.MinY;
            TransformA.RelativeRotation = 0.0;

            TetItemTransform TransformB;
            TransformB.OriginalId = AIndexB;
            TransformB.RelativeX = CellWidth + Gap - FeatureB.MinX;
            TransformB.RelativeY = 0.0 - FeatureB.MinY;
            TransformB.RelativeRotation = 0.0;

            AOutCandidate.BuilderName = "CircleBuilder";
            AOutCandidate.ClusterType = "CirclePair2";
            AOutCandidate.OriginalIndices = { AIndexA, AIndexB };
            AOutCandidate.Transforms = { TransformA, TransformB };
            AOutCandidate.Confidence = 1.0;

            CetClusterGeometryHelper Geometry;

            if (!Geometry.FinalizeCandidate(
                AOriginalItems,
                AOptions,
                AOutCandidate))
            {
                return false;
            }

            return true;
        }

        bool CetCircleClusterBuilder::_BuildBlock4Candidate(const CetTNestItemVector& AOriginalItems,const std::vector<TetShapeFeature>& AFeatures,int AIndex0,int AIndex1,int AIndex2,int AIndex3,const TetNestOptions& AOptions,TetClusterCandidate& AOutCandidate)
        {
            AOutCandidate = TetClusterCandidate{};
            const std::array<int, 4> Indices = { AIndex0, AIndex1, AIndex2, AIndex3 };

            for (std::size_t i = 0; i < Indices.size(); ++i) {
                if (Indices[i] < 0 || Indices[i] >= static_cast<int>(AFeatures.size())) { return false; }
                for (std::size_t j = i + 1; j < Indices.size(); ++j) {
                    if (Indices[i] == Indices[j]) { return false; }
                }
            }

            const TetShapeFeature& Feature0 = AFeatures[AIndex0];
            const TetShapeFeature& Feature1 = AFeatures[AIndex1];
            const TetShapeFeature& Feature2 = AFeatures[AIndex2];
            const TetShapeFeature& Feature3 = AFeatures[AIndex3];

            const std::array<const TetShapeFeature*, 4> Features = { &Feature0, &Feature1, &Feature2, &Feature3 };

            for (const TetShapeFeature* Feature : Features) {
                if (Feature == nullptr || Feature->ShapeType != MetShapeType::CircleLike || Feature->Width <= 0.0 || Feature->Height <= 0.0 || Feature->Area <= 0.0) { return false; }
            }

            // lambda 表达式参数改为 ValA 和 ValB
            auto NearlyEqual = [](double ValA, double ValB, double RelativeTolerance) {
                const double Denominator = std::max(1.0, std::max(std::abs(ValA), std::abs(ValB)));
                return std::abs(ValA - ValB) <= Denominator * RelativeTolerance;
                };

            constexpr double SizeTolerance = 0.03;
            for (std::size_t i = 1; i < Features.size(); ++i) {
                if (!NearlyEqual(Feature0.Width, Features[i]->Width, SizeTolerance) || !NearlyEqual(Feature0.Height, Features[i]->Height, SizeTolerance)) { return false; }
            }

            double CellWidth = 0.0;
            double CellHeight = 0.0;
            double RealArea = 0.0;

            for (const TetShapeFeature* Feature : Features) {
                CellWidth = std::max(CellWidth, Feature->Width);
                CellHeight = std::max(CellHeight, Feature->Height);
                RealArea += Feature->Area;
            }

            const double Gap = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));
            const double ClusterWidth = CellWidth * 2.0 + Gap;
            const double ClusterHeight = CellHeight * 2.0 + Gap;

            if (!_FitsBin(ClusterWidth, ClusterHeight, AOptions)) { return false; }

            const std::array<double, 4> TargetMinX = { 0.0, CellWidth + Gap, 0.0, CellWidth + Gap };
            const std::array<double, 4> TargetMinY = { 0.0, 0.0, CellHeight + Gap, CellHeight + Gap };

            AOutCandidate.Transforms.clear();
            AOutCandidate.Transforms.reserve(4);
            AOutCandidate.OriginalIndices.clear();
            AOutCandidate.OriginalIndices.reserve(4);

            for (std::size_t i = 0; i < Indices.size(); ++i) {
                TetItemTransform Transform;
                Transform.OriginalId = Indices[i];
                Transform.RelativeX = TargetMinX[i] - Features[i]->MinX;
                Transform.RelativeY = TargetMinY[i] - Features[i]->MinY;
                Transform.RelativeRotation = 0.0;
                AOutCandidate.Transforms.push_back(Transform);
                AOutCandidate.OriginalIndices.push_back(Indices[i]);
            }

            AOutCandidate.BuilderName = "CircleBuilder";
            AOutCandidate.ClusterType = "CircleBlock4";
            AOutCandidate.Confidence = 1.0;

            CetClusterGeometryHelper Geometry;

            if (!Geometry.FinalizeCandidate(
                AOriginalItems,
                AOptions,
                AOutCandidate))
            {
                return false;
            }

            return true;
        }

        bool CetCircleClusterBuilder::_FitsBin(double AClusterWidth, double AClusterHeight, const TetNestOptions& AOptions)
        {
            if (AClusterWidth <= 0.0 || AClusterHeight <= 0.0) { return false; }
            const double BinWidth = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth));
            const double BinHeight = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinHeight));
            if (BinWidth <= 0.0 || BinHeight <= 0.0) { return false; }
            const bool FitsNormally = AClusterWidth <= BinWidth && AClusterHeight <= BinHeight;
            const bool FitsAfterRotation = AOptions.Rotations > 1 && AClusterHeight <= BinWidth && AClusterWidth <= BinHeight;
            return FitsNormally || FitsAfterRotation;
        }

        double CetCircleClusterBuilder::_CalculateScore(const TetClusterCandidate& ACandidate)
        {
            if (!ACandidate.Valid || ACandidate.OriginalIndices.empty() || ACandidate.ClusterWidth <= 0.0 || ACandidate.ClusterHeight <= 0.0 || ACandidate.ProxyArea <= 0.0) { return -std::numeric_limits<double>::infinity(); }

            const double FillScore = ACandidate.FillRatio * 1000.0;
            const double ItemCountScore = static_cast<double>(ACandidate.OriginalIndices.size()) * 50.0;

            const double LongSide = std::max(ACandidate.ClusterWidth, ACandidate.ClusterHeight);
            const double ShortSide = std::min(ACandidate.ClusterWidth, ACandidate.ClusterHeight);
            const double CompactRatio = LongSide > 0.0 ? ShortSide / LongSide : 0.0;
            const double CompactScore = CompactRatio * 20.0;

            const double PerimeterPenalty = (ACandidate.ClusterWidth + ACandidate.ClusterHeight) * 0.000001;

            return FillScore + ItemCountScore + CompactScore - PerimeterPenalty;
        }

    }
}