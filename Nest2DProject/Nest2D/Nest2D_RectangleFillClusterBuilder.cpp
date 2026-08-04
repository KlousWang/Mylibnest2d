#include "pch.h"
#include "Nest2D_RectangleFillClusterBuilder.h"
#include "Nest2D_ClusterGeometryHelper.h"
#include "Nest2D_RotationUtils.h"
#include "NestUtils.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <utility>

namespace ET {
    namespace NEST2DMANAGERLIB {

        CetRectangleFillClusterBuilder::CetRectangleFillClusterBuilder() : CetCoreObject() {}
        CetRectangleFillClusterBuilder::~CetRectangleFillClusterBuilder() {}

        bool CetRectangleFillClusterBuilder::BuildCandidateForBase(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ABaseCandidate, const TetNestOptions& AOptions, const std::vector<bool>& AUsed, TetClusterCandidate& AOutCandidate)
        {
            AOutCandidate = TetClusterCandidate{};
            if (AOriginalItems.empty() || AFeatures.size() != AOriginalItems.size() || AUsed.size() != AOriginalItems.size()){
                return false;
            }
            if (!ABaseCandidate.Valid || ABaseCandidate.OriginalIndices.size() < 2 || ABaseCandidate.OriginalIndices.size() != ABaseCandidate.Transforms.size()){
                return false;
            }

            CetClusterGeometryHelper Geometry;
            TetClusterCandidate CurrentCandidate = ABaseCandidate;
            const double EnvelopeWidth = ABaseCandidate.ClusterWidth;
            const double EnvelopeHeight = ABaseCandidate.ClusterHeight;
            if (EnvelopeWidth <= 0.0 || EnvelopeHeight <= 0.0 || !Geometry.FinalizeCandidateInRectangle(AOriginalItems, AOptions, CurrentCandidate, EnvelopeWidth, EnvelopeHeight)){
                return false;
            }

            CurrentCandidate.BuilderName = "RectangleFillBuilder";
            CurrentCandidate.ClusterType += "_RectFill";

            std::vector<int> FillerIndices;
            FillerIndices.reserve(AOriginalItems.size());
            for (int Index = 0; Index < static_cast<int>(AOriginalItems.size()); ++Index){
                if (AUsed[Index] || _ContainsOriginalIndex(CurrentCandidate, Index)){
                    continue;
                }
                if (AFeatures[Index].Area <= 0.0 || AFeatures[Index].Width <= 0.0 || AFeatures[Index].Height <= 0.0){
                    continue;
                }
                FillerIndices.push_back(Index);
            }

            std::stable_sort(FillerIndices.begin(), FillerIndices.end(), [&](int AFirstIndex, int ASecondIndex) {
                const double FirstArea = _GetFeatureArea(AOriginalItems[AFirstIndex], AFeatures[AFirstIndex]);
                const double SecondArea = _GetFeatureArea(AOriginalItems[ASecondIndex], AFeatures[ASecondIndex]);
                if (std::abs(FirstArea - SecondArea) > 1.0){
                    return FirstArea > SecondArea;
                }
                return AFirstIndex < ASecondIndex;
            });

            bool AddedFiller = true;
            while (AddedFiller){
                AddedFiller = false;
                for (int FillerIndex : FillerIndices){
                    if (AUsed[FillerIndex] || _ContainsOriginalIndex(CurrentCandidate, FillerIndex)){
                        continue;
                    }
                    const double FillerArea = _GetFeatureArea(AOriginalItems[FillerIndex], AFeatures[FillerIndex]);
                    const double AreaTolerance = std::max(1.0, CurrentCandidate.ProxyArea * 1e-9);
                    if (FillerArea > CurrentCandidate.ProxyWasteArea + AreaTolerance){
                        continue;
                    }

                    TetClusterCandidate Candidate;
                    if (!_TryAddFiller(AOriginalItems, AFeatures, CurrentCandidate, FillerIndex, AOptions, EnvelopeWidth, EnvelopeHeight, Candidate)){
                        continue;
                    }

                    CurrentCandidate = std::move(Candidate);
                    AddedFiller = true;
                    std::cout << "[RECT_FILL][ACCEPT] OriginalId=" << FillerIndex << ", ChildCount=" << CurrentCandidate.OriginalIndices.size() << ", Width=" << CurrentCandidate.ClusterWidth << ", Height=" << CurrentCandidate.ClusterHeight << std::endl;
                    break;
                }
            }

            AOutCandidate = std::move(CurrentCandidate);
            return true;
        }

        bool CetRectangleFillClusterBuilder::_TryAddFiller(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ACurrentCandidate, int AFillerIndex, const TetNestOptions& AOptions, double AEnvelopeWidth, double AEnvelopeHeight, TetClusterCandidate& AOutCandidate)
        {
            AOutCandidate = TetClusterCandidate{};
            if (AFillerIndex < 0 || AFillerIndex >= static_cast<int>(AOriginalItems.size()) || AFillerIndex >= static_cast<int>(AFeatures.size()) || _ContainsOriginalIndex(ACurrentCandidate, AFillerIndex)){
                return false;
            }

            CetClusterGeometryHelper Geometry;
            const double RequiredGap = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));
            const std::vector<double> Rotations = CetRotationUtils::BuildAllowedRotations(AOptions.Rotations);
            if (Rotations.empty()){
                return false;
            }

            bool FoundPlacement = false;
            double BestPlacementScore = -std::numeric_limits<double>::infinity();
            TetItemTransform BestTransform;
            for (double Rotation : Rotations){
                const CetPath RotatedFiller = Geometry.TransformContour(Geometry.GetIdentityContour(AOriginalItems[AFillerIndex]), Rotation, 0.0, 0.0);
                double FillerMinX = 0.0;
                double FillerMinY = 0.0;
                double FillerMaxX = 0.0;
                double FillerMaxY = 0.0;
                if (!Geometry.GetBounds(RotatedFiller, FillerMinX, FillerMinY, FillerMaxX, FillerMaxY)){
                    continue;
                }

                const double FillerWidth = FillerMaxX - FillerMinX;
                const double FillerHeight = FillerMaxY - FillerMinY;
                if (FillerWidth <= 0.0 || FillerHeight <= 0.0 || FillerWidth > AEnvelopeWidth + CET_RECTANGLE_FILL_POSITION_TOLERANCE || FillerHeight > AEnvelopeHeight + CET_RECTANGLE_FILL_POSITION_TOLERANCE){
                    continue;
                }

                std::vector<std::pair<double, double>> ProbePositions;
                _BuildProbePositions(AOriginalItems, ACurrentCandidate, RotatedFiller, FillerMinX, FillerMinY, FillerMaxX, FillerMaxY, RequiredGap, ProbePositions);
                const double MaxX = AEnvelopeWidth - FillerWidth;
                const double MaxY = AEnvelopeHeight - FillerHeight;
                for (const auto& ProbePosition : ProbePositions){
                    const double PositionX = ProbePosition.first;
                    const double PositionY = ProbePosition.second;
                    if (PositionX < -CET_RECTANGLE_FILL_POSITION_TOLERANCE || PositionY < -CET_RECTANGLE_FILL_POSITION_TOLERANCE || PositionX > MaxX + CET_RECTANGLE_FILL_POSITION_TOLERANCE || PositionY > MaxY + CET_RECTANGLE_FILL_POSITION_TOLERANCE){
                        continue;
                    }

                    TetItemTransform FillerTransform;
                    FillerTransform.OriginalId = AFillerIndex;
                    FillerTransform.RelativeRotation = Rotation;
                    FillerTransform.RelativeX = PositionX - FillerMinX;
                    FillerTransform.RelativeY = PositionY - FillerMinY;

                    if (!Geometry.CanAppendTransformWithSpacing(AOriginalItems, AOptions, ACurrentCandidate.Transforms, FillerTransform)){
                        continue;
                    }

                    const double PlacementScore = _CalculatePlacementScore(AOriginalItems, ACurrentCandidate, PositionX, PositionY, PositionX + FillerWidth, PositionY + FillerHeight, RequiredGap);
                    if (!FoundPlacement || PlacementScore > BestPlacementScore + 1e-9){
                        FoundPlacement = true;
                        BestPlacementScore = PlacementScore;
                        BestTransform = FillerTransform;
                    }
                }
            }

            if (!FoundPlacement){
                return false;
            }
            TetClusterCandidate BestCandidate = ACurrentCandidate;
            BestCandidate.OriginalIndices.push_back(AFillerIndex);
            BestCandidate.Transforms.push_back(BestTransform);
            if (!Geometry.FinalizeCandidateInRectangle(AOriginalItems, AOptions, BestCandidate, AEnvelopeWidth, AEnvelopeHeight)){
                return false;
            }
            BestCandidate.BuilderName = "RectangleFillBuilder";
            AOutCandidate = std::move(BestCandidate);
            return true;
        }

        void CetRectangleFillClusterBuilder::_BuildProbePositions(const CetTNestItemVector& AOriginalItems, const TetClusterCandidate& ACandidate, const CetPath& ARotatedFiller, double AFillerMinX, double AFillerMinY, double AFillerMaxX, double AFillerMaxY, double ARequiredGap, std::vector<std::pair<double, double>>& AOutPositions)
        {
            AOutPositions.clear();
            const double AFillerWidth = AFillerMaxX - AFillerMinX;
            const double AFillerHeight = AFillerMaxY - AFillerMinY;
            const double MaxX = ACandidate.ClusterWidth - AFillerWidth;
            const double MaxY = ACandidate.ClusterHeight - AFillerHeight;
            if (MaxX < 0.0 || MaxY < 0.0){
                return;
            }

            auto AddPosition = [&](double AX, double AY) {
                _AppendProbePosition(AOutPositions, AX, AY, MaxX, MaxY);
            };

            AddPosition(0.0, 0.0);
            AddPosition(MaxX, 0.0);
            AddPosition(0.0, MaxY);
            AddPosition(MaxX, MaxY);
            AddPosition(MaxX * 0.5, MaxY * 0.5);

            std::vector<double> XEvents{ 0.0, MaxX };
            std::vector<double> YEvents{ 0.0, MaxY };
            auto AddAxisEvent = [](std::vector<double>& AEvents, double AValue, double AMaximum) {
                if (!std::isfinite(AValue) || AValue < -CET_RECTANGLE_FILL_POSITION_TOLERANCE || AValue > AMaximum + CET_RECTANGLE_FILL_POSITION_TOLERANCE){
                    return;
                }
                const double Quantized = std::llround(std::clamp(AValue, 0.0, AMaximum));
                for (double Existing : AEvents){
                    if (std::abs(Existing - Quantized) <= CET_RECTANGLE_FILL_POSITION_TOLERANCE){
                        return;
                    }
                }
                if (AEvents.size() < CET_RECTANGLE_FILL_MAX_AXIS_COORDINATES){
                    AEvents.push_back(Quantized);
                }
            };

            CetClusterGeometryHelper Geometry;
            for (const TetItemTransform& Transform : ACandidate.Transforms){
                if (Transform.OriginalId < 0 || Transform.OriginalId >= static_cast<int>(AOriginalItems.size())){
                    continue;
                }
                const CetPath Child = Geometry.TransformContour(Geometry.GetIdentityContour(AOriginalItems[Transform.OriginalId]), Transform.RelativeRotation, Transform.RelativeX, Transform.RelativeY);
                double MinX = 0.0;
                double MinY = 0.0;
                double ChildMaxX = 0.0;
                double ChildMaxY = 0.0;
                if (!Geometry.GetBounds(Child, MinX, MinY, ChildMaxX, ChildMaxY)){
                    continue;
                }
                AddPosition(ChildMaxX + ARequiredGap, MinY);
                AddPosition(MinX - AFillerWidth - ARequiredGap, MinY);
                AddPosition(MinX, ChildMaxY + ARequiredGap);
                AddPosition(MinX, MinY - AFillerHeight - ARequiredGap);
                AddPosition(ChildMaxX + ARequiredGap, ChildMaxY + ARequiredGap);

                AddAxisEvent(XEvents, ChildMaxX + ARequiredGap, MaxX);
                AddAxisEvent(XEvents, MinX - AFillerWidth - ARequiredGap, MaxX);
                AddAxisEvent(YEvents, ChildMaxY + ARequiredGap, MaxY);
                AddAxisEvent(YEvents, MinY - AFillerHeight - ARequiredGap, MaxY);

                for (const ClipperLib::IntPoint& ChildVertex : Child){
                    for (const ClipperLib::IntPoint& FillerVertex : ARotatedFiller){
                        AddAxisEvent(XEvents, static_cast<double>(ChildVertex.X - FillerVertex.X) + AFillerMinX + ARequiredGap, MaxX);
                        AddAxisEvent(XEvents, static_cast<double>(ChildVertex.X - FillerVertex.X) + AFillerMinX - ARequiredGap, MaxX);
                        AddAxisEvent(YEvents, static_cast<double>(ChildVertex.Y - FillerVertex.Y) + AFillerMinY + ARequiredGap, MaxY);
                        AddAxisEvent(YEvents, static_cast<double>(ChildVertex.Y - FillerVertex.Y) + AFillerMinY - ARequiredGap, MaxY);
                    }
                }
            }

            std::sort(XEvents.begin(), XEvents.end());
            std::sort(YEvents.begin(), YEvents.end());
            for (double Y : YEvents){
                for (double X : XEvents){
                    AddPosition(X, Y);
                    if (AOutPositions.size() >= CET_RECTANGLE_FILL_MAX_PROBE_COUNT){
                        return;
                    }
                }
            }

            for (int Row = 0; Row < CET_RECTANGLE_FILL_GRID_PROBE_COUNT; ++Row){
                const double YRatio = static_cast<double>(Row) / static_cast<double>(CET_RECTANGLE_FILL_GRID_PROBE_COUNT - 1);
                for (int Column = 0; Column < CET_RECTANGLE_FILL_GRID_PROBE_COUNT; ++Column){
                    const double XRatio = static_cast<double>(Column) / static_cast<double>(CET_RECTANGLE_FILL_GRID_PROBE_COUNT - 1);
                    AddPosition(MaxX * XRatio, MaxY * YRatio);
                    if (AOutPositions.size() >= CET_RECTANGLE_FILL_MAX_PROBE_COUNT){
                        return;
                    }
                }
            }
        }

        double CetRectangleFillClusterBuilder::_CalculatePlacementScore(const CetTNestItemVector& AOriginalItems, const TetClusterCandidate& ACandidate, double AFillerLeft, double AFillerTop, double AFillerRight, double AFillerBottom, double ARequiredGap) const
        {
            const double ContactTolerance = ARequiredGap + CET_RECTANGLE_FILL_POSITION_TOLERANCE;
            double Score = -(AFillerTop + AFillerLeft) / std::max(1.0, ACandidate.ClusterWidth + ACandidate.ClusterHeight);
            if (AFillerLeft <= CET_RECTANGLE_FILL_POSITION_TOLERANCE || ACandidate.ClusterWidth - AFillerRight <= CET_RECTANGLE_FILL_POSITION_TOLERANCE){
                Score += 1.0;
            }
            if (AFillerTop <= CET_RECTANGLE_FILL_POSITION_TOLERANCE || ACandidate.ClusterHeight - AFillerBottom <= CET_RECTANGLE_FILL_POSITION_TOLERANCE){
                Score += 1.0;
            }

            CetClusterGeometryHelper Geometry;
            for (const TetItemTransform& Transform : ACandidate.Transforms){
                if (Transform.OriginalId < 0 || Transform.OriginalId >= static_cast<int>(AOriginalItems.size())){
                    continue;
                }
                const CetPath Child = Geometry.TransformContour(Geometry.GetIdentityContour(AOriginalItems[Transform.OriginalId]), Transform.RelativeRotation, Transform.RelativeX, Transform.RelativeY);
                double MinX = 0.0;
                double MinY = 0.0;
                double MaxX = 0.0;
                double MaxY = 0.0;
                if (!Geometry.GetBounds(Child, MinX, MinY, MaxX, MaxY)){
                    continue;
                }
                const bool VerticalOverlap = AFillerTop <= MaxY + CET_RECTANGLE_FILL_POSITION_TOLERANCE && AFillerBottom >= MinY - CET_RECTANGLE_FILL_POSITION_TOLERANCE;
                const bool HorizontalOverlap = AFillerLeft <= MaxX + CET_RECTANGLE_FILL_POSITION_TOLERANCE && AFillerRight >= MinX - CET_RECTANGLE_FILL_POSITION_TOLERANCE;
                const double HorizontalGap = std::max({ MinX - AFillerRight, AFillerLeft - MaxX, 0.0 });
                const double VerticalGap = std::max({ MinY - AFillerBottom, AFillerTop - MaxY, 0.0 });
                if (VerticalOverlap && HorizontalGap <= ContactTolerance){
                    Score += 2.0;
                }
                if (HorizontalOverlap && VerticalGap <= ContactTolerance){
                    Score += 2.0;
                }
            }
            return Score;
        }

        void CetRectangleFillClusterBuilder::_AppendProbePosition(std::vector<std::pair<double, double>>& APositions, double AX, double AY, double AMaxX, double AMaxY) const
        {
            if (APositions.size() >= CET_RECTANGLE_FILL_MAX_PROBE_COUNT){
                return;
            }
            if (!std::isfinite(AX) || !std::isfinite(AY) || AX < -CET_RECTANGLE_FILL_POSITION_TOLERANCE || AY < -CET_RECTANGLE_FILL_POSITION_TOLERANCE || AX > AMaxX + CET_RECTANGLE_FILL_POSITION_TOLERANCE || AY > AMaxY + CET_RECTANGLE_FILL_POSITION_TOLERANCE){
                return;
            }

            const double QuantizedX = std::llround(std::clamp(AX, 0.0, AMaxX));
            const double QuantizedY = std::llround(std::clamp(AY, 0.0, AMaxY));
            for (const auto& Existing : APositions){
                if (std::abs(Existing.first - QuantizedX) <= CET_RECTANGLE_FILL_POSITION_TOLERANCE && std::abs(Existing.second - QuantizedY) <= CET_RECTANGLE_FILL_POSITION_TOLERANCE){
                    return;
                }
            }
            APositions.emplace_back(QuantizedX, QuantizedY);
        }

        bool CetRectangleFillClusterBuilder::_ContainsOriginalIndex(const TetClusterCandidate& ACandidate, int AOriginalIndex) const
        {
            return std::find(ACandidate.OriginalIndices.begin(), ACandidate.OriginalIndices.end(), AOriginalIndex) != ACandidate.OriginalIndices.end();
        }

        double CetRectangleFillClusterBuilder::_GetFeatureArea(const CetNestItem& AItem, const TetShapeFeature& AFeature) const
        {
            const double FeatureArea = std::abs(AFeature.Area);
            if (FeatureArea > 0.0 && std::isfinite(FeatureArea)){
                return FeatureArea;
            }
            return std::abs(static_cast<double>(AItem.area()));
        }

    }
}
