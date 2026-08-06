#include "pch.h"
#include "Nest2D_RectangleFillClusterBuilder.h"
#include "Nest2D_ClusterGeometryHelper.h"
#include "Nest2D_RotationUtils.h"
#include "NestUtils.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <unordered_set>
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
            const double EnvelopeArea = EnvelopeWidth * EnvelopeHeight;
            const double BaseAreaTolerance = std::max(1.0, EnvelopeArea * 1e-9);
            const double BaseAvailableArea = std::max(0.0, EnvelopeArea - ABaseCandidate.ReservedArea);
            if (!std::isfinite(EnvelopeArea) || EnvelopeArea <= 0.0 || BaseAvailableArea <= BaseAreaTolerance){
                return false;
            }
            if (EnvelopeWidth <= 0.0 || EnvelopeHeight <= 0.0 || !Geometry.FinalizeCandidateInRectangle(AOriginalItems, AOptions, CurrentCandidate, EnvelopeWidth, EnvelopeHeight)){
                return false;
            }

            std::vector<int> FillerIndices;
            FillerIndices.reserve(AOriginalItems.size());
            for (int Index = 0; Index < static_cast<int>(AOriginalItems.size()); ++Index){
                if (AUsed[Index] || _ContainsOriginalIndex(CurrentCandidate, Index)){
                    continue;
                }
                if (AFeatures[Index].Area <= 0.0 || AFeatures[Index].Width <= 0.0 || AFeatures[Index].Height <= 0.0){
                    continue;
                }
                if (_GetFeatureArea(AOriginalItems[Index], AFeatures[Index]) > BaseAvailableArea + BaseAreaTolerance){
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
            if (FillerIndices.size() > CET_RECTANGLE_FILL_MAX_CANDIDATE_ITEMS){
                constexpr std::size_t SmallCandidateCount = 16;
                const std::size_t LargeCandidateCount = CET_RECTANGLE_FILL_MAX_CANDIDATE_ITEMS - SmallCandidateCount;
                std::vector<int> BoundedIndices;
                BoundedIndices.reserve(CET_RECTANGLE_FILL_MAX_CANDIDATE_ITEMS);
                BoundedIndices.insert(BoundedIndices.end(),FillerIndices.begin(),FillerIndices.begin() + static_cast<std::vector<int>::difference_type>(LargeCandidateCount));
                BoundedIndices.insert(BoundedIndices.end(),FillerIndices.end() - static_cast<std::vector<int>::difference_type>(SmallCandidateCount),FillerIndices.end());
                FillerIndices = std::move(BoundedIndices);
            }

            const std::size_t InitialChildCount = CurrentCandidate.OriginalIndices.size();
            // A failed placement for a contour cannot become valid until another
            // filler changes the cluster geometry.  Orders frequently contain many
            // identical small parts; retrying each copy against an unchanged void
            // adds a large amount of repeated spacing/intersection work without
            // improving the result.  Keep the failure cache local to this base
            // candidate and clear it whenever a part is actually inserted.
            auto BuildFillerShapeSignature = [&](int AIndex) {
                std::uint64_t Hash = 1469598103934665603ULL;
                const auto Mix = [&](std::uint64_t AValue) {
                    Hash ^= AValue;
                    Hash *= 1099511628211ULL;
                };

                const TetShapeFeature& Feature = AFeatures[AIndex];
                Mix(static_cast<std::uint64_t>(Feature.ShapeType));
                Mix(static_cast<std::uint64_t>(Feature.NormalizedContour.size()));
                for (const ClipperLib::IntPoint& Point : Feature.NormalizedContour){
                    Mix(static_cast<std::uint64_t>(Point.X));
                    Mix(static_cast<std::uint64_t>(Point.Y));
                }
                // Hole contours are not represented by NormalizedContour. Do not
                // deduplicate them unless their full geometry is available here.
                if (Feature.HasHoles){
                    Mix(static_cast<std::uint64_t>(AIndex));
                }
                return Hash;
            };
            std::unordered_set<std::uint64_t> FailedFillerShapes;
            std::size_t MaxAcceptedFillerCount = InitialChildCount >= 16
                ? CET_RECTANGLE_FILL_LARGE_BASE_MAX_ACCEPTED_ITEMS
                : (InitialChildCount >= 8
                    ? CET_RECTANGLE_FILL_MEDIUM_BASE_MAX_ACCEPTED_ITEMS
                    : CET_RECTANGLE_FILL_MAX_ACCEPTED_ITEMS_PER_BASE);
            // Circle groups gain enough material-saving value from their
            // concave envelope voids to justify their dedicated bounded cap.
            const bool IsCircleOnlyBase = std::all_of(CurrentCandidate.OriginalIndices.begin(), CurrentCandidate.OriginalIndices.end(), [&](int AOriginalIndex) {
                return AOriginalIndex >= 0 && AOriginalIndex < static_cast<int>(AFeatures.size()) && AFeatures[AOriginalIndex].ShapeType == MetShapeType::CircleLike;
            });
            for (int FillerIndex : FillerIndices){
                if (CurrentCandidate.OriginalIndices.size() - InitialChildCount >= MaxAcceptedFillerCount){
                    break;
                }
                if (AUsed[FillerIndex] || _ContainsOriginalIndex(CurrentCandidate, FillerIndex)){
                    continue;
                }
                const double FillerArea = _GetFeatureArea(AOriginalItems[FillerIndex], AFeatures[FillerIndex]);
                if (IsCircleOnlyBase && AFeatures[FillerIndex].ShapeType == MetShapeType::CircleLike){
                    MaxAcceptedFillerCount = std::max(MaxAcceptedFillerCount, CET_CIRCLE_GAP_FILL_MAX_ACCEPTED_ITEMS);
                }
                const double AvailableArea = std::max(0.0, EnvelopeArea - CurrentCandidate.ReservedArea);
                const double AreaTolerance = std::max(1.0, EnvelopeArea * 1e-9);
                if (FillerArea > AvailableArea + AreaTolerance){
                    continue;
                }

                const std::uint64_t FillerShapeSignature = BuildFillerShapeSignature(FillerIndex);
                if (FailedFillerShapes.find(FillerShapeSignature) != FailedFillerShapes.end()){
                    continue;
                }

                TetClusterCandidate Candidate;
                if (!_TryAddFiller(AOriginalItems, AFeatures, CurrentCandidate, FillerIndex, AOptions, EnvelopeWidth, EnvelopeHeight, Candidate)){
                    FailedFillerShapes.insert(FillerShapeSignature);
                    continue;
                }

                CurrentCandidate = std::move(Candidate);
                FailedFillerShapes.clear();
                std::cout << "[GAP_FILL][ACCEPT] OriginalId=" << FillerIndex << ", ChildCount=" << CurrentCandidate.OriginalIndices.size() << ", Width=" << CurrentCandidate.ClusterWidth << ", Height=" << CurrentCandidate.ClusterHeight << std::endl;
            }

            if (CurrentCandidate.OriginalIndices.size() == InitialChildCount){
                return false;
            }

            CurrentCandidate.BuilderName = "GapFillBuilder";
            CurrentCandidate.ClusterType = ABaseCandidate.ClusterType + "_GapFill";
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
                _BuildProbePositions(AOriginalItems, AFeatures, ACurrentCandidate, AFillerIndex, RotatedFiller, FillerMinX, FillerMinY, FillerMaxX, FillerMaxY, RequiredGap, ProbePositions);
                const double MaxX = AEnvelopeWidth - FillerWidth;
                const double MaxY = AEnvelopeHeight - FillerHeight;
                const std::size_t ProbeCount = AOriginalItems.size() > CET_NEST_FULL_STRATEGY_ITEM_LIMIT
                    ? std::min(ProbePositions.size(),CET_RECTANGLE_FILL_LARGE_ORDER_MAX_PROBE_COUNT)
                    : ProbePositions.size();
                for (std::size_t ProbeIndex = 0; ProbeIndex < ProbeCount; ++ProbeIndex){
                    const auto& ProbePosition = ProbePositions[ProbeIndex];
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
            BestCandidate.BuilderName = "GapFillBuilder";
            AOutCandidate = std::move(BestCandidate);
            return true;
        }

        void CetRectangleFillClusterBuilder::_BuildProbePositions(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ACandidate, int AFillerIndex, const CetPath& ARotatedFiller, double AFillerMinX, double AFillerMinY, double AFillerMaxX, double AFillerMaxY, double ARequiredGap, std::vector<std::pair<double, double>>& AOutPositions)
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

            const bool IsCircleFiller = AFillerIndex >= 0 && AFillerIndex < static_cast<int>(AFeatures.size()) && AFeatures[AFillerIndex].ShapeType == MetShapeType::CircleLike;
            if (IsCircleFiller){
                struct TetCircleCenter
                {
                    double X = 0.0;
                    double Y = 0.0;
                    double Radius = 0.0;
                };

                std::vector<TetCircleCenter> CircleCenters;
                CircleCenters.reserve(ACandidate.Transforms.size());
                CetClusterGeometryHelper Geometry;
                for (const TetItemTransform& Transform : ACandidate.Transforms){
                    if (Transform.OriginalId < 0 || Transform.OriginalId >= static_cast<int>(AFeatures.size()) || AFeatures[Transform.OriginalId].ShapeType != MetShapeType::CircleLike){
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
                    const double Width = ChildMaxX - MinX;
                    const double Height = ChildMaxY - MinY;
                    if (Width <= 0.0 || Height <= 0.0 || std::abs(Width - Height) > std::max(1.0, std::max(Width, Height) * 0.02)){
                        continue;
                    }
                    CircleCenters.push_back({ (MinX + ChildMaxX) * 0.5, (MinY + ChildMaxY) * 0.5, std::min(Width, Height) * 0.5 });
                }

                const double FillerRadius = std::min(AFillerWidth, AFillerHeight) * 0.5;
                std::size_t PairProbeCount = 0;
                for (std::size_t First = 0; First < CircleCenters.size() && PairProbeCount < CET_CIRCLE_FILL_MAX_PAIR_PROBES; ++First){
                    for (std::size_t Second = First + 1; Second < CircleCenters.size() && PairProbeCount < CET_CIRCLE_FILL_MAX_PAIR_PROBES; ++Second){
                        const TetCircleCenter& A = CircleCenters[First];
                        const TetCircleCenter& B = CircleCenters[Second];
                        const double DeltaX = B.X - A.X;
                        const double DeltaY = B.Y - A.Y;
                        const double CenterDistance = std::hypot(DeltaX, DeltaY);
                        const double RequiredDistance = std::max(A.Radius, B.Radius) + FillerRadius + ARequiredGap;
                        if (CenterDistance <= 1e-9 || CenterDistance * 0.5 > RequiredDistance){
                            continue;
                        }
                        const double OffsetSquared = RequiredDistance * RequiredDistance - CenterDistance * CenterDistance * 0.25;
                        if (OffsetSquared < 0.0){
                            continue;
                        }
                        const double Offset = std::sqrt(OffsetSquared);
                        const double PerpendicularX = -DeltaY / CenterDistance;
                        const double PerpendicularY = DeltaX / CenterDistance;
                        const double MidX = (A.X + B.X) * 0.5;
                        const double MidY = (A.Y + B.Y) * 0.5;
                        AddPosition(MidX + PerpendicularX * Offset - AFillerWidth * 0.5, MidY + PerpendicularY * Offset - AFillerHeight * 0.5);
                        AddPosition(MidX - PerpendicularX * Offset - AFillerWidth * 0.5, MidY - PerpendicularY * Offset - AFillerHeight * 0.5);
                        ++PairProbeCount;
                    }
                }
            }

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

            // Reserve probes across the whole envelope before a complex base
            // contributes enough child-edge positions to consume the budget.
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
        }

        double CetRectangleFillClusterBuilder::_CalculatePlacementScore(const CetTNestItemVector& AOriginalItems, const TetClusterCandidate& ACandidate, double AFillerLeft, double AFillerTop, double AFillerRight, double AFillerBottom, double ARequiredGap) const
        {
            const double ContactTolerance = ARequiredGap + CET_RECTANGLE_FILL_POSITION_TOLERANCE;
            double Score = -(AFillerTop + AFillerLeft) / std::max(1.0, ACandidate.ClusterWidth + ACandidate.ClusterHeight);
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
                    Score += 3.0;
                }
                if (HorizontalOverlap && VerticalGap <= ContactTolerance){
                    Score += 3.0;
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
