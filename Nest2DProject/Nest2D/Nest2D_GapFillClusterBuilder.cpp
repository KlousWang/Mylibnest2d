#include "pch.h"
#include "Nest2D_GapFillClusterBuilder.h"
#include "Nest2D_ClusterGeometryHelper.h"
#include "Nest2D_RotationUtils.h"
#include "NestUtils.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace ET {
    namespace NEST2DMANAGERLIB {

        namespace {
            constexpr int CET_GAPFILL_MAX_FILLER_COUNT = 4;
            constexpr int CET_GAPFILL_GRID_PROBE_COUNT = 9;

            bool StartsWith(const std::string& AText, const std::string& APrefix)
            {
                return AText.size() >= APrefix.size() &&
                    std::equal(APrefix.begin(), APrefix.end(), AText.begin());
            }

            std::string MakeGapFillClusterType(const std::string& ACurrentType)
            {
                const std::string Marker = "_GapFill";
                const std::size_t MarkerPos = ACurrentType.find(Marker);
                const std::string BaseType = MarkerPos == std::string::npos? ACurrentType: ACurrentType.substr(0, MarkerPos);
                int NextFillCount = 1;
                if (MarkerPos != std::string::npos) {
                    const std::size_t NumberPos = MarkerPos + Marker.size();
                    if (NumberPos < ACurrentType.size()) {
                        int CurrentFillCount = 0;
                        bool HasNumber = false;
                        for (std::size_t CharacterIndex = NumberPos; CharacterIndex < ACurrentType.size(); ++CharacterIndex) {
                            const unsigned char Character = static_cast<unsigned char>(ACurrentType[CharacterIndex]);
                            if (!std::isdigit(Character)) {
                                break;
                            }
                            HasNumber = true;
                            CurrentFillCount = CurrentFillCount * 10 + (ACurrentType[CharacterIndex] - '0');
                        }

                        if (HasNumber) {
                            NextFillCount = CurrentFillCount + 1;
                        }
                    }
                }

                return BaseType + Marker + std::to_string(NextFillCount);
            }

            double GetFeatureArea(const TetShapeFeature& AFeature)
            {
                return std::max(0.0, AFeature.Area);
            }

            double GetFeatureBoxArea(const TetShapeFeature& AFeature)
            {
                return std::max(0.0, AFeature.Width * AFeature.Height);
            }

            bool IsLikelySmallerFiller(const TetShapeFeature& AFeature, const TetClusterCandidate& ABaseCandidate)
            {
                if (AFeature.Width <= 0.0 || AFeature.Height <= 0.0 || AFeature.Area <= 0.0) {
                    return false;
                }

                const double FeatureLongSide = std::max(AFeature.Width, AFeature.Height);
                const double BaseShortSide = std::min(ABaseCandidate.ClusterWidth, ABaseCandidate.ClusterHeight);
                if (BaseShortSide <= 0.0) {
                    return false;
                }

                return FeatureLongSide <= BaseShortSide * 0.65 &&
                    GetFeatureArea(AFeature) <= ABaseCandidate.ProxyArea * 0.25;
            }

            void AddProbePosition(std::vector<std::pair<double, double>>& AProbes, double AX, double AY, double AMaxX, double AMaxY, double ATolerance)
            {
                if (!std::isfinite(AX) || !std::isfinite(AY)) {
                    return;
                }

                if (AX < -ATolerance || AY < -ATolerance || AX > AMaxX + ATolerance || AY > AMaxY + ATolerance) {
                    return;
                }

                AX = std::clamp(AX, 0.0, AMaxX);
                AY = std::clamp(AY, 0.0, AMaxY);

                for (const auto& Probe : AProbes) {
                    if (std::abs(Probe.first - AX) <= ATolerance && std::abs(Probe.second - AY) <= ATolerance) {
                        return;
                    }
                }

                AProbes.emplace_back(AX, AY);
            }

            std::vector<std::pair<double, double>> BuildProbePositions(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ABaseCandidate, double AFillerWidth, double AFillerHeight)
            {
                std::vector<std::pair<double, double>> Probes;

                const double MaxX = ABaseCandidate.ClusterWidth - AFillerWidth;
                const double MaxY = ABaseCandidate.ClusterHeight - AFillerHeight;
                if (MaxX < 0.0 || MaxY < 0.0) {
                    return Probes;
                }

                const double ProbeTolerance = std::max(1.0, std::min(AFillerWidth, AFillerHeight) * 0.02);

                for (int ProbeY = 0; ProbeY < CET_GAPFILL_GRID_PROBE_COUNT; ++ProbeY) {
                    const double YRatio = CET_GAPFILL_GRID_PROBE_COUNT <= 1 ? 0.5 : static_cast<double>(ProbeY) / static_cast<double>(CET_GAPFILL_GRID_PROBE_COUNT - 1);
                    const double ProbeYPosition = MaxY * YRatio;

                    for (int ProbeX = 0; ProbeX < CET_GAPFILL_GRID_PROBE_COUNT; ++ProbeX) {
                        const double XRatio = CET_GAPFILL_GRID_PROBE_COUNT <= 1 ? 0.5 : static_cast<double>(ProbeX) / static_cast<double>(CET_GAPFILL_GRID_PROBE_COUNT - 1);
                        const double ProbeXPosition = MaxX * XRatio;
                        AddProbePosition(Probes, ProbeXPosition, ProbeYPosition, MaxX, MaxY, ProbeTolerance);
                    }
                }

                CetClusterGeometryHelper Geometry;
                std::vector<TetGapFillCircleCenter> BaseCenters;
                double MaxBaseSize = 0.0;

                for (const TetItemTransform& Transform : ABaseCandidate.Transforms) {
                    if (Transform.OriginalId < 0 ||
                        Transform.OriginalId >= static_cast<int>(AFeatures.size()) ||
                        Transform.OriginalId >= static_cast<int>(AOriginalItems.size())){
                        continue;
                    }

                    const TetShapeFeature& Feature = AFeatures[Transform.OriginalId];
                    if ((Feature.ShapeType != MetShapeType::CircleLike && Feature.ShapeType != MetShapeType::EllipseLike && Feature.ShapeType != MetShapeType::ArcLike) ||Feature.Width <= 0.0 ||
                        Feature.Height <= 0.0){
                        continue;
                    }

                    const CetPath ChildContour = Geometry.TransformContour(
                        Geometry.GetIdentityContour(AOriginalItems[Transform.OriginalId]),
                        Transform.RelativeRotation,
                        Transform.RelativeX,
                        Transform.RelativeY);

                    double ChildMinX = 0.0;
                    double ChildMinY = 0.0;
                    double ChildMaxX = 0.0;
                    double ChildMaxY = 0.0;
                    if (!Geometry.GetBounds(ChildContour, ChildMinX, ChildMinY, ChildMaxX, ChildMaxY)) {
                        continue;
                    }

                    const double ChildWidth = ChildMaxX - ChildMinX;
                    const double ChildHeight = ChildMaxY - ChildMinY;
                    if (ChildWidth <= 0.0 || ChildHeight <= 0.0) {
                        continue;
                    }

                    TetGapFillCircleCenter Center;
                    Center.X = (ChildMinX + ChildMaxX) * 0.5;
                    Center.Y = (ChildMinY + ChildMaxY) * 0.5;
                    Center.Size = std::max(ChildWidth, ChildHeight);
                    MaxBaseSize = std::max(MaxBaseSize, Center.Size);
                    BaseCenters.push_back(Center);
                }

                if (BaseCenters.empty() || MaxBaseSize <= 0.0) {
                    return Probes;
                }

                const double FillerCenterOffsetX = AFillerWidth * 0.5;
                const double FillerCenterOffsetY = AFillerHeight * 0.5;
                const double NeighborDistanceLimit = MaxBaseSize * 2.5;

                for (const TetGapFillCircleCenter& Center : BaseCenters) {
                    AddProbePosition(Probes, Center.X - FillerCenterOffsetX, Center.Y - FillerCenterOffsetY, MaxX, MaxY, ProbeTolerance);
                }

                for (std::size_t CenterIndex = 0; CenterIndex < BaseCenters.size(); ++CenterIndex) {
                    for (std::size_t OtherCenterIndex = CenterIndex + 1; OtherCenterIndex < BaseCenters.size(); ++OtherCenterIndex) {
                        const double DeltaX = BaseCenters[CenterIndex].X - BaseCenters[OtherCenterIndex].X;
                        const double DeltaY = BaseCenters[CenterIndex].Y - BaseCenters[OtherCenterIndex].Y;
                        const double Distance = std::sqrt(DeltaX * DeltaX + DeltaY * DeltaY);
                        if (Distance > NeighborDistanceLimit) {
                            continue;
                        }

                        const double CenterX = (BaseCenters[CenterIndex].X + BaseCenters[OtherCenterIndex].X) * 0.5;
                        const double CenterY = (BaseCenters[CenterIndex].Y + BaseCenters[OtherCenterIndex].Y) * 0.5;
                        AddProbePosition(Probes, CenterX - FillerCenterOffsetX, CenterY - FillerCenterOffsetY, MaxX, MaxY, ProbeTolerance);
                    }
                }

                for (std::size_t CenterIndex = 0; CenterIndex < BaseCenters.size(); ++CenterIndex) {
                    std::vector<std::pair<double, std::size_t>> Neighbors;

                    for (std::size_t NeighborCenterIndex = 0; NeighborCenterIndex < BaseCenters.size(); ++NeighborCenterIndex) {
                        if (CenterIndex == NeighborCenterIndex) {
                            continue;
                        }

                        const double DeltaX = BaseCenters[CenterIndex].X - BaseCenters[NeighborCenterIndex].X;
                        const double DeltaY = BaseCenters[CenterIndex].Y - BaseCenters[NeighborCenterIndex].Y;
                        const double Distance = std::sqrt(DeltaX * DeltaX + DeltaY * DeltaY);
                        if (Distance <= NeighborDistanceLimit) {
                            Neighbors.emplace_back(Distance, NeighborCenterIndex);
                        }
                    }

                    std::sort(
                        Neighbors.begin(),
                        Neighbors.end(),
                        [](const auto& FirstNeighbor, const auto& SecondNeighbor)
                        {
                            return FirstNeighbor.first < SecondNeighbor.first;
                        });

                    if (Neighbors.size() > 6) {
                        Neighbors.resize(6);
                    }

                    for (std::size_t FirstNeighborOffset = 0; FirstNeighborOffset < Neighbors.size(); ++FirstNeighborOffset) {
                        for (std::size_t SecondNeighborOffset = FirstNeighborOffset + 1; SecondNeighborOffset < Neighbors.size(); ++SecondNeighborOffset) {
                            const std::size_t FirstNeighborIndex = Neighbors[FirstNeighborOffset].second;
                            const std::size_t SecondNeighborIndex = Neighbors[SecondNeighborOffset].second;
                            const double NeighborDeltaX = BaseCenters[FirstNeighborIndex].X - BaseCenters[SecondNeighborIndex].X;
                            const double NeighborDeltaY = BaseCenters[FirstNeighborIndex].Y - BaseCenters[SecondNeighborIndex].Y;
                            const double NeighborDistance = std::sqrt(NeighborDeltaX * NeighborDeltaX + NeighborDeltaY * NeighborDeltaY);
                            if (NeighborDistance > NeighborDistanceLimit) {
                                continue;
                            }

                            const double CenterX = (BaseCenters[CenterIndex].X + BaseCenters[FirstNeighborIndex].X + BaseCenters[SecondNeighborIndex].X) / 3.0;
                            const double CenterY = (BaseCenters[CenterIndex].Y + BaseCenters[FirstNeighborIndex].Y + BaseCenters[SecondNeighborIndex].Y) / 3.0;
                            AddProbePosition(Probes, CenterX - FillerCenterOffsetX, CenterY - FillerCenterOffsetY, MaxX, MaxY, ProbeTolerance);
                        }
                    }
                }

                return Probes;
            }
        }

        CetGapFillClusterBuilder::CetGapFillClusterBuilder() : CetCoreObject() {}
        CetGapFillClusterBuilder::~CetGapFillClusterBuilder() {}

        void CetGapFillClusterBuilder::BuildCandidates(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<TetClusterCandidate>& ABaseCandidates, const TetNestOptions& AOptions, std::vector<TetClusterCandidate>& AOutCandidates)
        {
            if (AOriginalItems.empty() || AFeatures.size() != AOriginalItems.size() || ABaseCandidates.empty()) {
                return;
            }

            const std::size_t OldCandidateCount = AOutCandidates.size();

            for (const TetClusterCandidate& BaseCandidate : ABaseCandidates) {
                TetClusterCandidate Candidate;
                if (!_BuildGapFillCandidate(AOriginalItems, AFeatures, BaseCandidate, AOptions, nullptr, Candidate)) {
                    continue;
                }

                AOutCandidates.push_back(std::move(Candidate));
                std::cout << "[GAPFILL][CANDIDATE] BaseType = " << BaseCandidate.ClusterType
                    << ", ChildCount = " << AOutCandidates.back().OriginalIndices.size()
                    << ", Score = " << AOutCandidates.back().Score << std::endl;
            }

            std::cout << "[GAPFILL][BUILD CANDIDATES] BaseCandidateCount = " << ABaseCandidates.size()
                << ", NewCandidateCount = " << AOutCandidates.size() - OldCandidateCount << std::endl;
        }

        bool CetGapFillClusterBuilder::BuildCandidateForBase(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ABaseCandidate, const TetNestOptions& AOptions, const std::vector<bool>& AUsed, TetClusterCandidate& AOutCandidate)
        {
            AOutCandidate = TetClusterCandidate{};
            if (AOriginalItems.empty() || AFeatures.size() != AOriginalItems.size() || AUsed.size() != AOriginalItems.size()) {
                return false;
            }
            return _BuildGapFillCandidate(AOriginalItems, AFeatures, ABaseCandidate, AOptions, &AUsed, AOutCandidate);
        }
        bool CetGapFillClusterBuilder::_BuildGapFillCandidate(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ABaseCandidate, const TetNestOptions& AOptions, const std::vector<bool>* AUsed, TetClusterCandidate& AOutCandidate)
        {
            AOutCandidate = TetClusterCandidate{};

            if (!_IsSupportedBaseCandidate(ABaseCandidate) || !ABaseCandidate.Valid || ABaseCandidate.ProxyContour.size() < 3) {
                return false;
            }

            bool HasFiller = false;
            TetClusterCandidate CurrentCandidate = ABaseCandidate;

            for (int FillStep = 0; FillStep < CET_GAPFILL_MAX_FILLER_COUNT; ++FillStep) {
                bool HasBestNext = false;
                TetClusterCandidate BestNextCandidate;

                for (int Index = 0; Index < static_cast<int>(AFeatures.size()); ++Index) {
                    if (AUsed != nullptr) {
                        if (Index >= static_cast<int>(AUsed->size()) || (*AUsed)[Index]) {
                            continue;
                        }
                    }

                    if (_ContainsOriginalIndex(CurrentCandidate, Index)) {
                        continue;
                    }

                    if (!_CanUseAsFiller(AFeatures[Index], CurrentCandidate)) {
                        continue;
                    }

                    TetClusterCandidate Candidate;
                    if (!_TryAddFiller(AOriginalItems, AFeatures, CurrentCandidate, Index, AOptions, Candidate)) {
                        continue;
                    }

                    if (!HasBestNext || Candidate.Score > BestNextCandidate.Score) {
                        HasBestNext = true;
                        BestNextCandidate = std::move(Candidate);
                    }
                }

                if (!HasBestNext) {
                    break;
                }

                HasFiller = true;
                CurrentCandidate = std::move(BestNextCandidate);
            }

            if (!HasFiller) {
                return false;
            }

            AOutCandidate = std::move(CurrentCandidate);
            return true;
        }

        bool CetGapFillClusterBuilder::_TryAddFiller(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ABaseCandidate, int AFillerIndex, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate)
        {
            AOutCandidate = TetClusterCandidate{};

            if (AFillerIndex < 0 || AFillerIndex >= static_cast<int>(AOriginalItems.size()) || AFillerIndex >= static_cast<int>(AFeatures.size())) {
                return false;
            }

            const TetShapeFeature& FillerFeature = AFeatures[AFillerIndex];
            if (!_CanUseAsFiller(FillerFeature, ABaseCandidate)) {
                return false;
            }

            CetClusterGeometryHelper Geometry;
            const double RequiredGap = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));
            const double DimensionTolerance = std::max(1.0, RequiredGap * 0.001);

            const std::vector<double> Rotations = CetRotationUtils::BuildAllowedRotations(AOptions.Rotations);

            bool HasBest = false;
            TetClusterCandidate BestCandidate;

            for (double Rotation : Rotations) {
                CetPath RotatedFiller = Geometry.TransformContour(
                    Geometry.GetIdentityContour(AOriginalItems[AFillerIndex]),
                    Rotation,
                    0.0,
                    0.0);

                double FillerMinX = 0.0;
                double FillerMinY = 0.0;
                double FillerMaxX = 0.0;
                double FillerMaxY = 0.0;

                if (!Geometry.GetBounds(RotatedFiller, FillerMinX, FillerMinY, FillerMaxX, FillerMaxY)) {
                    continue;
                }

                const double FillerWidth = FillerMaxX - FillerMinX;
                const double FillerHeight = FillerMaxY - FillerMinY;
                if (FillerWidth <= 0.0 || FillerHeight <= 0.0) {
                    continue;
                }

                const double MaxX = ABaseCandidate.ClusterWidth - FillerWidth;
                const double MaxY = ABaseCandidate.ClusterHeight - FillerHeight;
                if (MaxX < 0.0 || MaxY < 0.0) {
                    continue;
                }

                const std::vector<std::pair<double, double>> ProbePositions = BuildProbePositions(AOriginalItems, AFeatures, ABaseCandidate, FillerWidth, FillerHeight);
                for (const auto& ProbePosition : ProbePositions) {
                    const double ProbeXPosition = ProbePosition.first;
                    const double ProbeYPosition = ProbePosition.second;
                    TetItemTransform FillerTransform;
                    FillerTransform.OriginalId = AFillerIndex;
                    FillerTransform.RelativeRotation = Rotation;
                    FillerTransform.RelativeX = ProbeXPosition - FillerMinX;
                    FillerTransform.RelativeY = ProbeYPosition - FillerMinY;

                    TetClusterCandidate Candidate = ABaseCandidate;
                    Candidate.Valid = false;
                    Candidate.BuilderName = "GapFillBuilder";
                    Candidate.ClusterType = MakeGapFillClusterType(ABaseCandidate.ClusterType);
                    Candidate.OriginalIndices.push_back(AFillerIndex);
                    Candidate.Transforms.push_back(FillerTransform);
                    Candidate.Confidence = std::min(1.0, ABaseCandidate.Confidence + 0.05);

                    if (!Geometry.FinalizeCandidate(AOriginalItems, AOptions, Candidate)) {
                        continue;
                    }

                    if (Candidate.ClusterWidth > ABaseCandidate.ClusterWidth + DimensionTolerance ||
                        Candidate.ClusterHeight > ABaseCandidate.ClusterHeight + DimensionTolerance)
                    {
                        continue;
                    }

                    Candidate.Score = _CalculateScore(Candidate);

                    if (!HasBest || Candidate.Score > BestCandidate.Score) {
                        HasBest = true;
                        BestCandidate = std::move(Candidate);
                    }
                }
            }

            if (!HasBest) {
                return false;
            }

            AOutCandidate = std::move(BestCandidate);
            return true;
        }

        bool CetGapFillClusterBuilder::_IsSupportedBaseCandidate(const TetClusterCandidate& ACandidate)
        {
            return ACandidate.BuilderName == "CircleBuilder" ||
                ACandidate.BuilderName == "EllipseBuilder" ||
                ACandidate.BuilderName == "ArcBuilder" ||
                StartsWith(ACandidate.ClusterType, "Circle") ||
                StartsWith(ACandidate.ClusterType, "Ellipse") ||
                StartsWith(ACandidate.ClusterType, "Arc") ||
                StartsWith(ACandidate.ClusterType, "SemiCircle");
        }

        bool CetGapFillClusterBuilder::_CanUseAsFiller(const TetShapeFeature& AFeature, const TetClusterCandidate& ABaseCandidate)
        {
            if (!IsLikelySmallerFiller(AFeature, ABaseCandidate)) {
                return false;
            }

            return AFeature.ShapeType != MetShapeType::CircleLike ||
                GetFeatureBoxArea(AFeature) <= ABaseCandidate.ProxyArea * 0.16;
        }

        bool CetGapFillClusterBuilder::_ContainsOriginalIndex(const TetClusterCandidate& ACandidate, int AOriginalIndex)
        {
            return std::find(ACandidate.OriginalIndices.begin(), ACandidate.OriginalIndices.end(), AOriginalIndex) != ACandidate.OriginalIndices.end();
        }

        double CetGapFillClusterBuilder::_CalculateScore(const TetClusterCandidate& ACandidate)
        {
            if (!ACandidate.Valid || ACandidate.OriginalIndices.empty() || ACandidate.ProxyArea <= 0.0 || ACandidate.ClusterWidth <= 0.0 || ACandidate.ClusterHeight <= 0.0) {
                return -std::numeric_limits<double>::infinity();
            }

            const double FillScore = ACandidate.FillRatio * 1200.0;
            const double ItemCountScore = static_cast<double>(ACandidate.OriginalIndices.size()) * 45.0;
            const double SavingScore = ACandidate.AreaSavingRatio * 700.0;
            const double FillerBonus = 80.0;
            const double SizePenalty = (ACandidate.ClusterWidth + ACandidate.ClusterHeight) * 0.000001;
            return FillScore + ItemCountScore + SavingScore + FillerBonus + ACandidate.Confidence - SizePenalty;
        }

    }
}
