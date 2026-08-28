#include "pch.h"
#include "Nest2D_ArcClusterBuilder.h"
#include "Nest2D_SelfFunction.h"
#include "Nest2D_ClusterGeometryHelper.h"
#include "Nest2D_ClusterMathUtils.h"
#include "Nest2D_RotationUtils.h"
#include "NestUtils.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>
namespace ET {
    namespace NEST2DMANAGERLIB {
        namespace {
            int NormalizeBulgeSign(int ABulgeSign) { return ABulgeSign < 0 ? -1 : 1; }
            double GetEffectiveSweepAngle(const TetShapeFeature &AFeature)
            {
                if (std::isfinite(AFeature.ArcSweepAngle) && AFeature.ArcSweepAngle > 0.0) {
                    return std::abs(AFeature.ArcSweepAngle);
                }
                return AFeature.ArcType == MetArcType::SemiCircleLike ? CET_CLUSTER_PI : 0.0;
            }
            MetArcSweepBucket GetSweepBucket(const TetShapeFeature &AFeature)
            {
                const double SweepAngle = GetEffectiveSweepAngle(AFeature);
                if (SweepAngle <= 0.0) {
                    return MetArcSweepBucket::Unknown;
                }
                if (std::abs(SweepAngle - CET_CLUSTER_PI) <= CET_ARC_SWEEP_TOLERANCE) {
                    return MetArcSweepBucket::SemiCircle;
                }
                return SweepAngle < CET_CLUSTER_PI ? MetArcSweepBucket::LessThanSemiCircle : MetArcSweepBucket::MoreThanSemiCircle;
            }
            const char *GetSweepBucketName(MetArcSweepBucket ASweepBucket)
            {
                switch (ASweepBucket) {
                case MetArcSweepBucket::LessThanSemiCircle:
                    return "Less180";
                case MetArcSweepBucket::SemiCircle:
                    return "SemiCircle";
                case MetArcSweepBucket::MoreThanSemiCircle:
                    return "More180";
                default:
                    return "Unknown";
                }
            }
            bool IsValidArcFeature(const TetShapeFeature &AFeature) { return AFeature.ShapeType == MetShapeType::ArcLike && AFeature.ArcType != MetArcType::None && GetSweepBucket(AFeature) != MetArcSweepBucket::Unknown && AFeature.ArcRadius > 0.0 && AFeature.ArcChordLength > 0.0 && AFeature.Width > 0.0 && AFeature.Height > 0.0 && AFeature.Area > 0.0; }
            TetArcIndexInfo MakeArcIndexInfo(int AOriginalIndex, const TetShapeFeature &AFeature)
            {
                TetArcIndexInfo Info;
                Info.Index = AOriginalIndex;
                Info.ArcType = AFeature.ArcType;
                Info.SweepBucket = GetSweepBucket(AFeature);
                Info.Radius = AFeature.ArcRadius;
                Info.ChordLength = AFeature.ArcChordLength;
                Info.SweepAngle = GetEffectiveSweepAngle(AFeature);
                Info.BulgeSign = NormalizeBulgeSign(AFeature.ArcBulgeSign);
                return Info;
            }
            bool AreCompatibleArcInfos(const TetArcIndexInfo &ABaseInfo, const TetArcIndexInfo &ATestInfo)
            {
                if (ABaseInfo.ArcType != ATestInfo.ArcType || ABaseInfo.SweepBucket != ATestInfo.SweepBucket || ABaseInfo.BulgeSign != ATestInfo.BulgeSign) {
                    return false;
                }
                const bool RadiusMatches = CetClusterMathUtils::NearlyEqual(ABaseInfo.Radius, ATestInfo.Radius, CET_ARC_SIZE_TOLERANCE);
                const bool ChordMatches = CetClusterMathUtils::NearlyEqual(ABaseInfo.ChordLength, ATestInfo.ChordLength, CET_ARC_SIZE_TOLERANCE);
                const bool SweepMatches = std::abs(ABaseInfo.SweepAngle - ATestInfo.SweepAngle) <= CET_ARC_SWEEP_TOLERANCE || CetClusterMathUtils::NearlyEqual(ABaseInfo.SweepAngle, ATestInfo.SweepAngle, CET_ARC_SIZE_TOLERANCE);
                return RadiusMatches && ChordMatches && SweepMatches;
            }
            std::vector<std::vector<int>> GroupCompatibleArcIndices(const std::vector<int> &AIndices, const std::vector<TetShapeFeature> &AFeatures)
            {
                std::vector<TetArcIndexInfo> Infos;
                Infos.reserve(AIndices.size());
                for (int OriginalIndex : AIndices) {
                    if (OriginalIndex < 0 || OriginalIndex >= static_cast<int>(AFeatures.size())) {
                        continue;
                    }
                    const TetShapeFeature &Feature = AFeatures[OriginalIndex];
                    if (!IsValidArcFeature(Feature)) {
                        continue;
                    }
                    Infos.push_back(MakeArcIndexInfo(OriginalIndex, Feature));
                }
                std::sort(Infos.begin(), Infos.end(), [](const TetArcIndexInfo &AFirstInfo, const TetArcIndexInfo &ASecondInfo) {
                    if (AFirstInfo.ArcType != ASecondInfo.ArcType) {
                        return static_cast<int>(AFirstInfo.ArcType) < static_cast<int>(ASecondInfo.ArcType);
                    }
                    if (AFirstInfo.SweepBucket != ASecondInfo.SweepBucket) {
                        return static_cast<int>(AFirstInfo.SweepBucket) < static_cast<int>(ASecondInfo.SweepBucket);
                    }
                    if (AFirstInfo.BulgeSign != ASecondInfo.BulgeSign) {
                        return AFirstInfo.BulgeSign < ASecondInfo.BulgeSign;
                    }
                    if (std::abs(AFirstInfo.Radius - ASecondInfo.Radius) > 1.0) {
                        return AFirstInfo.Radius < ASecondInfo.Radius;
                    }
                    if (std::abs(AFirstInfo.ChordLength - ASecondInfo.ChordLength) > 1.0) {
                        return AFirstInfo.ChordLength < ASecondInfo.ChordLength;
                    }
                    if (std::abs(AFirstInfo.SweepAngle - ASecondInfo.SweepAngle) > 1e-9) {
                        return AFirstInfo.SweepAngle < ASecondInfo.SweepAngle;
                    }
                    return AFirstInfo.Index < ASecondInfo.Index;
                });
                Infos.erase(std::unique(Infos.begin(), Infos.end(), [](const TetArcIndexInfo &AFirstInfo, const TetArcIndexInfo &ASecondInfo) { return AFirstInfo.Index == ASecondInfo.Index; }), Infos.end());
                std::vector<std::vector<int>> Groups;
                std::vector<int> CurrentGroup;
                TetArcIndexInfo CurrentBaseInfo;
                for (const TetArcIndexInfo &Info : Infos) {
                    if (CurrentGroup.empty()) {
                        CurrentBaseInfo = Info;
                        CurrentGroup.push_back(Info.Index);
                        continue;
                    }
                    if (AreCompatibleArcInfos(CurrentBaseInfo, Info)) {
                        CurrentGroup.push_back(Info.Index);
                    } else {
                        Groups.push_back(std::move(CurrentGroup));
                        CurrentGroup.clear();
                        CurrentBaseInfo = Info;
                        CurrentGroup.push_back(Info.Index);
                    }
                }
                if (!CurrentGroup.empty()) {
                    Groups.push_back(std::move(CurrentGroup));
                }
                return Groups;
            }
            bool GetArcOrientationBounds(const CetNestItem &AItem, const TetShapeFeature &AFeature, bool AReverseChordDirection, const TetNestOptions &AOptions, const CetClusterGeometryHelper &AGeometry, TetArcOrientationBounds &AOutBounds)
            {
                AOutBounds = TetArcOrientationBounds{};
                const double TargetRotation = AReverseChordDirection ? CET_CLUSTER_PI - AFeature.ArcChordAngle : -AFeature.ArcChordAngle;
                if (!CetRotationUtils::SnapToNearestAllowedRotation(TargetRotation, AOptions.Rotations, AOutBounds.Rotation)) {
                    return false;
                }
                const CetPath RotatedContour = AGeometry.TransformContour(AGeometry.GetIdentityContour(AItem), AOutBounds.Rotation, 0.0, 0.0);
                double MaxX = 0.0;
                double MaxY = 0.0;
                if (!AGeometry.GetBounds(RotatedContour, AOutBounds.MinX, AOutBounds.MinY, MaxX, MaxY)) {
                    return false;
                }
                AOutBounds.Width = MaxX - AOutBounds.MinX;
                AOutBounds.Height = MaxY - AOutBounds.MinY;
                return AOutBounds.Width > 0.0 && AOutBounds.Height > 0.0;
            }
            TetArcLayout MakeLineLayout(const TetArcLayoutRequest &ARequest)
            {
                const std::size_t AArcCount = ARequest.ArcCount; const auto &AForwardBounds = ARequest.ForwardBounds; const auto &AReverseBounds = ARequest.ReverseBounds; const double AGap = ARequest.Gap; const bool AVerticalStack = ARequest.VerticalStack; const bool AAlternateDirection = ARequest.AlternateDirection; const auto &AStyleName = ARequest.StyleName;
                TetArcLayout Layout;
                if (AArcCount < 2) {
                    return Layout;
                }
                Layout.Slots.reserve(AArcCount);
                if (AVerticalStack) {
                    double CurrentY = 0.0;
                    double MaxWidth = 0.0;
                    for (std::size_t arcOffset = 0; arcOffset < AArcCount; ++arcOffset) {
                        const bool ReverseChordDirection = AAlternateDirection && (arcOffset % 2 == 1);
                        const TetArcOrientationBounds &SlotBounds = ReverseChordDirection ? AReverseBounds : AForwardBounds;
                        Layout.Slots.push_back({0.0, CurrentY, ReverseChordDirection});
                        CurrentY += SlotBounds.Height + AGap;
                        MaxWidth = std::max(MaxWidth, SlotBounds.Width);
                    }
                    for (TetArcLayoutSlot &Slot : Layout.Slots) {
                        const TetArcOrientationBounds &SlotBounds = Slot.ReverseChordDirection ? AReverseBounds : AForwardBounds;
                        Slot.X = (MaxWidth - SlotBounds.Width) * 0.5;
                    }
                    Layout.Width = MaxWidth;
                    Layout.Height = CurrentY - AGap;
                    Layout.ClusterType = AArcCount == 2 && AStyleName == "SemiCircle" && AAlternateDirection ? "SemiCirclePair" : "ArcColumn_" + std::to_string(AArcCount) + "_" + AStyleName;
                    return Layout;
                }
                double CurrentX = 0.0;
                double MaxHeight = 0.0;
                for (std::size_t arcOffset = 0; arcOffset < AArcCount; ++arcOffset) {
                    const bool ReverseChordDirection = AAlternateDirection && (arcOffset % 2 == 1);
                    const TetArcOrientationBounds &SlotBounds = ReverseChordDirection ? AReverseBounds : AForwardBounds;
                    Layout.Slots.push_back({CurrentX, 0.0, ReverseChordDirection});
                    CurrentX += SlotBounds.Width + AGap;
                    MaxHeight = std::max(MaxHeight, SlotBounds.Height);
                }
                for (TetArcLayoutSlot &Slot : Layout.Slots) {
                    const TetArcOrientationBounds &SlotBounds = Slot.ReverseChordDirection ? AReverseBounds : AForwardBounds;
                    Slot.Y = (MaxHeight - SlotBounds.Height) * 0.5;
                }
                Layout.Width = CurrentX - AGap;
                Layout.Height = MaxHeight;
                Layout.ClusterType = AAlternateDirection ? "ArcAlternatingLine_" : "ArcLine_";
                Layout.ClusterType += std::to_string(AArcCount) + "_" + AStyleName;
                return Layout;
            }
            TetArcLayout MakeGridLayout(const TetArcLayoutRequest &ARequest)
            {
                const std::size_t AArcCount = ARequest.ArcCount; const int ARowCount = ARequest.RowCount; const auto &AForwardBounds = ARequest.ForwardBounds; const auto &AReverseBounds = ARequest.ReverseBounds; const double AGap = ARequest.Gap; const bool AAlternateDirection = ARequest.AlternateDirection; const auto &AStyleName = ARequest.StyleName;
                TetArcLayout Layout;
                if (AArcCount < 3 || ARowCount < 2 || ARowCount > static_cast<int>(AArcCount)) {
                    return Layout;
                }
                const int ColumnCount = static_cast<int>((AArcCount + static_cast<std::size_t>(ARowCount) - 1) / static_cast<std::size_t>(ARowCount));
                if (ColumnCount < 2) {
                    return Layout;
                }
                const double CellWidth = AAlternateDirection ? std::max(AForwardBounds.Width, AReverseBounds.Width) : AForwardBounds.Width;
                const double CellHeight = AAlternateDirection ? std::max(AForwardBounds.Height, AReverseBounds.Height) : AForwardBounds.Height;
                Layout.Slots.reserve(AArcCount);
                for (std::size_t arcOffset = 0; arcOffset < AArcCount; ++arcOffset) {
                    const int CurrentRow = static_cast<int>(arcOffset / static_cast<std::size_t>(ColumnCount));
                    const int CurrentColumn = static_cast<int>(arcOffset % static_cast<std::size_t>(ColumnCount));
                    const bool ReverseChordDirection = AAlternateDirection && ((CurrentRow + CurrentColumn) % 2 == 1);
                    const TetArcOrientationBounds &SlotBounds = ReverseChordDirection ? AReverseBounds : AForwardBounds;
                    const double SlotX = static_cast<double>(CurrentColumn) * (CellWidth + AGap) + (CellWidth - SlotBounds.Width) * 0.5;
                    const double SlotY = static_cast<double>(CurrentRow) * (CellHeight + AGap) + (CellHeight - SlotBounds.Height) * 0.5;
                    Layout.Slots.push_back({SlotX, SlotY, ReverseChordDirection});
                }
                Layout.Width = static_cast<double>(ColumnCount) * CellWidth + static_cast<double>(ColumnCount - 1) * AGap;
                Layout.Height = static_cast<double>(ARowCount) * CellHeight + static_cast<double>(ARowCount - 1) * AGap;
                Layout.ClusterType = AAlternateDirection ? "ArcAlternatingGrid_" : "ArcGrid_";
                Layout.ClusterType += std::to_string(AArcCount) + "_R" + std::to_string(ARowCount) + "_" + AStyleName;
                return Layout;
            }
            bool TryBuildArcCandidateFromLayout(const TetArcCandidateBuildRequest &ARequest, CetClusterGeometryHelper &AGeometry, TetClusterCandidate &AOutCandidate)
            {
                const auto &AOriginalItems = ARequest.OriginalItems; const auto &AFeatures = ARequest.Features; const auto &AIndices = ARequest.Indices; const auto &AOptions = ARequest.Options; const auto &ABaseInfo = ARequest.BaseInfo; const auto &ALayout = ARequest.Layout;
                AOutCandidate = TetClusterCandidate{};
                if (ALayout.Slots.size() != AIndices.size() || ALayout.Width <= 0.0 || ALayout.Height <= 0.0) {
                    return false;
                }
                TetClusterCandidate Candidate;
                Candidate.BuilderName = "ArcBuilder";
                Candidate.ClusterType = ALayout.ClusterType;
                Candidate.OriginalIndices = AIndices;
                Candidate.Confidence = ABaseInfo.SweepBucket == MetArcSweepBucket::SemiCircle ? 0.78 : 0.72;
                Candidate.Transforms.reserve(AIndices.size());
                for (std::size_t ArcOffset = 0; ArcOffset < AIndices.size(); ++ArcOffset) {
                    const int OriginalIndex = AIndices[ArcOffset];
                    const TetShapeFeature &Feature = AFeatures[OriginalIndex];
                    const TetArcLayoutSlot &Slot = ALayout.Slots[ArcOffset];
                    TetArcOrientationBounds SlotBounds;
                    if (!GetArcOrientationBounds(AOriginalItems[OriginalIndex], Feature, Slot.ReverseChordDirection, AOptions, AGeometry, SlotBounds)) {
                        return false;
                    }
                    TetItemTransform Transform;
                    Transform.OriginalId = OriginalIndex;
                    Transform.RelativeRotation = SlotBounds.Rotation;
                    Transform.RelativeX = Slot.X - SlotBounds.MinX;
                    Transform.RelativeY = Slot.Y - SlotBounds.MinY;
                    Candidate.Transforms.push_back(Transform);
                }
                if (!AGeometry.FinalizeCandidate(AOriginalItems, AOptions, Candidate)) {
                    return false;
                }
                AOutCandidate = std::move(Candidate);
                return true;
            }
        } // namespace
        CetArcClusterBuilder::CetArcClusterBuilder() : CetCoreObject() {}
        CetArcClusterBuilder::~CetArcClusterBuilder() {}
        void CetArcClusterBuilder::BuildCandidates(const CetTNestItemVector &AOriginalItems, const std::vector<TetShapeFeature> &AFeatures, const std::vector<int> &AIndices, const TetNestOptions &AOptions, std::vector<TetClusterCandidate> &AOutCandidates)
        {
            if (AOriginalItems.empty() || AOriginalItems.size() != AFeatures.size() || AIndices.size() < 2) {
                return;
            }
            const std::vector<std::vector<int>> Groups = GroupCompatibleArcIndices(AIndices, AFeatures);
            if (Groups.empty()) {
                return;
            }
            const std::size_t OldCandidateCount = AOutCandidates.size();
            for (const std::vector<int> &Group : Groups) {
                _BuildCompatibleArcClusterCandidates(AOriginalItems, AFeatures, Group, AOptions, AOutCandidates);
            }
            std::cout << "[ARC][BUILD CANDIDATES] GroupCount = " << Groups.size() << ", NewCandidateCount = " << AOutCandidates.size() - OldCandidateCount << std::endl;
        }
        void CetArcClusterBuilder::_BuildCompatibleArcClusterCandidates(const CetTNestItemVector &AOriginalItems, const std::vector<TetShapeFeature> &AFeatures, const std::vector<int> &AIndices, const TetNestOptions &AOptions, std::vector<TetClusterCandidate> &AOutCandidates)
        {
            std::vector<int> RemainingIndices = AIndices;
            std::sort(RemainingIndices.begin(), RemainingIndices.end());
            RemainingIndices.erase(std::unique(RemainingIndices.begin(), RemainingIndices.end()), RemainingIndices.end());
            if (RemainingIndices.size() < 2) {
                return;
            }
            CetClusterGeometryHelper Geometry;
            const std::size_t MaxTrialCount = std::min(RemainingIndices.size(), CET_ARC_MAX_CLUSTER_CHILDREN);
            std::size_t PreferredCount = 0;
            TetClusterCandidate FirstCandidate;
            for (std::size_t TrialCount = MaxTrialCount; TrialCount >= 2; --TrialCount) {
                std::vector<int> TrialIndices(RemainingIndices.begin(), RemainingIndices.begin() + static_cast<std::vector<int>::difference_type>(TrialCount));
                TetClusterCandidate Candidate;
                if (!_BuildClusterCandidate(AOriginalItems, AFeatures, TrialIndices, AOptions, Candidate)) {
                    continue;
                }
                const std::size_t RequiredCopies = std::min(CET_CLUSTER_TARGET_COPIES_PER_BOARD, RemainingIndices.size() / TrialCount);
                if (!Geometry.CanPlaceCandidateCopiesOnBoard(Candidate, AOptions, RequiredCopies)) {
                    continue;
                }
                PreferredCount = TrialCount;
                FirstCandidate = std::move(Candidate);
                break;
            }
            if (PreferredCount < 2) {
                std::cout << "[ARC][REJECT] No practical board-fitting compatible cluster can be built. Count = " << RemainingIndices.size() << std::endl;
                return;
            }
            std::size_t GroupOffset = 0;
            while (GroupOffset + 1 < RemainingIndices.size()) {
                const std::size_t RemainingCount = RemainingIndices.size() - GroupOffset;
                std::size_t TrialCount = std::min(RemainingCount, PreferredCount);
                std::size_t BestCount = 0;
                TetClusterCandidate BestCandidate;
                while (TrialCount >= 2) {
                    std::vector<int> TrialIndices(RemainingIndices.begin() + static_cast<std::vector<int>::difference_type>(GroupOffset), RemainingIndices.begin() + static_cast<std::vector<int>::difference_type>(GroupOffset + TrialCount));
                    if (GroupOffset == 0 && TrialCount == PreferredCount) {
                        BestCandidate = std::move(FirstCandidate);
                        BestCount = TrialCount;
                        break;
                    }
                    TetClusterCandidate Candidate;
                    if (_BuildClusterCandidate(AOriginalItems, AFeatures, TrialIndices, AOptions, Candidate)) {
                        BestCandidate = std::move(Candidate);
                        BestCount = TrialCount;
                        break;
                    }
                    --TrialCount;
                }
                if (BestCount < 2) {
                    std::cout << "[ARC][REJECT] No board-fitting compatible cluster can be built. RemainingCount = " << RemainingCount << std::endl;
                    return;
                }
                AOutCandidates.push_back(std::move(BestCandidate));
                std::cout << "[ARC][CANDIDATE] Size = " << BestCount << ", Type = " << AOutCandidates.back().ClusterType << ", Score = " << AOutCandidates.back().Score << std::endl;
                GroupOffset += BestCount;
            }
        }
        bool CetArcClusterBuilder::_BuildClusterCandidate(const CetTNestItemVector &AOriginalItems, const std::vector<TetShapeFeature> &AFeatures, const std::vector<int> &AIndices, const TetNestOptions &AOptions, TetClusterCandidate &AOutCandidate)
        {
            AOutCandidate = TetClusterCandidate{};
            if (AOriginalItems.size() != AFeatures.size() || AIndices.size() < 2 || AIndices.size() > CET_ARC_MAX_CLUSTER_CHILDREN) {
                return false;
            }
            std::vector<int> Indices = AIndices;
            std::sort(Indices.begin(), Indices.end());
            Indices.erase(std::unique(Indices.begin(), Indices.end()), Indices.end());
            if (Indices.size() < 2) {
                return false;
            }
            for (int OriginalIndex : Indices) {
                if (OriginalIndex < 0 || OriginalIndex >= static_cast<int>(AFeatures.size())) {
                    return false;
                }
            }
            const TetShapeFeature &BaseFeature = AFeatures[Indices.front()];
            if (!IsValidArcFeature(BaseFeature)) {
                return false;
            }
            const TetArcIndexInfo BaseInfo = MakeArcIndexInfo(Indices.front(), BaseFeature);
            for (int OriginalIndex : Indices) {
                if (!IsValidArcFeature(AFeatures[OriginalIndex])) {
                    return false;
                }
                if (!AreCompatibleArcInfos(BaseInfo, MakeArcIndexInfo(OriginalIndex, AFeatures[OriginalIndex]))) {
                    return false;
                }
            }
            CetClusterGeometryHelper Geometry;
            TetArcOrientationBounds ForwardBounds;
            TetArcOrientationBounds ReverseBounds;
            if (!GetArcOrientationBounds(AOriginalItems[Indices.front()], BaseFeature, false, AOptions, Geometry, ForwardBounds) || !GetArcOrientationBounds(AOriginalItems[Indices.front()], BaseFeature, true, AOptions, Geometry, ReverseBounds)) {
                return false;
            }
            const double RequiredGap = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));
            const double SafetyGap = RequiredGap > 0.0 ? std::max(CET_CLUSTER_MIN_SAFETY_GAP, RequiredGap * CET_ARC_SAFETY_GAP_RATIO) : 0.0;
            const double Gap = RequiredGap + SafetyGap;
            const std::string StyleName = GetSweepBucketName(BaseInfo.SweepBucket);
            std::vector<TetArcLayout> Layouts;
            Layouts.push_back(MakeLineLayout({Indices.size(), 0, ForwardBounds, ReverseBounds, Gap, false, false, StyleName}));
            Layouts.push_back(MakeLineLayout({Indices.size(), 0, ForwardBounds, ReverseBounds, Gap, true, false, StyleName}));
            Layouts.push_back(MakeLineLayout({Indices.size(), 0, ForwardBounds, ReverseBounds, Gap, false, true, StyleName}));
            Layouts.push_back(MakeLineLayout({Indices.size(), 0, ForwardBounds, ReverseBounds, Gap, true, true, StyleName}));
            const int MaxGridRowCount = static_cast<int>((Indices.size() + 1) / 2);
            for (int RowCount = 2; RowCount <= MaxGridRowCount; ++RowCount) {
                Layouts.push_back(MakeGridLayout({Indices.size(), RowCount, ForwardBounds, ReverseBounds, Gap, false, false, StyleName}));
                Layouts.push_back(MakeGridLayout({Indices.size(), RowCount, ForwardBounds, ReverseBounds, Gap, false, true, StyleName}));
            }
            bool HasBestCandidate = false;
            TetClusterCandidate BestCandidate;
            for (const TetArcLayout &Layout : Layouts) {
                if (!Nest2DUtils->Nest2DBord->FitsBin(Layout.Width, Layout.Height, AOptions)) {
                    continue;
                }
                TetClusterCandidate Candidate;
                if (!TryBuildArcCandidateFromLayout({AOriginalItems, AFeatures, Indices, AOptions, BaseInfo, Layout}, Geometry, Candidate)) {
                    continue;
                }
                Candidate.Score = _CalculateScore(Candidate, AOptions);
                if (!HasBestCandidate || Candidate.Score > BestCandidate.Score) {
                    HasBestCandidate = true;
                    BestCandidate = std::move(Candidate);
                }
            }
            if (!HasBestCandidate) {
                return false;
            }
            AOutCandidate = std::move(BestCandidate);
            return true;
        }
        double CetArcClusterBuilder::_CalculateScore(const TetClusterCandidate &ACandidate, const TetNestOptions &AOptions)
        {
            if (!ACandidate.Valid || ACandidate.OriginalIndices.empty() || ACandidate.ClusterWidth <= 0.0 || ACandidate.ClusterHeight <= 0.0 || ACandidate.ProxyArea <= 0.0) {
                return -std::numeric_limits<double>::infinity();
            }
            const double FillScore = ACandidate.FillRatio * 1000.0;
            const double SavingScore = ACandidate.AreaSavingRatio * 850.0;
            const double ItemCountScore = static_cast<double>(ACandidate.OriginalIndices.size()) * 48.0;
            const double LongSide = std::max(ACandidate.ClusterWidth, ACandidate.ClusterHeight);
            const double ShortSide = std::min(ACandidate.ClusterWidth, ACandidate.ClusterHeight);
            const double CompactRatio = LongSide > 0.0 ? ShortSide / LongSide : 0.0;
            const double CompactScore = CompactRatio * 25.0;
            double LayoutBonus = 0.0;
            if (ACandidate.ClusterType == "SemiCirclePair") {
                LayoutBonus = 80.0;
            } else if (ACandidate.ClusterType.find("AlternatingGrid") != std::string::npos) {
                LayoutBonus = 95.0;
            } else if (ACandidate.ClusterType.find("Grid") != std::string::npos) {
                LayoutBonus = 70.0;
            } else if (ACandidate.ClusterType.find("AlternatingLine") != std::string::npos) {
                LayoutBonus = 45.0;
            } else if (ACandidate.ClusterType.find("Column") != std::string::npos) {
                LayoutBonus = ACandidate.OriginalIndices.size() >= 4 ? -80.0 : 10.0;
            }
            const double BinWidth = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth));
            const double HorizontalSpreadRatio = BinWidth > 0.0 ? std::clamp(ACandidate.ClusterWidth / BinWidth, 0.0, 1.0) : 0.0;
            const double HorizontalSpreadScore = HorizontalSpreadRatio * 35.0;
            const double PerimeterPenalty = (ACandidate.ClusterWidth + ACandidate.ClusterHeight) * 0.000001;
            return FillScore + SavingScore + ItemCountScore + CompactScore + LayoutBonus + HorizontalSpreadScore + ACandidate.Confidence - PerimeterPenalty;
        }
    } // namespace NEST2DMANAGERLIB
} // namespace ET
