#include "pch.h"
#include "Nest2D_ArcClusterBuilder.h"
#include "Nest2D_ClusterGeometryHelper.h"
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
            constexpr double CET_ARC_SIZE_TOLERANCE = 0.05;
            constexpr double CET_ARC_SWEEP_TOLERANCE = CET_CLUSTER_PI / 36.0;
            constexpr double CET_ARC_SAFETY_GAP_RATIO = 0.05;

            bool NearlyEqual(double FirstValue, double SecondValue, double RelativeTolerance)
            {
                const double Denominator = std::max(1.0, std::max(std::abs(FirstValue), std::abs(SecondValue)));
                return std::abs(FirstValue - SecondValue) <= Denominator * RelativeTolerance;
            }

            int NormalizeBulgeSign(int BulgeSign)
            {
                return BulgeSign < 0 ? -1 : 1;
            }

            double GetEffectiveSweepAngle(const TetShapeFeature& Feature)
            {
                if (std::isfinite(Feature.ArcSweepAngle) && Feature.ArcSweepAngle > 0.0) {
                    return std::abs(Feature.ArcSweepAngle);
                }

                return Feature.ArcType == MetArcType::SemiCircleLike ? CET_CLUSTER_PI : 0.0;
            }

            MetArcSweepBucket GetSweepBucket(const TetShapeFeature& Feature)
            {
                const double SweepAngle = GetEffectiveSweepAngle(Feature);
                if (SweepAngle <= 0.0) {
                    return MetArcSweepBucket::Unknown;
                }

                if (std::abs(SweepAngle - CET_CLUSTER_PI) <= CET_ARC_SWEEP_TOLERANCE) {
                    return MetArcSweepBucket::SemiCircle;
                }

                return SweepAngle < CET_CLUSTER_PI
                    ? MetArcSweepBucket::LessThanSemiCircle
                    : MetArcSweepBucket::MoreThanSemiCircle;
            }

            const char* GetSweepBucketName(MetArcSweepBucket SweepBucket)
            {
                switch (SweepBucket) {
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

            bool IsValidArcFeature(const TetShapeFeature& Feature)
            {
                return Feature.ShapeType == MetShapeType::ArcLike &&
                    Feature.ArcType != MetArcType::None &&
                    GetSweepBucket(Feature) != MetArcSweepBucket::Unknown &&
                    Feature.ArcRadius > 0.0 &&
                    Feature.ArcChordLength > 0.0 &&
                    Feature.Width > 0.0 &&
                    Feature.Height > 0.0 &&
                    Feature.Area > 0.0;
            }

            TetArcIndexInfo MakeArcIndexInfo(int OriginalIndex, const TetShapeFeature& Feature)
            {
                TetArcIndexInfo Info;
                Info.Index = OriginalIndex;
                Info.ArcType = Feature.ArcType;
                Info.SweepBucket = GetSweepBucket(Feature);
                Info.Radius = Feature.ArcRadius;
                Info.ChordLength = Feature.ArcChordLength;
                Info.SweepAngle = GetEffectiveSweepAngle(Feature);
                Info.BulgeSign = NormalizeBulgeSign(Feature.ArcBulgeSign);
                return Info;
            }

            bool AreCompatibleArcInfos(const TetArcIndexInfo& BaseInfo, const TetArcIndexInfo& TestInfo)
            {
                if (BaseInfo.ArcType != TestInfo.ArcType ||
                    BaseInfo.SweepBucket != TestInfo.SweepBucket ||
                    BaseInfo.BulgeSign != TestInfo.BulgeSign)
                {
                    return false;
                }

                const bool RadiusMatches = NearlyEqual(BaseInfo.Radius, TestInfo.Radius, CET_ARC_SIZE_TOLERANCE);
                const bool ChordMatches = NearlyEqual(BaseInfo.ChordLength, TestInfo.ChordLength, CET_ARC_SIZE_TOLERANCE);
                const bool SweepMatches = std::abs(BaseInfo.SweepAngle - TestInfo.SweepAngle) <= CET_ARC_SWEEP_TOLERANCE ||
                    NearlyEqual(BaseInfo.SweepAngle, TestInfo.SweepAngle, CET_ARC_SIZE_TOLERANCE);
                return RadiusMatches && ChordMatches && SweepMatches;
            }

            std::vector<std::vector<int>> GroupCompatibleArcIndices(const std::vector<int>& Indices, const std::vector<TetShapeFeature>& Features)
            {
                std::vector<TetArcIndexInfo> Infos;
                Infos.reserve(Indices.size());

                for (int OriginalIndex : Indices) {
                    if (OriginalIndex < 0 || OriginalIndex >= static_cast<int>(Features.size())) {
                        continue;
                    }

                    const TetShapeFeature& Feature = Features[OriginalIndex];
                    if (!IsValidArcFeature(Feature)) {
                        continue;
                    }

                    Infos.push_back(MakeArcIndexInfo(OriginalIndex, Feature));
                }

                std::sort(
                    Infos.begin(),
                    Infos.end(),
                    [](const TetArcIndexInfo& FirstInfo, const TetArcIndexInfo& SecondInfo)
                    {
                        if (FirstInfo.ArcType != SecondInfo.ArcType) {
                            return static_cast<int>(FirstInfo.ArcType) < static_cast<int>(SecondInfo.ArcType);
                        }
                        if (FirstInfo.SweepBucket != SecondInfo.SweepBucket) {
                            return static_cast<int>(FirstInfo.SweepBucket) < static_cast<int>(SecondInfo.SweepBucket);
                        }
                        if (FirstInfo.BulgeSign != SecondInfo.BulgeSign) {
                            return FirstInfo.BulgeSign < SecondInfo.BulgeSign;
                        }
                        if (std::abs(FirstInfo.Radius - SecondInfo.Radius) > 1.0) {
                            return FirstInfo.Radius < SecondInfo.Radius;
                        }
                        if (std::abs(FirstInfo.ChordLength - SecondInfo.ChordLength) > 1.0) {
                            return FirstInfo.ChordLength < SecondInfo.ChordLength;
                        }
                        if (std::abs(FirstInfo.SweepAngle - SecondInfo.SweepAngle) > 1e-9) {
                            return FirstInfo.SweepAngle < SecondInfo.SweepAngle;
                        }
                        return FirstInfo.Index < SecondInfo.Index;
                    });

                Infos.erase(
                    std::unique(
                        Infos.begin(),
                        Infos.end(),
                        [](const TetArcIndexInfo& FirstInfo, const TetArcIndexInfo& SecondInfo)
                        {
                            return FirstInfo.Index == SecondInfo.Index;
                        }),
                    Infos.end());

                std::vector<std::vector<int>> Groups;
                std::vector<int> CurrentGroup;
                TetArcIndexInfo CurrentBaseInfo;

                for (const TetArcIndexInfo& Info : Infos) {
                    if (CurrentGroup.empty()) {
                        CurrentBaseInfo = Info;
                        CurrentGroup.push_back(Info.Index);
                        continue;
                    }

                    if (AreCompatibleArcInfos(CurrentBaseInfo, Info)) {
                        CurrentGroup.push_back(Info.Index);
                    }
                    else {
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

            bool FitsBin(double ClusterWidth, double ClusterHeight, const TetNestOptions& Options)
            {
                if (ClusterWidth <= 0.0 || ClusterHeight <= 0.0) {
                    return false;
                }

                const double BinWidth = static_cast<double>(NestUtils::ToNestCoord(Options.BinWidth));
                const double BinHeight = static_cast<double>(NestUtils::ToNestCoord(Options.BinHeight));
                if (BinWidth <= 0.0 || BinHeight <= 0.0) {
                    return false;
                }

                const bool FitsNormally = ClusterWidth <= BinWidth && ClusterHeight <= BinHeight;
                const bool QuarterTurnAllowed = CetRotationUtils::IsAllowedRotation(CET_CLUSTER_HALF_PI, Options.Rotations, 1e-9);
                const bool FitsAfterRotation = QuarterTurnAllowed && ClusterHeight <= BinWidth && ClusterWidth <= BinHeight;
                return FitsNormally || FitsAfterRotation;
            }

            bool GetArcOrientationBounds(const CetNestItem& Item, const TetShapeFeature& Feature, bool ReverseChordDirection, const TetNestOptions& Options, const CetClusterGeometryHelper& Geometry, TetArcOrientationBounds& OutBounds)
            {
                OutBounds = TetArcOrientationBounds{};
                const double TargetRotation = ReverseChordDirection? CET_CLUSTER_PI - Feature.ArcChordAngle: -Feature.ArcChordAngle;
                if (!CetRotationUtils::SnapToNearestAllowedRotation(TargetRotation, Options.Rotations, OutBounds.Rotation)) {
                    return false;
                }

                const CetPath RotatedContour = Geometry.TransformContour(
                    Geometry.GetIdentityContour(Item),
                    OutBounds.Rotation,
                    0.0,
                    0.0);

                double MaxX = 0.0;
                double MaxY = 0.0;
                if (!Geometry.GetBounds(RotatedContour, OutBounds.MinX, OutBounds.MinY, MaxX, MaxY)) {
                    return false;
                }

                OutBounds.Width = MaxX - OutBounds.MinX;
                OutBounds.Height = MaxY - OutBounds.MinY;
                return OutBounds.Width > 0.0 && OutBounds.Height > 0.0;
            }

            TetArcLayout MakeLineLayout(std::size_t ArcCount, const TetArcOrientationBounds& ForwardBounds, const TetArcOrientationBounds& ReverseBounds, double Gap, bool VerticalStack, bool AlternateDirection, const std::string& StyleName)
            {
                TetArcLayout Layout;
                if (ArcCount < 2) {
                    return Layout;
                }

                Layout.Slots.reserve(ArcCount);
                if (VerticalStack) {
                    double CurrentY = 0.0;
                    double MaxWidth = 0.0;
                    for (std::size_t arcOffset = 0; arcOffset < ArcCount; ++arcOffset) {
                        const bool ReverseChordDirection = AlternateDirection && (arcOffset % 2 == 1);
                        const TetArcOrientationBounds& SlotBounds = ReverseChordDirection ? ReverseBounds : ForwardBounds;
                        Layout.Slots.push_back({ 0.0, CurrentY, ReverseChordDirection });
                        CurrentY += SlotBounds.Height + Gap;
                        MaxWidth = std::max(MaxWidth, SlotBounds.Width);
                    }

                    for (TetArcLayoutSlot& Slot : Layout.Slots) {
                        const TetArcOrientationBounds& SlotBounds = Slot.ReverseChordDirection ? ReverseBounds : ForwardBounds;
                        Slot.X = (MaxWidth - SlotBounds.Width) * 0.5;
                    }

                    Layout.Width = MaxWidth;
                    Layout.Height = CurrentY - Gap;
                    Layout.ClusterType = ArcCount == 2 && StyleName == "SemiCircle" && AlternateDirection
                        ? "SemiCirclePair"
                        : "ArcColumn_" + std::to_string(ArcCount) + "_" + StyleName;
                    return Layout;
                }

                double CurrentX = 0.0;
                double MaxHeight = 0.0;
                for (std::size_t arcOffset = 0; arcOffset < ArcCount; ++arcOffset) {
                    const bool ReverseChordDirection = AlternateDirection && (arcOffset % 2 == 1);
                    const TetArcOrientationBounds& SlotBounds = ReverseChordDirection ? ReverseBounds : ForwardBounds;
                    Layout.Slots.push_back({ CurrentX, 0.0, ReverseChordDirection });
                    CurrentX += SlotBounds.Width + Gap;
                    MaxHeight = std::max(MaxHeight, SlotBounds.Height);
                }

                for (TetArcLayoutSlot& Slot : Layout.Slots) {
                    const TetArcOrientationBounds& SlotBounds = Slot.ReverseChordDirection ? ReverseBounds : ForwardBounds;
                    Slot.Y = (MaxHeight - SlotBounds.Height) * 0.5;
                }

                Layout.Width = CurrentX - Gap;
                Layout.Height = MaxHeight;
                Layout.ClusterType = AlternateDirection ? "ArcAlternatingLine_" : "ArcLine_";
                Layout.ClusterType += std::to_string(ArcCount) + "_" + StyleName;
                return Layout;
            }

            TetArcLayout MakeGridLayout(std::size_t ArcCount, int RowCount, const TetArcOrientationBounds& ForwardBounds, const TetArcOrientationBounds& ReverseBounds, double Gap, bool AlternateDirection, const std::string& StyleName)
            {
                TetArcLayout Layout;
                if (ArcCount < 3 || RowCount < 2 || RowCount > static_cast<int>(ArcCount)) {
                    return Layout;
                }

                const int ColumnCount = static_cast<int>((ArcCount + static_cast<std::size_t>(RowCount) - 1) / static_cast<std::size_t>(RowCount));
                const double CellWidth = AlternateDirection ? std::max(ForwardBounds.Width, ReverseBounds.Width) : ForwardBounds.Width;
                const double CellHeight = AlternateDirection ? std::max(ForwardBounds.Height, ReverseBounds.Height) : ForwardBounds.Height;

                Layout.Slots.reserve(ArcCount);
                for (std::size_t arcOffset = 0; arcOffset < ArcCount; ++arcOffset) {
                    const int CurrentRow = static_cast<int>(arcOffset / static_cast<std::size_t>(ColumnCount));
                    const int CurrentColumn = static_cast<int>(arcOffset % static_cast<std::size_t>(ColumnCount));
                    const bool ReverseChordDirection = AlternateDirection && ((CurrentRow + CurrentColumn) % 2 == 1);
                    const TetArcOrientationBounds& SlotBounds = ReverseChordDirection ? ReverseBounds : ForwardBounds;
                    const double SlotX = static_cast<double>(CurrentColumn) * (CellWidth + Gap) + (CellWidth - SlotBounds.Width) * 0.5;
                    const double SlotY = static_cast<double>(CurrentRow) * (CellHeight + Gap) + (CellHeight - SlotBounds.Height) * 0.5;
                    Layout.Slots.push_back({ SlotX, SlotY, ReverseChordDirection });
                }

                Layout.Width = static_cast<double>(ColumnCount) * CellWidth + static_cast<double>(ColumnCount - 1) * Gap;
                Layout.Height = static_cast<double>(RowCount) * CellHeight + static_cast<double>(RowCount - 1) * Gap;
                Layout.ClusterType = AlternateDirection ? "ArcAlternatingGrid_" : "ArcGrid_";
                Layout.ClusterType += std::to_string(ArcCount) + "_R" + std::to_string(RowCount) + "_" + StyleName;
                return Layout;
            }

            bool TryBuildArcCandidateFromLayout(
                const CetTNestItemVector& AOriginalItems,
                const std::vector<TetShapeFeature>& AFeatures,
                const std::vector<int>& AIndices,
                const TetNestOptions& AOptions,
                const TetArcIndexInfo& BaseInfo,
                const TetArcLayout& Layout,
                CetClusterGeometryHelper& Geometry,
                TetClusterCandidate& AOutCandidate)
            {
                AOutCandidate = TetClusterCandidate{};

                if (Layout.Slots.size() != AIndices.size() || Layout.Width <= 0.0 || Layout.Height <= 0.0) {
                    return false;
                }

                TetClusterCandidate Candidate;
                Candidate.BuilderName = "ArcBuilder";
                Candidate.ClusterType = Layout.ClusterType;
                Candidate.OriginalIndices = AIndices;
                Candidate.Confidence = BaseInfo.SweepBucket == MetArcSweepBucket::SemiCircle ? 0.78 : 0.72;
                Candidate.Transforms.reserve(AIndices.size());

                for (std::size_t ArcOffset = 0; ArcOffset < AIndices.size(); ++ArcOffset) {
                    const int OriginalIndex = AIndices[ArcOffset];
                    const TetShapeFeature& Feature = AFeatures[OriginalIndex];
                    const TetArcLayoutSlot& Slot = Layout.Slots[ArcOffset];

                    TetArcOrientationBounds SlotBounds;
                    if (!GetArcOrientationBounds(AOriginalItems[OriginalIndex], Feature, Slot.ReverseChordDirection, AOptions, Geometry, SlotBounds)) {
                        return false;
                    }

                    TetItemTransform Transform;
                    Transform.OriginalId = OriginalIndex;
                    Transform.RelativeRotation = SlotBounds.Rotation;
                    Transform.RelativeX = Slot.X - SlotBounds.MinX;
                    Transform.RelativeY = Slot.Y - SlotBounds.MinY;
                    Candidate.Transforms.push_back(Transform);
                }

                if (!Geometry.FinalizeCandidate(AOriginalItems, AOptions, Candidate)) {
                    return false;
                }

                AOutCandidate = std::move(Candidate);
                return true;
            }
        }

        CetArcClusterBuilder::CetArcClusterBuilder() : CetCoreObject() {}
        CetArcClusterBuilder::~CetArcClusterBuilder() {}

        void CetArcClusterBuilder::BuildCandidates(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, std::vector<TetClusterCandidate>& AOutCandidates)
        {
            if (AOriginalItems.empty() || AOriginalItems.size() != AFeatures.size() || AIndices.size() < 2) {
                return;
            }

            const std::vector<std::vector<int>> Groups = GroupCompatibleArcIndices(AIndices, AFeatures);
            if (Groups.empty()) {
                return;
            }

            const std::size_t OldCandidateCount = AOutCandidates.size();
            for (const std::vector<int>& Group : Groups) {
                _BuildCompatibleArcClusterCandidates(AOriginalItems, AFeatures, Group, AOptions, AOutCandidates);
            }

            std::cout << "[ARC][BUILD CANDIDATES] GroupCount = " << Groups.size()
                << ", NewCandidateCount = " << AOutCandidates.size() - OldCandidateCount << std::endl;
        }

        void CetArcClusterBuilder::_BuildCompatibleArcClusterCandidates(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, std::vector<TetClusterCandidate>& AOutCandidates)
        {
            std::vector<int> RemainingIndices = AIndices;
            std::sort(RemainingIndices.begin(), RemainingIndices.end());
            RemainingIndices.erase(std::unique(RemainingIndices.begin(), RemainingIndices.end()), RemainingIndices.end());

            while (RemainingIndices.size() >= 2) {
                std::size_t BestCount = 0;
                TetClusterCandidate BestCandidate;

                for (std::size_t TrialCount = RemainingIndices.size(); TrialCount >= 2; --TrialCount) {
                    std::vector<int> TrialIndices(
                        RemainingIndices.begin(),
                        RemainingIndices.begin() + static_cast<std::vector<int>::difference_type>(TrialCount));

                    TetClusterCandidate TrialCandidate;
                    if (_BuildClusterCandidate(AOriginalItems, AFeatures, TrialIndices, AOptions, TrialCandidate)) {
                        BestCount = TrialCount;
                        BestCandidate = std::move(TrialCandidate);
                        break;
                    }
                }

                if (BestCount < 2) {
                    std::cout << "[ARC][REJECT] No board-fitting compatible cluster can be built. RemainingCount = "
                        << RemainingIndices.size() << std::endl;
                    return;
                }

                AOutCandidates.push_back(std::move(BestCandidate));
                std::cout << "[ARC][CANDIDATE] Size = " << BestCount
                    << ", Type = " << AOutCandidates.back().ClusterType
                    << ", Score = " << AOutCandidates.back().Score << std::endl;

                RemainingIndices.erase(
                    RemainingIndices.begin(),
                    RemainingIndices.begin() + static_cast<std::vector<int>::difference_type>(BestCount));
            }
        }

        bool CetArcClusterBuilder::_BuildClusterCandidate(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate)
        {
            AOutCandidate = TetClusterCandidate{};

            if (AOriginalItems.size() != AFeatures.size() || AIndices.size() < 2) {
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

            const TetShapeFeature& BaseFeature = AFeatures[Indices.front()];
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
            if (!GetArcOrientationBounds(AOriginalItems[Indices.front()], BaseFeature, false, AOptions, Geometry, ForwardBounds) ||
                !GetArcOrientationBounds(AOriginalItems[Indices.front()], BaseFeature, true, AOptions, Geometry, ReverseBounds))
            {
                return false;
            }

            const double RequiredGap = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));
            const double SafetyGap = RequiredGap > 0.0 ? std::max(10.0, RequiredGap * CET_ARC_SAFETY_GAP_RATIO) : 0.0;
            const double Gap = RequiredGap + SafetyGap;
            const std::string StyleName = GetSweepBucketName(BaseInfo.SweepBucket);

            std::vector<TetArcLayout> Layouts;
            Layouts.push_back(MakeLineLayout(Indices.size(), ForwardBounds, ReverseBounds, Gap, false, false, StyleName));
            Layouts.push_back(MakeLineLayout(Indices.size(), ForwardBounds, ReverseBounds, Gap, true, false, StyleName));
            Layouts.push_back(MakeLineLayout(Indices.size(), ForwardBounds, ReverseBounds, Gap, false, true, StyleName));
            Layouts.push_back(MakeLineLayout(Indices.size(), ForwardBounds, ReverseBounds, Gap, true, true, StyleName));

            for (int RowCount = 2; RowCount <= static_cast<int>(Indices.size()); ++RowCount) {
                Layouts.push_back(MakeGridLayout(Indices.size(), RowCount, ForwardBounds, ReverseBounds, Gap, false, StyleName));
                Layouts.push_back(MakeGridLayout(Indices.size(), RowCount, ForwardBounds, ReverseBounds, Gap, true, StyleName));
            }

            bool HasBestCandidate = false;
            TetClusterCandidate BestCandidate;

            for (const TetArcLayout& Layout : Layouts) {
                if (!_FitsBin(Layout.Width, Layout.Height, AOptions)) {
                    continue;
                }

                TetClusterCandidate Candidate;
                if (!TryBuildArcCandidateFromLayout(AOriginalItems, AFeatures, Indices, AOptions, BaseInfo, Layout, Geometry, Candidate)) {
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

        bool CetArcClusterBuilder::_FitsBin(double AClusterWidth, double AClusterHeight, const TetNestOptions& AOptions)
        {
            return FitsBin(AClusterWidth, AClusterHeight, AOptions);
        }

        double CetArcClusterBuilder::_CalculateScore(const TetClusterCandidate& ACandidate, const TetNestOptions& AOptions)
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
            }
            else if (ACandidate.ClusterType.find("AlternatingGrid") != std::string::npos) {
                LayoutBonus = 95.0;
            }
            else if (ACandidate.ClusterType.find("Grid") != std::string::npos) {
                LayoutBonus = 70.0;
            }
            else if (ACandidate.ClusterType.find("AlternatingLine") != std::string::npos) {
                LayoutBonus = 45.0;
            }
            else if (ACandidate.ClusterType.find("Column") != std::string::npos) {
                LayoutBonus = ACandidate.OriginalIndices.size() >= 4 ? -80.0 : 10.0;
            }

            const double BinWidth = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth));
            const double HorizontalSpreadRatio = BinWidth > 0.0 ? std::clamp(ACandidate.ClusterWidth / BinWidth, 0.0, 1.0) : 0.0;
            const double HorizontalSpreadScore = HorizontalSpreadRatio * 35.0;
            const double PerimeterPenalty = (ACandidate.ClusterWidth + ACandidate.ClusterHeight) * 0.000001;
            return FillScore + SavingScore + ItemCountScore + CompactScore + LayoutBonus + HorizontalSpreadScore + ACandidate.Confidence - PerimeterPenalty;
        }

    }
}
