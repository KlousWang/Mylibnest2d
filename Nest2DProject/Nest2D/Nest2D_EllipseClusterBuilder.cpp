#include "pch.h"
#include "Nest2D_EllipseClusterBuilder.h"
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
            constexpr double CET_ELLIPSE_SIZE_TOLERANCE = 0.05;
            constexpr double CET_ELLIPSE_HONEYCOMB_ROW_RATIO = 0.90;

            bool NearlyEqual(double A, double B, double RelativeTolerance)
            {
                const double Denominator = std::max(1.0, std::max(std::abs(A), std::abs(B)));
                return std::abs(A - B) <= Denominator * RelativeTolerance;
            }

            bool IsValidEllipseFeature(const TetShapeFeature& Feature)
            {
                return Feature.ShapeType == MetShapeType::EllipseLike &&
                    Feature.EllipseMajorAxis > 0.0 &&
                    Feature.EllipseMinorAxis > 0.0 &&
                    Feature.Width > 0.0 &&
                    Feature.Height > 0.0 &&
                    Feature.Area > 0.0;
            }

            bool AreSameSizeEllipses(const TetShapeFeature& A, const TetShapeFeature& B)
            {
                return IsValidEllipseFeature(A) &&
                    IsValidEllipseFeature(B) &&
                    NearlyEqual(A.EllipseMajorAxis, B.EllipseMajorAxis, CET_ELLIPSE_SIZE_TOLERANCE) &&
                    NearlyEqual(A.EllipseMinorAxis, B.EllipseMinorAxis, CET_ELLIPSE_SIZE_TOLERANCE);
            }

            bool FitsBin(double Width, double Height, const TetNestOptions& Options)
            {
                if (Width <= 0.0 || Height <= 0.0) {
                    return false;
                }

                const double BinWidth = static_cast<double>(NestUtils::ToNestCoord(Options.BinWidth));
                const double BinHeight = static_cast<double>(NestUtils::ToNestCoord(Options.BinHeight));
                if (BinWidth <= 0.0 || BinHeight <= 0.0) {
                    return false;
                }

                const bool Normal = Width <= BinWidth && Height <= BinHeight;
                const bool QuarterTurnAllowed = CetRotationUtils::IsAllowedRotation(CET_CLUSTER_HALF_PI, Options.Rotations, 1e-9);
                const bool Rotated = QuarterTurnAllowed && Height <= BinWidth && Width <= BinHeight;
                return Normal || Rotated;
            }

            std::vector<std::vector<int>> GroupEllipseIndices(const std::vector<int>& Indices, const std::vector<TetShapeFeature>& Features)
            {
                std::vector<TetEllipseIndexInfo> Infos;
                Infos.reserve(Indices.size());

                for (int Index : Indices) {
                    if (Index < 0 || Index >= static_cast<int>(Features.size())) {
                        continue;
                    }

                    const TetShapeFeature& Feature = Features[Index];
                    if (!IsValidEllipseFeature(Feature)) {
                        continue;
                    }

                    Infos.push_back({ Index, Feature.EllipseMajorAxis, Feature.EllipseMinorAxis });
                }

                std::sort(
                    Infos.begin(),
                    Infos.end(),
                    [](const TetEllipseIndexInfo& A, const TetEllipseIndexInfo& B)
                    {
                        if (A.MajorAxis != B.MajorAxis) {
                            return A.MajorAxis < B.MajorAxis;
                        }
                        if (A.MinorAxis != B.MinorAxis) {
                            return A.MinorAxis < B.MinorAxis;
                        }
                        return A.Index < B.Index;
                    });

                Infos.erase(
                    std::unique(
                        Infos.begin(),
                        Infos.end(),
                        [](const TetEllipseIndexInfo& A, const TetEllipseIndexInfo& B)
                        {
                            return A.Index == B.Index;
                        }),
                    Infos.end());

                std::vector<std::vector<int>> Groups;
                std::vector<int> CurrentGroup;
                TetEllipseIndexInfo BaseInfo;

                for (const TetEllipseIndexInfo& Info : Infos) {
                    if (CurrentGroup.empty()) {
                        BaseInfo = Info;
                        CurrentGroup.push_back(Info.Index);
                        continue;
                    }

                    if (NearlyEqual(BaseInfo.MajorAxis, Info.MajorAxis, CET_ELLIPSE_SIZE_TOLERANCE) &&
                        NearlyEqual(BaseInfo.MinorAxis, Info.MinorAxis, CET_ELLIPSE_SIZE_TOLERANCE))
                    {
                        CurrentGroup.push_back(Info.Index);
                    }
                    else {
                        Groups.push_back(std::move(CurrentGroup));
                        CurrentGroup.clear();
                        BaseInfo = Info;
                        CurrentGroup.push_back(Info.Index);
                    }
                }

                if (!CurrentGroup.empty()) {
                    Groups.push_back(std::move(CurrentGroup));
                }

                return Groups;
            }

            bool GetOrientedBounds(const CetNestItem& Item, const TetShapeFeature& Feature, bool RotateToVertical, const TetNestOptions& Options, const CetClusterGeometryHelper& Geometry, double& OutRotation, double& OutMinX, double& OutMinY, double& OutWidth, double& OutHeight)
            {
                const double TargetAngle = RotateToVertical ? CET_CLUSTER_HALF_PI : 0.0;
                if (!CetRotationUtils::SnapToNearestAllowedRotation(TargetAngle - Feature.EllipseAngle, Options.Rotations, OutRotation)) {
                    return false;
                }

                const CetPath Contour = Geometry.TransformContour(Geometry.GetIdentityContour(Item), OutRotation, 0.0, 0.0);
                double MaxX = 0.0;
                double MaxY = 0.0;
                if (!Geometry.GetBounds(Contour, OutMinX, OutMinY, MaxX, MaxY)) {
                    return false;
                }

                OutWidth = MaxX - OutMinX;
                OutHeight = MaxY - OutMinY;
                return OutWidth > 0.0 && OutHeight > 0.0;
            }

            TetEllipseLayout MakeLineLayout(std::size_t Count, double HorizontalWidth, double HorizontalHeight, double VerticalWidth, double VerticalHeight, double Gap, bool VerticalStack, bool AlternateRotation)
            {
                TetEllipseLayout Layout;
                if (Count < 2) {
                    return Layout;
                }

                Layout.Slots.reserve(Count);
                if (VerticalStack) {
                    double Y = 0.0;
                    for (std::size_t I = 0; I < Count; ++I) {
                        Layout.Slots.push_back({ 0.0, Y, false });
                        Y += HorizontalHeight + Gap;
                    }
                    Layout.Width = HorizontalWidth;
                    Layout.Height = static_cast<double>(Count) * HorizontalHeight + static_cast<double>(Count - 1) * Gap;
                    Layout.ClusterType = "EllipseColumn_" + std::to_string(Count);
                    return Layout;
                }

                double X = 0.0;
                double MaxHeight = HorizontalHeight;
                for (std::size_t I = 0; I < Count; ++I) {
                    const bool RotateToVertical = AlternateRotation && (I % 2 == 1);
                    const double SlotWidth = RotateToVertical ? VerticalWidth : HorizontalWidth;
                    const double SlotHeight = RotateToVertical ? VerticalHeight : HorizontalHeight;
                    Layout.Slots.push_back({ X, 0.0, RotateToVertical });
                    X += SlotWidth + Gap;
                    MaxHeight = std::max(MaxHeight, SlotHeight);
                }

                for (TetEllipseLayoutSlot& Slot : Layout.Slots) {
                    const double SlotHeight = Slot.RotateToVertical ? VerticalHeight : HorizontalHeight;
                    Slot.Y = (MaxHeight - SlotHeight) * 0.5;
                }

                Layout.Width = X - Gap;
                Layout.Height = MaxHeight;
                Layout.ClusterType = AlternateRotation ? "EllipseAlternatingLine_" : "EllipseLine_";
                Layout.ClusterType += std::to_string(Count);
                return Layout;
            }

            TetEllipseLayout MakeGridLayout(std::size_t Count, int Rows, double HorizontalWidth, double HorizontalHeight, double VerticalWidth, double VerticalHeight, double Gap, bool AlternateRotation)
            {
                TetEllipseLayout Layout;
                if (Count < 2 || Rows < 1 || Rows > static_cast<int>(Count)) {
                    return Layout;
                }

                const int Columns = static_cast<int>((Count + static_cast<std::size_t>(Rows) - 1) / static_cast<std::size_t>(Rows));
                const double CellWidth = AlternateRotation ? std::max(HorizontalWidth, VerticalWidth) : HorizontalWidth;
                const double CellHeight = AlternateRotation ? std::max(HorizontalHeight, VerticalHeight) : HorizontalHeight;

                Layout.Slots.reserve(Count);
                for (std::size_t I = 0; I < Count; ++I) {
                    const int Row = static_cast<int>(I / static_cast<std::size_t>(Columns));
                    const int Col = static_cast<int>(I % static_cast<std::size_t>(Columns));
                    const bool RotateToVertical = AlternateRotation && ((Row + Col) % 2 == 1);
                    const double SlotWidth = RotateToVertical ? VerticalWidth : HorizontalWidth;
                    const double SlotHeight = RotateToVertical ? VerticalHeight : HorizontalHeight;

                    const double X = static_cast<double>(Col) * (CellWidth + Gap) + (CellWidth - SlotWidth) * 0.5;
                    const double Y = static_cast<double>(Row) * (CellHeight + Gap) + (CellHeight - SlotHeight) * 0.5;
                    Layout.Slots.push_back({ X, Y, RotateToVertical });
                }

                Layout.Width = static_cast<double>(Columns) * CellWidth + static_cast<double>(Columns - 1) * Gap;
                Layout.Height = static_cast<double>(Rows) * CellHeight + static_cast<double>(Rows - 1) * Gap;
                Layout.ClusterType = AlternateRotation ? "EllipseAlternatingGrid_" : "EllipseGrid_";
                Layout.ClusterType += std::to_string(Count) + "_R" + std::to_string(Rows);
                return Layout;
            }

            TetEllipseLayout MakeHoneycombLayout(std::size_t Count, int Rows, double Width, double Height, double Gap)
            {
                TetEllipseLayout Layout;
                if (Count < 5 || Rows < 2 || Rows > static_cast<int>(Count)) {
                    return Layout;
                }

                const int BaseCount = static_cast<int>(Count / static_cast<std::size_t>(Rows));
                int ExtraCount = static_cast<int>(Count % static_cast<std::size_t>(Rows));
                std::vector<int> RowCounts(static_cast<std::size_t>(Rows), BaseCount);

                for (int Row = 0; Row < Rows && ExtraCount > 0; Row += 2) {
                    ++RowCounts[static_cast<std::size_t>(Row)];
                    --ExtraCount;
                }
                for (int Row = 1; Row < Rows && ExtraCount > 0; Row += 2) {
                    ++RowCounts[static_cast<std::size_t>(Row)];
                    --ExtraCount;
                }

                const double StepX = Width + Gap;
                const double RowStep = (Height + Gap) * CET_ELLIPSE_HONEYCOMB_ROW_RATIO;
                Layout.Slots.reserve(Count);

                double MaxX = 0.0;
                double MaxY = 0.0;
                for (int Row = 0; Row < Rows; ++Row) {
                    const double ShiftX = (Row % 2 == 0) ? 0.0 : StepX * 0.5;
                    const double Y = static_cast<double>(Row) * RowStep;
                    for (int Col = 0; Col < RowCounts[static_cast<std::size_t>(Row)]; ++Col) {
                        const double X = ShiftX + static_cast<double>(Col) * StepX;
                        Layout.Slots.push_back({ X, Y, false });
                        MaxX = std::max(MaxX, X + Width);
                        MaxY = std::max(MaxY, Y + Height);
                    }
                }

                if (Layout.Slots.size() != Count) {
                    return {};
                }

                Layout.Width = MaxX;
                Layout.Height = MaxY;
                Layout.ClusterType = "EllipseHoneycomb_" + std::to_string(Count) + "_R" + std::to_string(Rows);
                return Layout;
            }
        }

        CetEllipseClusterBuilder::CetEllipseClusterBuilder() : CetCoreObject() {}
        CetEllipseClusterBuilder::~CetEllipseClusterBuilder() {}

        void CetEllipseClusterBuilder::BuildCandidates(const CetTNestItemVector& AItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, std::vector<TetClusterCandidate>& AOut)
        {
            if (AItems.empty() || AItems.size() != AFeatures.size() || AIndices.size() < 2) {
                return;
            }
            const std::vector<std::vector<int>> Groups = GroupEllipseIndices(AIndices, AFeatures);
            if (Groups.empty()) {
                return;
            }
            const std::size_t OldCandidateCount = AOut.size();
            for (const std::vector<int>& Group : Groups) {
                _BuildSameSizeClusterCandidates(AItems, AFeatures, Group, AOptions, AOut);
            }
            std::cout << "[ELLIPSE][BUILD CANDIDATES] GroupCount = " << Groups.size()<< ", NewCandidateCount = " << AOut.size() - OldCandidateCount << std::endl;
        }

        void CetEllipseClusterBuilder::_BuildSameSizeClusterCandidates(const CetTNestItemVector& AItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, std::vector<TetClusterCandidate>& AOut)
        {
            std::vector<int> Remaining = AIndices;
            std::sort(Remaining.begin(), Remaining.end());
            Remaining.erase(std::unique(Remaining.begin(), Remaining.end()), Remaining.end());

            while (Remaining.size() >= 2) {
                std::size_t BestCount = 0;
                TetClusterCandidate BestCandidate;

                for (std::size_t Count = Remaining.size(); Count >= 2; --Count) {
                    std::vector<int> TrialIndices(Remaining.begin(),Remaining.begin() + static_cast<std::vector<int>::difference_type>(Count));

                    TetClusterCandidate TrialCandidate;
                    if (_BuildClusterCandidate(AItems, AFeatures, TrialIndices, AOptions, TrialCandidate)) {
                        BestCount = Count;
                        BestCandidate = std::move(TrialCandidate);
                        break;
                    }
                }

                if (BestCount < 2) {
                    std::cout << "[ELLIPSE][REJECT] No board-fitting cluster can be built. RemainingCount = "<< Remaining.size() << std::endl;
                    return;
                }

                AOut.push_back(std::move(BestCandidate));
                std::cout << "[ELLIPSE][CANDIDATE] Size = " << BestCount<< ", Type = " << AOut.back().ClusterType<< ", Score = " << AOut.back().Score << std::endl;

                Remaining.erase(Remaining.begin(),Remaining.begin() + static_cast<std::vector<int>::difference_type>(BestCount));
            }
        }

        bool CetEllipseClusterBuilder::_BuildClusterCandidate(const CetTNestItemVector& AItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate)
        {
            AOutCandidate = TetClusterCandidate{};

            if (AItems.size() != AFeatures.size() || AIndices.size() < 2) {
                return false;
            }

            std::vector<int> Indices = AIndices;
            std::sort(Indices.begin(), Indices.end());
            Indices.erase(std::unique(Indices.begin(), Indices.end()), Indices.end());
            if (Indices.size() < 2) {
                return false;
            }

            for (int Index : Indices) {
                if (Index < 0 || Index >= static_cast<int>(AFeatures.size())) {
                    return false;
                }
            }

            const TetShapeFeature& BaseFeature = AFeatures[Indices.front()];
            if (!IsValidEllipseFeature(BaseFeature)) {
                return false;
            }

            for (int Index : Indices) {
                if (!AreSameSizeEllipses(BaseFeature, AFeatures[Index])) {
                    return false;
                }
            }

            CetClusterGeometryHelper Geometry;
            double HorizontalRotation = 0.0;
            double HorizontalMinX = 0.0;
            double HorizontalMinY = 0.0;
            double HorizontalWidth = 0.0;
            double HorizontalHeight = 0.0;
            if (!GetOrientedBounds(AItems[Indices.front()], BaseFeature, false, AOptions, Geometry, HorizontalRotation, HorizontalMinX, HorizontalMinY, HorizontalWidth, HorizontalHeight)) {
                return false;
            }

            double VerticalRotation = 0.0;
            double VerticalMinX = 0.0;
            double VerticalMinY = 0.0;
            double VerticalWidth = 0.0;
            double VerticalHeight = 0.0;
            if (!GetOrientedBounds(AItems[Indices.front()], BaseFeature, true, AOptions, Geometry, VerticalRotation, VerticalMinX, VerticalMinY, VerticalWidth, VerticalHeight)) {
                return false;
            }

            const double RequiredGap = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));
            const double SafetyGap = RequiredGap > 0.0 ? std::max(10.0, RequiredGap * 0.05) : 0.0;
            const double Gap = RequiredGap + SafetyGap;

            std::vector<TetEllipseLayout> Layouts;
            Layouts.push_back(MakeLineLayout(Indices.size(), HorizontalWidth, HorizontalHeight, VerticalWidth, VerticalHeight, Gap, false, false));
            Layouts.push_back(MakeLineLayout(Indices.size(), HorizontalWidth, HorizontalHeight, VerticalWidth, VerticalHeight, Gap, true, false));
            Layouts.push_back(MakeLineLayout(Indices.size(), HorizontalWidth, HorizontalHeight, VerticalWidth, VerticalHeight, Gap, false, true));

            for (int Rows = 2; Rows <= static_cast<int>(Indices.size()); ++Rows) {
                Layouts.push_back(MakeGridLayout(Indices.size(), Rows, HorizontalWidth, HorizontalHeight, VerticalWidth, VerticalHeight, Gap, false));
                Layouts.push_back(MakeGridLayout(Indices.size(), Rows, HorizontalWidth, HorizontalHeight, VerticalWidth, VerticalHeight, Gap, true));
                Layouts.push_back(MakeHoneycombLayout(Indices.size(), Rows, HorizontalWidth, HorizontalHeight, Gap));
            }

            bool HasBest = false;
            TetClusterCandidate BestCandidate;

            for (const TetEllipseLayout& Layout : Layouts) {
                if (Layout.Slots.size() != Indices.size() || Layout.Width <= 0.0 || Layout.Height <= 0.0) {
                    continue;
                }

                if (!FitsBin(Layout.Width, Layout.Height, AOptions)) {
                    continue;
                }

                TetClusterCandidate Candidate;
                Candidate.BuilderName = "EllipseBuilder";
                Candidate.ClusterType = Layout.ClusterType;
                Candidate.OriginalIndices = Indices;
                Candidate.Confidence = 0.9;
                Candidate.Transforms.reserve(Indices.size());

                bool TransformValid = true;
                for (std::size_t I = 0; I < Indices.size(); ++I) {
                    const int Index = Indices[I];
                    const TetShapeFeature& Feature = AFeatures[Index];
                    const TetEllipseLayoutSlot& Slot = Layout.Slots[I];

                    double Rotation = 0.0;
                    double MinX = 0.0;
                    double MinY = 0.0;
                    double Width = 0.0;
                    double Height = 0.0;
                    if (!GetOrientedBounds(AItems[Index], Feature, Slot.RotateToVertical, AOptions, Geometry, Rotation, MinX, MinY, Width, Height)) {
                        TransformValid = false;
                        break;
                    }

                    TetItemTransform Transform;
                    Transform.OriginalId = Index;
                    Transform.RelativeRotation = Rotation;
                    Transform.RelativeX = Slot.X - MinX;
                    Transform.RelativeY = Slot.Y - MinY;
                    Candidate.Transforms.push_back(Transform);
                }

                if (!TransformValid) {
                    continue;
                }

                if (!Geometry.FinalizeCandidate(AItems, AOptions, Candidate)) {
                    continue;
                }

                Candidate.Score = _CalculateScore(Candidate, AOptions);
                if (!HasBest || Candidate.Score > BestCandidate.Score) {
                    HasBest = true;
                    BestCandidate = std::move(Candidate);
                }
            }

            if (!HasBest) {
                return false;
            }

            AOutCandidate = std::move(BestCandidate);
            return true;
        }

        double CetEllipseClusterBuilder::_CalculateScore(const TetClusterCandidate& ACandidate, const TetNestOptions& AOptions)
        {
            if (!ACandidate.Valid || ACandidate.OriginalIndices.empty() || ACandidate.ClusterWidth <= 0.0 || ACandidate.ClusterHeight <= 0.0 || ACandidate.ProxyArea <= 0.0) {
                return -std::numeric_limits<double>::infinity();
            }

            const double FillScore = ACandidate.FillRatio * 1100.0;
            const double SavingScore = ACandidate.AreaSavingRatio * 800.0;
            const double ItemCountScore = static_cast<double>(ACandidate.OriginalIndices.size()) * 55.0;
            const double LongSide = std::max(ACandidate.ClusterWidth, ACandidate.ClusterHeight);
            const double ShortSide = std::min(ACandidate.ClusterWidth, ACandidate.ClusterHeight);
            const double CompactRatio = LongSide > 0.0 ? ShortSide / LongSide : 0.0;
            const double CompactScore = CompactRatio * 20.0;
            double LayoutBonus = 0.0;
            if (ACandidate.ClusterType.find("Honeycomb") != std::string::npos) {
                LayoutBonus = 145.0;
            }
            else if (ACandidate.ClusterType.find("AlternatingGrid") != std::string::npos) {
                LayoutBonus = 110.0;
            }
            else if (ACandidate.ClusterType.find("Grid") != std::string::npos) {
                LayoutBonus = 85.0;
            }
            else if (ACandidate.ClusterType.find("AlternatingLine") != std::string::npos) {
                LayoutBonus = 45.0;
            }
            else if (ACandidate.ClusterType.find("Column") != std::string::npos) {
                LayoutBonus = ACandidate.OriginalIndices.size() >= 4 ? -140.0 : -60.0;
            }

            const double BinWidth = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth));
            const double HorizontalSpreadRatio = BinWidth > 0.0 ? std::clamp(ACandidate.ClusterWidth / BinWidth, 0.0, 1.0) : 0.0;
            const double HorizontalSpreadScore = HorizontalSpreadRatio * 70.0;
            const double PerimeterPenalty = (ACandidate.ClusterWidth + ACandidate.ClusterHeight) * 0.000001;
            return FillScore + SavingScore + ItemCountScore + CompactScore + LayoutBonus + HorizontalSpreadScore + ACandidate.Confidence - PerimeterPenalty;
        }

    }
}
