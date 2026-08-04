#include "pch.h"
#include "Nest2D_EllipseClusterBuilder.h"
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

            bool IsValidEllipseFeature(const TetShapeFeature& AFeature)
            {
                return AFeature.ShapeType == MetShapeType::EllipseLike &&
                    AFeature.EllipseMajorAxis > 0.0 &&
                    AFeature.EllipseMinorAxis > 0.0 &&
                    AFeature.Width > 0.0 &&
                    AFeature.Height > 0.0 &&
                    AFeature.Area > 0.0;
            }

            bool AreSameSizeEllipses(const TetShapeFeature& A, const TetShapeFeature& AB)
            {
                return IsValidEllipseFeature(A) &&
                    IsValidEllipseFeature(AB) &&
                    CetClusterMathUtils::NearlyEqual(A.EllipseMajorAxis, AB.EllipseMajorAxis, CET_ELLIPSE_SIZE_TOLERANCE) &&
                    CetClusterMathUtils::NearlyEqual(A.EllipseMinorAxis, AB.EllipseMinorAxis, CET_ELLIPSE_SIZE_TOLERANCE);
            }

            bool FitsBin(double AWidth, double AHeight, const TetNestOptions& AOptions)
            {
                if (AWidth <= 0.0 || AHeight <= 0.0){
                    return false;
                }

                const double BinWidth = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth));
                const double BinHeight = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinHeight));
                if (BinWidth <= 0.0 || BinHeight <= 0.0){
                    return false;
                }

                const bool Normal = AWidth <= BinWidth && AHeight <= BinHeight;
                const bool QuarterTurnAllowed = CetRotationUtils::IsAllowedRotation(CET_CLUSTER_HALF_PI, AOptions.Rotations, 1e-9);
                const bool Rotated = QuarterTurnAllowed && AHeight <= BinWidth && AWidth <= BinHeight;
                return Normal || Rotated;
            }

            std::vector<std::vector<int>> GroupEllipseIndices(const std::vector<int>& AIndices, const std::vector<TetShapeFeature>& AFeatures)
            {
                std::vector<TetEllipseIndexInfo> Infos;
                Infos.reserve(AIndices.size());

                for (int Index : AIndices){
                    if (Index < 0 || Index >= static_cast<int>(AFeatures.size())){
                        continue;
                    }

                    const TetShapeFeature& Feature = AFeatures[Index];
                    if (!IsValidEllipseFeature(Feature)){
                        continue;
                    }

                    Infos.push_back({ Index, Feature.EllipseMajorAxis, Feature.EllipseMinorAxis });
                }

                std::sort(
                    Infos.begin(),
                    Infos.end(),
                    [](const TetEllipseIndexInfo& A, const TetEllipseIndexInfo& AB)
                    {
                        if (A.MajorAxis != AB.MajorAxis){
                            return A.MajorAxis < AB.MajorAxis;
                        }
                        if (A.MinorAxis != AB.MinorAxis){
                            return A.MinorAxis < AB.MinorAxis;
                        }
                        return A.Index < AB.Index;
                    });

                Infos.erase(
                    std::unique(
                        Infos.begin(),
                        Infos.end(),
                        [](const TetEllipseIndexInfo& A, const TetEllipseIndexInfo& AB)
                        {
                            return A.Index == AB.Index;
                        }),
                    Infos.end());

                std::vector<std::vector<int>> Groups;
                std::vector<int> CurrentGroup;
                TetEllipseIndexInfo BaseInfo;

                for (const TetEllipseIndexInfo& Info : Infos){
                    if (CurrentGroup.empty()){
                        BaseInfo = Info;
                        CurrentGroup.push_back(Info.Index);
                        continue;
                    }

                    if (CetClusterMathUtils::NearlyEqual(BaseInfo.MajorAxis, Info.MajorAxis, CET_ELLIPSE_SIZE_TOLERANCE) &&CetClusterMathUtils::NearlyEqual(BaseInfo.MinorAxis, Info.MinorAxis, CET_ELLIPSE_SIZE_TOLERANCE)){
                        CurrentGroup.push_back(Info.Index);
                    }
                    else {
                        Groups.push_back(std::move(CurrentGroup));
                        CurrentGroup.clear();
                        BaseInfo = Info;
                        CurrentGroup.push_back(Info.Index);
                    }
                }

                if (!CurrentGroup.empty()){
                    Groups.push_back(std::move(CurrentGroup));
                }

                return Groups;
            }

            bool GetOrientedBounds(const CetNestItem& AItem, const TetShapeFeature& AFeature, bool ARotateToVertical, const TetNestOptions& AOptions, const CetClusterGeometryHelper& AGeometry, double& AOutRotation, double& AOutMinX, double& AOutMinY, double& AOutWidth, double& AOutHeight)
            {
                const double TargetAngle = ARotateToVertical ? CET_CLUSTER_HALF_PI : 0.0;
                if (!CetRotationUtils::SnapToNearestAllowedRotation(TargetAngle - AFeature.EllipseAngle, AOptions.Rotations, AOutRotation)){
                    return false;
                }

                const CetPath Contour = AGeometry.TransformContour(AGeometry.GetIdentityContour(AItem), AOutRotation, 0.0, 0.0);
                double MaxX = 0.0;
                double MaxY = 0.0;
                if (!AGeometry.GetBounds(Contour, AOutMinX, AOutMinY, MaxX, MaxY)){
                    return false;
                }

                AOutWidth = MaxX - AOutMinX;
                AOutHeight = MaxY - AOutMinY;
                return AOutWidth > 0.0 && AOutHeight > 0.0;
            }

            TetEllipseLayout MakeLineLayout(std::size_t ACount, double AHorizontalWidth, double AHorizontalHeight, double AVerticalWidth, double AVerticalHeight, double AGap, bool AVerticalStack, bool AAlternateRotation)
            {
                TetEllipseLayout Layout;
                if (ACount < 2){
                    return Layout;
                }

                Layout.Slots.reserve(ACount);
                if (AVerticalStack){
                    double Y = 0.0;
                    for (std::size_t I = 0; I < ACount; ++I){
                        Layout.Slots.push_back({ 0.0, Y, false });
                        Y += AHorizontalHeight + AGap;
                    }
                    Layout.Width = AHorizontalWidth;
                    Layout.Height = static_cast<double>(ACount) * AHorizontalHeight + static_cast<double>(ACount - 1) * AGap;
                    Layout.ClusterType = "EllipseColumn_" + std::to_string(ACount);
                    return Layout;
                }

                double X = 0.0;
                double MaxHeight = AHorizontalHeight;
                for (std::size_t I = 0; I < ACount; ++I){
                    const bool RotateToVertical = AAlternateRotation && (I % 2 == 1);
                    const double SlotWidth = RotateToVertical ? AVerticalWidth : AHorizontalWidth;
                    const double SlotHeight = RotateToVertical ? AVerticalHeight : AHorizontalHeight;
                    Layout.Slots.push_back({ X, 0.0, RotateToVertical });
                    X += SlotWidth + AGap;
                    MaxHeight = std::max(MaxHeight, SlotHeight);
                }

                for (TetEllipseLayoutSlot& Slot : Layout.Slots){
                    const double SlotHeight = Slot.RotateToVertical ? AVerticalHeight : AHorizontalHeight;
                    Slot.Y = (MaxHeight - SlotHeight) * 0.5;
                }

                Layout.Width = X - AGap;
                Layout.Height = MaxHeight;
                Layout.ClusterType = AAlternateRotation ? "EllipseAlternatingLine_" : "EllipseLine_";
                Layout.ClusterType += std::to_string(ACount);
                return Layout;
            }

            TetEllipseLayout MakeGridLayout(std::size_t ACount, int ARows, double AHorizontalWidth, double AHorizontalHeight, double AVerticalWidth, double AVerticalHeight, double AGap, bool AAlternateRotation)
            {
                TetEllipseLayout Layout;
                if (ACount < 2 || ARows < 1 || ARows > static_cast<int>(ACount)){
                    return Layout;
                }

                const int Columns = static_cast<int>((ACount + static_cast<std::size_t>(ARows) - 1) / static_cast<std::size_t>(ARows));
                const double CellWidth = AAlternateRotation ? std::max(AHorizontalWidth, AVerticalWidth) : AHorizontalWidth;
                const double CellHeight = AAlternateRotation ? std::max(AHorizontalHeight, AVerticalHeight) : AHorizontalHeight;

                Layout.Slots.reserve(ACount);
                for (std::size_t I = 0; I < ACount; ++I){
                    const int Row = static_cast<int>(I / static_cast<std::size_t>(Columns));
                    const int Col = static_cast<int>(I % static_cast<std::size_t>(Columns));
                    const bool RotateToVertical = AAlternateRotation && ((Row + Col) % 2 == 1);
                    const double SlotWidth = RotateToVertical ? AVerticalWidth : AHorizontalWidth;
                    const double SlotHeight = RotateToVertical ? AVerticalHeight : AHorizontalHeight;

                    const double X = static_cast<double>(Col) * (CellWidth + AGap) + (CellWidth - SlotWidth) * 0.5;
                    const double Y = static_cast<double>(Row) * (CellHeight + AGap) + (CellHeight - SlotHeight) * 0.5;
                    Layout.Slots.push_back({ X, Y, RotateToVertical });
                }

                Layout.Width = static_cast<double>(Columns) * CellWidth + static_cast<double>(Columns - 1) * AGap;
                Layout.Height = static_cast<double>(ARows) * CellHeight + static_cast<double>(ARows - 1) * AGap;
                Layout.ClusterType = AAlternateRotation ? "EllipseAlternatingGrid_" : "EllipseGrid_";
                Layout.ClusterType += std::to_string(ACount) + "_R" + std::to_string(ARows);
                return Layout;
            }

            TetEllipseLayout MakeHoneycombLayout(std::size_t ACount, int ARows, double AWidth, double AHeight, double AGap)
            {
                TetEllipseLayout Layout;
                if (ACount < 5 || ARows < 2 || ARows > static_cast<int>(ACount)){
                    return Layout;
                }

                const int BaseCount = static_cast<int>(ACount / static_cast<std::size_t>(ARows));
                int ExtraCount = static_cast<int>(ACount % static_cast<std::size_t>(ARows));
                std::vector<int> RowCounts(static_cast<std::size_t>(ARows), BaseCount);

                for (int Row = 0; Row < ARows && ExtraCount > 0; Row += 2){
                    ++RowCounts[static_cast<std::size_t>(Row)];
                    --ExtraCount;
                }
                for (int Row = 1; Row < ARows && ExtraCount > 0; Row += 2){
                    ++RowCounts[static_cast<std::size_t>(Row)];
                    --ExtraCount;
                }

                const double StepX = AWidth + AGap;
                const double RowStep = (AHeight + AGap) * CET_ELLIPSE_HONEYCOMB_ROW_RATIO;
                Layout.Slots.reserve(ACount);

                double MaxX = 0.0;
                double MaxY = 0.0;
                for (int Row = 0; Row < ARows; ++Row){
                    const double ShiftX = (Row % 2 == 0) ? 0.0 : StepX * 0.5;
                    const double Y = static_cast<double>(Row) * RowStep;
                    for (int Col = 0; Col < RowCounts[static_cast<std::size_t>(Row)]; ++Col){
                        const double X = ShiftX + static_cast<double>(Col) * StepX;
                        Layout.Slots.push_back({ X, Y, false });
                        MaxX = std::max(MaxX, X + AWidth);
                        MaxY = std::max(MaxY, Y + AHeight);
                    }
                }

                if (Layout.Slots.size() != ACount){
                    return {};
                }

                Layout.Width = MaxX;
                Layout.Height = MaxY;
                Layout.ClusterType = "EllipseHoneycomb_" + std::to_string(ACount) + "_R" + std::to_string(ARows);
                return Layout;
            }
        }

        CetEllipseClusterBuilder::CetEllipseClusterBuilder() : CetCoreObject() {}
        CetEllipseClusterBuilder::~CetEllipseClusterBuilder() {}

        void CetEllipseClusterBuilder::BuildCandidates(const CetTNestItemVector& AItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, std::vector<TetClusterCandidate>& AOut)
        {
            if (AItems.empty() || AItems.size() != AFeatures.size() || AIndices.size() < 2){
                return;
            }
            const std::vector<std::vector<int>> Groups = GroupEllipseIndices(AIndices, AFeatures);
            if (Groups.empty()){
                return;
            }
            const std::size_t OldCandidateCount = AOut.size();
            for (const std::vector<int>& Group : Groups){
                _BuildSameSizeClusterCandidates(AItems, AFeatures, Group, AOptions, AOut);
            }
            std::cout << "[ELLIPSE][BUILD CANDIDATES] GroupCount = " << Groups.size() << ", NewCandidateCount = " << AOut.size() - OldCandidateCount << std::endl;
        }

        void CetEllipseClusterBuilder::_BuildSameSizeClusterCandidates(const CetTNestItemVector& AItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, std::vector<TetClusterCandidate>& AOut)
        {
            std::vector<int> Remaining = AIndices;
            std::sort(Remaining.begin(), Remaining.end());
            Remaining.erase(std::unique(Remaining.begin(), Remaining.end()), Remaining.end());

            while (Remaining.size() >= 2){
                std::size_t BestCount = 0;
                TetClusterCandidate BestCandidate;

                for (std::size_t Count = Remaining.size(); Count >= 2; --Count){
                    std::vector<int> TrialIndices(Remaining.begin(),Remaining.begin() + static_cast<std::vector<int>::difference_type>(Count));

                    TetClusterCandidate TrialCandidate;
                    if (_BuildClusterCandidate(AItems, AFeatures, TrialIndices, AOptions, TrialCandidate)){
                        BestCount = Count;
                        BestCandidate = std::move(TrialCandidate);
                        break;
                    }
                }

                if (BestCount < 2){
                    std::cout << "[ELLIPSE][REJECT] No board-fitting cluster can be built. RemainingCount = " << Remaining.size() << std::endl;
                    return;
                }

                AOut.push_back(std::move(BestCandidate));
                std::cout << "[ELLIPSE][CANDIDATE] Size = " << BestCount << ", Type = " << AOut.back().ClusterType << ", Score = " << AOut.back().Score << std::endl;

                Remaining.erase(Remaining.begin(),Remaining.begin() + static_cast<std::vector<int>::difference_type>(BestCount));
            }
        }

        bool CetEllipseClusterBuilder::_BuildClusterCandidate(const CetTNestItemVector& AItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate)
        {
            AOutCandidate = TetClusterCandidate{};

            if (AItems.size() != AFeatures.size() || AIndices.size() < 2){
                return false;
            }

            std::vector<int> Indices = AIndices;
            std::sort(Indices.begin(), Indices.end());
            Indices.erase(std::unique(Indices.begin(), Indices.end()), Indices.end());
            if (Indices.size() < 2){
                return false;
            }

            for (int Index : Indices){
                if (Index < 0 || Index >= static_cast<int>(AFeatures.size())){
                    return false;
                }
            }

            const TetShapeFeature& BaseFeature = AFeatures[Indices.front()];
            if (!IsValidEllipseFeature(BaseFeature)){
                return false;
            }

            for (int Index : Indices){
                if (!AreSameSizeEllipses(BaseFeature, AFeatures[Index])){
                    return false;
                }
            }

            CetClusterGeometryHelper Geometry;
            double HorizontalRotation = 0.0;
            double HorizontalMinX = 0.0;
            double HorizontalMinY = 0.0;
            double HorizontalWidth = 0.0;
            double HorizontalHeight = 0.0;
            if (!GetOrientedBounds(AItems[Indices.front()], BaseFeature, false, AOptions, Geometry, HorizontalRotation, HorizontalMinX, HorizontalMinY, HorizontalWidth, HorizontalHeight)){
                return false;
            }

            double VerticalRotation = 0.0;
            double VerticalMinX = 0.0;
            double VerticalMinY = 0.0;
            double VerticalWidth = 0.0;
            double VerticalHeight = 0.0;
            if (!GetOrientedBounds(AItems[Indices.front()], BaseFeature, true, AOptions, Geometry, VerticalRotation, VerticalMinX, VerticalMinY, VerticalWidth, VerticalHeight)){
                return false;
            }

            const double RequiredGap = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));
            const double SafetyGap = RequiredGap > 0.0 ? std::max(10.0, RequiredGap * 0.05) : 0.0;
            const double Gap = RequiredGap + SafetyGap;

            std::vector<TetEllipseLayout> Layouts;
            Layouts.push_back(MakeLineLayout(Indices.size(), HorizontalWidth, HorizontalHeight, VerticalWidth, VerticalHeight, Gap, false, false));
            Layouts.push_back(MakeLineLayout(Indices.size(), HorizontalWidth, HorizontalHeight, VerticalWidth, VerticalHeight, Gap, true, false));
            Layouts.push_back(MakeLineLayout(Indices.size(), HorizontalWidth, HorizontalHeight, VerticalWidth, VerticalHeight, Gap, false, true));

            for (int Rows = 2; Rows <= static_cast<int>(Indices.size()); ++Rows){
                Layouts.push_back(MakeGridLayout(Indices.size(), Rows, HorizontalWidth, HorizontalHeight, VerticalWidth, VerticalHeight, Gap, false));
                Layouts.push_back(MakeGridLayout(Indices.size(), Rows, HorizontalWidth, HorizontalHeight, VerticalWidth, VerticalHeight, Gap, true));
                Layouts.push_back(MakeHoneycombLayout(Indices.size(), Rows, HorizontalWidth, HorizontalHeight, Gap));
            }

            bool HasBest = false;
            TetClusterCandidate BestCandidate;

            for (const TetEllipseLayout& Layout : Layouts){
                if (Layout.Slots.size() != Indices.size() || Layout.Width <= 0.0 || Layout.Height <= 0.0){
                    continue;
                }

                if (!FitsBin(Layout.Width, Layout.Height, AOptions)){
                    continue;
                }

                TetClusterCandidate Candidate;
                Candidate.BuilderName = "EllipseBuilder";
                Candidate.ClusterType = Layout.ClusterType;
                Candidate.OriginalIndices = Indices;
                Candidate.Confidence = 0.9;
                Candidate.Transforms.reserve(Indices.size());

                bool TransformValid = true;
                for (std::size_t I = 0; I < Indices.size(); ++I){
                    const int Index = Indices[I];
                    const TetShapeFeature& Feature = AFeatures[Index];
                    const TetEllipseLayoutSlot& Slot = Layout.Slots[I];

                    double Rotation = 0.0;
                    double MinX = 0.0;
                    double MinY = 0.0;
                    double Width = 0.0;
                    double Height = 0.0;
                    if (!GetOrientedBounds(AItems[Index], Feature, Slot.RotateToVertical, AOptions, Geometry, Rotation, MinX, MinY, Width, Height)){
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

                if (!TransformValid){
                    continue;
                }

                if (!Geometry.FinalizeCandidate(AItems, AOptions, Candidate)){
                    continue;
                }

                Candidate.Score = _CalculateScore(Candidate, AOptions);
                if (!HasBest || Candidate.Score > BestCandidate.Score){
                    HasBest = true;
                    BestCandidate = std::move(Candidate);
                }
            }

            if (!HasBest){
                return false;
            }

            AOutCandidate = std::move(BestCandidate);
            return true;
        }

        double CetEllipseClusterBuilder::_CalculateScore(const TetClusterCandidate& ACandidate, const TetNestOptions& AOptions)
        {
            if (!ACandidate.Valid || ACandidate.OriginalIndices.empty() || ACandidate.ClusterWidth <= 0.0 || ACandidate.ClusterHeight <= 0.0 || ACandidate.ProxyArea <= 0.0){
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
            if (ACandidate.ClusterType.find("Honeycomb") != std::string::npos){
                LayoutBonus = 145.0;
            }
            else if (ACandidate.ClusterType.find("AlternatingGrid") != std::string::npos){
                LayoutBonus = 110.0;
            }
            else if (ACandidate.ClusterType.find("Grid") != std::string::npos){
                LayoutBonus = 85.0;
            }
            else if (ACandidate.ClusterType.find("AlternatingLine") != std::string::npos){
                LayoutBonus = 45.0;
            }
            else if (ACandidate.ClusterType.find("Column") != std::string::npos){
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
