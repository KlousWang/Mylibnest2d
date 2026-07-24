#include "pch.h"
#include "Nest2D_CircleClusterBuilder.h"
#include "Nest2D_ClusterGeometryHelper.h"
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
            constexpr double CET_CIRCLE_SIZE_TOLERANCE = 0.01;
            constexpr double CET_CIRCLE_HONEYCOMB_ROW_RATIO = 0.86602540378443864676;

            bool NearlyEqual(double AA, double AB, double ARelativeTolerance)
            {
                const double Denominator = std::max(1.0, std::max(std::abs(AA), std::abs(AB)));
                return std::abs(AA - AB) <= Denominator * ARelativeTolerance;
            }

            double GetCircleSizeKey(const TetShapeFeature& AFeature)
            {
                return std::max(AFeature.Width, AFeature.Height);
            }

            bool IsValidCircleFeature(const TetShapeFeature& AFeature)
            {
                return AFeature.ShapeType == MetShapeType::CircleLike &&AFeature.Width > 0.0 &&AFeature.Height > 0.0 &&AFeature.Area > 0.0;
            }

            std::vector<std::vector<int>> GroupCircleIndices(const std::vector<int>& AIndices, const std::vector<TetShapeFeature>& AFeatures)
            {
                std::vector<TetCircleIndexInfo> Infos;
                Infos.reserve(AIndices.size());

                for (int Index : AIndices) {
                    if (Index < 0 || Index >= static_cast<int>(AFeatures.size())) {
                        continue;
                    }

                    const TetShapeFeature& Feature = AFeatures[Index];
                    if (!IsValidCircleFeature(Feature)) {
                        continue;
                    }

                    Infos.push_back({ Index, GetCircleSizeKey(Feature) });
                }

                std::sort(
                    Infos.begin(),
                    Infos.end(),
                    [](const TetCircleIndexInfo& A, const TetCircleIndexInfo& B)
                    {
                        if (A.SizeKey != B.SizeKey) {
                            return A.SizeKey < B.SizeKey;
                        }
                        return A.Index < B.Index;
                    });

                Infos.erase(
                    std::unique(
                        Infos.begin(),
                        Infos.end(),
                        [](const TetCircleIndexInfo& A, const TetCircleIndexInfo& B)
                        {
                            return A.Index == B.Index;
                        }),
                    Infos.end());

                std::vector<std::vector<int>> Groups;
                if (Infos.empty()) {
                    return Groups;
                }

                std::vector<int> CurrentGroup;
                double CurrentBaseSize = 0.0;

                for (const TetCircleIndexInfo& Info : Infos) {
                    if (CurrentGroup.empty()) {
                        CurrentBaseSize = Info.SizeKey;
                        CurrentGroup.push_back(Info.Index);
                        continue;
                    }

                    if (NearlyEqual(CurrentBaseSize, Info.SizeKey, CET_CIRCLE_SIZE_TOLERANCE)) {
                        CurrentGroup.push_back(Info.Index);
                    }
                    else {
                        Groups.push_back(std::move(CurrentGroup));
                        CurrentGroup.clear();
                        CurrentBaseSize = Info.SizeKey;
                        CurrentGroup.push_back(Info.Index);
                    }
                }

                if (!CurrentGroup.empty()) {
                    Groups.push_back(std::move(CurrentGroup));
                }

                return Groups;
            }

            bool FitsBin(double AClusterWidth, double AClusterHeight, const TetNestOptions& AOptions)
            {
                if (AClusterWidth <= 0.0 || AClusterHeight <= 0.0) {
                    return false;
                }

                const double BinWidth = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth));
                const double BinHeight = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinHeight));
                if (BinWidth <= 0.0 || BinHeight <= 0.0) {
                    return false;
                }

                const bool FitsNormally = AClusterWidth <= BinWidth && AClusterHeight <= BinHeight;
                const bool FitsAfterRotation = AOptions.Rotations > 1 && AClusterHeight <= BinWidth && AClusterWidth <= BinHeight;
                return FitsNormally || FitsAfterRotation;
            }

            TetCircleLayout MakePairLayout(double ACellSize, double AGap)
            {
                TetCircleLayout Layout;
                const double Radius = ACellSize * 0.5;
                const double Step = ACellSize + AGap;

                Layout.Slots = {
                    { Radius, Radius },
                    { Radius + Step, Radius }
                };
                Layout.Width = ACellSize * 2.0 + AGap;
                Layout.Height = ACellSize;
                Layout.ClusterType = "CirclePair2";
                return Layout;
            }

            TetCircleLayout MakeTriangleLayout(double ACellSize, double AGap)
            {
                TetCircleLayout Layout;
                const double Radius = ACellSize * 0.5;
                const double Step = ACellSize + AGap;
                const double RowStep = Step * CET_CIRCLE_HONEYCOMB_ROW_RATIO;

                Layout.Slots = {
                    { Radius, Radius },
                    { Radius + Step, Radius },
                    { Radius + Step * 0.5, Radius + RowStep }
                };
                Layout.Width = ACellSize * 2.0 + AGap;
                Layout.Height = ACellSize + RowStep;
                Layout.ClusterType = "CircleTriangle3";
                return Layout;
            }

            TetCircleLayout MakeSquareLayout(double ACellSize, double AGap)
            {
                TetCircleLayout Layout;
                const double Radius = ACellSize * 0.5;
                const double Step = ACellSize + AGap;

                Layout.Slots = {
                    { Radius, Radius },
                    { Radius + Step, Radius },
                    { Radius, Radius + Step },
                    { Radius + Step, Radius + Step }
                };
                Layout.Width = ACellSize * 2.0 + AGap;
                Layout.Height = ACellSize * 2.0 + AGap;
                Layout.ClusterType = "CircleBlock4";
                return Layout;
            }

            TetCircleLayout MakeHoneycombLayout(std::size_t ACount, int ARows, double ACellSize, double AGap)
            {
                TetCircleLayout Layout;

                if (ACount < 5 || ARows < 2 || ARows > static_cast<int>(ACount)) {
                    return Layout;
                }

                const double Radius = ACellSize * 0.5;
                const double Step = ACellSize + AGap;
                const double RowStep = Step * CET_CIRCLE_HONEYCOMB_ROW_RATIO;

                const int BaseCount = static_cast<int>(ACount / static_cast<std::size_t>(ARows));
                int ExtraCount = static_cast<int>(ACount % static_cast<std::size_t>(ARows));

                std::vector<int> RowCounts(static_cast<std::size_t>(ARows), BaseCount);

                for (int Row = 0; Row < ARows && ExtraCount > 0; Row += 2) {
                    ++RowCounts[static_cast<std::size_t>(Row)];
                    --ExtraCount;
                }
                for (int Row = 1; Row < ARows && ExtraCount > 0; Row += 2) {
                    ++RowCounts[static_cast<std::size_t>(Row)];
                    --ExtraCount;
                }

                for (int Row = 0; Row < ARows; ++Row) {
                    const double ShiftX = (Row % 2 == 0) ? 0.0 : Step * 0.5;
                    const double CenterY = Radius + static_cast<double>(Row) * RowStep;
                    for (int Col = 0; Col < RowCounts[static_cast<std::size_t>(Row)]; ++Col) {
                        Layout.Slots.push_back({Radius + ShiftX + static_cast<double>(Col) * Step,CenterY});
                    }
                }

                if (Layout.Slots.size() != ACount) {
                    return {};
                }

                const double MinX = Radius;
                const double MinY = Radius;

                double MaxX = std::numeric_limits<double>::lowest();
                double MaxY = std::numeric_limits<double>::lowest();
                for (const TetCircleLayoutSlot& Slot : Layout.Slots) {
                    MaxX = std::max(MaxX, Slot.CenterX + Radius);
                    MaxY = std::max(MaxY, Slot.CenterY + Radius);
                }

                for (TetCircleLayoutSlot& Slot : Layout.Slots) {
                    Slot.CenterX -= MinX;
                    Slot.CenterY -= MinY;
                }

                Layout.Width = MaxX - MinX;
                Layout.Height = MaxY - MinY;
                Layout.ClusterType = "CircleHoneycomb_" + std::to_string(ACount) + "_R" + std::to_string(ARows);
                return Layout;
            }

            TetCircleLayout SelectBestHoneycombLayout(std::size_t ACount, double ACellSize, double AGap, const TetNestOptions& AOptions)
            {
                TetCircleLayout BestLayout;
                double BestCost = std::numeric_limits<double>::infinity();

                for (int Rows = 2; Rows <= static_cast<int>(ACount); ++Rows) {
                    TetCircleLayout Layout = MakeHoneycombLayout(ACount, Rows, ACellSize, AGap);
                    if (Layout.Slots.size() != ACount) {
                        continue;
                    }

                    if (!FitsBin(Layout.Width, Layout.Height, AOptions)) {
                        continue;
                    }

                    const double Area = Layout.Width * Layout.Height;
                    const double Aspect = Layout.Height > 0.0 ? Layout.Width / Layout.Height : 0.0;
                    double Cost = Area;

                    if (Aspect < 1.0) {
                        Cost *= 1.12;
                    }

                    Cost += static_cast<double>(Rows) * 0.001;

                    if (Cost < BestCost) {
                        BestCost = Cost;
                        BestLayout = std::move(Layout);
                    }
                }

                return BestLayout;
            }
        }

        CetCircleClusterBuilder::CetCircleClusterBuilder() : CetCoreObject() {}
        CetCircleClusterBuilder::~CetCircleClusterBuilder() {}

        void CetCircleClusterBuilder::BuildCandidates(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, std::vector<TetClusterCandidate>& AOutCandidates)
        {
            if (AOriginalItems.empty()) {
                return;
            }

            if (AFeatures.size() != AOriginalItems.size()) {
                std::cout << "[CIRCLE][ERROR] Feature count mismatch. OriginalItems = " << AOriginalItems.size()
                    << ", Features = " << AFeatures.size() << std::endl;
                return;
            }

            if (AIndices.size() < 2) {
                return;
            }

            const std::vector<std::vector<int>> Groups = GroupCircleIndices(AIndices, AFeatures);
            if (Groups.empty()) {
                return;
            }

            const std::size_t OldCandidateCount = AOutCandidates.size();

            for (const std::vector<int>& Group : Groups) {
                _BuildSameSizeClusterCandidates(AOriginalItems, AFeatures, Group, AOptions, AOutCandidates);
            }

            std::cout << "[CIRCLE][BUILD CANDIDATES] GroupCount = " << Groups.size()<< ", NewCandidateCount = " << AOutCandidates.size() - OldCandidateCount << std::endl;
        }

        void CetCircleClusterBuilder::_BuildSameSizeClusterCandidates(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, std::vector<TetClusterCandidate>& AOutCandidates)
        {
            if (AIndices.size() < 2) {
                return;
            }

            std::vector<int> Remaining = AIndices;
            std::sort(Remaining.begin(), Remaining.end());
            Remaining.erase(std::unique(Remaining.begin(), Remaining.end()), Remaining.end());

            while (Remaining.size() >= 2) {
                std::size_t Low = 2;
                std::size_t High = Remaining.size();
                std::size_t BestCount = 0;
                TetClusterCandidate BestCandidate;

                while (Low <= High) {
                    const std::size_t Mid = Low + (High - Low) / 2;
                    std::vector<int> TrialIndices(Remaining.begin(),Remaining.begin() + static_cast<std::vector<int>::difference_type>(Mid));

                    TetClusterCandidate TrialCandidate;
                    if (_BuildClusterCandidate(AOriginalItems, AFeatures, TrialIndices, AOptions, TrialCandidate)) {
                        BestCount = Mid;
                        BestCandidate = std::move(TrialCandidate);
                        Low = Mid + 1;
                    }
                    else {
                        if (Mid == 0) {
                            break;
                        }
                        High = Mid - 1;
                    }
                }

                if (BestCount < 2) {
                    std::cout << "[CIRCLE][REJECT] No board-fitting cluster can be built. RemainingCount = "<< Remaining.size() << std::endl;
                    return;
                }

                AOutCandidates.push_back(std::move(BestCandidate));
                std::cout << "[CIRCLE][CANDIDATE] Size = " << BestCount<< ", Type = " << AOutCandidates.back().ClusterType
                    << ", Score = " << AOutCandidates.back().Score << std::endl;

                Remaining.erase(Remaining.begin(),Remaining.begin() + static_cast<std::vector<int>::difference_type>(BestCount));
            }
        }

        bool CetCircleClusterBuilder::_BuildClusterCandidate(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate)
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
            for (int Index : Indices) {
                if (Index < 0 || Index >= static_cast<int>(AFeatures.size())) {
                    return false;
                }
            }
            const TetShapeFeature& BaseFeature = AFeatures[Indices.front()];
            if (!IsValidCircleFeature(BaseFeature)) {
                return false;
            }

            const double BaseSize = GetCircleSizeKey(BaseFeature);
            double CellSize = BaseSize;

            for (int Index : Indices) {
                const TetShapeFeature& Feature = AFeatures[Index];
                if (!IsValidCircleFeature(Feature)) {
                    return false;
                }

                const double SizeKey = GetCircleSizeKey(Feature);
                if (!NearlyEqual(BaseSize, SizeKey, CET_CIRCLE_SIZE_TOLERANCE)) {
                    return false;
                }

                CellSize = std::max(CellSize, SizeKey);
            }

            const double RequiredGap = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));
            const double SafetyGap = RequiredGap > 0.0 ? std::max(10.0, RequiredGap * 0.001) : 0.0;
            const double Gap = RequiredGap + SafetyGap;

            TetCircleLayout Layout;
            if (Indices.size() == 2) {
                Layout = MakePairLayout(CellSize, Gap);
            }
            else if (Indices.size() == 3) {
                Layout = MakeTriangleLayout(CellSize, Gap);
            }
            else if (Indices.size() == 4) {
                Layout = MakeSquareLayout(CellSize, Gap);
            }
            else {
                Layout = SelectBestHoneycombLayout(Indices.size(), CellSize, Gap, AOptions);
            }

            if (Layout.Slots.size() != Indices.size() || Layout.Width <= 0.0 || Layout.Height <= 0.0) {
                return false;
            }

            if (!FitsBin(Layout.Width, Layout.Height, AOptions)) {
                return false;
            }

            std::vector<TetItemTransform> Transforms;
            Transforms.reserve(Indices.size());

            for (std::size_t I = 0; I < Indices.size(); ++I) {
                const int Index = Indices[I];
                const TetShapeFeature& Feature = AFeatures[Index];
                const TetCircleLayoutSlot& Slot = Layout.Slots[I];

                TetItemTransform Transform;
                Transform.OriginalId = Index;
                Transform.RelativeRotation = 0.0;
                Transform.RelativeX = Slot.CenterX - (Feature.MinX + Feature.Width * 0.5);
                Transform.RelativeY = Slot.CenterY - (Feature.MinY + Feature.Height * 0.5);
                Transforms.push_back(Transform);
            }

            AOutCandidate.BuilderName = "CircleBuilder";
            AOutCandidate.ClusterType = Layout.ClusterType;
            AOutCandidate.OriginalIndices = Indices;
            AOutCandidate.Transforms = std::move(Transforms);
            AOutCandidate.Confidence = 1.0;

            CetClusterGeometryHelper Geometry;
            if (!Geometry.FinalizeCandidate(AOriginalItems, AOptions, AOutCandidate)) {
                return false;
            }

            AOutCandidate.Score = _CalculateScore(AOutCandidate);
            return true;
        }

        bool CetCircleClusterBuilder::_FitsBin(double AClusterWidth, double AClusterHeight, const TetNestOptions& AOptions)
        {
            return FitsBin(AClusterWidth, AClusterHeight, AOptions);
        }

        double CetCircleClusterBuilder::_CalculateScore(const TetClusterCandidate& ACandidate)
        {
            if (!ACandidate.Valid || ACandidate.OriginalIndices.empty() || ACandidate.ClusterWidth <= 0.0 || ACandidate.ClusterHeight <= 0.0 || ACandidate.ProxyArea <= 0.0) {
                return -std::numeric_limits<double>::infinity();
            }

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
