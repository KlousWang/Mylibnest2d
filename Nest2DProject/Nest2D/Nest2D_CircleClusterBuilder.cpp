#include "pch.h"
#include "Nest2D_CircleClusterBuilder.h"
#include "Nest2D_BoardUtils.h"
#include "Nest2D_ClusterGeometryHelper.h"
#include "Nest2D_ClusterMathUtils.h"
#include "Nest2D_SelfFunction.h"
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

            double GetCircleSizeKey(const TetShapeFeature &AFeature) { return std::max(AFeature.Width, AFeature.Height); }

            bool IsValidCircleFeature(const TetShapeFeature &AFeature) { return AFeature.ShapeType == MetShapeType::CircleLike && AFeature.Width > 0.0 && AFeature.Height > 0.0 && AFeature.Area > 0.0; }

            std::vector<std::vector<int>> GroupCircleIndices(const std::vector<int> &AIndices, const std::vector<TetShapeFeature> &AFeatures)
            {
                std::vector<TetCircleIndexInfo> Infos;
                Infos.reserve(AIndices.size());

                for (int Index : AIndices) {
                    if (Index < 0 || Index >= static_cast<int>(AFeatures.size())) {
                        continue;
                    }

                    const TetShapeFeature &Feature = AFeatures[Index];
                    if (!IsValidCircleFeature(Feature)) {
                        continue;
                    }

                    Infos.push_back({Index, GetCircleSizeKey(Feature)});
                }

                std::sort(Infos.begin(), Infos.end(), [](const TetCircleIndexInfo &A, const TetCircleIndexInfo &AB) {
                    if (A.SizeKey != AB.SizeKey) {
                        return A.SizeKey < AB.SizeKey;
                    }
                    return A.Index < AB.Index;
                });

                Infos.erase(std::unique(Infos.begin(), Infos.end(), [](const TetCircleIndexInfo &A, const TetCircleIndexInfo &AB) { return A.Index == AB.Index; }), Infos.end());

                std::vector<std::vector<int>> Groups;
                if (Infos.empty()) {
                    return Groups;
                }

                std::vector<int> CurrentGroup;
                double CurrentBaseSize = 0.0;

                for (const TetCircleIndexInfo &Info : Infos) {
                    if (CurrentGroup.empty()) {
                        CurrentBaseSize = Info.SizeKey;
                        CurrentGroup.push_back(Info.Index);
                        continue;
                    }

                    if (CetClusterMathUtils::NearlyEqual(CurrentBaseSize, Info.SizeKey, CET_CIRCLE_SIZE_TOLERANCE)) {
                        CurrentGroup.push_back(Info.Index);
                    } else {
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

            // A circle skeleton may be transposed without rotating any child
            // item.  This is different from rotating the finished cluster:
            // ROTATIONS limits each item's allowed pose, while the relative
            // direction of a group of rotationally symmetric circles is free.
            bool FitsBinDirectly(double AClusterWidth, double AClusterHeight, const TetNestOptions &AOptions)
            {
                if (AClusterWidth <= 0.0 || AClusterHeight <= 0.0)
                    return false;
                const double BinWidth = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth));
                const double BinHeight = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinHeight));
                return BinWidth > 0.0 && BinHeight > 0.0 && AClusterWidth <= BinWidth && AClusterHeight <= BinHeight;
            }

            void TransposeLayout(TetCircleLayout &ALayout)
            {
                for (TetCircleLayoutSlot &Slot : ALayout.Slots) {
                    std::swap(Slot.CenterX, Slot.CenterY);
                }
                std::swap(ALayout.Width, ALayout.Height);
            }

            void TransposeIfNeededForBin(TetCircleLayout &ALayout, const TetNestOptions &AOptions)
            {
                if (!FitsBinDirectly(ALayout.Width, ALayout.Height, AOptions) && FitsBinDirectly(ALayout.Height, ALayout.Width, AOptions)) {
                    TransposeLayout(ALayout);
                }
            }

            TetCircleLayout MakePairLayout(double ACellSize, double AGap)
            {
                TetCircleLayout Layout;
                const double Radius = ACellSize * 0.5;
                const double Step = ACellSize + AGap;

                Layout.Slots = {{Radius, Radius}, {Radius + Step, Radius}};
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

                Layout.Slots = {{Radius, Radius}, {Radius + Step, Radius}, {Radius + Step * 0.5, Radius + RowStep}};
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

                Layout.Slots = {{Radius, Radius}, {Radius + Step, Radius}, {Radius, Radius + Step}, {Radius + Step, Radius + Step}};
                Layout.Width = ACellSize * 2.0 + AGap;
                Layout.Height = ACellSize * 2.0 + AGap;
                Layout.ClusterType = "CircleBlock4";
                return Layout;
            }

            TetCircleLayout MakeThreeColumnSixCircleLayout(double ACellSize, double AGap)
            {
                TetCircleLayout Layout;
                const double Radius = ACellSize * 0.5;
                const double Step = ACellSize + AGap;

                Layout.Slots = {{Radius, Radius}, {Radius + Step, Radius}, {Radius + Step * 2.0, Radius}, {Radius, Radius + Step}, {Radius + Step, Radius + Step}, {Radius + Step * 2.0, Radius + Step}};
                Layout.Width = ACellSize * 3.0 + AGap * 2.0;
                Layout.Height = ACellSize * 2.0 + AGap;
                Layout.ClusterType = "CircleGrid_6_C3";
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
                        Layout.Slots.push_back({Radius + ShiftX + static_cast<double>(Col) * Step, CenterY});
                    }
                }

                if (Layout.Slots.size() != ACount) {
                    return {};
                }

                const double MinX = Radius;
                const double MinY = Radius;

                double MaxX = std::numeric_limits<double>::lowest();
                double MaxY = std::numeric_limits<double>::lowest();
                for (const TetCircleLayoutSlot &Slot : Layout.Slots) {
                    MaxX = std::max(MaxX, Slot.CenterX + Radius);
                    MaxY = std::max(MaxY, Slot.CenterY + Radius);
                }

                for (TetCircleLayoutSlot &Slot : Layout.Slots) {
                    Slot.CenterX -= MinX;
                    Slot.CenterY -= MinY;
                }

                Layout.Width = MaxX - MinX;
                Layout.Height = MaxY - MinY;
                Layout.ClusterType = "CircleHoneycomb_" + std::to_string(ACount) + "_R" + std::to_string(ARows);
                return Layout;
            }

            TetCircleLayout SelectBestHoneycombLayout(std::size_t ACount, double ACellSize, double AGap, const TetNestOptions &AOptions)
            {
                TetCircleLayout BestLayout;
                double BestCost = std::numeric_limits<double>::infinity();

                for (int Rows = 2; Rows <= static_cast<int>(ACount); ++Rows) {
                    TetCircleLayout Layout = MakeHoneycombLayout(ACount, Rows, ACellSize, AGap);
                    if (Layout.Slots.size() != ACount) {
                        continue;
                    }
                    
                    TransposeIfNeededForBin(Layout, AOptions);
                    if (!Nest2DUtils->Nest2DBord->FitsBin(Layout.Width, Layout.Height, AOptions)) {
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

            TetCircleLayout BuildCircleLayout(std::size_t ACount, double ACellSize, double AGap, const TetNestOptions &AOptions)
            {
                TetCircleLayout Layout;
                if (ACount == 2) {
                    Layout = MakePairLayout(ACellSize, AGap);
                } else if (ACount == 3) {
                    Layout = MakeTriangleLayout(ACellSize, AGap);
                } else if (ACount == 4) {
                    Layout = MakeSquareLayout(ACellSize, AGap);
                } else if (ACount == 6) {
                    // A 3x2 rectangle leaves predictable contour gaps for small
                    // fillers and stays narrower than a staggered 3+3 layout.
                    Layout = MakeThreeColumnSixCircleLayout(ACellSize, AGap);
                } else {
                    Layout = SelectBestHoneycombLayout(ACount, ACellSize, AGap, AOptions);
                }
                TransposeIfNeededForBin(Layout, AOptions);
                return Layout;
            }
        } // namespace

        CetCircleClusterBuilder::CetCircleClusterBuilder() : CetCoreObject() {}
        CetCircleClusterBuilder::~CetCircleClusterBuilder() {}

        void CetCircleClusterBuilder::BuildCandidates(const CetTNestItemVector &AOriginalItems, const std::vector<TetShapeFeature> &AFeatures, const std::vector<int> &AIndices, const TetNestOptions &AOptions, std::vector<TetClusterCandidate> &AOutCandidates)
        {
            if (AOriginalItems.empty()) {
                return;
            }

            if (AFeatures.size() != AOriginalItems.size()) {
                std::cout << "[CIRCLE][ERROR] Feature count mismatch. OriginalItems = " << AOriginalItems.size() << ", Features = " << AFeatures.size() << std::endl;
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

            for (std::size_t GroupIndex = 0; GroupIndex < Groups.size(); ++GroupIndex) {
                const std::vector<int> &Group = Groups[GroupIndex];
                std::vector<int> ClusterIndices = Group;

                // Keep a bounded tail of a substantially smaller circle family
                // as singles. These circles are useful for concave envelope
                // voids and for filling already-open sheets; a single large
                // honeycomb proxy cannot be inserted into either location.
                bool HasLargerCircleFamily = false;
                const double GroupSize = Group.empty() ? 0.0 : GetCircleSizeKey(AFeatures[Group.front()]);
                for (std::size_t LargerGroupIndex = GroupIndex + 1; LargerGroupIndex < Groups.size(); ++LargerGroupIndex) {
                    const std::vector<int> &LargerGroup = Groups[LargerGroupIndex];
                    if (LargerGroup.empty()) {
                        continue;
                    }
                    const double LargerSize = GetCircleSizeKey(AFeatures[LargerGroup.front()]);
                    if (GroupSize > 0.0 && LargerSize >= GroupSize * CET_CIRCLE_FILLER_MIN_SIZE_RATIO) {
                        HasLargerCircleFamily = true;
                        break;
                    }
                }

                if (HasLargerCircleFamily && Group.size() >= CET_CIRCLE_FILLER_SINGLE_RESERVE_MIN * 2) {
                    const std::size_t ReservedSingleCount = std::min(CET_CIRCLE_FILLER_SINGLE_RESERVE_MAX, std::max(CET_CIRCLE_FILLER_SINGLE_RESERVE_MIN, Group.size() / 4));
                    if (ClusterIndices.size() > ReservedSingleCount) {
                        ClusterIndices.resize(ClusterIndices.size() - ReservedSingleCount);
                        std::cout << "[CIRCLE][FILLER RESERVE] Diameter=" << GroupSize << ", Clustered=" << ClusterIndices.size() << ", Singles=" << ReservedSingleCount << std::endl;
                    }
                }

                _BuildSameSizeClusterCandidates(AOriginalItems, AFeatures, ClusterIndices, AOptions, AOutCandidates);
            }

            std::cout << "[CIRCLE][BUILD CANDIDATES] GroupCount = " << Groups.size() << ", NewCandidateCount = " << AOutCandidates.size() - OldCandidateCount << std::endl;
        }

        void CetCircleClusterBuilder::_BuildSameSizeClusterCandidates(const CetTNestItemVector &AOriginalItems, const std::vector<TetShapeFeature> &AFeatures, const std::vector<int> &AIndices, const TetNestOptions &AOptions, std::vector<TetClusterCandidate> &AOutCandidates)
        {
            if (AIndices.size() < 2) {
                return;
            }

            std::vector<int> Remaining = AIndices;
            std::sort(Remaining.begin(), Remaining.end());
            Remaining.erase(std::unique(Remaining.begin(), Remaining.end()), Remaining.end());
            const std::size_t BoardCapacity = _CalculatePeriodicBoardCapacity(AFeatures[Remaining.front()], AOptions);
            while (Remaining.size() >= 2) {
                const std::size_t PreferredCount = BoardCapacity >= CET_CIRCLE_PERIODIC_MIN_CHILD_COUNT ? std::min(BoardCapacity, Remaining.size()) : Remaining.size();
                TetClusterCandidate BestCandidate;
                std::size_t BestCount = 0;
                for (std::size_t Count = PreferredCount; Count >= 2; --Count) {
                    std::vector<int> TrialIndices(Remaining.begin(), Remaining.begin() + static_cast<std::vector<int>::difference_type>(Count));
                    TetClusterCandidate TrialCandidate;
                    if (_BuildClusterCandidate(AOriginalItems, AFeatures, TrialIndices, AOptions, TrialCandidate)) {
                        BestCount = Count;
                        BestCandidate = std::move(TrialCandidate);
                        break;
                    }
                    std::cout << "[FRAME][CIRCLE] Rejected ChildCount=" << Count << std::endl;
                }
                if (BestCount < 2)
                    return;
                AOutCandidates.push_back(std::move(BestCandidate));
                std::cout << "[FRAME][CIRCLE] Accepted BoardCapacity=" << BoardCapacity << ", ChildCount=" << BestCount << std::endl;
                Remaining.erase(Remaining.begin(), Remaining.begin() + static_cast<std::vector<int>::difference_type>(BestCount));
            }
        }

        std::size_t CetCircleClusterBuilder::_CalculatePeriodicBoardCapacity(const TetShapeFeature &AFeature, const TetNestOptions &AOptions)
        {
            const double CellSize = GetCircleSizeKey(AFeature);
            const double RequiredGap = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));
            const double SafetyGap = RequiredGap > 0.0 ? std::max(CET_CLUSTER_MIN_SAFETY_GAP, RequiredGap * 0.001) : 0.0;
            const double Step = CellSize + RequiredGap + SafetyGap;
            auto Calculate = [&](double AWidth, double AHeight) {
                const double RowStep = Step * CET_CIRCLE_HONEYCOMB_ROW_RATIO;
                if (CellSize <= 0.0 || Step <= 0.0 || RowStep <= 0.0 || AWidth < CellSize || AHeight < CellSize)
                    return std::size_t{0};
                const std::size_t RowCount = static_cast<std::size_t>(std::floor((AHeight - CellSize) / RowStep)) + 1;
                const std::size_t EvenColumns = static_cast<std::size_t>(std::floor((AWidth - CellSize) / Step)) + 1;
                const double OddWidth = AWidth - CellSize - Step * 0.5;
                const std::size_t OddColumns = OddWidth >= 0.0 ? static_cast<std::size_t>(std::floor(OddWidth / Step)) + 1 : 0;
                const std::size_t EvenRows = (RowCount + 1) / 2;
                const std::size_t OddRows = RowCount / 2;
                return EvenRows * EvenColumns + OddRows * OddColumns;
            };
            const double BinWidth = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth));
            const double BinHeight = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinHeight));
            return std::max(Calculate(BinWidth, BinHeight), Calculate(BinHeight, BinWidth));
        }

        bool CetCircleClusterBuilder::_BuildClusterCandidate(const CetTNestItemVector &AOriginalItems, const std::vector<TetShapeFeature> &AFeatures, const std::vector<int> &AIndices, const TetNestOptions &AOptions, TetClusterCandidate &AOutCandidate)
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
            const TetShapeFeature &BaseFeature = AFeatures[Indices.front()];
            if (!IsValidCircleFeature(BaseFeature)) {
                return false;
            }

            const double BaseSize = GetCircleSizeKey(BaseFeature);
            double CellSize = BaseSize;

            for (int Index : Indices) {
                const TetShapeFeature &Feature = AFeatures[Index];
                if (!IsValidCircleFeature(Feature)) {
                    return false;
                }

                const double SizeKey = GetCircleSizeKey(Feature);
                if (!CetClusterMathUtils::NearlyEqual(BaseSize, SizeKey, CET_CIRCLE_SIZE_TOLERANCE)) {
                    return false;
                }

                CellSize = std::max(CellSize, SizeKey);
            }

            const double RequiredGap = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));
            const double SafetyGap = RequiredGap > 0.0 ? std::max(CET_CLUSTER_MIN_SAFETY_GAP, RequiredGap * 0.001) : 0.0;
            const double Gap = RequiredGap + SafetyGap;

            TetCircleLayout Layout = BuildCircleLayout(Indices.size(), CellSize, Gap, AOptions);

            if (Layout.Slots.size() != Indices.size() || Layout.Width <= 0.0 || Layout.Height <= 0.0) {
                return false;
            }
            
            if (!Nest2DUtils->Nest2DBord->FitsBin(Layout.Width, Layout.Height, AOptions)) {
                return false;
            }

            std::vector<TetItemTransform> Transforms;
            Transforms.reserve(Indices.size());

            for (std::size_t I = 0; I < Indices.size(); ++I) {
                const int Index = Indices[I];
                const TetShapeFeature &Feature = AFeatures[Index];
                const TetCircleLayoutSlot &Slot = Layout.Slots[I];

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
            AOutCandidate.SkeletonChildCount = AOutCandidate.Transforms.size();
            AOutCandidate.Confidence = 1.0;

            CetClusterGeometryHelper Geometry;
            if (!Geometry.FinalizeCandidate(AOriginalItems, AOptions, AOutCandidate)) {
                return false;
            }

            AOutCandidate.Score = _CalculateScore(AOutCandidate);
            return true;
        }

        double CetCircleClusterBuilder::_CalculateScore(const TetClusterCandidate &ACandidate)
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

    } // namespace NEST2DMANAGERLIB
} // namespace ET
