#include "pch.h"
#include "Nest2D_RectangleGridOptimizer.h"
#include "Nest2D_SelfFunction.h"
#include "NestUtils.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <set>
namespace ET {
    namespace NEST2DMANAGERLIB {
        CetRectangleGridOptimizer::CetRectangleGridOptimizer() : CetCoreObject() {}
        CetRectangleGridOptimizer::~CetRectangleGridOptimizer() {}
        bool CetRectangleGridOptimizer::ValidatePlacedItemsSpacing(const CetTNestItemVector &AItems, const TetNestOptions &AOptions)
        {
            const auto SpacingCoord = NestUtils::ToNestCoord(std::max(0.0, AOptions.Spacing));
            for (std::size_t FirstIndex = 0; FirstIndex < AItems.size(); ++FirstIndex) {
                const CetNestItem &SourceItem = AItems[FirstIndex];
                if (SourceItem.binId() < 0) {
                    std::cout << "[NEST][SPACING][REJECT] Item " << FirstIndex << " was not placed on a bin." << std::endl;
                    return false;
                }
                CetNestItem FirstItem = SourceItem;
                FirstItem.inflation(0);
                if (SpacingCoord > 0)
                    FirstItem.inflation(static_cast<decltype(FirstItem.inflation())>(std::ceil(static_cast<double>(SpacingCoord) * 0.5)));
                for (std::size_t SecondIndex = FirstIndex + 1; SecondIndex < AItems.size(); ++SecondIndex) {
                    if (AItems[SecondIndex].binId() != SourceItem.binId())
                        continue;
                    CetNestItem SecondItem = AItems[SecondIndex];
                    SecondItem.inflation(0);
                    if (SpacingCoord > 0)
                        SecondItem.inflation(static_cast<decltype(SecondItem.inflation())>(std::ceil(static_cast<double>(SpacingCoord) * 0.5)));
                    if (CetNestItem::intersects(FirstItem, SecondItem) && !CetNestItem::touches(FirstItem, SecondItem)) {
                        std::cout << "[NEST][SPACING][REJECT] " << (SpacingCoord > 0 ? "Spacing violation" : "Overlap") << " between items " << FirstIndex << " and " << SecondIndex << " on bin " << SourceItem.binId() << std::endl;
                        return false;
                    }
                }
            }
            return true;
        }
        bool CetRectangleGridOptimizer::TryGetAxisAlignedRectangle(const CetNestItem &AItem, TetAxisAlignedRectangle &AOutRectangle)
        {
            CetNestItem Item = AItem;
            Item.inflation(0);
            const auto Bounds = Item.boundingBox();
            const double MinX = static_cast<double>(getX(Bounds.minCorner()));
            const double MinY = static_cast<double>(getY(Bounds.minCorner()));
            const double Width = static_cast<double>(Bounds.width());
            const double Height = static_cast<double>(Bounds.height());
            const double Area = std::abs(static_cast<double>(Item.area()));
            if (Width <= 0.0 || Height <= 0.0 || std::abs(Area - Width * Height) > std::max(1.0, Width * Height * 1e-9))
                return false;
            AOutRectangle.MinX = MinX;
            AOutRectangle.MinY = MinY;
            AOutRectangle.Width = Width;
            AOutRectangle.Height = Height;
            return true;
        }
        bool CetRectangleGridOptimizer::BuildRectangleGridGroup(const std::vector<TetAxisAlignedRectangle> &ARectangles, const TetNestOptions &AOptions, TetRectangleGridGroup &AOutGroup, bool ARequireMultipleRows)
        {
            AOutGroup = TetRectangleGridGroup{};
            if (ARectangles.size() < 2 || AOptions.Board.Enabled || AOptions.BinWidth <= 0.0 || AOptions.BinHeight <= 0.0)
                return false;
            const double Spacing = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));
            const double GridTolerance = std::max(1.0, static_cast<double>(NestUtils::ToNestCoord(0.01)));
            AOutGroup.Items = ARectangles;
            AOutGroup.OriginX = ARectangles.front().MinX;
            AOutGroup.OriginY = ARectangles.front().MinY;
            for (const TetAxisAlignedRectangle &Rectangle : ARectangles) {
                AOutGroup.OriginX = std::min(AOutGroup.OriginX, Rectangle.MinX);
                AOutGroup.OriginY = std::min(AOutGroup.OriginY, Rectangle.MinY);
            }
            AOutGroup.PitchX = ARectangles.front().Width + Spacing;
            AOutGroup.PitchY = ARectangles.front().Height + Spacing;
            if (AOutGroup.PitchX <= 0.0 || AOutGroup.PitchY <= 0.0)
                return false;
            const double BinWidth = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth));
            AOutGroup.ColumnCapacity = static_cast<std::size_t>(std::floor((BinWidth - AOutGroup.OriginX + Spacing) / AOutGroup.PitchX + 1e-9));
            if (AOutGroup.ColumnCapacity < 2)
                return false;
            std::map<long long, std::set<long long>> OccupiedCells;
            for (const TetAxisAlignedRectangle &Rectangle : ARectangles) {
                const long long Column = std::llround((Rectangle.MinX - AOutGroup.OriginX) / AOutGroup.PitchX);
                const long long Row = std::llround((Rectangle.MinY - AOutGroup.OriginY) / AOutGroup.PitchY);
                if (Column < 0 || static_cast<std::size_t>(Column) >= AOutGroup.ColumnCapacity)
                    return false;
                const double ExpectedX = AOutGroup.OriginX + static_cast<double>(Column) * AOutGroup.PitchX;
                const double ExpectedY = AOutGroup.OriginY + static_cast<double>(Row) * AOutGroup.PitchY;
                if (std::abs(Rectangle.MinX - ExpectedX) > GridTolerance || std::abs(Rectangle.MinY - ExpectedY) > GridTolerance)
                    return false;
                OccupiedCells[Row].insert(Column);
                AOutGroup.RowCoordinates[Row] = Rectangle.MinY;
            }
            if (ARequireMultipleRows && AOutGroup.RowCoordinates.size() < 2)
                return false;
            const long long FirstRow = AOutGroup.RowCoordinates.begin()->first;
            const long long LastRow = AOutGroup.RowCoordinates.rbegin()->first;
            for (long long Row = FirstRow; Row <= LastRow; ++Row) {
                const auto OccupiedIt = OccupiedCells.find(Row);
                for (std::size_t Column = 0; Column < AOutGroup.ColumnCapacity; ++Column) {
                    const bool Occupied = OccupiedIt != OccupiedCells.end() && OccupiedIt->second.count(static_cast<long long>(Column)) != 0;
                    if (!Occupied && Row != LastRow) {
                        ++AOutGroup.InternalGapCount;
                        AOutGroup.InternalGapArea += ARectangles.front().Width * ARectangles.front().Height;
                    }
                }
            }
            return true;
        }
        void CetRectangleGridOptimizer::EvaluateInternalGapMetrics(const CetTNestItemVector &AItems, const TetNestOptions &AOptions, TetTNestEvalResult &AInOutResult)
        {
            AInOutResult.HasInternalGapMetric = false;
            AInOutResult.InternalGapArea = 0.0;
            AInOutResult.InternalGapCount = 0;
            if (AOptions.Board.Enabled || AItems.empty())
                return;
            std::map<TetRectangleGridKey, std::vector<TetAxisAlignedRectangle>> Groups;
            for (std::size_t ItemIndex = 0; ItemIndex < AItems.size(); ++ItemIndex) {
                if (AItems[ItemIndex].binId() < 0)
                    continue;
                TetAxisAlignedRectangle Rectangle;
                if (!TryGetAxisAlignedRectangle(AItems[ItemIndex], Rectangle))
                    continue;
                Rectangle.ItemIndex = ItemIndex;
                Rectangle.BinId = AItems[ItemIndex].binId();
                const TetRectangleGridKey Key{Rectangle.BinId, std::llround(Rectangle.Width * 1000.0), std::llround(Rectangle.Height * 1000.0)};
                Groups[Key].push_back(Rectangle);
            }
            for (const auto &Entry : Groups) {
                TetRectangleGridGroup Group;
                if (!BuildRectangleGridGroup(Entry.second, AOptions, Group))
                    continue;
                AInOutResult.HasInternalGapMetric = true;
                AInOutResult.InternalGapArea += Group.InternalGapArea;
                AInOutResult.InternalGapCount += Group.InternalGapCount;
            }
        }
        bool CetRectangleGridOptimizer::AreItemsInsideRectangleBoard(const CetTNestItemVector &AItems, const TetNestOptions &AOptions)
        {
            const double BinWidth = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth));
            const double BinHeight = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinHeight));
            for (const CetNestItem &SourceItem : AItems) {
                if (SourceItem.binId() < 0)
                    return false;
                CetNestItem Item = SourceItem;
                Item.inflation(0);
                const auto Bounds = Item.boundingBox();
                if (static_cast<double>(getX(Bounds.minCorner())) < -1.0 || static_cast<double>(getY(Bounds.minCorner())) < -1.0 || static_cast<double>(getX(Bounds.maxCorner())) > BinWidth + 1.0 || static_cast<double>(getY(Bounds.maxCorner())) > BinHeight + 1.0)
                    return false;
            }
            return true;
        }
        bool CetRectangleGridOptimizer::TryCompactUniformRectangleHoles(CetTNestItemVector &AItems, const TetNestOptions &AOptions)
        {
            if (AOptions.Board.Enabled || AItems.empty())
                return false;
            std::set<int> UsedBins;
            for (const CetNestItem &Item : AItems)
                if (Item.binId() >= 0)
                    UsedBins.insert(Item.binId());
            if (UsedBins.size() != 1)
                return false;
            std::map<TetRectangleGridKey, std::vector<TetAxisAlignedRectangle>> Groups;
            for (std::size_t ItemIndex = 0; ItemIndex < AItems.size(); ++ItemIndex) {
                TetAxisAlignedRectangle Rectangle;
                if (!TryGetAxisAlignedRectangle(AItems[ItemIndex], Rectangle))
                    continue;
                Rectangle.ItemIndex = ItemIndex;
                Rectangle.BinId = AItems[ItemIndex].binId();
                const TetRectangleGridKey Key{Rectangle.BinId, std::llround(Rectangle.Width * 1000.0), std::llround(Rectangle.Height * 1000.0)};
                Groups[Key].push_back(Rectangle);
            }
            TetRectangleGridGroup BestGroup;
            for (const auto &Entry : Groups) {
                TetRectangleGridGroup Group;
                if (BuildRectangleGridGroup(Entry.second, AOptions, Group) && Group.InternalGapArea > BestGroup.InternalGapArea)
                    BestGroup = std::move(Group);
            }
            if (BestGroup.InternalGapCount == 0 || BestGroup.RowCoordinates.empty())
                return false;
            const std::size_t RequiredRows = (BestGroup.Items.size() + BestGroup.ColumnCapacity - 1) / BestGroup.ColumnCapacity;
            if (RequiredRows != BestGroup.RowCoordinates.size())
                return false;
            std::vector<TetAxisAlignedRectangle> OrderedItems = BestGroup.Items;
            std::stable_sort(OrderedItems.begin(), OrderedItems.end(), [](const TetAxisAlignedRectangle &A, const TetAxisAlignedRectangle &B) { return std::abs(A.MinY - B.MinY) > 1.0 ? A.MinY < B.MinY : A.MinX < B.MinX; });
            std::vector<double> RowCoordinates;
            for (const auto &Entry : BestGroup.RowCoordinates)
                RowCoordinates.push_back(Entry.second);
            TetTNestEvalResult Before{};
            EvaluateInternalGapMetrics(AItems, AOptions, Before);
            CetTNestItemVector Candidate = AItems;
            for (std::size_t Position = 0; Position < OrderedItems.size(); ++Position) {
                const std::size_t Row = Position / BestGroup.ColumnCapacity;
                const std::size_t Column = Position % BestGroup.ColumnCapacity;
                const double TargetX = BestGroup.OriginX + static_cast<double>(Column) * BestGroup.PitchX;
                const double TargetY = RowCoordinates[Row];
                const double DeltaX = TargetX - OrderedItems[Position].MinX;
                const double DeltaY = TargetY - OrderedItems[Position].MinY;
                auto Translation = Candidate[OrderedItems[Position].ItemIndex].translation();
                Candidate[OrderedItems[Position].ItemIndex].translation(ClipperLib::IntPoint(Translation.X + static_cast<ClipperLib::cInt>(std::llround(DeltaX)), Translation.Y + static_cast<ClipperLib::cInt>(std::llround(DeltaY))));
            }
            TetTNestEvalResult After{};
            EvaluateInternalGapMetrics(Candidate, AOptions, After);
            if (!After.HasInternalGapMetric || After.InternalGapArea >= Before.InternalGapArea - 1e-9 || !AreItemsInsideRectangleBoard(Candidate, AOptions) || !ValidatePlacedItemsSpacing(Candidate, AOptions))
                return false;
            std::cout << "[NEST][RECT_COMPACT] eligible=" << OrderedItems.size() << ", holes_before=" << Before.InternalGapCount << ", holes_after=" << After.InternalGapCount << ", applied=1" << std::endl;
            AItems = std::move(Candidate);
            return true;
        }
        std::size_t CetRectangleGridOptimizer::GetRectangleGridEdgeGapCount(const TetRectangleGridGroup &AGroup)
        {
            if (AGroup.ColumnCapacity == 0 || AGroup.Items.empty() || AGroup.InternalGapCount != 0)
                return 0;
            const std::size_t RequiredRows = (AGroup.Items.size() + AGroup.ColumnCapacity - 1) / AGroup.ColumnCapacity;
            if (RequiredRows != AGroup.RowCoordinates.size())
                return 0;
            const std::size_t LastRowItemCount = AGroup.Items.size() % AGroup.ColumnCapacity;
            return LastRowItemCount == 0 ? 0 : AGroup.ColumnCapacity - LastRowItemCount;
        }
        bool CetRectangleGridOptimizer::IsRectangleGridEdgeFillCandidateValid(const CetTNestItemVector &ACandidate, const TetNestOptions &AOptions)
        {
            TetTNestEvalResult Evaluation{};
            EvaluateInternalGapMetrics(ACandidate, AOptions, Evaluation);
            return AreItemsInsideRectangleBoard(ACandidate, AOptions) && ValidatePlacedItemsSpacing(ACandidate, AOptions) && (!Evaluation.HasInternalGapMetric || Evaluation.InternalGapArea <= 1e-9);
        }
        bool CetRectangleGridOptimizer::TryFillRectangleGridEdgeFromCompatibleGroup(CetTNestItemVector &AItems, const TetNestOptions &AOptions)
        {
            if (AOptions.Board.Enabled || AItems.empty())
                return false;
            std::set<int> UsedBins;
            std::map<TetRectangleGridKey, std::vector<TetAxisAlignedRectangle>> ExactGroups;
            for (std::size_t ItemIndex = 0; ItemIndex < AItems.size(); ++ItemIndex) {
                if (AItems[ItemIndex].binId() < 0)
                    return false;
                UsedBins.insert(AItems[ItemIndex].binId());
                TetAxisAlignedRectangle Rectangle;
                if (!TryGetAxisAlignedRectangle(AItems[ItemIndex], Rectangle))
                    continue;
                Rectangle.ItemIndex = ItemIndex;
                Rectangle.BinId = AItems[ItemIndex].binId();
                const TetRectangleGridKey Key{Rectangle.BinId, std::llround(Rectangle.Width * 1000.0), std::llround(Rectangle.Height * 1000.0)};
                ExactGroups[Key].push_back(Rectangle);
            }
            if (UsedBins.size() != 1)
                return false;
            std::map<TetRectangleFamilyKey, std::vector<TetRectangleGridGroup>> Families;
            for (const auto &Entry : ExactGroups) {
                TetRectangleGridGroup Group;
                if (!BuildRectangleGridGroup(Entry.second, AOptions, Group, false))
                    continue;
                const double ShortSide = std::min(Group.Items.front().Width, Group.Items.front().Height);
                const double LongSide = std::max(Group.Items.front().Width, Group.Items.front().Height);
                const TetRectangleFamilyKey FamilyKey{Entry.first.BinId, std::llround(ShortSide * 1000.0), std::llround(LongSide * 1000.0)};
                Families[FamilyKey].push_back(std::move(Group));
            }
            for (const auto &FamilyEntry : Families) {
                const std::vector<TetRectangleGridGroup> &FamilyGroups = FamilyEntry.second;
                for (std::size_t TargetIndex = 0; TargetIndex < FamilyGroups.size(); ++TargetIndex) {
                    const TetRectangleGridGroup &Target = FamilyGroups[TargetIndex];
                    const std::size_t TargetEdgeGapCount = GetRectangleGridEdgeGapCount(Target);
                    if (TargetEdgeGapCount == 0)
                        continue;
                    for (std::size_t DonorIndex = 0; DonorIndex < FamilyGroups.size(); ++DonorIndex) {
                        if (DonorIndex == TargetIndex)
                            continue;
                        const TetRectangleGridGroup &Donor = FamilyGroups[DonorIndex];
                        if (Donor.InternalGapCount != 0 || Donor.Items.size() <= TargetEdgeGapCount)
                            continue;
                        std::vector<TetAxisAlignedRectangle> OrderedDonors = Donor.Items;
                        std::stable_sort(OrderedDonors.begin(), OrderedDonors.end(), [](const TetAxisAlignedRectangle &A, const TetAxisAlignedRectangle &B) { return std::abs(A.MinY - B.MinY) > 1.0 ? A.MinY > B.MinY : A.MinX > B.MinX; });
                        if (OrderedDonors.size() < TargetEdgeGapCount || Target.RowCoordinates.empty())
                            continue;
                        const long long LastTargetRow = Target.RowCoordinates.rbegin()->first;
                        const double TargetY = Target.RowCoordinates.rbegin()->second;
                        std::set<long long> OccupiedColumns;
                        for (const TetAxisAlignedRectangle &Rectangle : Target.Items) {
                            const long long Row = std::llround((Rectangle.MinY - Target.OriginY) / Target.PitchY);
                            if (Row == LastTargetRow)
                                OccupiedColumns.insert(std::llround((Rectangle.MinX - Target.OriginX) / Target.PitchX));
                        }
                        std::vector<std::size_t> EmptyColumns;
                        for (std::size_t Column = 0; Column < Target.ColumnCapacity; ++Column)
                            if (OccupiedColumns.count(static_cast<long long>(Column)) == 0)
                                EmptyColumns.push_back(Column);
                        if (EmptyColumns.size() != TargetEdgeGapCount)
                            continue;
                        CetTNestItemVector Candidate = AItems;
                        const double TargetRotation = Candidate[Target.Items.front().ItemIndex].rotation();
                        for (std::size_t TransferIndex = 0; TransferIndex < TargetEdgeGapCount; ++TransferIndex) {
                            CetNestItem &Item = Candidate[OrderedDonors[TransferIndex].ItemIndex];
                            Item.inflation(0);
                            Item.rotation(TargetRotation);
                            const auto Bounds = Item.boundingBox();
                            const double TargetX = Target.OriginX + static_cast<double>(EmptyColumns[TransferIndex]) * Target.PitchX;
                            const auto Translation = Item.translation();
                            Item.translation(ClipperLib::IntPoint(Translation.X + static_cast<ClipperLib::cInt>(std::llround(TargetX - static_cast<double>(getX(Bounds.minCorner())))), Translation.Y + static_cast<ClipperLib::cInt>(std::llround(TargetY - static_cast<double>(getY(Bounds.minCorner()))))));
                        }
                        if (!IsRectangleGridEdgeFillCandidateValid(Candidate, AOptions))
                            continue;
                        std::cout << "[NEST][RECT_EDGE_FILL] transferred=" << TargetEdgeGapCount << ", target_edge_before=" << TargetEdgeGapCount << ", target_edge_after=0, applied=1" << std::endl;
                        AItems = std::move(Candidate);
                        return true;
                    }
                }
            }
            return false;
        }
    } // namespace NEST2DMANAGERLIB
} // namespace ET
