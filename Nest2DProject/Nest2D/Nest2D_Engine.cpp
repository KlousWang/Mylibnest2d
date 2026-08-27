
#include "pch.h"
#include "Nest2D_Engine.h"
#include "Nest2D_DataConst.h"
#include "Nest2D_PolygonBoardRepairer.h"
#include "Nest2D_PrivateDataType.h"
#include "Nest2D_RotationUtils.h"
#include "NestUtils.h"
#include "Nest2D_SelfFunction.h"
#include"Nest2D_ClusterManager.h"
#include"Nest2D_ShapeAnalyzer.h"
#include <map>
#include<vector>
#include<algorithm>
#include<limits>
#include<cmath>
#include<array>
#include<numeric>
#include<set>
#include<chrono>
#include<tuple>

//#include"libnest2d/optimizers/nlopt/subplex.hpp"

using namespace ClipperLib;
using namespace libnest2d;
namespace ET {
	namespace NEST2DMANAGERLIB {

		CetNest2DEngine::CetNest2DEngine() :CetCoreObject()
		{
		}
		CetNest2DEngine::~CetNest2DEngine()
		{
		}
		static void FillRotations(std::vector<libnest2d::Radians>& ARotations, int ARotationCount)
		{
			ARotations = CetRotationUtils::BuildAllowedLibRotations(ARotationCount);
		}

		template <typename TSelector>
		static std::size_t RunRectangleNestWithSelector(CetTNestItemVector& AItems,
			const TetNestOptions& AOptions, TetNestProgressTracker& ATracker, bool AAllowRotations)
		{
			const auto Width = NestUtils::ToNestCoord(AOptions.BinWidth);
			const auto Height = NestUtils::ToNestCoord(AOptions.BinHeight);
			Box Bin(Width, Height, { Width / 2, Height / 2 });
			using TPlacer = placers::_BottomLeftPlacer<CetPolygonImpl>;
			NestConfig<TPlacer, TSelector> Config;
			Config.placer_config.min_obj_distance = NestUtils::ToNestCoord(AOptions.Spacing);
			Config.placer_config.epsilon = 1;
			Config.placer_config.allow_rotations = AAllowRotations && CetRotationUtils::IsAllowedRotation(
				CET_CLUSTER_HALF_PI, AOptions.Rotations, 1e-9);
			std::cout << "================ DEBUG INFO ================" << std::endl;
			std::cout << "UsePolygonBoard: false" << std::endl;
			std::cout << "Bin Width: " << Bin.width() << ", Height: " << Bin.height() << std::endl;
			std::cout << "Spacing: " << NestUtils::ToNestCoord(AOptions.Spacing) << std::endl;
			std::cout << "============================================" << std::endl;
			const std::size_t Layers = nest(AItems, Bin, NestUtils::ToNestCoord(AOptions.Spacing),
				Config, ProgressFunction{ ATracker });
			std::cout << "[NEST] Layers = " << Layers << std::endl;
			Nest2DUtils->Nest2DStrategy->PrintBinCount(AItems);
			return Layers;
		}

		static std::size_t RunRectangleNestFromOppositeEdge(CetTNestItemVector& AItems,
			const TetNestOptions& AOptions, TetNestProgressTracker& ATracker)
		{
			const auto Width = NestUtils::ToNestCoord(AOptions.BinWidth);
			const auto Height = NestUtils::ToNestCoord(AOptions.BinHeight);
			Box Bin(Width, Height, { Width / 2, Height / 2 });
			using TPlacer = placers::_NofitPolyPlacer<CetPolygonImpl>;
			using TSelector = selections::_FirstFitSelection<CetPolygonImpl>;
			NestConfig<TPlacer, TSelector> Config;
			Config.placer_config.accuracy = AOptions.Placer.Accuracy;
			Config.placer_config.parallel = AOptions.Placer.Parallel;
			Config.placer_config.explore_holes = false;
			FillRotations(Config.placer_config.rotations, AOptions.Rotations);
			Config.placer_config.alignment = placers::NfpPConfig<CetPolygonImpl>::Alignment::TOP_RIGHT;
			Config.placer_config.starting_point = placers::NfpPConfig<CetPolygonImpl>::Alignment::TOP_RIGHT;
			const std::size_t Layers = nest(AItems, Bin, NestUtils::ToNestCoord(AOptions.Spacing),
				Config, ProgressFunction{ ATracker });
			std::cout << "[NEST][OPPOSITE EDGE] Layers=" << Layers << std::endl;
			Nest2DUtils->Nest2DStrategy->PrintBinCount(AItems);
			return Layers;
		}

		static void ApplyClusterEdgeClearance(CetTNestItemVector& AItems, const std::vector<TetMetaItem>& AMetaItems,
			const TetNestOptions& AOptions)
		{
			const auto Clearance = static_cast<libnest2d::Coord>(std::ceil(
				static_cast<double>(NestUtils::ToNestCoord(std::max(0.0, AOptions.Spacing))) * 0.5));
			for (std::size_t Index = 0; Index < AItems.size() && Index < AMetaItems.size(); ++Index) {
				if (AMetaItems[Index].IsCluster) AItems[Index].inflation(Clearance);
			}
		}

		static void ClearItemInflation(CetTNestItemVector& AItems)
		{
			for (CetNestItem& Item : AItems) Item.inflation(0);
		}
		static placers::NfpPConfig<CetPolygonImpl>::Alignment ToLibNestAlignment(MetNestAlignment AAlignment)
		{
			using CetAlignment = placers::NfpPConfig<CetPolygonImpl>::Alignment;

			switch (AAlignment){
			case MetNestAlignment::DontAlign:
				return CetAlignment::DONT_ALIGN;

			case MetNestAlignment::BottomLeft:
			default:
				return CetAlignment::BOTTOM_LEFT;
			}
		}

		static bool ValidatePlacedItemsSpacing(const CetTNestItemVector& AItems, const TetNestOptions& AOptions)
		{
			const auto SpacingCoord = NestUtils::ToNestCoord(std::max(0.0, AOptions.Spacing));
			for (std::size_t FirstIndex = 0; FirstIndex < AItems.size(); ++FirstIndex){
				const CetNestItem& SourceItem = AItems[FirstIndex];
				if (SourceItem.binId() < 0){
					std::cout << "[NEST][SPACING][REJECT] Item " << FirstIndex << " was not placed on a bin." << std::endl;
					return false;
				}
				CetNestItem FirstItem = SourceItem;
				FirstItem.inflation(0);
				if (SpacingCoord > 0){
					FirstItem.inflation(static_cast<decltype(FirstItem.inflation())>(std::ceil(static_cast<double>(SpacingCoord) * 0.5)));
				}
				for (std::size_t SecondIndex = FirstIndex + 1; SecondIndex < AItems.size(); ++SecondIndex){
					if (AItems[SecondIndex].binId() != SourceItem.binId()){
						continue;
					}
					CetNestItem SecondItem = AItems[SecondIndex];
					SecondItem.inflation(0);
					if (SpacingCoord > 0){
						SecondItem.inflation(static_cast<decltype(SecondItem.inflation())>(std::ceil(static_cast<double>(SpacingCoord) * 0.5)));
					}
					// libnest2d packs every item with half of the requested spacing.
					// At exactly the requested clearance those expanded outlines touch;
					// touching is legal, while an interior intersection is not.
					if (CetNestItem::intersects(FirstItem, SecondItem) && !CetNestItem::touches(FirstItem, SecondItem)){
						std::cout << "[NEST][SPACING][REJECT] "
							<< (SpacingCoord > 0 ? "Spacing violation" : "Overlap")
							<< " between items " << FirstIndex << " and " << SecondIndex
							<< " on bin " << SourceItem.binId() << std::endl;
						return false;
					}
				}
			}
			return true;
		}

		struct TetAxisAlignedRectangle
		{
			std::size_t ItemIndex = 0;
			int BinId = -1;
			double MinX = 0.0;
			double MinY = 0.0;
			double Width = 0.0;
			double Height = 0.0;
		};

		struct TetRectangleGridKey
		{
			int BinId = -1;
			long long WidthKey = 0;
			long long HeightKey = 0;

			bool operator<(const TetRectangleGridKey& AOther) const
			{
				return std::tie(BinId, WidthKey, HeightKey) < std::tie(AOther.BinId, AOther.WidthKey, AOther.HeightKey);
			}
		};

		struct TetRectangleFamilyKey
		{
			int BinId = -1;
			long long ShortSideKey = 0;
			long long LongSideKey = 0;

			bool operator<(const TetRectangleFamilyKey& AOther) const
			{
				return std::tie(BinId, ShortSideKey, LongSideKey) < std::tie(AOther.BinId, AOther.ShortSideKey, AOther.LongSideKey);
			}
		};

		struct TetRectangleGridGroup
		{
			std::vector<TetAxisAlignedRectangle> Items;
			double OriginX = 0.0;
			double OriginY = 0.0;
			double PitchX = 0.0;
			double PitchY = 0.0;
			std::size_t ColumnCapacity = 0;
			std::map<long long, double> RowCoordinates;
			std::size_t InternalGapCount = 0;
			double InternalGapArea = 0.0;
		};

		static bool TryGetAxisAlignedRectangle(const CetNestItem& AItem, TetAxisAlignedRectangle& AOutRectangle)
		{
			CetNestItem Item = AItem;
			Item.inflation(0);
			const auto Bounds = Item.boundingBox();
			const double MinX = static_cast<double>(getX(Bounds.minCorner()));
			const double MinY = static_cast<double>(getY(Bounds.minCorner()));
			const double Width = static_cast<double>(Bounds.width());
			const double Height = static_cast<double>(Bounds.height());
			const double Area = std::abs(static_cast<double>(Item.area()));
			if (Width <= 0.0 || Height <= 0.0 || std::abs(Area - Width * Height) > std::max(1.0, Width * Height * 1e-9)) {
				return false;
			}

			AOutRectangle.MinX = MinX;
			AOutRectangle.MinY = MinY;
			AOutRectangle.Width = Width;
			AOutRectangle.Height = Height;
			return true;
		}

		static bool BuildRectangleGridGroup(const std::vector<TetAxisAlignedRectangle>& ARectangles, const TetNestOptions& AOptions, TetRectangleGridGroup& AOutGroup, bool ARequireMultipleRows = true)
		{
			AOutGroup = TetRectangleGridGroup{};
			if (ARectangles.size() < 2 || AOptions.Board.Enabled || AOptions.BinWidth <= 0.0 || AOptions.BinHeight <= 0.0) {
				return false;
			}

			const double Spacing = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));
			// Cluster expansion and rotation can introduce sub-0.01 mm coordinate drift.
			// Retain a tight grid test while accepting the precision used by saved layouts.
			const double GridTolerance = std::max(1.0, static_cast<double>(NestUtils::ToNestCoord(0.01)));
			AOutGroup.Items = ARectangles;
			AOutGroup.OriginX = ARectangles.front().MinX;
			AOutGroup.OriginY = ARectangles.front().MinY;
			for (const TetAxisAlignedRectangle& Rectangle : ARectangles) {
				AOutGroup.OriginX = std::min(AOutGroup.OriginX, Rectangle.MinX);
				AOutGroup.OriginY = std::min(AOutGroup.OriginY, Rectangle.MinY);
			}
			AOutGroup.PitchX = ARectangles.front().Width + Spacing;
			AOutGroup.PitchY = ARectangles.front().Height + Spacing;
			if (AOutGroup.PitchX <= 0.0 || AOutGroup.PitchY <= 0.0) {
				return false;
			}

			const double BinWidth = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth));
			AOutGroup.ColumnCapacity = static_cast<std::size_t>(std::floor((BinWidth - AOutGroup.OriginX + Spacing) / AOutGroup.PitchX + 1e-9));
			if (AOutGroup.ColumnCapacity < 2) {
				return false;
			}

			std::map<long long, std::set<long long>> OccupiedCells;
			for (const TetAxisAlignedRectangle& Rectangle : ARectangles) {
				const long long Column = std::llround((Rectangle.MinX - AOutGroup.OriginX) / AOutGroup.PitchX);
				const long long Row = std::llround((Rectangle.MinY - AOutGroup.OriginY) / AOutGroup.PitchY);
				if (Column < 0 || static_cast<std::size_t>(Column) >= AOutGroup.ColumnCapacity) {
					return false;
				}
				const double ExpectedX = AOutGroup.OriginX + static_cast<double>(Column) * AOutGroup.PitchX;
				const double ExpectedY = AOutGroup.OriginY + static_cast<double>(Row) * AOutGroup.PitchY;
				if (std::abs(Rectangle.MinX - ExpectedX) > GridTolerance || std::abs(Rectangle.MinY - ExpectedY) > GridTolerance) {
					return false;
				}
				OccupiedCells[Row].insert(Column);
				AOutGroup.RowCoordinates[Row] = Rectangle.MinY;
			}

			if (ARequireMultipleRows && AOutGroup.RowCoordinates.size() < 2) {
				return false;
			}

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

		static void EvaluateInternalGapMetrics(const CetTNestItemVector& AItems, const TetNestOptions& AOptions, TetTNestEvalResult& AInOutResult)
		{
			AInOutResult.HasInternalGapMetric = false;
			AInOutResult.InternalGapArea = 0.0;
			AInOutResult.InternalGapCount = 0;
			if (AOptions.Board.Enabled || AItems.empty()) {
				return;
			}

			std::map<TetRectangleGridKey, std::vector<TetAxisAlignedRectangle>> Groups;
			for (std::size_t ItemIndex = 0; ItemIndex < AItems.size(); ++ItemIndex) {
				if (AItems[ItemIndex].binId() < 0) {
					continue;
				}
				TetAxisAlignedRectangle Rectangle;
				if (!TryGetAxisAlignedRectangle(AItems[ItemIndex], Rectangle)) {
					continue;
				}
				Rectangle.ItemIndex = ItemIndex;
				Rectangle.BinId = AItems[ItemIndex].binId();
				const TetRectangleGridKey Key{ Rectangle.BinId, std::llround(Rectangle.Width * 1000.0), std::llround(Rectangle.Height * 1000.0) };
				Groups[Key].push_back(Rectangle);
			}

			for (const auto& Entry : Groups) {
				TetRectangleGridGroup Group;
				if (!BuildRectangleGridGroup(Entry.second, AOptions, Group)) {
					continue;
				}
				AInOutResult.HasInternalGapMetric = true;
				AInOutResult.InternalGapArea += Group.InternalGapArea;
				AInOutResult.InternalGapCount += Group.InternalGapCount;
			}
		}

		static bool AreItemsInsideRectangleBoard(const CetTNestItemVector& AItems, const TetNestOptions& AOptions)
		{
			const double BinWidth = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth));
			const double BinHeight = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinHeight));
			for (const CetNestItem& SourceItem : AItems) {
				if (SourceItem.binId() < 0) {
					return false;
				}
				CetNestItem Item = SourceItem;
				Item.inflation(0);
				const auto Bounds = Item.boundingBox();
				if (static_cast<double>(getX(Bounds.minCorner())) < -1.0 || static_cast<double>(getY(Bounds.minCorner())) < -1.0
					|| static_cast<double>(getX(Bounds.maxCorner())) > BinWidth + 1.0 || static_cast<double>(getY(Bounds.maxCorner())) > BinHeight + 1.0) {
					return false;
				}
			}
			return true;
		}

		static bool TryCompactUniformRectangleHoles(CetTNestItemVector& AItems, const TetNestOptions& AOptions)
		{
			if (AOptions.Board.Enabled || AItems.empty()) {
				return false;
			}

			std::set<int> UsedBins;
			for (const CetNestItem& Item : AItems) {
				if (Item.binId() >= 0) {
					UsedBins.insert(Item.binId());
				}
			}
			if (UsedBins.size() != 1) {
				return false;
			}

			std::map<TetRectangleGridKey, std::vector<TetAxisAlignedRectangle>> Groups;
			for (std::size_t ItemIndex = 0; ItemIndex < AItems.size(); ++ItemIndex) {
				TetAxisAlignedRectangle Rectangle;
				if (!TryGetAxisAlignedRectangle(AItems[ItemIndex], Rectangle)) {
					continue;
				}
				Rectangle.ItemIndex = ItemIndex;
				Rectangle.BinId = AItems[ItemIndex].binId();
				const TetRectangleGridKey Key{ Rectangle.BinId, std::llround(Rectangle.Width * 1000.0), std::llround(Rectangle.Height * 1000.0) };
				Groups[Key].push_back(Rectangle);
			}

			TetRectangleGridGroup BestGroup;
			for (const auto& Entry : Groups) {
				TetRectangleGridGroup Group;
				if (BuildRectangleGridGroup(Entry.second, AOptions, Group) && Group.InternalGapArea > BestGroup.InternalGapArea) {
					BestGroup = std::move(Group);
				}
			}
			if (BestGroup.InternalGapCount == 0 || BestGroup.RowCoordinates.empty()) {
				return false;
			}

			const std::size_t RequiredRows = (BestGroup.Items.size() + BestGroup.ColumnCapacity - 1) / BestGroup.ColumnCapacity;
			if (RequiredRows != BestGroup.RowCoordinates.size()) {
				return false;
			}

			std::vector<TetAxisAlignedRectangle> OrderedItems = BestGroup.Items;
			std::stable_sort(OrderedItems.begin(), OrderedItems.end(), [](const TetAxisAlignedRectangle& A, const TetAxisAlignedRectangle& B) {
				if (std::abs(A.MinY - B.MinY) > 1.0) {
					return A.MinY < B.MinY;
				}
				return A.MinX < B.MinX;
			});
			std::vector<double> RowCoordinates;
			for (const auto& Entry : BestGroup.RowCoordinates) {
				RowCoordinates.push_back(Entry.second);
			}

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
				Candidate[OrderedItems[Position].ItemIndex].translation(ClipperLib::IntPoint(
					Translation.X + static_cast<ClipperLib::cInt>(std::llround(DeltaX)),
					Translation.Y + static_cast<ClipperLib::cInt>(std::llround(DeltaY))));
			}

			TetTNestEvalResult After{};
			EvaluateInternalGapMetrics(Candidate, AOptions, After);
			if (!After.HasInternalGapMetric || After.InternalGapArea >= Before.InternalGapArea - 1e-9
				|| !AreItemsInsideRectangleBoard(Candidate, AOptions) || !ValidatePlacedItemsSpacing(Candidate, AOptions)) {
				return false;
			}

			std::cout << "[NEST][RECT_COMPACT] eligible=" << OrderedItems.size()
				<< ", holes_before=" << Before.InternalGapCount
				<< ", holes_after=" << After.InternalGapCount
				<< ", applied=1" << std::endl;
			AItems = std::move(Candidate);
			return true;
		}

		static std::size_t GetRectangleGridEdgeGapCount(const TetRectangleGridGroup& AGroup)
		{
			if (AGroup.ColumnCapacity == 0 || AGroup.Items.empty() || AGroup.InternalGapCount != 0) {
				return 0;
			}
			const std::size_t RequiredRows = (AGroup.Items.size() + AGroup.ColumnCapacity - 1) / AGroup.ColumnCapacity;
			if (RequiredRows != AGroup.RowCoordinates.size()) {
				return 0;
			}
			const std::size_t LastRowItemCount = AGroup.Items.size() % AGroup.ColumnCapacity;
			return LastRowItemCount == 0 ? 0 : AGroup.ColumnCapacity - LastRowItemCount;
		}

		static bool TryFillRectangleGridEdgeFromCompatibleGroup(CetTNestItemVector& AItems, const TetNestOptions& AOptions)
		{
			if (AOptions.Board.Enabled || AItems.empty()) {
				return false;
			}

			std::set<int> UsedBins;
			std::map<TetRectangleGridKey, std::vector<TetAxisAlignedRectangle>> ExactGroups;
			for (std::size_t ItemIndex = 0; ItemIndex < AItems.size(); ++ItemIndex) {
				if (AItems[ItemIndex].binId() < 0) {
					return false;
				}
				UsedBins.insert(AItems[ItemIndex].binId());
				TetAxisAlignedRectangle Rectangle;
				if (!TryGetAxisAlignedRectangle(AItems[ItemIndex], Rectangle)) {
					continue;
				}
				Rectangle.ItemIndex = ItemIndex;
				Rectangle.BinId = AItems[ItemIndex].binId();
				const TetRectangleGridKey Key{ Rectangle.BinId, std::llround(Rectangle.Width * 1000.0), std::llround(Rectangle.Height * 1000.0) };
				ExactGroups[Key].push_back(Rectangle);
			}
			if (UsedBins.size() != 1) {
				return false;
			}

			std::map<TetRectangleFamilyKey, std::vector<TetRectangleGridGroup>> Families;
			for (const auto& Entry : ExactGroups) {
				TetRectangleGridGroup Group;
				if (!BuildRectangleGridGroup(Entry.second, AOptions, Group, false)) {
					continue;
				}
				const double ShortSide = std::min(Group.Items.front().Width, Group.Items.front().Height);
				const double LongSide = std::max(Group.Items.front().Width, Group.Items.front().Height);
				const TetRectangleFamilyKey FamilyKey{ Entry.first.BinId, std::llround(ShortSide * 1000.0), std::llround(LongSide * 1000.0) };
				Families[FamilyKey].push_back(std::move(Group));
			}

			for (const auto& FamilyEntry : Families) {
				const std::vector<TetRectangleGridGroup>& FamilyGroups = FamilyEntry.second;
				for (std::size_t TargetIndex = 0; TargetIndex < FamilyGroups.size(); ++TargetIndex) {
					const TetRectangleGridGroup& Target = FamilyGroups[TargetIndex];
					const std::size_t TargetEdgeGapCount = GetRectangleGridEdgeGapCount(Target);
					if (TargetEdgeGapCount == 0) {
						continue;
					}

					for (std::size_t DonorIndex = 0; DonorIndex < FamilyGroups.size(); ++DonorIndex) {
						if (DonorIndex == TargetIndex) {
							continue;
						}
						const TetRectangleGridGroup& Donor = FamilyGroups[DonorIndex];
						if (Donor.InternalGapCount != 0 || Donor.Items.size() <= TargetEdgeGapCount) {
							continue;
						}

						std::vector<TetAxisAlignedRectangle> OrderedDonors = Donor.Items;
						std::stable_sort(OrderedDonors.begin(), OrderedDonors.end(), [](const TetAxisAlignedRectangle& A, const TetAxisAlignedRectangle& B) {
							if (std::abs(A.MinY - B.MinY) > 1.0) {
								return A.MinY > B.MinY;
							}
							return A.MinX > B.MinX;
						});
						if (OrderedDonors.size() < TargetEdgeGapCount || Target.RowCoordinates.empty()) {
							continue;
						}

						const long long LastTargetRow = Target.RowCoordinates.rbegin()->first;
						const double TargetY = Target.RowCoordinates.rbegin()->second;
						std::set<long long> OccupiedColumns;
						for (const TetAxisAlignedRectangle& Rectangle : Target.Items) {
							const long long Row = std::llround((Rectangle.MinY - Target.OriginY) / Target.PitchY);
							if (Row == LastTargetRow) {
								OccupiedColumns.insert(std::llround((Rectangle.MinX - Target.OriginX) / Target.PitchX));
							}
						}

						std::vector<std::size_t> EmptyColumns;
						for (std::size_t Column = 0; Column < Target.ColumnCapacity; ++Column) {
							if (OccupiedColumns.count(static_cast<long long>(Column)) == 0) {
								EmptyColumns.push_back(Column);
							}
						}
						if (EmptyColumns.size() != TargetEdgeGapCount) {
							continue;
						}

						CetTNestItemVector Candidate = AItems;
						const double TargetRotation = Candidate[Target.Items.front().ItemIndex].rotation();
						for (std::size_t TransferIndex = 0; TransferIndex < TargetEdgeGapCount; ++TransferIndex) {
							CetNestItem& Item = Candidate[OrderedDonors[TransferIndex].ItemIndex];
							Item.inflation(0);
							Item.rotation(TargetRotation);
							const auto Bounds = Item.boundingBox();
							const double TargetX = Target.OriginX + static_cast<double>(EmptyColumns[TransferIndex]) * Target.PitchX;
							const auto Translation = Item.translation();
							Item.translation(ClipperLib::IntPoint(
								Translation.X + static_cast<ClipperLib::cInt>(std::llround(TargetX - static_cast<double>(getX(Bounds.minCorner())))),
								Translation.Y + static_cast<ClipperLib::cInt>(std::llround(TargetY - static_cast<double>(getY(Bounds.minCorner()))))));
						}

						TetTNestEvalResult After{};
						EvaluateInternalGapMetrics(Candidate, AOptions, After);
						const bool IsInsideBoard = AreItemsInsideRectangleBoard(Candidate, AOptions);
						const bool HasValidSpacing = IsInsideBoard && ValidatePlacedItemsSpacing(Candidate, AOptions);
						const bool HasNoInternalGap = !After.HasInternalGapMetric || After.InternalGapArea <= 1e-9;
						if (!IsInsideBoard || !HasValidSpacing || !HasNoInternalGap) {
							continue;
						}

						std::cout << "[NEST][RECT_EDGE_FILL] transferred=" << TargetEdgeGapCount
							<< ", target_edge_before=" << TargetEdgeGapCount
							<< ", target_edge_after=0"
							<< ", applied=1" << std::endl;
						AItems = std::move(Candidate);
						return true;
					}
				}
			}

			return false;
		}

		static bool IsLockedEnvelopeCluster(const TetMetaItem& AMeta)
		{
			if (!AMeta.IsCluster) return false;
			const bool IsCircleEnvelope = AMeta.ClusterType.find("Circle") == 0;
			const bool IsFilledEllipseEnvelope = AMeta.ClusterType.find("Ellipse") == 0
				&& AMeta.ClusterType.find("_EnvelopeFill") != std::string::npos;
			return IsCircleEnvelope || IsFilledEllipseEnvelope;
		}

		static bool HasLockedEnvelopeCluster(const std::vector<TetMetaItem>& AMetaItems)
		{
			for (const TetMetaItem& Meta : AMetaItems){
				if (IsLockedEnvelopeCluster(Meta)) return true;
			}
			return false;
		}

		static std::vector<std::size_t> CollectLockedEnvelopeChildren(const std::vector<TetMetaItem>& AMetaItems)
		{
			std::vector<std::size_t> Indices;
			for (const TetMetaItem& Meta : AMetaItems) {
				if (!IsLockedEnvelopeCluster(Meta)) continue;
				for (const TetItemTransform& Transform : Meta.TransformData) if (Transform.OriginalId >= 0) Indices.push_back(static_cast<std::size_t>(Transform.OriginalId));
			}
			std::sort(Indices.begin(), Indices.end());
			Indices.erase(std::unique(Indices.begin(), Indices.end()), Indices.end());
			return Indices;
		}

		static bool PreservesLockedChildren(const CetTNestItemVector& ABefore, const CetTNestItemVector& AAfter, const std::vector<std::size_t>& AIndices)
		{
			if (ABefore.size() != AAfter.size()) return false;
			for (std::size_t Index : AIndices) {
				if (Index >= ABefore.size() || Index >= AAfter.size()) return false;
				const Point BeforePoint = ABefore[Index].translation();
				const Point AfterPoint = AAfter[Index].translation();
				if (BeforePoint.X != AfterPoint.X || BeforePoint.Y != AfterPoint.Y
					|| std::abs(static_cast<double>(ABefore[Index].rotation()) - static_cast<double>(AAfter[Index].rotation())) > CET_CLUSTER_FILL_VARIANT_ROTATION_TOLERANCE) return false;
			}
			return true;
		}

		// Refill sheets with complete cluster proxies. Children stay glued through
		// their metadata and are expanded only after this multi-sheet pass.
		using ClusterBackfillPlacer = placers::_BottomLeftPlacer<CetPolygonImpl>;
		using ClusterBackfillConfig = placers::BLConfig<CetPolygonImpl>;

		bool RepackClusterItems(CetTNestItemVector& AItems, const std::vector<std::size_t>& AIndices, const Box& ABin, const ClusterBackfillConfig& AConfig, int ABinId, long long AInflation)
		{
			std::vector<std::size_t> OrderedIndices = AIndices;
			std::stable_sort(OrderedIndices.begin(), OrderedIndices.end(), [&](std::size_t A, std::size_t B) {
				return std::abs(static_cast<double>(AItems[A].area())) > std::abs(static_cast<double>(AItems[B].area()));
				});
			CetTNestItemVector Repacked;
			Repacked.reserve(OrderedIndices.size());
			for (std::size_t Index : OrderedIndices){
				CetNestItem Copy = AItems[Index];
				Copy.translation(ClipperLib::IntPoint(0,0));
				Copy.rotation(0.0);
				Copy.inflation(static_cast<decltype(Copy.inflation())>(AInflation));
				Repacked.push_back(std::move(Copy));
			}
			ClusterBackfillPlacer Repacker(ABin);
			Repacker.configure(AConfig);
			for (CetNestItem& Item : Repacked){
				if (!Repacker.pack(Item)) return false;
				Item.binId(ABinId);
				Item.inflation(0);
			}
			for (std::size_t Position = 0; Position < OrderedIndices.size(); ++Position) AItems[OrderedIndices[Position]] = std::move(Repacked[Position]);
			return true;
		}

		static std::size_t BackfillClusterSheets(CetTNestItemVector& AItems, const TetNestOptions& AOptions, std::size_t ALayers)
		{
			if (ALayers <= 1 || AItems.empty()) return ALayers;
			if (AItems.size() > CET_NEST_FULL_STRATEGY_ITEM_LIMIT) {
				// This optional repack repeatedly invokes the bottom-left placer. Once
				// local spacing fallback has expanded a large proxy set, doing it here
				// can cost minutes while not affecting the already valid main nest.
				std::cout << "[NEST][CLUSTER BACKFILL][SKIP] PackedItems=" << AItems.size()
					<< ", Limit=" << CET_NEST_FULL_STRATEGY_ITEM_LIMIT << std::endl;
				return ALayers;
			}
			const auto Width = NestUtils::ToNestCoord(AOptions.BinWidth);
			const auto Height = NestUtils::ToNestCoord(AOptions.BinHeight);
			Box Bin(Width, Height, { Width / 2, Height / 2 });
			placers::BLConfig<CetPolygonImpl> Config;
			// A proxy only approximates its expanded children. Reserve one spacing
			// margin on each side while repacking so the later child validation does
			// not reopen a sheet because a proxy boundary merely touched a single.
			Config.min_obj_distance = 0;
			Config.epsilon = 1;
			Config.allow_rotations = CetRotationUtils::IsAllowedRotation(CET_CLUSTER_HALF_PI, AOptions.Rotations, 1e-9);
			const long long BackfillInflation = static_cast<long long>(std::ceil(static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)) * 0.5));
			std::vector<std::size_t> AllIndices(AItems.size());
			std::iota(AllIndices.begin(), AllIndices.end(), 0);
			if (RepackClusterItems(AItems, AllIndices, Bin, Config, 0, BackfillInflation)) {
				std::cout << "[NEST][CLUSTER BACKFILL][FULL] All packed proxies fit on bin 0." << std::endl;
				return 1;
			}
			std::set<int> AffectedBins;
			std::size_t Moved = 0;

			for (std::size_t Target = 0; Target + 1 < ALayers; ++Target){
				std::vector<std::size_t> TargetIndices;
				for (std::size_t Index = 0; Index < AItems.size(); ++Index){
					if (AItems[Index].binId() == static_cast<int>(Target)) TargetIndices.push_back(Index);
				}

				std::vector<std::size_t> Candidates;
				for (std::size_t Index = 0; Index < AItems.size(); ++Index){
					if (AItems[Index].binId() > static_cast<int>(Target)) Candidates.push_back(Index);
				}
				std::stable_sort(Candidates.begin(), Candidates.end(), [&](std::size_t A, std::size_t B) {
					return AItems[A].area() < AItems[B].area();
				});
				std::size_t Attempts = 0;
				for (std::size_t Index : Candidates){
					if (Attempts++ >= 32) break;
					if (AItems[Index].binId() <= static_cast<int>(Target)) continue;
					const int Source = AItems[Index].binId();
					std::vector<std::size_t> TrialIndices = TargetIndices;
					TrialIndices.push_back(Index);
					// Repack the target and candidate together while preserving glued proxies.
					if (!RepackClusterItems(AItems, TrialIndices, Bin, Config, static_cast<int>(Target), BackfillInflation)) continue;
					TargetIndices.push_back(Index);
					AffectedBins.insert(Source);
					++Moved;
				}
				std::cout << "[NEST][CLUSTER BACKFILL] Target=" << Target
					<< ", Attempts=" << Attempts << std::endl;
			}

			// Repack only source sheets affected by proxy moves. All proxies remain
			// intact, so their child spacing and glued geometry are preserved.
			for (int SourceBin : AffectedBins){
				std::vector<std::size_t> Indices;
				for (std::size_t Index = 0; Index < AItems.size(); ++Index){
					if (AItems[Index].binId() == SourceBin) Indices.push_back(Index);
				}
				const bool Success = RepackClusterItems(AItems, Indices, Bin, Config, SourceBin, BackfillInflation);
				if (Success){
					// RepackClusterItems has already written the packed items back.
				}
				std::cout << "[NEST][CLUSTER REPACK] Bin=" << SourceBin
					<< ", Items=" << Indices.size()
					<< ", Applied=" << (Success ? 1 : 0) << std::endl;
			}

			std::set<int> UsedBins;
			for (const CetNestItem& Item : AItems) if (Item.binId() >= 0) UsedBins.insert(Item.binId());
			std::map<int,int> Dense;
			int NextBin = 0;
			for (int BinId : UsedBins) Dense[BinId] = NextBin++;
			for (CetNestItem& Item : AItems){
				auto It = Dense.find(Item.binId());
				if (It != Dense.end()) Item.binId(It->second);
			}
			std::cout << "[NEST][CLUSTER BACKFILL SUMMARY] Moved=" << Moved
				<< ", LayersBefore=" << ALayers
				<< ", LayersAfter=" << UsedBins.size() << std::endl;
			return UsedBins.size();
		}

		static std::vector<MetClusterStrategy> BuildClusterStrategies(const std::vector<TetShapeFeature>& AFeatures)
		{
			if (AFeatures.size() > CET_NEST_FULL_STRATEGY_ITEM_LIMIT){
				// TemplateCluster preserves every unclustered item as a single and
				// validates full coverage. Avoid an additional full NFP pass over a
				// large original order before evaluating the reduced proxy set.
				return { MetClusterStrategy::TemplateCluster };
			}
			const std::size_t CustomShapeCount = static_cast<std::size_t>(std::count_if(AFeatures.begin(), AFeatures.end(), [](const TetShapeFeature& AFeature) {
				return AFeature.ShapeType == MetShapeType::QuadrilateralLike ||
					AFeature.ShapeType == MetShapeType::ConvexPolygon ||
					AFeature.ShapeType == MetShapeType::ConcavePolygon;
				}));
			const bool HasLargeCustomMajority = AFeatures.size() >= 32 && CustomShapeCount * 2 >= AFeatures.size();
			if (HasLargeCustomMajority){
				return { MetClusterStrategy::TemplateCluster };
			}

			return { MetClusterStrategy::None, MetClusterStrategy::TemplateCluster };
		}

		static TetClusterBuildResult DissolvePackedClusters(const CetTNestItemVector& AOriginalItems, const TetClusterBuildResult& ASource, const std::set<int>& APackedIndices)
		{
			TetClusterBuildResult Result;
			Result.NestItems.reserve(ASource.NestItems.size() + APackedIndices.size() * 4);
			Result.MetaItems.reserve(ASource.MetaItems.size() + APackedIndices.size() * 4);
			for (std::size_t PackedIndex = 0; PackedIndex < ASource.NestItems.size() && PackedIndex < ASource.MetaItems.size(); ++PackedIndex) {
				const TetMetaItem& SourceMeta = ASource.MetaItems[PackedIndex];
				const bool Dissolve = APackedIndices.find(static_cast<int>(PackedIndex)) != APackedIndices.end() && SourceMeta.IsCluster;
				if (!Dissolve) {
					Result.NestItems.push_back(ASource.NestItems[PackedIndex]);
					TetMetaItem Meta = SourceMeta;
					Meta.PackedItemIndex = static_cast<int>(Result.MetaItems.size());
					Result.MetaItems.push_back(std::move(Meta));
					continue;
				}

				for (const TetItemTransform& Transform : SourceMeta.TransformData) {
					if (Transform.OriginalId < 0 || Transform.OriginalId >= static_cast<int>(AOriginalItems.size())) {
						continue;
					}
					Result.NestItems.push_back(AOriginalItems[Transform.OriginalId]);
					TetMetaItem Meta;
					Meta.PackedItemIndex = static_cast<int>(Result.MetaItems.size());
					Meta.IsCluster = false;
					Meta.ClusterType = "SpacingFallbackSingle";
					Meta.TransformData.push_back({ Transform.OriginalId, 0.0, 0.0, 0.0 });
					Result.MetaItems.push_back(std::move(Meta));
				}
			}
			return Result;
		}

		bool CetNest2DEngine::_RunLastBinEvacuation(CetTNestItemVector& ANestItems, const TetNestOptions& AOptions, std::size_t& ALayers)
		{
			if (!AOptions.EnableLastBinEvacuation || ANestItems.empty() || ALayers <= 1) {
				return false;
			}
			const CetTNestItemVector OriginalSolution = ANestItems;
			double BoardBinWidth = AOptions.BinWidth;
			double BoardBinHeight = AOptions.BinHeight;
			CetPolygonImpl BinPoly = Nest2DUtils->Nest2DBord->BuildBinPolygonFromOptions(AOptions, BoardBinWidth, BoardBinHeight);
			CetPolygonBoardRepairer Repairer(ANestItems, AOptions, BinPoly, BoardBinWidth, BoardBinHeight);
			TetLastBinEvacuationStats Stats;
			const bool Success = Repairer.EvacuateLastBin(ALayers, Stats);
			if (!Success) {
				ANestItems = OriginalSolution;
			}
#ifdef _DEBUG
			std::cout << "[LAST_BIN] Start UsedBins=" << Stats.BeforeUsedBins << ", LastBin=" << Stats.LastBinId << ", LastBinItems=" << Stats.LastBinItemCount << ", LastBinArea=" << Stats.LastBinArea << std::endl;
			std::cout << (Success ? "[LAST_BIN][SUCCESS]" : "[LAST_BIN][FAILED]")
				<< " UsedBins " << Stats.BeforeUsedBins << " -> " << Stats.AfterUsedBins
				<< ", DirectMoves=" << Stats.DirectMoves
				<< ", SameBinRelocations=" << Stats.SameBinRelocations
				<< ", RelocatedExistingSmallItems=" << Stats.RelocatedExistingSmallItemCount
				<< ", PlacementChecks=" << Stats.PlacementChecks
				<< ", SearchBudgetReached=" << Stats.SearchBudgetReached
				<< ", RemainingItems=" << Stats.RemainingItems
				<< ", TimeMs=" << Stats.TimeMs
				<< ", Rollback=" << Stats.RolledBack << std::endl;
			if (!Success) {
				std::cout << "[LAST_BIN][FAIL SUMMARY] Remaining=" << Stats.RemainingItems
					<< ", NoCandidatePosition=" << Stats.NoCandidatePosition
					<< ", RelocationFailed=" << Stats.RelocationFailed
					<< ", InsufficientFreeArea=" << Stats.InsufficientFreeArea << std::endl;
			}
#endif
			return Success;
		}

		bool CetNest2DEngine::_RepairAndEvacuate(CetTNestItemVector& ANestItems, const TetNestOptions& AOptions, const CetPolygonImpl& ABinPoly, double ABinWidth, double ABinHeight, std::size_t& ALayers)
		{
			const CetTNestItemVector OriginalSolution = ANestItems;
			const std::size_t OriginalLayers = ALayers;
			CetPolygonBoardRepairer Repairer(ANestItems, AOptions, ABinPoly, ABinWidth, ABinHeight);
			const auto RepairStart = std::chrono::steady_clock::now();
			Repairer.Repair(ALayers);
			const double BeforeLastBinMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - RepairStart).count();
			std::cout << "[NEST][TIMING] BeforeLastBinMs=" << BeforeLastBinMs << std::endl;
			const bool BoardFillImproved = Repairer.HadBoardFillChanges();
			const bool LastBinImproved = _RunLastBinEvacuation(ANestItems, AOptions, ALayers);
			if (!ValidatePlacedItemsSpacing(ANestItems, AOptions)){
				std::cout << "[NEST][REPAIR][ROLLBACK] Repair produced an invalid spacing result." << std::endl;
				ANestItems = OriginalSolution;
				ALayers = OriginalLayers;
				return false;
			}
			return BoardFillImproved || LastBinImproved;
		}

		bool CetNest2DEngine::_TryLockedEnvelopeBoardRepair(CetTNestItemVector& ANestItems, const TetNestOptions& AOptions, const CetPolygonImpl& ABinPoly, double ABinWidth, double ABinHeight, const std::vector<std::size_t>& ALockedChildren, std::size_t& ALayers)
		{
			if (ANestItems.empty() || ALockedChildren.empty() || ALayers == 0) return false;
			const CetTNestItemVector BeforeItems = ANestItems;
			const std::size_t BeforeLayers = ALayers;
			const TetTNestEvalResult BeforeEval = Nest2DUtils->Nest2DStrategy->EvaluateNestResult(BeforeItems, BeforeLayers);
			CetPolygonBoardRepairer Repairer(ANestItems, AOptions, ABinPoly, ABinWidth, ABinHeight);
			Repairer.RepairLockedEnvelope(ALayers, ALockedChildren);
			const bool Preserved = PreservesLockedChildren(BeforeItems, ANestItems, ALockedChildren);
			const bool Valid = Preserved && ValidatePlacedItemsSpacing(ANestItems, AOptions);
			const TetTNestEvalResult AfterEval = Nest2DUtils->Nest2DStrategy->EvaluateNestResult(ANestItems, ALayers);
			const bool Improved = Valid && Nest2DUtils->Nest2DStrategy->IsBetterNestResult(AfterEval, BeforeEval);
			if (!Improved) {
				ANestItems = BeforeItems;
				ALayers = BeforeLayers;
				std::cout << "[NEST][LOCKED ENVELOPE][BOARD REPAIR] Rollback Preserved=" << Preserved
					<< " Valid=" << Valid << " Improved=" << Improved << std::endl;
				return false;
			}
			std::cout << "[NEST][LOCKED ENVELOPE][BOARD REPAIR] Accepted Layers="
				<< BeforeLayers << " -> " << ALayers << std::endl;
			return true;
		}

		static bool BuildBoardPath(const std::vector<TetNestPoint>& AVertices, bool AOuter, CetPath& AOutPath)
		{
			AOutPath.clear();
			for (const TetNestPoint& Point : AVertices) AOutPath.push_back({ NestUtils::ToNestCoord(Point.X), NestUtils::ToNestCoord(Point.Y) });
			ClipperLib::CleanPolygon(AOutPath, 1.0);
			if (AOutPath.size() < 3 || std::abs(ClipperLib::Area(AOutPath)) <= 0.0) return false;
			if (ClipperLib::Orientation(AOutPath) != AOuter) std::reverse(AOutPath.begin(), AOutPath.end());
			return true;
		}

		static bool BuildBoardSubjectContours(const TetNestOptions& AOptions, ClipperLib::Paths& AOutContours)
		{
			AOutContours.clear();
			if (AOptions.Board.Enabled && AOptions.Board.Vertices.size() >= 3) {
				CetPath Outer;
				if (!BuildBoardPath(AOptions.Board.Vertices, true, Outer)) return false;
				AOutContours.push_back(std::move(Outer));
				for (const auto& HoleVertices : AOptions.Board.Holes) {
					CetPath Hole;
					if (!BuildBoardPath(HoleVertices, false, Hole)) return false;
					AOutContours.push_back(std::move(Hole));
				}
				return true;
			}
			const double Width = AOptions.BinWidth;
			const double Height = AOptions.BinHeight;
			if (Width <= 0.0 || Height <= 0.0) return false;
			std::vector<TetNestPoint> Rectangle{ { 0.0, 0.0 }, { Width, 0.0 }, { Width, Height }, { 0.0, Height } };
			CetPath Outer;
			if (!BuildBoardPath(Rectangle, true, Outer)) return false;
			AOutContours.push_back(std::move(Outer));
			return true;
		}

		static bool BuildPlacedReservedContours(const CetTNestItemVector& AItems, int ABinId, double ASpacing,
			ClipperLib::Paths& AOutContours, libnest2d::Coord AExtraInflation = 0)
		{
			AOutContours.clear();
			const auto HalfSpacing = static_cast<libnest2d::Coord>(std::ceil(static_cast<double>(NestUtils::ToNestCoord(std::max(0.0, ASpacing))) * 0.5));
			for (const CetNestItem& SourceItem : AItems) {
				if (SourceItem.binId() != ABinId) continue;
				CetNestItem Item = SourceItem;
				Item.inflation(HalfSpacing + std::max<libnest2d::Coord>(0, AExtraInflation));
				CetPolygonImpl Shape = Item.transformedShape();
				ClipperLib::CleanPolygon(Shape.Contour, 1.0);
				if (Shape.Contour.size() < 3 || std::abs(ClipperLib::Area(Shape.Contour)) <= 0.0) return false;
				if (!ClipperLib::Orientation(Shape.Contour)) std::reverse(Shape.Contour.begin(), Shape.Contour.end());
				AOutContours.push_back(std::move(Shape.Contour));
				for (CetPath Hole : Shape.Holes) {
					ClipperLib::CleanPolygon(Hole, 1.0);
					if (Hole.size() < 3 || std::abs(ClipperLib::Area(Hole)) <= 0.0) return false;
					if (ClipperLib::Orientation(Hole)) std::reverse(Hole.begin(), Hole.end());
					AOutContours.push_back(std::move(Hole));
				}
			}
			return true;
		}

		struct TetLocalCompactEnvelope
		{
			bool Valid = false;
			double MinX = 0.0;
			double MinY = 0.0;
			double MaxX = 0.0;
			double MaxY = 0.0;
			double Width = 0.0;
			double Height = 0.0;
			double Area = 0.0;
			double LongSide = 0.0;
		};

		struct TetLocalCompactTarget
		{
			std::vector<std::size_t> Indices;
			std::vector<TetItemTransform> Transforms;
			std::string Type = "Single";
			bool IsCluster = false;
			double CurrentRotation = 0.0;
			double CurrentAnchorX = 0.0;
			double CurrentAnchorY = 0.0;
		};

		struct TetLocalCompactCandidate
		{
			bool Valid = false;
			double Rotation = 0.0;
			double AnchorX = 0.0;
			double AnchorY = 0.0;
			TetLocalCompactEnvelope Envelope;
			int ContactScore = 0;
			double TranslationDistance = 0.0;
			double RotationDelta = 0.0;
		};

		struct TetLocalCompactFreeSpaceMetric
		{
			bool Valid = false;
			std::size_t RegionCount = 0;
			double LargestArea = 0.0;
			double FragmentedArea = 0.0;
		};

		static constexpr std::size_t CET_LOCAL_COMPACT_MAX_FREE_REGIONS = 8;
		static constexpr std::size_t CET_LOCAL_COMPACT_MAX_CONTACT_VERTICES = 4;
		static constexpr std::size_t CET_LOCAL_COMPACT_MAX_HOLE_CONTACTS = 8;
		static constexpr std::size_t CET_LOCAL_COMPACT_MAX_FREE_SPACE_EVALUATIONS = 8;
		static constexpr std::size_t CET_LOCAL_COMPACT_MAX_TARGETS = 8;
		static constexpr std::size_t CET_LOCAL_COMPACT_MAX_ANCHORS_PER_ROTATION = 96;
		static constexpr long long CET_LOCAL_COMPACT_MAX_TIME_MS = 300;

		static double LocalCompactAngleDistance(double ALeft, double ARight)
		{
			const double Left = CetRotationUtils::NormalizeAngle(ALeft);
			const double Right = CetRotationUtils::NormalizeAngle(ARight);
			const double Delta = std::abs(Left - Right);
			return std::min(Delta, CET_CLUSTER_TWO_PI - Delta);
		}

		static bool LocalCompactGetBounds(const CetNestItem& AItem, double& AOutMinX, double& AOutMinY,
			double& AOutMaxX, double& AOutMaxY)
		{
			CetNestItem Item = AItem;
			Item.inflation(0);
			const auto Bounds = Item.boundingBox();
			AOutMinX = static_cast<double>(getX(Bounds.minCorner()));
			AOutMinY = static_cast<double>(getY(Bounds.minCorner()));
			AOutMaxX = static_cast<double>(getX(Bounds.maxCorner()));
			AOutMaxY = static_cast<double>(getY(Bounds.maxCorner()));
			return std::isfinite(AOutMinX) && std::isfinite(AOutMinY)
				&& std::isfinite(AOutMaxX) && std::isfinite(AOutMaxY)
				&& AOutMaxX >= AOutMinX && AOutMaxY >= AOutMinY;
		}

		static TetLocalCompactEnvelope LocalCompactCalculateEnvelope(const CetTNestItemVector& AItems, int ABinId,
			const std::vector<bool>* AExcluded = nullptr)
		{
			TetLocalCompactEnvelope Result;
			for (std::size_t Index = 0; Index < AItems.size(); ++Index) {
				if (AItems[Index].binId() != ABinId || (AExcluded != nullptr && Index < AExcluded->size() && (*AExcluded)[Index])) continue;
				double MinX = 0.0, MinY = 0.0, MaxX = 0.0, MaxY = 0.0;
				if (!LocalCompactGetBounds(AItems[Index], MinX, MinY, MaxX, MaxY)) continue;
				if (!Result.Valid) {
					Result.Valid = true;
					Result.MinX = MinX; Result.MinY = MinY; Result.MaxX = MaxX; Result.MaxY = MaxY;
				}
				else {
					Result.MinX = std::min(Result.MinX, MinX); Result.MinY = std::min(Result.MinY, MinY);
					Result.MaxX = std::max(Result.MaxX, MaxX); Result.MaxY = std::max(Result.MaxY, MaxY);
				}
			}
			if (Result.Valid) {
				Result.Width = Result.MaxX - Result.MinX;
				Result.Height = Result.MaxY - Result.MinY;
				Result.Area = Result.Width * Result.Height;
				Result.LongSide = std::max(Result.Width, Result.Height);
			}
			return Result;
		}

		static void LocalCompactAppendFreeRegions(const ClipperLib::PolyNode& ANode,
			std::vector<TetClusterFreeRegion>& AOutRegions)
		{
			if (!ANode.IsHole() && ANode.Contour.size() >= 3) {
				TetClusterFreeRegion Region;
				Region.Contour = ANode.Contour;
				Region.IsClosed = true;
				Region.Area = std::abs(static_cast<double>(ClipperLib::Area(Region.Contour)));
				if (Region.Area > 0.0 && std::isfinite(Region.Area)) {
					Region.MinX = Region.MaxX = static_cast<double>(Region.Contour.front().X);
					Region.MinY = Region.MaxY = static_cast<double>(Region.Contour.front().Y);
					for (const ClipperLib::IntPoint& Point : Region.Contour) {
						Region.MinX = std::min(Region.MinX, static_cast<double>(Point.X));
						Region.MinY = std::min(Region.MinY, static_cast<double>(Point.Y));
						Region.MaxX = std::max(Region.MaxX, static_cast<double>(Point.X));
						Region.MaxY = std::max(Region.MaxY, static_cast<double>(Point.Y));
					}
					for (const ClipperLib::PolyNode* Child : ANode.Childs) {
						if (Child != nullptr && Child->IsHole()) Region.Holes.push_back(Child->Contour);
					}
					Region.Width = Region.MaxX - Region.MinX;
					Region.Height = Region.MaxY - Region.MinY;
					if (Region.Width > 0.0 && Region.Height > 0.0) AOutRegions.push_back(std::move(Region));
				}
			}
			for (const ClipperLib::PolyNode* Child : ANode.Childs) if (Child != nullptr) {
				LocalCompactAppendFreeRegions(*Child, AOutRegions);
			}
		}

		static bool BuildLocalCompactFreeRegions(const CetTNestItemVector& AItems, const TetNestOptions& AOptions,
			int ABinId, const std::vector<bool>& ATargetMask, std::vector<TetClusterFreeRegion>& AOutRegions)
		{
			AOutRegions.clear();
			ClipperLib::Paths Board;
			if (!BuildBoardSubjectContours(AOptions, Board)) return false;
			CetTNestItemVector FrozenItems = AItems;
			for (std::size_t Index = 0; Index < FrozenItems.size() && Index < ATargetMask.size(); ++Index) {
				if (ATargetMask[Index]) FrozenItems[Index].binId(-1);
			}
			const auto HalfSpacing = static_cast<libnest2d::Coord>(std::ceil(
				static_cast<double>(NestUtils::ToNestCoord(std::max(0.0, AOptions.Spacing))) * 0.5));
			ClipperLib::Paths Reserved;
			if (!BuildPlacedReservedContours(FrozenItems, ABinId, AOptions.Spacing, Reserved, HalfSpacing)) return false;
			ClipperLib::Clipper Difference;
			if (!Difference.AddPaths(Board, ClipperLib::ptSubject, true)
				|| (!Reserved.empty() && !Difference.AddPaths(Reserved, ClipperLib::ptClip, true))) return false;
			ClipperLib::PolyTree Tree;
			if (!Difference.Execute(ClipperLib::ctDifference, Tree, ClipperLib::pftNonZero, ClipperLib::pftNonZero)) return false;
			for (const ClipperLib::PolyNode* Node : Tree.Childs) if (Node != nullptr) LocalCompactAppendFreeRegions(*Node, AOutRegions);
			std::stable_sort(AOutRegions.begin(), AOutRegions.end(), [](const TetClusterFreeRegion& ALeft, const TetClusterFreeRegion& ARight) {
				return ALeft.Area != ARight.Area ? ALeft.Area > ARight.Area : ALeft.MinY < ARight.MinY;
			});
			if (AOutRegions.size() > CET_LOCAL_COMPACT_MAX_FREE_REGIONS) AOutRegions.resize(CET_LOCAL_COMPACT_MAX_FREE_REGIONS);
			return !AOutRegions.empty();
		}

		static TetLocalCompactFreeSpaceMetric LocalCompactCalculateFreeSpaceMetric(const CetTNestItemVector& AItems,
			const TetNestOptions& AOptions, int ABinId)
		{
			TetLocalCompactFreeSpaceMetric Result;
			ClipperLib::Paths Board;
			if (!BuildBoardSubjectContours(AOptions, Board)) return Result;
			const auto HalfSpacing = static_cast<libnest2d::Coord>(std::ceil(
				static_cast<double>(NestUtils::ToNestCoord(std::max(0.0, AOptions.Spacing))) * 0.5));
			ClipperLib::Paths Reserved;
			if (!BuildPlacedReservedContours(AItems, ABinId, AOptions.Spacing, Reserved, HalfSpacing)) return Result;
			ClipperLib::Clipper Difference;
			if (!Difference.AddPaths(Board, ClipperLib::ptSubject, true)
				|| (!Reserved.empty() && !Difference.AddPaths(Reserved, ClipperLib::ptClip, true))) return Result;
			ClipperLib::PolyTree Tree;
			if (!Difference.Execute(ClipperLib::ctDifference, Tree, ClipperLib::pftNonZero, ClipperLib::pftNonZero)) return Result;
			double TotalArea = 0.0;
			auto Accumulate = [&](const auto& Self, const ClipperLib::PolyNode& ANode) -> void {
				if (!ANode.IsHole() && ANode.Contour.size() >= 3) {
					const double Area = std::abs(static_cast<double>(ClipperLib::Area(ANode.Contour)));
					if (Area > 0.0 && std::isfinite(Area)) {
						++Result.RegionCount;
						TotalArea += Area;
						Result.LargestArea = std::max(Result.LargestArea, Area);
					}
				}
				for (const ClipperLib::PolyNode* Child : ANode.Childs) if (Child != nullptr) Self(Self, *Child);
			};
			for (const ClipperLib::PolyNode* Node : Tree.Childs) if (Node != nullptr) Accumulate(Accumulate, *Node);
			Result.Valid = Result.RegionCount > 0;
			Result.FragmentedArea = std::max(0.0, TotalArea - Result.LargestArea);
			return Result;
		}

		static double LocalCompactCalculateBoardArea(const TetNestOptions& AOptions)
		{
			ClipperLib::Paths Board;
			if (!BuildBoardSubjectContours(AOptions, Board)) return 0.0;
			double Area = 0.0;
			for (const CetPath& Contour : Board) {
				const double ContourArea = std::abs(static_cast<double>(ClipperLib::Area(Contour)));
				if (ContourArea <= 0.0 || !std::isfinite(ContourArea)) continue;
				Area += ClipperLib::Orientation(Contour) ? ContourArea : -ContourArea;
			}
			return std::max(0.0, Area);
		}

		static std::map<int, std::string> LocalCompactBuildSkippedBins(const CetTNestItemVector& AItems,
			const TetNestOptions& AOptions)
		{
			std::map<int, std::string> SkippedBins;
			std::map<int, std::size_t> ItemCounts;
			for (const CetNestItem& Item : AItems) if (Item.binId() >= 0) ++ItemCounts[static_cast<int>(Item.binId())];
			const double BoardArea = LocalCompactCalculateBoardArea(AOptions);
			for (const auto& Entry : ItemCounts) {
				const int BinId = Entry.first;
				const std::size_t ItemCount = Entry.second;
				if (ItemCount <= 1) {
					SkippedBins[BinId] = "INSUFFICIENT_ITEMS";
					continue;
				}
				const TetLocalCompactFreeSpaceMetric FreeSpace = LocalCompactCalculateFreeSpaceMetric(AItems, AOptions, BinId);
				if (!FreeSpace.Valid) {
					SkippedBins[BinId] = "NO_FREE_REGION";
					continue;
				}
				const double DominantGapRatio = BoardArea > 0.0 ? FreeSpace.LargestArea / BoardArea : 0.0;
				// A single region is not by itself a reason to skip: useful compacting
				// moves often happen inside one connected free-space component. Skip
				// only when that component is genuinely dominant (nearly empty board).
				if (FreeSpace.RegionCount <= 1 && DominantGapRatio >= 0.80) {
					SkippedBins[BinId] = "SINGLE_CONTIGUOUS_GAP";
					continue;
				}
				if (DominantGapRatio >= 0.80) {
					SkippedBins[BinId] = "DOMINANT_EMPTY_BOARD";
					continue;
				}
			}
			return SkippedBins;
		}

		static std::vector<std::size_t> LocalCompactSelectContactVertices(const CetPath& AContour)
		{
			std::vector<std::size_t> Result;
			if (AContour.empty()) return Result;
			std::array<std::size_t, 4> Extremes{ 0, 0, 0, 0 };
			for (std::size_t Index = 1; Index < AContour.size(); ++Index) {
				if (AContour[Index].X < AContour[Extremes[0]].X) Extremes[0] = Index;
				if (AContour[Index].X > AContour[Extremes[1]].X) Extremes[1] = Index;
				if (AContour[Index].Y < AContour[Extremes[2]].Y) Extremes[2] = Index;
				if (AContour[Index].Y > AContour[Extremes[3]].Y) Extremes[3] = Index;
			}
			for (const std::size_t Index : Extremes) {
				if (std::find(Result.begin(), Result.end(), Index) == Result.end()) Result.push_back(Index);
			}
			for (std::size_t Slot = 0; Result.size() < CET_LOCAL_COMPACT_MAX_CONTACT_VERTICES && Slot < AContour.size(); ++Slot) {
				const std::size_t Index = Slot * AContour.size() / CET_LOCAL_COMPACT_MAX_CONTACT_VERTICES;
				if (std::find(Result.begin(), Result.end(), Index) == Result.end()) Result.push_back(Index);
			}
			return Result;
		}

		static void LocalCompactApplyPose(CetTNestItemVector& AItems, const TetLocalCompactTarget& ATarget,
			double ARotation, double AAnchorX, double AAnchorY)
		{
			const double CosRotation = std::cos(ARotation);
			const double SinRotation = std::sin(ARotation);
			for (std::size_t Slot = 0; Slot < ATarget.Indices.size() && Slot < ATarget.Transforms.size(); ++Slot) {
				CetNestItem& Item = AItems[ATarget.Indices[Slot]];
				const TetItemTransform& Transform = ATarget.Transforms[Slot];
				const double X = AAnchorX + Transform.RelativeX * CosRotation - Transform.RelativeY * SinRotation;
				const double Y = AAnchorY + Transform.RelativeX * SinRotation + Transform.RelativeY * CosRotation;
				Item.translation(ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(std::llround(X)),
					static_cast<ClipperLib::cInt>(std::llround(Y))));
				Item.rotation(ARotation + Transform.RelativeRotation);
				Item.inflation(0);
			}
		}

		struct TetLocalCompactFixedItem
		{
			CetNestItem Raw;
			CetNestItem Spaced;
			double RawMinX = 0.0, RawMinY = 0.0, RawMaxX = 0.0, RawMaxY = 0.0;
			double SpacedMinX = 0.0, SpacedMinY = 0.0, SpacedMaxX = 0.0, SpacedMaxY = 0.0;
		};

		static std::vector<TetLocalCompactFixedItem> LocalCompactBuildFixedItemCache(
			const CetTNestItemVector& AItems, const std::vector<bool>& ATargetMask,
			const TetNestOptions& AOptions, int ABinId)
		{
			std::vector<TetLocalCompactFixedItem> Result;
			const auto HalfSpacing = static_cast<libnest2d::Coord>(std::ceil(
				static_cast<double>(NestUtils::ToNestCoord(std::max(0.0, AOptions.Spacing))) * 0.5));
			for (std::size_t Index = 0; Index < AItems.size(); ++Index) {
				if ((Index < ATargetMask.size() && ATargetMask[Index]) || AItems[Index].binId() != ABinId) continue;
				CetNestItem Raw = AItems[Index];
				Raw.inflation(0);
				CetNestItem Spaced = Raw;
				if (HalfSpacing > 0) Spaced.inflation(HalfSpacing);
				const auto RawBounds = Raw.boundingBox();
				const auto SpacedBounds = Spaced.boundingBox();
				Result.push_back({ Raw, Spaced,
					static_cast<double>(getX(RawBounds.minCorner())), static_cast<double>(getY(RawBounds.minCorner())),
					static_cast<double>(getX(RawBounds.maxCorner())), static_cast<double>(getY(RawBounds.maxCorner())),
					static_cast<double>(getX(SpacedBounds.minCorner())), static_cast<double>(getY(SpacedBounds.minCorner())),
					static_cast<double>(getX(SpacedBounds.maxCorner())), static_cast<double>(getY(SpacedBounds.maxCorner())) });
			}
			return Result;
		}

		static bool LocalCompactIsTargetPoseValid(const CetTNestItemVector& AItems, const TetLocalCompactTarget& ATarget,
			const std::vector<bool>& ATargetMask, const CetPolygonImpl& ABinPoly, const TetNestOptions& AOptions,
			int ABinId, const std::vector<TetLocalCompactFixedItem>& AFixedItems, const char*& AOutReason)
		{
			AOutReason = "OUT_OF_BIN";
			const auto HalfSpacing = static_cast<libnest2d::Coord>(std::ceil(
				static_cast<double>(NestUtils::ToNestCoord(std::max(0.0, AOptions.Spacing))) * 0.5));
			for (const std::size_t TargetIndex : ATarget.Indices) {
				CetNestItem Target = AItems[TargetIndex];
				Target.inflation(0);
				if (Target.binId() != ABinId || !Target.isInside(ABinPoly)) return false;
				const auto RawBounds = Target.boundingBox();
				CetNestItem SpacedTarget = Target;
				if (HalfSpacing > 0) SpacedTarget.inflation(HalfSpacing);
				const auto SpacedTargetBounds = SpacedTarget.boundingBox();
				for (const TetLocalCompactFixedItem& Other : AFixedItems) {
					const bool RawBoundsOverlap = !(getX(RawBounds.maxCorner()) < Other.RawMinX
						|| getX(RawBounds.minCorner()) > Other.RawMaxX
						|| getY(RawBounds.maxCorner()) < Other.RawMinY
						|| getY(RawBounds.minCorner()) > Other.RawMaxY);
					if (RawBoundsOverlap && CetNestItem::intersects(Target, Other.Raw) && !CetNestItem::touches(Target, Other.Raw)) {
						AOutReason = "COLLISION";
						return false;
					}
					if (HalfSpacing > 0) {
						if (!(getX(SpacedTargetBounds.maxCorner()) < Other.SpacedMinX
							|| getX(SpacedTargetBounds.minCorner()) > Other.SpacedMaxX
							|| getY(SpacedTargetBounds.maxCorner()) < Other.SpacedMinY
							|| getY(SpacedTargetBounds.minCorner()) > Other.SpacedMaxY)
							&& CetNestItem::intersects(SpacedTarget, Other.Spaced) && !CetNestItem::touches(SpacedTarget, Other.Spaced)) {
							AOutReason = "SPACING";
							return false;
						}
					}
				}
			}
			AOutReason = "VALID";
			return true;
		}

		static TetLocalCompactEnvelope LocalCompactCalculateTargetEnvelope(const CetTNestItemVector& AItems,
			const TetLocalCompactTarget& ATarget)
		{
			TetLocalCompactEnvelope Result;
			for (const std::size_t Index : ATarget.Indices) {
				if (Index >= AItems.size()) continue;
				double MinX = 0.0, MinY = 0.0, MaxX = 0.0, MaxY = 0.0;
				if (!LocalCompactGetBounds(AItems[Index], MinX, MinY, MaxX, MaxY)) continue;
				if (!Result.Valid) {
					Result.Valid = true;
					Result.MinX = MinX; Result.MinY = MinY; Result.MaxX = MaxX; Result.MaxY = MaxY;
				}
				else {
					Result.MinX = std::min(Result.MinX, MinX); Result.MinY = std::min(Result.MinY, MinY);
					Result.MaxX = std::max(Result.MaxX, MaxX); Result.MaxY = std::max(Result.MaxY, MaxY);
				}
			}
			if (Result.Valid) {
				Result.Width = Result.MaxX - Result.MinX;
				Result.Height = Result.MaxY - Result.MinY;
				Result.Area = Result.Width * Result.Height;
				Result.LongSide = std::max(Result.Width, Result.Height);
			}
			return Result;
		}

		static TetLocalCompactEnvelope LocalCompactMergeEnvelopes(const TetLocalCompactEnvelope& AFixed,
			const TetLocalCompactEnvelope& ATarget)
		{
			if (!AFixed.Valid) return ATarget;
			if (!ATarget.Valid) return AFixed;
			TetLocalCompactEnvelope Result;
			Result.Valid = true;
			Result.MinX = std::min(AFixed.MinX, ATarget.MinX);
			Result.MinY = std::min(AFixed.MinY, ATarget.MinY);
			Result.MaxX = std::max(AFixed.MaxX, ATarget.MaxX);
			Result.MaxY = std::max(AFixed.MaxY, ATarget.MaxY);
			Result.Width = Result.MaxX - Result.MinX;
			Result.Height = Result.MaxY - Result.MinY;
			Result.Area = Result.Width * Result.Height;
			Result.LongSide = std::max(Result.Width, Result.Height);
			return Result;
		}

		static bool LocalCompactTryBuildClusterTarget(const CetTNestItemVector& AItems, const TetMetaItem& AMeta,
			TetLocalCompactTarget& AOutTarget)
		{
			AOutTarget = TetLocalCompactTarget{};
			if (!AMeta.IsCluster || AMeta.TransformData.size() < 2) return false;
			for (const TetItemTransform& Transform : AMeta.TransformData) {
				if (Transform.OriginalId < 0 || Transform.OriginalId >= static_cast<int>(AItems.size())) return false;
			}
			const TetItemTransform& FirstTransform = AMeta.TransformData.front();
			const CetNestItem& FirstItem = AItems[FirstTransform.OriginalId];
			if (FirstItem.binId() < 0) return false;
			const double Rotation = CetRotationUtils::NormalizeAngle(static_cast<double>(FirstItem.rotation()) - FirstTransform.RelativeRotation);
			const double CosRotation = std::cos(Rotation);
			const double SinRotation = std::sin(Rotation);
			const Point FirstTranslation = FirstItem.translation();
			const double AnchorX = static_cast<double>(FirstTranslation.X) - FirstTransform.RelativeX * CosRotation + FirstTransform.RelativeY * SinRotation;
			const double AnchorY = static_cast<double>(FirstTranslation.Y) - FirstTransform.RelativeX * SinRotation - FirstTransform.RelativeY * CosRotation;
			for (const TetItemTransform& Transform : AMeta.TransformData) {
				const CetNestItem& Item = AItems[Transform.OriginalId];
				if (Item.binId() != FirstItem.binId()
					|| LocalCompactAngleDistance(static_cast<double>(Item.rotation()), Rotation + Transform.RelativeRotation) > 1e-8) return false;
				const Point Translation = Item.translation();
				const double ExpectedX = AnchorX + Transform.RelativeX * CosRotation - Transform.RelativeY * SinRotation;
				const double ExpectedY = AnchorY + Transform.RelativeX * SinRotation + Transform.RelativeY * CosRotation;
				if (std::abs(static_cast<double>(Translation.X) - ExpectedX) > 1.0
					|| std::abs(static_cast<double>(Translation.Y) - ExpectedY) > 1.0) return false;
			}
			AOutTarget.Type = AMeta.ClusterType.empty() ? "Cluster" : AMeta.ClusterType;
			AOutTarget.IsCluster = true;
			AOutTarget.CurrentRotation = Rotation;
			AOutTarget.CurrentAnchorX = AnchorX;
			AOutTarget.CurrentAnchorY = AnchorY;
			AOutTarget.Transforms = AMeta.TransformData;
			for (const TetItemTransform& Transform : AMeta.TransformData) AOutTarget.Indices.push_back(static_cast<std::size_t>(Transform.OriginalId));
			return true;
		}

		static bool LocalCompactTryRecoverCurrentClusterTarget(const CetTNestItemVector& AItems,
			const TetMetaItem& AMeta, TetLocalCompactTarget& AOutTarget)
		{
			AOutTarget = TetLocalCompactTarget{};
			if (!AMeta.IsCluster || AMeta.TransformData.size() < 2) return false;
			const int FirstIndex = AMeta.TransformData.front().OriginalId;
			if (FirstIndex < 0 || FirstIndex >= static_cast<int>(AItems.size()) || AItems[FirstIndex].binId() < 0) return false;
			const int BinId = static_cast<int>(AItems[FirstIndex].binId());
			const Point Anchor = AItems[FirstIndex].translation();
			AOutTarget.Type = AMeta.ClusterType.empty() ? "RecoveredCluster" : AMeta.ClusterType;
			AOutTarget.IsCluster = true;
			AOutTarget.CurrentRotation = 0.0;
			AOutTarget.CurrentAnchorX = static_cast<double>(Anchor.X);
			AOutTarget.CurrentAnchorY = static_cast<double>(Anchor.Y);
			for (const TetItemTransform& Source : AMeta.TransformData) {
				if (Source.OriginalId < 0 || Source.OriginalId >= static_cast<int>(AItems.size())
					|| AItems[Source.OriginalId].binId() != BinId) return false;
				const Point Translation = AItems[Source.OriginalId].translation();
				AOutTarget.Indices.push_back(static_cast<std::size_t>(Source.OriginalId));
				AOutTarget.Transforms.push_back({ Source.OriginalId,
					static_cast<double>(Translation.X - Anchor.X),
					static_cast<double>(Translation.Y - Anchor.Y),
					static_cast<double>(AItems[Source.OriginalId].rotation()) });
			}
			return true;
		}

		static std::vector<TetLocalCompactTarget> BuildLocalCompactTargets(const CetTNestItemVector& AItems,
			const std::vector<TetMetaItem>* AMetaItems)
		{
			std::vector<TetLocalCompactTarget> Targets;
			std::vector<bool> ClusterMembers(AItems.size(), false);
			if (AMetaItems != nullptr) for (const TetMetaItem& Meta : *AMetaItems) {
				if (!Meta.IsCluster || Meta.TransformData.size() < 2) continue;
				bool ValidIndices = true;
				for (const TetItemTransform& Transform : Meta.TransformData) {
					if (Transform.OriginalId < 0 || Transform.OriginalId >= static_cast<int>(AItems.size())) { ValidIndices = false; break; }
				}
				if (!ValidIndices) {
					std::cout << "[LOCAL COMPACT][SKIP] type=" << Meta.ClusterType << " reason=INVALID_CLUSTER_INDEX" << std::endl;
					continue;
				}
				TetLocalCompactTarget Target;
				if (LocalCompactTryBuildClusterTarget(AItems, Meta, Target)
					|| LocalCompactTryRecoverCurrentClusterTarget(AItems, Meta, Target)) {
					for (const TetItemTransform& Transform : Meta.TransformData) ClusterMembers[Transform.OriginalId] = true;
					Targets.push_back(std::move(Target));
				}
				else std::cout << "[LOCAL COMPACT][SKIP] type=" << Meta.ClusterType << " reason=CLUSTER_RECOVERY_FAILED" << std::endl;
			}
			for (std::size_t Index = 0; Index < AItems.size(); ++Index) {
				if (ClusterMembers[Index] || AItems[Index].binId() < 0) continue;
				TetLocalCompactTarget Target;
				Target.Indices.push_back(Index);
				Target.Transforms.push_back({ static_cast<int>(Index), 0.0, 0.0, 0.0 });
				Target.CurrentRotation = CetRotationUtils::NormalizeAngle(static_cast<double>(AItems[Index].rotation()));
				const Point Translation = AItems[Index].translation();
				Target.CurrentAnchorX = static_cast<double>(Translation.X);
				Target.CurrentAnchorY = static_cast<double>(Translation.Y);
				Targets.push_back(std::move(Target));
			}
			std::stable_sort(Targets.begin(), Targets.end(), [](const TetLocalCompactTarget& ALeft,
				const TetLocalCompactTarget& ARight) {
				if (ALeft.IsCluster != ARight.IsCluster) return ALeft.IsCluster;
				return ALeft.Indices.size() > ARight.Indices.size();
			});
			return Targets;
		}

		static bool LocalCompactIsStrictImprovement(const TetLocalCompactEnvelope& ACandidate,
			const TetLocalCompactEnvelope& ABaseline)
		{
			if (!ACandidate.Valid || !ABaseline.Valid) return false;
			const double AreaEpsilon = std::max({ 1.0, std::abs(ACandidate.Area), std::abs(ABaseline.Area) }) * 1e-9;
			if (ACandidate.Area < ABaseline.Area - AreaEpsilon) return true;
			if (std::abs(ACandidate.Area - ABaseline.Area) > AreaEpsilon) return false;
			const double LongSideEpsilon = std::max({ 1.0, std::abs(ACandidate.LongSide), std::abs(ABaseline.LongSide) }) * 1e-9;
			return ACandidate.LongSide < ABaseline.LongSide - LongSideEpsilon;
		}

		static bool LocalCompactIsNonWorsening(const TetLocalCompactEnvelope& ACandidate,
			const TetLocalCompactEnvelope& ABaseline)
		{
			if (!ACandidate.Valid || !ABaseline.Valid) return false;
			const double AreaEpsilon = std::max({ 1.0, std::abs(ACandidate.Area), std::abs(ABaseline.Area) }) * 1e-9;
			if (ACandidate.Area > ABaseline.Area + AreaEpsilon) return false;
			const double LongSideEpsilon = std::max({ 1.0, std::abs(ACandidate.LongSide), std::abs(ABaseline.LongSide) }) * 1e-9;
			return ACandidate.LongSide <= ABaseline.LongSide + LongSideEpsilon;
		}

		static bool LocalCompactIsFreeSpaceBetter(const TetLocalCompactFreeSpaceMetric& ACandidate,
			const TetLocalCompactFreeSpaceMetric& ABaseline)
		{
			if (!ACandidate.Valid || !ABaseline.Valid) return false;
			if (ACandidate.RegionCount != ABaseline.RegionCount)
				return ACandidate.RegionCount < ABaseline.RegionCount;
			const double AreaEpsilon = std::max({ 1.0, std::abs(ACandidate.LargestArea), std::abs(ABaseline.LargestArea) }) * 1e-9;
			if (ACandidate.LargestArea > ABaseline.LargestArea + AreaEpsilon) return true;
			if (std::abs(ACandidate.LargestArea - ABaseline.LargestArea) > AreaEpsilon) return false;
			const double FragmentedEpsilon = std::max({ 1.0, std::abs(ACandidate.FragmentedArea), std::abs(ABaseline.FragmentedArea) }) * 1e-9;
			return ACandidate.FragmentedArea < ABaseline.FragmentedArea - FragmentedEpsilon;
		}

		static bool LocalCompactIsCandidateBetter(const TetLocalCompactCandidate& ACandidate,
			const TetLocalCompactCandidate& ABest)
		{
			const double AreaEpsilon = std::max({ 1.0, std::abs(ACandidate.Envelope.Area), std::abs(ABest.Envelope.Area) }) * 1e-9;
			if (std::abs(ACandidate.Envelope.Area - ABest.Envelope.Area) > AreaEpsilon)
				return ACandidate.Envelope.Area < ABest.Envelope.Area;
			const double LongSideEpsilon = std::max({ 1.0, std::abs(ACandidate.Envelope.LongSide), std::abs(ABest.Envelope.LongSide) }) * 1e-9;
			if (std::abs(ACandidate.Envelope.LongSide - ABest.Envelope.LongSide) > LongSideEpsilon)
				return ACandidate.Envelope.LongSide < ABest.Envelope.LongSide;
			if (ACandidate.ContactScore != ABest.ContactScore)
				return ACandidate.ContactScore > ABest.ContactScore;
			if (std::abs(ACandidate.TranslationDistance - ABest.TranslationDistance) > 1e-9)
				return ACandidate.TranslationDistance < ABest.TranslationDistance;
			return ACandidate.RotationDelta < ABest.RotationDelta - 1e-12;
		}

	static void RunLocalCompactPass(CetTNestItemVector& AItems, const TetNestOptions& AOptions,
		const std::vector<TetMetaItem>* AMetaItems)
	{
		if (!AOptions.EnableLocalCompactPass) {
			std::cout << "[LOCAL COMPACT][SKIP] reason=DISABLED" << std::endl;
			return;
		}
		if (AItems.empty()) {
			std::cout << "[LOCAL COMPACT][SKIP] reason=EMPTY_ITEMS" << std::endl;
			return;
		}
			double BoardWidth = AOptions.BinWidth;
			double BoardHeight = AOptions.BinHeight;
			const CetPolygonImpl BinPoly = Nest2DUtils->Nest2DBord->BuildBinPolygonFromOptions(AOptions, BoardWidth, BoardHeight);
			if (BinPoly.Contour.size() < 3) {
				std::cout << "[LOCAL COMPACT][SKIP] reason=INVALID_BIN" << std::endl;
				return;
			}
			const CetTNestItemVector BaselineItems = AItems;
			const auto PassStart = std::chrono::steady_clock::now();
			const std::vector<TetLocalCompactTarget> Targets = BuildLocalCompactTargets(AItems, AMetaItems);
			const std::map<int, std::string> SkippedBins = LocalCompactBuildSkippedBins(AItems, AOptions);
			auto TimeBudgetReached = [&PassStart]() {
				return std::chrono::duration_cast<std::chrono::milliseconds>(
					std::chrono::steady_clock::now() - PassStart).count() >= CET_LOCAL_COMPACT_MAX_TIME_MS;
			};
			for (const auto& Entry : SkippedBins)
				std::cout << "[LOCAL COMPACT][SKIP BIN] bin=" << Entry.first << " reason=" << Entry.second << std::endl;
			const std::vector<double> AllowedRotations = CetRotationUtils::BuildAllowedRotations(AOptions.Rotations);
			std::cout << "[LOCAL COMPACT][PASS] targets=" << Targets.size() << " rotations=" << AllowedRotations.size() << std::endl;

			std::size_t ProcessedTargets = 0;
			bool TimedOut = false;
			for (std::size_t TargetNumber = 0; TargetNumber < Targets.size(); ++TargetNumber) {
				if (TimeBudgetReached() || ProcessedTargets >= CET_LOCAL_COMPACT_MAX_TARGETS) {
					TimedOut = TimeBudgetReached();
					break;
				}
				const TetLocalCompactTarget& Target = Targets[TargetNumber];
				if (Target.Indices.empty() || Target.Indices.front() >= AItems.size()) continue;
				const int BinId = static_cast<int>(AItems[Target.Indices.front()].binId());
				if (BinId < 0) continue;
				if (SkippedBins.find(BinId) != SkippedBins.end()) continue;
				std::vector<bool> TargetMask(AItems.size(), false);
				for (const std::size_t Index : Target.Indices) if (Index < TargetMask.size()) TargetMask[Index] = true;
				const TetLocalCompactEnvelope OldEnvelope = LocalCompactCalculateEnvelope(AItems, BinId);
				const TetLocalCompactEnvelope FixedEnvelope = LocalCompactCalculateEnvelope(AItems, BinId, &TargetMask);
				const TetLocalCompactEnvelope CurrentTargetEnvelope = LocalCompactCalculateTargetEnvelope(AItems, Target);
				const double BoundaryTolerance = 1.0;
				const bool TouchesEnvelope = OldEnvelope.Valid && CurrentTargetEnvelope.Valid
					&& (CurrentTargetEnvelope.MinX <= OldEnvelope.MinX + BoundaryTolerance
						|| CurrentTargetEnvelope.MinY <= OldEnvelope.MinY + BoundaryTolerance
						|| CurrentTargetEnvelope.MaxX >= OldEnvelope.MaxX - BoundaryTolerance
						|| CurrentTargetEnvelope.MaxY >= OldEnvelope.MaxY - BoundaryTolerance);
				if (!TouchesEnvelope) continue;
				++ProcessedTargets;
				const std::vector<TetLocalCompactFixedItem> FixedItems = LocalCompactBuildFixedItemCache(
					AItems, TargetMask, AOptions, BinId);
				std::vector<TetClusterFreeRegion> FreeRegions;
				const bool HasFreeRegions = BuildLocalCompactFreeRegions(AItems, AOptions, BinId, TargetMask, FreeRegions);
				std::cout << "[LOCAL COMPACT][BEGIN] index=" << TargetNumber << " type=" << Target.Type
					<< " isCluster=" << Target.IsCluster << " oldRotation=" << Target.CurrentRotation
					<< " allowedRotations=" << AllowedRotations.size() << " oldEnvelope=("
					<< OldEnvelope.Width << "," << OldEnvelope.Height << "," << OldEnvelope.Area << ","
					<< OldEnvelope.LongSide << ") freeRegions=" << FreeRegions.size() << std::endl;
				if (!HasFreeRegions || !OldEnvelope.Valid) {
					std::cout << "[LOCAL COMPACT][BEST] accepted=false reason=NO_FREE_REGION" << std::endl;
					continue;
				}

				bool HasBest = false;
				TetLocalCompactCandidate Best;
				std::size_t CandidateCount = 0;
				std::size_t ValidCandidateCount = 0;
				std::size_t OutOfBinCount = 0;
				std::size_t CollisionCount = 0;
				std::size_t SpacingCount = 0;
				const TetLocalCompactFreeSpaceMetric BaselineFreeSpace = LocalCompactCalculateFreeSpaceMetric(AItems, AOptions, BinId);
				std::vector<TetLocalCompactCandidate> FreeSpaceCandidates;
				std::vector<double> TargetRotations = AllowedRotations;
				if (Target.IsCluster) {
					std::stable_sort(TargetRotations.begin(), TargetRotations.end(), [&](double ALeft, double ARight) {
						const double LeftDelta = LocalCompactAngleDistance(ALeft, Target.CurrentRotation);
						const double RightDelta = LocalCompactAngleDistance(ARight, Target.CurrentRotation);
						const double LeftQuarterDistance = std::abs(LeftDelta - CET_CLUSTER_HALF_PI);
						const double RightQuarterDistance = std::abs(RightDelta - CET_CLUSTER_HALF_PI);
						return LeftQuarterDistance < RightQuarterDistance;
					});
				}
				for (const double Rotation : TargetRotations) {
					if (TimeBudgetReached()) { TimedOut = true; break; }
					CetTNestItemVector CandidateItems = AItems;
					LocalCompactApplyPose(CandidateItems, Target, Rotation, 0.0, 0.0);
					const TetLocalCompactEnvelope TemplateEnvelope = LocalCompactCalculateTargetEnvelope(CandidateItems, Target);
					if (!TemplateEnvelope.Valid) continue;
					CetPath TargetVertices;
					for (const std::size_t Index : Target.Indices) {
						const CetPolygonImpl Shape = CandidateItems[Index].transformedShape();
						TargetVertices.insert(TargetVertices.end(), Shape.Contour.begin(), Shape.Contour.end());
					}
					const std::vector<std::size_t> TargetContacts = LocalCompactSelectContactVertices(TargetVertices);
					std::map<std::pair<ClipperLib::cInt, ClipperLib::cInt>, int> Anchors;
					auto AddAnchor = [&Anchors](ClipperLib::cInt AX, ClipperLib::cInt AY, int AContactScore) {
						const auto Key = std::make_pair(AX, AY);
						auto Existing = Anchors.find(Key);
						if (Existing == Anchors.end()) Anchors.emplace(Key, AContactScore);
						else Existing->second = std::max(Existing->second, AContactScore);
					};
					auto AddContactAnchors = [&](const CetPath& AContactContour, int AContactScore) {
						const std::vector<std::size_t> ContactVertices = LocalCompactSelectContactVertices(AContactContour);
						for (const std::size_t TargetVertex : TargetContacts) for (const std::size_t ContactVertex : ContactVertices) {
							const ClipperLib::cInt X = AContactContour[ContactVertex].X - TargetVertices[TargetVertex].X;
							const ClipperLib::cInt Y = AContactContour[ContactVertex].Y - TargetVertices[TargetVertex].Y;
							// Free-region boundaries are spacing-inflated. Probe one integer unit around an exact
							// contact so rounding cannot turn a legal clearance touch into a spacing overlap.
							for (ClipperLib::cInt DX = -1; DX <= 1; ++DX) for (ClipperLib::cInt DY = -1; DY <= 1; ++DY)
								AddAnchor(X + DX, Y + DY, AContactScore);
						}
					};
					std::vector<std::pair<double, const CetPath*>> HoleContacts;
					for (const TetClusterFreeRegion& Region : FreeRegions) for (const CetPath& Hole : Region.Holes) {
						if (Hole.empty()) continue;
						double DistanceSquared = std::numeric_limits<double>::infinity();
						for (const std::size_t Vertex : LocalCompactSelectContactVertices(Hole)) {
							const double DX = static_cast<double>(Hole[Vertex].X) - Target.CurrentAnchorX;
							const double DY = static_cast<double>(Hole[Vertex].Y) - Target.CurrentAnchorY;
							DistanceSquared = std::min(DistanceSquared, DX * DX + DY * DY);
						}
						HoleContacts.emplace_back(DistanceSquared, &Hole);
					}
					std::stable_sort(HoleContacts.begin(), HoleContacts.end(), [](const auto& ALeft, const auto& ARight) {
						return ALeft.first < ARight.first;
					});
					std::vector<const CetPath*> SelectedHoleContacts;
					auto AddSelectedHole = [&SelectedHoleContacts](const CetPath* AHole) {
						if (AHole != nullptr && std::find(SelectedHoleContacts.begin(), SelectedHoleContacts.end(), AHole) == SelectedHoleContacts.end())
							SelectedHoleContacts.push_back(AHole);
					};
					if (!HoleContacts.empty()) {
						auto SelectExtreme = [&](auto ACoordinate, bool ASelectMinimum) {
							const auto Extreme = std::min_element(HoleContacts.begin(), HoleContacts.end(), [&](const auto& ALeft, const auto& ARight) {
								const double Left = ACoordinate(*ALeft.second);
								const double Right = ACoordinate(*ARight.second);
								return ASelectMinimum ? Left < Right : Left > Right;
							});
							AddSelectedHole(Extreme->second);
						};
						SelectExtreme([](const CetPath& APath) { return static_cast<double>(std::min_element(APath.begin(), APath.end(),
							[](const auto& ALeft, const auto& ARight) { return ALeft.X < ARight.X; })->X); }, true);
						SelectExtreme([](const CetPath& APath) { return static_cast<double>(std::max_element(APath.begin(), APath.end(),
							[](const auto& ALeft, const auto& ARight) { return ALeft.X < ARight.X; })->X); }, false);
						SelectExtreme([](const CetPath& APath) { return static_cast<double>(std::min_element(APath.begin(), APath.end(),
							[](const auto& ALeft, const auto& ARight) { return ALeft.Y < ARight.Y; })->Y); }, true);
						SelectExtreme([](const CetPath& APath) { return static_cast<double>(std::max_element(APath.begin(), APath.end(),
							[](const auto& ALeft, const auto& ARight) { return ALeft.Y < ARight.Y; })->Y); }, false);
					}
					for (const auto& HoleContact : HoleContacts) {
						if (SelectedHoleContacts.size() >= CET_LOCAL_COMPACT_MAX_HOLE_CONTACTS) break;
						AddSelectedHole(HoleContact.second);
					}
					for (const TetClusterFreeRegion& Region : FreeRegions) {
						const std::array<double, 2> XAnchors{ Region.MinX - TemplateEnvelope.MinX, Region.MaxX - TemplateEnvelope.MaxX };
						const std::array<double, 2> YAnchors{ Region.MinY - TemplateEnvelope.MinY, Region.MaxY - TemplateEnvelope.MaxY };
						for (const double X : XAnchors) for (const double Y : YAnchors)
							AddAnchor(static_cast<ClipperLib::cInt>(std::llround(X)), static_cast<ClipperLib::cInt>(std::llround(Y)), 0);
						AddContactAnchors(Region.Contour, 1);
					}
					// Keep both directional extremes and nearby holes. The extremes preserve empty
					// regions below/above a target that would otherwise be missed by nearest-only sampling.
					for (const CetPath* Hole : SelectedHoleContacts) AddContactAnchors(*Hole, 2);

					using TetAnchor = std::pair<std::pair<ClipperLib::cInt, ClipperLib::cInt>, int>;
					std::vector<TetAnchor> AllAnchors;
					AllAnchors.reserve(Anchors.size());
					for (const auto& Anchor : Anchors) AllAnchors.push_back({ Anchor.first, Anchor.second });
					std::vector<TetAnchor> CandidateAnchors;
					auto AddCandidateAnchor = [&CandidateAnchors](const TetAnchor& AAnchor) {
						if (std::find_if(CandidateAnchors.begin(), CandidateAnchors.end(), [&](const TetAnchor& AExisting) {
							return AExisting.first == AAnchor.first;
						}) == CandidateAnchors.end()) CandidateAnchors.push_back(AAnchor);
					};
					if (AllAnchors.size() <= CET_LOCAL_COMPACT_MAX_ANCHORS_PER_ROTATION) CandidateAnchors = AllAnchors;
					else {
						const int MaxScore = std::max_element(AllAnchors.begin(), AllAnchors.end(), [](const TetAnchor& ALeft, const TetAnchor& ARight) {
							return ALeft.second < ARight.second;
						})->second;
						std::vector<TetAnchor> BestContacts;
						for (const TetAnchor& Anchor : AllAnchors) if (Anchor.second == MaxScore) BestContacts.push_back(Anchor);
						if (!BestContacts.empty()) {
							AddCandidateAnchor(BestContacts.front());
							AddCandidateAnchor(BestContacts.back());
							AddCandidateAnchor(*std::min_element(BestContacts.begin(), BestContacts.end(), [](const TetAnchor& ALeft, const TetAnchor& ARight) { return ALeft.first.second < ARight.first.second; }));
							AddCandidateAnchor(*std::max_element(BestContacts.begin(), BestContacts.end(), [](const TetAnchor& ALeft, const TetAnchor& ARight) { return ALeft.first.second < ARight.first.second; }));
						}
						const std::size_t Remaining = CET_LOCAL_COMPACT_MAX_ANCHORS_PER_ROTATION - CandidateAnchors.size();
						for (std::size_t Slot = 0; Slot < Remaining; ++Slot) {
							const std::size_t Index = Remaining > 1 ? Slot * (AllAnchors.size() - 1) / (Remaining - 1) : 0;
							AddCandidateAnchor(AllAnchors[Index]);
						}
					}
					std::stable_sort(CandidateAnchors.begin(), CandidateAnchors.end(), [&](const TetAnchor& ALeft, const TetAnchor& ARight) {
						auto EstimatedEnvelope = [&](const TetAnchor& AAnchor) {
							TetLocalCompactEnvelope Placed = TemplateEnvelope;
							Placed.MinX += static_cast<double>(AAnchor.first.first);
							Placed.MaxX += static_cast<double>(AAnchor.first.first);
							Placed.MinY += static_cast<double>(AAnchor.first.second);
							Placed.MaxY += static_cast<double>(AAnchor.first.second);
							return LocalCompactMergeEnvelopes(FixedEnvelope, Placed);
						};
						const TetLocalCompactEnvelope LeftEnvelope = EstimatedEnvelope(ALeft);
						const TetLocalCompactEnvelope RightEnvelope = EstimatedEnvelope(ARight);
						if (LeftEnvelope.Area != RightEnvelope.Area) return LeftEnvelope.Area < RightEnvelope.Area;
						if (LeftEnvelope.LongSide != RightEnvelope.LongSide) return LeftEnvelope.LongSide < RightEnvelope.LongSide;
						return ALeft.second > ARight.second;
					});
					for (const TetAnchor& Anchor : CandidateAnchors) {
						if (TimeBudgetReached()) { TimedOut = true; break; }
						++CandidateCount;
						const double AnchorX = static_cast<double>(Anchor.first.first);
						const double AnchorY = static_cast<double>(Anchor.first.second);
						LocalCompactApplyPose(CandidateItems, Target, Rotation, AnchorX, AnchorY);
						const char* Reason = "OUT_OF_BIN";
						const bool Valid = LocalCompactIsTargetPoseValid(CandidateItems, Target, TargetMask, BinPoly,
							AOptions, BinId, FixedItems, Reason);
						TetLocalCompactCandidate Candidate;
						Candidate.Valid = Valid;
						Candidate.Rotation = Rotation;
						Candidate.AnchorX = AnchorX;
						Candidate.AnchorY = AnchorY;
						Candidate.ContactScore = Anchor.second;
						Candidate.TranslationDistance = std::hypot(AnchorX - Target.CurrentAnchorX, AnchorY - Target.CurrentAnchorY);
						Candidate.RotationDelta = LocalCompactAngleDistance(Rotation, Target.CurrentRotation);
						if (Valid) Candidate.Envelope = LocalCompactMergeEnvelopes(FixedEnvelope, LocalCompactCalculateTargetEnvelope(CandidateItems, Target));
						const bool StrictImprovement = Valid && LocalCompactIsStrictImprovement(Candidate.Envelope, OldEnvelope);
						const bool ContactImprovement = Valid && Candidate.ContactScore > 0
							&& LocalCompactIsNonWorsening(Candidate.Envelope, OldEnvelope);
						const bool Improved = StrictImprovement || ContactImprovement;
						if (Valid) ++ValidCandidateCount;
						else if (std::string(Reason) == "OUT_OF_BIN") ++OutOfBinCount;
						else if (std::string(Reason) == "COLLISION") ++CollisionCount;
						else if (std::string(Reason) == "SPACING") ++SpacingCount;
						if (Valid && LocalCompactIsNonWorsening(Candidate.Envelope, OldEnvelope)) FreeSpaceCandidates.push_back(Candidate);
						if (Improved && (!HasBest || LocalCompactIsCandidateBetter(Candidate, Best))) {
							std::cout << "[LOCAL COMPACT][CANDIDATE] rotation=" << Rotation << " translation=(" << AnchorX << "," << AnchorY
								<< ") contactScore=" << Candidate.ContactScore << " newEnvelope=(" << Candidate.Envelope.Width << ","
								<< Candidate.Envelope.Height << "," << Candidate.Envelope.Area << ") reason="
								<< (StrictImprovement ? "STRICT_IMPROVEMENT" : "CONTACT_IMPROVEMENT") << std::endl;
							HasBest = true;
							Best = Candidate;
						}
					}
					if (TimedOut) break;
				}
				if (BaselineFreeSpace.Valid && !FreeSpaceCandidates.empty()) {
					std::stable_sort(FreeSpaceCandidates.begin(), FreeSpaceCandidates.end(), LocalCompactIsCandidateBetter);
					std::vector<std::size_t> EvaluationIndices;
					auto AddEvaluationIndex = [&EvaluationIndices](std::size_t AIndex) {
						if (std::find(EvaluationIndices.begin(), EvaluationIndices.end(), AIndex) == EvaluationIndices.end())
							EvaluationIndices.push_back(AIndex);
					};
					for (std::size_t Index = 0; Index < FreeSpaceCandidates.size()
						&& EvaluationIndices.size() < CET_LOCAL_COMPACT_MAX_FREE_SPACE_EVALUATIONS / 2; ++Index) AddEvaluationIndex(Index);
					auto AddExtremeCandidate = [&](auto ACoordinate, bool ASelectMinimum) {
						const auto Extreme = std::min_element(FreeSpaceCandidates.begin(), FreeSpaceCandidates.end(), [&](const auto& ALeft, const auto& ARight) {
							const double Left = ACoordinate(ALeft);
							const double Right = ACoordinate(ARight);
							return ASelectMinimum ? Left < Right : Left > Right;
						});
						AddEvaluationIndex(static_cast<std::size_t>(std::distance(FreeSpaceCandidates.begin(), Extreme)));
					};
					AddExtremeCandidate([](const TetLocalCompactCandidate& ACandidate) { return ACandidate.AnchorX; }, true);
					AddExtremeCandidate([](const TetLocalCompactCandidate& ACandidate) { return ACandidate.AnchorX; }, false);
					AddExtremeCandidate([](const TetLocalCompactCandidate& ACandidate) { return ACandidate.AnchorY; }, true);
					AddExtremeCandidate([](const TetLocalCompactCandidate& ACandidate) { return ACandidate.AnchorY; }, false);
					for (const std::size_t Index : EvaluationIndices) {
						const TetLocalCompactCandidate& Candidate = FreeSpaceCandidates[Index];
						CetTNestItemVector CandidateItems = AItems;
						LocalCompactApplyPose(CandidateItems, Target, Candidate.Rotation, Candidate.AnchorX, Candidate.AnchorY);
						const TetLocalCompactFreeSpaceMetric CandidateFreeSpace = LocalCompactCalculateFreeSpaceMetric(CandidateItems, AOptions, BinId);
						if (!LocalCompactIsFreeSpaceBetter(CandidateFreeSpace, BaselineFreeSpace)) continue;
						if (!HasBest || LocalCompactIsCandidateBetter(Candidate, Best)) {
							std::cout << "[LOCAL COMPACT][RECOMPOSE] translation=(" << Candidate.AnchorX << "," << Candidate.AnchorY
								<< ") regions=" << BaselineFreeSpace.RegionCount << "->" << CandidateFreeSpace.RegionCount
								<< " largestFree=" << BaselineFreeSpace.LargestArea << "->" << CandidateFreeSpace.LargestArea << std::endl;
							HasBest = true;
							Best = Candidate;
						}
					}
				}
				std::cout << "[LOCAL COMPACT][SUMMARY] candidates=" << CandidateCount << " valid=" << ValidCandidateCount
					<< " outOfBin=" << OutOfBinCount << " collision=" << CollisionCount << " spacing=" << SpacingCount << std::endl;
				if (HasBest) {
					LocalCompactApplyPose(AItems, Target, Best.Rotation, Best.AnchorX, Best.AnchorY);
				}
				std::cout << "[LOCAL COMPACT][BEST] accepted=" << HasBest
					<< " rotation=" << (HasBest ? Best.Rotation : Target.CurrentRotation)
					<< " translation=(" << (HasBest ? Best.AnchorX : Target.CurrentAnchorX) << ","
					<< (HasBest ? Best.AnchorY : Target.CurrentAnchorY) << ") oldArea=" << OldEnvelope.Area
					<< " newArea=" << (HasBest ? Best.Envelope.Area : OldEnvelope.Area)
					<< " oldLongSide=" << OldEnvelope.LongSide
					<< " newLongSide=" << (HasBest ? Best.Envelope.LongSide : OldEnvelope.LongSide) << std::endl;
			}
			if (TimedOut) std::cout << "[LOCAL COMPACT][BUDGET] elapsedMs=" << CET_LOCAL_COMPACT_MAX_TIME_MS
				<< " processedTargets=" << ProcessedTargets << std::endl;

			if (!ValidatePlacedItemsSpacing(AItems, AOptions)) {
				AItems = BaselineItems;
				std::cout << "[LOCAL COMPACT][ROLLBACK] reason=FINAL_SPACING_VALIDATION" << std::endl;
			}
		}

		static void AccumulateFreeRegions(const ClipperLib::PolyNode& ANode, std::size_t& AInOutCount, double& AInOutArea, double& AInOutLargest)
		{
			if (!ANode.IsHole() && ANode.Contour.size() >= 3) {
				const double Area = std::abs(ClipperLib::Area(ANode.Contour));
				if (Area > 0.0) {
					++AInOutCount;
					AInOutArea += Area;
					AInOutLargest = std::max(AInOutLargest, Area);
				}
			}
			for (const ClipperLib::PolyNode* Child : ANode.Childs) if (Child != nullptr) {
				AccumulateFreeRegions(*Child, AInOutCount, AInOutArea, AInOutLargest);
			}
		}

		static void EvaluateBoardFreeRegionMetrics(const CetTNestItemVector& AItems, const TetNestOptions& AOptions, TetTNestEvalResult& AInOutResult)
		{
			AInOutResult.HasBoardFreeRegionMetric = false;
			AInOutResult.BoardFreeRegionCount = 0;
			AInOutResult.LargestFreeRegionArea = 0.0;
			AInOutResult.FragmentedFreeArea = 0.0;
			ClipperLib::Paths Board;
			if (!BuildBoardSubjectContours(AOptions, Board)) return;
			for (std::size_t Bin = 0; Bin < AInOutResult.Layers; ++Bin) {
				ClipperLib::Paths Reserved;
				if (!BuildPlacedReservedContours(AItems, static_cast<int>(Bin), AOptions.Spacing, Reserved)) return;
				ClipperLib::Clipper Difference;
				if (!Difference.AddPaths(Board, ClipperLib::ptSubject, true) || (!Reserved.empty() && !Difference.AddPaths(Reserved, ClipperLib::ptClip, true))) return;
				ClipperLib::PolyTree Tree;
				if (!Difference.Execute(ClipperLib::ctDifference, Tree, ClipperLib::pftNonZero, ClipperLib::pftNonZero)) return;
				std::size_t Count = 0;
				double TotalArea = 0.0;
				double LargestArea = 0.0;
				for (const ClipperLib::PolyNode* Node : Tree.Childs) if (Node != nullptr) AccumulateFreeRegions(*Node, Count, TotalArea, LargestArea);
				AInOutResult.HasBoardFreeRegionMetric = AInOutResult.HasBoardFreeRegionMetric || Count > 0;
				AInOutResult.BoardFreeRegionCount += Count;
				AInOutResult.LargestFreeRegionArea += LargestArea;
				AInOutResult.FragmentedFreeArea += std::max(0.0, TotalArea - LargestArea);
			}
		}

		static double GetMinimumPassableWidth(const CetTNestItemVector& AItems, const TetNestOptions& AOptions)
		{
			double MinimumWidth = std::numeric_limits<double>::infinity();
			const std::vector<Radians> Rotations = CetRotationUtils::BuildAllowedLibRotations(AOptions.Rotations);
			for (const CetNestItem& SourceItem : AItems) {
				CetNestItem Item = SourceItem;
				Item.translation(Point(0, 0));
				Item.inflation(0);
				for (const Radians Rotation : Rotations) {
					Item.rotation(Rotation);
					const auto Bounds = Item.boundingBox();
					const double ShortSide = std::min(std::abs(static_cast<double>(Bounds.width())),
						std::abs(static_cast<double>(Bounds.height())));
					if (ShortSide > 1.0 && std::isfinite(ShortSide)) MinimumWidth = std::min(MinimumWidth, ShortSide);
				}
			}
			return std::isfinite(MinimumWidth) ? MinimumWidth : 0.0;
		}

		static bool BuildInsetBoardContours(const ClipperLib::Paths& ABoard, libnest2d::Coord AInset,
			ClipperLib::Paths& AOutContours)
		{
			AOutContours = ABoard;
			if (AInset <= 0) return !AOutContours.empty();
			ClipperLib::ClipperOffset Offset(2.0, std::max(1.0, static_cast<double>(AInset) * 0.02));
			Offset.AddPaths(ABoard, ClipperLib::jtRound, ClipperLib::etClosedPolygon);
			Offset.Execute(AOutContours, -static_cast<double>(AInset));
			ClipperLib::CleanPolygons(AOutContours, 1.0);
			return !AOutContours.empty();
		}

		static void EvaluatePassableFreeRegionMetrics(const CetTNestItemVector& AItems, const TetNestOptions& AOptions,
			TetTNestEvalResult& AInOutResult)
		{
			AInOutResult.HasPassableFreeRegionMetric = false;
			AInOutResult.PassableFreeRegionCount = 0;
			AInOutResult.LargestPassableFreeRegionArea = 0.0;
			AInOutResult.FragmentedPassableFreeArea = 0.0;
			AInOutResult.MinimumPassableWidth = GetMinimumPassableWidth(AItems, AOptions);
			if (AInOutResult.MinimumPassableWidth <= 0.0) return;
			ClipperLib::Paths Board;
			if (!BuildBoardSubjectContours(AOptions, Board)) return;
			const auto SideAllowance = static_cast<libnest2d::Coord>(std::ceil(AInOutResult.MinimumPassableWidth * 0.5));
			ClipperLib::Paths AvailableBoard;
			if (!BuildInsetBoardContours(Board, SideAllowance, AvailableBoard)) return;
			for (std::size_t Bin = 0; Bin < AInOutResult.Layers; ++Bin) {
				ClipperLib::Paths Reserved;
				if (!BuildPlacedReservedContours(AItems, static_cast<int>(Bin), AOptions.Spacing, Reserved, SideAllowance)) return;
				ClipperLib::Clipper Difference;
				if (!Difference.AddPaths(AvailableBoard, ClipperLib::ptSubject, true)
					|| (!Reserved.empty() && !Difference.AddPaths(Reserved, ClipperLib::ptClip, true))) return;
				ClipperLib::PolyTree Tree;
				if (!Difference.Execute(ClipperLib::ctDifference, Tree, ClipperLib::pftNonZero, ClipperLib::pftNonZero)) return;
				std::size_t Count = 0;
				double TotalArea = 0.0;
				double LargestArea = 0.0;
				for (const ClipperLib::PolyNode* Node : Tree.Childs) if (Node != nullptr) AccumulateFreeRegions(*Node, Count, TotalArea, LargestArea);
				AInOutResult.HasPassableFreeRegionMetric = AInOutResult.HasPassableFreeRegionMetric || Count > 0;
				AInOutResult.PassableFreeRegionCount += Count;
				AInOutResult.LargestPassableFreeRegionArea += LargestArea;
				AInOutResult.FragmentedPassableFreeArea += std::max(0.0, TotalArea - LargestArea);
			}
		}

		static bool PreservesPassableFreeSpace(const TetTNestEvalResult& ACandidate, const TetTNestEvalResult& ABaseline)
		{
			if (!ACandidate.HasPassableFreeRegionMetric || !ABaseline.HasPassableFreeRegionMetric) return true;
			const double AreaTolerance = std::max({ 1.0, std::abs(ACandidate.LargestPassableFreeRegionArea),
				std::abs(ABaseline.LargestPassableFreeRegionArea) }) * 1e-9;
			if (ACandidate.FragmentedPassableFreeArea > ABaseline.FragmentedPassableFreeArea + AreaTolerance) return false;
			if (ACandidate.LargestPassableFreeRegionArea + AreaTolerance < ABaseline.LargestPassableFreeRegionArea) return false;
			return ACandidate.PassableFreeRegionCount <= ABaseline.PassableFreeRegionCount;
		}

		static bool IsBetterContinuousFreeSpace(const TetTNestEvalResult& ACandidate,
			const TetTNestEvalResult& ABaseline)
		{
			const bool UsePassableMetrics = ACandidate.HasPassableFreeRegionMetric
				&& ABaseline.HasPassableFreeRegionMetric;
			const bool UseBoardMetrics = ACandidate.HasBoardFreeRegionMetric
				&& ABaseline.HasBoardFreeRegionMetric;
			if (!UsePassableMetrics && !UseBoardMetrics) return false;

			const double CandidateArea = UsePassableMetrics
				? ACandidate.LargestPassableFreeRegionArea : ACandidate.LargestFreeRegionArea;
			const double BaselineArea = UsePassableMetrics
				? ABaseline.LargestPassableFreeRegionArea : ABaseline.LargestFreeRegionArea;
			const double RequiredGain = std::max(1.0, std::abs(BaselineArea) * 0.01);
			return CandidateArea >= BaselineArea + RequiredGain;
		}

		struct TetAllBinRemnantMetric
		{
			bool Valid = false;
			double ReusableStripArea = 0.0;
			double SkylineWasteArea = 0.0;
			double UsedEnvelopeArea = 0.0;
			std::vector<double> BinReusableStripAreas;
		};

		static TetAllBinRemnantMetric EvaluateAllBinRemnantMetric(const CetTNestItemVector& AItems,
			const TetNestOptions& AOptions, std::size_t ALayers)
		{
			TetAllBinRemnantMetric Result;
			if ((AOptions.Board.Enabled && AOptions.Board.Vertices.size() >= 3) || ALayers == 0) return Result;
			const double BinWidth = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth));
			const double BinHeight = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinHeight));
			if (BinWidth <= 0.0 || BinHeight <= 0.0) return Result;

			std::vector<std::vector<TetRemnantPartBounds>> BoundsByBin(ALayers);
			for (const CetNestItem& Item : AItems) {
				const int BinId = Item.binId();
				if (BinId < 0 || static_cast<std::size_t>(BinId) >= ALayers) continue;
				const auto Bounds = Item.boundingBox();
				TetRemnantPartBounds PartBounds;
				PartBounds.MinX = static_cast<double>(getX(Bounds.minCorner()));
				PartBounds.MinY = static_cast<double>(getY(Bounds.minCorner()));
				PartBounds.MaxX = static_cast<double>(getX(Bounds.maxCorner()));
				PartBounds.MaxY = static_cast<double>(getY(Bounds.maxCorner()));
				if (PartBounds.MaxX > PartBounds.MinX && PartBounds.MaxY > PartBounds.MinY) {
					BoundsByBin[static_cast<std::size_t>(BinId)].push_back(PartBounds);
				}
			}

			Result.BinReusableStripAreas.resize(ALayers, 0.0);
			for (std::size_t BinId = 0; BinId < BoundsByBin.size(); ++BinId) {
				const std::vector<TetRemnantPartBounds>& BinBounds = BoundsByBin[BinId];
				if (BinBounds.empty()) continue;
				std::array<double, CET_REMNANT_SKYLINE_SAMPLES> HorizontalSkyline{};
				std::array<double, CET_REMNANT_SKYLINE_SAMPLES> VerticalSkyline{};
				double UsedMaxX = 0.0;
				double UsedMaxY = 0.0;
				for (const TetRemnantPartBounds& Bounds : BinBounds) {
					const double MinX = std::clamp(Bounds.MinX, 0.0, BinWidth);
					const double MaxX = std::clamp(Bounds.MaxX, 0.0, BinWidth);
					const double MinY = std::clamp(Bounds.MinY, 0.0, BinHeight);
					const double MaxY = std::clamp(Bounds.MaxY, 0.0, BinHeight);
					UsedMaxX = std::max(UsedMaxX, MaxX);
					UsedMaxY = std::max(UsedMaxY, MaxY);
					const std::size_t StartX = std::min(CET_REMNANT_SKYLINE_SAMPLES - 1,
						static_cast<std::size_t>(std::floor(MinX / BinWidth * CET_REMNANT_SKYLINE_SAMPLES)));
					const std::size_t EndX = std::min(CET_REMNANT_SKYLINE_SAMPLES,
						static_cast<std::size_t>(std::ceil(MaxX / BinWidth * CET_REMNANT_SKYLINE_SAMPLES)));
					for (std::size_t Sample = StartX; Sample < EndX; ++Sample) {
						HorizontalSkyline[Sample] = std::max(HorizontalSkyline[Sample], MaxY);
					}
					const std::size_t StartY = std::min(CET_REMNANT_SKYLINE_SAMPLES - 1,
						static_cast<std::size_t>(std::floor(MinY / BinHeight * CET_REMNANT_SKYLINE_SAMPLES)));
					const std::size_t EndY = std::min(CET_REMNANT_SKYLINE_SAMPLES,
						static_cast<std::size_t>(std::ceil(MaxY / BinHeight * CET_REMNANT_SKYLINE_SAMPLES)));
					for (std::size_t Sample = StartY; Sample < EndY; ++Sample) {
						VerticalSkyline[Sample] = std::max(VerticalSkyline[Sample], MaxX);
					}
				}

				const double HorizontalStep = BinWidth / static_cast<double>(CET_REMNANT_SKYLINE_SAMPLES);
				const double VerticalStep = BinHeight / static_cast<double>(CET_REMNANT_SKYLINE_SAMPLES);
				const std::size_t UsedHorizontalSamples = std::min(CET_REMNANT_SKYLINE_SAMPLES,
					static_cast<std::size_t>(std::ceil(UsedMaxX / BinWidth * CET_REMNANT_SKYLINE_SAMPLES)));
				const std::size_t UsedVerticalSamples = std::min(CET_REMNANT_SKYLINE_SAMPLES,
					static_cast<std::size_t>(std::ceil(UsedMaxY / BinHeight * CET_REMNANT_SKYLINE_SAMPLES)));
				double TopWaste = 0.0;
				for (std::size_t Sample = 0; Sample < UsedHorizontalSamples; ++Sample) {
					TopWaste += std::max(0.0, UsedMaxY - HorizontalSkyline[Sample]) * HorizontalStep;
				}
				double RightWaste = 0.0;
				for (std::size_t Sample = 0; Sample < UsedVerticalSamples; ++Sample) {
					RightWaste += std::max(0.0, UsedMaxX - VerticalSkyline[Sample]) * VerticalStep;
				}
				const double TopArea = BinWidth * std::max(0.0, BinHeight - UsedMaxY);
				const double RightArea = BinHeight * std::max(0.0, BinWidth - UsedMaxX);
				const bool PreferTop = TopArea > RightArea || (std::abs(TopArea - RightArea) <= 1.0 && TopWaste <= RightWaste);
				const double ReusableArea = PreferTop ? TopArea : RightArea;
				Result.BinReusableStripAreas[BinId] = ReusableArea;
				Result.ReusableStripArea += ReusableArea;
				Result.SkylineWasteArea += PreferTop ? TopWaste : RightWaste;
				Result.UsedEnvelopeArea += UsedMaxX * UsedMaxY;
				Result.Valid = true;
			}
			return Result;
		}

		bool CetNest2DEngine::_TryBoardFeedbackNest(CetTNestItemVector& ANestItems, const TetNestOptions& AOptions, TetNestProgressTracker& ATracker, std::size_t& ALayers)
		{
			if (ANestItems.empty() || ALayers <= 1 || ANestItems.size() > CET_BOARD_FEEDBACK_NEST_MAX_ITEM_COUNT) {
				std::cout << "[BOARD FEEDBACK][SKIP] Items=" << ANestItems.size()
					<< " Layers=" << ALayers
					<< " Limit=" << CET_BOARD_FEEDBACK_NEST_MAX_ITEM_COUNT << std::endl;
				return false;
			}
			const CetTNestItemVector OriginalSolution = ANestItems;
			const std::size_t OriginalLayers = ALayers;
			const TetTNestEvalResult OriginalEval = Nest2DUtils->Nest2DStrategy->EvaluateNestResult(OriginalSolution, OriginalLayers);
			std::vector<std::size_t> Order(ANestItems.size());
			std::iota(Order.begin(), Order.end(), 0);
			std::stable_sort(Order.begin(), Order.end(), [&](std::size_t A, std::size_t B) {
				const CetNestItem& First = OriginalSolution[A];
				const CetNestItem& Second = OriginalSolution[B];
				if (First.binId() != Second.binId()) return First.binId() < Second.binId();
				const Point FirstTranslation = First.translation();
				const Point SecondTranslation = Second.translation();
				if (FirstTranslation.Y != SecondTranslation.Y) return FirstTranslation.Y < SecondTranslation.Y;
				if (FirstTranslation.X != SecondTranslation.X) return FirstTranslation.X < SecondTranslation.X;
				return A < B;
				});
			CetTNestItemVector FeedbackItems;
			FeedbackItems.reserve(Order.size());
			for (std::size_t Index : Order) {
				CetNestItem Item = OriginalSolution[Index];
				Item.binId(-1);
				Item.translation(Point(0, 0));
				Item.rotation(Radians(0.0));
				Item.inflation(0);
				FeedbackItems.push_back(std::move(Item));
			}
			const bool UsePolygonBoard = AOptions.Board.Enabled && AOptions.Board.Vertices.size() >= 3;
			std::size_t FeedbackLayers = UsePolygonBoard
				? RunPolygonNestOnce(FeedbackItems, AOptions, ATracker)
				: RunRectangleNestOnce(FeedbackItems, AOptions, ATracker);
			CetTNestItemVector FeedbackSolution = OriginalSolution;
			for (std::size_t Position = 0; Position < Order.size(); ++Position) FeedbackSolution[Order[Position]] = std::move(FeedbackItems[Position]);
			const TetTNestEvalResult FeedbackEval = Nest2DUtils->Nest2DStrategy->EvaluateNestResult(FeedbackSolution, FeedbackLayers);
			const bool Valid = FeedbackLayers > 0 && ValidatePlacedItemsSpacing(FeedbackSolution, AOptions);
			const bool Improved = Valid && Nest2DUtils->Nest2DStrategy->IsBetterNestResult(FeedbackEval, OriginalEval);
			std::cout << "[BOARD FEEDBACK][RESULT] BeforeLayers=" << OriginalLayers
				<< " AfterLayers=" << FeedbackLayers
				<< " BeforeFirstArea=" << OriginalEval.FirstBinArea
				<< " AfterFirstArea=" << FeedbackEval.FirstBinArea
				<< " Valid=" << Valid << " Improved=" << Improved << std::endl;
			if (!Improved) return false;
			ANestItems = std::move(FeedbackSolution);
			ALayers = FeedbackLayers;
			return true;
		}
		
		int CetNest2DEngine::RunNesting_Impl(CetTNestItemVector& ANestItems, const TetNestOptions& AOptions, std::size_t* AUsedBins)
		{
			std::cout << "[DLL]this is running nesting" << std::endl;
			if (AUsedBins != nullptr){
				*AUsedBins = 0;
			}

			if (ANestItems.empty()){
				return NEST2D_ERR_CORE_EMPTY_INPUT;
			}
			const bool UsePolygonBoard = AOptions.Board.Enabled && AOptions.Board.Vertices.size() >= 3;

			int TotalItems = static_cast<int>(ANestItems.size());
			TetNestProgressTracker Tracker(TotalItems, AOptions.ProgressCallback);

			std::size_t Layers = 0;

			if (UsePolygonBoard){
				Layers = RunPolygonBoardNesting(ANestItems, AOptions, Tracker);
			}
			else {
				Layers = RunRectangleBoardNesting(ANestItems, AOptions, Tracker);
			}
			std::cout << "[NEST] after polygon nest, Layers = " << Layers << std::endl;

			if (Layers == 0){
				return NEST2D_ERR_CORE_NESTING_FAILED;
			}
			if (!ValidatePlacedItemsSpacing(ANestItems, AOptions)){
				std::cout << "[NEST][FINAL][ERROR] Result does not meet spacing requirements." << std::endl;
				return NEST2D_ERR_CORE_NESTING_FAILED;
			}

			if (AUsedBins != nullptr){
				*AUsedBins = Layers;
			}

			return Nest2D_Success;
		}

		std::size_t CetNest2DEngine::RunPolygonBoardNesting(CetTNestItemVector& ANestItems, const TetNestOptions& AOptions, TetNestProgressTracker& ATracker)
		{
			std::cout << "[NEST] use custom polygon board with strategy loop" << std::endl;

			CetTNestItemVector OriginalItems = ANestItems;
			const std::vector<TetShapeFeature> Features =Nest2DUtils->Nest2DShape->AnalyzeALL(OriginalItems);
			std::cout << "[SHAPE ANALYZER][DONE]" << " ItemCount = " << OriginalItems.size() << ", FeatureCount = " << Features.size() << std::endl;

			bool HasBest = false;
			CetTNestItemVector BestItems;
			TetTNestEvalResult BestEval{};
			std::size_t BestLayers = 0;
			std::vector<TetMetaItem> BestMetaItems;
			bool BestHasCluster = false;
			bool BestHasLockedEnvelope = false;

			const std::vector<MetClusterStrategy> ClusterStrategies = BuildClusterStrategies(Features);

			for (auto ClusterStrategy : ClusterStrategies){
				TetClusterBuildResult ClusterResult =Nest2DUtils->Nest2DCluster->BuildClusterItemsWithFeatures(OriginalItems,Features,AOptions,ClusterStrategy);
				int ClusterCount = 0;
				for (const auto& Meta : ClusterResult.MetaItems){
					if (Meta.IsCluster){
						ClusterCount++;
					}
				}

				std::cout << "[POLYGON][CLUSTER][BUILD] Strategy = "
					<< static_cast<int>(ClusterStrategy)
					<< ", OriginalItems = " << OriginalItems.size()
					<< ", PackedItems = " << ClusterResult.NestItems.size()
					<< ", MetaItems = " << ClusterResult.MetaItems.size()
					<< ", ClusterCount = " << ClusterCount
					<< std::endl;

				TetExpandedSpacingFailure SpacingFailure;
				TetLocalBestResult LocalResult = EvaluateSortingStrategies(ClusterResult, OriginalItems, AOptions, ATracker, &SpacingFailure);
				if (!LocalResult.HasBest && SpacingFailure.Valid) {
					LocalResult = _TryLocalClusterSpacingFallback(ClusterResult, OriginalItems, AOptions, ATracker, SpacingFailure);
				}

				const bool LocalHasLockedEnvelope = HasLockedEnvelopeCluster(LocalResult.MetaItems);
				bool Better = ShoouldUpdateGlobalBest(LocalResult, HasBest, BestEval, BestLayers, BestHasCluster);
				if (!Better && LocalHasLockedEnvelope && !BestHasLockedEnvelope) {
					// A locked envelope protects its fixed outer contour during repair,
					// but it must not displace a layout with better board utilization.
					Better = !Nest2DUtils->Nest2DStrategy->IsBetterNestResult(BestEval, LocalResult.Eval);
				}

				if (Better){
					HasBest = true;
					BestEval = LocalResult.Eval;
					BestLayers = LocalResult.Layers;
					BestItems = std::move(LocalResult.Items);
					BestMetaItems = std::move(LocalResult.MetaItems);
					BestHasCluster = LocalResult.HasCluster;
					BestHasLockedEnvelope = LocalHasLockedEnvelope;

					std::cout << "[POLYGON][GLOBAL BEST UPDATE] HasCluster = "
						<< BestHasCluster
						<< ", count = " << BestEval.FirstBinCount
						<< ", area = " << BestEval.FirstBinArea
						<< ", layers = " << BestEval.Layers
						<< ", packedItems = " << BestItems.size()
						<< std::endl;
				}
			}

			if (!HasBest){
				std::cout << "[NEST][SPACING FALLBACK][FULL SINGLE] Trigger=all clustered candidates rejected." << std::endl;
				TetClusterBuildResult SingleItems = Nest2DUtils->Nest2DCluster->BuildClusterItems(OriginalItems, AOptions, MetClusterStrategy::None);
				TetLocalBestResult FallbackResult = _EvaluateSingleSortingStrategy(SingleItems, OriginalItems, AOptions, ATracker, MetENestOrderStrategy::LargeFirst);
				if (FallbackResult.HasBest) {
					HasBest = true;
					BestEval = FallbackResult.Eval;
					BestLayers = FallbackResult.Layers;
					BestItems = std::move(FallbackResult.Items);
					BestMetaItems = std::move(FallbackResult.MetaItems);
					BestHasCluster = false;
					std::cout << "[NEST][SPACING FALLBACK][FULL SINGLE][VALID] Layers=" << BestLayers << std::endl;
				}
				else {
					std::cout << "[POLYGON][FINAL] no valid best result." << std::endl;
					return 0;
				}
			}

			if (!BestHasCluster){
				std::cout << "[POLYGON][FINAL BEST] Restore normal item order." << std::endl;
			}
			else {
				std::cout << "[POLYGON][FINAL BEST] Use cluster expand." << std::endl;
			}
			const long double CoordinateScale = NestUtils::NestScale();
			std::cout << "[POLYGON][FINAL REMNANT] MetricsAvailable=" << BestEval.HasRemnantMetrics
				<< ", AreaMm2=" << static_cast<double>(BestEval.ReusableRemnantArea / (CoordinateScale * CoordinateScale))
				<< ", ShortSideMm=" << static_cast<double>(BestEval.ReusableRemnantShortSide / CoordinateScale)
				<< ", UsedDepthMm=" << static_cast<double>(BestEval.UsedDepth / CoordinateScale)
				<< ", Direction=" << (BestEval.RemnantIsTopStrip ? "Top" : "Right")
				<< std::endl;
			// Sorting strategies reorder packed items, so metadata restoration is required for singles and clusters.
			Nest2DUtils->Nest2DCluster->ExpandClusterResultToOriginalItems(OriginalItems,BestItems,BestMetaItems,ANestItems);
			
			double BoardBinWidth = AOptions.BinWidth;
			double BoardBinHeight = AOptions.BinHeight;
			CetPolygonImpl BinPoly = Nest2DUtils->Nest2DBord->BuildBinPolygonFromOptions(AOptions,BoardBinWidth,BoardBinHeight);

			if (!BestHasLockedEnvelope && _RepairAndEvacuate(ANestItems, AOptions, BinPoly, BoardBinWidth, BoardBinHeight, BestLayers)) {
				_TryBoardFeedbackNest(ANestItems, AOptions, ATracker, BestLayers);
			}
			else if (BestHasLockedEnvelope) {
				std::cout << "[POLYGON][LOCKED ENVELOPE] Skip expanded-item repair." << std::endl;
			}
			BestEval = Nest2DUtils->Nest2DStrategy->EvaluateNestResult(ANestItems, BestLayers);
			RunLocalCompactPass(ANestItems, AOptions, &BestMetaItems);
			BestEval = Nest2DUtils->Nest2DStrategy->EvaluateNestResult(ANestItems, BestLayers);
			std::cout << "================ POLYGON BEST NEST RESULT ================" << std::endl;
			std::cout << "[POLYGON BEST] bin0 count = " << BestEval.FirstBinCount << ", bin0 area = " << BestEval.FirstBinArea << ", layers = " << BestLayers << std::endl;

			Nest2DUtils->Nest2DStrategy->PrintBinCount(ANestItems);
			std::cout << "===========================================================" << std::endl;

			return BestLayers;
		}

		std::size_t CetNest2DEngine::RunPolygonNestOnce(CetTNestItemVector& ATestItems, const TetNestOptions& AOptions, TetNestProgressTracker& ATracker)
		{
			double BoardBinWidth = AOptions.BinWidth;
			double BoardBinHeight = AOptions.BinHeight;

			CetPolygonImpl BinPoly = Nest2DUtils->Nest2DBord->BuildBinPolygonFromOptions(AOptions,BoardBinWidth,BoardBinHeight);

			using CetMyPlacer = placers::_NofitPolyPlacer<CetPolygonImpl, CetPolygonImpl>;
			// Keep the primary ordering stable; expanded items are backfilled after nesting.
			using CetMySelector = selections::_FirstFitSelection<CetPolygonImpl>;

			NestConfig<CetMyPlacer, CetMySelector> cfg;

			cfg.placer_config.alignment =
				placers::NfpPConfig<CetPolygonImpl>::Alignment::DONT_ALIGN;

			cfg.placer_config.starting_point =
				placers::NfpPConfig<CetPolygonImpl>::Alignment::BOTTOM_LEFT;

			cfg.placer_config.accuracy = 1.0f;
			cfg.placer_config.parallel = true;
			cfg.placer_config.explore_holes = false;

			FillRotations(cfg.placer_config.rotations, AOptions.Rotations);

			std::cout << "================ POLYGON ONCE DEBUG ================" << std::endl;
			std::cout << "UsePolygonBoard: true" << std::endl;
			std::cout << "BoardBinWidth: " << BoardBinWidth << ", BoardBinHeight: " << BoardBinHeight << std::endl;
			std::cout << "Spacing: " << NestUtils::ToNestCoord(AOptions.Spacing) << std::endl;
			std::cout << "Board.Vertices.size: " << AOptions.Board.Vertices.size() << std::endl;
			std::cout << "====================================================" << std::endl;

			std::size_t Layers = nest(ATestItems,BinPoly,NestUtils::ToNestCoord(AOptions.Spacing),cfg,ProgressFunction{ ATracker });

			std::cout << "[POLYGON ONCE] before repair, Layers = " << Layers << std::endl;

			Nest2DUtils->Nest2DPolygonBord->SetContext(ATestItems,AOptions,BinPoly,BoardBinWidth,BoardBinHeight);

			Nest2DUtils->Nest2DPolygonBord->Repair(Layers);

			std::cout << "[POLYGON ONCE] after repair, Layers = " << Layers << std::endl;

			Nest2DUtils->Nest2DStrategy->PrintBinCount(ATestItems);

			return Layers;
		}

		std::size_t CetNest2DEngine::RunRectangleBoardNesting(CetTNestItemVector& ANestItems, const TetNestOptions& AOptions, TetNestProgressTracker& ATracker)
		{
			std::cout << "[NEST] use original rectangle BIN" << std::endl;
			CetTNestItemVector OriginalItems = ANestItems;
			//CetShapeAnalyzer ShapeAnalyzer;
			const std::vector<TetShapeFeature> Features =Nest2DUtils->Nest2DShape->AnalyzeALL(OriginalItems);
			std::cout << "[SHAPE ANALYZER][DONE]" << " ItemCount = " << OriginalItems.size() << ", FeatureCount = " << Features.size() << std::endl;
			
			bool HasBest = false;
			CetTNestItemVector BestItems;
			TetTNestEvalResult BestEval{};
			std::size_t BestLayers = 0;
			std::vector<TetMetaItem> BestMetaItems;
			bool BestHasCluster = false;
			bool BestHasLockedEnvelope = false;
			
			const std::vector<MetClusterStrategy> ClusterStrategies = BuildClusterStrategies(Features);
			for (auto ClusterStrategy : ClusterStrategies){
				
			//	TetClusterBuildResult ClusterResult = Nest2DUtils->Nest2DCluster->BuildClusterItems(OriginalItems, AOptions, ClusterStrategy);
				TetClusterBuildResult ClusterResult = Nest2DUtils->Nest2DCluster->BuildClusterItemsWithFeatures(OriginalItems,Features, AOptions, ClusterStrategy);
				
				int ClusterCount = 0;
				for (const auto& Meta : ClusterResult.MetaItems){
					if (Meta.IsCluster) ClusterCount++;
				}
				std::cout << "[CLUSTER][BUILD] Strategy = " << static_cast<int>(ClusterStrategy)
					<< ", OriginalItems = " << OriginalItems.size()
					<< ", PackedItems = " << ClusterResult.NestItems.size()
					<< ", MetaItems = " << ClusterResult.MetaItems.size()
					<< ", ClusterCount = " << ClusterCount << std::endl;  

				
				TetExpandedSpacingFailure SpacingFailure;
				TetLocalBestResult LocalResult = EvaluateSortingStrategies(ClusterResult, OriginalItems, AOptions, ATracker, &SpacingFailure);
				if (!LocalResult.HasBest && SpacingFailure.Valid) {
					LocalResult = _TryLocalClusterSpacingFallback(ClusterResult, OriginalItems, AOptions, ATracker, SpacingFailure);
				}
				const bool LocalHasLockedEnvelope = HasLockedEnvelopeCluster(LocalResult.MetaItems);
				bool Better = ShoouldUpdateGlobalBest(LocalResult, HasBest, BestEval, BestLayers, BestHasCluster);
				if (!Better && LocalHasLockedEnvelope && !BestHasLockedEnvelope) {
					// Keep the fixed outline intact only when it does not regress the
					// evaluated layout; the protection is not a selection score.
					Better = !Nest2DUtils->Nest2DStrategy->IsBetterNestResult(BestEval, LocalResult.Eval);
				}
				
				if (Better){
					HasBest = true;
					BestEval = LocalResult.Eval;
					BestLayers = LocalResult.Layers;
					
					BestItems = std::move(LocalResult.Items);
					BestMetaItems = std::move(LocalResult.MetaItems);
					BestHasCluster = LocalResult.HasCluster;
					BestHasLockedEnvelope = LocalHasLockedEnvelope;
					if (BestHasLockedEnvelope) {
						std::cout << "[NEST][LOCKED ENVELOPE] Preserve completed envelope-fill composite." << std::endl;
					}
					std::cout << "[NEST][GLOBAL BEST UPDATE] HasCluster = " << BestHasCluster
						<< ", count = " << BestEval.FirstBinCount
						<< ", area = " << BestEval.FirstBinArea
						<< ", layers = " << BestEval.Layers
						<< ", packedItems = " << BestItems.size() << std::endl;
				}
			}
			
			if (!HasBest) {
				std::cout << "[NEST][SPACING FALLBACK][FULL SINGLE] Trigger=all clustered candidates rejected." << std::endl;
				TetClusterBuildResult SingleItems = Nest2DUtils->Nest2DCluster->BuildClusterItems(OriginalItems, AOptions, MetClusterStrategy::None);
				TetLocalBestResult FallbackResult = _EvaluateSingleSortingStrategy(SingleItems, OriginalItems, AOptions, ATracker, MetENestOrderStrategy::LargeFirst);
				if (FallbackResult.HasBest) {
					HasBest = true;
					BestEval = FallbackResult.Eval;
					BestLayers = FallbackResult.Layers;
					BestItems = std::move(FallbackResult.Items);
					BestMetaItems = std::move(FallbackResult.MetaItems);
					BestHasCluster = false;
					std::cout << "[NEST][SPACING FALLBACK][FULL SINGLE][VALID] Layers=" << BestLayers << std::endl;
				}
			}

			if (HasBest){
				std::cout << "[NEST][FINAL BEST] BestHasCluster = " << BestHasCluster << ", BestItems.size = " << BestItems.size() << ", BestMetaItems.size = " << BestMetaItems.size() << std::endl;

				if (!BestHasCluster){
					std::cout << "[NEST][FINAL BEST] Restore normal item order." << std::endl;
				}
				else {
					std::cout << "[NEST][FINAL BEST] Use cluster expand." << std::endl;
				}
				const long double CoordinateScale = NestUtils::NestScale();
				std::cout << "[NEST][FINAL REMNANT] MetricsAvailable=" << BestEval.HasRemnantMetrics
					<< ", AreaMm2=" << static_cast<double>(BestEval.ReusableRemnantArea / (CoordinateScale * CoordinateScale))
					<< ", ShortSideMm=" << static_cast<double>(BestEval.ReusableRemnantShortSide / CoordinateScale)
					<< ", SkylineWasteMm2=" << static_cast<double>(BestEval.SkylineWasteArea / (CoordinateScale * CoordinateScale))
					<< ", UsedDepthMm=" << static_cast<double>(BestEval.UsedDepth / CoordinateScale)
					<< ", Direction=" << (BestEval.RemnantIsTopStrip ? "Top" : "Right")
					<< std::endl;
				// Sorting strategies reorder packed items, so metadata restoration is required for singles and clusters.
				const CetTNestItemVector ItemsBeforeBackfill = BestItems;
				const std::size_t LayersBeforeBackfill = BestLayers;
				BestLayers = BackfillClusterSheets(BestItems, AOptions, BestLayers);
				if (!Nest2DUtils->Nest2DCluster->ValidatePackedResultSpacing(OriginalItems, BestItems, BestMetaItems, AOptions)){
					std::cout << "[NEST][CLUSTER BACKFILL][ROLLBACK] Expanded validation failed." << std::endl;
					BestItems = ItemsBeforeBackfill;
					BestLayers = LayersBeforeBackfill;
				}
				else {
					BestEval = Nest2DUtils->Nest2DStrategy->EvaluatePackedResultWithMeta(BestItems, BestMetaItems, OriginalItems, AOptions, BestLayers);
					std::cout << "[NEST][CLUSTER BACKFILL][VALID] FirstBinCount=" << BestEval.FirstBinCount
						<< ", Layers=" << BestEval.Layers << std::endl;
				}
				Nest2DUtils->Nest2DCluster->ExpandClusterResultToOriginalItems(OriginalItems, BestItems, BestMetaItems, ANestItems);
				std::cout << "[NEST][REPAIR PREP] ExpandedItems=" << ANestItems.size() << " Layers=" << BestLayers << std::endl;
				CetPolygonImpl RectBinPoly = Nest2DUtils->Nest2DBord->BuildRectangleBinPolygon(AOptions.BinWidth, AOptions.BinHeight);
				std::cout << "[NEST][REPAIR PREP] RectangleBinReady Contour=" << RectBinPoly.Contour.size() << std::endl;
				if (!BestHasLockedEnvelope && _RepairAndEvacuate(ANestItems, AOptions, RectBinPoly, AOptions.BinWidth, AOptions.BinHeight, BestLayers)) {
					_TryBoardFeedbackNest(ANestItems, AOptions, ATracker, BestLayers);
				}
				else if (BestHasLockedEnvelope) {
					const CetTNestItemVector BeforeEvacuation = ANestItems;
					const std::size_t LayersBeforeEvacuation = BestLayers;
					const std::vector<std::size_t> LockedChildren = CollectLockedEnvelopeChildren(BestMetaItems);
					if (!_TryLockedEnvelopeBoardRepair(ANestItems, AOptions, RectBinPoly, AOptions.BinWidth,
						AOptions.BinHeight, LockedChildren, BestLayers)) {
						TetNestOptions EvacuationOptions = AOptions;
						EvacuationOptions.EnableLastBinEvacuation = true;
						if (_RunLastBinEvacuation(ANestItems, EvacuationOptions, BestLayers)
							&& PreservesLockedChildren(BeforeEvacuation, ANestItems, LockedChildren)) {
							std::cout << "[NEST][LOCKED ENVELOPE] Last-bin direct backfill accepted." << std::endl;
						}
						else {
							ANestItems = BeforeEvacuation;
							BestLayers = LayersBeforeEvacuation;
							std::cout << "[NEST][LOCKED ENVELOPE] Board repair and direct backfill rejected." << std::endl;
						}
					}
				}
				TryCompactUniformRectangleHoles(ANestItems, AOptions);
				TryFillRectangleGridEdgeFromCompatibleGroup(ANestItems, AOptions);
				if (!BestHasCluster) {
					RunLocalCompactPass(ANestItems, AOptions, &BestMetaItems);
				}
				BestEval = Nest2DUtils->Nest2DStrategy->EvaluateNestResult(ANestItems, BestLayers);
				EvaluateInternalGapMetrics(ANestItems, AOptions, BestEval);
			}
				std::cout << "================ BEST NEST RESULT ================" << std::endl;
				std::cout << "[NEST BEST] bin0 count = " << BestEval.FirstBinCount
					<< ", bin0 area = " << BestEval.FirstBinArea
					<< ", layers = " << BestEval.Layers
					<< ", internal gap area = " << BestEval.InternalGapArea
					<< ", internal gap count = " << BestEval.InternalGapCount
					<< std::endl;
			Nest2DUtils->Nest2DStrategy->PrintBinCount(ANestItems);
			std::cout << "==================================================" << std::endl;

			return BestLayers;
		}

		std::size_t CetNest2DEngine::RunRectangleNestOnce(CetTNestItemVector& ATestItems,
			const TetNestOptions& AOptions, TetNestProgressTracker& ATracker, bool AUseFillerSelector,
			bool AAllowRotations)
		{
			if (AUseFillerSelector) {
				return RunRectangleNestWithSelector<selections::_FillerSelection<CetPolygonImpl>>(
					ATestItems, AOptions, ATracker, AAllowRotations);
			}
			return RunRectangleNestWithSelector<selections::_FirstFitSelection<CetPolygonImpl>>(
				ATestItems, AOptions, ATracker, AAllowRotations);
		}

		bool CetNest2DEngine::_HasClusterItems(const std::vector<TetMetaItem>& AMetaItems) const
		{
			return std::any_of(AMetaItems.begin(), AMetaItems.end(), [](const TetMetaItem& AMeta) { return AMeta.IsCluster; });
		}

		std::vector<std::size_t> CetNest2DEngine::_BuildPriorityOrder(CetTNestItemVector& AItems, const TetNestOptions& AOptions, MetENestOrderStrategy AStrategy) const
		{
			Nest2DUtils->Nest2DStrategy->ApplyNestPriorityStrategy(AItems, AOptions, AStrategy);
			std::vector<std::size_t> Indices(AItems.size());
			std::iota(Indices.begin(), Indices.end(), 0);
			std::stable_sort(Indices.begin(), Indices.end(), [&](std::size_t A, std::size_t AB) {
				const int PriorityA = AItems[A].priority();
				const int PriorityB = AItems[AB].priority();
				if (PriorityA != PriorityB) return PriorityA > PriorityB;
				const double AreaA = std::abs(static_cast<double>(AItems[A].area()));
				const double AreaB = std::abs(static_cast<double>(AItems[AB].area()));
				return std::abs(AreaA - AreaB) > 1e-6 ? AreaA > AreaB : A < AB;
			});
			return Indices;
		}

		void CetNest2DEngine::_BuildSortedTestData(CetTNestItemVector& APriorityItems, const std::vector<TetMetaItem>& AMetaItems, const std::vector<std::size_t>& ASortedIndices, CetTNestItemVector& AOutItems, std::vector<TetMetaItem>& AOutMetaItems) const
		{
			AOutItems.reserve(APriorityItems.size());
			AOutMetaItems.reserve(AMetaItems.size());
			for (std::size_t Index : ASortedIndices){
				AOutItems.push_back(std::move(APriorityItems[Index]));
				AOutMetaItems.push_back(AMetaItems[Index]);
				AOutMetaItems.back().PackedItemIndex = static_cast<int>(AOutMetaItems.size() - 1);
			}
		}

		void CetNest2DEngine::_UpdateLocalBest(TetLocalBestResult& ALocalBest, TetTNestEvalResult AEvaluation, std::size_t ALayers, CetTNestItemVector& AItems, std::vector<TetMetaItem>& AMetaItems, bool AHasCluster) const
		{
			bool Better = !ALocalBest.HasBest;
			if (!Better && Nest2DUtils->Nest2DStrategy->IsBetterNestResult(AEvaluation, ALocalBest.Eval)) Better = true;
			if (!Better && AHasCluster && !ALocalBest.HasCluster) Better = !Nest2DUtils->Nest2DStrategy->IsBetterNestResult(ALocalBest.Eval, AEvaluation);
			if (!Better) return;
			ALocalBest.HasBest = true;
			ALocalBest.Eval = AEvaluation;
			ALocalBest.Layers = ALayers;
			ALocalBest.Items = std::move(AItems);
			ALocalBest.MetaItems = std::move(AMetaItems);
			ALocalBest.HasCluster = AHasCluster;
			std::cout << "[NEST][LOCAL BEST UPDATE] HasCluster = " << ALocalBest.HasCluster << ", count = " << ALocalBest.Eval.FirstBinCount << ", area = " << ALocalBest.Eval.FirstBinArea << ", layers = " << ALocalBest.Eval.Layers << ", packedItems = " << ALocalBest.Items.size() << std::endl;
		}

		void CetNest2DEngine::_TryQuarterTurnCandidates(TetLocalBestResult& ALocalBest,const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions,TetNestProgressTracker& ATracker, bool AHasCluster)
		{
			if (!ALocalBest.HasBest || ALocalBest.Items.size() > CET_NEST_FULL_STRATEGY_ITEM_LIMIT
				|| AOptions.Board.Enabled
				|| !CetRotationUtils::IsAllowedRotation(CET_CLUSTER_HALF_PI, AOptions.Rotations, 1e-9)) return;
			(void)ATracker;
			constexpr std::size_t MaxTargets = 8;
			constexpr std::size_t MaxPlacementsPerTarget = 48;
			const auto StartTime = std::chrono::steady_clock::now();
			constexpr auto Budget = std::chrono::milliseconds(250);
			const auto BinWidth = NestUtils::ToNestCoord(AOptions.BinWidth);
			const auto BinHeight = NestUtils::ToNestCoord(AOptions.BinHeight);
			const auto Spacing = NestUtils::ToNestCoord(std::max(0.0, AOptions.Spacing));
			const auto HalfSpacing = static_cast<libnest2d::Coord>(std::ceil(static_cast<double>(Spacing) * 0.5));
			if (BinWidth <= 0 || BinHeight <= 0) return;

			auto AddCoordinate = [](std::vector<ClipperLib::cInt>& ACoordinates, double AValue,
				double AMaximum) {
				if (AValue < -1.0 || AValue > AMaximum + 1.0) return;
				ACoordinates.push_back(static_cast<ClipperLib::cInt>(std::llround(
					std::clamp(AValue, 0.0, AMaximum))));
			};
			auto CanPlaceTarget = [&](const CetTNestItemVector& AItems, std::size_t ATargetIndex) {
				if (ATargetIndex >= AItems.size()) return false;
				CetNestItem Target = AItems[ATargetIndex];
				Target.inflation(0);
				const auto RawBounds = Target.boundingBox();
				if (getX(RawBounds.minCorner()) < 0 || getY(RawBounds.minCorner()) < 0
					|| getX(RawBounds.maxCorner()) > BinWidth || getY(RawBounds.maxCorner()) > BinHeight) {
					return false;
				}
				Target.inflation(HalfSpacing);
				const auto TargetBounds = Target.boundingBox();
				for (std::size_t Index = 0; Index < AItems.size(); ++Index) {
					if (Index == ATargetIndex || AItems[Index].binId() != Target.binId()) continue;
					CetNestItem Other = AItems[Index];
					Other.inflation(HalfSpacing);
					const auto OtherBounds = Other.boundingBox();
					if (getX(TargetBounds.maxCorner()) < getX(OtherBounds.minCorner())
						|| getX(TargetBounds.minCorner()) > getX(OtherBounds.maxCorner())
						|| getY(TargetBounds.maxCorner()) < getY(OtherBounds.minCorner())
						|| getY(TargetBounds.minCorner()) > getY(OtherBounds.maxCorner())) continue;
					if (CetNestItem::intersects(Target, Other) && !CetNestItem::touches(Target, Other)) return false;
				}
				return true;
			};

			struct TetQuarterTurnTarget
			{
				double Score = 0.0;
				std::size_t Index = 0;
			};
			std::vector<TetQuarterTurnTarget> Targets;
			for (std::size_t Index = 0; Index < ALocalBest.Items.size(); ++Index) {
				if (!ALocalBest.MetaItems[Index].IsCluster || ALocalBest.MetaItems[Index].TransformData.size() < 2) continue;
				const auto Bounds = ALocalBest.Items[Index].boundingBox();
				const double Width = std::abs(static_cast<double>(Bounds.width()));
				const double Height = std::abs(static_cast<double>(Bounds.height()));
				if (Width <= 0.0 || Height <= 0.0) continue;
				const double Aspect = std::max(Width, Height) / std::min(Width, Height);
				if (Aspect <= 1.1) continue;
				const int BinId = ALocalBest.Items[Index].binId();
				const TetLocalCompactEnvelope FullEnvelope = LocalCompactCalculateEnvelope(ALocalBest.Items, BinId);
				if (!FullEnvelope.Valid) continue;
				const double Tolerance = std::max(1.0, static_cast<double>(Spacing));
				const bool OnEnvelope = std::abs(static_cast<double>(getX(Bounds.minCorner())) - FullEnvelope.MinX) <= Tolerance
					|| std::abs(static_cast<double>(getX(Bounds.maxCorner())) - FullEnvelope.MaxX) <= Tolerance
					|| std::abs(static_cast<double>(getY(Bounds.minCorner())) - FullEnvelope.MinY) <= Tolerance
					|| std::abs(static_cast<double>(getY(Bounds.maxCorner())) - FullEnvelope.MaxY) <= Tolerance;
				if (!OnEnvelope) continue;
				Targets.push_back({ Aspect, Index });
			}
			std::stable_sort(Targets.begin(), Targets.end(), [](const TetQuarterTurnTarget& ALeft,
				const TetQuarterTurnTarget& ARight) { return ALeft.Score > ARight.Score; });
			if (Targets.size() > MaxTargets) Targets.resize(MaxTargets);
			std::set<std::size_t> AppliedTargets;

			std::size_t Pass = 0;
			while (AppliedTargets.size() < Targets.size()) {
				if (std::chrono::steady_clock::now() - StartTime >= Budget) break;
				++Pass;
				const CetTNestItemVector PassBaseItems = ALocalBest.Items;
				const std::vector<TetMetaItem> PassBaseMetaItems = ALocalBest.MetaItems;
				const TetTNestEvalResult PassBaseEval = ALocalBest.Eval;
				CetTNestItemVector ExpandedBaseline;
				Nest2DUtils->Nest2DCluster->ExpandClusterResultToOriginalItems(
					AOriginalItems, PassBaseItems, PassBaseMetaItems, ExpandedBaseline, false);
				const TetAllBinRemnantMetric PassBaseRemnant = EvaluateAllBinRemnantMetric(
					ExpandedBaseline, AOptions, PassBaseEval.Layers);

				bool HasPassBest = false;
				std::size_t PassBestTarget = 0;
				std::size_t PassBestLayers = 0;
				TetTNestEvalResult PassBestEval{};
				TetAllBinRemnantMetric PassBestRemnant = PassBaseRemnant;
				CetTNestItemVector PassBestItems;
				std::vector<TetMetaItem> PassBestMetaItems;

				for (const auto& Target : Targets) {
					if (std::chrono::steady_clock::now() - StartTime >= Budget) break;
					const std::size_t TargetIndex = Target.Index;
					if (AppliedTargets.find(TargetIndex) != AppliedTargets.end()) continue;
					const std::string& ClusterType = PassBaseMetaItems[TargetIndex].ClusterType;
					std::cout << "[NEST][QUARTER TURN CANDIDATE] Pass=" << Pass + 1
						<< ", Index=" << TargetIndex << ", Type=" << ClusterType
						<< ", Aspect=" << Target.Score << std::endl;

					CetNestItem RotatedTarget = PassBaseItems[TargetIndex];
					RotatedTarget.inflation(0);
					RotatedTarget.rotation(Radians(
						static_cast<double>(RotatedTarget.rotation()) + CET_CLUSTER_HALF_PI));
					const auto RotatedBounds = RotatedTarget.boundingBox();
					const double RotatedWidth = static_cast<double>(RotatedBounds.width());
					const double RotatedHeight = static_cast<double>(RotatedBounds.height());
					if (RotatedWidth <= 0.0 || RotatedHeight <= 0.0
						|| RotatedWidth > static_cast<double>(BinWidth) || RotatedHeight > static_cast<double>(BinHeight)) continue;

					// Moving a cluster to another already-used sheet may improve an aggregate
					// score while consuming a clean remnant there. Cross-sheet relocation is
					// reserved for the separate sheet-elimination pass.
					const int CandidateBin = PassBaseItems[TargetIndex].binId();
					if (CandidateBin < 0) continue;
					const TetLocalCompactEnvelope CandidateBaseEnvelope = LocalCompactCalculateEnvelope(
						PassBaseItems, CandidateBin);
					std::size_t CheckedPlacements = 0;
					std::size_t ValidPlacements = 0;
						std::vector<ClipperLib::cInt> XCoordinates;
						std::vector<ClipperLib::cInt> YCoordinates;
						const double MaxX = static_cast<double>(BinWidth) - RotatedWidth;
						const double MaxY = static_cast<double>(BinHeight) - RotatedHeight;
						AddCoordinate(XCoordinates, 0.0, MaxX);
						AddCoordinate(XCoordinates, MaxX, MaxX);
						AddCoordinate(YCoordinates, 0.0, MaxY);
						AddCoordinate(YCoordinates, MaxY, MaxY);
						for (std::size_t Index = 0; Index < PassBaseItems.size(); ++Index) {
							if (Index == TargetIndex || PassBaseItems[Index].binId() != CandidateBin) continue;
							CetNestItem Other = PassBaseItems[Index];
							Other.inflation(0);
							const auto Bounds = Other.boundingBox();
							AddCoordinate(XCoordinates, static_cast<double>(getX(Bounds.maxCorner())) + Spacing, MaxX);
							AddCoordinate(XCoordinates, static_cast<double>(getX(Bounds.minCorner())) - Spacing - RotatedWidth, MaxX);
							AddCoordinate(YCoordinates, static_cast<double>(getY(Bounds.maxCorner())) + Spacing, MaxY);
							AddCoordinate(YCoordinates, static_cast<double>(getY(Bounds.minCorner())) - Spacing - RotatedHeight, MaxY);
						}
						std::sort(XCoordinates.begin(), XCoordinates.end());
						XCoordinates.erase(std::unique(XCoordinates.begin(), XCoordinates.end()), XCoordinates.end());
						std::sort(YCoordinates.begin(), YCoordinates.end());
						YCoordinates.erase(std::unique(YCoordinates.begin(), YCoordinates.end()), YCoordinates.end());

						for (ClipperLib::cInt MinY : YCoordinates) {
							for (ClipperLib::cInt MinX : XCoordinates) {
								if (CheckedPlacements >= MaxPlacementsPerTarget) break;
								++CheckedPlacements;
								CetTNestItemVector TestItems = PassBaseItems;
								CetNestItem& TestTarget = TestItems[TargetIndex];
								TestTarget.inflation(0);
								TestTarget.rotation(RotatedTarget.rotation());
								TestTarget.binId(CandidateBin);
								const auto Bounds = TestTarget.boundingBox();
								const Point Translation = TestTarget.translation();
								TestTarget.translation(Point(
									Translation.X + MinX - getX(Bounds.minCorner()),
									Translation.Y + MinY - getY(Bounds.minCorner())));
								if (!CanPlaceTarget(TestItems, TargetIndex)) continue;
								const TetLocalCompactEnvelope TestEnvelope = LocalCompactCalculateEnvelope(TestItems, CandidateBin);
								const double EnvelopeTolerance = std::max(1.0, static_cast<double>(Spacing));
								const bool BetterEnvelope = TestEnvelope.Valid && CandidateBaseEnvelope.Valid
									&& (TestEnvelope.Area + EnvelopeTolerance * EnvelopeTolerance < CandidateBaseEnvelope.Area
										|| (std::abs(TestEnvelope.Area - CandidateBaseEnvelope.Area) <= EnvelopeTolerance * EnvelopeTolerance
											&& TestEnvelope.LongSide + EnvelopeTolerance < CandidateBaseEnvelope.LongSide));
								if (!BetterEnvelope) continue;
								++ValidPlacements;

								CetTNestItemVector ExpandedItems;
								Nest2DUtils->Nest2DCluster->ExpandClusterResultToOriginalItems(
									AOriginalItems, TestItems, PassBaseMetaItems, ExpandedItems, false);
								const TetAllBinRemnantMetric Remnant = EvaluateAllBinRemnantMetric(
									ExpandedItems, AOptions, PassBaseEval.Layers);
								TetExpandedSpacingFailure Failure;
								if (AHasCluster && !Nest2DUtils->Nest2DCluster->ValidatePackedResultSpacing(
									AOriginalItems, TestItems, PassBaseMetaItems, AOptions, &Failure)) continue;

								TetTNestEvalResult Eval = Nest2DUtils->Nest2DStrategy->EvaluatePackedResultWithMeta(
									TestItems, PassBaseMetaItems, AOriginalItems, AOptions, PassBaseEval.Layers);
								EvaluateInternalGapMetrics(ExpandedItems, AOptions, Eval);
								EvaluateBoardFreeRegionMetrics(ExpandedItems, AOptions, Eval);
								EvaluatePassableFreeRegionMetrics(ExpandedItems, AOptions, Eval);
								const TetTNestEvalResult& Comparison = HasPassBest ? PassBestEval : PassBaseEval;
								if (HasPassBest && !IsBetterContinuousFreeSpace(Eval, Comparison)) continue;
								HasPassBest = true;
								PassBestTarget = TargetIndex;
								PassBestLayers = PassBaseEval.Layers;
								PassBestEval = std::move(Eval);
								PassBestRemnant = Remnant;
								PassBestItems = std::move(TestItems);
								PassBestMetaItems = PassBaseMetaItems;
							}
						}
					std::cout << "[NEST][QUARTER TURN LOCAL EVAL] Type=" << ClusterType
						<< ", Bin=" << CandidateBin << ", Checked=" << CheckedPlacements << ", Valid=" << ValidPlacements
						<< ", Improved=" << (HasPassBest && PassBestTarget == TargetIndex ? 1 : 0) << std::endl;
				}

				if (!HasPassBest) break;
				AppliedTargets.insert(PassBestTarget);
				ALocalBest.HasBest = true;
				ALocalBest.Eval = std::move(PassBestEval);
				ALocalBest.Layers = PassBestLayers;
				ALocalBest.Items = std::move(PassBestItems);
				ALocalBest.MetaItems = std::move(PassBestMetaItems);
				ALocalBest.HasCluster = AHasCluster;
				std::cout << "[NEST][QUARTER TURN LOCAL ACCEPT] Pass=" << Pass + 1
					<< ", Index=" << PassBestTarget
					<< ", Type=" << ALocalBest.MetaItems[PassBestTarget].ClusterType
					<< ", ReusableStrip=" << PassBestRemnant.ReusableStripArea
					<< ", SkylineWaste=" << PassBestRemnant.SkylineWasteArea << std::endl;
			}
		}

		void CetNest2DEngine::_TryOppositeEdgeCandidate(TetLocalBestResult& ALocalBest,
			const TetClusterBuildResult& AClusterResult, const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions,
			TetNestProgressTracker& ATracker, bool AHasCluster)
		{
			if (!ALocalBest.HasBest || AClusterResult.NestItems.size() > CET_NEST_FULL_STRATEGY_ITEM_LIMIT
				|| AClusterResult.NestItems.size() != AClusterResult.MetaItems.size()) return;
			const TetTNestEvalResult BaselineEval = ALocalBest.Eval;
			const std::array<MetENestOrderStrategy, 2> Strategies{
				MetENestOrderStrategy::LargeFirst, MetENestOrderStrategy::AreaDensityFirst };
			for (const MetENestOrderStrategy Strategy : Strategies) {
				CetTNestItemVector PriorityItems = AClusterResult.NestItems;
				const std::vector<std::size_t> Order = _BuildPriorityOrder(PriorityItems, AOptions, Strategy);
				CetTNestItemVector TestItems;
				std::vector<TetMetaItem> TestMetaItems;
				_BuildSortedTestData(PriorityItems, AClusterResult.MetaItems, Order, TestItems, TestMetaItems);
				ApplyClusterEdgeClearance(TestItems, TestMetaItems, AOptions);
				const std::size_t Layers = RunRectangleNestFromOppositeEdge(TestItems, AOptions, ATracker);
				ClearItemInflation(TestItems);
				if (Layers == 0) continue;
				TetExpandedSpacingFailure Failure;
				if (AHasCluster && !Nest2DUtils->Nest2DCluster->ValidatePackedResultSpacing(
					AOriginalItems, TestItems, TestMetaItems, AOptions, &Failure)) {
					std::cout << "[NEST][OPPOSITE EDGE EVAL][REJECT] Strategy=" << static_cast<int>(Strategy)
						<< ", reason=expanded cluster spacing violation" << std::endl;
					continue;
				}
				TetTNestEvalResult Eval = Nest2DUtils->Nest2DStrategy->EvaluatePackedResultWithMeta(
					TestItems, TestMetaItems, AOriginalItems, AOptions, Layers);
				CetTNestItemVector ExpandedItems;
				Nest2DUtils->Nest2DCluster->ExpandClusterResultToOriginalItems(
					AOriginalItems, TestItems, TestMetaItems, ExpandedItems, false);
				EvaluateInternalGapMetrics(ExpandedItems, AOptions, Eval);
				EvaluateBoardFreeRegionMetrics(ExpandedItems, AOptions, Eval);
				EvaluatePassableFreeRegionMetrics(ExpandedItems, AOptions, Eval);
				bool PreservesBoardUsage = Eval.BinAreas.size() == BaselineEval.BinAreas.size();
				for (std::size_t Bin = 0; PreservesBoardUsage && Bin < Eval.BinAreas.size(); ++Bin) {
					PreservesBoardUsage = Eval.BinAreas[Bin] + 1.0 >= BaselineEval.BinAreas[Bin];
				}
				const bool PreservesFreeSpace = PreservesPassableFreeSpace(Eval, BaselineEval);
				if (!PreservesBoardUsage || !PreservesFreeSpace) {
					std::cout << "[NEST][OPPOSITE EDGE EVAL][REJECT] Strategy=" << static_cast<int>(Strategy)
						<< ", BoardUsage=" << PreservesBoardUsage << ", PassableFreeSpace=" << PreservesFreeSpace << std::endl;
					continue;
				}
				std::cout << "[NEST][OPPOSITE EDGE EVAL] Strategy=" << static_cast<int>(Strategy)
					<< ", FirstBinArea=" << Eval.FirstBinArea << ", Layers=" << Eval.Layers
					<< ", PassableRegions=" << Eval.PassableFreeRegionCount << std::endl;
				_UpdateLocalBest(ALocalBest, Eval, Layers, TestItems, TestMetaItems, AHasCluster);
			}
		}

		TetLocalBestResult CetNest2DEngine::_EvaluateSingleSortingStrategy(const TetClusterBuildResult& AClusterResult, const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, TetNestProgressTracker& ATracker, MetENestOrderStrategy AStrategy, TetExpandedSpacingFailure* AOutSpacingFailure)
		{
			TetLocalBestResult LocalBest;
			if (AOutSpacingFailure != nullptr) {
				*AOutSpacingFailure = TetExpandedSpacingFailure{};
			}
			if (AClusterResult.NestItems.size() != AClusterResult.MetaItems.size()) {
				return LocalBest;
			}

			const bool UsePolygonBoard = AOptions.Board.Enabled && AOptions.Board.Vertices.size() >= 3;
			const bool HasCluster = _HasClusterItems(AClusterResult.MetaItems);
			CetTNestItemVector PriorityItems = AClusterResult.NestItems;
			const std::vector<std::size_t> SortedIndices = _BuildPriorityOrder(PriorityItems, AOptions, AStrategy);
			CetTNestItemVector TestItems;
			std::vector<TetMetaItem> TestMetaItems;
			_BuildSortedTestData(PriorityItems, AClusterResult.MetaItems, SortedIndices, TestItems, TestMetaItems);
			const std::size_t Layers = UsePolygonBoard ? RunPolygonNestOnce(TestItems, AOptions, ATracker) : RunRectangleNestOnce(TestItems, AOptions, ATracker);
			if (Layers == 0) {
				std::cout << "[NEST][SPACING FALLBACK][SKIP] Strategy=" << static_cast<int>(AStrategy) << ", reason=no packed layers" << std::endl;
				return LocalBest;
			}
			if (HasCluster && !Nest2DUtils->Nest2DCluster->ValidatePackedResultSpacing(AOriginalItems, TestItems, TestMetaItems, AOptions, AOutSpacingFailure)) {
				std::cout << "[NEST][SPACING FALLBACK][SKIP] Strategy=" << static_cast<int>(AStrategy) << ", reason=expanded cluster spacing violation" << std::endl;
				return LocalBest;
			}

			TetTNestEvalResult Eval = Nest2DUtils->Nest2DStrategy->EvaluatePackedResultWithMeta(TestItems, TestMetaItems, AOriginalItems, AOptions, Layers);
			CetTNestItemVector ExpandedItems;
			Nest2DUtils->Nest2DCluster->ExpandClusterResultToOriginalItems(AOriginalItems, TestItems, TestMetaItems, ExpandedItems, false);
			EvaluateInternalGapMetrics(ExpandedItems, AOptions, Eval);
			EvaluateBoardFreeRegionMetrics(ExpandedItems, AOptions, Eval);
			EvaluatePassableFreeRegionMetrics(ExpandedItems, AOptions, Eval);
			_UpdateLocalBest(LocalBest, Eval, Layers, TestItems, TestMetaItems, HasCluster);
			return LocalBest;
		}

		TetLocalBestResult CetNest2DEngine::_TryLocalClusterSpacingFallback(const TetClusterBuildResult& AClusterResult, const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, TetNestProgressTracker& ATracker, const TetExpandedSpacingFailure& AInitialFailure)
		{
			TetLocalBestResult NoResult;
			if (!AInitialFailure.Valid || AClusterResult.NestItems.size() != AClusterResult.MetaItems.size()) {
				return NoResult;
			}

			std::set<int> PackedIndices;
			TetExpandedSpacingFailure Failure = AInitialFailure;
			constexpr int MaxLocalRetries = 2;
			for (int Attempt = 0; Attempt < MaxLocalRetries && Failure.Valid; ++Attempt) {
				const std::size_t ClusterCountBefore = PackedIndices.size();
				for (std::size_t PackedIndex = 0; PackedIndex < AClusterResult.MetaItems.size(); ++PackedIndex) {
					const TetMetaItem& Meta = AClusterResult.MetaItems[PackedIndex];
					if (!Meta.IsCluster) {
						continue;
					}
					for (const TetItemTransform& Transform : Meta.TransformData) {
						if (Transform.OriginalId == Failure.FirstOriginalIndex || Transform.OriginalId == Failure.SecondOriginalIndex) {
							PackedIndices.insert(static_cast<int>(PackedIndex));
							break;
						}
					}
				}
				if (PackedIndices.size() == ClusterCountBefore) {
					std::cout << "[NEST][SPACING FALLBACK][LOCAL][STOP] Attempt=" << Attempt + 1
						<< ", reason=no additional conflicting cluster." << std::endl;
					break;
				}

				std::cout << "[NEST][SPACING FALLBACK][LOCAL] Attempt=" << Attempt + 1
					<< ", OriginalPair=" << Failure.FirstOriginalIndex << "," << Failure.SecondOriginalIndex
					<< ", DissolvedClusters=";
				for (int PackedIndex : PackedIndices) std::cout << PackedIndex << " ";
				std::cout << ", RawOverlap=" << (Failure.RawContoursIntersect ? 1 : 0) << std::endl;

				const TetClusterBuildResult DissolvedResult = DissolvePackedClusters(AOriginalItems, AClusterResult, PackedIndices);
				TetExpandedSpacingFailure RetryFailure;
				TetLocalBestResult LocalResult = _EvaluateSingleSortingStrategy(DissolvedResult, AOriginalItems, AOptions, ATracker, MetENestOrderStrategy::LargeFirst, &RetryFailure);
				if (LocalResult.HasBest) {
					std::cout << "[NEST][SPACING FALLBACK][LOCAL][VALID] Attempt=" << Attempt + 1
						<< ", Layers=" << LocalResult.Layers << ", PackedItems=" << LocalResult.Items.size() << std::endl;
					return LocalResult;
				}
				Failure = RetryFailure;
			}

			std::cout << "[NEST][SPACING FALLBACK][LOCAL][FAILED] Attempts=" << MaxLocalRetries << std::endl;
			return NoResult;
		}

		TetLocalBestResult CetNest2DEngine::EvaluateSortingStrategies(const TetClusterBuildResult& AClusterResult, const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, TetNestProgressTracker& ATracker, TetExpandedSpacingFailure* AOutSpacingFailure)
		{
			TetLocalBestResult LocalBest;
			if (AOutSpacingFailure != nullptr) {
				*AOutSpacingFailure = TetExpandedSpacingFailure{};
			}
			if (AClusterResult.NestItems.size() != AClusterResult.MetaItems.size()){
				std::cout << "[NEST][EVAL][ERROR] Cluster NestItems size != MetaItems size. NestItems = " << AClusterResult.NestItems.size() << ", MetaItems = " << AClusterResult.MetaItems.size() << std::endl;
				return LocalBest;
			}
			const bool UsePolygonBoard = AOptions.Board.Enabled && AOptions.Board.Vertices.size() >= 3;
			const bool CurrentHasCluster = _HasClusterItems(AClusterResult.MetaItems);
			std::vector<MetENestOrderStrategy> Strategies;
			if (AClusterResult.NestItems.size() > CET_NEST_REDUCED_STRATEGY_ITEM_LIMIT){
				Strategies = { MetENestOrderStrategy::LargeFirst, MetENestOrderStrategy::AreaDensityFirst };
			}
			else if (AClusterResult.NestItems.size() > CET_NEST_FULL_STRATEGY_ITEM_LIMIT){
				Strategies = { MetENestOrderStrategy::LargeFirst, MetENestOrderStrategy::AreaDensityFirst, MetENestOrderStrategy::LongSideFirst };
			}
			else {
				Strategies = { MetENestOrderStrategy::LargeFirst, MetENestOrderStrategy::AreaDensityFirst, MetENestOrderStrategy::SmallFirst, MetENestOrderStrategy::LongSideFirst, MetENestOrderStrategy::ThinFirst };
			}
			std::set<std::vector<std::size_t>> EvaluatedOrders;
			for (MetENestOrderStrategy Strategy : Strategies){
				CetTNestItemVector PriorityItems = AClusterResult.NestItems;
				const std::vector<std::size_t> SortedIndices = _BuildPriorityOrder(PriorityItems, AOptions, Strategy);
				if (!EvaluatedOrders.insert(SortedIndices).second){
					std::cout << "[NEST][EVAL][SKIP] Strategy = " << static_cast<int>(Strategy) << ", reason = duplicate item order" << std::endl;
					continue;
				}
				CetTNestItemVector TestItems;
				std::vector<TetMetaItem> TestMetaItems;
				_BuildSortedTestData(PriorityItems, AClusterResult.MetaItems, SortedIndices, TestItems, TestMetaItems);
				const std::size_t Layers = UsePolygonBoard ? RunPolygonNestOnce(TestItems, AOptions, ATracker) : RunRectangleNestOnce(TestItems, AOptions, ATracker);
				if (Layers == 0) {
					std::cout << "[NEST][EVAL][SKIP] Strategy = " << static_cast<int>(Strategy) << ", reason = no packed layers" << std::endl;
					continue;
				}
				TetExpandedSpacingFailure SpacingFailure;
				if (CurrentHasCluster && !Nest2DUtils->Nest2DCluster->ValidatePackedResultSpacing(AOriginalItems, TestItems, TestMetaItems, AOptions, &SpacingFailure)){
					if (AOutSpacingFailure != nullptr && !AOutSpacingFailure->Valid) {
						*AOutSpacingFailure = SpacingFailure;
					}
					std::cout << "[NEST][EVAL][SKIP] Strategy = " << static_cast<int>(Strategy) << ", reason = expanded cluster spacing violation" << std::endl;
					continue;
				}
				TetTNestEvalResult Eval = Nest2DUtils->Nest2DStrategy->EvaluatePackedResultWithMeta(TestItems, TestMetaItems, AOriginalItems, AOptions, Layers);
				CetTNestItemVector ExpandedItems;
				Nest2DUtils->Nest2DCluster->ExpandClusterResultToOriginalItems(AOriginalItems, TestItems, TestMetaItems, ExpandedItems, false);
				EvaluateInternalGapMetrics(ExpandedItems, AOptions, Eval);
				EvaluateBoardFreeRegionMetrics(ExpandedItems, AOptions, Eval);
				EvaluatePassableFreeRegionMetrics(ExpandedItems, AOptions, Eval);
				std::cout << "[NEST][EVAL] Strategy = " << static_cast<int>(Strategy) << ", HasCluster = " << CurrentHasCluster << ", Eval.FirstBinCount = " << Eval.FirstBinCount << ", Eval.FirstBinArea = " << Eval.FirstBinArea << ", Eval.Layers = " << Eval.Layers << ", Eval.InternalGapArea = " << Eval.InternalGapArea << ", Eval.InternalGapCount = " << Eval.InternalGapCount << ", Eval.FreeRegions = " << Eval.BoardFreeRegionCount << ", Eval.FragmentedFreeArea = " << Eval.FragmentedFreeArea << ", Eval.LargestFreeArea = " << Eval.LargestFreeRegionArea << ", Eval.PassableRegions = " << Eval.PassableFreeRegionCount << ", Eval.PassableFragmentedArea = " << Eval.FragmentedPassableFreeArea << ", Eval.LargestPassableArea = " << Eval.LargestPassableFreeRegionArea << ", Eval.PassableWidth = " << Eval.MinimumPassableWidth << ", Eval.RemnantArea = " << Eval.ReusableRemnantArea << ", Eval.RemnantShortSide = " << Eval.ReusableRemnantShortSide << ", Eval.SkylineWaste = " << Eval.SkylineWasteArea << ", Eval.RemnantDirection = " << (Eval.RemnantIsTopStrip ? "Top" : "Right") << ", LocalBest.FirstBinCount = " << LocalBest.Eval.FirstBinCount << ", LocalBest.FirstBinArea = " << LocalBest.Eval.FirstBinArea << ", LocalBest.Layers = " << LocalBest.Eval.Layers << std::endl;
				_UpdateLocalBest(LocalBest, Eval, Layers, TestItems, TestMetaItems, CurrentHasCluster);
				if (AOriginalItems.size() > CET_NEST_FULL_STRATEGY_ITEM_LIMIT && LocalBest.HasBest && LocalBest.Layers == 1){
					// One sheet is already the minimum possible. On large orders a
					// second full NFP pass can cost minutes for only a secondary
					// remnant-shape comparison; the clustered strategy is still
					// evaluated separately and can replace this result.
					std::cout << "[NEST][EVAL][SKIP REMAINING] OriginalCount=" << AOriginalItems.size() << ", PackedCount=" << AClusterResult.NestItems.size() << ", reason=one-sheet optimum at large-order limit" << std::endl;
					break;
				}
			}
			if (!UsePolygonBoard) {
				if (AOptions.EnableLocalCompactPass && CurrentHasCluster) {
					_TryQuarterTurnCandidates(LocalBest, AOriginalItems, AOptions, ATracker, CurrentHasCluster);
				}
				_TryOppositeEdgeCandidate(LocalBest, AClusterResult, AOriginalItems, AOptions, ATracker, CurrentHasCluster);
			}
			return LocalBest;
		}
		bool CetNest2DEngine::ShoouldUpdateGlobalBest(const TetLocalBestResult& ALocalResult, bool AHasBest, const TetTNestEvalResult& ABestEval, std::size_t ABestLayers, bool ABestHasCluster)
		{
			if (!ALocalResult.HasBest){
				return false;
			}

			
			if (!AHasBest){
				return true;
			}

			
			if (Nest2DUtils->Nest2DStrategy->IsBetterNestResult(ALocalResult.Eval, ABestEval)){
				return true;
			}

			
			const bool Equivalent = !Nest2DUtils->Nest2DStrategy->IsBetterNestResult(ABestEval,ALocalResult.Eval);

			if (Equivalent){
				
				if (ALocalResult.HasCluster && !ABestHasCluster){
					return true;
				}
			}

			return false;
		}

	}
}
