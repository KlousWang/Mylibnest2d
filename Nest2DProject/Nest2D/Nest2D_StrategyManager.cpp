#include "pch.h"
#include "Nest2D_StrategyManager.h"
//#include"Nest2D_PrivateDataType.h"
#include "NestUtils.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>

using namespace libnest2d;

namespace {
	/*constexpr std::size_t CET_REMNANT_SKYLINE_SAMPLES = 64;
	constexpr double CET_LARGE_ANCHOR_RATIO = 0.15;
	constexpr std::size_t CET_LARGE_ANCHOR_MAX_COUNT = 8;

	struct TetRemnantPartBounds
	{
		double MinX = 0.0;
		double MinY = 0.0;
		double MaxX = 0.0;
		double MaxY = 0.0;
	};*/

	bool AreMetricValuesDifferent(double AFirst, double ASecond)
	{
		const double Scale = std::max({ 1.0, std::abs(AFirst), std::abs(ASecond) });
		return std::abs(AFirst - ASecond) > Scale * 1e-9;
	}
}

ET::NEST2DMANAGERLIB::CetStrategyManager::CetStrategyManager() :CetCoreObject()
{
}

ET::NEST2DMANAGERLIB::CetStrategyManager::~CetStrategyManager()
{
}

TetTNestEvalResult ET::NEST2DMANAGERLIB::CetStrategyManager::EvaluateNestResult(const CetTNestItemVector& AItems, std::size_t ALayers)
{
	TetTNestEvalResult Result{};
	Result.Layers = ALayers;
	for (const auto& Item : AItems){
		if (Item.binId() == 0){
			Result.FirstBinCount++;
			Result.FirstBinArea += std::abs( static_cast<double>(Item.area()));
		}
	}
	return Result;
}

bool ET::NEST2DMANAGERLIB::CetStrategyManager::IsBetterNestResult(const TetTNestEvalResult& A, const TetTNestEvalResult& AB)
{
	if (A.Layers != AB.Layers){
		return A.Layers < AB.Layers;
	}

	if (A.HasRemnantMetrics && AB.HasRemnantMetrics){
		// Fill completed sheets first. With a fixed total part area this also
		// minimizes the material consumed on the final sheet.
		if (AreMetricValuesDifferent(A.FirstBinArea,AB.FirstBinArea)){
			return A.FirstBinArea > AB.FirstBinArea;
		}
		if (A.FirstBinCount != AB.FirstBinCount){
			return A.FirstBinCount > AB.FirstBinCount;
		}
		if (AreMetricValuesDifferent(A.ReusableRemnantArea,AB.ReusableRemnantArea)){
			return A.ReusableRemnantArea > AB.ReusableRemnantArea;
		}
		if (AreMetricValuesDifferent(A.ReusableRemnantShortSide,AB.ReusableRemnantShortSide)){
			return A.ReusableRemnantShortSide > AB.ReusableRemnantShortSide;
		}
		if (AreMetricValuesDifferent(A.SkylineWasteArea,AB.SkylineWasteArea)){
			return A.SkylineWasteArea < AB.SkylineWasteArea;
		}
		if (AreMetricValuesDifferent(A.UsedDepth,AB.UsedDepth)){
			return A.UsedDepth < AB.UsedDepth;
		}
	}

	if (AreMetricValuesDifferent(A.FirstBinArea,AB.FirstBinArea)){
		return A.FirstBinArea > AB.FirstBinArea;
	}

	if (A.FirstBinCount != AB.FirstBinCount){
		return A.FirstBinCount > AB.FirstBinCount;
	}
	return false;
}

void ET::NEST2DMANAGERLIB::CetStrategyManager::ApplyNestPriorityStrategy(CetTNestItemVector& AItems, MetENestOrderStrategy AStrategy)
{
	std::vector<std::size_t> Indices;
	Indices.reserve(AItems.size());
	for (std::size_t i = 0; i < AItems.size(); ++i){
		Indices.push_back(i);
	}
	auto GetArea = [&](std::size_t AIndex) -> double{
			return std::abs(static_cast<double>(AItems[AIndex].area()));
		};
	auto GetBox = [&](std::size_t AIndex){
			return AItems[AIndex].boundingBox();
		};
	auto GetWidth = [&](std::size_t AIndex) -> double{
		return static_cast<double>(GetBox(AIndex).width());
		};
	auto GetHeight = [&](std::size_t AIndex) -> double{
			return static_cast<double>(GetBox(AIndex).height());
		};
	std::vector<std::size_t> AreaOrder = Indices;
	std::stable_sort(AreaOrder.begin(),AreaOrder.end(),[&](std::size_t A, std::size_t AB){
		const double AreaA = GetArea(A);
		const double AreaB = GetArea(AB);
		if (AreMetricValuesDifferent(AreaA,AreaB)){
			return AreaA > AreaB;
		}
		return A < AB;
		});
	const std::size_t AnchorCount = AItems.empty()
		? 0
		: std::min(CET_LARGE_ANCHOR_MAX_COUNT,std::max<std::size_t>(1,static_cast<std::size_t>(std::ceil(static_cast<double>(AItems.size()) * CET_LARGE_ANCHOR_RATIO))));
	std::vector<int> AnchorRanks(AItems.size(),-1);
	for (std::size_t Rank = 0; Rank < AnchorCount; ++Rank){
		AnchorRanks[AreaOrder[Rank]] = static_cast<int>(Rank);
	}

	std::stable_sort(Indices.begin(),Indices.end(),[&](std::size_t A, std::size_t AB){
		const bool AIsAnchor = AnchorRanks[A] >= 0;
		const bool BIsAnchor = AnchorRanks[AB] >= 0;
		if (AIsAnchor != BIsAnchor){
			return AIsAnchor;
		}
		if (AIsAnchor){
			return AnchorRanks[A] < AnchorRanks[AB];
		}

		bool Before = false;
			switch (AStrategy){
			case MetENestOrderStrategy::LargeFirst:
				Before = GetArea(A) > GetArea(AB);
				break;
			case MetENestOrderStrategy::SmallFirst:
				Before = GetArea(A) < GetArea(AB);
				break;
			case MetENestOrderStrategy::LongSideFirst:{
				double LongA = std::max(GetWidth(A), GetHeight(A));
				double LongB = std::max(GetWidth(AB), GetHeight(AB));
				Before = LongA > LongB;
				break;
			}
			case MetENestOrderStrategy::ThinFirst:{
				double WA = GetWidth(A);
				double HA = GetHeight(A);
				double WB = GetWidth(AB);
				double HB = GetHeight(AB);
				double ShortA = std::max(1.0, std::min(WA, HA));
				double ShortB = std::max(1.0, std::min(WB, HB));
				double RatioA = std::max(WA, HA) / ShortA;
				double RatioB = std::max(WB, HB) / ShortB;
				Before = RatioA > RatioB;
				break;
			}
			default:
				Before = GetArea(A) > GetArea(AB);
				break;
			}
		if (Before){
			return true;
		}
		// Resolve metric ties deterministically without changing the selected strategy.
		switch (AStrategy){
		case MetENestOrderStrategy::SmallFirst:
			if (AreMetricValuesDifferent(GetArea(A),GetArea(AB))) return false;
			break;
		case MetENestOrderStrategy::LongSideFirst:
			if (AreMetricValuesDifferent(std::max(GetWidth(A),GetHeight(A)),std::max(GetWidth(AB),GetHeight(AB)))) return false;
			break;
		case MetENestOrderStrategy::ThinFirst:{
			const double RatioA = std::max(GetWidth(A),GetHeight(A)) / std::max(1.0,std::min(GetWidth(A),GetHeight(A)));
			const double RatioB = std::max(GetWidth(AB),GetHeight(AB)) / std::max(1.0,std::min(GetWidth(AB),GetHeight(AB)));
			if (AreMetricValuesDifferent(RatioA,RatioB)) return false;
			break;
		}
		default:
			if (AreMetricValuesDifferent(GetArea(A),GetArea(AB))) return false;
			break;
		}
		return A < AB;
		});

	int Priority = static_cast<int>(AItems.size());

	for (std::size_t Index : Indices){
		AItems[Index].priority(Priority--);
	}
	std::cout << "[NEST][ORDER] Strategy=" << static_cast<int>(AStrategy) << ", AnchorCount=" << AnchorCount << ", PackedCount=" << AItems.size() << std::endl;
}

void ET::NEST2DMANAGERLIB::CetStrategyManager::PrintBinCount(const CetTNestItemVector& AItems)
{
	std::map<int, int> BinCount;
	for (const auto& Item : AItems){
		BinCount[Item.binId()]++;
	}
	for (const auto& Pair : BinCount){
		std::cout << "[NEST] binId = " << Pair.first << ", count = " << Pair.second << std::endl;
	}
}

TetTNestEvalResult ET::NEST2DMANAGERLIB::CetStrategyManager::EvaluatePackedResultWithMeta(const CetTNestItemVector& AItems, const std::vector<TetMetaItem>& AMetaItems, const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, std::size_t ALayers)
{
	TetTNestEvalResult Result{};
	Result.Layers = ALayers;
	if(AItems.size() != AMetaItems.size()){
		std::cout << "[NEST][EVAL][ERROR] PackedItems size != MetaItems size. " << "PackedItems = " << AItems.size() << ", MetaItems = " << AMetaItems.size() << std::endl;
		return Result;
	}

	int LastBinId = -1;
	for (const CetNestItem& PackedItem : AItems){
		LastBinId = std::max(LastBinId,PackedItem.binId());
	}

	std::vector<TetRemnantPartBounds> LastBinBounds;
	LastBinBounds.reserve(AOriginalItems.size());
	for(std::size_t PackedIndex = 0; PackedIndex < AItems.size(); ++PackedIndex){
		const auto& PackedItem = AItems[PackedIndex];
		const auto& Meta = AMetaItems[PackedIndex];
		int PackedBinId = PackedItem.binId();
		if (PackedBinId < 0){
			continue;
		}
		const auto PackedTranslation = PackedItem.translation();
		const double PackedX = static_cast<double>(PackedTranslation.X);
		const double PackedY = static_cast<double>(PackedTranslation.Y);
		const double PackedRotation = PackedItem.rotation();
		const double CosRotation = std::cos(PackedRotation);
		const double SinRotation = std::sin(PackedRotation);

		for(const auto& Transform : Meta.TransformData){
			int OriginalId = Transform.OriginalId;
			if(OriginalId < 0 || OriginalId >= static_cast<int>(AOriginalItems.size())){
				std::cout << "[NEST][EVAL][WARN] Invalid OriginalId = " << OriginalId << ", OriginalItems.size = " << AOriginalItems.size() << std::endl;
				continue;
			}
			double OriginalArea = std::abs(static_cast<double>(AOriginalItems[OriginalId].area()));
			if (PackedBinId == 0){
				Result.FirstBinCount++;
				Result.FirstBinArea += OriginalArea;
			}
			if (PackedBinId != LastBinId){
				continue;
			}

			Result.LastBinCount++;
			Result.LastBinArea += OriginalArea;
			const double RotatedLocalX = Transform.RelativeX * CosRotation - Transform.RelativeY * SinRotation;
			const double RotatedLocalY = Transform.RelativeX * SinRotation + Transform.RelativeY * CosRotation;
			CetNestItem ExpandedItem = AOriginalItems[OriginalId];
			ExpandedItem.binId(PackedBinId);
			ExpandedItem.translation(ClipperLib::IntPoint(
				static_cast<ClipperLib::cInt>(std::llround(PackedX + RotatedLocalX)),
				static_cast<ClipperLib::cInt>(std::llround(PackedY + RotatedLocalY))));
			ExpandedItem.rotation(PackedRotation + Transform.RelativeRotation);
			ExpandedItem.inflation(0);
			const auto Bounds = ExpandedItem.boundingBox();
			TetRemnantPartBounds PartBounds;
			PartBounds.MinX = static_cast<double>(getX(Bounds.minCorner()));
			PartBounds.MinY = static_cast<double>(getY(Bounds.minCorner()));
			PartBounds.MaxX = static_cast<double>(getX(Bounds.maxCorner()));
			PartBounds.MaxY = static_cast<double>(getY(Bounds.maxCorner()));
			if (PartBounds.MaxX > PartBounds.MinX && PartBounds.MaxY > PartBounds.MinY){
				LastBinBounds.push_back(PartBounds);
			}
		}
	}

	const bool UseRectangleBoard = !AOptions.Board.Enabled || AOptions.Board.Vertices.size() < 3;
	const double BinWidth = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth));
	const double BinHeight = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinHeight));
	if (UseRectangleBoard && BinWidth > 0.0 && BinHeight > 0.0 && !LastBinBounds.empty()){
		std::array<double,CET_REMNANT_SKYLINE_SAMPLES> HorizontalSkyline{};
		std::array<double,CET_REMNANT_SKYLINE_SAMPLES> VerticalSkyline{};
		double UsedMaxX = 0.0;
		double UsedMaxY = 0.0;
		for (const TetRemnantPartBounds& Bounds : LastBinBounds){
			const double MinX = std::clamp(Bounds.MinX,0.0,BinWidth);
			const double MaxX = std::clamp(Bounds.MaxX,0.0,BinWidth);
			const double MinY = std::clamp(Bounds.MinY,0.0,BinHeight);
			const double MaxY = std::clamp(Bounds.MaxY,0.0,BinHeight);
			UsedMaxX = std::max(UsedMaxX,MaxX);
			UsedMaxY = std::max(UsedMaxY,MaxY);

			const std::size_t StartX = std::min(CET_REMNANT_SKYLINE_SAMPLES - 1,static_cast<std::size_t>(std::floor(MinX / BinWidth * CET_REMNANT_SKYLINE_SAMPLES)));
			const std::size_t EndX = std::min(CET_REMNANT_SKYLINE_SAMPLES,static_cast<std::size_t>(std::ceil(MaxX / BinWidth * CET_REMNANT_SKYLINE_SAMPLES)));
			for (std::size_t Sample = StartX; Sample < EndX; ++Sample){
				HorizontalSkyline[Sample] = std::max(HorizontalSkyline[Sample],MaxY);
			}

			const std::size_t StartY = std::min(CET_REMNANT_SKYLINE_SAMPLES - 1,static_cast<std::size_t>(std::floor(MinY / BinHeight * CET_REMNANT_SKYLINE_SAMPLES)));
			const std::size_t EndY = std::min(CET_REMNANT_SKYLINE_SAMPLES,static_cast<std::size_t>(std::ceil(MaxY / BinHeight * CET_REMNANT_SKYLINE_SAMPLES)));
			for (std::size_t Sample = StartY; Sample < EndY; ++Sample){
				VerticalSkyline[Sample] = std::max(VerticalSkyline[Sample],MaxX);
			}
		}

		const double HorizontalStep = BinWidth / static_cast<double>(CET_REMNANT_SKYLINE_SAMPLES);
		const double VerticalStep = BinHeight / static_cast<double>(CET_REMNANT_SKYLINE_SAMPLES);
		const std::size_t UsedHorizontalSamples = std::min(CET_REMNANT_SKYLINE_SAMPLES,static_cast<std::size_t>(std::ceil(UsedMaxX / BinWidth * CET_REMNANT_SKYLINE_SAMPLES)));
		const std::size_t UsedVerticalSamples = std::min(CET_REMNANT_SKYLINE_SAMPLES,static_cast<std::size_t>(std::ceil(UsedMaxY / BinHeight * CET_REMNANT_SKYLINE_SAMPLES)));
		double TopSkylineWaste = 0.0;
		for (std::size_t Sample = 0; Sample < UsedHorizontalSamples; ++Sample){
			TopSkylineWaste += std::max(0.0,UsedMaxY - HorizontalSkyline[Sample]) * HorizontalStep;
		}
		double RightSkylineWaste = 0.0;
		for (std::size_t Sample = 0; Sample < UsedVerticalSamples; ++Sample){
			RightSkylineWaste += std::max(0.0,UsedMaxX - VerticalSkyline[Sample]) * VerticalStep;
		}

		const double TopFreeDepth = std::max(0.0,BinHeight - UsedMaxY);
		const double RightFreeDepth = std::max(0.0,BinWidth - UsedMaxX);
		const double TopRemnantArea = BinWidth * TopFreeDepth;
		const double RightRemnantArea = BinHeight * RightFreeDepth;
		const double TopShortSide = std::min(BinWidth,TopFreeDepth);
		const double RightShortSide = std::min(BinHeight,RightFreeDepth);
		const bool PreferTop = AreMetricValuesDifferent(TopRemnantArea,RightRemnantArea)
			? TopRemnantArea > RightRemnantArea
			: (AreMetricValuesDifferent(TopShortSide,RightShortSide) ? TopShortSide > RightShortSide : TopSkylineWaste <= RightSkylineWaste);

		Result.HasRemnantMetrics = true;
		Result.RemnantIsTopStrip = PreferTop;
		Result.ReusableRemnantArea = PreferTop ? TopRemnantArea : RightRemnantArea;
		Result.ReusableRemnantShortSide = PreferTop ? TopShortSide : RightShortSide;
		Result.SkylineWasteArea = PreferTop ? TopSkylineWaste : RightSkylineWaste;
		Result.UsedDepth = PreferTop ? UsedMaxY : UsedMaxX;
	}

	std::cout << "[NEST][EVAL][RETURN] Count=" << Result.FirstBinCount
		<< ", Area=" << Result.FirstBinArea
		<< ", Layers=" << Result.Layers
		<< ", LastBinCount=" << Result.LastBinCount
		<< ", RemnantArea=" << Result.ReusableRemnantArea
		<< ", RemnantShortSide=" << Result.ReusableRemnantShortSide
		<< ", SkylineWaste=" << Result.SkylineWasteArea
		<< ", UsedDepth=" << Result.UsedDepth
		<< ", RemnantDirection=" << (Result.RemnantIsTopStrip ? "Top" : "Right")
		<< std::endl;
	return Result;
}
