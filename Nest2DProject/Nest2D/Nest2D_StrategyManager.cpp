#include "pch.h"
#include "Nest2D_StrategyManager.h"
//#include"Nest2D_PrivateDataType.h"
#include "Nest2D_RotationUtils.h"
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

	struct TetAreaDensityMetric
	{
		double Area = 0.0;
		double Density = 0.0;
		double LongSide = 0.0;
		int AreaBand = 0;
	};

	TetAreaDensityMetric BuildAreaDensityMetric(const CetNestItem& AItem, const TetNestOptions& AOptions)
	{
		TetAreaDensityMetric Result;
		Result.Area = std::abs(static_cast<double>(AItem.area()));
		if (Result.Area <= 0.0) return Result;

		const double Spacing = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));
		double BestEnvelopeArea = std::numeric_limits<double>::infinity();
		for (const Radians Rotation : ET::NEST2DMANAGERLIB::CetRotationUtils::BuildAllowedLibRotations(AOptions.Rotations)) {
			CetNestItem RotatedItem = AItem;
			RotatedItem.rotation(Rotation);
			RotatedItem.inflation(0);
			const auto Bounds = RotatedItem.boundingBox();
			const double Width = std::max(0.0, static_cast<double>(Bounds.width()) + Spacing * 2.0);
			const double Height = std::max(0.0, static_cast<double>(Bounds.height()) + Spacing * 2.0);
			const double EnvelopeArea = Width * Height;
			if (EnvelopeArea < BestEnvelopeArea) {
				BestEnvelopeArea = EnvelopeArea;
				Result.LongSide = std::max(Width, Height);
			}
		}
		Result.Density = Result.Area / std::max(1.0, BestEnvelopeArea);
		Result.AreaBand = static_cast<int>(std::floor(std::log2(std::max(1.0, Result.Area))));
		return Result;
	}

	bool IsPriorityBefore(const CetTNestItemVector& AItems, MetENestOrderStrategy AStrategy, const std::vector<int>& AAnchorRanks, const std::vector<TetAreaDensityMetric>* AAreaDensityMetrics, std::size_t A, std::size_t AB)
	{
		const bool AIsAnchor = AAnchorRanks[A] >= 0;
		const bool BIsAnchor = AAnchorRanks[AB] >= 0;
		if (AIsAnchor != BIsAnchor) return AIsAnchor;
		if (AIsAnchor) return AAnchorRanks[A] < AAnchorRanks[AB];
		if (AStrategy == MetENestOrderStrategy::AreaDensityFirst && AAreaDensityMetrics != nullptr) {
			const TetAreaDensityMetric& MetricA = (*AAreaDensityMetrics)[A];
			const TetAreaDensityMetric& MetricB = (*AAreaDensityMetrics)[AB];
			if (MetricA.AreaBand != MetricB.AreaBand) return MetricA.AreaBand > MetricB.AreaBand;
			if (AreMetricValuesDifferent(MetricA.Density, MetricB.Density)) return MetricA.Density > MetricB.Density;
			if (AreMetricValuesDifferent(MetricA.LongSide, MetricB.LongSide)) return MetricA.LongSide > MetricB.LongSide;
			if (AreMetricValuesDifferent(MetricA.Area, MetricB.Area)) return MetricA.Area > MetricB.Area;
			return A < AB;
		}

		const auto Width = [&](std::size_t Index) { return static_cast<double>(AItems[Index].boundingBox().width()); };
		const auto Height = [&](std::size_t Index) { return static_cast<double>(AItems[Index].boundingBox().height()); };
		const double AreaA = std::abs(static_cast<double>(AItems[A].area()));
		const double AreaB = std::abs(static_cast<double>(AItems[AB].area()));
		double MetricA = AreaA;
		double MetricB = AreaB;
		if (AStrategy == MetENestOrderStrategy::LongSideFirst){
			MetricA = std::max(Width(A), Height(A));
			MetricB = std::max(Width(AB), Height(AB));
		}
		else if (AStrategy == MetENestOrderStrategy::ThinFirst){
			MetricA = std::max(Width(A), Height(A)) / std::max(1.0, std::min(Width(A), Height(A)));
			MetricB = std::max(Width(AB), Height(AB)) / std::max(1.0, std::min(Width(AB), Height(AB)));
		}
		if (AreMetricValuesDifferent(MetricA, MetricB)){
			return AStrategy == MetENestOrderStrategy::SmallFirst ? MetricA < MetricB : MetricA > MetricB;
		}
		return A < AB;
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

void ET::NEST2DMANAGERLIB::CetStrategyManager::ApplyNestPriorityStrategy(CetTNestItemVector& AItems, const TetNestOptions& AOptions, MetENestOrderStrategy AStrategy)
{
	std::vector<std::size_t> Indices;
	Indices.reserve(AItems.size());
	for (std::size_t i = 0; i < AItems.size(); ++i){
		Indices.push_back(i);
	}
	auto GetArea = [&](std::size_t AIndex) -> double{
			return std::abs(static_cast<double>(AItems[AIndex].area()));
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
	std::vector<TetAreaDensityMetric> AreaDensityMetrics;
	if (AStrategy == MetENestOrderStrategy::AreaDensityFirst) {
		AreaDensityMetrics.reserve(AItems.size());
		for (const CetNestItem& Item : AItems) {
			AreaDensityMetrics.push_back(BuildAreaDensityMetric(Item, AOptions));
		}
	}

	std::stable_sort(Indices.begin(), Indices.end(), [&](std::size_t A, std::size_t AB) {
		const auto* Metrics = AreaDensityMetrics.empty() ? nullptr : &AreaDensityMetrics;
		return IsPriorityBefore(AItems, AStrategy, AnchorRanks, Metrics, A, AB);
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
	if (AItems.size() != AMetaItems.size()) {
		std::cout << "[NEST][EVAL][ERROR] PackedItems size != MetaItems size. PackedItems = " << AItems.size() << ", MetaItems = " << AMetaItems.size() << std::endl;
		return Result;
	}

	int LastBinId = -1;
	for (const CetNestItem& PackedItem : AItems) {
		LastBinId = std::max(LastBinId, PackedItem.binId());
	}

	// 1. 提取包围盒并统计基础面积数据
	std::vector<TetRemnantPartBounds> LastBinBounds = _ExtractLastBinBounds(AItems, AMetaItems, AOriginalItems, LastBinId, Result);

	// 2. 计算天际线与余料评估指标
	_CalculateRemnantMetrics(LastBinBounds, AOptions, Result);

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

std::vector<TetRemnantPartBounds> ET::NEST2DMANAGERLIB::CetStrategyManager::_ExtractLastBinBounds(const CetTNestItemVector& AItems, const std::vector<TetMetaItem>& AMetaItems, const CetTNestItemVector& AOriginalItems, int ALastBinId, TetTNestEvalResult& AOutResult) const
{
	std::vector<TetRemnantPartBounds> LastBinBounds;
	LastBinBounds.reserve(AOriginalItems.size());

	for (std::size_t PackedIndex = 0; PackedIndex < AItems.size(); ++PackedIndex) {
		const auto& PackedItem = AItems[PackedIndex];
		const auto& Meta = AMetaItems[PackedIndex];
		int PackedBinId = PackedItem.binId();

		if (PackedBinId < 0) {
			continue;
		}

		const auto PackedTranslation = PackedItem.translation();
		const double PackedX = static_cast<double>(PackedTranslation.X);
		const double PackedY = static_cast<double>(PackedTranslation.Y);
		const double PackedRotation = PackedItem.rotation();
		const double CosRotation = std::cos(PackedRotation);
		const double SinRotation = std::sin(PackedRotation);

		for (const auto& Transform : Meta.TransformData) {
			int OriginalId = Transform.OriginalId;
			if (OriginalId < 0 || OriginalId >= static_cast<int>(AOriginalItems.size())) {
				std::cout << "[NEST][EVAL][WARN] Invalid OriginalId = " << OriginalId << ", OriginalItems.size = " << AOriginalItems.size() << std::endl;
				continue;
			}

			double OriginalArea = std::abs(static_cast<double>(AOriginalItems[OriginalId].area()));
			if (PackedBinId == 0) {
				AOutResult.FirstBinCount++;
				AOutResult.FirstBinArea += OriginalArea;
			}
			if (PackedBinId != ALastBinId) {
				continue;
			}

			AOutResult.LastBinCount++;
			AOutResult.LastBinArea += OriginalArea;

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

			if (PartBounds.MaxX > PartBounds.MinX && PartBounds.MaxY > PartBounds.MinY) {
				LastBinBounds.push_back(PartBounds);
			}
		}
	}
	return LastBinBounds;
}
void ET::NEST2DMANAGERLIB::CetStrategyManager::_CalculateRemnantMetrics(const std::vector<TetRemnantPartBounds>& ALastBinBounds, const TetNestOptions& AOptions, TetTNestEvalResult& AOutResult) const
{
    const bool UseRectangleBoard = !AOptions.Board.Enabled || AOptions.Board.Vertices.size() < 3;
    const double BinWidth = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth));
    const double BinHeight = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinHeight));

    if (!UseRectangleBoard || BinWidth <= 0.0 || BinHeight <= 0.0 || ALastBinBounds.empty()){
        return;
    }

    std::array<double, CET_REMNANT_SKYLINE_SAMPLES> HorizontalSkyline{};
    std::array<double, CET_REMNANT_SKYLINE_SAMPLES> VerticalSkyline{};
    double UsedMaxX = 0.0;
    double UsedMaxY = 0.0;

    for (const TetRemnantPartBounds& Bounds : ALastBinBounds){
        const double MinX = std::clamp(Bounds.MinX, 0.0, BinWidth);
        const double MaxX = std::clamp(Bounds.MaxX, 0.0, BinWidth);
        const double MinY = std::clamp(Bounds.MinY, 0.0, BinHeight);
        const double MaxY = std::clamp(Bounds.MaxY, 0.0, BinHeight);
        UsedMaxX = std::max(UsedMaxX, MaxX);
        UsedMaxY = std::max(UsedMaxY, MaxY);

        const std::size_t StartX = std::min(CET_REMNANT_SKYLINE_SAMPLES - 1, static_cast<std::size_t>(std::floor(MinX / BinWidth * CET_REMNANT_SKYLINE_SAMPLES)));
        const std::size_t EndX = std::min(CET_REMNANT_SKYLINE_SAMPLES, static_cast<std::size_t>(std::ceil(MaxX / BinWidth * CET_REMNANT_SKYLINE_SAMPLES)));
        for (std::size_t Sample = StartX; Sample < EndX; ++Sample){
            HorizontalSkyline[Sample] = std::max(HorizontalSkyline[Sample], MaxY);
        }

        const std::size_t StartY = std::min(CET_REMNANT_SKYLINE_SAMPLES - 1, static_cast<std::size_t>(std::floor(MinY / BinHeight * CET_REMNANT_SKYLINE_SAMPLES)));
        const std::size_t EndY = std::min(CET_REMNANT_SKYLINE_SAMPLES, static_cast<std::size_t>(std::ceil(MaxY / BinHeight * CET_REMNANT_SKYLINE_SAMPLES)));
        for (std::size_t Sample = StartY; Sample < EndY; ++Sample){
            VerticalSkyline[Sample] = std::max(VerticalSkyline[Sample], MaxX);
        }
    }

    const double HorizontalStep = BinWidth / static_cast<double>(CET_REMNANT_SKYLINE_SAMPLES);
    const double VerticalStep = BinHeight / static_cast<double>(CET_REMNANT_SKYLINE_SAMPLES);
    const std::size_t UsedHorizontalSamples = std::min(CET_REMNANT_SKYLINE_SAMPLES, static_cast<std::size_t>(std::ceil(UsedMaxX / BinWidth * CET_REMNANT_SKYLINE_SAMPLES)));
    const std::size_t UsedVerticalSamples = std::min(CET_REMNANT_SKYLINE_SAMPLES, static_cast<std::size_t>(std::ceil(UsedMaxY / BinHeight * CET_REMNANT_SKYLINE_SAMPLES)));

    double TopSkylineWaste = 0.0;
    for (std::size_t Sample = 0; Sample < UsedHorizontalSamples; ++Sample){
        TopSkylineWaste += std::max(0.0, UsedMaxY - HorizontalSkyline[Sample]) * HorizontalStep;
    }

    double RightSkylineWaste = 0.0;
    for (std::size_t Sample = 0; Sample < UsedVerticalSamples; ++Sample){
        RightSkylineWaste += std::max(0.0, UsedMaxX - VerticalSkyline[Sample]) * VerticalStep;
    }

    const double TopFreeDepth = std::max(0.0, BinHeight - UsedMaxY);
    const double RightFreeDepth = std::max(0.0, BinWidth - UsedMaxX);
    const double TopRemnantArea = BinWidth * TopFreeDepth;
    const double RightRemnantArea = BinHeight * RightFreeDepth;
    const double TopShortSide = std::min(BinWidth, TopFreeDepth);
    const double RightShortSide = std::min(BinHeight, RightFreeDepth);

    const bool PreferTop = AreMetricValuesDifferent(TopRemnantArea, RightRemnantArea)
        ? TopRemnantArea > RightRemnantArea
        : (AreMetricValuesDifferent(TopShortSide, RightShortSide) ? TopShortSide > RightShortSide : TopSkylineWaste <= RightSkylineWaste);

    AOutResult.HasRemnantMetrics = true;
    AOutResult.RemnantIsTopStrip = PreferTop;
    AOutResult.ReusableRemnantArea = PreferTop ? TopRemnantArea : RightRemnantArea;
    AOutResult.ReusableRemnantShortSide = PreferTop ? TopShortSide : RightShortSide;
    AOutResult.SkylineWasteArea = PreferTop ? TopSkylineWaste : RightSkylineWaste;
    AOutResult.UsedDepth = PreferTop ? UsedMaxY : UsedMaxX;
}
