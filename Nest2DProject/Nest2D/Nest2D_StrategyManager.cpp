#include "pch.h"
#include "Nest2D_StrategyManager.h"

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
	std::cout << "[NEST][COMPARE FUNC] " << "A.Count = " << A.FirstBinCount << ", A.Area = " << A.FirstBinArea << ", A.Layers = " << A.Layers << ", B.Count = " << AB.FirstBinCount
		<< ", B.Area = " << AB.FirstBinArea<< ", B.Layers = " << AB.Layers<< std::endl;
	
	if (A.Layers != AB.Layers){
		bool Result = A.Layers < AB.Layers;
		std::cout << "[NEST][COMPARE FUNC] compare layers, result = " << Result << std::endl;
		return Result;
	}
	
	if (std::abs(A.FirstBinArea - AB.FirstBinArea) > 1e-6){
		bool Result = A.FirstBinArea > AB.FirstBinArea;
		std::cout << "[NEST][COMPARE FUNC] compare area, result = " << Result << std::endl;
		return Result;
	}
	
	if (A.FirstBinCount != AB.FirstBinCount){
		bool Result = A.FirstBinCount > AB.FirstBinCount;
		std::cout << "[NEST][COMPARE FUNC] compare count, result = " << Result << std::endl;
		return Result;
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
			return static_cast<double>(AItems[AIndex].area());
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
	std::sort(Indices.begin(),Indices.end(),[&](std::size_t A, std::size_t AB){
			switch (AStrategy){
			case MetENestOrderStrategy::LargeFirst:
				return GetArea(A) > GetArea(AB);
			case MetENestOrderStrategy::SmallFirst:
				return GetArea(A) < GetArea(AB);
			case MetENestOrderStrategy::LongSideFirst:{
				double LongA = std::max(GetWidth(A), GetHeight(A));
				double LongB = std::max(GetWidth(AB), GetHeight(AB));
				return LongA > LongB;
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
				return RatioA > RatioB;
			}
			default:
				return GetArea(A) > GetArea(AB);
			}
		});

	int Priority = static_cast<int>(AItems.size());

	for (std::size_t Index : Indices){
		
		
		AItems[Index].priority(Priority--);
	}
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

TetTNestEvalResult ET::NEST2DMANAGERLIB::CetStrategyManager::EvaluatePackedResultWithMeta(const CetTNestItemVector& AItems, const std::vector<TetMetaItem>& AMetaItems, const CetTNestItemVector& AOriginalItems, std::size_t ALayers)
{
	TetTNestEvalResult Result{};
	Result.Layers = ALayers;
	if(AItems.size() != AMetaItems.size()){
		std::cout << "[NEST][EVAL][ERROR] PackedItems size != MetaItems size. " << "PackedItems = " << AItems.size() << ", MetaItems = " << AMetaItems.size() << std::endl;
		return Result;
	}

	for(std::size_t PackedIndex = 0; PackedIndex < AItems.size(); ++PackedIndex){
		const auto& PackedItem = AItems[PackedIndex];
		const auto& Meta = AMetaItems[PackedIndex];
		int PackedBinId = PackedItem.binId();
		std::cout << "[NEST][EVAL][PACKED] " << "PackedIndex = " << PackedIndex << ", BinId = " << PackedBinId << ", IsCluster = " << Meta.IsCluster << ", Children = " << Meta.TransformData.size() << std::endl;

		if(PackedItem.binId() != 0){
			continue;
		}
		for(const auto& Transform : Meta.TransformData){
			int OriginalId = Transform.OriginalId;
			if(OriginalId < 0 || OriginalId >= static_cast<int>(AOriginalItems.size())){
				std::cout << "[NEST][EVAL][WARN] Invalid OriginalId = " << OriginalId << ", OriginalItems.size = " << AOriginalItems.size() << std::endl;
				continue;
			}
			double OriginalArea = std::abs(static_cast<double>(AOriginalItems[OriginalId].area()));
			Result.FirstBinCount++;
			Result.FirstBinArea += OriginalArea;
			std::cout << "[NEST][EVAL][CHILD] " << "OriginalId = " << OriginalId << ", OriginalArea = " << OriginalArea << std::endl;
		}
	}
	std::cout << "[NEST][EVAL][RETURN] " << "Count = " << Result.FirstBinCount << ", Area = " << Result.FirstBinArea << ", Layers = " << Result.Layers << std::endl;
	return Result;
}

