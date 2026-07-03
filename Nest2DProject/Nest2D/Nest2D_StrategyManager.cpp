#include "pch.h"
#include "Nest2D_StrategyManager.h"

ET::NEST2DMANAGERLIB::CetStrategyManager::CetStrategyManager() :CetCoreObject()
{
}

ET::NEST2DMANAGERLIB::CetStrategyManager::~CetStrategyManager()
{
}

TetTetTNestEvalResult ET::NEST2DMANAGERLIB::CetStrategyManager::EvaluateNestResult(const CetTNestItemVector& Items, std::size_t Layers)
{
	TetTetTNestEvalResult Result;
	Result.Layers = Layers;
	for (const auto& Item : Items)
	{
		if (Item.binId() == 0)
		{
			Result.FirstBinCount++;
			Result.FirstBinArea += std::abs( static_cast<double>(Item.area()));
		}
	}

	return Result;
}

bool ET::NEST2DMANAGERLIB::CetStrategyManager::IsBetterNestResult(const TetTetTNestEvalResult& A, const TetTetTNestEvalResult& B)
{
	// 第一个 bin 里零件数量最多
	if (A.FirstBinCount != B.FirstBinCount)
	{
		return A.FirstBinCount > B.FirstBinCount;
	}

	//数量相同，则第一个 bin 里使用面积更大
	if (std::abs(A.FirstBinArea - B.FirstBinArea) > 1e-6)
	{
		return A.FirstBinArea > B.FirstBinArea;
	}

	// 总层数更少
	return A.Layers < B.Layers;
}

void ET::NEST2DMANAGERLIB::CetStrategyManager::ApplyNestPriorityStrategy(CetTNestItemVector& AItems, MetENestOrderStrategy AStrategy)
{
	std::vector<std::size_t> Indices;
	Indices.reserve(AItems.size());
	for (std::size_t i = 0; i < AItems.size(); ++i){
		Indices.push_back(i);
	}
	auto GetArea = [&](std::size_t Index) -> double{
			return static_cast<double>(AItems[Index].area());
		};

	auto GetBox = [&](std::size_t Index){
			return AItems[Index].boundingBox();
		};

	auto GetWidth = [&](std::size_t Index) -> double{
		return static_cast<double>(GetBox(Index).width());
		};

	auto GetHeight = [&](std::size_t Index) -> double{
			return static_cast<double>(GetBox(Index).height());
		};

	std::sort(Indices.begin(),Indices.end(),[&](std::size_t A, std::size_t B){
			switch (AStrategy){
			case MetENestOrderStrategy::LargeFirst:
				return GetArea(A) > GetArea(B);

			case MetENestOrderStrategy::SmallFirst:
				return GetArea(A) < GetArea(B);

			case MetENestOrderStrategy::LongSideFirst:{
				double LongA = std::max(GetWidth(A), GetHeight(A));
				double LongB = std::max(GetWidth(B), GetHeight(B));
				return LongA > LongB;
			}
			case MetENestOrderStrategy::ThinFirst:{
				double WA = GetWidth(A);
				double HA = GetHeight(A);
				double WB = GetWidth(B);
				double HB = GetHeight(B);

				double ShortA = std::max(1.0, std::min(WA, HA));
				double ShortB = std::max(1.0, std::min(WB, HB));

				double RatioA = std::max(WA, HA) / ShortA;
				double RatioB = std::max(WB, HB) / ShortB;

				return RatioA > RatioB;
			}

			default:
				return GetArea(A) > GetArea(B);
			}
		});

	int Priority = static_cast<int>(AItems.size());

	for (std::size_t Index : Indices)
	{
		// 这里要求 CetTNestItem 支持 priority(int) setter。
		// 如果编译不过，需要在你的 Item 封装里暴露 priority 设置接口。
		AItems[Index].priority(Priority--);
	}
}

void ET::NEST2DMANAGERLIB::CetStrategyManager::PrintBinCount(const CetTNestItemVector& AItems)
{
	std::map<int, int> BinCount;

	for (const auto& Item : AItems)
	{
		BinCount[Item.binId()]++;
	}

	for (const auto& Pair : BinCount)
	{
		std::cout << "[NEST] binId = " << Pair.first
			<< ", count = " << Pair.second << std::endl;
	}
}

