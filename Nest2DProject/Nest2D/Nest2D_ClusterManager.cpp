#include "pch.h"
#include "Nest2D_SelfFunction.h"
#include "Nest2D_PrivateDataType.h"
#include "Nest2D_TriangleClusterBuilder.h"
#include "Nest2D_CircleClusterBuilder.h"
#include "Nest2D_RectangleFillClusterBuilder.h"
#include "Nest2D_EllipseClusterBuilder.h"
#include "Nest2D_RectangleClusterBuilder.h"
#include "Nest2D_ArcClusterBuilder.h"
#include "Nest2D_CustomClusterBuilder.h"
#include "Nest2D_ClusterBoundary.h"
#include "Nest2D_ClusterGeometryHelper.h"
#include "Nest2D_RotationUtils.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>
#include <map>
#include <sstream>
#include <unordered_map>

using namespace ClipperLib;
using namespace libnest2d;

namespace {

constexpr int kMaxSwapRounds = 2;
constexpr int kMaxSwapClusters = 64;
constexpr double kMinSwapGainRatio = 0.05;

struct TetClusterFillSearchConfig {
	std::size_t BeamWidth = CET_CLUSTER_FILL_BEAM_WIDTH;
	std::size_t MaxDepth = CET_CLUSTER_FILL_MAX_DEPTH;
	std::size_t MaxCandidateFillers = CET_CLUSTER_FILL_MAX_CANDIDATE_FILLERS;
	std::size_t MaxPlacementAttempts = CET_CLUSTER_FILL_MAX_PLACEMENT_ATTEMPTS;
};

struct TetClusterFillSearchState {
	TetClusterCandidate Candidate;
	std::size_t FillerCount = 0;
};

struct TetClusterFillSearchStats {
	std::size_t GeneratedVariantCount = 0;
	std::size_t DeduplicatedVariantCount = 0;
	std::size_t SearchAttempts = 0;
	std::size_t FreeRegionCount = 0;
	std::size_t EnvelopeGeneratedVariantCount = 0;
	std::size_t EnvelopeDeduplicatedVariantCount = 0;
	std::size_t EnvelopeSearchAttempts = 0;
	std::size_t EnvelopeFreeRegionCount = 0;
	double BaseFillRatioSum = 0.0;
	double FilledFillRatioSum = 0.0;
	double BestFillRatioGain = 0.0;
	double BestEnvelopeFillRatioGain = 0.0;
};

TetClusterFillSearchConfig GetClusterFillSearchConfig(std::size_t AItemCount) {
	if (AItemCount > CET_NEST_REDUCED_STRATEGY_ITEM_LIMIT) {
		return { CET_CLUSTER_FILL_REDUCED_ORDER_BEAM_WIDTH, CET_CLUSTER_FILL_REDUCED_ORDER_MAX_DEPTH, CET_CLUSTER_FILL_REDUCED_ORDER_MAX_CANDIDATE_FILLERS, CET_CLUSTER_FILL_REDUCED_ORDER_MAX_PLACEMENT_ATTEMPTS };
	}
	if (AItemCount > CET_NEST_FULL_STRATEGY_ITEM_LIMIT) {
		// Once the order is larger than the full strategy limit, keep template
		// fill to one bounded layer.  Exact Clipper containment is comparatively
		// expensive for 100+ item inputs and a deeper beam would delay the main
		// nesting pass without reliably improving global selection.
		return { CET_CLUSTER_FILL_REDUCED_ORDER_BEAM_WIDTH, CET_CLUSTER_FILL_REDUCED_ORDER_MAX_DEPTH, CET_CLUSTER_FILL_REDUCED_ORDER_MAX_CANDIDATE_FILLERS, CET_CLUSTER_FILL_REDUCED_ORDER_MAX_PLACEMENT_ATTEMPTS };
	}
	return {};
}

TetClusterFillSearchConfig GetClusterEnvelopeFillSearchConfig(std::size_t AItemCount) {
	if (AItemCount > CET_NEST_FULL_STRATEGY_ITEM_LIMIT) {
		return {
			CET_CLUSTER_ENVELOPE_FILL_LARGE_ORDER_BEAM_WIDTH,
			CET_CLUSTER_ENVELOPE_FILL_LARGE_ORDER_MAX_DEPTH,
			CET_CLUSTER_ENVELOPE_FILL_LARGE_ORDER_MAX_CANDIDATE_FILLERS,
			CET_CLUSTER_ENVELOPE_FILL_LARGE_ORDER_MAX_PLACEMENT_ATTEMPTS
		};
	}
	return GetClusterFillSearchConfig(AItemCount);
}

bool ContainsOriginalIndex(const TetClusterCandidate& ACandidate, int AOriginalIndex) {
	return std::find(ACandidate.OriginalIndices.begin(), ACandidate.OriginalIndices.end(), AOriginalIndex) != ACandidate.OriginalIndices.end();
}

bool IsFillMetricLess(double ALeft, double ARight) {
	return ALeft < ARight - CET_CLUSTER_FILL_VARIANT_POSITION_TOLERANCE;
}

bool IsFilledVariantBetter(const TetClusterFillSearchState& AFirst, const TetClusterFillSearchState& ASecond) {
	const TetClusterCandidate& First = AFirst.Candidate;
	const TetClusterCandidate& Second = ASecond.Candidate;
	if (IsFillMetricLess(First.ProxyWasteArea, Second.ProxyWasteArea)) return true;
	if (IsFillMetricLess(Second.ProxyWasteArea, First.ProxyWasteArea)) return false;
	if (First.FillRatio > Second.FillRatio + 1e-9) return true;
	if (Second.FillRatio > First.FillRatio + 1e-9) return false;
	if (First.ProxyWasteRatio < Second.ProxyWasteRatio - 1e-9) return true;
	if (Second.ProxyWasteRatio < First.ProxyWasteRatio - 1e-9) return false;
	if (IsFillMetricLess(First.ProxyArea, Second.ProxyArea)) return true;
	if (IsFillMetricLess(Second.ProxyArea, First.ProxyArea)) return false;
	if (First.FragmentationRisk < Second.FragmentationRisk - 1e-9) return true;
	if (Second.FragmentationRisk < First.FragmentationRisk - 1e-9) return false;
	if (AFirst.FillerCount != ASecond.FillerCount) return AFirst.FillerCount < ASecond.FillerCount;
	return First.OriginalIndices < Second.OriginalIndices;
}

bool IsFilledVariantWorthKeeping(const TetClusterCandidate& ABaseCandidate, const TetClusterCandidate& ACandidate) {
	const double AreaTolerance = std::max(1.0, ABaseCandidate.ProxyArea * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE);
	return ACandidate.Valid && ACandidate.ProxyWasteArea < ABaseCandidate.ProxyWasteArea - AreaTolerance
		&& ACandidate.FillRatio > ABaseCandidate.FillRatio + 1e-9;
}

bool HasFullRectangleProxy(const TetClusterCandidate& ACandidate) {
	const double BoundingArea = ACandidate.ClusterWidth * ACandidate.ClusterHeight;
	const double AreaTolerance = std::max(1.0, BoundingArea * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE);
	return ACandidate.Valid && ACandidate.ClusterWidth > 0.0 && ACandidate.ClusterHeight > 0.0
		&& std::isfinite(BoundingArea) && std::abs(ACandidate.ProxyArea - BoundingArea) <= AreaTolerance;
}

bool BuildRectangleEnvelopeCandidate(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const TetClusterCandidate& ABaseCandidate, TetClusterCandidate& AOutEnvelopeCandidate) {
	AOutEnvelopeCandidate = TetClusterCandidate{};
	if (!ABaseCandidate.Valid || ABaseCandidate.OriginalIndices.size() < 2 || HasFullRectangleProxy(ABaseCandidate)) {
		return false;
	}
	AOutEnvelopeCandidate = ABaseCandidate;
	ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
	if (!Geometry.FinalizeCandidateInRectangle(AOriginalItems, AOptions, AOutEnvelopeCandidate, ABaseCandidate.ClusterWidth, ABaseCandidate.ClusterHeight)) {
		return false;
	}
	AOutEnvelopeCandidate.BuilderName = ABaseCandidate.BuilderName;
	AOutEnvelopeCandidate.ClusterType = ABaseCandidate.ClusterType;
	return AOutEnvelopeCandidate.ProxyArea > ABaseCandidate.ProxyArea + std::max(1.0, ABaseCandidate.ProxyArea * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE);
}

bool IsEnvelopeFilledVariantWorthKeeping(const TetClusterCandidate& ABaseCandidate, const TetClusterCandidate& AEnvelopeCandidate, const TetClusterCandidate& ACandidate) {
	const double EnvelopeTolerance = std::max(1.0, AEnvelopeCandidate.ProxyArea * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE);
	// The original proxy may follow a circle or ellipse boundary. Once this
	// candidate reserves a board-axis rectangle, compare density in that same
	// rectangle so an exterior filler is not rejected merely for regularizing
	// the skeleton's outline.
	const double BaseEnvelopeFillRatio = ABaseCandidate.BoundingFillRatio;
	const double BaseGain = ACandidate.FillRatio - BaseEnvelopeFillRatio;
	return ACandidate.Valid && ACandidate.OriginalIndices.size() > ABaseCandidate.OriginalIndices.size()
		&& std::abs(ACandidate.ProxyArea - AEnvelopeCandidate.ProxyArea) <= EnvelopeTolerance
		&& ACandidate.ProxyWasteArea < AEnvelopeCandidate.ProxyWasteArea - EnvelopeTolerance
		&& ACandidate.FillRatio > AEnvelopeCandidate.FillRatio + 1e-9
		&& BaseGain >= CET_CLUSTER_ENVELOPE_FILL_MIN_FILL_RATIO_GAIN;
}

std::string MakeFilledVariantKey(const TetClusterCandidate& ACandidate) {
	std::vector<TetItemTransform> Transforms = ACandidate.Transforms;
	std::sort(Transforms.begin(), Transforms.end(), [](const TetItemTransform& AFirst, const TetItemTransform& ASecond) { return AFirst.OriginalId < ASecond.OriginalId; });
	std::vector<int> Indices = ACandidate.OriginalIndices;
	std::sort(Indices.begin(), Indices.end());
	std::ostringstream Stream;
	for (int Index : Indices) Stream << Index << ',';
	Stream << '|';
	for (const TetItemTransform& Transform : Transforms) {
		Stream << Transform.OriginalId << ':'
			<< std::llround(Transform.RelativeX / CET_CLUSTER_FILL_VARIANT_POSITION_TOLERANCE) << ':'
			<< std::llround(Transform.RelativeY / CET_CLUSTER_FILL_VARIANT_POSITION_TOLERANCE) << ':'
			<< std::llround(Transform.RelativeRotation / CET_CLUSTER_FILL_VARIANT_ROTATION_TOLERANCE) << ';';
	}
	return Stream.str();
}

void DeduplicateFilledStates(std::vector<TetClusterFillSearchState>& AStates) {
	std::map<std::string, TetClusterFillSearchState> UniqueStates;
	for (const TetClusterFillSearchState& State : AStates) {
		const std::string Key = MakeFilledVariantKey(State.Candidate);
		auto It = UniqueStates.find(Key);
		if (It == UniqueStates.end() || IsFilledVariantBetter(State, It->second)) UniqueStates[Key] = State;
	}
	AStates.clear();
	for (const auto& Entry : UniqueStates) AStates.push_back(Entry.second);
}

void TrimFillBeam(std::vector<TetClusterFillSearchState>& AStates, std::size_t AMaxCount) {
	std::stable_sort(AStates.begin(), AStates.end(), IsFilledVariantBetter);
	if (AStates.size() > AMaxCount) AStates.resize(AMaxCount);
}

bool FitsAnyFreeRegion(const TetShapeFeature& AFeature, const std::vector<TetClusterFreeRegion>& AFreeRegions) {
	for (const TetClusterFreeRegion& Region : AFreeRegions) {
		const bool FitsNormal = AFeature.Width <= Region.Width && AFeature.Height <= Region.Height;
		const bool FitsRotated = AFeature.Height <= Region.Width && AFeature.Width <= Region.Height;
		if (AFeature.Area <= Region.Area && (FitsNormal || FitsRotated)) return true;
	}
	return false;
}

std::vector<int> CollectCompatibleFillers(const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ABaseCandidate, const std::vector<TetClusterFreeRegion>& AFreeRegions, const TetClusterFillSearchConfig& AConfig) {
	std::vector<int> Fillers;
	const double AvailableArea = std::max(0.0, ABaseCandidate.ProxyWasteArea);
	const double AreaTolerance = std::max(1.0, ABaseCandidate.ProxyArea * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE);
	for (int Index = 0; Index < static_cast<int>(AFeatures.size()); ++Index) {
		const TetShapeFeature& Feature = AFeatures[Index];
		if (!ContainsOriginalIndex(ABaseCandidate, Index) && std::isfinite(Feature.Area) && Feature.Area > 0.0
			&& Feature.Area <= AvailableArea + AreaTolerance && FitsAnyFreeRegion(Feature, AFreeRegions)) Fillers.push_back(Index);
	}
	std::stable_sort(Fillers.begin(), Fillers.end(), [&](int AFirst, int ASecond) {
		if (std::abs(AFeatures[AFirst].Area - AFeatures[ASecond].Area) > 1.0) return AFeatures[AFirst].Area > AFeatures[ASecond].Area;
		if (std::abs(AFeatures[AFirst].OrientedFillRatio - AFeatures[ASecond].OrientedFillRatio) > 1e-9) return AFeatures[AFirst].OrientedFillRatio > AFeatures[ASecond].OrientedFillRatio;
		return AFirst < ASecond;
	});
	if (Fillers.size() > AConfig.MaxCandidateFillers) {
		const std::size_t LargestCount = (AConfig.MaxCandidateFillers + 1) / 2;
		const std::size_t SmallestCount = AConfig.MaxCandidateFillers - LargestCount;
		std::vector<int> BoundedFillers;
		BoundedFillers.reserve(AConfig.MaxCandidateFillers);
		BoundedFillers.insert(BoundedFillers.end(), Fillers.begin(), Fillers.begin() + static_cast<std::vector<int>::difference_type>(LargestCount));
		BoundedFillers.insert(BoundedFillers.end(), Fillers.end() - static_cast<std::vector<int>::difference_type>(SmallestCount), Fillers.end());
		Fillers = std::move(BoundedFillers);
	}
	return Fillers;
}

void BuildFilledVariantsForBase(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions, const TetClusterCandidate& ABaseCandidate, const TetClusterFillSearchConfig& AConfig, std::vector<TetClusterCandidate>& AOutVariants, TetClusterFillSearchStats& AStats) {
	AOutVariants.clear();
	if (!ABaseCandidate.Valid || ABaseCandidate.OriginalIndices.size() < 2 || ABaseCandidate.ProxyWasteArea <= 0.0) return;
	ET::NEST2DMANAGERLIB::CetRectangleFillClusterBuilder Builder;
	ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
	std::vector<TetClusterFillSearchState> Beam{ { ABaseCandidate, 0 } };
	std::vector<TetClusterFillSearchState> AllVariants;
	for (std::size_t Depth = 0; Depth < AConfig.MaxDepth && !Beam.empty(); ++Depth) {
		std::vector<TetClusterFillSearchState> NextBeam;
		for (const TetClusterFillSearchState& State : Beam) {
			std::vector<TetClusterFreeRegion> FreeRegions;
			if (!Geometry.ExtractCandidateFreeRegions(AOriginalItems, AOptions, State.Candidate, FreeRegions) || FreeRegions.empty()) continue;
			AStats.FreeRegionCount += FreeRegions.size();
			const std::vector<int> Fillers = CollectCompatibleFillers(AFeatures, State.Candidate, FreeRegions, AConfig);
			for (int FillerIndex : Fillers) {
				if (AStats.SearchAttempts >= AConfig.MaxPlacementAttempts) break;
				if (ContainsOriginalIndex(State.Candidate, FillerIndex)) continue;
				++AStats.SearchAttempts;
				TetClusterCandidate Candidate;
				if (!Builder.TryAppendFillerInFreeRegions(AOriginalItems, AFeatures, ABaseCandidate, State.Candidate, FreeRegions, FillerIndex, AOptions, Candidate)) continue;
				if (!IsFilledVariantWorthKeeping(ABaseCandidate, Candidate)) continue;
				NextBeam.push_back({ std::move(Candidate), State.FillerCount + 1 });
				++AStats.GeneratedVariantCount;
			}
		}
		if (AStats.SearchAttempts >= AConfig.MaxPlacementAttempts) break;
		DeduplicateFilledStates(NextBeam);
		TrimFillBeam(NextBeam, AConfig.BeamWidth);
		AllVariants.insert(AllVariants.end(), NextBeam.begin(), NextBeam.end());
		Beam = std::move(NextBeam);
	}
	DeduplicateFilledStates(AllVariants);
	TrimFillBeam(AllVariants, CET_CLUSTER_FILL_MAX_VARIANTS_PER_BASE);
	AStats.DeduplicatedVariantCount += AllVariants.size();
	for (const TetClusterFillSearchState& State : AllVariants) {
		AStats.FilledFillRatioSum += State.Candidate.FillRatio;
		AStats.BestFillRatioGain = std::max(AStats.BestFillRatioGain, State.Candidate.FillRatio - ABaseCandidate.FillRatio);
		AOutVariants.push_back(State.Candidate);
	}
}

void BuildEnvelopeFilledVariantsForBase(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions, const TetClusterCandidate& ABaseCandidate, const TetClusterFillSearchConfig& AConfig, std::vector<TetClusterCandidate>& AOutVariants, TetClusterFillSearchStats& AStats) {
	AOutVariants.clear();
	TetClusterCandidate EnvelopeCandidate;
	if (!BuildRectangleEnvelopeCandidate(AOriginalItems, AOptions, ABaseCandidate, EnvelopeCandidate) || EnvelopeCandidate.ProxyWasteArea <= 0.0) {
		return;
	}

	ET::NEST2DMANAGERLIB::CetRectangleFillClusterBuilder Builder;
	ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
	std::vector<TetClusterFillSearchState> Beam{ { EnvelopeCandidate, 0 } };
	std::vector<TetClusterFillSearchState> AllVariants;
	for (std::size_t Depth = 0; Depth < AConfig.MaxDepth && !Beam.empty(); ++Depth) {
		std::vector<TetClusterFillSearchState> NextBeam;
		for (const TetClusterFillSearchState& State : Beam) {
			std::vector<TetClusterFreeRegion> FreeRegions;
			if (!Geometry.ExtractCandidateFreeRegions(AOriginalItems, AOptions, State.Candidate, FreeRegions) || FreeRegions.empty()) continue;
			AStats.EnvelopeFreeRegionCount += FreeRegions.size();
			const std::vector<int> Fillers = CollectCompatibleFillers(AFeatures, State.Candidate, FreeRegions, AConfig);
			for (int FillerIndex : Fillers) {
				if (AStats.EnvelopeSearchAttempts >= AConfig.MaxPlacementAttempts) break;
				if (ContainsOriginalIndex(State.Candidate, FillerIndex)) continue;
				++AStats.EnvelopeSearchAttempts;
				TetClusterCandidate Candidate;
				if (!Builder.TryAppendFillerInRectangleEnvelope(AOriginalItems, AFeatures, ABaseCandidate, EnvelopeCandidate, State.Candidate, FreeRegions, FillerIndex, AOptions, Candidate)) continue;
				if (!IsEnvelopeFilledVariantWorthKeeping(ABaseCandidate, EnvelopeCandidate, Candidate)) continue;
				const double FillGain = Candidate.FillRatio - ABaseCandidate.BoundingFillRatio;
				Candidate.Score = std::max(Candidate.Score, ABaseCandidate.Score + FillGain * CET_CLUSTER_ENVELOPE_FILL_SCORE_PER_RATIO);
				NextBeam.push_back({ std::move(Candidate), State.FillerCount + 1 });
				++AStats.EnvelopeGeneratedVariantCount;
			}
		}
		if (AStats.EnvelopeSearchAttempts >= AConfig.MaxPlacementAttempts) break;
		DeduplicateFilledStates(NextBeam);
		TrimFillBeam(NextBeam, AConfig.BeamWidth);
		AllVariants.insert(AllVariants.end(), NextBeam.begin(), NextBeam.end());
		Beam = std::move(NextBeam);
	}
	DeduplicateFilledStates(AllVariants);
	TrimFillBeam(AllVariants, CET_CLUSTER_FILL_MAX_VARIANTS_PER_BASE);
	AStats.EnvelopeDeduplicatedVariantCount += AllVariants.size();
	for (const TetClusterFillSearchState& State : AllVariants) {
		AStats.BestEnvelopeFillRatioGain = std::max(AStats.BestEnvelopeFillRatioGain, State.Candidate.FillRatio - ABaseCandidate.BoundingFillRatio);
		AOutVariants.push_back(State.Candidate);
	}
}

TetPairCandidateKey MakePairCandidateKey(int AFirst, int ASecond) {
	if (AFirst > ASecond) {
		std::swap(AFirst, ASecond);
	}
	return { AFirst, ASecond };
}

using TPairCandidateLookup = std::unordered_map<TetPairCandidateKey, const TetClusterCandidate*, TetPairCandidateKeyHash>;

bool IsPairCandidateUsable(const TetClusterCandidate& ACandidate, int AOriginalItemCount) {
	if (!ACandidate.Valid || ACandidate.OriginalIndices.size() != 2 || ACandidate.Transforms.size() != 2 || ACandidate.ProxyContour.size() < 3 || ACandidate.ClusterWidth <= 0.0 || ACandidate.ClusterHeight <= 0.0 || ACandidate.ProxyArea <= 0.0 || !std::isfinite(ACandidate.Score)) {
		return false;
	}
	const int FirstIndex = ACandidate.OriginalIndices[0];
	const int SecondIndex = ACandidate.OriginalIndices[1];
	return FirstIndex >= 0 && SecondIndex >= 0 && FirstIndex < AOriginalItemCount && SecondIndex < AOriginalItemCount && FirstIndex != SecondIndex;
}

void BuildPairCandidateLookup(const std::vector<TetClusterCandidate>& ACandidates, int AOriginalItemCount, TPairCandidateLookup& AOutLookup) {
	AOutLookup.clear();
	AOutLookup.reserve(ACandidates.size());
	for (const TetClusterCandidate& Candidate : ACandidates) {
		if (!IsPairCandidateUsable(Candidate, AOriginalItemCount)) {
			continue;
		}
		const TetPairCandidateKey Key = MakePairCandidateKey(Candidate.OriginalIndices[0], Candidate.OriginalIndices[1]);
		auto It = AOutLookup.find(Key);
		if (It == AOutLookup.end() || Candidate.Score > It->second->Score) {
			AOutLookup[Key] = &Candidate;
		}
	}
}

std::vector<std::size_t> CollectPairCandidatePositions(const std::vector<TetClusterCandidate>& ACandidates) {
	std::vector<std::size_t> PairPositions;
	PairPositions.reserve(ACandidates.size());
	for (std::size_t CandidateIndex = 0; CandidateIndex < ACandidates.size(); ++CandidateIndex) {
		const TetClusterCandidate& Candidate = ACandidates[CandidateIndex];
		if (Candidate.Valid && Candidate.OriginalIndices.size() == 2 && std::isfinite(Candidate.Score)) {
			PairPositions.push_back(CandidateIndex);
		}
	}
	std::stable_sort(PairPositions.begin(), PairPositions.end(), [&](std::size_t AFirstPosition, std::size_t ASecondPosition) {
		return ACandidates[AFirstPosition].Score > ACandidates[ASecondPosition].Score;
		});
	if (PairPositions.size() > static_cast<std::size_t>(kMaxSwapClusters)) {
		PairPositions.resize(kMaxSwapClusters);
	}
	return PairPositions;
}

bool TryFindBetterPairSwap(const TPairCandidateLookup& APairCandidateLookup, const TetClusterCandidate& AFirstCandidate, const TetClusterCandidate& ASecondCandidate, const TetClusterCandidate*& AOutFirstCandidate, const TetClusterCandidate*& AOutSecondCandidate) {
	AOutFirstCandidate = nullptr;
	AOutSecondCandidate = nullptr;
	if (AFirstCandidate.OriginalIndices.size() != 2 || ASecondCandidate.OriginalIndices.size() != 2) {
		return false;
	}
	const int A = AFirstCandidate.OriginalIndices[0];
	const int B = AFirstCandidate.OriginalIndices[1];
	const int C = ASecondCandidate.OriginalIndices[0];
	const int D = ASecondCandidate.OriginalIndices[1];
	if (A == B || A == C || A == D || B == C || B == D || C == D) {
		return false;
	}

	double BestScore = AFirstCandidate.Score + ASecondCandidate.Score;
	const auto TrySelectSwap = [&](int AFirstIndex, int ASecondIndex, int BFirstIndex, int BSecondIndex) {
		const auto FirstIt = APairCandidateLookup.find(MakePairCandidateKey(AFirstIndex, ASecondIndex));
		const auto SecondIt = APairCandidateLookup.find(MakePairCandidateKey(BFirstIndex, BSecondIndex));
		if (FirstIt == APairCandidateLookup.end() || SecondIt == APairCandidateLookup.end() || FirstIt->second == SecondIt->second) {
			return;
		}
		const double NewScore = FirstIt->second->Score + SecondIt->second->Score;
		const double GainRatio = (NewScore - BestScore) / std::max(std::abs(BestScore), 1.0);
		if (GainRatio >= kMinSwapGainRatio) {
			BestScore = NewScore;
			AOutFirstCandidate = FirstIt->second;
			AOutSecondCandidate = SecondIt->second;
		}
		};
	TrySelectSwap(A, C, B, D);
	TrySelectSwap(A, D, B, C);
	return AOutFirstCandidate != nullptr && AOutSecondCandidate != nullptr;
}

}

namespace ET {
	namespace NEST2DMANAGERLIB {

		CetClusterManager::CetClusterManager() : CetCoreObject()
		{
		}

		CetClusterManager::~CetClusterManager()
		{
		}

		TetClusterBuildResult CetClusterManager::BuildClusterItems(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, MetClusterStrategy AStrategy)
		{
			TetClusterBuildResult Result;
			Result.NestItems.reserve(AOriginalItems.size());
			Result.MetaItems.reserve(AOriginalItems.size());
			if (AOriginalItems.empty()) {
				return Result;
			}
			//zanshibuzuhe
			if (AStrategy == MetClusterStrategy::None) {
				for (int i = 0; i < static_cast<int>(AOriginalItems.size()); ++i) {
					_AddSingleItem(AOriginalItems, i, Result);
				}
				return Result;
			}


			if (AStrategy == MetClusterStrategy::RightTrianglePair) {
				std::vector<bool> Used(AOriginalItems.size(), false);
				for (int i = 0; i < static_cast<int>(AOriginalItems.size()); ++i) {
					if (Used[i]) {
						continue;
					}
					bool Paired = false;
					for (int j = i + 1; j < static_cast<int>(AOriginalItems.size()); ++j) {
						if (Used[j]) {
							continue;
						}
						// if (_TryMakeRightTrianglePair(AOriginalItems, i, j, AOptions, Result)) 
						if (Nest2DUtils->Nest2dClusterTri->TryMakeRightTrianglePair(AOriginalItems, i, j, AOptions, Result)) {
							Used[i] = true;
							Used[j] = true;
							Paired = true;
							std::cout << "[CLUSTER] Pair accepted: " << i << " + " << j << ", PackedCount = " << Result.NestItems.size() << std::endl;
							break;
						}
					}
					if (!Paired) {
						Used[i] = true;
						_AddSingleItem(AOriginalItems, i, Result);
					}
				}
				return Result;
			}
			if (AStrategy == MetClusterStrategy::AutoPairCluster) {
				return _BuildAutoPairClusters(AOriginalItems, AOptions);
			}

			for (int i = 0; i < static_cast<int>(AOriginalItems.size()); ++i) {
				_AddSingleItem(AOriginalItems, i, Result);
			}
			return Result;
		}

		TetClusterBuildResult CetClusterManager::BuildClusterItemsWithFeatures(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions, MetClusterStrategy AStrategy)
		{

			if (AStrategy == MetClusterStrategy::TemplateCluster) {
				return _BuildTemplateClusters(AOriginalItems, AFeatures, AOptions);
			}
			return BuildClusterItems(AOriginalItems, AOptions, AStrategy);
		}

        std::vector<int> CetClusterManager::RankBoardCompositeSkeletons(const CetTNestItemVector& AItems, const std::vector<TetShapeFeature>& AFeatures, int ATargetBin, const TetClusterFreeRegion& AFreeRegion, std::size_t AMaxCount) const
        {
            std::vector<std::pair<double, int>> Ranked;
            if (AItems.size() != AFeatures.size() || AMaxCount == 0 || AFreeRegion.Area <= 0.0) return {};
            const double RegionAspect = AFreeRegion.Height > 0.0 ? AFreeRegion.Width / AFreeRegion.Height : 0.0;
            for (std::size_t Index = 0; Index < AItems.size(); ++Index) {
                const TetShapeFeature& Feature = AFeatures[Index];
                if (AItems[Index].binId() >= 0 && AItems[Index].binId() <= ATargetBin) continue;
                if (Feature.Width <= 0.0 || Feature.Height <= 0.0 || Feature.BoxArea > AFreeRegion.Area) continue;
                const bool IsSkeleton = Feature.ShapeType == MetShapeType::CircleLike || Feature.ShapeType == MetShapeType::EllipseLike
                    || Feature.ShapeType == MetShapeType::TriangleLike || Feature.ShapeType == MetShapeType::RectangleLike
                    || Feature.ShapeType == MetShapeType::ArcLike || Feature.ShapeType == MetShapeType::ConvexPolygon
                    || Feature.ShapeType == MetShapeType::ConcavePolygon || Feature.ShapeType == MetShapeType::QuadrilateralLike;
                if (!IsSkeleton) continue;
                const double Aspect = Feature.Height > 0.0 ? Feature.Width / Feature.Height : 0.0;
                const double AspectMatch = RegionAspect > 0.0 && Aspect > 0.0 ? std::min(Aspect, RegionAspect) / std::max(Aspect, RegionAspect) : 0.0;
                const double EnvelopeOpportunity = std::max(0.0, Feature.BoxArea - Feature.Area);
                const double Score = Feature.Area + EnvelopeOpportunity * CET_BOARD_COMPOSITE_ENVELOPE_OPPORTUNITY_WEIGHT + AspectMatch * Feature.BoxArea;
                Ranked.emplace_back(Score, static_cast<int>(Index));
            }
            std::stable_sort(Ranked.begin(), Ranked.end(), [](const auto& ALeft, const auto& ARight) {
                return std::abs(ALeft.first - ARight.first) > CET_BOARD_COMPOSITE_SCORE_COMPARISON_TOLERANCE ? ALeft.first > ARight.first : ALeft.second < ARight.second;
            });
            std::vector<int> Result;
            for (const auto& Entry : Ranked) { Result.push_back(Entry.second); if (Result.size() >= AMaxCount) break; }
            return Result;
        }

        std::vector<int> CetClusterManager::RankExistingBoardCompositeSkeletons(const CetTNestItemVector& AItems, const std::vector<TetShapeFeature>& AFeatures, int ATargetBin, const TetClusterFreeRegion& AFreeRegion, std::size_t AMaxCount) const
        {
            std::vector<std::pair<double, int>> Ranked;
            if (AItems.size() != AFeatures.size() || AMaxCount == 0 || AFreeRegion.Area <= 0.0) return {};
            for (std::size_t Index = 0; Index < AItems.size(); ++Index) {
                const TetShapeFeature& Feature = AFeatures[Index];
                if (AItems[Index].binId() != ATargetBin || Feature.Width <= 0.0 || Feature.Height <= 0.0) continue;
                const bool IsSkeleton = Feature.ShapeType == MetShapeType::CircleLike || Feature.ShapeType == MetShapeType::EllipseLike
                    || Feature.ShapeType == MetShapeType::TriangleLike || Feature.ShapeType == MetShapeType::RectangleLike
                    || Feature.ShapeType == MetShapeType::ArcLike || Feature.ShapeType == MetShapeType::ConvexPolygon
                    || Feature.ShapeType == MetShapeType::ConcavePolygon || Feature.ShapeType == MetShapeType::QuadrilateralLike;
                if (!IsSkeleton) continue;
                const auto Bounds = AItems[Index].boundingBox();
                const double MinX = static_cast<double>(getX(Bounds.minCorner()));
                const double MinY = static_cast<double>(getY(Bounds.minCorner()));
                const double MaxX = static_cast<double>(getX(Bounds.maxCorner()));
                const double MaxY = static_cast<double>(getY(Bounds.maxCorner()));
                const double GapX = std::max({ 0.0, AFreeRegion.MinX - MaxX, MinX - AFreeRegion.MaxX });
                const double GapY = std::max({ 0.0, AFreeRegion.MinY - MaxY, MinY - AFreeRegion.MaxY });
                const double Distance = std::hypot(GapX, GapY);
                const double Opportunity = std::max(0.0, Feature.BoxArea - Feature.Area);
                Ranked.emplace_back(Opportunity - Distance, static_cast<int>(Index));
            }
            std::stable_sort(Ranked.begin(), Ranked.end(), [](const auto& ALeft, const auto& ARight) {
                return std::abs(ALeft.first - ARight.first) > CET_BOARD_COMPOSITE_SCORE_COMPARISON_TOLERANCE ? ALeft.first > ARight.first : ALeft.second < ARight.second;
            });
            std::vector<int> Result;
            for (const auto& Entry : Ranked) { Result.push_back(Entry.second); if (Result.size() >= AMaxCount) break; }
            return Result;
        }

		void CetClusterManager::ExpandClusterResultToOriginalItems(const CetTNestItemVector& AOriginalItems, const CetTNestItemVector& APackedItems, const std::vector<TetMetaItem>& AMetaItems, CetTNestItemVector& AOutOriginalItems)
		{
			AOutOriginalItems = AOriginalItems;
			if (APackedItems.size() != AMetaItems.size()) {
				std::cout << "[CLUSTER][ERROR] PackedItems size != MetaItems size. PackedItems = " << APackedItems.size() << ", MetaItems = " << AMetaItems.size() << std::endl;
				return;
			}
			for (std::size_t PackedIndex = 0; PackedIndex < APackedItems.size(); ++PackedIndex) {
				const auto& PackedItem = APackedItems[PackedIndex];
				const auto& Meta = AMetaItems[PackedIndex];
				auto PackedTranslation = PackedItem.translation();
				double PackedX = static_cast<double>(PackedTranslation.X);
				double PackedY = static_cast<double>(PackedTranslation.Y);
				double PackedRotation = PackedItem.rotation();
				double CosR = std::cos(PackedRotation);
				double SinR = std::sin(PackedRotation);
				std::cout << "[CLUSTER][EXPAND PACKED] PackedIndex = " << PackedIndex << ", IsCluster = " << Meta.IsCluster << ", PackedBin = " << PackedItem.binId() << ", PackedX = " << PackedX << ", PackedY = " << PackedY << ", PackedRotation = " << PackedRotation << ", Children = " << Meta.TransformData.size() << std::endl;
				_ExpandClusterChildren(PackedItem, Meta, AOutOriginalItems);
			}
			std::cout << "[CLUSTER] ExpandClusterResultToOriginalItems done. Original count = " << AOutOriginalItems.size() << std::endl;
		}

		bool CetClusterManager::ValidatePackedResultSpacing(const CetTNestItemVector& AOriginalItems, const CetTNestItemVector& APackedItems, const std::vector<TetMetaItem>& AMetaItems, const TetNestOptions& AOptions, TetExpandedSpacingFailure* AOutFailure)
		{
			if (AOutFailure != nullptr) {
				*AOutFailure = TetExpandedSpacingFailure{};
			}
			if (APackedItems.size() != AMetaItems.size()) {
				return false;
			}

			CetTNestItemVector ExpandedItems = AOriginalItems;
			std::vector<int> OriginalToPackedIndex(AOriginalItems.size(), -1);
			for (std::size_t PackedIndex = 0; PackedIndex < APackedItems.size(); ++PackedIndex) {
				for (const TetItemTransform& Transform : AMetaItems[PackedIndex].TransformData) {
					if (Transform.OriginalId >= 0 && Transform.OriginalId < static_cast<int>(OriginalToPackedIndex.size())) {
						OriginalToPackedIndex[Transform.OriginalId] = static_cast<int>(PackedIndex);
					}
				}
				_ExpandClusterChildren(APackedItems[PackedIndex], AMetaItems[PackedIndex], ExpandedItems, false);
			}

			// Cluster construction and packing both use one-sided inflation: only
			// the first contour is expanded by the requested spacing.  Match that
			// convention here so a valid tangency at the required clearance is not
			// tested as two spacing widths after cluster expansion.
			const auto SpacingCoord = NestUtils::ToNestCoord(std::max(0.0, AOptions.Spacing));
			for (std::size_t FirstIndex = 0; FirstIndex < ExpandedItems.size(); ++FirstIndex) {
				if (ExpandedItems[FirstIndex].binId() < 0) {
					continue;
				}

				CetNestItem FirstItem = ExpandedItems[FirstIndex];
				FirstItem.inflation(0);
				const CetNestItem FirstRawItem = FirstItem;
				if (SpacingCoord > 0) {
					FirstItem.inflation(static_cast<decltype(FirstItem.inflation())>(std::ceil(static_cast<double>(SpacingCoord) * 0.5)));
				}
				for (std::size_t SecondIndex = FirstIndex + 1; SecondIndex < ExpandedItems.size(); ++SecondIndex) {
					if (ExpandedItems[SecondIndex].binId() != FirstItem.binId()) {
						continue;
					}

					CetNestItem SecondItem = ExpandedItems[SecondIndex];
					SecondItem.inflation(0);
					if (SpacingCoord > 0) {
						SecondItem.inflation(static_cast<decltype(SecondItem.inflation())>(std::ceil(static_cast<double>(SpacingCoord) * 0.5)));
					}
					// A tangency of the spacing-expanded contour is a legal result: the
					// nester uses half-spacing inflation on both items. Reject only an
					// interior intersection, not a shared boundary at exact clearance.
					if (CetNestItem::intersects(FirstItem, SecondItem) && !CetNestItem::touches(FirstItem, SecondItem)) {
						const bool RawContoursIntersect = CetNestItem::intersects(FirstRawItem, SecondItem);
						if (AOutFailure != nullptr) {
							AOutFailure->Valid = true;
							AOutFailure->RawContoursIntersect = RawContoursIntersect;
							AOutFailure->FirstOriginalIndex = static_cast<int>(FirstIndex);
							AOutFailure->SecondOriginalIndex = static_cast<int>(SecondIndex);
							AOutFailure->FirstPackedIndex = OriginalToPackedIndex[FirstIndex];
							AOutFailure->SecondPackedIndex = OriginalToPackedIndex[SecondIndex];
							AOutFailure->BinId = FirstItem.binId();
						}
						std::cout << "[CLUSTER][EXPANDED VALIDATION][REJECT] "
							<< (RawContoursIntersect ? "Raw contour overlap" : (SpacingCoord > 0 ? "Spacing violation" : "Overlap"))
							<< " between original items " << FirstIndex << " and " << SecondIndex
							<< " on bin " << FirstItem.binId()
							<< ", packed items " << OriginalToPackedIndex[FirstIndex] << " and " << OriginalToPackedIndex[SecondIndex]
							<< ", required spacing " << AOptions.Spacing << std::endl;
						return false;
					}
				}
			}

			return true;
		}

		void CetClusterManager::_AddSingleItem(const CetTNestItemVector& AOriginalItems, int AOriginalIndex, TetClusterBuildResult& AResult)
		{
			const int PackedIndex = static_cast<int>(AResult.NestItems.size());
			AResult.NestItems.push_back(AOriginalItems[AOriginalIndex]);
			TetMetaItem Meta;
			Meta.PackedItemIndex = PackedIndex;
			Meta.IsCluster = false;
			Meta.ClusterType = "Single";
			TetItemTransform Transform;
			Transform.OriginalId = AOriginalIndex;
			Transform.RelativeX = 0.0;
			Transform.RelativeY = 0.0;
			Transform.RelativeRotation = 0.0;
			Meta.TransformData.push_back(Transform);
			AResult.MetaItems.push_back(Meta);
		}

		double CetClusterManager::_GetItemWidth(const CetNestItem& AItem)
		{
			return static_cast<double>(AItem.boundingBox().width());
		}

		double CetClusterManager::_GetItemHeight(const CetNestItem& AItem)
		{
			return static_cast<double>(AItem.boundingBox().height());
		}

		void CetClusterManager::_ExpandClusterChildren(const CetNestItem& APackedItem, const TetMetaItem& AMeta, CetTNestItemVector& AOutOriginalItems, bool ALog)
		{
			auto PackedTranslation = APackedItem.translation();
			double PackedX = static_cast<double>(PackedTranslation.X);
			double PackedY = static_cast<double>(PackedTranslation.Y);
			double PackedRotation = APackedItem.rotation();

			double CosR = std::cos(PackedRotation);
			double SinR = std::sin(PackedRotation);
			if (ALog) {
				std::cout << "[CLUSTER][EXPAND PACKED] IsCluster = " << AMeta.IsCluster << ", PackedBin = " << APackedItem.binId() << ", PackedX = " << PackedX << ", PackedY = " << PackedY << ", PackedRotation = " << PackedRotation << ", Children = " << AMeta.TransformData.size() << std::endl;
			}
			for (const auto& Transform : AMeta.TransformData) {
				int originalId = Transform.OriginalId;
				if (originalId < 0 || originalId >= static_cast<int>(AOutOriginalItems.size())) {
					std::cout << "[ClusTer][WARN] Invalid originalId in TransformData: " << originalId << std::endl;
					continue;
				}
				auto& OriginalItem = AOutOriginalItems[originalId];
				double LocalX = Transform.RelativeX;
				double LocalY = Transform.RelativeY;

				double RotatedLocalX = LocalX * CosR - LocalY * SinR;
				double RotatedLocalY = LocalX * SinR + LocalY * CosR;
				double FinalX = PackedX + RotatedLocalX;
				double FinalY = PackedY + RotatedLocalY;
				double FinalRotation = PackedRotation + Transform.RelativeRotation;

				OriginalItem.binId(APackedItem.binId());
				OriginalItem.translation(ClipperLib::IntPoint(
					static_cast<ClipperLib::cInt>(std::llround(FinalX)),
					static_cast<ClipperLib::cInt>(std::llround(FinalY))));
				OriginalItem.rotation(FinalRotation);
				if (ALog) {
					std::cout << "[CLUSTER][EXPAND ITEM] OriginalId = " << originalId << ", Local = (" << LocalX << ", " << LocalY << ")" << ", Final = (" << FinalX << ", " << FinalY << ")" << ", FinalRotation = " << FinalRotation << ", Bin = " << APackedItem.binId() << std::endl;
				}
			}
		}

		TetClusterBuildResult CetClusterManager::_BuildTemplateClusters(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions)
		{			
			const int Count = static_cast<int>(AOriginalItems.size());
			if (Count <= 0) {
				return TetClusterBuildResult{};
			}
			if (AFeatures.size() != AOriginalItems.size()) {
				std::cout << "[TEMPLATE][ERROR] Feature count mismatch. OriginalItems=" << AOriginalItems.size() << ", Features=" << AFeatures.size() << std::endl;
				return _BuildAllSingles(AOriginalItems);
			}

			TetClusterBuildResult Result;
			Result.NestItems.reserve(AOriginalItems.size());
			Result.MetaItems.reserve(AOriginalItems.size());

			std::vector<bool> Used(Count, false);
			std::map<MetShapeType, std::vector<int>> IndicesByType;

			//  褰㈢姸鍒嗙被
			_CollectTemplateShapeIndices(AFeatures, IndicesByType);
			// Collect all template candidates.
			std::vector<TetClusterCandidate> BaseCandidates;
			_BuildTemplateCandidates(AOriginalItems, AFeatures, AOptions, IndicesByType, BaseCandidates);
			// Keep skeletons and their bounded fill variants together for global selection.
			std::vector<TetClusterCandidate> ExpandedCandidates;
			_BuildFilledTemplateCandidateVariants(AOriginalItems, AFeatures, AOptions, BaseCandidates, ExpandedCandidates);
			std::vector<TetClusterCandidate> AcceptedCandidates = _SelectAndOptimizeTemplateCandidates(AOriginalItems, AOptions, ExpandedCandidates, Used, Count);
			// Assemble clusters and append remaining singles.
			int AcceptedClusterCount = 0;
			int AcceptedFilledClusterCount = 0;
			for (const TetClusterCandidate& Candidate : AcceptedCandidates) {
				if (!_AddClusterCandidate(Candidate, Result)) {
					std::cout << "[TEMPLATE][ADD FAILED] Builder=" << Candidate.BuilderName << " Type=" << Candidate.ClusterType << std::endl;
					continue;
				}
				++AcceptedClusterCount;
				if (Candidate.BuilderName == "TemplateFillSearch" || Candidate.BuilderName == "EnvelopeFillSearch") {
					++AcceptedFilledClusterCount;
				}
				std::cout << "[TEMPLATE][ACCEPT] Builder=" << Candidate.BuilderName << " Type=" << Candidate.ClusterType << " ChildCount=" << Candidate.OriginalIndices.size() << " Score=" << Candidate.Score << std::endl;
			}
			int SingleCount = 0;
			for (int i = 0; i < Count; ++i) {
				if (Used[i]) {
					continue;
				}
				_AddSingleItem(AOriginalItems, i, Result);
				Used[i] = true;
				++SingleCount;
			}
			const bool CoverageValid = _ValidateBuildResultCoverage(Result, Count);
			std::cout << "[TEMPLATE][SUMMARY] OriginalCount=" << Count << " BaseCandidateCount=" << BaseCandidates.size() << " ExpandedCandidateCount=" << ExpandedCandidates.size() << " AcceptedClusterCount=" << AcceptedClusterCount << " AcceptedFilledClusterCount=" << AcceptedFilledClusterCount << " SingleCount=" << SingleCount << " PackedItemCount=" << Result.NestItems.size() << " MetaItemCount=" << Result.MetaItems.size() << " CoverageValid=" << CoverageValid << std::endl;
			if (!CoverageValid) {
				std::cout << "[TEMPLATE][FALLBACK] Coverage invalid, use all singles." << std::endl;
				return _BuildAllSingles(AOriginalItems);
			}

			return Result;
		}
		void CetClusterManager::_CollectTemplateShapeIndices(const std::vector<TetShapeFeature>& AFeatures, std::map<MetShapeType, std::vector<int>>& AIndicesByType)
		{
			const int Count = static_cast<int>(AFeatures.size());
			for (int i = 0; i < Count; ++i) {
				const TetShapeFeature& Feature = AFeatures[i];
				if (Feature.Width <= 0.0 || Feature.Height <= 0.0) {
					continue;
				}
				AIndicesByType[Feature.ShapeType].push_back(i);
			}

			std::cout << "[TEMPLATE][SHAPE COUNTS] Triangle=" << AIndicesByType[MetShapeType::TriangleLike].size()
				<< " Circle=" << AIndicesByType[MetShapeType::CircleLike].size()
				<< " Ellipse=" << AIndicesByType[MetShapeType::EllipseLike].size()
				<< " Rectangle=" << AIndicesByType[MetShapeType::RectangleLike].size()
				<< " Arc=" << AIndicesByType[MetShapeType::ArcLike].size()
				<< " Convex=" << AIndicesByType[MetShapeType::ConvexPolygon].size()
				<< " Concave=" << AIndicesByType[MetShapeType::ConcavePolygon].size() << std::endl;
		}
		void CetClusterManager::_BuildTemplateCandidates(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions, const std::map<MetShapeType, std::vector<int>>& AIndicesByType, std::vector<TetClusterCandidate>& ABaseCandidates)
		{
			auto AppendBuilderLog = [&](const char* ABuilderName, std::size_t AOldCount) {
				std::cout << "[TEMPLATE][BUILDER] " << ABuilderName << " NewCandidates=" << ABaseCandidates.size() - AOldCount << std::endl;
				};

			{
				const std::size_t OldCount = ABaseCandidates.size();
				CetTriangleClusterBuilder Builder;
				Builder.BuildCandidates(AOriginalItems, AFeatures, AIndicesByType.at(MetShapeType::TriangleLike), AOptions, ABaseCandidates);
				AppendBuilderLog("TriangleBuilder", OldCount);
			}
			{
				const std::size_t OldCount = ABaseCandidates.size();
				CetCircleClusterBuilder Builder;
				Builder.BuildCandidates(AOriginalItems, AFeatures, AIndicesByType.at(MetShapeType::CircleLike), AOptions, ABaseCandidates);
				AppendBuilderLog("CircleBuilder", OldCount);
			}
			{
				const std::size_t OldCount = ABaseCandidates.size();
				CetEllipseClusterBuilder Builder;
				Builder.BuildCandidates(AOriginalItems, AFeatures, AIndicesByType.at(MetShapeType::EllipseLike), AOptions, ABaseCandidates);
				AppendBuilderLog("EllipseBuilder", OldCount);
			}
			{
				const std::size_t OldCount = ABaseCandidates.size();
				CetRectangleClusterBuilder Builder;
				Builder.BuildCandidates(AOriginalItems, AFeatures, AIndicesByType.at(MetShapeType::RectangleLike), AOptions, ABaseCandidates);
				AppendBuilderLog("RectangleBuilder", OldCount);
			}
			{
				const std::size_t OldCount = ABaseCandidates.size();
				CetArcClusterBuilder Builder;
				Builder.BuildCandidates(AOriginalItems, AFeatures, AIndicesByType.at(MetShapeType::ArcLike), AOptions, ABaseCandidates);
				AppendBuilderLog("ArcBuilder", OldCount);
			}
			{
				std::vector<int> CustomIndices;
				const auto AppendCustomIndices = [&](MetShapeType AShapeType) {
					auto It = AIndicesByType.find(AShapeType);
					if (It != AIndicesByType.end()) {
						const std::vector<int>& TypeIndices = It->second;
						CustomIndices.insert(CustomIndices.end(), TypeIndices.begin(), TypeIndices.end());
					}
					};
				AppendCustomIndices(MetShapeType::QuadrilateralLike);
				AppendCustomIndices(MetShapeType::ConvexPolygon);
				AppendCustomIndices(MetShapeType::ConcavePolygon);

				const std::size_t OldCount = ABaseCandidates.size();
				CetCustomClusterBuilder Builder;
				Builder.BuildCandidates(AOriginalItems, AFeatures, CustomIndices, AOptions, ABaseCandidates);
				AppendBuilderLog("CustomBuilder", OldCount);
			}
		}

		void CetClusterManager::_BuildFilledTemplateCandidateVariants(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions, const std::vector<TetClusterCandidate>& ABaseCandidates, std::vector<TetClusterCandidate>& AOutCandidates)
		{
			AOutCandidates = ABaseCandidates;
			TetClusterFillSearchStats Stats;
			const TetClusterFillSearchConfig Config = GetClusterFillSearchConfig(AOriginalItems.size());
			std::size_t ValidBaseCandidateCount = 0;
			std::size_t FilledVariantCount = 0;
			for (const TetClusterCandidate& BaseCandidate : ABaseCandidates) {
				if (!BaseCandidate.Valid) continue;
				++ValidBaseCandidateCount;
				Stats.BaseFillRatioSum += BaseCandidate.FillRatio;
				std::vector<TetClusterCandidate> Variants;
				BuildFilledVariantsForBase(AOriginalItems, AFeatures, AOptions, BaseCandidate, Config, Variants, Stats);
				if (!Variants.empty()) {
					FilledVariantCount += Variants.size();
					std::cout << "[TEMPLATE][FILL VARIANT] BaseType=" << BaseCandidate.ClusterType << " Generated=" << Variants.size() << " BaseFillRatio=" << BaseCandidate.FillRatio << " BestFillRatio=" << Variants.front().FillRatio << std::endl;
					AOutCandidates.insert(AOutCandidates.end(), Variants.begin(), Variants.end());
				}
			}
			const TetClusterFillSearchConfig EnvelopeConfig = GetClusterEnvelopeFillSearchConfig(AOriginalItems.size());
			std::vector<const TetClusterCandidate*> EnvelopeBaseCandidates;
			EnvelopeBaseCandidates.reserve(ABaseCandidates.size());
			for (const TetClusterCandidate& BaseCandidate : ABaseCandidates) {
				if (BaseCandidate.Valid && !HasFullRectangleProxy(BaseCandidate)) {
					EnvelopeBaseCandidates.push_back(&BaseCandidate);
				}
			}
			std::stable_sort(EnvelopeBaseCandidates.begin(), EnvelopeBaseCandidates.end(), [](const TetClusterCandidate* AFirst, const TetClusterCandidate* ASecond) {
				const double FirstAvailableArea = std::max(0.0, AFirst->BoundingBoxArea - AFirst->ReservedArea);
				const double SecondAvailableArea = std::max(0.0, ASecond->BoundingBoxArea - ASecond->ReservedArea);
				if (std::abs(FirstAvailableArea - SecondAvailableArea) > 1.0) return FirstAvailableArea > SecondAvailableArea;
				return AFirst->ClusterType < ASecond->ClusterType;
				});
			const std::size_t EnvelopeBaseLimit = AOriginalItems.size() > CET_NEST_FULL_STRATEGY_ITEM_LIMIT
				? CET_CLUSTER_ENVELOPE_FILL_LARGE_ORDER_MAX_BASE_CANDIDATES
				: CET_CLUSTER_ENVELOPE_FILL_MAX_BASE_CANDIDATES;
			if (EnvelopeBaseCandidates.size() > EnvelopeBaseLimit) {
				EnvelopeBaseCandidates.resize(EnvelopeBaseLimit);
			}
			for (const TetClusterCandidate* BaseCandidate : EnvelopeBaseCandidates) {
				std::vector<TetClusterCandidate> Variants;
				BuildEnvelopeFilledVariantsForBase(AOriginalItems, AFeatures, AOptions, *BaseCandidate, EnvelopeConfig, Variants, Stats);
				if (Variants.empty()) continue;
				const TetClusterCandidate& BestVariant = Variants.front();
				std::cout << "[TEMPLATE][ENVELOPE FILL] BaseType=" << BaseCandidate->ClusterType
					<< " Generated=" << Variants.size()
					<< " BaseEnvelopeFill=" << BaseCandidate->BoundingFillRatio
					<< " BestFillRatio=" << BestVariant.FillRatio
					<< " FillGain=" << (BestVariant.FillRatio - BaseCandidate->BoundingFillRatio)
					<< " Width=" << BestVariant.ClusterWidth
					<< " Height=" << BestVariant.ClusterHeight << std::endl;
				AOutCandidates.insert(AOutCandidates.end(), Variants.begin(), Variants.end());
			}
			const double BaseAverage = ValidBaseCandidateCount == 0 ? 0.0 : Stats.BaseFillRatioSum / static_cast<double>(ValidBaseCandidateCount);
			const double FilledAverage = FilledVariantCount == 0 ? 0.0 : Stats.FilledFillRatioSum / static_cast<double>(FilledVariantCount);
			std::cout << "[TEMPLATE][FILL SUMMARY] BaseCandidateCount=" << ABaseCandidates.size()
				<< " GeneratedVariantCount=" << Stats.GeneratedVariantCount
				<< " DeduplicatedVariantCount=" << Stats.DeduplicatedVariantCount
				<< " FilledVariantCount=" << FilledVariantCount
				<< " AverageBaseFillRatio=" << BaseAverage
				<< " AverageFilledFillRatio=" << FilledAverage
				<< " BestFillRatioGain=" << Stats.BestFillRatioGain
				<< " FreeRegionCount=" << Stats.FreeRegionCount
				<< " SearchAttempts=" << Stats.SearchAttempts
				<< " EnvelopeGeneratedVariantCount=" << Stats.EnvelopeGeneratedVariantCount
				<< " EnvelopeDeduplicatedVariantCount=" << Stats.EnvelopeDeduplicatedVariantCount
				<< " EnvelopeFreeRegionCount=" << Stats.EnvelopeFreeRegionCount
				<< " EnvelopeSearchAttempts=" << Stats.EnvelopeSearchAttempts
				<< " BestEnvelopeFillRatioGain=" << Stats.BestEnvelopeFillRatioGain
				<< " EnvelopeBeamWidth=" << EnvelopeConfig.BeamWidth
				<< " EnvelopeMaxDepth=" << EnvelopeConfig.MaxDepth
				<< " EnvelopeMaxFillers=" << EnvelopeConfig.MaxCandidateFillers
				<< " EnvelopeMaxPlacementAttempts=" << EnvelopeConfig.MaxPlacementAttempts
				<< " BeamWidth=" << Config.BeamWidth
				<< " MaxDepth=" << Config.MaxDepth
				<< " MaxFillers=" << Config.MaxCandidateFillers
				<< " MaxPlacementAttempts=" << Config.MaxPlacementAttempts << std::endl;
		}

		std::vector<TetClusterCandidate> CetClusterManager::_SelectTemplateCandidates(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const std::vector<TetClusterCandidate>& ABaseCandidates, std::vector<bool>& AUsed)
		{
			const int Count = static_cast<int>(AOriginalItems.size());
			std::vector<TetClusterCandidate> SortedCandidates = ABaseCandidates;

			std::stable_sort(SortedCandidates.begin(), SortedCandidates.end(), [](const TetClusterCandidate& A, const TetClusterCandidate& AB) {
				if (std::abs(A.Score - AB.Score) > 1e-9) {
					return A.Score > AB.Score;
				}
				if (std::abs(A.SheetReuseScore - AB.SheetReuseScore) > 1e-9) {
					return A.SheetReuseScore > AB.SheetReuseScore;
				}
				if (A.OriginalIndices.size() != AB.OriginalIndices.size()) {
					return A.OriginalIndices.size() > AB.OriginalIndices.size();
				}
				if (std::abs(A.ProxyArea - AB.ProxyArea) > 1e-9) {
					return A.ProxyArea < AB.ProxyArea;
				}
				return A.ClusterType < AB.ClusterType;
				});

			std::cout << "[TEMPLATE][BASE CANDIDATE TOTAL] " << SortedCandidates.size() << std::endl;

			std::vector<TetClusterCandidate> AcceptedCandidates;
			AcceptedCandidates.reserve(SortedCandidates.size());
			for (const TetClusterCandidate& Candidate : SortedCandidates) {
				if (!_CanAcceptClusterCandidate(AOriginalItems, AOptions, Candidate, AUsed, Count)) {
					std::cout << "[TEMPLATE][REJECT] Builder=" << Candidate.BuilderName << " Type=" << Candidate.ClusterType << " Score=" << Candidate.Score << std::endl;
					continue;
				}

				AcceptedCandidates.push_back(Candidate);
				for (int OriginalIndex : Candidate.OriginalIndices) {
					AUsed[OriginalIndex] = true;
				}

				std::cout << "[TEMPLATE][BASE ACCEPT] Builder=" << Candidate.BuilderName << " Type=" << Candidate.ClusterType << " ChildCount=" << Candidate.OriginalIndices.size() << " Score=" << Candidate.Score << std::endl;
			}

			return AcceptedCandidates;
		}

		std::vector<TetClusterCandidate> CetClusterManager::_SelectAndOptimizeTemplateCandidates(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const std::vector<TetClusterCandidate>& ABaseCandidates, std::vector<bool>& AUsed, int AOriginalItemCount)
		{
#ifdef _DEBUG
			const auto GreedyStartTime = std::chrono::steady_clock::now();
#endif
			std::vector<TetClusterCandidate> AcceptedCandidates = _SelectTemplateCandidates(AOriginalItems, AOptions, ABaseCandidates, AUsed);
#ifdef _DEBUG
			const auto GreedyEndTime = std::chrono::steady_clock::now();
			const std::vector<TetClusterCandidate> GreedyResult = AcceptedCandidates;
			const double GreedyScore = _CalculateCandidateSelectionScore(GreedyResult);
			const auto OptimizeStartTime = std::chrono::steady_clock::now();
#endif
			const int SwapCount = _OptimizePairClusterSelection(ABaseCandidates, AcceptedCandidates, AOriginalItemCount);
#ifndef _DEBUG
			(void)SwapCount;
#endif
#ifdef _DEBUG
			const auto OptimizeEndTime = std::chrono::steady_clock::now();
			const std::size_t OptimizedCandidateCount = AcceptedCandidates.size();
			const double OptimizedScore = _CalculateCandidateSelectionScore(AcceptedCandidates);
#endif
			std::fill(AUsed.begin(), AUsed.end(), false);
			for (const TetClusterCandidate& Candidate : AcceptedCandidates) {
				for (int OriginalIndex : Candidate.OriginalIndices) {
					if (OriginalIndex >= 0 && OriginalIndex < AOriginalItemCount) {
						AUsed[OriginalIndex] = true;
					}
				}
			}
#ifdef _DEBUG
			const auto ToMilliseconds = [](const auto& AStart, const auto& AEnd) {
				return std::chrono::duration<double, std::milli>(AEnd - AStart).count();
				};
			std::cout << "[ClusterSelection] Greedy Score = " << GreedyScore
				<< ", Optimized Score = " << OptimizedScore
				<< ", Improvement = " << OptimizedScore - GreedyScore
				<< ", Swap Count = " << SwapCount << std::endl;
			std::cout << "[ClusterPerf] Candidates: " << ABaseCandidates.size()
				<< ", AcceptedGreedy: " << GreedyResult.size()
				<< ", AcceptedOptimized: " << OptimizedCandidateCount
				<< ", SwapCount: " << SwapCount
				<< ", GreedyMs: " << ToMilliseconds(GreedyStartTime, GreedyEndTime)
				<< ", OptimizeMs: " << ToMilliseconds(OptimizeStartTime, OptimizeEndTime) << std::endl;
#endif
			return AcceptedCandidates;
		}

		int CetClusterManager::_OptimizePairClusterSelection(const std::vector<TetClusterCandidate>& AAllCandidates, std::vector<TetClusterCandidate>& AAcceptedCandidates, int AOriginalItemCount)
		{
			if (AAcceptedCandidates.size() < 2 || AOriginalItemCount <= 0) {
				return 0;
			}

			TPairCandidateLookup PairCandidateLookup;
			BuildPairCandidateLookup(AAllCandidates, AOriginalItemCount, PairCandidateLookup);
			if (PairCandidateLookup.empty()) {
				return 0;
			}

			const std::vector<TetClusterCandidate> GreedyResult = AAcceptedCandidates;
			const double GreedyScore = _CalculateCandidateSelectionScore(GreedyResult);
			if (!_ValidateClusterSelection(GreedyResult, AOriginalItemCount)) {
				return 0;
			}

			int SwapCount = 0;
			for (int Round = 0; Round < kMaxSwapRounds; ++Round) {
				const std::vector<std::size_t> PairPositions = CollectPairCandidatePositions(AAcceptedCandidates);
				if (PairPositions.size() < 2) {
					break;
				}

				bool Changed = false;
				for (std::size_t FirstPairIndex = 0; FirstPairIndex + 1 < PairPositions.size(); ++FirstPairIndex) {
					for (std::size_t SecondPairIndex = FirstPairIndex + 1; SecondPairIndex < PairPositions.size(); ++SecondPairIndex) {
						const std::size_t FirstPosition = PairPositions[FirstPairIndex];
						const std::size_t SecondPosition = PairPositions[SecondPairIndex];
						const TetClusterCandidate* FirstReplacement = nullptr;
						const TetClusterCandidate* SecondReplacement = nullptr;
						if (!TryFindBetterPairSwap(PairCandidateLookup, AAcceptedCandidates[FirstPosition], AAcceptedCandidates[SecondPosition], FirstReplacement, SecondReplacement)) {
							continue;
						}
						std::vector<TetClusterCandidate> TrialSelection = AAcceptedCandidates;
						TrialSelection[FirstPosition] = *FirstReplacement;
						TrialSelection[SecondPosition] = *SecondReplacement;
						if (!_ValidateClusterSelection(TrialSelection, AOriginalItemCount)) {
							continue;
						}
						AAcceptedCandidates = std::move(TrialSelection);
						++SwapCount;
						Changed = true;
					}
				}
				if (!Changed) {
					break;
				}
			}

			const bool SelectionValid = _ValidateClusterSelection(AAcceptedCandidates, AOriginalItemCount);
			const double OptimizedScore = _CalculateCandidateSelectionScore(AAcceptedCandidates);
			if (!SelectionValid) {
				std::cout << "[ClusterSelection][ROLLBACK] Invalid pair-swap selection." << std::endl;
				AAcceptedCandidates = GreedyResult;
				return 0;
			}
			const double GainRatio = (OptimizedScore - GreedyScore) / std::max(std::abs(GreedyScore), 1.0);
			if (SwapCount > 0 && GainRatio < kMinSwapGainRatio) {
				AAcceptedCandidates = GreedyResult;
				return 0;
			}

			return SwapCount;
		}

		double CetClusterManager::_CalculateCandidateSelectionScore(const std::vector<TetClusterCandidate>& ACandidates)
		{
			double TotalScore = 0.0;
			for (const TetClusterCandidate& Candidate : ACandidates) {
				TotalScore += Candidate.Score;
			}
			return TotalScore;
		}

		bool CetClusterManager::_ValidateClusterSelection(const std::vector<TetClusterCandidate>& ACandidates, int AOriginalItemCount)
		{
			if (AOriginalItemCount < 0) {
				return false;
			}
			std::vector<bool> Used(static_cast<std::size_t>(AOriginalItemCount), false);
			for (const TetClusterCandidate& Candidate : ACandidates) {
				if (!Candidate.Valid || Candidate.OriginalIndices.empty() || Candidate.OriginalIndices.size() != Candidate.Transforms.size() || !std::isfinite(Candidate.Score)) {
					return false;
				}
				std::set<int> CandidateIds;
				std::set<int> TransformIds;
				for (int OriginalIndex : Candidate.OriginalIndices) {
					if (OriginalIndex < 0 || OriginalIndex >= AOriginalItemCount || Used[OriginalIndex] || !CandidateIds.insert(OriginalIndex).second) {
						return false;
					}
					Used[OriginalIndex] = true;
				}
				for (const TetItemTransform& Transform : Candidate.Transforms) {
					if (Transform.OriginalId < 0 || Transform.OriginalId >= AOriginalItemCount || !std::isfinite(Transform.RelativeX) || !std::isfinite(Transform.RelativeY) || !std::isfinite(Transform.RelativeRotation) || !TransformIds.insert(Transform.OriginalId).second) {
						return false;
					}
				}
				if (CandidateIds != TransformIds) {
					return false;
				}
			}
			return true;
		}

		TetClusterBuildResult CetClusterManager::_BuildAutoPairClusters(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions)
		{
			TetClusterBuildResult Result;
			Result.NestItems.reserve(AOriginalItems.size());
			Result.MetaItems.reserve(AOriginalItems.size());
			const int Count = static_cast<int>(AOriginalItems.size());
			std::vector<bool> Used(Count, false);
			std::vector<TetAutoPairCandidate> AllCandidates;
			std::vector<bool> WorthAutoPair(Count, false);
			const long long TotalPairs = static_cast<long long>(Count) * static_cast<long long>(Count - 1) / 2;
			long long CheckedPairs = 0;

			for (int i = 0; i < Count; ++i) {
				for (int j = i + 1; j < Count; ++j) {

					if (!WorthAutoPair[i] && !WorthAutoPair[j]) {
						continue;
					}
					TetAutoPairCandidate Candidate;
					if (_TryFindBestAutoPairCandidate(AOriginalItems, i, j, AOptions, Candidate)) {
						if (Candidate.Valid) {
							AllCandidates.push_back(std::move(Candidate));
						}
						if (CheckedPairs == 1 || CheckedPairs % 100 == 0 || CheckedPairs == TotalPairs) {
							const double Percent = TotalPairs > 0 ? 100.0 * static_cast<double>(CheckedPairs) / static_cast<double>(TotalPairs) : 100.0;
							std::cout << "[AUTO_PAIR][PROGRESS] " << CheckedPairs << " / " << TotalPairs << " (" << Percent << "%)" << std::endl;
						}
					}
				}
			}
			std::cout << "[AUTO_PAIR][SEARCH DONE] CheckedPairs = " << CheckedPairs << ", CandidateCount = " << AllCandidates.size() << std::endl;

			std::sort(AllCandidates.begin(), AllCandidates.end(), [](const TetAutoPairCandidate& A, const TetAutoPairCandidate& AB) { return A.Score > AB.Score; });
			std::cout << "[AUTO_PAIR][GLOBAL] CandidateCount = " << AllCandidates.size() << std::endl;

			for (const auto& Candidate : AllCandidates) {
				if (!Candidate.Valid) {
					continue;
				}
				if (Used[Candidate.AIndex] || Used[Candidate.BIndex]) {
					continue;
				}
				_AddAutoPairCluster(AOriginalItems, AOptions, Candidate, Result);
				Used[Candidate.AIndex] = true;
				Used[Candidate.BIndex] = true;
				std::cout << "[AUTO_PAIR][GLOBAL ACCEPT] " << Candidate.AIndex << " + " << Candidate.BIndex << ", Score = " << Candidate.Score << ", ClusterW = " << Candidate.ClusterW << ", ClusterH = " << Candidate.ClusterH << std::endl;
			}

			for (int i = 0; i < Count; ++i) {
				if (!Used[i]) {
					_AddSingleItem(AOriginalItems, i, Result);
					Used[i] = true;
				}
			}
			return Result;
		}

		bool CetClusterManager::_TryFindBestEdgePairCandidate(const CetTNestItemVector& AOriginalItems, int AIndex, int ABIndex, const TetNestOptions& AOptions, TetAutoPairCandidate& ABestCandidate)
		{
			if (AIndex < 0 || ABIndex < 0 || AIndex >= static_cast<int>(AOriginalItems.size()) || ABIndex >= static_cast<int>(AOriginalItems.size()) || AIndex == ABIndex) {
				return false;
			}
			CetClusterGeometryHelper Geometry;
			const ClipperLib::Path ContourA = Geometry.GetIdentityContour(AOriginalItems[AIndex]);
			const ClipperLib::Path ContourB = Geometry.GetIdentityContour(AOriginalItems[ABIndex]);
			const std::vector<TetEdgeInfo> EdgesA = _CollectEdges(ContourA);
			const std::vector<TetEdgeInfo> EdgesB = _CollectEdges(ContourB);
			if (EdgesA.empty() || EdgesB.empty()) return false;
			double SpacingCoord = static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing));
			TetEdgePairContext ctx = { AOriginalItems, AIndex, ABIndex, AOptions, std::max(0.0, SpacingCoord) + std::max(CET_CLUSTER_MIN_SAFETY_GAP, SpacingCoord * 0.001), std::max(1.0, std::min(std::max(_GetItemWidth(AOriginalItems[AIndex]), _GetItemHeight(AOriginalItems[AIndex])), std::max(_GetItemWidth(AOriginalItems[ABIndex]), _GetItemHeight(AOriginalItems[ABIndex])))), _IsSimilarTriangleByEdges(EdgesA, EdgesB) };
			bool Found = false;
			for (const TetEdgeInfo& EdgeA : EdgesA) {
				for (const TetEdgeInfo& EdgeB : EdgesB) {
					if (_EvaluateEdgePair(ctx, EdgeA, EdgeB, ABestCandidate)) {
						Found = true;
					}
				}
			}
			return Found;
		}

		bool CetClusterManager::_TryFindBestAutoPairCandidate(const CetTNestItemVector& AOriginalItems, int AIndex, int ABIndex, const TetNestOptions& AOptions, TetAutoPairCandidate& ABestCandidate)
		{
			if (AIndex < 0 || ABIndex < 0 || AIndex >= static_cast<int>(AOriginalItems.size()) || ABIndex >= static_cast<int>(AOriginalItems.size())) {
				return false;
			}
			if (_GetItemWidth(AOriginalItems[AIndex]) <= 0.0 || _GetItemHeight(AOriginalItems[AIndex]) <= 0.0 || _GetItemWidth(AOriginalItems[ABIndex]) <= 0.0 || _GetItemHeight(AOriginalItems[ABIndex]) <= 0.0) {
				return false;
			}

			if (_TryFindBestEdgePairCandidate(AOriginalItems, AIndex, ABIndex, AOptions, ABestCandidate)) {
				return true;
			}
			std::vector<double> Rotations = CetRotationUtils::BuildAllowedRotations(AOptions.Rotations);
			TetAutoPairContext ctx = { AOriginalItems, AIndex, ABIndex, AOptions };
			return _RunGridSearchAllAngles(ctx, Rotations, ABestCandidate);
		}

		bool CetClusterManager::_TryBuildAutoPairAt(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const TetAutoPairBuildInput& AInput, TetAutoPairCandidate& ACandidate)
		{
			using NestItemType = CetTNestItemVector::value_type;
			if (AInput.AIndex < 0 || AInput.BIndex < 0 || AInput.AIndex >= static_cast<int>(AOriginalItems.size()) || AInput.BIndex >= static_cast<int>(AOriginalItems.size())) {
				return false;
			}
			const auto& AItem = AOriginalItems[AInput.AIndex];
			const auto& BItem = AOriginalItems[AInput.BIndex];
			CetNestItem A = AItem;
			CetNestItem B = BItem;
			A.translation(libnest2d::Point(0, 0));
			A.rotation(libnest2d::Radians(AInput.ARotation));
			A.inflation(0);
			const ClipperLib::cInt QuantizedOffsetX = static_cast<ClipperLib::cInt>(std::llround(AInput.BOffsetX));
			const ClipperLib::cInt QuantizedOffsetY = static_cast<ClipperLib::cInt>(std::llround(AInput.BOffsetY));
			B.translation(libnest2d::Point(QuantizedOffsetX, QuantizedOffsetY));
			B.rotation(libnest2d::Radians(AInput.BRotation));
			B.inflation(0);
			const double SpacingCoord = static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing));
			if (SpacingCoord > 0.0) {
				const auto OldInflation = A.inflation();
				A.inflation(static_cast<decltype(OldInflation)>(SpacingCoord));
				if (NestItemType::intersects(A, B)) {
					return false;
				}
				A.inflation(OldInflation);
			}
			else if (NestItemType::intersects(A, B)) {
				return false;
			}
			const auto BBA = A.boundingBox();
			const auto BBB = B.boundingBox();
			const double AMinX = static_cast<double>(getX(BBA.minCorner()));
			const double AMinY = static_cast<double>(getY(BBA.minCorner()));
			const double AMaxX = static_cast<double>(getX(BBA.maxCorner()));
			const double AMaxY = static_cast<double>(getY(BBA.maxCorner()));
			const double BMinX = static_cast<double>(getX(BBB.minCorner()));
			const double BMinY = static_cast<double>(getY(BBB.minCorner()));
			const double BMaxX = static_cast<double>(getX(BBB.maxCorner()));
			const double BMaxY = static_cast<double>(getY(BBB.maxCorner()));
			const double MinX = std::min(AMinX, BMinX);
			const double MinY = std::min(AMinY, BMinY);
			const double MaxX = std::max(AMaxX, BMaxX);
			const double MaxY = std::max(AMaxY, BMaxY);
			const double ClusterW = MaxX - MinX;
			const double ClusterH = MaxY - MinY;
			if (ClusterW <= 0.0 || ClusterH <= 0.0) {
				return false;
			}
			const double BinW = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth));
			const double BinH = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinHeight));
			const bool FitsNormally = ClusterW <= BinW && ClusterH <= BinH;
			const bool QuarterTurnAllowed = CetRotationUtils::IsAllowedRotation(CET_CLUSTER_HALF_PI, AOptions.Rotations, 1e-9);
			const bool FitsAfter90DegreeRotation = QuarterTurnAllowed && ClusterH <= BinW && ClusterW <= BinH;
			if (!FitsNormally && !FitsAfter90DegreeRotation) {
				return false;
			}
			const double RotatedBBoxAreaA = std::abs((AMaxX - AMinX) * (AMaxY - AMinY));
			const double RotatedBBoxAreaB = std::abs((BMaxX - BMinX) * (BMaxY - BMinY));
			const double BeforeBBoxArea = RotatedBBoxAreaA + RotatedBBoxAreaB;
			const double AfterBBoxArea = ClusterW * ClusterH;
			if (BeforeBBoxArea <= 0.0 || AfterBBoxArea <= 0.0) {
				return false;
			}
			const double SaveArea = BeforeBBoxArea - AfterBBoxArea;
			const double SaveRatio = SaveArea / BeforeBBoxArea;
			if (SaveRatio < 0.03) {
				return false;
			}
			const double RealArea = std::abs(static_cast<double>(AItem.area())) + std::abs(static_cast<double>(BItem.area()));
			const double Score = _CalcAutoPairScore(BeforeBBoxArea, AfterBBoxArea, RealArea, ClusterW, ClusterH);
			ACandidate.Valid = true;
			ACandidate.AIndex = AInput.AIndex;
			ACandidate.BIndex = AInput.BIndex;
			ACandidate.RelAX = -MinX;
			ACandidate.RelAY = -MinY;
			ACandidate.RelARotation = AInput.ARotation;
			ACandidate.RelBX = static_cast<double>(QuantizedOffsetX) - MinX;
			ACandidate.RelBY = static_cast<double>(QuantizedOffsetY) - MinY;
			ACandidate.RelBRotation = AInput.BRotation;
			ACandidate.RawBOffsetX = static_cast<double>(QuantizedOffsetX);
			ACandidate.RawBOffsetY = static_cast<double>(QuantizedOffsetY);
			ACandidate.ClusterW = ClusterW;
			ACandidate.ClusterH = ClusterH;
			ACandidate.Score = Score;
			return true;
		}

		void CetClusterManager::_AddAutoPairCluster(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const TetAutoPairCandidate& ACandidate, TetClusterBuildResult& AResult)
		{
			if (!ACandidate.Valid) {
				return;
			}
			auto ClusterItem = _MakeUnionNestItemFromCandidate(AOriginalItems, AOptions, ACandidate);
			const int PackedIndex = static_cast<int>(AResult.NestItems.size());
			AResult.NestItems.push_back(std::move(ClusterItem));
			TetMetaItem Meta;
			Meta.PackedItemIndex = PackedIndex;
			Meta.IsCluster = true;
			Meta.ClusterType = "AutoPairCluster";
			TetItemTransform TransformA;
			TransformA.OriginalId = ACandidate.AIndex;
			TransformA.RelativeX = ACandidate.RelAX;
			TransformA.RelativeY = ACandidate.RelAY;
			TransformA.RelativeRotation = ACandidate.RelARotation;
			Meta.TransformData.push_back(TransformA);
			TetItemTransform TransformB;
			TransformB.OriginalId = ACandidate.BIndex;
			TransformB.RelativeX = ACandidate.RelBX;
			TransformB.RelativeY = ACandidate.RelBY;
			TransformB.RelativeRotation = ACandidate.RelBRotation;
			Meta.TransformData.push_back(TransformB);
			AResult.MetaItems.push_back(Meta);
		}

		double CetClusterManager::_CalcAutoPairScore(double ABeforeBBoxArea, double AAfterBBoxArea, double ARealArea, double AClusterW, double AClusterH)
		{
			if (ABeforeBBoxArea <= 0.0 || AAfterBBoxArea <= 0.0) {
				return -1.0;
			}
			double SaveArea = ABeforeBBoxArea - AAfterBBoxArea;
			double SaveRatio = SaveArea / ABeforeBBoxArea;
			double FillRatio = 0.0;
			if (AAfterBBoxArea > 0.0) {
				FillRatio = ARealArea / AAfterBBoxArea;
			}

			double Score = SaveRatio * 1000.0 + FillRatio * 100.0 - (AClusterW + AClusterH) * 0.000001;
			return Score;
		}

		bool CetClusterManager::_RunAutoPairGridSearch(const CetTNestItemVector& AOriginalItems, int AIndex, int ABIndex, const TetNestOptions& AOptions, const TetAutoPairGridConfig& AConfig, TetAutoPairCandidate& AOutBest)
		{
			if (AConfig.Step <= 0.0) {
				return false;
			}
			const double AWidth = AConfig.RotAMaxX - AConfig.RotAMinX;
			const double AHeight = AConfig.RotAMaxY - AConfig.RotAMinY;
			const double BWidth = AConfig.RotBMaxX - AConfig.RotBMinX;
			const double BHeight = AConfig.RotBMaxY - AConfig.RotBMinY;
			const double BeforeBBoxArea = AWidth * AHeight + BWidth * BHeight;
			if (BeforeBBoxArea <= 0.0) {
				return false;
			}
			bool Found = false;
			int CheckedCount = 0;
			for (double OffsetY = AConfig.MinOffsetY; OffsetY <= AConfig.MaxOffsetY; OffsetY += AConfig.Step) {
				for (double OffsetX = AConfig.MinOffsetX; OffsetX <= AConfig.MaxOffsetX; OffsetX += AConfig.Step) {
					++CheckedCount;
					if (CheckedCount > AConfig.MaxCheckedCount) {
						return Found;
					}
					const double QuickMinX = std::min(AConfig.RotAMinX, OffsetX + AConfig.RotBMinX);
					const double QuickMinY = std::min(AConfig.RotAMinY, OffsetY + AConfig.RotBMinY);
					const double QuickMaxX = std::max(AConfig.RotAMaxX, OffsetX + AConfig.RotBMaxX);
					const double QuickMaxY = std::max(AConfig.RotAMaxY, OffsetY + AConfig.RotBMaxY);
					const double QuickW = QuickMaxX - QuickMinX;
					const double QuickH = QuickMaxY - QuickMinY;
					if (QuickW <= 0.0 || QuickH <= 0.0) {
						continue;
					}
					const double QuickAfterArea = QuickW * QuickH;
					if (QuickAfterArea >= BeforeBBoxArea * 0.97) {
						continue;
					}
					TetAutoPairBuildInput Input;
					Input.AIndex = AIndex;
					Input.BIndex = ABIndex;
					Input.ARotation = AConfig.ARot;
					Input.BRotation = AConfig.BRot;
					Input.BOffsetX = OffsetX;
					Input.BOffsetY = OffsetY;
					TetAutoPairCandidate Candidate;
					if (!_TryBuildAutoPairAt(AOriginalItems, AOptions, Input, Candidate)) {
						continue;
					}
					if (!Found || Candidate.Score > AOutBest.Score) {
						AOutBest = Candidate;
						Found = true;
					}
				}
			}
			return Found;
		}

		CetNestItem CetClusterManager::_MakeUnionNestItemFromCandidate(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const TetAutoPairCandidate& ACandidate)
		{
			CetClusterGeometryHelper Geometry;
			auto MakeRectangleFallback = [&Geometry, &ACandidate]() {
				CetPath Rectangle = Geometry.MakeRectangleContour(std::ceil(ACandidate.ClusterW), std::ceil(ACandidate.ClusterH));
				return Geometry.MakeNestItemFromProxyContour(Rectangle);
				};

			if (!ACandidate.Valid || ACandidate.AIndex < 0 || ACandidate.BIndex < 0 || ACandidate.AIndex >= static_cast<int>(AOriginalItems.size()) || ACandidate.BIndex >= static_cast<int>(AOriginalItems.size())) {
				return MakeRectangleFallback();
			}

			std::vector<TetItemTransform> Transforms;
			Transforms.reserve(2);
			Transforms.push_back({ ACandidate.AIndex, ACandidate.RelAX, ACandidate.RelAY, ACandidate.RelARotation });
			Transforms.push_back({ ACandidate.BIndex, ACandidate.RelBX, ACandidate.RelBY, ACandidate.RelBRotation });

			CetClusterBoundary BoundaryBuilder;
			TetClusterBoundaryResult BoundaryResult;
			if (!BoundaryBuilder.BuildBoundaryWithResult(AOriginalItems, Transforms, AOptions, BoundaryResult)) {
				std::cout << "[AUTO_PAIR][BOUNDARY][WARN] BuildBoundary failed, fallback to rectangle." << std::endl;
				return MakeRectangleFallback();
			}

			return Geometry.MakeNestItemFromProxyContour(BoundaryResult.Boundary);
		}
		double CetClusterManager::_CalcEdgeLength(const ClipperLib::IntPoint& A, const ClipperLib::IntPoint& AB)
		{
			const double DX = static_cast<double>(AB.X - A.X);
			const double DY = static_cast<double>(AB.Y - A.Y);
			return std::sqrt(DX * DX + DY * DY);
		}

		std::vector<TetEdgeInfo> CetClusterManager::_CollectEdges(const ClipperLib::Path& AContour)
		{
			std::vector<TetEdgeInfo> Result;
			if (AContour.size() < 3) {
				return Result;
			}
			Result.reserve(AContour.size());
			for (std::size_t i = 0; i < AContour.size(); ++i) {
				const auto& Start = AContour[i];
				const auto& End = AContour[(i + 1) % AContour.size()];
				const double Length = _CalcEdgeLength(Start, End);
				if (Length <= 1.0) {
					continue;
				}
				const double DX = static_cast<double>(End.X - Start.X);
				const double DY = static_cast<double>(End.Y - Start.Y);
				TetEdgeInfo Edge;
				Edge.Start = Start;
				Edge.End = End;
				Edge.Length = Length;
				Edge.Angle = std::atan2(DY, DX);
				Result.push_back(Edge);
			}
			std::sort(Result.begin(), Result.end(), [](const TetEdgeInfo& A, const TetEdgeInfo& AB) { return A.Length > AB.Length; });
			constexpr std::size_t MAX_EDGE_COUNT = 24;
			if (Result.size() > MAX_EDGE_COUNT) {
				Result.resize(MAX_EDGE_COUNT);
			}
			return Result;
		}

		bool CetClusterManager::_IsSimilarTriangleByEdges(std::vector<TetEdgeInfo> AEdges, std::vector<TetEdgeInfo> ABEdges)
		{
			if (AEdges.size() != 3 || ABEdges.size() != 3) {
				return false;
			}
			auto LongerFirst = [](const TetEdgeInfo& A, const TetEdgeInfo& AB) { return A.Length > AB.Length; };
			std::sort(AEdges.begin(), AEdges.end(), LongerFirst);
			std::sort(ABEdges.begin(), ABEdges.end(), LongerFirst);
			if (AEdges.front().Length <= 0.0 || ABEdges.front().Length <= 0.0) {
				return false;
			}
			const double Scale = AEdges.front().Length / ABEdges.front().Length;
			constexpr double SHAPE_TOLERANCE = 0.08;
			for (std::size_t i = 0; i < 3; ++i) {
				const double ScaledBLength = ABEdges[i].Length * Scale;
				const double Denominator = std::max(1.0, std::max(AEdges[i].Length, ScaledBLength));
				const double RelativeError = std::abs(AEdges[i].Length - ScaledBLength) / Denominator;
				if (RelativeError > SHAPE_TOLERANCE) {
					return false;
				}
			}
			return true;
		}

		bool CetClusterManager::_SnapToAllowedRotation(double ATarget, int ARotations, double& AOutRotation)
		{
			constexpr double MAX_ANGLE_ERROR = 0.0523598775598299; // 3 degrees
			return CetRotationUtils::SnapToAllowedRotation(ATarget, ARotations, AOutRotation, MAX_ANGLE_ERROR);
		}

		bool CetClusterManager::_EvaluateEdgePair(const TetEdgePairContext& Actx, const TetEdgeInfo& AEdgeA, const TetEdgeInfo& AEdgeB, TetAutoPairCandidate& ABestCandidate)
		{
			constexpr double MIN_EDGE_MATCH_RATIO = 0.80;
			const double MaxLength = std::max(AEdgeA.Length, AEdgeB.Length);
			const double MinLength = std::min(AEdgeA.Length, AEdgeB.Length);
			if (MaxLength <= 0.0 || (MinLength / MaxLength) < MIN_EDGE_MATCH_RATIO) return false;
			const double TargetBRotation = AEdgeA.Angle + CET_CLUSTER_PI - AEdgeB.Angle;
			double BRotation = 0.0;
			if (!_SnapToAllowedRotation(TargetBRotation, Actx.Options.Rotations, BRotation)) return false;
			const double CosR = std::cos(BRotation), SinR = std::sin(BRotation);
			auto RotatePt = [&](const ClipperLib::IntPoint& APt, double& AOutX, double& AOutY) {
				AOutX = static_cast<double>(APt.X) * CosR - static_cast<double>(APt.Y) * SinR;
				AOutY = static_cast<double>(APt.X) * SinR + static_cast<double>(APt.Y) * CosR;
				};
			double RotBStartX = 0.0, RotBStartY = 0.0, RotBEndX = 0.0, RotBEndY = 0.0;
			RotatePt(AEdgeB.Start, RotBStartX, RotBStartY);
			RotatePt(AEdgeB.End, RotBEndX, RotBEndY);
			const double AMidX = (static_cast<double>(AEdgeA.Start.X) + static_cast<double>(AEdgeA.End.X)) * 0.5;
			const double AMidY = (static_cast<double>(AEdgeA.Start.Y) + static_cast<double>(AEdgeA.End.Y)) * 0.5;
			const double BMidX = (RotBStartX + RotBEndX) * 0.5;
			const double BMidY = (RotBStartY + RotBEndY) * 0.5;
			TetEdgeMatchState state;
			state.BRotation = BRotation;
			state.LengthMatchRatio = MinLength / MaxLength;
			state.MinLength = MinLength;
			state.BaseOffsets = {
				{ AMidX - BMidX, AMidY - BMidY },
				{ static_cast<double>(AEdgeA.Start.X) - RotBEndX, static_cast<double>(AEdgeA.Start.Y) - RotBEndY },
				{ static_cast<double>(AEdgeA.End.X) - RotBStartX, static_cast<double>(AEdgeA.End.Y) - RotBStartY }
			};
			return _TestEdgeOffsets(Actx, state, AEdgeA, ABestCandidate);
		}

		bool CetClusterManager::_TestEdgeOffsets(const TetEdgePairContext& Actx, const TetEdgeMatchState& Astate, const TetEdgeInfo& AEdgeA, TetAutoPairCandidate& ABestCandidate)
		{
			const double EdgeDX = static_cast<double>(AEdgeA.End.X - AEdgeA.Start.X);
			const double EdgeDY = static_cast<double>(AEdgeA.End.Y - AEdgeA.Start.Y);
			const double EdgeLength = std::sqrt(EdgeDX * EdgeDX + EdgeDY * EdgeDY);
			if (EdgeLength <= 0.0) return false;
			const double NormalX = -EdgeDY / EdgeLength, NormalY = EdgeDX / EdgeLength;
			bool Found = false;
			for (double Direction : { -1.0, 1.0 }) {
				for (const auto& BaseOffset : Astate.BaseOffsets) {
					TetAutoPairBuildInput Input;
					Input.AIndex = Actx.AIndex;
					Input.BIndex = Actx.BIndex;
					Input.ARotation = 0.0;
					Input.BRotation = Astate.BRotation;
					Input.BOffsetX = BaseOffset.first + NormalX * Actx.RequiredGap * Direction;
					Input.BOffsetY = BaseOffset.second + NormalY * Actx.RequiredGap * Direction;
					TetAutoPairCandidate Candidate;
					if (!_TryBuildAutoPairAt(Actx.OriginalItems, Actx.Options, Input, Candidate)) continue;
					double EdgeCoverage = std::max(0.0, std::min(1.0, Astate.MinLength / Actx.RefLength));
					Candidate.Score += Astate.LengthMatchRatio * 60.0 + EdgeCoverage * 40.0 + (Actx.SimilarTrianglePair ? 50.0 : 0.0);
					if (!Found || Candidate.Score > ABestCandidate.Score) {
						ABestCandidate = Candidate;
						Found = true;
					}
				}
			}
			return Found;
		}

		bool CetClusterManager::_RunGridSearchAllAngles(const TetAutoPairContext& Actx, const std::vector<double>& Arotations, TetAutoPairCandidate& ABestCandidate)
		{
			bool Found = false;
			for (double ARot : Arotations) {
				for (double BRot : Arotations) {
					if (_EvaluateRotationPair(Actx, ARot, BRot, ABestCandidate)) {
						Found = true;
					}
				}
			}
			return Found;
		}

		bool CetClusterManager::_EvaluateRotationPair(const TetAutoPairContext& Actx, double ARot, double ABRot, TetAutoPairCandidate& ABestCandidate)
		{
			auto GetRotatedBBox = [&](const CetNestItem& ASrcItem, double ARotation, double& AOutMinX, double& AOutMinY, double& AOutMaxX, double& AOutMaxY, double& AOutW, double& AOutH) {
				CetNestItem Tmp = ASrcItem;
				Tmp.translation(libnest2d::Point(0, 0));
				Tmp.rotation(libnest2d::Radians(ARotation));
				Tmp.inflation(0);
				const auto BB = Tmp.boundingBox();
				AOutMinX = static_cast<double>(getX(BB.minCorner()));
				AOutMinY = static_cast<double>(getY(BB.minCorner()));
				AOutMaxX = static_cast<double>(getX(BB.maxCorner()));
				AOutMaxY = static_cast<double>(getY(BB.maxCorner()));
				AOutW = AOutMaxX - AOutMinX;
				AOutH = AOutMaxY - AOutMinY;
				};
			double RotAMinX = 0, RotAMinY = 0, RotAMaxX = 0, RotAMaxY = 0, RotWA = 0, RotHA = 0;
			double RotBMinX = 0, RotBMinY = 0, RotBMaxX = 0, RotBMaxY = 0, RotWB = 0, RotHB = 0;
			GetRotatedBBox(Actx.OriginalItems[Actx.AIndex], ARot, RotAMinX, RotAMinY, RotAMaxX, RotAMaxY, RotWA, RotHA);
			GetRotatedBBox(Actx.OriginalItems[Actx.BIndex], ABRot, RotBMinX, RotBMinY, RotBMaxX, RotBMaxY, RotWB, RotHB);
			if (RotWA <= 0.0 || RotHA <= 0.0 || RotWB <= 0.0 || RotHB <= 0.0) return false;
			double BaseSize = std::min(std::min(RotWA, RotHA), std::min(RotWB, RotHB));
			if (BaseSize <= 0.0) return false;
			double SpacingCoord = static_cast<double>(NestUtils::ToNestCoord(Actx.Options.Spacing));
			double CoarseStep = std::max(SpacingCoord > 0.0 ? SpacingCoord : 1.0, BaseSize / 4.0);
			double FineStep = std::max(SpacingCoord > 0.0 ? SpacingCoord / 2.0 : 1.0, BaseSize / 16.0);
			TetAutoPairGridConfig CoarseConfig;
			CoarseConfig.ARot = ARot;
			CoarseConfig.BRot = ABRot;
			CoarseConfig.RotWA = RotWA;
			CoarseConfig.RotHA = RotHA;
			CoarseConfig.RotWB = RotWB;
			CoarseConfig.RotHB = RotHB;
			CoarseConfig.RotAMinX = RotAMinX;
			CoarseConfig.RotAMinY = RotAMinY;
			CoarseConfig.RotAMaxX = RotAMaxX;
			CoarseConfig.RotAMaxY = RotAMaxY;
			CoarseConfig.RotBMinX = RotBMinX;
			CoarseConfig.RotBMinY = RotBMinY;
			CoarseConfig.RotBMaxX = RotBMaxX;
			CoarseConfig.RotBMaxY = RotBMaxY;
			CoarseConfig.MinOffsetX = RotAMinX - RotBMaxX - SpacingCoord;
			CoarseConfig.MaxOffsetX = RotAMaxX - RotBMinX + SpacingCoord;
			CoarseConfig.MinOffsetY = RotAMinY - RotBMaxY - SpacingCoord;
			CoarseConfig.MaxOffsetY = RotAMaxY - RotBMinY + SpacingCoord;
			CoarseConfig.Step = CoarseStep;
			CoarseConfig.MaxCheckedCount = 5000;
			TetAutoPairCandidate CoarseBest;
			if (!_RunAutoPairGridSearch(Actx.OriginalItems, Actx.AIndex, Actx.BIndex, Actx.Options, CoarseConfig, CoarseBest)) return false;
			TetAutoPairGridConfig FineConfig = CoarseConfig;
			FineConfig.MinOffsetX = CoarseBest.RawBOffsetX - CoarseStep;
			FineConfig.MaxOffsetX = CoarseBest.RawBOffsetX + CoarseStep;
			FineConfig.MinOffsetY = CoarseBest.RawBOffsetY - CoarseStep;
			FineConfig.MaxOffsetY = CoarseBest.RawBOffsetY + CoarseStep;
			FineConfig.Step = FineStep;
			FineConfig.MaxCheckedCount = 3000;
			TetAutoPairCandidate FineBest;
			bool FineFound = _RunAutoPairGridSearch(Actx.OriginalItems, Actx.AIndex, Actx.BIndex, Actx.Options, FineConfig, FineBest);
			const TetAutoPairCandidate& CurrentBest = FineFound ? FineBest : CoarseBest;
			if (CurrentBest.Score > ABestCandidate.Score) {
				ABestCandidate = CurrentBest;
				return true;
			}
			return false;
		}

		bool CetClusterManager::_CanAcceptClusterCandidate(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const TetClusterCandidate& ACandidate, const std::vector<bool>& AUsed, int AOriginalCount)
		{
			if (AOriginalCount < 0 || AOriginalItems.size() != static_cast<std::size_t>(AOriginalCount)) {
				return false;
			}
			if (!ACandidate.Valid || ACandidate.OriginalIndices.empty() || ACandidate.OriginalIndices.size() != ACandidate.Transforms.size() || ACandidate.ProxyContour.size() < 3) {
				return false;
			}
			if (ACandidate.ClusterWidth <= 0.0 || ACandidate.ClusterHeight <= 0.0 || ACandidate.ProxyArea <= 0.0) {
				return false;
			}
			std::set<int> CandidateIds;
			std::set<int> TransformIds;
			for (int OriginalIndex : ACandidate.OriginalIndices) {
				if (OriginalIndex < 0 || OriginalIndex >= AOriginalCount || OriginalIndex >= static_cast<int>(AUsed.size()) || AUsed[OriginalIndex]) {
					return false;
				}
				if (!CandidateIds.insert(OriginalIndex).second) {
					return false;
				}
			}

			for (const TetItemTransform& Transform : ACandidate.Transforms) {
				if (Transform.OriginalId < 0 || Transform.OriginalId >= AOriginalCount) {
					return false;
				}
				if (!std::isfinite(Transform.RelativeX) || !std::isfinite(Transform.RelativeY) || !std::isfinite(Transform.RelativeRotation)) {
					return false;
				}
				if (!TransformIds.insert(Transform.OriginalId).second) {
					return false;
				}
			}

			return CandidateIds == TransformIds;
		}
		CetNestItem CetClusterManager::_MakeClusterProxyItem(const TetClusterCandidate& ACandidate)
		{
			CetClusterGeometryHelper Geometry;
			if (ACandidate.ProxyContour.size() >= 3) {
				return Geometry.MakeNestItemFromProxyContour(ACandidate.ProxyContour);
			}
			return Geometry.MakeNestItemFromProxyContour(Geometry.MakeRectangleContour(ACandidate.ClusterWidth, ACandidate.ClusterHeight));
		}

		bool CetClusterManager::_AddClusterCandidate(const TetClusterCandidate& ACandidate, TetClusterBuildResult& AResult)
		{
			if (!ACandidate.Valid || ACandidate.OriginalIndices.empty() || ACandidate.Transforms.empty()) {
				return false;
			}
			CetNestItem ClusterItem = _MakeClusterProxyItem(ACandidate);
			const int PackedIndex = static_cast<int>(AResult.NestItems.size());
			AResult.NestItems.push_back(std::move(ClusterItem));
			TetMetaItem Meta;
			Meta.PackedItemIndex = PackedIndex;
			Meta.IsCluster = true;
			Meta.ClusterType = ACandidate.ClusterType.empty() ? "UnknownTemplateCluster" : ACandidate.ClusterType;
			Meta.TransformData = ACandidate.Transforms;
			AResult.MetaItems.push_back(std::move(Meta));
			std::cout << "[TEMPLATE][CANDIDATE ADD] Builder=" << ACandidate.BuilderName << " Type=" << ACandidate.ClusterType << " ChildCount=" << ACandidate.OriginalIndices.size() << " Width=" << ACandidate.ClusterWidth << " Height=" << ACandidate.ClusterHeight << " FillRatio=" << ACandidate.FillRatio << " BoundingFill=" << ACandidate.BoundingFillRatio << " Reuse=" << ACandidate.SheetReuseScore << " Score=" << ACandidate.Score << " PackedIndex=" << PackedIndex << std::endl;
			return true;
		}

		TetClusterBuildResult CetClusterManager::_BuildAllSingles(const CetTNestItemVector& AOriginalItems)
		{
			TetClusterBuildResult Result;
			Result.NestItems.reserve(AOriginalItems.size());
			Result.MetaItems.reserve(AOriginalItems.size());
			for (int i = 0; i < static_cast<int>(AOriginalItems.size()); ++i) {
				_AddSingleItem(AOriginalItems, i, Result);
			}
			return Result;
		}

		bool CetClusterManager::_ValidateBuildResultCoverage(const TetClusterBuildResult& AResult, int AOriginalCount)
		{
			if (AOriginalCount < 0) {
				std::cout << "[TEMPLATE][COVERAGE ERROR] OriginalCount < 0." << std::endl;
				return false;
			}
			if (AResult.NestItems.size() != AResult.MetaItems.size()) {
				std::cout << "[TEMPLATE][COVERAGE ERROR] NestItems.size != MetaItems.size. NestItems=" << AResult.NestItems.size() << ", MetaItems=" << AResult.MetaItems.size() << std::endl;
				return false;
			}
			std::vector<int> HitCount(static_cast<std::size_t>(AOriginalCount), 0);
			for (std::size_t MetaIndex = 0; MetaIndex < AResult.MetaItems.size(); ++MetaIndex) {
				const TetMetaItem& Meta = AResult.MetaItems[MetaIndex];
				if (Meta.PackedItemIndex < 0 || Meta.PackedItemIndex >= static_cast<int>(AResult.NestItems.size())) {
					std::cout << "[TEMPLATE][COVERAGE ERROR] Invalid PackedItemIndex. MetaIndex=" << MetaIndex << ", PackedItemIndex=" << Meta.PackedItemIndex << ", NestItems.size=" << AResult.NestItems.size() << std::endl;
					return false;
				}
				if (Meta.TransformData.empty()) {
					std::cout << "[TEMPLATE][COVERAGE ERROR] Empty TransformData. MetaIndex=" << MetaIndex << ", ClusterType=" << Meta.ClusterType << std::endl;
					return false;
				}
				for (const TetItemTransform& Transform : Meta.TransformData) {
					const int OriginalId = Transform.OriginalId;
					if (OriginalId < 0 || OriginalId >= AOriginalCount) {
						std::cout << "[TEMPLATE][COVERAGE ERROR] Invalid OriginalId. MetaIndex=" << MetaIndex << ", OriginalId=" << OriginalId << ", OriginalCount=" << AOriginalCount << std::endl;
						return false;
					}
					++HitCount[OriginalId];
				}
			}
			for (int OriginalId = 0; OriginalId < AOriginalCount; ++OriginalId) {
				if (HitCount[OriginalId] != 1) {
					std::cout << "[TEMPLATE][COVERAGE ERROR] Original item coverage invalid. OriginalId=" << OriginalId << ", HitCount=" << HitCount[OriginalId] << std::endl;
					return false;
				}
			}
			return true;
		}

	}
}
