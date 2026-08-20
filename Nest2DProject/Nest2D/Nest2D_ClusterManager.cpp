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
#include <cstdint>
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
	std::size_t MinDepthBeforeTimeout = 0;
	long long MaxElapsedMs = 0;
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
	std::size_t EnvelopeTimeLimitHits = 0;
	std::size_t EnvelopeMaxDepthReached = 0;
	std::size_t EnvelopeBestFillerCount = 0;
	double EnvelopeSearchMs = 0.0;
	double EnvelopeTrueContourMs = 0.0;
	double BaseFillRatioSum = 0.0;
	double FilledFillRatioSum = 0.0;
	double BestFillRatioGain = 0.0;
	double BestEnvelopeFillRatioGain = 0.0;
	double BestEnvelopeRectangleFillRatio = 0.0;
};

TetClusterFillSearchConfig GetClusterFillSearchConfig(std::size_t AItemCount)
{
	if (AItemCount > CET_NEST_REDUCED_STRATEGY_ITEM_LIMIT) {
		return { CET_CLUSTER_FILL_REDUCED_ORDER_BEAM_WIDTH, std::max<std::size_t>(1, AItemCount), std::max<std::size_t>(1, AItemCount), 0 };
	}
	if (AItemCount > CET_NEST_FULL_STRATEGY_ITEM_LIMIT) {
		return { CET_CLUSTER_FILL_REDUCED_ORDER_BEAM_WIDTH, std::max<std::size_t>(1, AItemCount), std::max<std::size_t>(1, AItemCount), 0 };
	}
	return {};
}

TetClusterFillSearchConfig GetClusterEnvelopeFillSearchConfig(std::size_t AItemCount)
{
	const std::size_t InventoryDepth = std::max<std::size_t>(1, AItemCount);
	if (AItemCount > CET_NEST_FULL_STRATEGY_ITEM_LIMIT) {
		return {
			CET_CLUSTER_ENVELOPE_FILL_LARGE_ORDER_BEAM_WIDTH,
			std::min(InventoryDepth, CET_CLUSTER_ENVELOPE_FILL_LARGE_ORDER_MAX_DEPTH),
			CET_CLUSTER_ENVELOPE_FILL_LARGE_ORDER_MAX_CANDIDATE_FILLERS,
			CET_CLUSTER_ENVELOPE_FILL_LARGE_ORDER_MAX_PLACEMENT_ATTEMPTS,
			CET_CLUSTER_ENVELOPE_FILL_LARGE_ORDER_MIN_DEPTH_BEFORE_TIMEOUT,
			CET_CLUSTER_ENVELOPE_FILL_LARGE_ORDER_MAX_SEARCH_TIME_MS
		};
	}
	return {
		CET_CLUSTER_ENVELOPE_FILL_BEAM_WIDTH,
		std::min(InventoryDepth, CET_CLUSTER_ENVELOPE_FILL_MAX_DEPTH),
		CET_CLUSTER_ENVELOPE_FILL_MAX_CANDIDATE_FILLERS,
		CET_CLUSTER_ENVELOPE_FILL_MAX_PLACEMENT_ATTEMPTS,
		CET_CLUSTER_ENVELOPE_FILL_MIN_DEPTH_BEFORE_TIMEOUT,
		CET_CLUSTER_ENVELOPE_FILL_MAX_SEARCH_TIME_MS
	};
}

bool ContainsOriginalIndex(const TetClusterCandidate& ACandidate, int AOriginalIndex)
{
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

bool SkipsGenericTemplateFill(const TetClusterCandidate& ACandidate) {
	return ACandidate.BuilderName == "CircleBuilder"
		|| ACandidate.BuilderName == "EllipseBuilder"
		|| ACandidate.BuilderName == "ArcBuilder";
}

bool IsFixedCircleEnvelopeBase(const TetClusterCandidate& ACandidate,
	const std::vector<TetShapeFeature>& AFeatures) {
	if (!ACandidate.Valid || ACandidate.BuilderName != "CircleBuilder"
		|| ACandidate.OriginalIndices.size() < 2) return false;
	for (int Index : ACandidate.OriginalIndices) {
		if (Index < 0 || Index >= static_cast<int>(AFeatures.size())
			|| AFeatures[Index].ShapeType != MetShapeType::CircleLike) return false;
	}
	return true;
}

bool IsFixedEllipseEnvelopeBase(const TetClusterCandidate& ACandidate,
	const std::vector<TetShapeFeature>& AFeatures) {
	if (!ACandidate.Valid || ACandidate.BuilderName != "EllipseBuilder"
		|| ACandidate.OriginalIndices.size() < 2) return false;
	for (int Index : ACandidate.OriginalIndices) {
		if (Index < 0 || Index >= static_cast<int>(AFeatures.size())
			|| AFeatures[Index].ShapeType != MetShapeType::EllipseLike) return false;
	}
	return true;
}

bool IsCompletedEnvelopeFill(const TetClusterCandidate& ACandidate) {
	return ACandidate.BuilderName == "EnvelopeFillSearch"
		&& ACandidate.ProxyMode != MetClusterProxyMode::Unknown
		&& ACandidate.OriginalIndices.size() >= 3;
}

bool IsCircleSkeletonCandidate(const TetClusterCandidate& ACandidate) {
	return ACandidate.SkeletonChildCount >= 2
		&& (ACandidate.BuilderName == "CircleBuilder"
			|| ACandidate.ClusterType.find("Circle") == 0);
}

bool IsEllipseSkeletonCandidate(const TetClusterCandidate& ACandidate) {
	return ACandidate.SkeletonChildCount >= 2
		&& (ACandidate.BuilderName == "EllipseBuilder"
			|| ACandidate.ClusterType.find("Ellipse") == 0);
}

bool IsInventorySkeletonCandidate(const TetClusterCandidate& ACandidate) {
	return IsCircleSkeletonCandidate(ACandidate) || IsEllipseSkeletonCandidate(ACandidate);
}

bool IsCompletedEnvelopeFillBetter(const TetClusterCandidate& AFirst,
	const TetClusterCandidate& ASecond) {
	// A circle framework is the virtual board for its fillers.  Keep the
	// largest complete framework intact before comparing local filler gains;
	// otherwise an 8-circle frame with one extra small part can consume the
	// inventory ahead of a 12-circle frame and fragment the intended layout.
	if (AFirst.SkeletonChildCount != ASecond.SkeletonChildCount) {
		return AFirst.SkeletonChildCount > ASecond.SkeletonChildCount;
	}
	const std::size_t FirstFillers = AFirst.OriginalIndices.size() - AFirst.SkeletonChildCount;
	const std::size_t SecondFillers = ASecond.OriginalIndices.size() - ASecond.SkeletonChildCount;
	if (FirstFillers != SecondFillers) return FirstFillers > SecondFillers;
	if (std::abs(AFirst.FillRatio - ASecond.FillRatio) > 1e-9) {
		return AFirst.FillRatio > ASecond.FillRatio;
	}
	if (std::abs(AFirst.FragmentationRisk - ASecond.FragmentationRisk) > 1e-9) {
		return AFirst.FragmentationRisk < ASecond.FragmentationRisk;
	}
	if (std::abs(AFirst.SheetReuseScore - ASecond.SheetReuseScore) > 1e-9) {
		return AFirst.SheetReuseScore > ASecond.SheetReuseScore;
	}
	if (std::abs(AFirst.Score - ASecond.Score) > 1e-9) return AFirst.Score > ASecond.Score;
	return AFirst.ProxyArea < ASecond.ProxyArea;
}

std::string BuildCircleGapTemplateCacheKey(const TetClusterCandidate& ABaseCandidate) {
	std::ostringstream Stream;
	Stream << ABaseCandidate.BuilderName << '|' << ABaseCandidate.ClusterType << '|'
		<< ABaseCandidate.SkeletonChildCount << '|'
		<< std::llround(ABaseCandidate.ClusterWidth / CET_RECTANGLE_FILL_POSITION_TOLERANCE) << '|'
		<< std::llround(ABaseCandidate.ClusterHeight / CET_RECTANGLE_FILL_POSITION_TOLERANCE);
	return Stream.str();
}

bool BuildRectangleEnvelopeCandidate(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const TetClusterCandidate& ABaseCandidate, TetClusterCandidate& AOutEnvelopeCandidate)
{
	AOutEnvelopeCandidate = TetClusterCandidate{};
	if (!ABaseCandidate.Valid || ABaseCandidate.OriginalIndices.size() < 2 || HasFullRectangleProxy(ABaseCandidate)) {
		return false;
	}
	AOutEnvelopeCandidate = ABaseCandidate;
	ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
	if (!Geometry.FinalizeCandidateInRectangle(AOriginalItems, AOptions, AOutEnvelopeCandidate,
		ABaseCandidate.ClusterWidth, ABaseCandidate.ClusterHeight)) return false;
	AOutEnvelopeCandidate.BuilderName = ABaseCandidate.BuilderName;
	AOutEnvelopeCandidate.ClusterType = ABaseCandidate.ClusterType;
	return AOutEnvelopeCandidate.ProxyArea > ABaseCandidate.ProxyArea + std::max(1.0,
		ABaseCandidate.ProxyArea * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE);
}

bool IsEnvelopeFillStateWorthExpanding(const TetClusterCandidate& AEnvelopeCandidate,
	const TetClusterCandidate& ACandidate) {
	const double EnvelopeTolerance = std::max(1.0, AEnvelopeCandidate.ProxyArea * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE);
	return ACandidate.Valid
		&& std::abs(ACandidate.ProxyArea - AEnvelopeCandidate.ProxyArea) <= EnvelopeTolerance
		&& ACandidate.ProxyWasteArea < AEnvelopeCandidate.ProxyWasteArea - EnvelopeTolerance;
}

bool PreservesBaseTransforms(const TetClusterCandidate& ABaseCandidate,
	const TetClusterCandidate& ACandidate) {
	if (ABaseCandidate.Transforms.size() > ACandidate.Transforms.size()) return false;
	for (std::size_t Index = 0; Index < ABaseCandidate.Transforms.size(); ++Index) {
		const TetItemTransform& Base = ABaseCandidate.Transforms[Index];
		const TetItemTransform& Current = ACandidate.Transforms[Index];
		if (Base.OriginalId != Current.OriginalId
			|| std::abs(Base.RelativeX - Current.RelativeX) > CET_RECTANGLE_FILL_POSITION_TOLERANCE
			|| std::abs(Base.RelativeY - Current.RelativeY) > CET_RECTANGLE_FILL_POSITION_TOLERANCE
			|| std::abs(Base.RelativeRotation - Current.RelativeRotation) > CET_CLUSTER_FILL_VARIANT_ROTATION_TOLERANCE) return false;
	}
	return true;
}

bool RebuildEnvelopeFillWithTrueContour(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions,
	const TetClusterCandidate& ABaseCandidate, const TetClusterCandidate& AEnvelopeFilledCandidate,
	TetClusterCandidate& AOutCandidate) {
	AOutCandidate = TetClusterCandidate{};
	if (!AEnvelopeFilledCandidate.Valid
		|| AEnvelopeFilledCandidate.OriginalIndices.size() <= ABaseCandidate.OriginalIndices.size()) return false;
	AOutCandidate = AEnvelopeFilledCandidate;
	ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
	if (!Geometry.FinalizeCandidate(AOriginalItems, AOptions, AOutCandidate)) return false;
	// The rectangle envelope is the selected main frame. Rebuilding validates
	// child geometry, then restores that immutable frame rather than letting a
	// filler contour redefine the proxy used by the board-level nesting phase.
	AOutCandidate.ProxyContour = AEnvelopeFilledCandidate.ProxyContour;
	AOutCandidate.ProxyContourNormalized = AEnvelopeFilledCandidate.ProxyContourNormalized;
	AOutCandidate.ProxyArea = AEnvelopeFilledCandidate.ProxyArea;
	AOutCandidate.ProxyMode = AEnvelopeFilledCandidate.ProxyMode;
	AOutCandidate.ClusterWidth = AEnvelopeFilledCandidate.ClusterWidth;
	AOutCandidate.ClusterHeight = AEnvelopeFilledCandidate.ClusterHeight;
	AOutCandidate.BoundingBoxArea = AEnvelopeFilledCandidate.BoundingBoxArea;
	AOutCandidate.ReservedArea = std::min(AOutCandidate.ProxyArea, AOutCandidate.RealArea);
	AOutCandidate.ProxyWasteArea = std::max(0.0, AOutCandidate.ProxyArea - AOutCandidate.ReservedArea);
	AOutCandidate.ProxyWasteRatio = AOutCandidate.ProxyArea > 0.0
		? AOutCandidate.ProxyWasteArea / AOutCandidate.ProxyArea : 1.0;
	AOutCandidate.FillRatio = AOutCandidate.ProxyArea > 0.0
		? std::clamp(AOutCandidate.RealArea / AOutCandidate.ProxyArea, 0.0, 1.0) : 0.0;
	AOutCandidate.BoundingFillRatio = AOutCandidate.FillRatio;
	const std::size_t FillerCount = AOutCandidate.OriginalIndices.size() - ABaseCandidate.OriginalIndices.size();
	const double TrueDensityGain = AOutCandidate.FillRatio - ABaseCandidate.FillRatio;
	AOutCandidate.BuilderName = "EnvelopeFillSearch";
	AOutCandidate.ClusterType = ABaseCandidate.ClusterType + "_EnvelopeFill";
	AOutCandidate.Score = std::max(AOutCandidate.Score, ABaseCandidate.Score
		+ static_cast<double>(FillerCount) * CET_CLUSTER_ENVELOPE_FILL_CHILD_SCORE
		+ TrueDensityGain * CET_CLUSTER_ENVELOPE_FILL_TRUE_DENSITY_SCORE);
	return true;
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

bool IsEnvelopeStateBetter(const TetClusterFillSearchState& AFirst,
	const TetClusterFillSearchState& ASecond) {
	const TetClusterCandidate& First = AFirst.Candidate;
	const TetClusterCandidate& Second = ASecond.Candidate;
	// The envelope dimensions are fixed.  Rank states by the material they
	// actually recover from that envelope, so a locally verified set of small
	// gap fillers is not discarded in favour of a higher temporary probe score.
	if (IsFillMetricLess(First.ProxyWasteArea, Second.ProxyWasteArea)) return true;
	if (IsFillMetricLess(Second.ProxyWasteArea, First.ProxyWasteArea)) return false;
	if (std::abs(First.RealArea - Second.RealArea) > 1.0) return First.RealArea > Second.RealArea;
	if (std::abs(Second.RealArea - First.RealArea) > 1.0) return false;
	if (First.FillRatio > Second.FillRatio + 1e-9) return true;
	if (Second.FillRatio > First.FillRatio + 1e-9) return false;
	if (AFirst.FillerCount != ASecond.FillerCount) return AFirst.FillerCount > ASecond.FillerCount;
	if (std::abs(First.Score - Second.Score) > 1e-9) return First.Score > Second.Score;
	return First.OriginalIndices < Second.OriginalIndices;
}

std::uint64_t MakeFillerFamilyKey(const TetShapeFeature& AFeature);

std::uint64_t GetEnvelopeSeedFamilyKey(const TetClusterFillSearchState& AState,
	const std::vector<TetShapeFeature>& AFeatures, std::size_t ASkeletonChildCount)
{
	if (AState.Candidate.Transforms.size() <= ASkeletonChildCount) return 0;
	const int OriginalId = AState.Candidate.Transforms[ASkeletonChildCount].OriginalId;
	if (OriginalId < 0 || OriginalId >= static_cast<int>(AFeatures.size())) return 0;
	return MakeFillerFamilyKey(AFeatures[OriginalId]);
}

void TrimEnvelopeBeam(std::vector<TetClusterFillSearchState>& AStates, std::size_t AMaxCount,
	const std::vector<TetShapeFeature>& AFeatures, std::size_t ASkeletonChildCount,
	bool APreferFillerCount = false)
{
	std::stable_sort(AStates.begin(), AStates.end(), [&](const TetClusterFillSearchState& AFirst,
		const TetClusterFillSearchState& ASecond) {
		if (APreferFillerCount && AFirst.FillerCount != ASecond.FillerCount) {
			return AFirst.FillerCount > ASecond.FillerCount;
		}
		return IsEnvelopeStateBetter(AFirst, ASecond);
	});
	if (AStates.size() <= AMaxCount) return;
	std::vector<TetClusterFillSearchState> Selected;
	std::set<std::uint64_t> SeedFamilies;
	for (const TetClusterFillSearchState& State : AStates) {
		const std::uint64_t Family = GetEnvelopeSeedFamilyKey(State, AFeatures, ASkeletonChildCount);
		if (Family != 0 && SeedFamilies.insert(Family).second) Selected.push_back(State);
	}
	for (const TetClusterFillSearchState& State : AStates) {
		if (Selected.size() >= AMaxCount) break;
		if (std::find_if(Selected.begin(), Selected.end(), [&](const TetClusterFillSearchState& Existing) {
			return MakeFilledVariantKey(Existing.Candidate) == MakeFilledVariantKey(State.Candidate);
			}) == Selected.end()) Selected.push_back(State);
	}
	if (Selected.size() > AMaxCount) Selected.resize(AMaxCount);
	AStates = std::move(Selected);
}

std::uint64_t MakeFillerFamilyKey(const TetShapeFeature& AFeature) {
	std::uint64_t Hash = 1469598103934665603ULL;
	auto Mix = [&](std::uint64_t AValue) { Hash ^= AValue; Hash *= 1099511628211ULL; };
	Mix(static_cast<std::uint64_t>(AFeature.ShapeType));
	Mix(static_cast<std::uint64_t>(AFeature.HoleCount));
	Mix(static_cast<std::uint64_t>(AFeature.NormalizedContour.size()));
	for (const ClipperLib::IntPoint& Point : AFeature.NormalizedContour) {
		Mix(static_cast<std::uint64_t>(Point.X));
		Mix(static_cast<std::uint64_t>(Point.Y));
	}
	return Hash;
}

void FinalizeEnvelopeFilledStates(const CetTNestItemVector& AOriginalItems,
	const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions,
	const TetClusterCandidate& ABaseCandidate, std::vector<TetClusterFillSearchState>& AStates,
	std::vector<TetClusterCandidate>& AOutVariants, TetClusterFillSearchStats& AStats) {
	DeduplicateFilledStates(AStates);
	TrimEnvelopeBeam(AStates, CET_CLUSTER_ENVELOPE_FILL_MAX_TRUE_CONTOUR_STATES,
		AFeatures, ABaseCandidate.SkeletonChildCount);
	const auto Start = std::chrono::steady_clock::now();
	for (const TetClusterFillSearchState& State : AStates) {
		if (!PreservesBaseTransforms(ABaseCandidate, State.Candidate)) continue;
		TetClusterCandidate TrueContourCandidate;
		if (!RebuildEnvelopeFillWithTrueContour(AOriginalItems, AOptions, ABaseCandidate,
			State.Candidate, TrueContourCandidate)) continue;
		AStats.BestEnvelopeFillRatioGain = std::max(AStats.BestEnvelopeFillRatioGain,
			State.Candidate.FillRatio - ABaseCandidate.BoundingFillRatio);
		AStats.BestEnvelopeRectangleFillRatio = std::max(
			AStats.BestEnvelopeRectangleFillRatio, State.Candidate.FillRatio);
		AStats.EnvelopeBestFillerCount = std::max(AStats.EnvelopeBestFillerCount, State.FillerCount);
		AOutVariants.push_back(std::move(TrueContourCandidate));
	}
	AStats.EnvelopeTrueContourMs += std::chrono::duration<double, std::milli>(
		std::chrono::steady_clock::now() - Start).count();
	AStats.EnvelopeDeduplicatedVariantCount += AOutVariants.size();
}

bool HasValidCandidateInventory(const TetClusterCandidate& ACandidate,
	const std::vector<TetShapeFeature>& AFeatures, const std::vector<bool>& AUsed) {
	return ACandidate.SkeletonChildCount <= ACandidate.Transforms.size()
		&& AFeatures.size() == AUsed.size()
		&& ACandidate.OriginalIndices.size() == ACandidate.Transforms.size();
}

int FindAvailableFamilyItem(const std::vector<TetShapeFeature>& AFeatures,
	const std::vector<bool>& AUsed, const std::set<int>& AReserved, int APrototypeId) {
	if (APrototypeId < 0 || APrototypeId >= static_cast<int>(AFeatures.size())) return -1;
	const std::uint64_t FamilyKey = MakeFillerFamilyKey(AFeatures[APrototypeId]);
	for (int Index = 0; Index < static_cast<int>(AFeatures.size()); ++Index) {
		if (!AUsed[Index] && AReserved.find(Index) == AReserved.end()
			&& MakeFillerFamilyKey(AFeatures[Index]) == FamilyKey) return Index;
	}
	return -1;
}

bool TryBindCandidateInventory(const TetClusterCandidate& ACandidate,
	const std::vector<TetShapeFeature>& AFeatures, const std::vector<bool>& AUsed,
	TetClusterCandidate& AOutCandidate) {
	AOutCandidate = ACandidate;
	if (!HasValidCandidateInventory(ACandidate, AFeatures, AUsed)) return false;
	if (ACandidate.SkeletonChildCount == 0) return true;
	std::set<int> Reserved;
	for (std::size_t Index = 0; Index < ACandidate.SkeletonChildCount; ++Index) {
		const int OriginalId = ACandidate.Transforms[Index].OriginalId;
		if (OriginalId < 0 || OriginalId >= static_cast<int>(AUsed.size())
			|| AUsed[OriginalId] || !Reserved.insert(OriginalId).second) return false;
	}
	for (std::size_t Index = ACandidate.SkeletonChildCount; Index < ACandidate.Transforms.size(); ++Index) {
		const int PrototypeId = ACandidate.Transforms[Index].OriginalId;
		const int BoundId = FindAvailableFamilyItem(AFeatures, AUsed, Reserved, PrototypeId);
		if (BoundId < 0) return false;
		AOutCandidate.Transforms[Index].OriginalId = BoundId;
		Reserved.insert(BoundId);
	}
	AOutCandidate.OriginalIndices.clear();
	AOutCandidate.OriginalIndices.reserve(AOutCandidate.Transforms.size());
	for (const TetItemTransform& Transform : AOutCandidate.Transforms) {
		AOutCandidate.OriginalIndices.push_back(Transform.OriginalId);
	}
	return true;
}

std::vector<TetCircleCenter> CollectCircleCenters(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ACandidate)
{
	std::vector<TetCircleCenter> Centers;
	ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
	for (const TetItemTransform& Transform : ACandidate.Transforms) {
		if (Transform.OriginalId < 0 || Transform.OriginalId >= static_cast<int>(AFeatures.size())
			|| AFeatures[Transform.OriginalId].ShapeType != MetShapeType::CircleLike) continue;
		const CetPath Contour = Geometry.TransformContour(Geometry.GetIdentityContour(AOriginalItems[Transform.OriginalId]),
			Transform.RelativeRotation, Transform.RelativeX, Transform.RelativeY);
		double MinX = 0.0, MinY = 0.0, MaxX = 0.0, MaxY = 0.0;
		if (Geometry.GetBounds(Contour, MinX, MinY, MaxX, MaxY)) {
			Centers.push_back({ (MinX + MaxX) * 0.5, (MinY + MaxY) * 0.5, std::min(MaxX - MinX, MaxY - MinY) * 0.5 });
		}
	}
	return Centers;
}

double FindNearestCircleDistance(const std::vector<TetCircleCenter>& ACenters)
{
	double Nearest = std::numeric_limits<double>::infinity();
	for (std::size_t First = 0; First < ACenters.size(); ++First) {
		for (std::size_t Second = First + 1; Second < ACenters.size(); ++Second) {
			Nearest = std::min(Nearest, std::hypot(ACenters[Second].X - ACenters[First].X,
				ACenters[Second].Y - ACenters[First].Y));
		}
	}
	return Nearest;
}

std::vector<std::vector<std::size_t>> BuildCircleNeighborLists(const std::vector<TetCircleCenter>& ACenters, double ALimit)
{
	std::vector<std::vector<std::pair<double, std::size_t>>> Ranked(ACenters.size());
	for (std::size_t First = 0; First < ACenters.size(); ++First) {
		for (std::size_t Second = First + 1; Second < ACenters.size(); ++Second) {
			const double Distance = std::hypot(ACenters[Second].X - ACenters[First].X,
				ACenters[Second].Y - ACenters[First].Y);
			if (Distance <= CET_RECTANGLE_FILL_POSITION_TOLERANCE || Distance > ALimit) continue;
			Ranked[First].push_back({ Distance, Second });
			Ranked[Second].push_back({ Distance, First });
		}
	}
	std::vector<std::vector<std::size_t>> Neighbors(ACenters.size());
	for (std::size_t Index = 0; Index < Ranked.size(); ++Index) {
		std::stable_sort(Ranked[Index].begin(), Ranked[Index].end());
		if (Ranked[Index].size() > CET_CIRCLE_GAP_MAX_NEIGHBORS) Ranked[Index].resize(CET_CIRCLE_GAP_MAX_NEIGHBORS);
		for (const auto& Entry : Ranked[Index]) Neighbors[Index].push_back(Entry.second);
	}
	return Neighbors;
}

bool AreCircleNeighbors(const std::vector<std::vector<std::size_t>>& ANeighbors, std::size_t AFirst, std::size_t ASecond)
{
	if (AFirst >= ANeighbors.size() || ASecond >= ANeighbors.size()) return false;
	const auto Contains = [&](std::size_t AIndex, std::size_t ATarget) {
		return std::find(ANeighbors[AIndex].begin(), ANeighbors[AIndex].end(), ATarget) != ANeighbors[AIndex].end();
	};
	return Contains(AFirst, ASecond) && Contains(ASecond, AFirst);
}

void AppendPairGapAnchors(const std::vector<TetCircleCenter>& ACenters,
	const std::vector<std::vector<std::size_t>>& ANeighbors, std::vector<TetCircleGapTemplateAnchor>& AAnchors)
{
	for (std::size_t First = 0; First < ANeighbors.size(); ++First) {
		for (std::size_t Second : ANeighbors[First]) {
			if (Second <= First || !AreCircleNeighbors(ANeighbors, First, Second)) continue;
			const double DeltaX = ACenters[Second].X - ACenters[First].X;
			const double DeltaY = ACenters[Second].Y - ACenters[First].Y;
			AAnchors.push_back({ (ACenters[First].X + ACenters[Second].X) * 0.5,
				(ACenters[First].Y + ACenters[Second].Y) * 0.5, std::atan2(DeltaY, DeltaX),
				std::hypot(DeltaX, DeltaY), 2 });
		}
	}
}

bool TryBuildTripleGapAnchor(const TetCircleCenter& AFirst, const TetCircleCenter& ASecond,
	const TetCircleCenter& AThird, double ANeighborLimit, TetCircleGapTemplateAnchor& AAnchor)
{
	const double AB = std::hypot(ASecond.X - AFirst.X, ASecond.Y - AFirst.Y);
	const double AC = std::hypot(AThird.X - AFirst.X, AThird.Y - AFirst.Y);
	const double BC = std::hypot(AThird.X - ASecond.X, AThird.Y - ASecond.Y);
	const double MinSide = std::min({ AB, AC, BC });
	const double MaxSide = std::max({ AB, AC, BC });
	const double Cross = (ASecond.X - AFirst.X) * (AThird.Y - AFirst.Y)
		- (ASecond.Y - AFirst.Y) * (AThird.X - AFirst.X);
	if (MinSide <= CET_RECTANGLE_FILL_POSITION_TOLERANCE || MaxSide > MinSide * 1.15
		|| MaxSide > ANeighborLimit || std::abs(Cross) <= CET_RECTANGLE_FILL_POSITION_TOLERANCE) return false;
	const double FirstSq = AFirst.X * AFirst.X + AFirst.Y * AFirst.Y;
	const double SecondSq = ASecond.X * ASecond.X + ASecond.Y * ASecond.Y;
	const double ThirdSq = AThird.X * AThird.X + AThird.Y * AThird.Y;
	AAnchor.CenterX = (FirstSq * (ASecond.Y - AThird.Y) + SecondSq * (AThird.Y - AFirst.Y)
		+ ThirdSq * (AFirst.Y - ASecond.Y)) / (2.0 * Cross);
	AAnchor.CenterY = (FirstSq * (AThird.X - ASecond.X) + SecondSq * (AFirst.X - AThird.X)
		+ ThirdSq * (ASecond.X - AFirst.X)) / (2.0 * Cross);
	AAnchor.Angle = std::atan2(ASecond.Y - AFirst.Y, ASecond.X - AFirst.X);
	AAnchor.Distance = std::hypot(AAnchor.CenterX - AFirst.X, AAnchor.CenterY - AFirst.Y);
	AAnchor.NeighborCount = 3;
	return true;
}

void AppendTripleGapAnchors(const std::vector<TetCircleCenter>& ACenters,
	const std::vector<std::vector<std::size_t>>& ANeighbors, double ALimit,
	std::vector<TetCircleGapTemplateAnchor>& AAnchors)
{
	for (std::size_t First = 0; First < ANeighbors.size(); ++First) {
		const std::vector<std::size_t>& Local = ANeighbors[First];
		for (std::size_t Left = 0; Left < Local.size(); ++Left) {
			for (std::size_t Right = Left + 1; Right < Local.size(); ++Right) {
				const std::size_t Second = Local[Left], Third = Local[Right];
				if (Second <= First || Third <= First || !AreCircleNeighbors(ANeighbors, Second, Third)) continue;
				TetCircleGapTemplateAnchor Anchor;
				if (TryBuildTripleGapAnchor(ACenters[First], ACenters[Second], ACenters[Third], ALimit, Anchor)) {
					AAnchors.push_back(Anchor);
				}
			}
		}
	}
}

std::vector<TetCircleGapTemplateAnchor> CollectCircleGapTemplateAnchors(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ABaseCandidate)
{
	const std::vector<TetCircleCenter> Centers = CollectCircleCenters(AOriginalItems, AFeatures, ABaseCandidate);
	const double NearestCenterDistance = FindNearestCircleDistance(Centers);
	if (!std::isfinite(NearestCenterDistance)) return {};
	const double NeighborLimit = NearestCenterDistance * 1.15 + CET_RECTANGLE_FILL_POSITION_TOLERANCE;
	const std::vector<std::vector<std::size_t>> Neighbors = BuildCircleNeighborLists(Centers, NeighborLimit);
	std::vector<TetCircleGapTemplateAnchor> Anchors;
	AppendPairGapAnchors(Centers, Neighbors, Anchors);
	AppendTripleGapAnchors(Centers, Neighbors, NeighborLimit, Anchors);
	return Anchors;
}

bool AppendLocalFreeRegion(const ClipperLib::PolyNode& ANode, TetClusterFreeRegion& AOutRegion)
{
	if (ANode.IsHole() || ANode.Contour.size() < 3) return false;
	AOutRegion = TetClusterFreeRegion{};
	AOutRegion.Contour = ANode.Contour;
	AOutRegion.IsClosed = true;
	AOutRegion.Area = std::abs(static_cast<double>(ClipperLib::Area(ANode.Contour)));
	ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
	if (AOutRegion.Area <= 0.0 || !Geometry.GetBounds(AOutRegion.Contour, AOutRegion.MinX, AOutRegion.MinY,
		AOutRegion.MaxX, AOutRegion.MaxY)) return false;
	for (const ClipperLib::PolyNode* Child : ANode.Childs) {
		if (Child == nullptr || !Child->IsHole() || Child->Contour.size() < 3) continue;
		AOutRegion.Holes.push_back(Child->Contour);
		AOutRegion.Area -= std::abs(static_cast<double>(ClipperLib::Area(Child->Contour)));
	}
	AOutRegion.Width = AOutRegion.MaxX - AOutRegion.MinX;
	AOutRegion.Height = AOutRegion.MaxY - AOutRegion.MinY;
	return AOutRegion.Area > 0.0;
}

bool ClipFreeRegionsToGapWindow(const std::vector<TetClusterFreeRegion>& AFreeRegions, const TetCircleGapWindow& AWindow, std::vector<TetClusterFreeRegion>& AOutRegions)
{
	AOutRegions.clear();
	const double MinX = AWindow.CenterX - AWindow.HalfWidth, MinY = AWindow.CenterY - AWindow.HalfHeight;
	const double MaxX = AWindow.CenterX + AWindow.HalfWidth, MaxY = AWindow.CenterY + AWindow.HalfHeight;
	if (MaxX <= MinX || MaxY <= MinY) return false;
	const CetPath Window{ { static_cast<ClipperLib::cInt>(std::llround(MinX)), static_cast<ClipperLib::cInt>(std::llround(MinY)) },
		{ static_cast<ClipperLib::cInt>(std::llround(MaxX)), static_cast<ClipperLib::cInt>(std::llround(MinY)) },
		{ static_cast<ClipperLib::cInt>(std::llround(MaxX)), static_cast<ClipperLib::cInt>(std::llround(MaxY)) },
		{ static_cast<ClipperLib::cInt>(std::llround(MinX)), static_cast<ClipperLib::cInt>(std::llround(MaxY)) } };
	for (const TetClusterFreeRegion& Region : AFreeRegions) {
		ClipperLib::Clipper Clipper;
		if (!Region.IsClosed || !Clipper.AddPath(Region.Contour, ClipperLib::ptSubject, true)
			|| (!Region.Holes.empty() && !Clipper.AddPaths(Region.Holes, ClipperLib::ptSubject, true))
			|| !Clipper.AddPath(Window, ClipperLib::ptClip, true)) continue;
		ClipperLib::PolyTree Tree;
		if (!Clipper.Execute(ClipperLib::ctIntersection, Tree, ClipperLib::pftNonZero, ClipperLib::pftNonZero)) continue;
		for (const ClipperLib::PolyNode* Node : Tree.Childs) {
			TetClusterFreeRegion Local;
			if (Node != nullptr && AppendLocalFreeRegion(*Node, Local)) AOutRegions.push_back(std::move(Local));
		}
	}
	return !AOutRegions.empty();
}

bool BuildEllipseGapRegionSignature(const CetTNestItemVector& AOriginalItems,
	const TetNestOptions& AOptions, const TetClusterCandidate& ACandidate,
	const TetCircleGapWindow& AWindow, std::string& AOutSignature)
{
	AOutSignature.clear();
	ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
	std::vector<TetClusterFreeRegion> FreeRegions, LocalRegions;
	if (!Geometry.ExtractCandidateFreeRegions(AOriginalItems, AOptions, ACandidate, FreeRegions)
		|| !ClipFreeRegionsToGapWindow(FreeRegions, AWindow, LocalRegions)) return false;
	std::stable_sort(LocalRegions.begin(), LocalRegions.end(), [](const TetClusterFreeRegion& A, const TetClusterFreeRegion& B) {
		if (std::abs(A.Area - B.Area) > 1.0) return A.Area > B.Area;
		if (std::abs(A.Width - B.Width) > 1.0) return A.Width > B.Width;
		return A.Height > B.Height;
		});
	std::ostringstream Stream;
	Stream << AWindow.ClassKey;
	for (const TetClusterFreeRegion& Region : LocalRegions) {
		Stream << '|' << std::llround(Region.Area / 1000.0)
			<< ':' << std::llround(Region.Width / 1000.0)
			<< ':' << std::llround(Region.Height / 1000.0)
			<< ':' << Region.Contour.size() << ':' << Region.Holes.size();
	}
	AOutSignature = Stream.str();
	return true;
}

long long QuantizeCircleGapValue(double AValue)
{
	return std::llround(std::max(0.0, AValue) / CET_RECTANGLE_FILL_POSITION_TOLERANCE);
}

std::string BuildCircleGapClassKey(const char* AKind, double AHalfWidth, double AHalfHeight, double AScale)
{
	long long FirstSize = QuantizeCircleGapValue(AHalfWidth);
	long long SecondSize = QuantizeCircleGapValue(AHalfHeight);
	if (SecondSize < FirstSize) std::swap(FirstSize, SecondSize);
	return std::string(AKind) + "-" + std::to_string(FirstSize) + "-"
		+ std::to_string(SecondSize) + "-" + std::to_string(QuantizeCircleGapValue(AScale));
}

int GetCircleGapPriority(const std::string& AClassKey)
{
	if (AClassKey.rfind("quad-", 0) == 0) return 0;
	if (AClassKey.rfind("triple-", 0) == 0) return 1;
	if (AClassKey.rfind("pair-", 0) == 0) return 2;
	return 3;
}

void AppendBoundaryGapWindow(std::vector<TetCircleGapWindow>& AWindows, double ACenterX, double ACenterY,
	double AAngle, double AHalfWidth, double AHalfHeight, double ARadius)
{
	if (AHalfWidth <= CET_RECTANGLE_FILL_POSITION_TOLERANCE
		|| AHalfHeight <= CET_RECTANGLE_FILL_POSITION_TOLERANCE) return;
	AWindows.push_back({ ACenterX, ACenterY, AAngle, AHalfWidth, AHalfHeight,
		BuildCircleGapClassKey("edge", AHalfWidth, AHalfHeight, ARadius) });
}

void AppendBoundaryGapWindows(const std::vector<TetCircleCenter>& ACenters, const TetClusterCandidate& AEnvelope, std::vector<TetCircleGapWindow>& AWindows)
{
	for (const TetCircleCenter& Circle : ACenters) {
		const double Top = Circle.Y - Circle.Radius, Bottom = AEnvelope.ClusterHeight - Circle.Y - Circle.Radius;
		const double Left = Circle.X - Circle.Radius, Right = AEnvelope.ClusterWidth - Circle.X - Circle.Radius;
		AppendBoundaryGapWindow(AWindows, Circle.X, Top * 0.5, -CET_CLUSTER_HALF_PI,
			Circle.Radius, Top * 0.5, Circle.Radius);
		AppendBoundaryGapWindow(AWindows, Circle.X, AEnvelope.ClusterHeight - Bottom * 0.5,
			CET_CLUSTER_HALF_PI, Circle.Radius, Bottom * 0.5, Circle.Radius);
		AppendBoundaryGapWindow(AWindows, Left * 0.5, Circle.Y, CET_CLUSTER_PI,
			Left * 0.5, Circle.Radius, Circle.Radius);
		AppendBoundaryGapWindow(AWindows, AEnvelope.ClusterWidth - Right * 0.5, Circle.Y, 0.0,
			Right * 0.5, Circle.Radius, Circle.Radius);
	}
}

std::vector<TetCircleGapWindow> CollectCircleGapWindows(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ABase, const TetClusterCandidate& AEnvelope)
{
	const std::vector<TetCircleCenter> Centers = CollectCircleCenters(AOriginalItems, AFeatures, ABase);
	const std::vector<TetCircleGapTemplateAnchor> Anchors = CollectCircleGapTemplateAnchors(AOriginalItems, AFeatures, ABase);
	std::vector<TetCircleGapWindow> Windows;
	for (const TetCircleGapTemplateAnchor& Anchor : Anchors) {
		const double Span = std::max(Anchor.Distance, CET_RECTANGLE_FILL_POSITION_TOLERANCE * 4.0);
		if (Anchor.NeighborCount == 3) {
			const double HalfSize = Span * 0.30;
			Windows.push_back({ Anchor.CenterX, Anchor.CenterY, Anchor.Angle, HalfSize, HalfSize,
				BuildCircleGapClassKey("triple", HalfSize, HalfSize, Span) });
			continue;
		}
		const double NX = -std::sin(Anchor.Angle), NY = std::cos(Anchor.Angle);
		const double HalfWidth = Span * 0.60, HalfHeight = Span * 0.46;
		const std::string ClassKey = BuildCircleGapClassKey("pair", HalfWidth, HalfHeight, Span);
		for (int Side : { -1, 1 }) Windows.push_back({ Anchor.CenterX + NX * Span * 0.42 * Side,
			Anchor.CenterY + NY * Span * 0.42 * Side, Anchor.Angle + Side * CET_CLUSTER_HALF_PI,
			HalfWidth, HalfHeight, ClassKey });
	}
	AppendBoundaryGapWindows(Centers, AEnvelope, Windows);
	std::map<std::string, double> AreaByClass;
	for (const TetCircleGapWindow& Window : Windows) {
		AreaByClass[Window.ClassKey] += Window.HalfWidth * Window.HalfHeight * 4.0;
	}
	std::stable_sort(Windows.begin(), Windows.end(), [&](const TetCircleGapWindow& A, const TetCircleGapWindow& B) {
		const int APriority = GetCircleGapPriority(A.ClassKey);
		const int BPriority = GetCircleGapPriority(B.ClassKey);
		if (APriority != BPriority) return APriority < BPriority;
		if (std::abs(AreaByClass[A.ClassKey] - AreaByClass[B.ClassKey]) > 1.0)
			return AreaByClass[A.ClassKey] > AreaByClass[B.ClassKey];
		if (A.ClassKey != B.ClassKey) return A.ClassKey < B.ClassKey;
		if (A.CenterY != B.CenterY) return A.CenterY < B.CenterY;
		return A.CenterX < B.CenterX;
	});
	return Windows;
}

bool TryAppendAlternativeCircleGapFiller(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions, const TetClusterCandidate& ABaseCandidate, const TetClusterCandidate& AEnvelopeCandidate, const TetClusterCandidate& ACurrentCandidate, TetClusterCandidate& AOutCandidate)
{
	AOutCandidate = TetClusterCandidate{};
	ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
	std::vector<TetClusterFreeRegion> FreeRegions;
	if (!Geometry.ExtractCandidateFreeRegions(AOriginalItems, AOptions, ACurrentCandidate, FreeRegions)
		|| FreeRegions.empty()) return false;
	std::vector<int> Alternatives;
	for (int Index = 0; Index < static_cast<int>(AFeatures.size()); ++Index) {
		const TetShapeFeature& Feature = AFeatures[Index];
		if (!ContainsOriginalIndex(ACurrentCandidate, Index) && Feature.Area > 0.0
			&& Feature.Area <= ACurrentCandidate.ProxyWasteArea + 1.0) Alternatives.push_back(Index);
	}
	std::stable_sort(Alternatives.begin(), Alternatives.end(), [&](int A, int B) {
		return AFeatures[A].Area > AFeatures[B].Area;
		});
	ET::NEST2DMANAGERLIB::CetRectangleFillClusterBuilder Builder;
	for (int Index : Alternatives) {
		TetClusterCandidate Candidate;
		if (Builder.TryAppendFillerInRectangleEnvelope(AOriginalItems, AFeatures, ABaseCandidate,
			AEnvelopeCandidate, ACurrentCandidate, FreeRegions, Index, AOptions, Candidate)) {
			AOutCandidate = std::move(Candidate);
			return true;
		}
	}
	return false;
}

bool TryCopyCircleGapTemplate(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions, const TetClusterCandidate& ABaseCandidate, const TetClusterCandidate& AEnvelopeCandidate, const std::vector<TetCircleGapTemplateAnchor>& AAnchors, const TetClusterCandidate& ASeedCandidate, std::size_t AMaxCopies, TetClusterCandidate& AOutCandidate, std::size_t& AOutCopies)
{
	AOutCandidate = ASeedCandidate;
	AOutCopies = 0;
	if (AMaxCopies == 0 || ASeedCandidate.Transforms.size() <= ABaseCandidate.Transforms.size()) return false;
	const TetItemTransform& Prototype = ASeedCandidate.Transforms.back();
	if (Prototype.OriginalId < 0 || Prototype.OriginalId >= static_cast<int>(AFeatures.size())) return false;
	if (AAnchors.size() < 2) return false;
	ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
	const CetPath PrototypeContour = Geometry.TransformContour(Geometry.GetIdentityContour(AOriginalItems[Prototype.OriginalId]), Prototype.RelativeRotation, Prototype.RelativeX, Prototype.RelativeY);
	double MinX = 0.0, MinY = 0.0, MaxX = 0.0, MaxY = 0.0;
	if (!Geometry.GetBounds(PrototypeContour, MinX, MinY, MaxX, MaxY)) return false;
	const double FillerX = (MinX + MaxX) * 0.5, FillerY = (MinY + MaxY) * 0.5;
	auto SourceIt = std::min_element(AAnchors.begin(), AAnchors.end(), [&](const TetCircleGapTemplateAnchor& A, const TetCircleGapTemplateAnchor& B) {
		return std::hypot(A.CenterX - FillerX, A.CenterY - FillerY) < std::hypot(B.CenterX - FillerX, B.CenterY - FillerY);
		});
	if (SourceIt == AAnchors.end()) return false;
	const std::uint64_t Family = MakeFillerFamilyKey(AFeatures[Prototype.OriginalId]);
	const std::vector<double> AllowedRotations = ET::NEST2DMANAGERLIB::CetRotationUtils::BuildAllowedRotations(AOptions.Rotations);
	if (AllowedRotations.empty()) return false;
	const double SourceOffsetX = FillerX - SourceIt->CenterX;
	const double SourceOffsetY = FillerY - SourceIt->CenterY;
	auto NormalizeAngle = [](double AAngle) {
		while (AAngle > CET_CLUSTER_PI) AAngle -= 2.0 * CET_CLUSTER_PI;
		while (AAngle < -CET_CLUSTER_PI) AAngle += 2.0 * CET_CLUSTER_PI;
		return AAngle;
	};
	ET::NEST2DMANAGERLIB::CetRectangleFillClusterBuilder Builder;
	for (const TetCircleGapTemplateAnchor& Target : AAnchors) {
		if (&Target == &(*SourceIt) || AOutCopies >= AMaxCopies || Target.NeighborCount != SourceIt->NeighborCount
			|| std::abs(Target.Distance - SourceIt->Distance) > CET_RECTANGLE_FILL_POSITION_TOLERANCE) continue;
		int CopyIndex = -1;
		for (int Index = 0; Index < static_cast<int>(AFeatures.size()); ++Index) {
			if (!ContainsOriginalIndex(AOutCandidate, Index) && MakeFillerFamilyKey(AFeatures[Index]) == Family) { CopyIndex = Index; break; }
		}
		if (CopyIndex < 0) {
			TetClusterCandidate Alternative;
			if (TryAppendAlternativeCircleGapFiller(AOriginalItems, AFeatures, AOptions, ABaseCandidate,
				AEnvelopeCandidate, AOutCandidate, Alternative)) {
				AOutCandidate = std::move(Alternative);
				++AOutCopies;
				std::cout << "[TEMPLATE][GAP ALTERNATIVE] Template family exhausted, Filler="
					<< AOutCandidate.Transforms.back().OriginalId << std::endl;
			}
			continue;
		}
		const double DesiredRotation = Prototype.RelativeRotation + Target.Angle - SourceIt->Angle;
		std::vector<double> CandidateRotations = AllowedRotations;
		std::stable_sort(CandidateRotations.begin(), CandidateRotations.end(), [&](double A, double B) {
			return std::abs(NormalizeAngle(A - DesiredRotation)) < std::abs(NormalizeAngle(B - DesiredRotation));
			});
		for (double Rotation : CandidateRotations) {
			TetItemTransform Copy;
			Copy.OriginalId = CopyIndex;
			Copy.RelativeRotation = Rotation;
			const CetPath Rotated = Geometry.TransformContour(Geometry.GetIdentityContour(AOriginalItems[CopyIndex]), Rotation, 0.0, 0.0);
			if (!Geometry.GetBounds(Rotated, MinX, MinY, MaxX, MaxY)) continue;
			const double DeltaAngle = Target.Angle - SourceIt->Angle;
			const double Cosine = std::cos(DeltaAngle);
			const double Sine = std::sin(DeltaAngle);
			const double TargetCenterX = Target.CenterX + SourceOffsetX * Cosine - SourceOffsetY * Sine;
			const double TargetCenterY = Target.CenterY + SourceOffsetX * Sine + SourceOffsetY * Cosine;
			Copy.RelativeX = TargetCenterX - (MinX + MaxX) * 0.5;
			Copy.RelativeY = TargetCenterY - (MinY + MaxY) * 0.5;
			TetClusterCandidate Next;
			if (Builder.TryAppendFillerTemplateInRectangleEnvelope(AOriginalItems, AFeatures, ABaseCandidate, AEnvelopeCandidate, AOutCandidate, Copy, AOptions, Next)) {
				AOutCandidate = std::move(Next);
				++AOutCopies;
				break;
			}
		}
	}
	return AOutCopies > 0;
}

bool GetCircleGapFreeRegions(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const TetClusterCandidate& ACandidate, const TetCircleGapWindow& AWindow, std::vector<TetClusterFreeRegion>& AOutRegions)
{
	ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
	std::vector<TetClusterFreeRegion> FreeRegions;
	return Geometry.ExtractCandidateFreeRegions(AOriginalItems, AOptions, ACandidate, FreeRegions)
		&& ClipFreeRegionsToGapWindow(FreeRegions, AWindow, AOutRegions);
}

bool FitsCircleGapRegion(const TetShapeFeature& AFeature, const std::vector<TetClusterFreeRegion>& ARegions)
{
	for (const TetClusterFreeRegion& Region : ARegions) {
		const bool FitsBounds = (AFeature.Width <= Region.Width && AFeature.Height <= Region.Height)
			|| (AFeature.Height <= Region.Width && AFeature.Width <= Region.Height);
		if (AFeature.Area <= Region.Area + 1.0 && FitsBounds) return true;
	}
	return false;
}

std::vector<int> CollectCircleGapFillers(const std::vector<TetShapeFeature>& AFeatures,
	const TetClusterCandidate& ACandidate, const std::vector<TetClusterFreeRegion>& ARegions)
{
	std::vector<int> Fillers;
	for (int Index = 0; Index < static_cast<int>(AFeatures.size()); ++Index) {
		if (!ContainsOriginalIndex(ACandidate, Index) && AFeatures[Index].Area > 0.0
			&& FitsCircleGapRegion(AFeatures[Index], ARegions)) Fillers.push_back(Index);
	}
	std::stable_sort(Fillers.begin(), Fillers.end(), [&](int A, int B) {
		return AFeatures[A].Area > AFeatures[B].Area;
	});
	if (Fillers.size() <= CET_CIRCLE_GAP_SEARCH_MAX_CANDIDATES) return Fillers;
	const std::size_t LargeCount = (CET_CIRCLE_GAP_SEARCH_MAX_CANDIDATES + 1) / 2;
	const std::size_t SmallCount = CET_CIRCLE_GAP_SEARCH_MAX_CANDIDATES - LargeCount;
	std::vector<int> Bounded(Fillers.begin(), Fillers.begin() + static_cast<std::ptrdiff_t>(LargeCount));
	Bounded.insert(Bounded.end(), Fillers.end() - static_cast<std::ptrdiff_t>(SmallCount), Fillers.end());
	return Bounded;
}

bool IsCircleGapStateBetter(const TetClusterFillSearchState& AFirst, const TetClusterFillSearchState& ASecond)
{
	if (std::abs(AFirst.Candidate.RealArea - ASecond.Candidate.RealArea) > 1.0) {
		return AFirst.Candidate.RealArea > ASecond.Candidate.RealArea;
	}
	if (std::abs(AFirst.Candidate.ProxyWasteArea - ASecond.Candidate.ProxyWasteArea) > 1.0) {
		return AFirst.Candidate.ProxyWasteArea < ASecond.Candidate.ProxyWasteArea;
	}
	if (std::abs(AFirst.Candidate.Score - ASecond.Candidate.Score) > 1e-9) {
		return AFirst.Candidate.Score > ASecond.Candidate.Score;
	}
	return AFirst.Candidate.OriginalIndices < ASecond.Candidate.OriginalIndices;
}

void TrimCircleGapBeam(std::vector<TetClusterFillSearchState>& AStates)
{
	std::map<std::string, TetClusterFillSearchState> Unique;
	for (const TetClusterFillSearchState& State : AStates) {
		const std::string Key = MakeFilledVariantKey(State.Candidate);
		auto It = Unique.find(Key);
		if (It == Unique.end() || IsCircleGapStateBetter(State, It->second)) Unique[Key] = State;
	}
	AStates.clear();
	for (const auto& Entry : Unique) AStates.push_back(Entry.second);
	std::stable_sort(AStates.begin(), AStates.end(), IsCircleGapStateBetter);
	if (AStates.size() > CET_CIRCLE_GAP_SEARCH_BEAM_WIDTH) AStates.resize(CET_CIRCLE_GAP_SEARCH_BEAM_WIDTH);
}

bool CircleGapSearchTimeReached(const std::chrono::steady_clock::time_point& AStart, long long ALimitMs)
{
	return ALimitMs > 0 && std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::steady_clock::now() - AStart).count() >= ALimitMs;
}

bool SearchCircleGapTemplate(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures,
	const TetNestOptions& AOptions, const TetClusterCandidate& ABaseCandidate,
	const TetClusterCandidate& AEnvelopeCandidate, const TetCircleGapWindow& AWindow,
	const TetClusterCandidate& ACurrentCandidate, TetClusterCandidate& AOutCandidate)
{
	const auto SearchStart = std::chrono::steady_clock::now();
	std::size_t Attempts = 0;
	std::vector<TetClusterFillSearchState> Beam{ { ACurrentCandidate, 0 } };
	TetClusterFillSearchState Best{ ACurrentCandidate, 0 };
	ET::NEST2DMANAGERLIB::CetRectangleFillClusterBuilder Builder;
	for (std::size_t Depth = 0; Depth < CET_CIRCLE_GAP_FILL_MAX_ACCEPTED_ITEMS && !Beam.empty(); ++Depth) {
		std::vector<TetClusterFillSearchState> NextBeam;
		for (const TetClusterFillSearchState& State : Beam) {
			std::vector<TetClusterFreeRegion> LocalRegions;
			if (!GetCircleGapFreeRegions(AOriginalItems, AOptions, State.Candidate, AWindow, LocalRegions)) continue;
			for (int Index : CollectCircleGapFillers(AFeatures, State.Candidate, LocalRegions)) {
				if (Attempts++ >= CET_CIRCLE_GAP_SEARCH_MAX_ATTEMPTS
					|| CircleGapSearchTimeReached(SearchStart, CET_CIRCLE_GAP_SEARCH_MAX_TIME_MS)) break;
				TetClusterCandidate Candidate;
				if (!Builder.TryAppendFillerInRectangleEnvelope(AOriginalItems, AFeatures, ABaseCandidate,
					AEnvelopeCandidate, State.Candidate, LocalRegions, Index, AOptions, Candidate)) continue;
				TetClusterFillSearchState Next{ std::move(Candidate), State.FillerCount + 1 };
				if (IsCircleGapStateBetter(Next, Best)) Best = Next;
				NextBeam.push_back(std::move(Next));
			}
		}
		TrimCircleGapBeam(NextBeam);
		Beam = std::move(NextBeam);
		if (Attempts >= CET_CIRCLE_GAP_SEARCH_MAX_ATTEMPTS
			|| CircleGapSearchTimeReached(SearchStart, CET_CIRCLE_GAP_SEARCH_MAX_TIME_MS)) break;
	}
	AOutCandidate = std::move(Best.Candidate);
	return Best.FillerCount > 0;
}

bool IsContourInsideGapRegions(const CetPath& AContour, const std::vector<TetClusterFreeRegion>& ARegions)
{
	ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
	for (const TetClusterFreeRegion& Region : ARegions) {
		if (Geometry.IsContourInsideFreeRegion(AContour, Region, std::max(1.0, Region.Area * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE))) return true;
	}
	return false;
}

bool TryCopyCircleGapTemplateAtAngle(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions, const TetClusterCandidate& ABaseCandidate, const TetClusterCandidate& AEnvelopeCandidate, const TetCircleGapTemplate& ATemplate, const TetCircleGapWindow& ATarget, double ADelta, TetClusterCandidate& AInOutCandidate)
{
	ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
	ET::NEST2DMANAGERLIB::CetRectangleFillClusterBuilder Builder;
	const std::vector<double> Allowed = ET::NEST2DMANAGERLIB::CetRotationUtils::BuildAllowedRotations(AOptions.Rotations);
	if (ATemplate.Transforms.empty() || Allowed.empty()) return false;
	const double Cosine = std::cos(ADelta), Sine = std::sin(ADelta);
	for (const TetItemTransform& Source : ATemplate.Transforms) {
		int CopyIndex = -1;
		const std::uint64_t Family = MakeFillerFamilyKey(AFeatures[Source.OriginalId]);
		for (int Index = 0; Index < static_cast<int>(AFeatures.size()); ++Index) {
			if (!ContainsOriginalIndex(AInOutCandidate, Index) && MakeFillerFamilyKey(AFeatures[Index]) == Family) { CopyIndex = Index; break; }
		}
		if (CopyIndex < 0) return false;
		const CetPath SourceContour = Geometry.TransformContour(Geometry.GetIdentityContour(AOriginalItems[Source.OriginalId]),
			Source.RelativeRotation, Source.RelativeX, Source.RelativeY);
		double MinX = 0.0, MinY = 0.0, MaxX = 0.0, MaxY = 0.0;
		if (!Geometry.GetBounds(SourceContour, MinX, MinY, MaxX, MaxY)) return false;
		const double OffsetX = (MinX + MaxX) * 0.5 - ATemplate.Source.CenterX;
		const double OffsetY = (MinY + MaxY) * 0.5 - ATemplate.Source.CenterY;
		std::vector<double> Rotations = Allowed;
		const double Desired = Source.RelativeRotation + ADelta;
		std::stable_sort(Rotations.begin(), Rotations.end(), [&](double A, double B) { return std::abs(A - Desired) < std::abs(B - Desired); });
		bool Copied = false;
		for (double Rotation : Rotations) {
			const CetPath Rotated = Geometry.TransformContour(Geometry.GetIdentityContour(AOriginalItems[CopyIndex]), Rotation, 0.0, 0.0);
			if (!Geometry.GetBounds(Rotated, MinX, MinY, MaxX, MaxY)) continue;
			TetItemTransform Copy;
			Copy.OriginalId = CopyIndex;
			Copy.RelativeRotation = Rotation;
			Copy.RelativeX = ATarget.CenterX + OffsetX * Cosine - OffsetY * Sine - (MinX + MaxX) * 0.5;
			Copy.RelativeY = ATarget.CenterY + OffsetX * Sine + OffsetY * Cosine - (MinY + MaxY) * 0.5;
			std::vector<TetClusterFreeRegion> LocalRegions;
			const CetPath Contour = Geometry.TransformContour(Rotated, 0.0, Copy.RelativeX, Copy.RelativeY);
			TetClusterCandidate Next;
			if (GetCircleGapFreeRegions(AOriginalItems, AOptions, AInOutCandidate, ATarget, LocalRegions)
				&& IsContourInsideGapRegions(Contour, LocalRegions)
				&& Builder.TryAppendFillerTemplateInRectangleEnvelope(AOriginalItems, AFeatures, ABaseCandidate,
					AEnvelopeCandidate, AInOutCandidate, Copy, AOptions, Next)) {
				AInOutCandidate = std::move(Next);
				Copied = true;
				break;
			}
		}
		if (!Copied) return false;
	}
	return true;
}

bool CopyCircleGapTemplate(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions, const TetClusterCandidate& ABaseCandidate, const TetClusterCandidate& AEnvelopeCandidate, const TetCircleGapTemplate& ATemplate, const TetCircleGapWindow& ATarget, TetClusterCandidate& AInOutCandidate)
{
	std::vector<double> Deltas{ ATarget.Angle - ATemplate.Source.Angle };
	if (ATemplate.Source.ClassKey.rfind("triple-", 0) == 0) {
		Deltas.push_back(Deltas.front() + CET_CLUSTER_PI);
	}
	for (double Delta : Deltas) {
		TetClusterCandidate Candidate = AInOutCandidate;
		if (TryCopyCircleGapTemplateAtAngle(AOriginalItems, AFeatures, AOptions, ABaseCandidate,
			AEnvelopeCandidate, ATemplate, ATarget, Delta, Candidate)) {
			AInOutCandidate = std::move(Candidate);
			return true;
		}
	}
	return false;
}

bool BuildLocalCircleGapFilledCandidate(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions, const TetClusterCandidate& ABaseCandidate, const TetClusterCandidate& AEnvelopeCandidate, std::map<std::string, TetCircleGapTemplate>& ATemplates, TetClusterCandidate& AOutCandidate)
{
	AOutCandidate = AEnvelopeCandidate;
	const auto SearchStart = std::chrono::steady_clock::now();
	const std::vector<TetCircleGapWindow> Windows = CollectCircleGapWindows(AOriginalItems, AFeatures, ABaseCandidate, AEnvelopeCandidate);
	std::set<std::string> ExhaustedClasses;
	for (const TetCircleGapWindow& Window : Windows) {
		if (CircleGapSearchTimeReached(SearchStart, CET_CIRCLE_GAP_TOTAL_SEARCH_MAX_TIME_MS)) break;
		if (ExhaustedClasses.find(Window.ClassKey) != ExhaustedClasses.end()) continue;
		auto It = ATemplates.find(Window.ClassKey);
		if (It != ATemplates.end() && CopyCircleGapTemplate(AOriginalItems, AFeatures, AOptions,
			ABaseCandidate, AEnvelopeCandidate, It->second, Window, AOutCandidate)) {
			std::cout << "[TEMPLATE][GAP CACHE COPY] Class=" << Window.ClassKey << std::endl;
			continue;
		}
		const std::size_t StartCount = AOutCandidate.Transforms.size();
		TetClusterCandidate SearchedCandidate;
		if (!SearchCircleGapTemplate(AOriginalItems, AFeatures, AOptions, ABaseCandidate,
			AEnvelopeCandidate, Window, AOutCandidate, SearchedCandidate)) {
			ExhaustedClasses.insert(Window.ClassKey);
			continue;
		}
		std::vector<TetItemTransform> Added(SearchedCandidate.Transforms.begin() + static_cast<std::ptrdiff_t>(StartCount),
			SearchedCandidate.Transforms.end());
		AOutCandidate = std::move(SearchedCandidate);
		ATemplates[Window.ClassKey] = TetCircleGapTemplate{ Window, std::move(Added) };
	}
	return AOutCandidate.Transforms.size() > ABaseCandidate.Transforms.size();
}

bool FitsAnyFreeRegion(const TetShapeFeature& AFeature, const std::vector<TetClusterFreeRegion>& AFreeRegions) {
	for (const TetClusterFreeRegion& Region : AFreeRegions) {
		const bool FitsNormal = AFeature.Width <= Region.Width && AFeature.Height <= Region.Height;
		const bool FitsRotated = AFeature.Height <= Region.Width && AFeature.Width <= Region.Height;
		if (AFeature.Area <= Region.Area && (FitsNormal || FitsRotated)) return true;
	}
	return false;
}

bool IsInventoryRebalanceCandidate(const TetClusterCandidate& ACandidate)
{
	return ACandidate.Valid && ACandidate.SkeletonChildCount >= 2
		// Inventory rebalance preserves the existing proxy.  It is therefore
		// valid only for candidates that were already finalized as a fixed
		// envelope-fill virtual board; a raw skeleton may have an irregular
		// proxy whose outer boundary changes when a child is appended.
		&& (ACandidate.BuilderName == "EnvelopeFillSearch"
			|| ACandidate.BuilderName == "GlobalInventoryRebalance"
			|| (IsInventorySkeletonCandidate(ACandidate)
				&& HasFullRectangleProxy(ACandidate)))
		&& ACandidate.SkeletonChildCount <= ACandidate.Transforms.size()
		&& ACandidate.OriginalIndices.size() == ACandidate.Transforms.size()
		&& ACandidate.ProxyContour.size() >= 3 && ACandidate.ProxyArea > 0.0;
}

bool IsRebalanceOrderBetter(const TetClusterCandidate& AFirst, const TetClusterCandidate& ASecond)
{
	if (AFirst.SkeletonChildCount != ASecond.SkeletonChildCount) {
		return AFirst.SkeletonChildCount > ASecond.SkeletonChildCount;
	}
	if (std::abs(AFirst.ProxyWasteArea - ASecond.ProxyWasteArea) > 1.0) {
		return AFirst.ProxyWasteArea > ASecond.ProxyWasteArea;
	}
	return AFirst.ClusterType < ASecond.ClusterType;
}

void PreserveInventoryProxy(const TetClusterCandidate& AProxySource,
	TetClusterCandidate& AInOutCandidate)
{
	AInOutCandidate.ProxyContour = AProxySource.ProxyContour;
	AInOutCandidate.ProxyContourNormalized = AProxySource.ProxyContourNormalized;
	AInOutCandidate.ProxyArea = AProxySource.ProxyArea;
	AInOutCandidate.ProxyMode = AProxySource.ProxyMode;
	AInOutCandidate.ClusterWidth = AProxySource.ClusterWidth;
	AInOutCandidate.ClusterHeight = AProxySource.ClusterHeight;
	AInOutCandidate.BoundingBoxArea = AProxySource.BoundingBoxArea;
	AInOutCandidate.ReservedArea = std::min(AProxySource.ProxyArea, AInOutCandidate.RealArea);
	AInOutCandidate.ProxyWasteArea = std::max(0.0, AProxySource.ProxyArea - AInOutCandidate.ReservedArea);
	AInOutCandidate.ProxyWasteRatio = AProxySource.ProxyArea > 0.0
		? AInOutCandidate.ProxyWasteArea / AProxySource.ProxyArea : 1.0;
	AInOutCandidate.FillRatio = AProxySource.ProxyArea > 0.0
		? std::clamp(AInOutCandidate.RealArea / AProxySource.ProxyArea, 0.0, 1.0) : 0.0;
	AInOutCandidate.BoundingFillRatio = AInOutCandidate.BoundingBoxArea > 0.0
		? std::clamp(AInOutCandidate.RealArea / AInOutCandidate.BoundingBoxArea, 0.0, 1.0) : 0.0;
	AInOutCandidate.SheetReuseScore = AProxySource.SheetReuseScore;
	AInOutCandidate.FragmentationRisk = AProxySource.FragmentationRisk;
	AInOutCandidate.Score = AProxySource.Score;
}

bool TryAppendInventoryFiller(const CetTNestItemVector& AOriginalItems,
	const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions,
	const TetClusterCandidate& ACurrentCandidate, int AFillerIndex,
	TetClusterCandidate& AOutCandidate)
{
	AOutCandidate = TetClusterCandidate{};
	ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
	std::vector<TetClusterFreeRegion> FreeRegions;
	if (!Geometry.ExtractCandidateFreeRegions(AOriginalItems, AOptions, ACurrentCandidate, FreeRegions)) {
		std::cout << "[TEMPLATE][GLOBAL FILL REJECT] Filler=" << AFillerIndex
			<< " Reason=FreeRegionExtract" << std::endl;
		return false;
	}
	if (!FitsAnyFreeRegion(AFeatures[AFillerIndex], FreeRegions)) {
		std::cout << "[TEMPLATE][GLOBAL FILL REJECT] Filler=" << AFillerIndex
			<< " Reason=FreeRegionBounds RegionCount=" << FreeRegions.size() << std::endl;
		return false;
	}
	ET::NEST2DMANAGERLIB::CetRectangleFillClusterBuilder Builder;
	if (!Builder.TryAppendFillerInFreeRegions(AOriginalItems, AFeatures, ACurrentCandidate,
		ACurrentCandidate, FreeRegions, AFillerIndex, AOptions, AOutCandidate)) {
		std::cout << "[TEMPLATE][GLOBAL FILL REJECT] Filler=" << AFillerIndex
			<< " Reason=ContourOrSpacing" << std::endl;
		return false;
	}
	PreserveInventoryProxy(ACurrentCandidate, AOutCandidate);
	AOutCandidate.BuilderName = "GlobalInventoryRebalance";
	AOutCandidate.ClusterType = ACurrentCandidate.ClusterType;
	return true;
}

bool TryRemoveInventoryFiller(const CetTNestItemVector& AOriginalItems,
	const TetNestOptions& AOptions, const TetClusterCandidate& ACandidate,
	std::size_t ATransformIndex, TetClusterCandidate& AOutCandidate)
{
	AOutCandidate = TetClusterCandidate{};
	if (!IsInventoryRebalanceCandidate(ACandidate)
		|| ATransformIndex < ACandidate.SkeletonChildCount
		|| ATransformIndex >= ACandidate.Transforms.size()) return false;
	AOutCandidate = ACandidate;
	const int OriginalId = AOutCandidate.Transforms[ATransformIndex].OriginalId;
	AOutCandidate.Transforms.erase(AOutCandidate.Transforms.begin() + static_cast<std::ptrdiff_t>(ATransformIndex));
	auto IndexIt = std::find(AOutCandidate.OriginalIndices.begin(), AOutCandidate.OriginalIndices.end(), OriginalId);
	if (IndexIt == AOutCandidate.OriginalIndices.end()) return false;
	AOutCandidate.OriginalIndices.erase(IndexIt);
	ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
	if (!Geometry.FinalizeCandidate(AOriginalItems, AOptions, AOutCandidate)) return false;
	PreserveInventoryProxy(ACandidate, AOutCandidate);
	AOutCandidate.BuilderName = "GlobalInventoryRebalance";
	AOutCandidate.ClusterType = ACandidate.ClusterType;
	return true;
}

bool IsTriangleBuilderCandidate(const TetClusterCandidate& ACandidate)
{
	return ACandidate.Valid && ACandidate.BuilderName == "TriangleBuilder"
		&& ACandidate.OriginalIndices.size() == ACandidate.Transforms.size()
		&& ACandidate.OriginalIndices.size() >= 4;
}

bool IsDeferredTriangleCandidate(const TetClusterCandidate& ACandidate)
{
	return ACandidate.BuilderName == "TriangleBuilder";
}

bool HasFilledEllipseCandidate(const std::vector<TetClusterCandidate>& ACandidates)
{
	for (const TetClusterCandidate& Candidate : ACandidates) {
		if (Candidate.ClusterType.find("Ellipse") != std::string::npos
			&& Candidate.OriginalIndices.size() > Candidate.SkeletonChildCount) return true;
	}
	return false;
}

bool HasExactCandidateInventory(const TetClusterCandidate& ACandidate,
	const std::vector<int>& AExpectedIndices)
{
	if (!ACandidate.Valid || ACandidate.OriginalIndices.size() != AExpectedIndices.size()
		|| ACandidate.Transforms.size() != AExpectedIndices.size()) return false;
	std::set<int> Actual(ACandidate.OriginalIndices.begin(), ACandidate.OriginalIndices.end());
	std::set<int> Expected(AExpectedIndices.begin(), AExpectedIndices.end());
	return Actual == Expected && Actual.size() == AExpectedIndices.size();
}

bool IsInventoryTransferWorthKeeping(const TetClusterCandidate& ATargetBefore,
	const TetClusterCandidate& ATargetAfter, const TetClusterCandidate& ASourceBefore,
	const TetClusterCandidate& ASourceAfter);

bool TryBuildReducedTriangleCandidate(const CetTNestItemVector& AOriginalItems,
	const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions,
	const TetClusterCandidate& ASourceCandidate, const std::set<int>& ARemovedIds,
	TetClusterCandidate& AOutCandidate)
{
	AOutCandidate = TetClusterCandidate{};
	std::vector<int> Remaining;
	for (int OriginalId : ASourceCandidate.OriginalIndices) {
		if (ARemovedIds.find(OriginalId) == ARemovedIds.end()) Remaining.push_back(OriginalId);
	}
	if (Remaining.size() < 2) return false;
	ET::NEST2DMANAGERLIB::CetTriangleClusterBuilder Builder;
	std::vector<TetClusterCandidate> Candidates;
	Builder.BuildCandidates(AOriginalItems, AFeatures, Remaining, AOptions, Candidates);
	for (const TetClusterCandidate& Candidate : Candidates) {
		if (HasExactCandidateInventory(Candidate, Remaining)) {
			AOutCandidate = Candidate;
			PreserveInventoryProxy(ASourceCandidate, AOutCandidate);
			AOutCandidate.BuilderName = "GlobalInventoryRebalance";
			AOutCandidate.ClusterType = ASourceCandidate.ClusterType + "_Reduced";
			return true;
		}
	}
	return false;
}

bool IsTriangleTransferWorthKeeping(const TetClusterCandidate& ATargetBefore,
	const TetClusterCandidate& ATargetAfter, const TetClusterCandidate& ASourceBefore,
	const TetClusterCandidate& ASourceAfter);

bool TryTransferTrianglePair(const CetTNestItemVector& AOriginalItems,
	const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions,
	const TetClusterCandidate& ATargetBefore, const TetClusterCandidate& ASourceBefore,
	TetClusterCandidate& AOutTarget, TetClusterCandidate& AOutSource,
	std::pair<int, int>& AOutPair)
{
	const std::size_t ItemCount = ASourceBefore.OriginalIndices.size();
	for (std::size_t First = ItemCount - 2; ; --First) {
		for (std::size_t Second = ItemCount - 1; Second > First; --Second) {
			const int FirstId = ASourceBefore.OriginalIndices[First];
			const int SecondId = ASourceBefore.OriginalIndices[Second];
			TetClusterCandidate ExpandedTarget;
			if (!TryAppendInventoryFiller(AOriginalItems, AFeatures, AOptions,
				ATargetBefore, FirstId, ExpandedTarget)) {
				continue;
			}
			TetClusterCandidate ExpandedTargetPair;
			if (!TryAppendInventoryFiller(AOriginalItems, AFeatures, AOptions,
				ExpandedTarget, SecondId, ExpandedTargetPair)) {
				std::cout << "[TEMPLATE][TRIANGLE TRANSFER PROBE] Pair="
					<< FirstId << "," << SecondId << " SecondAppend=reject" << std::endl;
				continue;
			}
			const std::set<int> RemovedIds = { FirstId, SecondId };
			TetClusterCandidate ReducedSource;
			if (!TryBuildReducedTriangleCandidate(AOriginalItems, AFeatures, AOptions,
				ASourceBefore, RemovedIds, ReducedSource)) {
				std::cout << "[TEMPLATE][TRIANGLE TRANSFER PROBE] Pair="
					<< FirstId << "," << SecondId << " ReducedSource=reject" << std::endl;
				continue;
			}
			if (!IsTriangleTransferWorthKeeping(ATargetBefore, ExpandedTargetPair,
				ASourceBefore, ReducedSource)) {
				std::cout << "[TEMPLATE][TRIANGLE TRANSFER PROBE] Pair="
					<< FirstId << "," << SecondId << " Gain=reject" << std::endl;
				continue;
			}
			AOutTarget = std::move(ExpandedTargetPair);
			AOutSource = std::move(ReducedSource);
			AOutPair = { FirstId, SecondId };
			return true;
		}
		if (First == 0) break;
	}
	return false;
}

bool IsTransferPairGloballyUnique(const std::vector<TetClusterCandidate>& AAcceptedCandidates,
	std::size_t ASourceIndex, const std::pair<int, int>& APair)
{
	for (std::size_t CandidateIndex = 0; CandidateIndex < AAcceptedCandidates.size(); ++CandidateIndex) {
		if (CandidateIndex == ASourceIndex) continue;
		for (int OriginalId : AAcceptedCandidates[CandidateIndex].OriginalIndices) {
			if (OriginalId == APair.first || OriginalId == APair.second) return false;
		}
	}
	return APair.first >= 0 && APair.second >= 0 && APair.first != APair.second;
}

bool IsTriangleTransferWorthKeeping(const TetClusterCandidate& ATargetBefore,
	const TetClusterCandidate& ATargetAfter, const TetClusterCandidate& ASourceBefore,
	const TetClusterCandidate& ASourceAfter)
{
	if (ATargetAfter.RealArea <= ATargetBefore.RealArea) return false;
	const double TotalBefore = ATargetBefore.ProxyWasteArea + ASourceBefore.ProxyWasteArea;
	const double TotalAfter = ATargetAfter.ProxyWasteArea + ASourceAfter.ProxyWasteArea;
	const double Tolerance = std::max(1.0, ATargetBefore.ProxyArea
		* CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE);
	std::cout << "[TEMPLATE][TRIANGLE TRANSFER METRIC] TargetArea="
		<< ATargetBefore.RealArea << "->" << ATargetAfter.RealArea
		<< " Waste=" << TotalBefore << "->" << TotalAfter << std::endl;
	return TotalAfter <= TotalBefore + Tolerance;
}

bool IsInventoryTransferWorthKeeping(const TetClusterCandidate& ATargetBefore,
	const TetClusterCandidate& ATargetAfter, const TetClusterCandidate& ASourceBefore,
	const TetClusterCandidate& ASourceAfter)
{
	const double TargetTolerance = std::max(1.0, ATargetBefore.ProxyArea * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE);
	const double TotalBefore = ATargetBefore.ProxyWasteArea + ASourceBefore.ProxyWasteArea;
	const double TotalAfter = ATargetAfter.ProxyWasteArea + ASourceAfter.ProxyWasteArea;
	return ATargetAfter.ProxyWasteArea < ATargetBefore.ProxyWasteArea - TargetTolerance
		&& TotalAfter <= TotalBefore + TargetTolerance;
}

bool TryBuildInventorySkeletonEnvelope(const CetTNestItemVector& AOriginalItems,
	const TetNestOptions& AOptions, const TetClusterCandidate& ACandidate,
	TetClusterCandidate& AOutCandidate)
{
	AOutCandidate = TetClusterCandidate{};
	if (!IsInventorySkeletonCandidate(ACandidate)
		|| !BuildRectangleEnvelopeCandidate(AOriginalItems, AOptions, ACandidate, AOutCandidate)) return false;
	AOutCandidate.BuilderName = "EnvelopeFillSearch";
	AOutCandidate.ClusterType = ACandidate.ClusterType + "_InventoryEnvelope";
	AOutCandidate.SkeletonChildCount = ACandidate.SkeletonChildCount;
	AOutCandidate.Score = ACandidate.Score;
	return true;
}

void RebalanceAcceptedClusterInventory(const CetTNestItemVector& AOriginalItems,
	const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions,
	std::vector<TetClusterCandidate>& AAcceptedCandidates, std::vector<bool>& AUsed)
{
	const auto Start = std::chrono::steady_clock::now();
	std::vector<std::size_t> Targets;
	for (std::size_t Index = 0; Index < AAcceptedCandidates.size(); ++Index) {
		if (IsInventoryRebalanceCandidate(AAcceptedCandidates[Index])
			|| IsInventorySkeletonCandidate(AAcceptedCandidates[Index])) Targets.push_back(Index);
	}
	std::stable_sort(Targets.begin(), Targets.end(), [&](std::size_t A, std::size_t B) {
		return IsRebalanceOrderBetter(AAcceptedCandidates[A], AAcceptedCandidates[B]);
	});
	std::size_t Attempts = 0;
	for (std::size_t TargetIndex : Targets) {
		if (Attempts >= CET_CLUSTER_GLOBAL_REBALANCE_MAX_ATTEMPTS
			|| CircleGapSearchTimeReached(Start, CET_CLUSTER_GLOBAL_REBALANCE_MAX_SEARCH_TIME_MS)) break;
		std::vector<int> Available;
		for (int Index = 0; Index < static_cast<int>(AFeatures.size()) && Available.size() < CET_CLUSTER_GLOBAL_REBALANCE_MAX_UNASSIGNED_FILLERS; ++Index) {
			if (!AUsed[Index] && AFeatures[Index].Area > 0.0) Available.push_back(Index);
		}
		std::stable_sort(Available.begin(), Available.end(), [&](int A, int B) { return AFeatures[A].Area > AFeatures[B].Area; });
		TetClusterCandidate TargetCandidate = AAcceptedCandidates[TargetIndex];
		if (!IsInventoryRebalanceCandidate(TargetCandidate)) {
			TetClusterCandidate EnvelopeCandidate;
			if (!TryBuildInventorySkeletonEnvelope(AOriginalItems, AOptions, TargetCandidate, EnvelopeCandidate)) continue;
			TargetCandidate = std::move(EnvelopeCandidate);
		}
		for (int FillerIndex : Available) {
			if (Attempts++ >= CET_CLUSTER_GLOBAL_REBALANCE_MAX_ATTEMPTS) break;
			TetClusterCandidate Expanded;
			if (!TryAppendInventoryFiller(AOriginalItems, AFeatures, AOptions,
				TargetCandidate, FillerIndex, Expanded)) continue;
			AAcceptedCandidates[TargetIndex] = std::move(Expanded);
			TargetCandidate = AAcceptedCandidates[TargetIndex];
			AUsed[FillerIndex] = true;
			std::cout << "[TEMPLATE][GLOBAL FILL] Target=" << TargetIndex
				<< " Filler=" << FillerIndex << std::endl;
		}
	}
	std::size_t Transfers = 0;
	for (std::size_t TargetIndex : Targets) {
		if (Transfers >= CET_CLUSTER_GLOBAL_REBALANCE_MAX_TRANSFERS
			|| CircleGapSearchTimeReached(Start, CET_CLUSTER_GLOBAL_REBALANCE_MAX_SEARCH_TIME_MS)) break;
		if (!IsInventoryRebalanceCandidate(AAcceptedCandidates[TargetIndex])) continue;
		for (std::size_t SourceIndex : Targets) {
			if (TargetIndex == SourceIndex || AAcceptedCandidates[TargetIndex].SkeletonChildCount
				<= AAcceptedCandidates[SourceIndex].SkeletonChildCount) continue;
			for (std::size_t FillerIndex = AAcceptedCandidates[SourceIndex].SkeletonChildCount;
				FillerIndex < AAcceptedCandidates[SourceIndex].Transforms.size(); ++FillerIndex) {
				if (Transfers >= CET_CLUSTER_GLOBAL_REBALANCE_MAX_TRANSFERS
					|| CircleGapSearchTimeReached(Start, CET_CLUSTER_GLOBAL_REBALANCE_MAX_SEARCH_TIME_MS)) return;
				const int OriginalId = AAcceptedCandidates[SourceIndex].Transforms[FillerIndex].OriginalId;
				TetClusterCandidate ReducedSource;
				TetClusterCandidate ExpandedTarget;
				if (!TryRemoveInventoryFiller(AOriginalItems, AOptions, AAcceptedCandidates[SourceIndex], FillerIndex, ReducedSource)
					|| !TryAppendInventoryFiller(AOriginalItems, AFeatures, AOptions, AAcceptedCandidates[TargetIndex], OriginalId, ExpandedTarget)
					|| !IsInventoryTransferWorthKeeping(AAcceptedCandidates[TargetIndex], ExpandedTarget,
						AAcceptedCandidates[SourceIndex], ReducedSource)) continue;
				AAcceptedCandidates[TargetIndex] = std::move(ExpandedTarget);
				AAcceptedCandidates[SourceIndex] = std::move(ReducedSource);
				++Transfers;
				std::cout << "[TEMPLATE][GLOBAL TRANSFER] Source=" << SourceIndex
					<< " Target=" << TargetIndex << " Filler=" << OriginalId << std::endl;
				break;
			}
		}
	}
	const auto TriangleTransferStart = std::chrono::steady_clock::now();
	for (std::size_t TargetIndex : Targets) {
		if (Transfers >= CET_CLUSTER_GLOBAL_REBALANCE_MAX_TRANSFERS
			|| CircleGapSearchTimeReached(TriangleTransferStart, CET_CLUSTER_GLOBAL_REBALANCE_MAX_SEARCH_TIME_MS)) break;
		if (!IsInventoryRebalanceCandidate(AAcceptedCandidates[TargetIndex])) continue;
		for (std::size_t SourceIndex = 0; SourceIndex < AAcceptedCandidates.size(); ++SourceIndex) {
			if (TargetIndex == SourceIndex || !IsTriangleBuilderCandidate(AAcceptedCandidates[SourceIndex])) continue;
			TetClusterCandidate ExpandedTarget;
			TetClusterCandidate ReducedSource;
			std::pair<int, int> MovedPair = { -1, -1 };
			if (!TryTransferTrianglePair(AOriginalItems, AFeatures, AOptions,
				AAcceptedCandidates[TargetIndex], AAcceptedCandidates[SourceIndex],
				ExpandedTarget, ReducedSource, MovedPair)) continue;
			if (!IsTransferPairGloballyUnique(AAcceptedCandidates, SourceIndex, MovedPair)) {
			std::cout << "[TEMPLATE][TRIANGLE TRANSFER] Pair=" << MovedPair.first
				<< "," << MovedPair.second << " DuplicateInventory=reject" << std::endl;
			continue;
		}
			AAcceptedCandidates[TargetIndex] = std::move(ExpandedTarget);
			AAcceptedCandidates[SourceIndex] = std::move(ReducedSource);
			++Transfers;
			std::cout << "[TEMPLATE][TRIANGLE TRANSFER] Source=" << SourceIndex
				<< " Target=" << TargetIndex << " Fillers="
				<< MovedPair.first << "," << MovedPair.second << std::endl;
			break;
		}
	}
}

std::vector<int> CollectCompatibleFillers(const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ABaseCandidate, const std::vector<TetClusterFreeRegion>& AFreeRegions, const TetClusterFillSearchConfig& AConfig, bool ADeduplicateFamilies = false) {
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
	if (ADeduplicateFamilies) {
		std::set<std::uint64_t> SeenFamilies;
		Fillers.erase(std::remove_if(Fillers.begin(), Fillers.end(), [&](int Index) {
			return !SeenFamilies.insert(MakeFillerFamilyKey(AFeatures[Index])).second;
			}), Fillers.end());
	}
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

double NormalizeClusterAngle(double AAngle);

std::vector<TetEllipseCenter> CollectEllipseGapCenters(const CetTNestItemVector& AOriginalItems,
	const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ACandidate)
{
	std::vector<TetEllipseCenter> Centers;
	ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
	for (const TetItemTransform& Transform : ACandidate.Transforms) {
		if (Transform.OriginalId < 0 || Transform.OriginalId >= static_cast<int>(AFeatures.size())
			|| AFeatures[Transform.OriginalId].ShapeType != MetShapeType::EllipseLike) continue;
		const CetPath Contour = Geometry.TransformContour(Geometry.GetIdentityContour(
			AOriginalItems[Transform.OriginalId]), Transform.RelativeRotation,
			Transform.RelativeX, Transform.RelativeY);
		double MinX = 0.0, MinY = 0.0, MaxX = 0.0, MaxY = 0.0;
		if (Geometry.GetBounds(Contour, MinX, MinY, MaxX, MaxY)) {
			Centers.push_back({ (MinX + MaxX) * 0.5, (MinY + MaxY) * 0.5,
				(MaxX - MinX) * 0.5, (MaxY - MinY) * 0.5,
				Transform.RelativeRotation });
		}
	}
	return Centers;
}

void AppendEllipseGapWindow(std::vector<TetCircleGapWindow>& AWindows, double AX, double AY,
	double AAngle, double AHalfWidth, double AHalfHeight, const char* AKind, double AScale)
{
	if (AHalfWidth <= CET_RECTANGLE_FILL_POSITION_TOLERANCE
		|| AHalfHeight <= CET_RECTANGLE_FILL_POSITION_TOLERANCE) return;
	const long long AngleKey = std::llround(NormalizeClusterAngle(AAngle) / CET_ELLIPSE_GAP_TEMPLATE_ANGLE_TOLERANCE);
	AWindows.push_back({ AX, AY, AAngle, AHalfWidth, AHalfHeight,
		BuildCircleGapClassKey(AKind, AHalfWidth, AHalfHeight, AScale) + "|a=" + std::to_string(AngleKey) });
}

void AppendEllipseQuadGapWindows(const std::vector<TetEllipseCenter>& ACenters,
	const std::vector<std::vector<std::size_t>>& ANeighbors, std::vector<TetCircleGapWindow>& AWindows)
{
	std::set<std::string> Seen;
	for (std::size_t First = 0; First < ANeighbors.size(); ++First) {
		const std::vector<std::size_t>& Local = ANeighbors[First];
		for (std::size_t Left = 0; Left < Local.size(); ++Left) for (std::size_t Right = Left + 1; Right < Local.size(); ++Right) {
			const std::size_t Second = Local[Left], Third = Local[Right];
			const double Cross = (ACenters[Second].X - ACenters[First].X) * (ACenters[Third].Y - ACenters[First].Y)
				- (ACenters[Second].Y - ACenters[First].Y) * (ACenters[Third].X - ACenters[First].X);
			if (std::abs(Cross) <= CET_RECTANGLE_FILL_POSITION_TOLERANCE) continue;
			for (std::size_t Fourth : ANeighbors[Second]) {
				if (Fourth == First || Fourth == Second || Fourth == Third
					|| !AreCircleNeighbors(ANeighbors, Third, Fourth)) continue;
				std::vector<std::size_t> Indices{ First, Second, Third, Fourth };
				std::sort(Indices.begin(), Indices.end());
				std::ostringstream Key;
				for (std::size_t Index : Indices) Key << Index << ',';
				if (!Seen.insert(Key.str()).second) continue;
				double MinX = ACenters[First].X, MaxX = MinX, MinY = ACenters[First].Y, MaxY = MinY;
				double Scale = std::min(ACenters[First].HalfWidth, ACenters[First].HalfHeight);
				for (std::size_t Index : Indices) {
					MinX = std::min(MinX, ACenters[Index].X); MaxX = std::max(MaxX, ACenters[Index].X);
					MinY = std::min(MinY, ACenters[Index].Y); MaxY = std::max(MaxY, ACenters[Index].Y);
					Scale = std::min(Scale, std::min(ACenters[Index].HalfWidth, ACenters[Index].HalfHeight));
				}
				AppendEllipseGapWindow(AWindows, (MinX + MaxX) * 0.5, (MinY + MaxY) * 0.5, 0.0,
					std::max(CET_RECTANGLE_FILL_POSITION_TOLERANCE * 2.0, Scale * 0.40),
					std::max(CET_RECTANGLE_FILL_POSITION_TOLERANCE * 2.0, Scale * 0.40), "quad-ellipse", Scale);
			}
		}
	}
}

std::vector<TetCircleGapWindow> CollectEllipseGapWindows(const CetTNestItemVector& AOriginalItems,
	const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ABaseCandidate,
	const TetClusterCandidate& AEnvelopeCandidate)
{
	const std::vector<TetEllipseCenter> Centers = CollectEllipseGapCenters(AOriginalItems, AFeatures, ABaseCandidate);
	std::vector<TetCircleCenter> Proxies;
	for (const TetEllipseCenter& Center : Centers) {
		Proxies.push_back({ Center.X, Center.Y, std::max(Center.HalfWidth, Center.HalfHeight) });
	}
	const double Nearest = FindNearestCircleDistance(Proxies);
	double MaxExtent = 0.0;
	for (const TetEllipseCenter& Center : Centers) {
		MaxExtent = std::max(MaxExtent, std::max(Center.HalfWidth, Center.HalfHeight));
	}
	const double NeighborLimit = std::isfinite(Nearest)
		? std::max(Nearest * 1.35, MaxExtent * 2.2)
			+ CET_RECTANGLE_FILL_POSITION_TOLERANCE : 0.0;
	const std::vector<std::vector<std::size_t>> Neighbors = NeighborLimit > 0.0
		? BuildCircleNeighborLists(Proxies, NeighborLimit) : std::vector<std::vector<std::size_t>>{};
	std::vector<TetCircleGapWindow> Windows;
	AppendEllipseQuadGapWindows(Centers, Neighbors, Windows);
	for (std::size_t First = 0; First < Centers.size(); ++First) {
		for (std::size_t Second : Neighbors[First]) {
			if (Second <= First) continue;
			const double DX = Centers[Second].X - Centers[First].X;
			const double DY = Centers[Second].Y - Centers[First].Y;
			const double Distance = std::hypot(DX, DY);
			if (Distance <= CET_RECTANGLE_FILL_POSITION_TOLERANCE) continue;
			const double Scale = std::min({ Centers[First].HalfWidth, Centers[First].HalfHeight,
				Centers[Second].HalfWidth, Centers[Second].HalfHeight });
			const double HalfSize = std::max(CET_RECTANGLE_FILL_POSITION_TOLERANCE * 2.0,
				std::min(Scale * 0.35, Distance * 0.30));
			AppendEllipseGapWindow(Windows, (Centers[First].X + Centers[Second].X) * 0.5,
				(Centers[First].Y + Centers[Second].Y) * 0.5, std::atan2(DY, DX),
				HalfSize, HalfSize, "pair-ellipse", Distance);
		}
	}
	for (std::size_t First = 0; First < Centers.size(); ++First) {
		const std::vector<std::size_t>& Local = Neighbors[First];
		for (std::size_t Left = 0; Left < Local.size(); ++Left) for (std::size_t Right = Left + 1; Right < Local.size(); ++Right) {
			const std::size_t Second = Local[Left], Third = Local[Right];
			if (Second <= First || Third <= First || !AreCircleNeighbors(Neighbors, Second, Third)) continue;
			const double Scale = std::min({ Centers[First].HalfWidth, Centers[First].HalfHeight,
				Centers[Second].HalfWidth, Centers[Second].HalfHeight,
				Centers[Third].HalfWidth, Centers[Third].HalfHeight });
			const double HalfSize = std::max(CET_RECTANGLE_FILL_POSITION_TOLERANCE * 2.0, Scale * 0.28);
			AppendEllipseGapWindow(Windows, (Centers[First].X + Centers[Second].X + Centers[Third].X) / 3.0,
				(Centers[First].Y + Centers[Second].Y + Centers[Third].Y) / 3.0, 0.0,
				HalfSize, HalfSize, "triple-ellipse", Scale);
		}
	}
	for (const TetEllipseCenter& Center : Centers) {
		const double Top = Center.Y - Center.HalfHeight;
		const double Bottom = AEnvelopeCandidate.ClusterHeight - Center.Y - Center.HalfHeight;
		const double Left = Center.X - Center.HalfWidth;
		const double Right = AEnvelopeCandidate.ClusterWidth - Center.X - Center.HalfWidth;
		if (Top > CET_RECTANGLE_FILL_POSITION_TOLERANCE) AppendEllipseGapWindow(Windows, Center.X, Top * 0.5, 0.0,
			std::min(Center.HalfWidth, AEnvelopeCandidate.ClusterWidth * 0.5), Top * 0.5, "edge-ellipse", Top);
		if (Bottom > CET_RECTANGLE_FILL_POSITION_TOLERANCE) AppendEllipseGapWindow(Windows, Center.X,
			AEnvelopeCandidate.ClusterHeight - Bottom * 0.5, CET_CLUSTER_PI, std::min(Center.HalfWidth,
			AEnvelopeCandidate.ClusterWidth * 0.5), Bottom * 0.5, "edge-ellipse", Bottom);
		if (Left > CET_RECTANGLE_FILL_POSITION_TOLERANCE) AppendEllipseGapWindow(Windows, Left * 0.5, Center.Y,
			CET_CLUSTER_HALF_PI, Left * 0.5, std::min(Center.HalfHeight, AEnvelopeCandidate.ClusterHeight * 0.5), "edge-ellipse", Left);
		if (Right > CET_RECTANGLE_FILL_POSITION_TOLERANCE) AppendEllipseGapWindow(Windows,
			AEnvelopeCandidate.ClusterWidth - Right * 0.5, Center.Y, -CET_CLUSTER_HALF_PI, Right * 0.5,
			std::min(Center.HalfHeight, AEnvelopeCandidate.ClusterHeight * 0.5), "edge-ellipse", Right);
	}
	std::stable_sort(Windows.begin(), Windows.end(), [](const TetCircleGapWindow& A, const TetCircleGapWindow& B) {
		const int APriority = GetCircleGapPriority(A.ClassKey), BPriority = GetCircleGapPriority(B.ClassKey);
		if (APriority != BPriority) return APriority < BPriority;
		const double AArea = A.HalfWidth * A.HalfHeight;
		const double BArea = B.HalfWidth * B.HalfHeight;
		if (std::abs(AArea - BArea) > 1.0) return AArea > BArea;
		return A.ClassKey < B.ClassKey;
	});
	if (Windows.size() > CET_ELLIPSE_GAP_FILL_MAX_WINDOWS) Windows.resize(CET_ELLIPSE_GAP_FILL_MAX_WINDOWS);
	return Windows;
}

bool SearchEllipseGapWindow(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures,
	const TetNestOptions& AOptions, const TetClusterCandidate& ABaseCandidate,
	const TetClusterCandidate& AEnvelopeCandidate, const TetCircleGapWindow& AWindow,
	const TetClusterFillSearchConfig& AConfig, const TetClusterFillSearchState& AInitial,
	TetClusterCandidate& AOutCandidate)
{
	const auto SearchStart = std::chrono::steady_clock::now();
	std::vector<TetClusterFillSearchState> Beam{ AInitial }, Next;
	TetClusterFillSearchState Best = AInitial;
	ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
	ET::NEST2DMANAGERLIB::CetRectangleFillClusterBuilder Builder;
	std::size_t Attempts = 0;
	const std::size_t MaxDepth = std::min<std::size_t>(1, AConfig.MaxDepth);
	for (std::size_t Depth = 0; Depth < MaxDepth && !Beam.empty(); ++Depth) {
		Next.clear();
		for (const TetClusterFillSearchState& State : Beam) {
			std::vector<TetClusterFreeRegion> FreeRegions, LocalRegions;
			if (!Geometry.ExtractCandidateFreeRegions(AOriginalItems, AOptions, State.Candidate, FreeRegions)) continue;
			if (!ClipFreeRegionsToGapWindow(FreeRegions, AWindow, LocalRegions)) continue;
			if (LocalRegions.empty()) continue;
			// One representative per family keeps the local beam compositional while
			// avoiding identical contour probes for every remaining source instance.
			for (int Filler : CollectCompatibleFillers(AFeatures, State.Candidate, LocalRegions, AConfig, true)) {
				if (Attempts++ >= CET_ELLIPSE_GAP_FILL_MAX_ATTEMPTS) break;
				TetClusterCandidate Candidate;
				if (!Builder.TryAppendFillerInRectangleEnvelope(AOriginalItems, AFeatures, ABaseCandidate,
					AEnvelopeCandidate, State.Candidate, LocalRegions, Filler, AOptions, Candidate)) continue;
				TetClusterFillSearchState StateCandidate{ std::move(Candidate), State.FillerCount + 1 };
				if (IsEnvelopeStateBetter(StateCandidate, Best)) Best = StateCandidate;
				Next.push_back(std::move(StateCandidate));
			}
		}
		TrimEnvelopeBeam(Next, CET_CIRCLE_GAP_SEARCH_BEAM_WIDTH, AFeatures,
			ABaseCandidate.SkeletonChildCount);
		Beam = std::move(Next);
		if (Attempts >= CET_ELLIPSE_GAP_FILL_MAX_ATTEMPTS || CircleGapSearchTimeReached(SearchStart,
			CET_ELLIPSE_GAP_FILL_MAX_TIME_MS)) break;
	}
	std::cout << "[TEMPLATE][ELLIPSE GAP SEARCH] Class=" << AWindow.ClassKey
		<< " Attempts=" << Attempts << " Fillers=" << Best.FillerCount
		<< " Ms=" << std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::steady_clock::now() - SearchStart).count() << std::endl;
	AOutCandidate = std::move(Best.Candidate);
	return Best.FillerCount > AInitial.FillerCount;
}

bool TryCopyEllipseGapWindowTemplate(const CetTNestItemVector& AOriginalItems,
	const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions,
	const TetClusterCandidate& ABaseCandidate, const TetClusterCandidate& AEnvelopeCandidate,
	const TetEllipseGapWindowTemplate& ATemplate, const TetCircleGapWindow& ATarget,
	const TetClusterCandidate& ACurrent, TetClusterCandidate& AOutCandidate)
{
	AOutCandidate = TetClusterCandidate{};
	if (ATemplate.Transforms.empty() || ATemplate.Source.ClassKey != ATarget.ClassKey) return false;
	if (std::abs(NormalizeClusterAngle(ATarget.Angle - ATemplate.Source.Angle))
		> CET_ELLIPSE_GAP_TEMPLATE_ANGLE_TOLERANCE) return false;
	ET::NEST2DMANAGERLIB::CetRectangleFillClusterBuilder Builder;
	TetClusterCandidate Current = ACurrent;
	std::set<int> Reserved(Current.OriginalIndices.begin(), Current.OriginalIndices.end());
	for (const TetItemTransform& Source : ATemplate.Transforms) {
		const int TargetId = FindAvailableFamilyItem(AFeatures, std::vector<bool>(AFeatures.size(), false),
			Reserved, Source.OriginalId);
		if (TargetId < 0) return false;
		TetItemTransform Transform = Source;
		Transform.OriginalId = TargetId;
		Transform.RelativeX += ATarget.CenterX - ATemplate.Source.CenterX;
		Transform.RelativeY += ATarget.CenterY - ATemplate.Source.CenterY;
		TetClusterCandidate Next;
		if (!Builder.TryAppendFillerTemplateInRectangleEnvelope(AOriginalItems, AFeatures, ABaseCandidate,
			AEnvelopeCandidate, Current, Transform, AOptions, Next)) return false;
		Current = std::move(Next);
		Reserved.insert(TargetId);
	}
	AOutCandidate = std::move(Current);
	return true;
}

bool BuildLocalEllipseGapFilledCandidate(const CetTNestItemVector& AOriginalItems,
	const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions,
	const TetClusterCandidate& ABaseCandidate, const TetClusterCandidate& AEnvelopeCandidate,
	const TetClusterFillSearchConfig& AConfig, TetClusterCandidate& AOutCandidate)
{
	AOutCandidate = AEnvelopeCandidate;
	const std::vector<TetCircleGapWindow> Windows = CollectEllipseGapWindows(AOriginalItems, AFeatures,
		ABaseCandidate, AEnvelopeCandidate);
	const auto SearchStart = std::chrono::steady_clock::now();
	TetClusterFillSearchState Current{ AOutCandidate, 0 };
	TetEllipseGapWindowTemplateCache Templates;
	TetClusterFillSearchConfig LayerConfig = AConfig;
	LayerConfig.MaxDepth = 1;
	for (std::size_t Layer = 0; Layer < CET_ELLIPSE_GAP_FILL_MAX_COMPOSITE_DEPTH; ++Layer) {
		bool FilledLayer = false;
		for (const TetCircleGapWindow& Window : Windows) {
			if (CircleGapSearchTimeReached(SearchStart, CET_ELLIPSE_GAP_FILL_TOTAL_TIME_MS)) break;
			std::string RegionKey;
			if (!BuildEllipseGapRegionSignature(AOriginalItems, AOptions, Current.Candidate, Window, RegionKey)) continue;
			RegionKey += "|layer=" + std::to_string(Layer);
			const auto Cached = Templates.find(RegionKey);
			TetClusterCandidate Copied;
			if (Cached != Templates.end() && TryCopyEllipseGapWindowTemplate(AOriginalItems, AFeatures,
				AOptions, ABaseCandidate, AEnvelopeCandidate, Cached->second, Window, Current.Candidate, Copied)) {
				Current = { std::move(Copied), Current.FillerCount + Cached->second.Transforms.size() };
				FilledLayer = true;
				continue;
			}
			const std::size_t TransformCount = Current.Candidate.Transforms.size();
			TetClusterCandidate Candidate;
			if (!SearchEllipseGapWindow(AOriginalItems, AFeatures, AOptions, ABaseCandidate,
				AEnvelopeCandidate, Window, LayerConfig, Current, Candidate)) continue;
			TetEllipseGapWindowTemplate Template;
			Template.Source = Window;
			Template.Transforms.assign(Candidate.Transforms.begin() + static_cast<std::ptrdiff_t>(TransformCount),
				Candidate.Transforms.end());
			if (!Template.Transforms.empty()) Templates.emplace(RegionKey, std::move(Template));
			Current = { std::move(Candidate), Current.Candidate.Transforms.size() - ABaseCandidate.Transforms.size() };
			FilledLayer = true;
		}
		if (!FilledLayer || CircleGapSearchTimeReached(SearchStart, CET_ELLIPSE_GAP_FILL_TOTAL_TIME_MS)) break;
	}
	AOutCandidate = std::move(Current.Candidate);
	return AOutCandidate.Transforms.size() > ABaseCandidate.Transforms.size();
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
			if (State.FillerCount >= AConfig.MaxDepth) continue;
			std::vector<TetClusterFreeRegion> FreeRegions;
			if (!Geometry.ExtractCandidateFreeRegions(AOriginalItems, AOptions, State.Candidate, FreeRegions) || FreeRegions.empty()) continue;
			AStats.FreeRegionCount += FreeRegions.size();
			const std::vector<int> Fillers = CollectCompatibleFillers(AFeatures, State.Candidate, FreeRegions, AConfig);
			for (int FillerIndex : Fillers) {
				if (AConfig.MaxPlacementAttempts > 0 && AStats.SearchAttempts >= AConfig.MaxPlacementAttempts) break;
				if (ContainsOriginalIndex(State.Candidate, FillerIndex)) continue;
				++AStats.SearchAttempts;
				TetClusterCandidate Candidate;
				if (!Builder.TryAppendFillerInFreeRegions(AOriginalItems, AFeatures, ABaseCandidate, State.Candidate, FreeRegions, FillerIndex, AOptions, Candidate)) continue;
				if (!IsFilledVariantWorthKeeping(ABaseCandidate, Candidate)) continue;
				NextBeam.push_back({ std::move(Candidate), State.FillerCount + 1 });
				++AStats.GeneratedVariantCount;
			}
		}
		if (AConfig.MaxPlacementAttempts > 0 && AStats.SearchAttempts >= AConfig.MaxPlacementAttempts) break;
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

long long QuantizeEllipseTemplateRatio(double AValue)
{
	return std::llround(std::max(0.0, AValue) / CET_ELLIPSE_GAP_TEMPLATE_SIZE_TOLERANCE);
}

std::string BuildEllipseGapTemplateCacheKey(const TetClusterCandidate& ABaseCandidate,
	const std::vector<TetShapeFeature>& AFeatures)
{
	std::vector<long long> AspectRatios;
	for (std::size_t Index = 0; Index < ABaseCandidate.SkeletonChildCount
		&& Index < ABaseCandidate.OriginalIndices.size(); ++Index) {
		const int OriginalIndex = ABaseCandidate.OriginalIndices[Index];
		if (OriginalIndex < 0 || OriginalIndex >= static_cast<int>(AFeatures.size())) continue;
		const TetShapeFeature& Feature = AFeatures[OriginalIndex];
		AspectRatios.push_back(QuantizeEllipseTemplateRatio(
			std::min(Feature.Width, Feature.Height) / std::max(1.0, std::max(Feature.Width, Feature.Height))));
	}
	std::sort(AspectRatios.begin(), AspectRatios.end());
	const double ShortSide = std::max(1.0, std::min(ABaseCandidate.ClusterWidth, ABaseCandidate.ClusterHeight));
	const double LongSide = std::max(ABaseCandidate.ClusterWidth, ABaseCandidate.ClusterHeight);
	std::vector<long long> SkeletonDistances;
	const double Normalizer = std::max(1.0, std::hypot(ABaseCandidate.ClusterWidth, ABaseCandidate.ClusterHeight));
	for (std::size_t First = 0; First < ABaseCandidate.SkeletonChildCount
		&& First < ABaseCandidate.Transforms.size(); ++First) {
		for (std::size_t Second = First + 1; Second < ABaseCandidate.SkeletonChildCount
			&& Second < ABaseCandidate.Transforms.size(); ++Second) {
			const TetItemTransform& FirstTransform = ABaseCandidate.Transforms[First];
			const TetItemTransform& SecondTransform = ABaseCandidate.Transforms[Second];
			SkeletonDistances.push_back(QuantizeEllipseTemplateRatio(std::hypot(
				SecondTransform.RelativeX - FirstTransform.RelativeX,
				SecondTransform.RelativeY - FirstTransform.RelativeY) / Normalizer));
		}
	}
	std::sort(SkeletonDistances.begin(), SkeletonDistances.end());
	std::ostringstream Stream;
	Stream << ABaseCandidate.ClusterType << '|' << ABaseCandidate.SkeletonChildCount << '|'
		<< QuantizeEllipseTemplateRatio(ShortSide / std::max(1.0, LongSide));
	for (long long Ratio : AspectRatios) Stream << '|' << Ratio;
	for (long long Distance : SkeletonDistances) Stream << '|' << Distance;
	return Stream.str();
}

double GetEllipseTemplateFrameAngle(const CetTNestItemVector& AOriginalItems,
	const TetClusterCandidate& ACandidate)
{
	if (ACandidate.SkeletonChildCount < 2 || ACandidate.Transforms.size() < 2) return 0.0;
	ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
	auto GetCenter = [&](const TetItemTransform& Transform, double& AX, double& AY) {
		if (Transform.OriginalId < 0
			|| Transform.OriginalId >= static_cast<int>(AOriginalItems.size())) return false;
		const CetPath Contour = Geometry.TransformContour(Geometry.GetIdentityContour(
			AOriginalItems[Transform.OriginalId]), Transform.RelativeRotation, Transform.RelativeX, Transform.RelativeY);
		double MinX = 0.0, MinY = 0.0, MaxX = 0.0, MaxY = 0.0;
		if (!Geometry.GetBounds(Contour, MinX, MinY, MaxX, MaxY)) return false;
		AX = (MinX + MaxX) * 0.5;
		AY = (MinY + MaxY) * 0.5;
		return true;
	};
	double FirstX = 0.0, FirstY = 0.0, SecondX = 0.0, SecondY = 0.0;
	if (!GetCenter(ACandidate.Transforms[0], FirstX, FirstY)
		|| !GetCenter(ACandidate.Transforms[1], SecondX, SecondY)) return 0.0;
	return std::atan2(SecondY - FirstY, SecondX - FirstX);
}

double NormalizeClusterAngle(double AAngle)
{
	while (AAngle > CET_CLUSTER_PI) AAngle -= CET_CLUSTER_TWO_PI;
	while (AAngle < -CET_CLUSTER_PI) AAngle += CET_CLUSTER_TWO_PI;
	return AAngle;
}

bool BuildEllipseTemplateTransform(const CetTNestItemVector& AOriginalItems,
	const TetEllipseGapTemplate& ATemplate, const TetClusterCandidate& AEnvelopeCandidate,
	const TetItemTransform& ASourceTransform, int ATargetId, double ATargetAngle, bool AMirror,
	double ARotation, TetItemTransform& AOutTransform)
{
	ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
	if (ASourceTransform.OriginalId < 0
		|| ASourceTransform.OriginalId >= static_cast<int>(AOriginalItems.size())
		|| ATargetId < 0 || ATargetId >= static_cast<int>(AOriginalItems.size())) return false;
	const CetPath SourceContour = Geometry.TransformContour(Geometry.GetIdentityContour(
		AOriginalItems[ASourceTransform.OriginalId]), ASourceTransform.RelativeRotation,
		ASourceTransform.RelativeX, ASourceTransform.RelativeY);
	double SourceMinX = 0.0, SourceMinY = 0.0, SourceMaxX = 0.0, SourceMaxY = 0.0;
	if (!Geometry.GetBounds(SourceContour, SourceMinX, SourceMinY, SourceMaxX, SourceMaxY)) return false;
	const double Delta = NormalizeClusterAngle(ATargetAngle - ATemplate.SourceAngle);
	const bool SwapsEnvelopeAxes = std::abs(std::sin(Delta)) > std::abs(std::cos(Delta));
	const double ScaleX = (SwapsEnvelopeAxes ? AEnvelopeCandidate.ClusterHeight
		: AEnvelopeCandidate.ClusterWidth) / ATemplate.EnvelopeWidth;
	const double ScaleY = (SwapsEnvelopeAxes ? AEnvelopeCandidate.ClusterWidth
		: AEnvelopeCandidate.ClusterHeight) / ATemplate.EnvelopeHeight;
	double OffsetX = ((SourceMinX + SourceMaxX) - ATemplate.EnvelopeWidth) * 0.5;
	double OffsetY = ((SourceMinY + SourceMaxY) - ATemplate.EnvelopeHeight) * 0.5;
	if (AMirror) OffsetX = -OffsetX;
	OffsetX *= ScaleX;
	OffsetY *= ScaleY;
	const double Cosine = std::cos(Delta), Sine = std::sin(Delta);
	const double CenterX = AEnvelopeCandidate.ClusterWidth * 0.5 + OffsetX * Cosine - OffsetY * Sine;
	const double CenterY = AEnvelopeCandidate.ClusterHeight * 0.5 + OffsetX * Sine + OffsetY * Cosine;
	const CetPath Rotated = Geometry.TransformContour(Geometry.GetIdentityContour(AOriginalItems[ATargetId]),
		ARotation, 0.0, 0.0);
	double MinX = 0.0, MinY = 0.0, MaxX = 0.0, MaxY = 0.0;
	if (!Geometry.GetBounds(Rotated, MinX, MinY, MaxX, MaxY)) return false;
	AOutTransform.OriginalId = ATargetId;
	AOutTransform.RelativeRotation = ARotation;
	AOutTransform.RelativeX = CenterX - (MinX + MaxX) * 0.5;
	AOutTransform.RelativeY = CenterY - (MinY + MaxY) * 0.5;
	return true;
}

bool TryBuildEllipseTemplateVariant(const CetTNestItemVector& AOriginalItems,
	const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions,
	const TetClusterCandidate& ABaseCandidate, const TetClusterCandidate& AEnvelopeCandidate,
	const TetEllipseGapTemplate& ATemplate, bool AMirror, TetClusterCandidate& AOutCandidate)
{
	AOutCandidate = TetClusterCandidate{};
	if (ATemplate.Transforms.size() <= ATemplate.SkeletonChildCount
		|| ATemplate.SkeletonChildCount != ABaseCandidate.SkeletonChildCount) return false;
	const double TargetAngle = GetEllipseTemplateFrameAngle(AOriginalItems, ABaseCandidate);
	const double Delta = NormalizeClusterAngle(TargetAngle - ATemplate.SourceAngle);
	const std::vector<double> Allowed = ET::NEST2DMANAGERLIB::CetRotationUtils::BuildAllowedRotations(AOptions.Rotations);
	if (Allowed.empty()) return false;
	ET::NEST2DMANAGERLIB::CetRectangleFillClusterBuilder Builder;
	TetClusterCandidate Current = AEnvelopeCandidate;
	std::set<int> Reserved(Current.OriginalIndices.begin(), Current.OriginalIndices.end());
	for (std::size_t Index = ATemplate.SkeletonChildCount; Index < ATemplate.Transforms.size(); ++Index) {
		const TetItemTransform& Source = ATemplate.Transforms[Index];
		const int TargetId = FindAvailableFamilyItem(AFeatures, std::vector<bool>(AFeatures.size(), false), Reserved, Source.OriginalId);
		if (TargetId < 0) return false;
		const double Desired = AMirror ? CET_CLUSTER_PI - Source.RelativeRotation + Delta
			: Source.RelativeRotation + Delta;
		std::vector<double> Rotations = Allowed;
		std::stable_sort(Rotations.begin(), Rotations.end(), [&](double A, double B) {
			const bool AAligned = std::abs(NormalizeClusterAngle(A - Desired))
				<= CET_ELLIPSE_GAP_TEMPLATE_ANGLE_TOLERANCE;
			const bool BAligned = std::abs(NormalizeClusterAngle(B - Desired))
				<= CET_ELLIPSE_GAP_TEMPLATE_ANGLE_TOLERANCE;
			if (AAligned != BAligned) return AAligned;
			return std::abs(NormalizeClusterAngle(A - Desired))
				< std::abs(NormalizeClusterAngle(B - Desired));
		});
		bool Appended = false;
		for (double Rotation : Rotations) {
			TetItemTransform Transform;
			if (!BuildEllipseTemplateTransform(AOriginalItems, ATemplate, AEnvelopeCandidate, Source,
				TargetId, TargetAngle, AMirror, Rotation, Transform)) continue;
			TetClusterCandidate Next;
			if (Builder.TryAppendFillerTemplateInRectangleEnvelope(AOriginalItems, AFeatures, ABaseCandidate,
				AEnvelopeCandidate, Current, Transform, AOptions, Next)) {
				Current = std::move(Next);
				Reserved.insert(TargetId);
				Appended = true;
				break;
			}
		}
		if (!Appended) return false;
	}
	AOutCandidate = std::move(Current);
	return IsEnvelopeFillStateWorthExpanding(AEnvelopeCandidate, AOutCandidate);
}

bool TryBuildCachedEllipseTemplateVariant(const CetTNestItemVector& AOriginalItems,
	const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions,
	const TetClusterCandidate& ABaseCandidate, const TetClusterCandidate& AEnvelopeCandidate,
	const TetEllipseGapTemplateCache& ATemplates, TetClusterCandidate& AOutCandidate)
{
	AOutCandidate = TetClusterCandidate{};
	const auto It = ATemplates.find(BuildEllipseGapTemplateCacheKey(ABaseCandidate, AFeatures));
	if (It == ATemplates.end()) return false;
	const TetEllipseGapTemplate& Template = It->second;
	const double TargetAngle = GetEllipseTemplateFrameAngle(AOriginalItems, ABaseCandidate);
	const double Delta = NormalizeClusterAngle(TargetAngle - Template.SourceAngle);
	const bool SwapsEnvelopeAxes = std::abs(std::sin(Delta)) > std::abs(std::cos(Delta));
	const double WidthRatio = (SwapsEnvelopeAxes ? AEnvelopeCandidate.ClusterHeight
		: AEnvelopeCandidate.ClusterWidth) / std::max(1.0, Template.EnvelopeWidth);
	const double HeightRatio = (SwapsEnvelopeAxes ? AEnvelopeCandidate.ClusterWidth
		: AEnvelopeCandidate.ClusterHeight) / std::max(1.0, Template.EnvelopeHeight);
	if (std::abs(WidthRatio - 1.0) > CET_ELLIPSE_GAP_TEMPLATE_SIZE_TOLERANCE
		|| std::abs(HeightRatio - 1.0) > CET_ELLIPSE_GAP_TEMPLATE_SIZE_TOLERANCE) return false;
	if (TryBuildEllipseTemplateVariant(AOriginalItems, AFeatures, AOptions, ABaseCandidate,
		AEnvelopeCandidate, Template, false, AOutCandidate)) return true;
	return TryBuildEllipseTemplateVariant(AOriginalItems, AFeatures, AOptions, ABaseCandidate,
		AEnvelopeCandidate, Template, true, AOutCandidate);
}

void CacheEllipseTemplateVariant(const CetTNestItemVector& AOriginalItems,
	const TetClusterCandidate& ABaseCandidate, const TetClusterCandidate& AEnvelopeCandidate,
	const std::vector<TetShapeFeature>& AFeatures, const std::vector<TetClusterFillSearchState>& AStates,
	bool AHasValidatedVariant,
	TetEllipseGapTemplateCache& ATemplates)
{
	if (!AHasValidatedVariant || AStates.empty()) return;
	const TetClusterFillSearchState* Best = &AStates.front();
	for (const TetClusterFillSearchState& State : AStates) {
		if (IsEnvelopeStateBetter(State, *Best)) Best = &State;
	}
	if (Best->Candidate.Transforms.size() <= ABaseCandidate.SkeletonChildCount) return;
	const std::string Key = BuildEllipseGapTemplateCacheKey(ABaseCandidate, AFeatures);
	if (ATemplates.find(Key) != ATemplates.end()) return;
	TetEllipseGapTemplate Template;
	Template.SourceAngle = GetEllipseTemplateFrameAngle(AOriginalItems, ABaseCandidate);
	Template.EnvelopeWidth = AEnvelopeCandidate.ClusterWidth;
	Template.EnvelopeHeight = AEnvelopeCandidate.ClusterHeight;
	Template.SkeletonChildCount = ABaseCandidate.SkeletonChildCount;
	const std::size_t TransformCount = std::min(Best->Candidate.Transforms.size(),
		Template.SkeletonChildCount + CET_ELLIPSE_GAP_TEMPLATE_MAX_COPIES);
	Template.Transforms.assign(Best->Candidate.Transforms.begin(),
		Best->Candidate.Transforms.begin() + TransformCount);
	ATemplates.emplace(Key, std::move(Template));
	std::cout << "[TEMPLATE][ELLIPSE GAP CACHE STORE] Fillers="
		<< TransformCount - ABaseCandidate.SkeletonChildCount << std::endl;
}

TetClusterFillSearchState BuildEnvelopeFillSeed(const CetTNestItemVector& AOriginalItems,
	const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions,
	const TetClusterCandidate& ABaseCandidate, const TetClusterCandidate& AEnvelopeCandidate,
	std::map<std::string, TetCircleGapTemplate>& AGapTemplates,
	const TetClusterFillSearchState* ASeedState, bool AIsCircleBase, bool AIsEllipseBase,
	const TetClusterFillSearchConfig& AConfig, std::vector<TetClusterFillSearchState>& AOutStates)
{
	AOutStates.clear();
	if (ASeedState != nullptr && PreservesBaseTransforms(ABaseCandidate, ASeedState->Candidate)
		&& IsEnvelopeFillStateWorthExpanding(AEnvelopeCandidate, ASeedState->Candidate)) {
		AOutStates.push_back(*ASeedState);
	}
	TetClusterCandidate LocalGap;
	if (AIsCircleBase && BuildLocalCircleGapFilledCandidate(AOriginalItems, AFeatures, AOptions,
		ABaseCandidate, AEnvelopeCandidate, AGapTemplates, LocalGap)
		&& PreservesBaseTransforms(ABaseCandidate, LocalGap)
		&& IsEnvelopeFillStateWorthExpanding(AEnvelopeCandidate, LocalGap)) {
		AOutStates.push_back({ LocalGap, LocalGap.Transforms.size() - ABaseCandidate.Transforms.size() });
		std::cout << "[TEMPLATE][LOCAL GAP] Fillers=" << AOutStates.back().FillerCount << std::endl;
	}
	TetClusterCandidate EllipseGap;
	if (AIsEllipseBase && BuildLocalEllipseGapFilledCandidate(AOriginalItems, AFeatures, AOptions,
		ABaseCandidate, AEnvelopeCandidate, AConfig, EllipseGap)
		&& PreservesBaseTransforms(ABaseCandidate, EllipseGap)
		&& IsEnvelopeFillStateWorthExpanding(AEnvelopeCandidate, EllipseGap)) {
		const std::size_t FillerCount = EllipseGap.Transforms.size() > ABaseCandidate.Transforms.size()
			? EllipseGap.Transforms.size() - ABaseCandidate.Transforms.size() : 0;
		AOutStates.push_back({ std::move(EllipseGap), FillerCount });
		std::cout << "[TEMPLATE][ELLIPSE LOCAL GAP] Fillers="
			<< AOutStates.back().FillerCount << std::endl;
	}
	TetClusterFillSearchState Initial{ AEnvelopeCandidate, 0 };
	if (!AOutStates.empty()) {
		Initial = *std::max_element(AOutStates.begin(), AOutStates.end(),
			[](const TetClusterFillSearchState& AFirst, const TetClusterFillSearchState& ASecond) {
				return IsEnvelopeStateBetter(ASecond, AFirst);
			});
	}
	std::cout << "[TEMPLATE][ELLIPSE SEARCH SEED] IsEllipse=" << AIsEllipseBase
		<< ", States=" << AOutStates.size() << ", Fillers=" << Initial.FillerCount << std::endl;
	return Initial;
}

void SearchEnvelopeFillVariants(const CetTNestItemVector& AOriginalItems,
	const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions,
	const TetClusterCandidate& ABaseCandidate, const TetClusterCandidate& AEnvelopeCandidate,
	const TetClusterFillSearchConfig& AConfig, std::map<std::string, TetCircleGapTemplate>& AGapTemplates,
	const TetClusterFillSearchState* ASeedState, std::vector<TetClusterFillSearchState>& AOutStates,
	TetClusterFillSearchStats& AStats)
{
	std::size_t LocalAttempts = 0;
	const bool IsCircleBase = IsFixedCircleEnvelopeBase(ABaseCandidate, AFeatures);
	const std::vector<TetCircleGapTemplateAnchor> Anchors = IsCircleBase
		? CollectCircleGapTemplateAnchors(AOriginalItems, AFeatures, ABaseCandidate)
		: std::vector<TetCircleGapTemplateAnchor>{};
	ET::NEST2DMANAGERLIB::CetRectangleFillClusterBuilder Builder;
	ET::NEST2DMANAGERLIB::CetClusterGeometryHelper Geometry;
	AOutStates.clear();
	const bool IsEllipseBase = IsFixedEllipseEnvelopeBase(ABaseCandidate, AFeatures);
	const TetClusterFillSearchState Initial = BuildEnvelopeFillSeed(AOriginalItems, AFeatures,
		AOptions, ABaseCandidate, AEnvelopeCandidate, AGapTemplates, ASeedState,
		IsCircleBase, IsEllipseBase, AConfig, AOutStates);
	const auto SearchStart = std::chrono::steady_clock::now();
	auto TimeLimitReached = [&]() {
		const long long Limit = IsEllipseBase && AConfig.MaxElapsedMs > 0
			? std::min(AConfig.MaxElapsedMs, CET_ELLIPSE_GAP_FILL_GENERIC_TIME_MS)
			: AConfig.MaxElapsedMs;
		return CircleGapSearchTimeReached(SearchStart, Limit);
	};
	// Keep every distinct seed long enough for the fixed-envelope search to
	// compare a locally dense layout with a less dense layout that may leave a
	// better-shaped pocket for the remaining inventory.
	std::vector<TetClusterFillSearchState> Beam = AOutStates;
	Beam.push_back(Initial);
	DeduplicateFilledStates(Beam);
	TrimEnvelopeBeam(Beam, AConfig.BeamWidth, AFeatures,
		ABaseCandidate.SkeletonChildCount, IsEllipseBase);
	const std::size_t MaxFillers = ABaseCandidate.Transforms.size() + AConfig.MaxDepth
		+ CET_ELLIPSE_GAP_FILL_MAX_COMPOSITE_DEPTH;
	AOutStates.insert(AOutStates.end(), Beam.begin(), Beam.end());
	for (std::size_t Depth = 0; Depth < AConfig.MaxDepth && !Beam.empty(); ++Depth) {
		if (TimeLimitReached()) { ++AStats.EnvelopeTimeLimitHits; break; }
		std::vector<TetClusterFillSearchState> Next;
		for (const TetClusterFillSearchState& State : Beam) {
			if (State.FillerCount >= MaxFillers) continue;
			std::vector<TetClusterFreeRegion> StateFreeRegions;
			if (!Geometry.ExtractCandidateFreeRegions(AOriginalItems, AOptions, State.Candidate,
				StateFreeRegions) || StateFreeRegions.empty()) continue;
			AStats.EnvelopeFreeRegionCount += StateFreeRegions.size();
			const std::vector<int> Fillers = CollectCompatibleFillers(AFeatures, State.Candidate,
				StateFreeRegions, AConfig, true);
			for (int Filler : Fillers) {
				if ((AConfig.MaxPlacementAttempts > 0 && LocalAttempts >= AConfig.MaxPlacementAttempts)
					|| TimeLimitReached()) break;
				if (ContainsOriginalIndex(State.Candidate, Filler)) continue;
				++LocalAttempts; ++AStats.EnvelopeSearchAttempts;
				TetClusterCandidate Candidate;
				if (!Builder.TryAppendFillerInRectangleEnvelope(AOriginalItems, AFeatures, ABaseCandidate,
					AEnvelopeCandidate, State.Candidate, StateFreeRegions, Filler, AOptions, Candidate)) continue;
				std::size_t Copies = 0;
				TetClusterCandidate Copied;
				const std::size_t Remaining = MaxFillers > State.FillerCount + 1 ? MaxFillers - State.FillerCount - 1 : 0;
				if (IsCircleBase && TryCopyCircleGapTemplate(AOriginalItems, AFeatures, AOptions, ABaseCandidate,
					AEnvelopeCandidate, Anchors, Candidate, Remaining, Copied, Copies)) Candidate = std::move(Copied);
				if (!PreservesBaseTransforms(ABaseCandidate, Candidate)
					|| !IsEnvelopeFillStateWorthExpanding(AEnvelopeCandidate, Candidate)) continue;
				Next.push_back({ std::move(Candidate), State.FillerCount + 1 + Copies });
				++AStats.EnvelopeGeneratedVariantCount;
			}
		}
		DeduplicateFilledStates(Next);
		TrimEnvelopeBeam(Next, AConfig.BeamWidth, AFeatures, ABaseCandidate.SkeletonChildCount,
			IsEllipseBase);
		AOutStates.insert(AOutStates.end(), Next.begin(), Next.end());
		Beam = std::move(Next);
		AStats.EnvelopeMaxDepthReached = std::max(AStats.EnvelopeMaxDepthReached, Depth + 1);
		if (AConfig.MaxPlacementAttempts > 0 && LocalAttempts >= AConfig.MaxPlacementAttempts
			|| TimeLimitReached()) {
			if (TimeLimitReached()) ++AStats.EnvelopeTimeLimitHits;
			break;
		}
	}
	AStats.EnvelopeSearchMs += std::chrono::duration<double, std::milli>(
		std::chrono::steady_clock::now() - SearchStart).count();
}

bool TryBuildCachedEllipseTemplateSeed(const CetTNestItemVector& AOriginalItems,
	const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions,
	const TetClusterCandidate& ABaseCandidate, const TetClusterCandidate& AEnvelopeCandidate,
	const TetEllipseGapTemplateCache& ATemplates, TetClusterFillSearchState& AOutSeed)
{
	AOutSeed = TetClusterFillSearchState{};
	TetClusterCandidate Candidate;
	if (!TryBuildCachedEllipseTemplateVariant(AOriginalItems, AFeatures, AOptions, ABaseCandidate,
		AEnvelopeCandidate, ATemplates, Candidate)) return false;
	AOutSeed.Candidate = std::move(Candidate);
	AOutSeed.FillerCount = AOutSeed.Candidate.Transforms.size() - ABaseCandidate.Transforms.size();
	return true;
}

void BuildEnvelopeFilledVariantsForBase(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions, const TetClusterCandidate& ABaseCandidate, const TetClusterFillSearchConfig& AConfig, std::map<std::string, TetCircleGapTemplate>& AGapTemplates, TetEllipseGapTemplateCache& AEllipseTemplates, std::vector<TetClusterCandidate>& AOutVariants, TetClusterFillSearchStats& AStats)
{
	AOutVariants.clear();
	TetClusterCandidate EnvelopeCandidate;
	if (!BuildRectangleEnvelopeCandidate(AOriginalItems, AOptions, ABaseCandidate, EnvelopeCandidate)
		|| EnvelopeCandidate.ProxyWasteArea <= 0.0) return;
	const bool IsEllipseBase = IsFixedEllipseEnvelopeBase(ABaseCandidate, AFeatures);
	TetClusterFillSearchState CachedSeed;
	if (IsEllipseBase && TryBuildCachedEllipseTemplateSeed(AOriginalItems, AFeatures, AOptions,
		ABaseCandidate, EnvelopeCandidate, AEllipseTemplates, CachedSeed)) {
		std::cout << "[TEMPLATE][ELLIPSE GAP CACHE SEED] Fillers="
			<< CachedSeed.FillerCount << std::endl;
	}
	std::vector<TetClusterFillSearchState> States;
	SearchEnvelopeFillVariants(AOriginalItems, AFeatures, AOptions, ABaseCandidate, EnvelopeCandidate,
		AConfig, AGapTemplates, CachedSeed.Candidate.Valid ? &CachedSeed : nullptr, States, AStats);
	FinalizeEnvelopeFilledStates(AOriginalItems, AFeatures, AOptions, ABaseCandidate, States, AOutVariants, AStats);
	if (IsEllipseBase) CacheEllipseTemplateVariant(AOriginalItems, ABaseCandidate, EnvelopeCandidate,
		AFeatures, States, !AOutVariants.empty(), AEllipseTemplates);
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

		void CetClusterManager::ExpandClusterResultToOriginalItems(const CetTNestItemVector& AOriginalItems, const CetTNestItemVector& APackedItems, const std::vector<TetMetaItem>& AMetaItems, CetTNestItemVector& AOutOriginalItems, bool ALog)
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
				if (ALog) {
					std::cout << "[CLUSTER][EXPAND PACKED] PackedIndex = " << PackedIndex << ", IsCluster = " << Meta.IsCluster << ", PackedBin = " << PackedItem.binId() << ", PackedX = " << PackedX << ", PackedY = " << PackedY << ", PackedRotation = " << PackedRotation << ", Children = " << Meta.TransformData.size() << std::endl;
				}
				_ExpandClusterChildren(PackedItem, Meta, AOutOriginalItems, ALog);
			}
			if (ALog) {
				std::cout << "[CLUSTER] ExpandClusterResultToOriginalItems done. Original count = " << AOutOriginalItems.size() << std::endl;
			}
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
			std::vector<TetClusterCandidate> AcceptedCandidates = _SelectAndOptimizeTemplateCandidates(AOriginalItems, AFeatures, AOptions, ExpandedCandidates, Used, Count);
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
			const bool HasFixedEnvelopeBase = std::any_of(ABaseCandidates.begin(), ABaseCandidates.end(),
				[&](const TetClusterCandidate& Candidate) {
					return IsFixedCircleEnvelopeBase(Candidate, AFeatures)
						|| IsFixedEllipseEnvelopeBase(Candidate, AFeatures);
				});
			for (const TetClusterCandidate& BaseCandidate : ABaseCandidates) {
				if (!BaseCandidate.Valid) continue;
				++ValidBaseCandidateCount;
				Stats.BaseFillRatioSum += BaseCandidate.FillRatio;
				if (HasFixedEnvelopeBase || SkipsGenericTemplateFill(BaseCandidate)) continue;
				std::vector<TetClusterCandidate> Variants;
				BuildFilledVariantsForBase(AOriginalItems, AFeatures, AOptions, BaseCandidate, Config, Variants, Stats);
				if (!Variants.empty()) {
					FilledVariantCount += Variants.size();
					std::cout << "[TEMPLATE][FILL VARIANT] BaseType=" << BaseCandidate.ClusterType << " Generated=" << Variants.size() << " BaseFillRatio=" << BaseCandidate.FillRatio << " BestFillRatio=" << Variants.front().FillRatio << std::endl;
					AOutCandidates.insert(AOutCandidates.end(), Variants.begin(), Variants.end());
				}
			}
			const TetClusterFillSearchConfig EnvelopeConfig = GetClusterEnvelopeFillSearchConfig(AOriginalItems.size());
			TetCircleGapTemplateCache CircleGapTemplateCache;
			TetEllipseGapTemplateCache EllipseGapTemplateCache;
			std::vector<const TetClusterCandidate*> EnvelopeBaseCandidates;
			EnvelopeBaseCandidates.reserve(ABaseCandidates.size());
			for (const TetClusterCandidate& BaseCandidate : ABaseCandidates) {
				if ((IsFixedCircleEnvelopeBase(BaseCandidate, AFeatures)
					|| IsFixedEllipseEnvelopeBase(BaseCandidate, AFeatures))
					&& !HasFullRectangleProxy(BaseCandidate)) {
					EnvelopeBaseCandidates.push_back(&BaseCandidate);
				}
			}
			std::stable_sort(EnvelopeBaseCandidates.begin(), EnvelopeBaseCandidates.end(), [](const TetClusterCandidate* AFirst, const TetClusterCandidate* ASecond) {
				const bool FirstIsEllipse = AFirst->BuilderName == "EllipseBuilder";
				const bool SecondIsEllipse = ASecond->BuilderName == "EllipseBuilder";
				if (FirstIsEllipse != SecondIsEllipse) return FirstIsEllipse;
				const double FirstAvailableArea = std::max(0.0, AFirst->BoundingBoxArea - AFirst->ReservedArea);
				const double SecondAvailableArea = std::max(0.0, ASecond->BoundingBoxArea - ASecond->ReservedArea);
				if (std::abs(FirstAvailableArea - SecondAvailableArea) > 1.0) return FirstAvailableArea > SecondAvailableArea;
				return AFirst->ClusterType < ASecond->ClusterType;
				});
			const bool HasEllipseEnvelopeBase = std::any_of(EnvelopeBaseCandidates.begin(),
				EnvelopeBaseCandidates.end(), [&](const TetClusterCandidate* Candidate) {
					return IsFixedEllipseEnvelopeBase(*Candidate, AFeatures);
				});
			const std::size_t EnvelopeBaseLimit = HasEllipseEnvelopeBase
				? (AOriginalItems.size() > CET_NEST_FULL_STRATEGY_ITEM_LIMIT
					? CET_ELLIPSE_GAP_FILL_LARGE_ORDER_MAX_BASE_CANDIDATES
					: CET_ELLIPSE_GAP_FILL_MAX_BASE_CANDIDATES)
				: (AOriginalItems.size() > CET_NEST_FULL_STRATEGY_ITEM_LIMIT
					? CET_CLUSTER_ENVELOPE_FILL_LARGE_ORDER_MAX_BASE_CANDIDATES
					: CET_CLUSTER_ENVELOPE_FILL_MAX_BASE_CANDIDATES);
			if (EnvelopeBaseCandidates.size() > EnvelopeBaseLimit) {
				EnvelopeBaseCandidates.resize(EnvelopeBaseLimit);
			}
			for (const TetClusterCandidate* BaseCandidate : EnvelopeBaseCandidates) {
				std::vector<TetClusterCandidate> Variants;
				std::map<std::string, TetCircleGapTemplate>& GapTemplates =
					CircleGapTemplateCache[BuildCircleGapTemplateCacheKey(*BaseCandidate)];
				BuildEnvelopeFilledVariantsForBase(AOriginalItems, AFeatures, AOptions, *BaseCandidate,
					EnvelopeConfig, GapTemplates, EllipseGapTemplateCache, Variants, Stats);
				if (Variants.empty()) continue;
				const TetClusterCandidate& BestVariant = Variants.front();
				std::cout << "[TEMPLATE][ENVELOPE FILL] BaseType=" << BaseCandidate->ClusterType
					<< " Generated=" << Variants.size()
					<< " BaseEnvelopeFill=" << BaseCandidate->BoundingFillRatio
					<< " BestFillRatio=" << BestVariant.FillRatio
					<< " EnvelopeFillGain=" << (BestVariant.FillRatio - BaseCandidate->BoundingFillRatio)
					<< " ProxyMode=" << ToString(BestVariant.ProxyMode)
					<< " ProxyArea=" << BestVariant.ProxyArea
					<< " FillerCount=" << BestVariant.OriginalIndices.size() - BaseCandidate->OriginalIndices.size()
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
				<< " EnvelopeTimeLimitHits=" << Stats.EnvelopeTimeLimitHits
				<< " EnvelopeMaxDepthReached=" << Stats.EnvelopeMaxDepthReached
				<< " EnvelopeBestFillerCount=" << Stats.EnvelopeBestFillerCount
				<< " EnvelopeSearchMs=" << Stats.EnvelopeSearchMs
				<< " EnvelopeTrueContourMs=" << Stats.EnvelopeTrueContourMs
				<< " BestEnvelopeFillRatioGain=" << Stats.BestEnvelopeFillRatioGain
				<< " BestEnvelopeRectangleFillRatio=" << Stats.BestEnvelopeRectangleFillRatio
				<< " EnvelopeBeamWidth=" << EnvelopeConfig.BeamWidth
				<< " EnvelopeMaxDepth=" << EnvelopeConfig.MaxDepth
				<< " EnvelopeMaxFillers=" << EnvelopeConfig.MaxCandidateFillers
				<< " EnvelopeMaxPlacementAttempts=" << EnvelopeConfig.MaxPlacementAttempts
				<< " BeamWidth=" << Config.BeamWidth
				<< " MaxDepth=" << Config.MaxDepth
				<< " MaxFillers=" << Config.MaxCandidateFillers
				<< " MaxPlacementAttempts=" << Config.MaxPlacementAttempts << std::endl;
		}

		std::vector<TetClusterCandidate> CetClusterManager::_SelectTemplateCandidates(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions, const std::vector<TetClusterCandidate>& ABaseCandidates, std::vector<bool>& AUsed)
		{
			const int Count = static_cast<int>(AOriginalItems.size());
			std::vector<TetClusterCandidate> SortedCandidates = ABaseCandidates;
			auto CountFillerFamilies = [&](const TetClusterCandidate& Candidate) {
				std::set<std::uint64_t> Families;
				const std::size_t Start = std::min(Candidate.SkeletonChildCount, Candidate.OriginalIndices.size());
				for (std::size_t Index = Start; Index < Candidate.OriginalIndices.size(); ++Index) {
					const int OriginalIndex = Candidate.OriginalIndices[Index];
					if (OriginalIndex >= 0 && OriginalIndex < static_cast<int>(AFeatures.size())) {
						Families.insert(MakeFillerFamilyKey(AFeatures[OriginalIndex]));
					}
				}
				return Families.size();
			};

			std::stable_sort(SortedCandidates.begin(), SortedCandidates.end(), [&](const TetClusterCandidate& A, const TetClusterCandidate& AB) {
				const bool ACompletedEnvelopeFill = IsCompletedEnvelopeFill(A);
				const bool BCompletedEnvelopeFill = IsCompletedEnvelopeFill(AB);
				if (ACompletedEnvelopeFill != BCompletedEnvelopeFill) {
					return ACompletedEnvelopeFill;
				}
				if (ACompletedEnvelopeFill && BCompletedEnvelopeFill) {
					const std::size_t AFillCount = A.OriginalIndices.size() - std::min(A.SkeletonChildCount, A.OriginalIndices.size());
					const std::size_t BFillCount = AB.OriginalIndices.size() - std::min(AB.SkeletonChildCount, AB.OriginalIndices.size());
					if (A.SkeletonChildCount == AB.SkeletonChildCount && AFillCount == BFillCount) {
						const std::size_t AFamilyCount = CountFillerFamilies(A);
						const std::size_t BFamilyCount = CountFillerFamilies(AB);
						if (AFamilyCount != BFamilyCount) return AFamilyCount > BFamilyCount;
					}
					return IsCompletedEnvelopeFillBetter(A, AB);
				}
				const bool ARegularSkeleton = IsInventorySkeletonCandidate(A);
				const bool BRegularSkeleton = IsInventorySkeletonCandidate(AB);
				if (ARegularSkeleton != BRegularSkeleton) {
					return ARegularSkeleton;
				}
				const bool ACircleSkeleton = IsCircleSkeletonCandidate(A);
				const bool BCircleSkeleton = IsCircleSkeletonCandidate(AB);
				if (ACircleSkeleton != BCircleSkeleton) {
					return ACircleSkeleton;
				}
				if (ARegularSkeleton && BRegularSkeleton
					&& A.SkeletonChildCount != AB.SkeletonChildCount) {
					return A.SkeletonChildCount > AB.SkeletonChildCount;
				}
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
		std::vector<TetClusterCandidate> DeferredTriangles;
		for (const TetClusterCandidate& Candidate : SortedCandidates) {
			if (IsDeferredTriangleCandidate(Candidate)) {
				DeferredTriangles.push_back(Candidate);
				continue;
			}
			TetClusterCandidate BoundCandidate;
				if (!TryBindCandidateInventory(Candidate, AFeatures, AUsed, BoundCandidate)
					|| !_CanAcceptClusterCandidate(AOriginalItems, AOptions, BoundCandidate, AUsed, Count)) {
					std::cout << "[TEMPLATE][REJECT] Builder=" << Candidate.BuilderName << " Type=" << Candidate.ClusterType << " Score=" << Candidate.Score << std::endl;
					continue;
				}

				AcceptedCandidates.push_back(std::move(BoundCandidate));
				for (int OriginalIndex : AcceptedCandidates.back().OriginalIndices) {
					AUsed[OriginalIndex] = true;
				}

				std::cout << "[TEMPLATE][BASE ACCEPT] Builder=" << Candidate.BuilderName << " Type=" << Candidate.ClusterType << " ChildCount=" << Candidate.OriginalIndices.size() << " Score=" << Candidate.Score << std::endl;
			}

			RebalanceAcceptedClusterInventory(AOriginalItems, AFeatures, AOptions, AcceptedCandidates, AUsed);
		const bool DeferRemainingTriangles = HasFilledEllipseCandidate(AcceptedCandidates);
		for (const TetClusterCandidate& Candidate : DeferredTriangles) {
			if (DeferRemainingTriangles) {
				std::cout << "[TEMPLATE][DEFERRED TRIANGLE SINGLE] Type="
					<< Candidate.ClusterType << std::endl;
				continue;
			}
			TetClusterCandidate BoundCandidate;
			if (!TryBindCandidateInventory(Candidate, AFeatures, AUsed, BoundCandidate)
				|| !_CanAcceptClusterCandidate(AOriginalItems, AOptions, BoundCandidate, AUsed, Count)) {
				std::cout << "[TEMPLATE][DEFERRED TRIANGLE REJECT] Type="
					<< Candidate.ClusterType << std::endl;
				continue;
			}
			AcceptedCandidates.push_back(std::move(BoundCandidate));
			for (int OriginalIndex : AcceptedCandidates.back().OriginalIndices) {
				AUsed[OriginalIndex] = true;
			}
			std::cout << "[TEMPLATE][DEFERRED TRIANGLE ACCEPT] Type="
				<< Candidate.ClusterType << " ChildCount="
				<< Candidate.OriginalIndices.size() << std::endl;
		}
			return AcceptedCandidates;
		}

		std::vector<TetClusterCandidate> CetClusterManager::_SelectAndOptimizeTemplateCandidates(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions, const std::vector<TetClusterCandidate>& ABaseCandidates, std::vector<bool>& AUsed, int AOriginalItemCount)
		{
#ifdef _DEBUG
			const auto GreedyStartTime = std::chrono::steady_clock::now();
#endif
			std::vector<TetClusterCandidate> AcceptedCandidates = _SelectTemplateCandidates(AOriginalItems, AFeatures, AOptions, ABaseCandidates, AUsed);
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
