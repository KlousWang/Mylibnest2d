#include "pch.h"
#include "Nest2D_ClusterTemplateFillOptimizer.h"
#include "Nest2D_ClusterInventoryRebalancer.h"
#include "Nest2D_ClusterFillSearchEngine.h"
#include "Nest2D_TemplateCandidateGenerator.h"
#include "Nest2D_CircleGapFiller.h"
#include "Nest2D_ArcClusterBuilder.h"
#include "Nest2D_CircleClusterBuilder.h"
#include "Nest2D_ClusterBoundary.h"
#include "Nest2D_ClusterGeometryHelper.h"
#include "Nest2D_CustomClusterBuilder.h"
#include "Nest2D_EllipseClusterBuilder.h"
#include "Nest2D_RectangleClusterBuilder.h"
#include "Nest2D_RectangleFillClusterBuilder.h"
#include "Nest2D_RotationUtils.h"
#include "Nest2D_SelfFunction.h"
#include "Nest2D_TriangleClusterBuilder.h"
#include "Nest2D_EllipseGapFiller.h"
#include "NestUtils.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>

using namespace ClipperLib;
using namespace libnest2d;

namespace ET { namespace NEST2DMANAGERLIB {
    CetNest2DInvokeFunctor *_GetNest2DInvokeFunctor()
    {
        return Nest2DUtils;
    }
}}

namespace {
    bool CircleGapSearchTimeReached(const std::chrono::steady_clock::time_point &AStart, long long ALimitMs)
    {
        return ALimitMs > 0 && std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - AStart).count() >= ALimitMs;
    }
    bool SkipsGenericTemplateFill(const TetClusterCandidate &ACandidate) { return ACandidate.BuilderName == "CircleBuilder" || ACandidate.BuilderName == "EllipseBuilder" || ACandidate.BuilderName == "ArcBuilder"; }
    bool IsFixedCircleEnvelopeBase(const TetClusterCandidate &ACandidate, const std::vector<TetShapeFeature> &AFeatures)
    {
        if (!ACandidate.Valid || ACandidate.BuilderName != "CircleBuilder" || ACandidate.OriginalIndices.size() < 2)
            return false;
        for (int Index : ACandidate.OriginalIndices) {
            if (Index < 0 || Index >= static_cast<int>(AFeatures.size()) || AFeatures[Index].ShapeType != MetShapeType::CircleLike)
                return false;
        }
        return true;
    }
    bool IsFixedEllipseEnvelopeBase(const TetClusterCandidate &ACandidate, const std::vector<TetShapeFeature> &AFeatures)
    {
        if (!ACandidate.Valid || ACandidate.BuilderName != "EllipseBuilder" || ACandidate.OriginalIndices.size() < 2)
            return false;
        for (int Index : ACandidate.OriginalIndices) {
            if (Index < 0 || Index >= static_cast<int>(AFeatures.size()) || AFeatures[Index].ShapeType != MetShapeType::EllipseLike)
                return false;
        }
        return true;
    }
    bool IsCompletedEnvelopeFill(const TetClusterCandidate &ACandidate) { return ACandidate.BuilderName == "EnvelopeFillSearch" && ACandidate.ProxyMode != MetClusterProxyMode::Unknown && ACandidate.OriginalIndices.size() >= 3; }
    bool IsCircleSkeletonCandidate(const TetClusterCandidate &ACandidate) { return ACandidate.SkeletonChildCount >= 2 && (ACandidate.BuilderName == "CircleBuilder" || ACandidate.ClusterType.find("Circle") == 0); }
    bool IsEllipseSkeletonCandidate(const TetClusterCandidate &ACandidate) { return ACandidate.SkeletonChildCount >= 2 && (ACandidate.BuilderName == "EllipseBuilder" || ACandidate.ClusterType.find("Ellipse") == 0); }
    bool IsInventorySkeletonCandidate(const TetClusterCandidate &ACandidate) { return IsCircleSkeletonCandidate(ACandidate) || IsEllipseSkeletonCandidate(ACandidate); }
    bool IsCompletedEnvelopeFillBetter(const TetClusterCandidate &AFirst, const TetClusterCandidate &ASecond)
    {
        // A circle framework is the virtual board for its fillers.  Keep the
        // largest complete framework intact before comparing local filler gains;
        // otherwise an 8-circle frame with one extra small part can consume the
        // inventory ahead of a 12-circle frame and fragment the intended layout.
        if (AFirst.SkeletonChildCount != ASecond.SkeletonChildCount) {
            return AFirst.SkeletonChildCount > ASecond.SkeletonChildCount;
        }
        const std::size_t FirstFillers = AFirst.OriginalIndices.size() - AFirst.SkeletonChildCount;
        const std::size_t SecondFillers = ASecond.OriginalIndices.size() - ASecond.SkeletonChildCount;
        if (FirstFillers != SecondFillers)
            return FirstFillers > SecondFillers;
        if (std::abs(AFirst.FillRatio - ASecond.FillRatio) > 1e-9) {
            return AFirst.FillRatio > ASecond.FillRatio;
        }
        if (std::abs(AFirst.FragmentationRisk - ASecond.FragmentationRisk) > 1e-9) {
            return AFirst.FragmentationRisk < ASecond.FragmentationRisk;
        }
        if (std::abs(AFirst.SheetReuseScore - ASecond.SheetReuseScore) > 1e-9) {
            return AFirst.SheetReuseScore > ASecond.SheetReuseScore;
        }
        if (std::abs(AFirst.Score - ASecond.Score) > 1e-9)
            return AFirst.Score > ASecond.Score;
        return AFirst.ProxyArea < ASecond.ProxyArea;
    }
    std::string BuildCircleGapTemplateCacheKey(const TetClusterCandidate &ABaseCandidate)
    {
        std::ostringstream Stream;
        Stream << ABaseCandidate.BuilderName << '|' << ABaseCandidate.ClusterType << '|' << ABaseCandidate.SkeletonChildCount << '|' << std::llround(ABaseCandidate.ClusterWidth / CET_RECTANGLE_FILL_POSITION_TOLERANCE) << '|' << std::llround(ABaseCandidate.ClusterHeight / CET_RECTANGLE_FILL_POSITION_TOLERANCE);
        return Stream.str();
    }
    bool IsDeferredTriangleCandidate(const TetClusterCandidate &ACandidate) { return ACandidate.BuilderName == "TriangleBuilder"; }
    bool HasFilledEllipseCandidate(const std::vector<TetClusterCandidate> &ACandidates)
    {
        for (const TetClusterCandidate &Candidate : ACandidates) {
            if (Candidate.ClusterType.find("Ellipse") != std::string::npos && Candidate.OriginalIndices.size() > Candidate.SkeletonChildCount)
                return true;
        }
        return false;
    }
    TetPairCandidateKey MakePairCandidateKey(int AFirst, int ASecond)
    {
        if (AFirst > ASecond) {
            std::swap(AFirst, ASecond);
        }
        return {AFirst, ASecond};
    }
    using TPairCandidateLookup = std::unordered_map<TetPairCandidateKey, const TetClusterCandidate *, TetPairCandidateKeyHash>;
    bool IsPairCandidateUsable(const TetClusterCandidate &ACandidate, int AOriginalItemCount)
    {
        if (!ACandidate.Valid || ACandidate.OriginalIndices.size() != 2 || ACandidate.Transforms.size() != 2 || ACandidate.ProxyContour.size() < 3 || ACandidate.ClusterWidth <= 0.0 || ACandidate.ClusterHeight <= 0.0 || ACandidate.ProxyArea <= 0.0 || !std::isfinite(ACandidate.Score)) {
            return false;
        }
        const int FirstIndex = ACandidate.OriginalIndices[0];
        const int SecondIndex = ACandidate.OriginalIndices[1];
        return FirstIndex >= 0 && SecondIndex >= 0 && FirstIndex < AOriginalItemCount && SecondIndex < AOriginalItemCount && FirstIndex != SecondIndex;
    }
    void BuildPairCandidateLookup(const std::vector<TetClusterCandidate> &ACandidates, int AOriginalItemCount, TPairCandidateLookup &AOutLookup)
    {
        AOutLookup.clear();
        AOutLookup.reserve(ACandidates.size());
        for (const TetClusterCandidate &Candidate : ACandidates) {
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
    std::vector<std::size_t> CollectPairCandidatePositions(const std::vector<TetClusterCandidate> &ACandidates)
    {
        std::vector<std::size_t> PairPositions;
        PairPositions.reserve(ACandidates.size());
        for (std::size_t CandidateIndex = 0; CandidateIndex < ACandidates.size(); ++CandidateIndex) {
            const TetClusterCandidate &Candidate = ACandidates[CandidateIndex];
            if (Candidate.Valid && Candidate.OriginalIndices.size() == 2 && std::isfinite(Candidate.Score)) {
                PairPositions.push_back(CandidateIndex);
            }
        }
        std::stable_sort(PairPositions.begin(), PairPositions.end(), [&](std::size_t AFirstPosition, std::size_t ASecondPosition) { return ACandidates[AFirstPosition].Score > ACandidates[ASecondPosition].Score; });
        if (PairPositions.size() > static_cast<std::size_t>(CET_CLUSTER_FILL_MAX_SWAP_CLUSTERS)) {
            PairPositions.resize(CET_CLUSTER_FILL_MAX_SWAP_CLUSTERS);
        }
        return PairPositions;
    }
    bool TryFindBetterPairSwap(const TPairCandidateLookup &APairCandidateLookup, const TetClusterCandidate &AFirstCandidate, const TetClusterCandidate &ASecondCandidate, const TetClusterCandidate *&AOutFirstCandidate, const TetClusterCandidate *&AOutSecondCandidate)
    {
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
            if (GainRatio >= CET_CLUSTER_FILL_MIN_SWAP_GAIN_RATIO) {
                BestScore = NewScore;
                AOutFirstCandidate = FirstIt->second;
                AOutSecondCandidate = SecondIt->second;
            }
        };
        TrySelectSwap(A, C, B, D);
        TrySelectSwap(A, D, B, C);
        return AOutFirstCandidate != nullptr && AOutSecondCandidate != nullptr;
    }
} // namespace

namespace ET {
    namespace NEST2DMANAGERLIB {
        CetClusterTemplateFillOptimizer::CetClusterTemplateFillOptimizer() : CetCoreObject() {}
        CetClusterTemplateFillOptimizer::~CetClusterTemplateFillOptimizer() {}

        TetClusterBuildResult CetClusterTemplateFillOptimizer::BuildTemplateClusters(const CetTNestItemVector &AOriginalItems, const std::vector<TetShapeFeature> &AFeatures, const TetNestOptions &AOptions)
        {
            const int Count = static_cast<int>(AOriginalItems.size());
            if (Count <= 0) {
                return TetClusterBuildResult{};
            }
            if (AFeatures.size() != AOriginalItems.size()) {
                std::cout << "[TEMPLATE][ERROR] Feature count mismatch. OriginalItems=" << AOriginalItems.size() << ", Features=" << AFeatures.size() << std::endl;
                return Nest2DUtils->Nest2DCluster->BuildClusterResultFromCandidates(AOriginalItems, {}, AOptions);
            }
            std::vector<bool> Used(Count, false);
            std::map<MetShapeType, std::vector<int>> IndicesByType;
            CetTemplateCandidateGenerator CandidateGenerator;
            // 形状分类
            CandidateGenerator.CollectTemplateShapeIndices(AFeatures, IndicesByType);
            // Collect all template candidates.
            std::vector<TetClusterCandidate> BaseCandidates;
            CandidateGenerator.BuildTemplateCandidates(AOriginalItems, AFeatures, AOptions, IndicesByType, BaseCandidates);
            // Keep skeletons and their bounded fill variants together for global selection.
            std::vector<TetClusterCandidate> ExpandedCandidates;
            _BuildFilledTemplateCandidateVariants(AOriginalItems, AFeatures, AOptions, BaseCandidates, ExpandedCandidates);
            std::vector<TetClusterCandidate> AcceptedCandidates = _SelectAndOptimizeTemplateCandidates(AOriginalItems, AFeatures, AOptions, ExpandedCandidates, Used, Count);
            return Nest2DUtils->Nest2DCluster->BuildClusterResultFromCandidates(AOriginalItems, AcceptedCandidates, AOptions);
        }
        // Legacy candidate-generation implementation retained after moving logic to CetTemplateCandidateGenerator.
        void CetClusterTemplateFillOptimizer::_BuildFilledTemplateCandidateVariants(const CetTNestItemVector &AOriginalItems, const std::vector<TetShapeFeature> &AFeatures, const TetNestOptions &AOptions, const std::vector<TetClusterCandidate> &ABaseCandidates, std::vector<TetClusterCandidate> &AOutCandidates)
        {
            AOutCandidates = ABaseCandidates;
            TetClusterFillSearchStats Stats;
            CetClusterFillSearchEngine SearchEngine;
            const TetClusterFillSearchConfig Config = CetClusterFillSearchEngine::GetClusterFillSearchConfig(AOriginalItems.size());
            std::size_t ValidBaseCandidateCount = 0;
            std::size_t FilledVariantCount = 0;
            const bool HasFixedEnvelopeBase = std::any_of(ABaseCandidates.begin(), ABaseCandidates.end(), [&](const TetClusterCandidate &Candidate) { return IsFixedCircleEnvelopeBase(Candidate, AFeatures) || IsFixedEllipseEnvelopeBase(Candidate, AFeatures); });
            for (const TetClusterCandidate &BaseCandidate : ABaseCandidates) {
                if (!BaseCandidate.Valid)
                    continue;
                ++ValidBaseCandidateCount;
                Stats.BaseFillRatioSum += BaseCandidate.FillRatio;
                if (HasFixedEnvelopeBase || SkipsGenericTemplateFill(BaseCandidate))
                    continue;
                std::vector<TetClusterCandidate> Variants;
                SearchEngine.BuildFilledVariantsForBase({AOriginalItems, AFeatures, AOptions, BaseCandidate, BaseCandidate}, Config, Variants, Stats);
                if (!Variants.empty()) {
                    FilledVariantCount += Variants.size();
                    std::cout << "[TEMPLATE][FILL VARIANT] BaseType=" << BaseCandidate.ClusterType << " Generated=" << Variants.size() << " BaseFillRatio=" << BaseCandidate.FillRatio << " BestFillRatio=" << Variants.front().FillRatio << std::endl;
                    AOutCandidates.insert(AOutCandidates.end(), Variants.begin(), Variants.end());
                }
            }
            const TetClusterFillSearchConfig EnvelopeConfig = CetClusterFillSearchEngine::GetClusterEnvelopeFillSearchConfig(AOriginalItems.size());
            TetCircleGapTemplateCache CircleGapTemplateCache;
            TetEllipseGapTemplateCache EllipseGapTemplateCache;
            std::vector<const TetClusterCandidate *> EnvelopeBaseCandidates;
            EnvelopeBaseCandidates.reserve(ABaseCandidates.size());
            for (const TetClusterCandidate &BaseCandidate : ABaseCandidates) {
                if (IsFixedCircleEnvelopeBase(BaseCandidate, AFeatures) || IsFixedEllipseEnvelopeBase(BaseCandidate, AFeatures)) {
                    EnvelopeBaseCandidates.push_back(&BaseCandidate);
                }
            }
            std::stable_sort(EnvelopeBaseCandidates.begin(), EnvelopeBaseCandidates.end(), [](const TetClusterCandidate *AFirst, const TetClusterCandidate *ASecond) {
                const bool FirstIsEllipse = AFirst->BuilderName == "EllipseBuilder";
                const bool SecondIsEllipse = ASecond->BuilderName == "EllipseBuilder";
                if (FirstIsEllipse != SecondIsEllipse)
                    return FirstIsEllipse;
                const double FirstAvailableArea = std::max(0.0, AFirst->BoundingBoxArea - AFirst->ReservedArea);
                const double SecondAvailableArea = std::max(0.0, ASecond->BoundingBoxArea - ASecond->ReservedArea);
                if (std::abs(FirstAvailableArea - SecondAvailableArea) > 1.0)
                    return FirstAvailableArea > SecondAvailableArea;
                return AFirst->ClusterType < ASecond->ClusterType;
            });
            for (const TetClusterCandidate *BaseCandidate : EnvelopeBaseCandidates) {
                std::vector<TetClusterCandidate> Variants;
                std::map<std::string, TetCircleGapTemplate> &GapTemplates = CircleGapTemplateCache[BuildCircleGapTemplateCacheKey(*BaseCandidate)];
                SearchEngine.BuildEnvelopeFilledVariantsForBase({AOriginalItems, AFeatures, AOptions, *BaseCandidate, EnvelopeConfig, GapTemplates, EllipseGapTemplateCache, Variants, Stats});
                if (Variants.empty())
                    continue;
                const TetClusterCandidate &BestVariant = Variants.front();
                std::cout << "[TEMPLATE][ENVELOPE FILL] BaseType=" << BaseCandidate->ClusterType << " Generated=" << Variants.size() << " BaseEnvelopeFill=" << BaseCandidate->BoundingFillRatio << " BestFillRatio=" << BestVariant.FillRatio << " EnvelopeFillGain=" << (BestVariant.FillRatio - BaseCandidate->BoundingFillRatio) << " ProxyMode=" << ToString(BestVariant.ProxyMode) << " ProxyArea=" << BestVariant.ProxyArea << " FillerCount=" << BestVariant.OriginalIndices.size() - BaseCandidate->OriginalIndices.size() << " Width=" << BestVariant.ClusterWidth << " Height=" << BestVariant.ClusterHeight << std::endl;
                AOutCandidates.insert(AOutCandidates.end(), Variants.begin(), Variants.end());
            }
            const double BaseAverage = ValidBaseCandidateCount == 0 ? 0.0 : Stats.BaseFillRatioSum / static_cast<double>(ValidBaseCandidateCount);
            const double FilledAverage = FilledVariantCount == 0 ? 0.0 : Stats.FilledFillRatioSum / static_cast<double>(FilledVariantCount);
            std::cout << "[TEMPLATE][FILL SUMMARY] BaseCandidateCount=" << ABaseCandidates.size() << " GeneratedVariantCount=" << Stats.GeneratedVariantCount << " DeduplicatedVariantCount=" << Stats.DeduplicatedVariantCount << " FilledVariantCount=" << FilledVariantCount << " AverageBaseFillRatio=" << BaseAverage << " AverageFilledFillRatio=" << FilledAverage << " BestFillRatioGain=" << Stats.BestFillRatioGain << " FreeRegionCount=" << Stats.FreeRegionCount << " SearchAttempts=" << Stats.SearchAttempts << " EnvelopeGeneratedVariantCount=" << Stats.EnvelopeGeneratedVariantCount << " EnvelopeDeduplicatedVariantCount=" << Stats.EnvelopeDeduplicatedVariantCount << " EnvelopeFreeRegionCount=" << Stats.EnvelopeFreeRegionCount << " EnvelopeSearchAttempts=" << Stats.EnvelopeSearchAttempts << " EnvelopeTimeLimitHits=" << Stats.EnvelopeTimeLimitHits << " EnvelopeMaxDepthReached=" << Stats.EnvelopeMaxDepthReached << " EnvelopeBestFillerCount=" << Stats.EnvelopeBestFillerCount << " EnvelopeSearchMs=" << Stats.EnvelopeSearchMs << " EnvelopeTrueContourMs=" << Stats.EnvelopeTrueContourMs << " BestEnvelopeFillRatioGain=" << Stats.BestEnvelopeFillRatioGain << " BestEnvelopeRectangleFillRatio=" << Stats.BestEnvelopeRectangleFillRatio << " EnvelopeBeamWidth=" << EnvelopeConfig.BeamWidth << " EnvelopeMaxDepth=" << EnvelopeConfig.MaxDepth << " EnvelopeMaxFillers=" << EnvelopeConfig.MaxCandidateFillers << " EnvelopeMaxPlacementAttempts=" << EnvelopeConfig.MaxPlacementAttempts << " BeamWidth=" << Config.BeamWidth << " MaxDepth=" << Config.MaxDepth << " MaxFillers=" << Config.MaxCandidateFillers << " MaxPlacementAttempts=" << Config.MaxPlacementAttempts << std::endl;
        }
        std::vector<TetClusterCandidate> CetClusterTemplateFillOptimizer::_SelectTemplateCandidates(const CetTNestItemVector &AOriginalItems, const std::vector<TetShapeFeature> &AFeatures, const TetNestOptions &AOptions, const std::vector<TetClusterCandidate> &ABaseCandidates, std::vector<bool> &AUsed)
        {
            const int Count = static_cast<int>(AOriginalItems.size());
            std::vector<TetClusterCandidate> SortedCandidates = ABaseCandidates;
            auto CountFillerFamilies = [&](const TetClusterCandidate &Candidate) {
                std::set<std::uint64_t> Families;
                const std::size_t Start = std::min(Candidate.SkeletonChildCount, Candidate.OriginalIndices.size());
                for (std::size_t Index = Start; Index < Candidate.OriginalIndices.size(); ++Index) {
                    const int OriginalIndex = Candidate.OriginalIndices[Index];
                    if (OriginalIndex >= 0 && OriginalIndex < static_cast<int>(AFeatures.size())) {
                        Families.insert(CetClusterFillSearchEngine::MakeFillerFamilyKey(AFeatures[OriginalIndex]));
                    }
                }
                return Families.size();
            };
            std::stable_sort(SortedCandidates.begin(), SortedCandidates.end(), [&](const TetClusterCandidate &A, const TetClusterCandidate &AB) {
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
                        if (AFamilyCount != BFamilyCount)
                            return AFamilyCount > BFamilyCount;
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
                if (ARegularSkeleton && BRegularSkeleton && A.SkeletonChildCount != AB.SkeletonChildCount) {
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
            CetClusterInventoryRebalancer InventoryRebalancer;
            for (const TetClusterCandidate &Candidate : SortedCandidates) {
                if (IsDeferredTriangleCandidate(Candidate)) {
                    DeferredTriangles.push_back(Candidate);
                    continue;
                }
                TetClusterCandidate BoundCandidate;
                if (!InventoryRebalancer.TryBindCandidateInventory(Candidate, AFeatures, AUsed, BoundCandidate) || !_CanAcceptClusterCandidate(AOriginalItems, AOptions, BoundCandidate, AUsed, Count)) {
                    std::cout << "[TEMPLATE][REJECT] Builder=" << Candidate.BuilderName << " Type=" << Candidate.ClusterType << " Score=" << Candidate.Score << std::endl;
                    continue;
                }
                AcceptedCandidates.push_back(std::move(BoundCandidate));
                for (int OriginalIndex : AcceptedCandidates.back().OriginalIndices) {
                    AUsed[OriginalIndex] = true;
                }
                std::cout << "[TEMPLATE][BASE ACCEPT] Builder=" << Candidate.BuilderName << " Type=" << Candidate.ClusterType << " ChildCount=" << Candidate.OriginalIndices.size() << " Score=" << Candidate.Score << std::endl;
            }
            InventoryRebalancer.RebalanceAcceptedClusterInventory(AOriginalItems, AFeatures, AOptions, AcceptedCandidates, AUsed);
            const bool DeferRemainingTriangles = HasFilledEllipseCandidate(AcceptedCandidates);
            for (const TetClusterCandidate &Candidate : DeferredTriangles) {
                if (DeferRemainingTriangles) {
                    std::cout << "[TEMPLATE][DEFERRED TRIANGLE SINGLE] Type=" << Candidate.ClusterType << std::endl;
                    continue;
                }
                TetClusterCandidate BoundCandidate;
                if (!InventoryRebalancer.TryBindCandidateInventory(Candidate, AFeatures, AUsed, BoundCandidate) || !_CanAcceptClusterCandidate(AOriginalItems, AOptions, BoundCandidate, AUsed, Count)) {
                    std::cout << "[TEMPLATE][DEFERRED TRIANGLE REJECT] Type=" << Candidate.ClusterType << std::endl;
                    continue;
                }
                AcceptedCandidates.push_back(std::move(BoundCandidate));
                for (int OriginalIndex : AcceptedCandidates.back().OriginalIndices) {
                    AUsed[OriginalIndex] = true;
                }
                std::cout << "[TEMPLATE][DEFERRED TRIANGLE ACCEPT] Type=" << Candidate.ClusterType << " ChildCount=" << Candidate.OriginalIndices.size() << std::endl;
            }
            return AcceptedCandidates;
        }
        std::vector<TetClusterCandidate> CetClusterTemplateFillOptimizer::_SelectAndOptimizeTemplateCandidates(const CetTNestItemVector &AOriginalItems, const std::vector<TetShapeFeature> &AFeatures, const TetNestOptions &AOptions, const std::vector<TetClusterCandidate> &ABaseCandidates, std::vector<bool> &AUsed, int AOriginalItemCount)
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
            for (const TetClusterCandidate &Candidate : AcceptedCandidates) {
                for (int OriginalIndex : Candidate.OriginalIndices) {
                    if (OriginalIndex >= 0 && OriginalIndex < AOriginalItemCount) {
                        AUsed[OriginalIndex] = true;
                    }
                }
            }
#ifdef _DEBUG
            const auto ToMilliseconds = [](const auto &AStart, const auto &AEnd) { return std::chrono::duration<double, std::milli>(AEnd - AStart).count(); };
            std::cout << "[ClusterSelection] Greedy Score = " << GreedyScore << ", Optimized Score = " << OptimizedScore << ", Improvement = " << OptimizedScore - GreedyScore << ", Swap Count = " << SwapCount << std::endl;
            std::cout << "[ClusterPerf] Candidates: " << ABaseCandidates.size() << ", AcceptedGreedy: " << GreedyResult.size() << ", AcceptedOptimized: " << OptimizedCandidateCount << ", SwapCount: " << SwapCount << ", GreedyMs: " << ToMilliseconds(GreedyStartTime, GreedyEndTime) << ", OptimizeMs: " << ToMilliseconds(OptimizeStartTime, OptimizeEndTime) << std::endl;
#endif
            return AcceptedCandidates;
        }
        int CetClusterTemplateFillOptimizer::_OptimizePairClusterSelection(const std::vector<TetClusterCandidate> &AAllCandidates, std::vector<TetClusterCandidate> &AAcceptedCandidates, int AOriginalItemCount)
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
            for (int Round = 0; Round < CET_CLUSTER_FILL_MAX_SWAP_ROUNDS; ++Round) {
                const std::vector<std::size_t> PairPositions = CollectPairCandidatePositions(AAcceptedCandidates);
                if (PairPositions.size() < 2) {
                    break;
                }
                bool Changed = false;
                for (std::size_t FirstPairIndex = 0; FirstPairIndex + 1 < PairPositions.size(); ++FirstPairIndex) {
                    for (std::size_t SecondPairIndex = FirstPairIndex + 1; SecondPairIndex < PairPositions.size(); ++SecondPairIndex) {
                        const std::size_t FirstPosition = PairPositions[FirstPairIndex];
                        const std::size_t SecondPosition = PairPositions[SecondPairIndex];
                        const TetClusterCandidate *FirstReplacement = nullptr;
                        const TetClusterCandidate *SecondReplacement = nullptr;
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
            if (SwapCount > 0 && GainRatio < CET_CLUSTER_FILL_MIN_SWAP_GAIN_RATIO) {
                AAcceptedCandidates = GreedyResult;
                return 0;
            }
            return SwapCount;
        }
        double CetClusterTemplateFillOptimizer::_CalculateCandidateSelectionScore(const std::vector<TetClusterCandidate> &ACandidates)
        {
            double TotalScore = 0.0;
            for (const TetClusterCandidate &Candidate : ACandidates) {
                TotalScore += Candidate.Score;
            }
            return TotalScore;
        }
        bool CetClusterTemplateFillOptimizer::_ValidateClusterSelection(const std::vector<TetClusterCandidate> &ACandidates, int AOriginalItemCount)
        {
            if (AOriginalItemCount < 0) {
                return false;
            }
            std::vector<bool> Used(static_cast<std::size_t>(AOriginalItemCount), false);
            for (const TetClusterCandidate &Candidate : ACandidates) {
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
                for (const TetItemTransform &Transform : Candidate.Transforms) {
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
        bool CetClusterTemplateFillOptimizer::_CanAcceptClusterCandidate(const CetTNestItemVector &AOriginalItems, const TetNestOptions &AOptions, const TetClusterCandidate &ACandidate, const std::vector<bool> &AUsed, int AOriginalCount)
        {
            if (AOriginalCount < 0 || AOriginalItems.size() != static_cast<std::size_t>(AOriginalCount)) return false;
            if (!ACandidate.Valid || ACandidate.OriginalIndices.empty() || ACandidate.OriginalIndices.size() != ACandidate.Transforms.size() || ACandidate.ProxyContour.size() < 3) return false;
            if (ACandidate.ClusterWidth <= 0.0 || ACandidate.ClusterHeight <= 0.0 || ACandidate.ProxyArea <= 0.0) return false;
            std::set<int> CandidateIds;
            std::set<int> TransformIds;
            for (int OriginalIndex : ACandidate.OriginalIndices) {
                if (OriginalIndex < 0 || OriginalIndex >= AOriginalCount || OriginalIndex >= static_cast<int>(AUsed.size()) || AUsed[OriginalIndex] || !CandidateIds.insert(OriginalIndex).second) return false;
            }
            for (const TetItemTransform &Transform : ACandidate.Transforms) {
                if (Transform.OriginalId < 0 || Transform.OriginalId >= AOriginalCount || !std::isfinite(Transform.RelativeX) || !std::isfinite(Transform.RelativeY) || !std::isfinite(Transform.RelativeRotation) || !TransformIds.insert(Transform.OriginalId).second) return false;
            }
            return CandidateIds == TransformIds;
        }

    }
}
