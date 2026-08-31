#include "pch.h"
#include "Nest2D_ClusterManager.h"
#include "Nest2D_AutoPairClusterBuilder.h"
#include "Nest2D_ClusterTemplateFillOptimizer.h"
#include "Nest2D_ClusterGeometryHelper.h"
#include "Nest2D_SelfFunction.h"
#include "Nest2D_TriangleClusterBuilder.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <set>
namespace ET {
    namespace NEST2DMANAGERLIB {
        CetClusterManager::CetClusterManager() : CetCoreObject() {}
        CetClusterManager::~CetClusterManager() {}
        TetClusterBuildResult CetClusterManager::BuildClusterItems(const CetTNestItemVector &AOriginalItems, const TetNestOptions &AOptions, MetClusterStrategy AStrategy)
        {
            TetClusterBuildResult Result;
            Result.NestItems.reserve(AOriginalItems.size());
            Result.MetaItems.reserve(AOriginalItems.size());
            if (AOriginalItems.empty()) {
                return Result;
            }
            // zanshibuzuhe
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
                return Nest2DUtils->Nest2dAutoPairClusterBuilder->BuildAutoPairClusters(AOriginalItems, AOptions);
            }
            for (int i = 0; i < static_cast<int>(AOriginalItems.size()); ++i) {
                _AddSingleItem(AOriginalItems, i, Result);
            }
            return Result;
        }
        TetClusterBuildResult CetClusterManager::BuildClusterItemsWithFeatures(const CetTNestItemVector &AOriginalItems, const std::vector<TetShapeFeature> &AFeatures, const TetNestOptions &AOptions, MetClusterStrategy AStrategy)
        {
            if (AStrategy == MetClusterStrategy::TemplateCluster) {
                return Nest2DUtils->Nest2dClusterTemplateFillOptimizer->BuildTemplateClusters(AOriginalItems, AFeatures, AOptions);
            }
            return BuildClusterItems(AOriginalItems, AOptions, AStrategy);
        }
        TetClusterBuildResult CetClusterManager::BuildClusterResultFromCandidates(const CetTNestItemVector &AOriginalItems, const std::vector<TetClusterCandidate> &ACandidates, const TetNestOptions &AOptions)
        {
            TetClusterBuildResult Result;
            Result.NestItems.reserve(AOriginalItems.size());
            Result.MetaItems.reserve(AOriginalItems.size());
            std::vector<bool> Used(AOriginalItems.size(), false);
            for (const TetClusterCandidate &Candidate : ACandidates) {
                if (!Candidate.Valid || Candidate.OriginalIndices.empty() || Candidate.OriginalIndices.size() != Candidate.Transforms.size()) {
                    continue;
                }
                bool Conflict = false;
                std::set<int> CandidateIds;
                for (int OriginalIndex : Candidate.OriginalIndices) {
                    if (OriginalIndex < 0 || OriginalIndex >= static_cast<int>(Used.size()) || Used[OriginalIndex] || !CandidateIds.insert(OriginalIndex).second) {
                        Conflict = true;
                        break;
                    }
                }
                if (Conflict || !_AddClusterCandidate(Candidate, Result)) {
                    continue;
                }
                for (int OriginalIndex : Candidate.OriginalIndices) {
                    Used[OriginalIndex] = true;
                }
            }
            for (int OriginalIndex = 0; OriginalIndex < static_cast<int>(AOriginalItems.size()); ++OriginalIndex) {
                if (!Used[OriginalIndex]) {
                    _AddSingleItem(AOriginalItems, OriginalIndex, Result);
                }
            }
            (void)AOptions;
            return _ValidateBuildResultCoverage(Result, static_cast<int>(AOriginalItems.size())) ? Result : _BuildAllSingles(AOriginalItems);
        }
        std::vector<int> CetClusterManager::RankBoardCompositeSkeletons(const CetTNestItemVector &AItems, const std::vector<TetShapeFeature> &AFeatures, int ATargetBin, const TetClusterFreeRegion &AFreeRegion, std::size_t AMaxCount) const
        {
            std::vector<std::pair<double, int>> Ranked;
            if (AItems.size() != AFeatures.size() || AMaxCount == 0 || AFreeRegion.Area <= 0.0)
                return {};
            const double RegionAspect = AFreeRegion.Height > 0.0 ? AFreeRegion.Width / AFreeRegion.Height : 0.0;
            for (std::size_t Index = 0; Index < AItems.size(); ++Index) {
                const TetShapeFeature &Feature = AFeatures[Index];
                if (AItems[Index].binId() >= 0 && AItems[Index].binId() <= ATargetBin)
                    continue;
                if (Feature.Width <= 0.0 || Feature.Height <= 0.0 || Feature.BoxArea > AFreeRegion.Area)
                    continue;
                const bool IsSkeleton = Feature.ShapeType == MetShapeType::CircleLike || Feature.ShapeType == MetShapeType::EllipseLike || Feature.ShapeType == MetShapeType::TriangleLike || Feature.ShapeType == MetShapeType::RectangleLike || Feature.ShapeType == MetShapeType::ArcLike || Feature.ShapeType == MetShapeType::ConvexPolygon || Feature.ShapeType == MetShapeType::ConcavePolygon || Feature.ShapeType == MetShapeType::QuadrilateralLike;
                if (!IsSkeleton)
                    continue;
                const double Aspect = Feature.Height > 0.0 ? Feature.Width / Feature.Height : 0.0;
                const double AspectMatch = RegionAspect > 0.0 && Aspect > 0.0 ? std::min(Aspect, RegionAspect) / std::max(Aspect, RegionAspect) : 0.0;
                const double EnvelopeOpportunity = std::max(0.0, Feature.BoxArea - Feature.Area);
                const double Score = Feature.Area + EnvelopeOpportunity * CET_BOARD_COMPOSITE_ENVELOPE_OPPORTUNITY_WEIGHT + AspectMatch * Feature.BoxArea;
                Ranked.emplace_back(Score, static_cast<int>(Index));
            }
            std::stable_sort(Ranked.begin(), Ranked.end(), [](const auto &ALeft, const auto &ARight) { return std::abs(ALeft.first - ARight.first) > CET_BOARD_COMPOSITE_SCORE_COMPARISON_TOLERANCE ? ALeft.first > ARight.first : ALeft.second < ARight.second; });
            std::vector<int> Result;
            for (const auto &Entry : Ranked) {
                Result.push_back(Entry.second);
                if (Result.size() >= AMaxCount)
                    break;
            }
            return Result;
        }
        std::vector<int> CetClusterManager::RankExistingBoardCompositeSkeletons(const CetTNestItemVector &AItems, const std::vector<TetShapeFeature> &AFeatures, int ATargetBin, const TetClusterFreeRegion &AFreeRegion, std::size_t AMaxCount) const
        {
            std::vector<std::pair<double, int>> Ranked;
            if (AItems.size() != AFeatures.size() || AMaxCount == 0 || AFreeRegion.Area <= 0.0)
                return {};
            for (std::size_t Index = 0; Index < AItems.size(); ++Index) {
                const TetShapeFeature &Feature = AFeatures[Index];
                if (AItems[Index].binId() != ATargetBin || Feature.Width <= 0.0 || Feature.Height <= 0.0)
                    continue;
                const bool IsSkeleton = Feature.ShapeType == MetShapeType::CircleLike || Feature.ShapeType == MetShapeType::EllipseLike || Feature.ShapeType == MetShapeType::TriangleLike || Feature.ShapeType == MetShapeType::RectangleLike || Feature.ShapeType == MetShapeType::ArcLike || Feature.ShapeType == MetShapeType::ConvexPolygon || Feature.ShapeType == MetShapeType::ConcavePolygon || Feature.ShapeType == MetShapeType::QuadrilateralLike;
                if (!IsSkeleton)
                    continue;
                const auto Bounds = AItems[Index].boundingBox();
                const double MinX = static_cast<double>(getX(Bounds.minCorner()));
                const double MinY = static_cast<double>(getY(Bounds.minCorner()));
                const double MaxX = static_cast<double>(getX(Bounds.maxCorner()));
                const double MaxY = static_cast<double>(getY(Bounds.maxCorner()));
                const double GapX = std::max({0.0, AFreeRegion.MinX - MaxX, MinX - AFreeRegion.MaxX});
                const double GapY = std::max({0.0, AFreeRegion.MinY - MaxY, MinY - AFreeRegion.MaxY});
                const double Distance = std::hypot(GapX, GapY);
                const double Opportunity = std::max(0.0, Feature.BoxArea - Feature.Area);
                Ranked.emplace_back(Opportunity - Distance, static_cast<int>(Index));
            }
            std::stable_sort(Ranked.begin(), Ranked.end(), [](const auto &ALeft, const auto &ARight) { return std::abs(ALeft.first - ARight.first) > CET_BOARD_COMPOSITE_SCORE_COMPARISON_TOLERANCE ? ALeft.first > ARight.first : ALeft.second < ARight.second; });
            std::vector<int> Result;
            for (const auto &Entry : Ranked) {
                Result.push_back(Entry.second);
                if (Result.size() >= AMaxCount)
                    break;
            }
            return Result;
        }
        void CetClusterManager::ExpandClusterResultToOriginalItems(const CetTNestItemVector &AOriginalItems, const CetTNestItemVector &APackedItems, const std::vector<TetMetaItem> &AMetaItems, CetTNestItemVector &AOutOriginalItems, bool ALog)
        {
            AOutOriginalItems = AOriginalItems;
            if (APackedItems.size() != AMetaItems.size()) {
                std::cout << "[CLUSTER][ERROR] PackedItems size != MetaItems size. PackedItems = " << APackedItems.size() << ", MetaItems = " << AMetaItems.size() << std::endl;
                return;
            }
            for (std::size_t PackedIndex = 0; PackedIndex < APackedItems.size(); ++PackedIndex) {
                const auto &PackedItem = APackedItems[PackedIndex];
                const auto &Meta = AMetaItems[PackedIndex];
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
        bool CetClusterManager::ValidatePackedResultSpacing(const CetTNestItemVector &AOriginalItems, const CetTNestItemVector &APackedItems, const std::vector<TetMetaItem> &AMetaItems, const TetNestOptions &AOptions, TetExpandedSpacingFailure *AOutFailure)
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
                for (const TetItemTransform &Transform : AMetaItems[PackedIndex].TransformData) {
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
                        std::cout << "[CLUSTER][EXPANDED VALIDATION][REJECT] " << (RawContoursIntersect ? "Raw contour overlap" : (SpacingCoord > 0 ? "Spacing violation" : "Overlap")) << " between original items " << FirstIndex << " and " << SecondIndex << " on bin " << FirstItem.binId() << ", packed items " << OriginalToPackedIndex[FirstIndex] << " and " << OriginalToPackedIndex[SecondIndex] << ", required spacing " << AOptions.Spacing << std::endl;
                        return false;
                    }
                }
            }
            return true;
        }
        void CetClusterManager::_AddSingleItem(const CetTNestItemVector &AOriginalItems, int AOriginalIndex, TetClusterBuildResult &AResult)
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
        double CetClusterManager::_GetItemWidth(const CetNestItem &AItem) { return static_cast<double>(AItem.boundingBox().width()); }
        double CetClusterManager::_GetItemHeight(const CetNestItem &AItem) { return static_cast<double>(AItem.boundingBox().height()); }
        void CetClusterManager::_ExpandClusterChildren(const CetNestItem &APackedItem, const TetMetaItem &AMeta, CetTNestItemVector &AOutOriginalItems, bool ALog)
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
            for (const auto &Transform : AMeta.TransformData) {
                int originalId = Transform.OriginalId;
                if (originalId < 0 || originalId >= static_cast<int>(AOutOriginalItems.size())) {
                    std::cout << "[ClusTer][WARN] Invalid originalId in TransformData: " << originalId << std::endl;
                    continue;
                }
                auto &OriginalItem = AOutOriginalItems[originalId];
                double LocalX = Transform.RelativeX;
                double LocalY = Transform.RelativeY;
                double RotatedLocalX = LocalX * CosR - LocalY * SinR;
                double RotatedLocalY = LocalX * SinR + LocalY * CosR;
                double FinalX = PackedX + RotatedLocalX;
                double FinalY = PackedY + RotatedLocalY;
                double FinalRotation = PackedRotation + Transform.RelativeRotation;
                OriginalItem.binId(APackedItem.binId());
                OriginalItem.translation(ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(std::llround(FinalX)), static_cast<ClipperLib::cInt>(std::llround(FinalY))));
                OriginalItem.rotation(FinalRotation);
                if (ALog) {
                    std::cout << "[CLUSTER][EXPAND ITEM] OriginalId = " << originalId << ", Local = (" << LocalX << ", " << LocalY << ")" << ", Final = (" << FinalX << ", " << FinalY << ")" << ", FinalRotation = " << FinalRotation << ", Bin = " << APackedItem.binId() << std::endl;
                }
            }
        }
        CetNestItem CetClusterManager::_MakeClusterProxyItem(const TetClusterCandidate &ACandidate)
        {
            CetClusterGeometryHelper Geometry;
            if (ACandidate.ProxyContour.size() >= 3) {
                return Geometry.MakeNestItemFromProxyContour(ACandidate.ProxyContour);
            }
            return Geometry.MakeNestItemFromProxyContour(Geometry.MakeRectangleContour(ACandidate.ClusterWidth, ACandidate.ClusterHeight));
        }
        bool CetClusterManager::_AddClusterCandidate(const TetClusterCandidate &ACandidate, TetClusterBuildResult &AResult)
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
            return true;
        }
        TetClusterBuildResult CetClusterManager::_BuildAllSingles(const CetTNestItemVector &AOriginalItems)
        {
            TetClusterBuildResult Result;
            Result.NestItems.reserve(AOriginalItems.size());
            Result.MetaItems.reserve(AOriginalItems.size());
            for (int i = 0; i < static_cast<int>(AOriginalItems.size()); ++i) {
                _AddSingleItem(AOriginalItems, i, Result);
            }
            return Result;
        }
        bool CetClusterManager::_ValidateBuildResultCoverage(const TetClusterBuildResult &AResult, int AOriginalCount)
        {
            if (AOriginalCount < 0 || AResult.NestItems.size() != AResult.MetaItems.size()) {
                return false;
            }
            std::vector<int> HitCount(static_cast<std::size_t>(AOriginalCount), 0);
            for (const TetMetaItem &Meta : AResult.MetaItems) {
                if (Meta.PackedItemIndex < 0 || Meta.PackedItemIndex >= static_cast<int>(AResult.NestItems.size()) || Meta.TransformData.empty()) {
                    return false;
                }
                for (const TetItemTransform &Transform : Meta.TransformData) {
                    if (Transform.OriginalId < 0 || Transform.OriginalId >= AOriginalCount) {
                        return false;
                    }
                    ++HitCount[Transform.OriginalId];
                }
            }
            return std::all_of(HitCount.begin(), HitCount.end(), [](int Count) { return Count == 1; });
        }
    } // namespace NEST2DMANAGERLIB
} // namespace ET
