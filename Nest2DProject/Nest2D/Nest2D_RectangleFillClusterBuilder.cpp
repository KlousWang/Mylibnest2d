#include "pch.h"
#include "Nest2D_RectangleFillClusterBuilder.h"
#include "Nest2D_ClusterGeometryHelper.h"
#include "Nest2D_RotationUtils.h"
#include "NestUtils.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <map>
#include <unordered_set>
#include <utility>

namespace ET {
    namespace NEST2DMANAGERLIB {

        CetRectangleFillClusterBuilder::CetRectangleFillClusterBuilder() : CetCoreObject() {}
        CetRectangleFillClusterBuilder::~CetRectangleFillClusterBuilder() {}

        bool CetRectangleFillClusterBuilder::BuildCandidateForBase(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ABaseCandidate, const TetNestOptions& AOptions, const std::vector<bool>& AUsed, TetClusterCandidate& AOutCandidate)
        {
            AOutCandidate = TetClusterCandidate{};
            if (AOriginalItems.empty() || AFeatures.size() != AOriginalItems.size() || AUsed.size() != AOriginalItems.size()) {
                return false;
            }
            if (!ABaseCandidate.Valid || ABaseCandidate.OriginalIndices.size() < 2 || ABaseCandidate.OriginalIndices.size() != ABaseCandidate.Transforms.size()) {
                return false;
            }

            CetClusterGeometryHelper Geometry;
            TetClusterCandidate CurrentCandidate = ABaseCandidate;
            const double EnvelopeWidth = ABaseCandidate.ClusterWidth;
            const double EnvelopeHeight = ABaseCandidate.ClusterHeight;
            const double EnvelopeArea = EnvelopeWidth * EnvelopeHeight;
            const double BaseAreaTolerance = std::max(1.0, EnvelopeArea * 1e-9);
            const double BaseAvailableArea = std::max(0.0, EnvelopeArea - ABaseCandidate.ReservedArea);

            if (!std::isfinite(EnvelopeArea) || EnvelopeArea <= 0.0 || BaseAvailableArea <= BaseAreaTolerance) {
                return false;
            }
            if (EnvelopeWidth <= 0.0 || EnvelopeHeight <= 0.0 || !Geometry.FinalizeCandidateInRectangle(AOriginalItems, AOptions, CurrentCandidate, EnvelopeWidth, EnvelopeHeight)) {
                return false;
            }

            // 将繁杂的收集和排序提取出去
            TetRectangleFillContext FillCtx{ AOriginalItems, AFeatures, AOptions, AUsed };
            std::vector<int> FillerIndices = _PrepareFillerIndices(FillCtx, CurrentCandidate, BaseAvailableArea);

            const std::size_t InitialChildCount = CurrentCandidate.OriginalIndices.size();
            std::unordered_set<std::uint64_t> FailedFillerShapes;

			const bool IsCircleOnlyBase = std::all_of(CurrentCandidate.OriginalIndices.begin(), CurrentCandidate.OriginalIndices.end(), [&](int AOriginalIndex) {
				return AOriginalIndex >= 0 && AOriginalIndex < static_cast<int>(AFeatures.size()) && AFeatures[AOriginalIndex].ShapeType == MetShapeType::CircleLike;
				});
			std::size_t MaxAcceptedFillerCount = IsCircleOnlyBase ? AOriginalItems.size()
				: (InitialChildCount >= 16 ? CET_RECTANGLE_FILL_LARGE_BASE_MAX_ACCEPTED_ITEMS
					: (InitialChildCount >= 8 ? CET_RECTANGLE_FILL_MEDIUM_BASE_MAX_ACCEPTED_ITEMS : CET_RECTANGLE_FILL_MAX_ACCEPTED_ITEMS_PER_BASE));

            for (int FillerIndex : FillerIndices) {
                if (CurrentCandidate.OriginalIndices.size() - InitialChildCount >= MaxAcceptedFillerCount) {
                    break;
                }
                if (AUsed[FillerIndex] || _ContainsOriginalIndex(CurrentCandidate, FillerIndex)) {
                    continue;
                }
                const double FillerArea = _GetFeatureArea(AOriginalItems[FillerIndex], AFeatures[FillerIndex]);
                const double AvailableArea = std::max(0.0, EnvelopeArea - CurrentCandidate.ReservedArea);
                const double AreaTolerance = std::max(1.0, EnvelopeArea * 1e-9);
                if (FillerArea > AvailableArea + AreaTolerance) {
                    continue;
                }

                // 调用提取出的签名计算函数
                const std::uint64_t FillerShapeSignature = _BuildFillerShapeSignature(AFeatures[FillerIndex], FillerIndex);
                if (FailedFillerShapes.find(FillerShapeSignature) != FailedFillerShapes.end()) {
                    continue;
                }

                TetClusterCandidate Candidate;
                if (!_TryAddFiller(AOriginalItems, AFeatures, CurrentCandidate, FillerIndex, AOptions, EnvelopeWidth, EnvelopeHeight, Candidate)) {
                    FailedFillerShapes.insert(FillerShapeSignature);
                    continue;
                }

                CurrentCandidate = std::move(Candidate);
                FailedFillerShapes.clear();
                std::cout << "[GAP_FILL][ACCEPT] OriginalId=" << FillerIndex << ", ChildCount=" << CurrentCandidate.OriginalIndices.size() << ", Width=" << CurrentCandidate.ClusterWidth << ", Height=" << CurrentCandidate.ClusterHeight << std::endl;
            }

            if (CurrentCandidate.OriginalIndices.size() == InitialChildCount) {
                return false;
            }

            CurrentCandidate.BuilderName = "GapFillBuilder";
            CurrentCandidate.ClusterType = ABaseCandidate.ClusterType + "_GapFill";
            AOutCandidate = std::move(CurrentCandidate);
            return true;
        }
        bool CetRectangleFillClusterBuilder::_TryAddFiller(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ACurrentCandidate, int AFillerIndex, const TetNestOptions& AOptions, double AEnvelopeWidth, double AEnvelopeHeight, TetClusterCandidate& AOutCandidate)
        {
            AOutCandidate = TetClusterCandidate{};
            TetItemTransform FillerTransform;
            CetClusterGeometryHelper Geometry;
            std::vector<TetClusterFreeRegion> FreeRegions;
            if (!Geometry.ExtractCandidateFreeRegions(AOriginalItems, AOptions, ACurrentCandidate, FreeRegions)
                || !_TryFindFillerTransform(AOriginalItems, AFeatures, ACurrentCandidate, FreeRegions, AFillerIndex, AOptions, AEnvelopeWidth, AEnvelopeHeight, FillerTransform)) {
                return false;
            }

            TetClusterCandidate BestCandidate = ACurrentCandidate;
            BestCandidate.OriginalIndices.push_back(AFillerIndex);
            BestCandidate.Transforms.push_back(FillerTransform);
            if (!Geometry.FinalizeCandidateInRectangle(AOriginalItems, AOptions, BestCandidate, AEnvelopeWidth, AEnvelopeHeight)) {
                return false;
            }

            BestCandidate.BuilderName = "GapFillBuilder";
            AOutCandidate = std::move(BestCandidate);
            return true;
        }

        bool CetRectangleFillClusterBuilder::TryAppendFiller(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ABaseCandidate, const TetClusterCandidate& ACurrentCandidate, int AFillerIndex, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate)
        {
            CetClusterGeometryHelper Geometry;
            std::vector<TetClusterFreeRegion> FreeRegions;
            return Geometry.ExtractCandidateFreeRegions(AOriginalItems, AOptions, ACurrentCandidate, FreeRegions)
                && TryAppendFillerInFreeRegions(AOriginalItems, AFeatures, ABaseCandidate, ACurrentCandidate, FreeRegions, AFillerIndex, AOptions, AOutCandidate);
        }

        bool CetRectangleFillClusterBuilder::TryAppendFillerInFreeRegions(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ABaseCandidate, const TetClusterCandidate& ACurrentCandidate, const std::vector<TetClusterFreeRegion>& AFreeRegions, int AFillerIndex, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate)
        {
            AOutCandidate = TetClusterCandidate{};
            if (!ABaseCandidate.Valid || !ACurrentCandidate.Valid || ABaseCandidate.ProxyContour.size() < 3 || ABaseCandidate.ProxyArea <= 0.0 || AFreeRegions.empty()) {
                return false;
            }

            TetItemTransform FillerTransform;
            if (!_TryFindFillerTransform(AOriginalItems, AFeatures, ACurrentCandidate, AFreeRegions, AFillerIndex, AOptions, ABaseCandidate.ClusterWidth, ABaseCandidate.ClusterHeight, FillerTransform)) {
                return false;
            }

            CetClusterGeometryHelper Geometry;
            const CetPath FillerContour = Geometry.TransformContour(Geometry.GetIdentityContour(AOriginalItems[AFillerIndex]), FillerTransform.RelativeRotation, FillerTransform.RelativeX, FillerTransform.RelativeY);
            const double AreaTolerance = std::max(1.0, ABaseCandidate.ProxyArea * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE);
            if (!Geometry.IsContourFullyContained(FillerContour, ACurrentCandidate.ProxyContour, AreaTolerance)) {
                return false;
            }

            TetClusterCandidate Candidate = ACurrentCandidate;
            Candidate.OriginalIndices.push_back(AFillerIndex);
            Candidate.Transforms.push_back(FillerTransform);
            if (!Geometry.FinalizeCandidate(AOriginalItems, AOptions, Candidate)) {
                return false;
            }
            if (Candidate.ProxyArea > ABaseCandidate.ProxyArea * (1.0 + CET_CLUSTER_FILL_MAX_PROXY_GROWTH_RATIO)) {
                return false;
            }

            Candidate.BuilderName = "TemplateFillSearch";
            Candidate.ClusterType = ABaseCandidate.ClusterType + "_Fill";
            AOutCandidate = std::move(Candidate);
            return true;
        }

        bool CetRectangleFillClusterBuilder::TryAppendFillerInRectangleEnvelope(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ABaseCandidate, const TetClusterCandidate& AEnvelopeCandidate, const TetClusterCandidate& ACurrentCandidate, const std::vector<TetClusterFreeRegion>& AFreeRegions, int AFillerIndex, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate)
        {
            AOutCandidate = TetClusterCandidate{};
            if (!ABaseCandidate.Valid || !AEnvelopeCandidate.Valid || !ACurrentCandidate.Valid
                || AEnvelopeCandidate.ProxyContour.size() < 4 || AEnvelopeCandidate.ClusterWidth <= 0.0
                || AEnvelopeCandidate.ClusterHeight <= 0.0 || AFreeRegions.empty()) {
                return false;
            }
            const double RequiredGap = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));

            TetItemTransform FillerTransform;
            if (!_TryFindFillerTransform(AOriginalItems, AFeatures, ACurrentCandidate, AFreeRegions, AFillerIndex, AOptions,
                AEnvelopeCandidate.ClusterWidth, AEnvelopeCandidate.ClusterHeight, FillerTransform)) {
                return false;
            }

            CetClusterGeometryHelper Geometry;
            const CetPath FillerContour = Geometry.TransformContour(Geometry.GetIdentityContour(AOriginalItems[AFillerIndex]),
                FillerTransform.RelativeRotation, FillerTransform.RelativeX, FillerTransform.RelativeY);
            const double AreaTolerance = std::max(1.0, AEnvelopeCandidate.ProxyArea * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE);
            if (!Geometry.IsContourFullyContained(FillerContour, AEnvelopeCandidate.ProxyContour, AreaTolerance)) {
                return false;
            }

            TetClusterCandidate Candidate = ACurrentCandidate;
            Candidate.OriginalIndices.push_back(AFillerIndex);
            Candidate.Transforms.push_back(FillerTransform);

            // The probe already passed exact free-region containment and pairwise
            // spacing. Keep the fixed rectangle state lightweight during beam
            // search; the selected completion receives full union/offset and true
            // contour validation once in RebuildEnvelopeFillWithTrueContour.
            const double FillerArea = std::abs(static_cast<double>(AOriginalItems[AFillerIndex].area()));
            if (!std::isfinite(FillerArea) || FillerArea <= 0.0) return false;
            Candidate.RealArea += FillerArea;
            Candidate.BaselineArea += std::max(FillerArea, AFeatures[AFillerIndex].BoxArea);
            Candidate.OccupiedArea += FillerArea;
            Candidate.ReservedArea = std::min(Candidate.ProxyArea, Candidate.ReservedArea + FillerArea);
            Candidate.ProxyWasteArea = std::max(0.0, Candidate.ProxyArea - Candidate.ReservedArea);
            Candidate.ProxyWasteRatio = Candidate.ProxyArea > 0.0 ? Candidate.ProxyWasteArea / Candidate.ProxyArea : 1.0;
            Candidate.FillRatio = Candidate.ProxyArea > 0.0 ? std::clamp(Candidate.RealArea / Candidate.ProxyArea, 0.0, 1.0) : 0.0;
            double FillerMinX = 0.0;
            double FillerMinY = 0.0;
            double FillerMaxX = 0.0;
            double FillerMaxY = 0.0;
            if (!Geometry.GetBounds(FillerContour, FillerMinX, FillerMinY, FillerMaxX, FillerMaxY)) return false;
            Candidate.Score += _CalculatePlacementScore(AOriginalItems, ACurrentCandidate,
                FillerMinX, FillerMinY, FillerMaxX, FillerMaxY, RequiredGap);
            Candidate.BoundingFillRatio = Candidate.FillRatio;
            Candidate.AreaSavingRatio = Candidate.BaselineArea > 0.0 ? 1.0 - Candidate.ProxyArea / Candidate.BaselineArea : 0.0;

            Candidate.BuilderName = "EnvelopeFillSearch";
            Candidate.ClusterType = ABaseCandidate.ClusterType + "_EnvelopeFill";
            AOutCandidate = std::move(Candidate);
            return true;
        }

        bool CetRectangleFillClusterBuilder::TryAppendFillerTemplateInRectangleEnvelope(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ABaseCandidate, const TetClusterCandidate& AEnvelopeCandidate, const TetClusterCandidate& ACurrentCandidate, const TetItemTransform& ATemplateTransform, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate)
        {
            AOutCandidate = TetClusterCandidate{};
            if (!ABaseCandidate.Valid || !AEnvelopeCandidate.Valid || !ACurrentCandidate.Valid
                || ATemplateTransform.OriginalId < 0 || ATemplateTransform.OriginalId >= static_cast<int>(AOriginalItems.size())
                || _ContainsOriginalIndex(ACurrentCandidate, ATemplateTransform.OriginalId)) return false;
            CetClusterGeometryHelper Geometry;
            std::vector<TetClusterFreeRegion> FreeRegions;
            if (!Geometry.ExtractCandidateFreeRegions(AOriginalItems, AOptions, ACurrentCandidate, FreeRegions)) return false;
            const CetPath Contour = Geometry.TransformContour(Geometry.GetIdentityContour(AOriginalItems[ATemplateTransform.OriginalId]),
                ATemplateTransform.RelativeRotation, ATemplateTransform.RelativeX, ATemplateTransform.RelativeY);
            const double Tolerance = std::max(1.0, AEnvelopeCandidate.ProxyArea * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE);
            if (!Geometry.IsContourFullyContained(Contour, AEnvelopeCandidate.ProxyContour, Tolerance)
                || !_IsContourInsideFreeRegions(Contour, FreeRegions)
                || !Geometry.CanAppendTransformWithSpacing(AOriginalItems, AOptions, ACurrentCandidate.Transforms, ATemplateTransform)) return false;
            TetClusterCandidate Candidate = ACurrentCandidate;
            Candidate.OriginalIndices.push_back(ATemplateTransform.OriginalId);
            Candidate.Transforms.push_back(ATemplateTransform);
            const double Area = std::abs(static_cast<double>(AOriginalItems[ATemplateTransform.OriginalId].area()));
            if (!std::isfinite(Area) || Area <= 0.0) return false;
            Candidate.RealArea += Area;
            Candidate.BaselineArea += std::max(Area, AFeatures[ATemplateTransform.OriginalId].BoxArea);
            Candidate.OccupiedArea += Area;
            Candidate.ReservedArea = std::min(Candidate.ProxyArea, Candidate.ReservedArea + Area);
            Candidate.ProxyWasteArea = std::max(0.0, Candidate.ProxyArea - Candidate.ReservedArea);
            Candidate.ProxyWasteRatio = Candidate.ProxyArea > 0.0 ? Candidate.ProxyWasteArea / Candidate.ProxyArea : 1.0;
            Candidate.FillRatio = Candidate.ProxyArea > 0.0 ? std::clamp(Candidate.RealArea / Candidate.ProxyArea, 0.0, 1.0) : 0.0;
            Candidate.BoundingFillRatio = Candidate.FillRatio;
            Candidate.AreaSavingRatio = Candidate.BaselineArea > 0.0 ? 1.0 - Candidate.ProxyArea / Candidate.BaselineArea : 0.0;
            Candidate.BuilderName = "EnvelopeFillSearch";
            Candidate.ClusterType = ABaseCandidate.ClusterType + "_EnvelopeFill";
            AOutCandidate = std::move(Candidate);
            return true;
        }

        bool CetRectangleFillClusterBuilder::_TryFindFillerTransform(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ACurrentCandidate, const std::vector<TetClusterFreeRegion>& AFreeRegions, int AFillerIndex, const TetNestOptions& AOptions, double AEnvelopeWidth, double AEnvelopeHeight, TetItemTransform& AOutTransform) const
        {
            if (AFillerIndex < 0 || AFillerIndex >= static_cast<int>(AOriginalItems.size()) || AFillerIndex >= static_cast<int>(AFeatures.size()) || _ContainsOriginalIndex(ACurrentCandidate, AFillerIndex)) {
                return false;
            }

            CetClusterGeometryHelper Geometry;
            const double RequiredGap = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));
            const std::vector<double> Rotations = CetRotationUtils::BuildAllowedRotations(AOptions.Rotations);
            if (Rotations.empty()) return false;

            bool FoundPlacement = false;
            double BestPlacementScore = -std::numeric_limits<double>::infinity();
            TetItemTransform BestTransform;

            for (double Rotation : Rotations) {
                const CetPath RotatedFiller = Geometry.TransformContour(Geometry.GetIdentityContour(AOriginalItems[AFillerIndex]), Rotation, 0.0, 0.0);
                double FillerMinX = 0.0, FillerMinY = 0.0, FillerMaxX = 0.0, FillerMaxY = 0.0;
                if (!Geometry.GetBounds(RotatedFiller, FillerMinX, FillerMinY, FillerMaxX, FillerMaxY)) {
                    continue;
                }

                const double FillerWidth = FillerMaxX - FillerMinX;
                const double FillerHeight = FillerMaxY - FillerMinY;
                if (FillerWidth <= 0.0 || FillerHeight <= 0.0 || FillerWidth > AEnvelopeWidth + CET_RECTANGLE_FILL_POSITION_TOLERANCE || FillerHeight > AEnvelopeHeight + CET_RECTANGLE_FILL_POSITION_TOLERANCE) {
                    continue;
                }

                const double MaxX = AEnvelopeWidth - FillerWidth;
                const double MaxY = AEnvelopeHeight - FillerHeight;

                // 初始化 Probe 探测上下文结构体并传递
                TetProbeContext ProbeCtx{
                    AOriginalItems, AFeatures, ACurrentCandidate, AFillerIndex,
                    RotatedFiller, FillerMinX, FillerMinY, FillerWidth, FillerHeight,
                    RequiredGap, MaxX, MaxY
                };
                std::vector<std::pair<double, double>> ProbePositions;
                _BuildProbePositions(ProbeCtx, ProbePositions);

                const std::size_t EllipseCount = static_cast<std::size_t>(std::count_if(
                    ACurrentCandidate.Transforms.begin(), ACurrentCandidate.Transforms.end(),
                    [&](const TetItemTransform& Transform) {
                        return Transform.OriginalId >= 0
                            && Transform.OriginalId < static_cast<int>(AFeatures.size())
                            && AFeatures[Transform.OriginalId].ShapeType == MetShapeType::EllipseLike;
                    }));
                const bool HasDenseEllipseSkeleton = EllipseCount >= 8;
                if (HasDenseEllipseSkeleton) {
                    std::vector<std::pair<double, double>> FreeRegionPositions;
                    _BuildFreeRegionProbePositions(ProbeCtx, AFreeRegions, FreeRegionPositions);
                    // Keep both generic and contour-contact probes. The exact
                    // free-region vertices cover narrow pockets that a regular
                    // ellipse grid cannot represent.
                    const std::size_t FillerCount = ACurrentCandidate.Transforms.size()
                        - std::min(ACurrentCandidate.SkeletonChildCount, ACurrentCandidate.Transforms.size());
                    if (FillerCount >= CET_ELLIPSE_GAP_FILL_MAX_COMPOSITE_DEPTH) {
                        FreeRegionPositions.insert(FreeRegionPositions.end(), ProbePositions.begin(), ProbePositions.end());
                        ProbePositions = std::move(FreeRegionPositions);
                    } else {
                        ProbePositions.insert(ProbePositions.end(), FreeRegionPositions.begin(), FreeRegionPositions.end());
                    }
                }
                const std::size_t ProbeLimit = HasDenseEllipseSkeleton
                    ? CET_ELLIPSE_GAP_FILL_FREE_REGION_PROBE_COUNT
                    : (AOriginalItems.size() > CET_NEST_FULL_STRATEGY_ITEM_LIMIT
                        ? CET_RECTANGLE_FILL_LARGE_ORDER_MAX_PROBE_COUNT : ProbePositions.size());
                const std::size_t ProbeCount = std::min(ProbePositions.size(), ProbeLimit);

                for (std::size_t ProbeIndex = 0; ProbeIndex < ProbeCount; ++ProbeIndex) {
                    const auto& ProbePosition = ProbePositions[ProbeIndex];
                    const double PositionX = ProbePosition.first;
                    const double PositionY = ProbePosition.second;

                    if (PositionX < -CET_RECTANGLE_FILL_POSITION_TOLERANCE || PositionY < -CET_RECTANGLE_FILL_POSITION_TOLERANCE || PositionX > MaxX + CET_RECTANGLE_FILL_POSITION_TOLERANCE || PositionY > MaxY + CET_RECTANGLE_FILL_POSITION_TOLERANCE) {
                        continue;
                    }

                    TetItemTransform FillerTransform;
                    FillerTransform.OriginalId = AFillerIndex;
                    FillerTransform.RelativeRotation = Rotation;
                    FillerTransform.RelativeX = PositionX - FillerMinX;
                    FillerTransform.RelativeY = PositionY - FillerMinY;

                    const CetPath FillerContour = Geometry.TransformContour(RotatedFiller, 0.0,
                        FillerTransform.RelativeX, FillerTransform.RelativeY);
                    if (!_IsContourInsideFreeRegions(FillerContour, AFreeRegions)
                        || !Geometry.CanAppendTransformWithSpacing(AOriginalItems, AOptions, ACurrentCandidate.Transforms, FillerTransform)) {
                        continue;
                    }

                    if (HasDenseEllipseSkeleton) {
                        AOutTransform = FillerTransform;
                        return true;
                    }

                    const double PlacementScore = _CalculatePlacementScore(AOriginalItems, ACurrentCandidate, PositionX, PositionY, PositionX + FillerWidth, PositionY + FillerHeight, RequiredGap);
                    if (!FoundPlacement || PlacementScore > BestPlacementScore + 1e-9) {
                        FoundPlacement = true;
                        BestPlacementScore = PlacementScore;
                        BestTransform = FillerTransform;
                    }
                }
            }

            if (!FoundPlacement) {
                return false;
            }
            AOutTransform = BestTransform;
            return true;
        }

        bool CetRectangleFillClusterBuilder::_IsContourInsideFreeRegions(const CetPath& AContour, const std::vector<TetClusterFreeRegion>& AFreeRegions) const
        {
            CetClusterGeometryHelper Geometry;
            for (const TetClusterFreeRegion& FreeRegion : AFreeRegions) {
                const double Tolerance = std::max(1.0, FreeRegion.Area * CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE);
                if (Geometry.IsContourInsideFreeRegion(AContour, FreeRegion, Tolerance)) return true;
            }
            return false;
        }
        void CetRectangleFillClusterBuilder::_BuildProbePositions(const TetProbeContext& ACtx, std::vector<std::pair<double, double>>& AOutPositions) const
        {
            AOutPositions.clear();
            if (ACtx.MaxX < 0.0 || ACtx.MaxY < 0.0) {
                return;
            }

            const bool HasCircleSkeleton = std::any_of(ACtx.Candidate.Transforms.begin(), ACtx.Candidate.Transforms.end(), [&](const TetItemTransform& Transform) {
                return Transform.OriginalId >= 0 && Transform.OriginalId < static_cast<int>(ACtx.Features.size())
                    && ACtx.Features[Transform.OriginalId].ShapeType == MetShapeType::CircleLike;
                });
            const bool HasEllipseSkeleton = std::any_of(ACtx.Candidate.Transforms.begin(), ACtx.Candidate.Transforms.end(), [&](const TetItemTransform& Transform) {
                return Transform.OriginalId >= 0 && Transform.OriginalId < static_cast<int>(ACtx.Features.size())
                    && ACtx.Features[Transform.OriginalId].ShapeType == MetShapeType::EllipseLike;
                });
            if (HasCircleSkeleton) {
                _BuildCircleProbePositions(ACtx, AOutPositions);
            }
            if (HasEllipseSkeleton) {
                _BuildEllipseProbePositions(ACtx, AOutPositions);
            }

            auto AddPosition = [&](double AX, double AY) {
                _AppendProbePosition(AOutPositions, AX, AY, ACtx.MaxX, ACtx.MaxY);
                };

            AddPosition(0.0, 0.0);
            AddPosition(ACtx.MaxX, 0.0);
            AddPosition(0.0, ACtx.MaxY);
            AddPosition(ACtx.MaxX, ACtx.MaxY);
            AddPosition(ACtx.MaxX * 0.5, ACtx.MaxY * 0.5);

			// Geometry-driven contact probes must precede the uniform grid. Large
			// orders intentionally truncate the probe list, and putting the grid
			// first starved left/right/bottom ellipse-corner positions.
			const bool NeedsChildContourProbes = HasEllipseSkeleton
				|| (!HasCircleSkeleton && !HasEllipseSkeleton)
				|| AOutPositions.size() < CET_CIRCLE_FILL_SPECIALIZED_PROBE_THRESHOLD;
			if (NeedsChildContourProbes) {
				_BuildChildContourProbePositions(ACtx, AOutPositions);
			}

            for (int Row = 0; Row < CET_RECTANGLE_FILL_GRID_PROBE_COUNT; ++Row) {
                const double YRatio = static_cast<double>(Row) / static_cast<double>(CET_RECTANGLE_FILL_GRID_PROBE_COUNT - 1);
                for (int Column = 0; Column < CET_RECTANGLE_FILL_GRID_PROBE_COUNT; ++Column) {
                    const double XRatio = static_cast<double>(Column) / static_cast<double>(CET_RECTANGLE_FILL_GRID_PROBE_COUNT - 1);
                    AddPosition(ACtx.MaxX * XRatio, ACtx.MaxY * YRatio);
                    if (AOutPositions.size() >= CET_RECTANGLE_FILL_MAX_PROBE_COUNT) {
                        return;
                    }
                }
            }
        }
        void CetRectangleFillClusterBuilder::_BuildCircleProbePositions(const TetProbeContext& ACtx, std::vector<std::pair<double, double>>& AOutPositions) const
        {
            std::vector<TetCircleCenter> CircleCenters;
            CircleCenters.reserve(ACtx.Candidate.Transforms.size());
            CetClusterGeometryHelper Geometry;

            for (const TetItemTransform& Transform : ACtx.Candidate.Transforms) {
                if (Transform.OriginalId < 0 || Transform.OriginalId >= static_cast<int>(ACtx.Features.size()) || ACtx.Features[Transform.OriginalId].ShapeType != MetShapeType::CircleLike) {
                    continue;
                }
                const CetPath Child = Geometry.TransformContour(Geometry.GetIdentityContour(ACtx.OriginalItems[Transform.OriginalId]), Transform.RelativeRotation, Transform.RelativeX, Transform.RelativeY);
                double MinX = 0.0, MinY = 0.0, ChildMaxX = 0.0, ChildMaxY = 0.0;
                if (!Geometry.GetBounds(Child, MinX, MinY, ChildMaxX, ChildMaxY)) {
                    continue;
                }
                const double Width = ChildMaxX - MinX;
                const double Height = ChildMaxY - MinY;
                if (Width <= 0.0 || Height <= 0.0 || std::abs(Width - Height) > std::max(1.0, std::max(Width, Height) * 0.02)) {
                    continue;
                }
                CircleCenters.push_back({ (MinX + ChildMaxX) * 0.5, (MinY + ChildMaxY) * 0.5, std::min(Width, Height) * 0.5 });
            }

            // Probe the midpoint as well as both sides of every pair.  The
            // midpoint is important for the triangular void between three
            // circles; the side probes cover exterior/corner voids.  Exact
            // contour and spacing checks below reject false positives.
            const double FillerRadius = std::min(ACtx.FillerWidth, ACtx.FillerHeight) * 0.5;
            std::size_t PairProbeCount = 0;
            const std::size_t MaxPairProbes = std::max<std::size_t>(1, std::min(
                CET_CIRCLE_FILL_MAX_PAIR_PROBES, CET_RECTANGLE_FILL_MAX_PROBE_COUNT / 6));

            for (std::size_t First = 0; First < CircleCenters.size() && PairProbeCount < MaxPairProbes; ++First) {
                for (std::size_t Second = First + 1; Second < CircleCenters.size() && PairProbeCount < MaxPairProbes; ++Second) {
                    const TetCircleCenter& A = CircleCenters[First];
                    const TetCircleCenter& B = CircleCenters[Second];
                    const double DeltaX = B.X - A.X;
                    const double DeltaY = B.Y - A.Y;
                    const double CenterDistance = std::hypot(DeltaX, DeltaY);
                    const double RequiredDistance = std::max(A.Radius, B.Radius) + FillerRadius + ACtx.RequiredGap;

                    if (CenterDistance <= 1e-9 || CenterDistance * 0.5 > RequiredDistance) {
                        continue;
                    }
                    const double OffsetSquared = RequiredDistance * RequiredDistance - CenterDistance * CenterDistance * 0.25;
                    if (OffsetSquared < 0.0) {
                        continue;
                    }
                    const double Offset = std::sqrt(OffsetSquared);
                    const double PerpendicularX = -DeltaY / CenterDistance;
                    const double PerpendicularY = DeltaX / CenterDistance;
                    const double MidX = (A.X + B.X) * 0.5;
                    const double MidY = (A.Y + B.Y) * 0.5;

                    for (double Scale : { 0.0, 0.5, 1.0, 1.5, 2.0, 2.5 }) {
                        const double SideOffset = Offset * Scale;
                        _AppendProbePosition(AOutPositions, MidX + PerpendicularX * SideOffset - ACtx.FillerWidth * 0.5,
                            MidY + PerpendicularY * SideOffset - ACtx.FillerHeight * 0.5, ACtx.MaxX, ACtx.MaxY);
                        if (Scale > 0.0) _AppendProbePosition(AOutPositions, MidX - PerpendicularX * SideOffset - ACtx.FillerWidth * 0.5,
                            MidY - PerpendicularY * SideOffset - ACtx.FillerHeight * 0.5, ACtx.MaxX, ACtx.MaxY);
                    }
                    ++PairProbeCount;
                }
            }

            // A three-circle void has a different optimum from any pair
            // midpoint.  Its circumcenter is the only useful seed when the
            // spacing leaves a very narrow triangular pocket.
            std::size_t TripleProbeCount = 0;
            for (std::size_t First = 0; First < CircleCenters.size() && TripleProbeCount < MaxPairProbes; ++First) for (std::size_t Second = First + 1; Second < CircleCenters.size() && TripleProbeCount < MaxPairProbes; ++Second) for (std::size_t Third = Second + 1; Third < CircleCenters.size() && TripleProbeCount < MaxPairProbes; ++Third) {
                const double AB = std::hypot(CircleCenters[Second].X - CircleCenters[First].X, CircleCenters[Second].Y - CircleCenters[First].Y);
                const double AC = std::hypot(CircleCenters[Third].X - CircleCenters[First].X, CircleCenters[Third].Y - CircleCenters[First].Y);
                const double BC = std::hypot(CircleCenters[Third].X - CircleCenters[Second].X, CircleCenters[Third].Y - CircleCenters[Second].Y);
                const double MinSide = std::min({ AB, AC, BC }), MaxSide = std::max({ AB, AC, BC });
                const double Cross = (CircleCenters[Second].X - CircleCenters[First].X) * (CircleCenters[Third].Y - CircleCenters[First].Y) - (CircleCenters[Second].Y - CircleCenters[First].Y) * (CircleCenters[Third].X - CircleCenters[First].X);
                if (MinSide <= CET_RECTANGLE_FILL_POSITION_TOLERANCE || MaxSide > MinSide * 1.15 || std::abs(Cross) <= CET_RECTANGLE_FILL_POSITION_TOLERANCE) continue;
                const double FirstSq = CircleCenters[First].X * CircleCenters[First].X + CircleCenters[First].Y * CircleCenters[First].Y;
                const double SecondSq = CircleCenters[Second].X * CircleCenters[Second].X + CircleCenters[Second].Y * CircleCenters[Second].Y;
                const double ThirdSq = CircleCenters[Third].X * CircleCenters[Third].X + CircleCenters[Third].Y * CircleCenters[Third].Y;
                const double CenterX = (FirstSq * (CircleCenters[Second].Y - CircleCenters[Third].Y) + SecondSq * (CircleCenters[Third].Y - CircleCenters[First].Y) + ThirdSq * (CircleCenters[First].Y - CircleCenters[Second].Y)) / (2.0 * Cross);
                const double CenterY = (FirstSq * (CircleCenters[Third].X - CircleCenters[Second].X) + SecondSq * (CircleCenters[First].X - CircleCenters[Third].X) + ThirdSq * (CircleCenters[Second].X - CircleCenters[First].X)) / (2.0 * Cross);
                _AppendProbePosition(AOutPositions, CenterX - ACtx.FillerWidth * 0.5, CenterY - ACtx.FillerHeight * 0.5, ACtx.MaxX, ACtx.MaxY);
                ++TripleProbeCount;
            }
        }
        void CetRectangleFillClusterBuilder::_BuildEllipseProbePositions(const TetProbeContext& ACtx, std::vector<std::pair<double, double>>& AOutPositions) const
        {
            std::vector<TetEllipseCenter> Centers;
            CetClusterGeometryHelper Geometry;
            for (const TetItemTransform& Transform : ACtx.Candidate.Transforms) {
                if (Transform.OriginalId < 0 || Transform.OriginalId >= static_cast<int>(ACtx.Features.size())
                    || ACtx.Features[Transform.OriginalId].ShapeType != MetShapeType::EllipseLike) continue;
                const CetPath Contour = Geometry.TransformContour(Geometry.GetIdentityContour(
                    ACtx.OriginalItems[Transform.OriginalId]), Transform.RelativeRotation,
                    Transform.RelativeX, Transform.RelativeY);
                double MinX = 0.0, MinY = 0.0, MaxX = 0.0, MaxY = 0.0;
                if (Geometry.GetBounds(Contour, MinX, MinY, MaxX, MaxY)) {
                    Centers.push_back({ (MinX + MaxX) * 0.5, (MinY + MaxY) * 0.5,
                        (MaxX - MinX) * 0.5, (MaxY - MinY) * 0.5 });
                }
            }
            const auto AddCentered = [&](double AX, double AY) {
                _AppendProbePosition(AOutPositions, AX - ACtx.FillerWidth * 0.5,
                    AY - ACtx.FillerHeight * 0.5, ACtx.MaxX, ACtx.MaxY);
            };
            std::vector<std::vector<std::size_t>> Neighbors;
            _BuildEllipseNeighborLists(Centers, Neighbors);
            std::size_t PairCount = 0;
            for (std::size_t First = 0; First < Neighbors.size() && PairCount < CET_ELLIPSE_GAP_FILL_MAX_PAIR_PROBES; ++First) {
                for (std::size_t Second : Neighbors[First]) {
                    if (Second <= First) continue;
                    const double DeltaX = Centers[Second].X - Centers[First].X;
                    const double DeltaY = Centers[Second].Y - Centers[First].Y;
                    const double Distance = std::hypot(DeltaX, DeltaY);
                    if (Distance <= CET_RECTANGLE_FILL_POSITION_TOLERANCE) continue;
                    const double NormalX = -DeltaY / Distance;
                    const double NormalY = DeltaX / Distance;
                    const double MidX = (Centers[First].X + Centers[Second].X) * 0.5;
                    const double MidY = (Centers[First].Y + Centers[Second].Y) * 0.5;
                    AddCentered(MidX, MidY);
                    const double Offset = std::min({ Centers[First].HalfWidth, Centers[First].HalfHeight,
                        Centers[Second].HalfWidth, Centers[Second].HalfHeight }) * 0.25;
                    AddCentered(MidX + NormalX * Offset, MidY + NormalY * Offset);
                    AddCentered(MidX - NormalX * Offset, MidY - NormalY * Offset);
                    ++PairCount;
                }
            }
            std::size_t TripleCount = 0;
            for (std::size_t First = 0; First < Neighbors.size() && TripleCount < CET_ELLIPSE_GAP_FILL_MAX_TRIPLE_PROBES; ++First) {
                const std::vector<std::size_t>& Local = Neighbors[First];
                for (std::size_t Left = 0; Left < Local.size() && TripleCount < CET_ELLIPSE_GAP_FILL_MAX_TRIPLE_PROBES; ++Left) {
                    for (std::size_t Right = Left + 1; Right < Local.size() && TripleCount < CET_ELLIPSE_GAP_FILL_MAX_TRIPLE_PROBES; ++Right) {
                        const std::size_t Second = Local[Left], Third = Local[Right];
                        const double Cross = (Centers[Second].X - Centers[First].X) * (Centers[Third].Y - Centers[First].Y)
                            - (Centers[Second].Y - Centers[First].Y) * (Centers[Third].X - Centers[First].X);
                        if (std::abs(Cross) <= CET_RECTANGLE_FILL_POSITION_TOLERANCE) continue;
                        AddCentered((Centers[First].X + Centers[Second].X + Centers[Third].X) / 3.0,
                            (Centers[First].Y + Centers[Second].Y + Centers[Third].Y) / 3.0);
                        ++TripleCount;
                    }
                }
            }
        }
        void CetRectangleFillClusterBuilder::_BuildEllipseNeighborLists(
            const std::vector<TetEllipseCenter>& ACenters,
            std::vector<std::vector<std::size_t>>& AOutNeighbors) const
        {
            double MaxExtent = 0.0;
            for (const TetEllipseCenter& Center : ACenters) {
                MaxExtent = std::max(MaxExtent, std::max(Center.HalfWidth, Center.HalfHeight));
            }
            const double CellSize = std::max(1.0, MaxExtent * 2.0);
            std::map<std::pair<long long, long long>, std::vector<std::size_t>> Grid;
            for (std::size_t Index = 0; Index < ACenters.size(); ++Index) {
                Grid[{ static_cast<long long>(std::floor(ACenters[Index].X / CellSize)),
                    static_cast<long long>(std::floor(ACenters[Index].Y / CellSize)) }].push_back(Index);
            }
            AOutNeighbors.assign(ACenters.size(), {});
            for (std::size_t First = 0; First < ACenters.size(); ++First) {
                std::vector<std::pair<double, std::size_t>> Ranked;
                const long long CellX = static_cast<long long>(std::floor(ACenters[First].X / CellSize));
                const long long CellY = static_cast<long long>(std::floor(ACenters[First].Y / CellSize));
                const double Reach = std::max(ACenters[First].HalfWidth, ACenters[First].HalfHeight) * 5.0;
                for (long long Y = CellY - 2; Y <= CellY + 2; ++Y) for (long long X = CellX - 2; X <= CellX + 2; ++X) {
                    const auto It = Grid.find({ X, Y });
                    if (It == Grid.end()) continue;
                    for (std::size_t Second : It->second) {
                        if (First == Second) continue;
                        const double Distance = std::hypot(ACenters[Second].X - ACenters[First].X,
                            ACenters[Second].Y - ACenters[First].Y);
                        if (Distance <= Reach + std::max(ACenters[Second].HalfWidth, ACenters[Second].HalfHeight) * 2.0) {
                            Ranked.push_back({ Distance, Second });
                        }
                    }
                }
                std::stable_sort(Ranked.begin(), Ranked.end());
                if (Ranked.size() > CET_ELLIPSE_GAP_FILL_MAX_NEIGHBORS) Ranked.resize(CET_ELLIPSE_GAP_FILL_MAX_NEIGHBORS);
                for (const auto& Entry : Ranked) AOutNeighbors[First].push_back(Entry.second);
            }
        }
        void CetRectangleFillClusterBuilder::_BuildFreeRegionProbePositions(const TetProbeContext& ACtx,
            const std::vector<TetClusterFreeRegion>& AFreeRegions,
            std::vector<std::pair<double, double>>& AOutPositions) const
        {
            AOutPositions.clear();
            const auto Append = [&](double AX, double AY) {
                if (!std::isfinite(AX) || !std::isfinite(AY) || AX < 0.0 || AY < 0.0
                    || AX > ACtx.MaxX || AY > ACtx.MaxY
                    || AOutPositions.size() >= CET_ELLIPSE_GAP_FILL_FREE_REGION_PROBE_COUNT) return;
                for (const auto& Existing : AOutPositions) {
                    if (std::abs(Existing.first - AX) <= CET_RECTANGLE_FILL_POSITION_TOLERANCE
                        && std::abs(Existing.second - AY) <= CET_RECTANGLE_FILL_POSITION_TOLERANCE) return;
                }
                AOutPositions.emplace_back(AX, AY);
            };
            const auto AddContour = [&](const CetPath& AContour) {
                if (AContour.empty()) return;
                const std::size_t Step = std::max<std::size_t>(1, AContour.size() / 4);
                for (std::size_t Index = 0; Index < AContour.size(); Index += Step) {
                    for (const ClipperLib::IntPoint& FillerVertex : ACtx.RotatedFiller) {
                        Append(static_cast<double>(AContour[Index].X - FillerVertex.X) + ACtx.FillerMinX,
                            static_cast<double>(AContour[Index].Y - FillerVertex.Y) + ACtx.FillerMinY);
                    }
                }
            };
            for (const TetClusterFreeRegion& Region : AFreeRegions) {
                for (const CetPath& Hole : Region.Holes) AddContour(Hole);
                AddContour(Region.Contour);
            }
        }
        void CetRectangleFillClusterBuilder::_BuildChildContourProbePositions(const TetProbeContext& ACtx, std::vector<std::pair<double, double>>& AOutPositions) const
        {
            std::vector<double> XEvents{ 0.0, ACtx.MaxX };
            std::vector<double> YEvents{ 0.0, ACtx.MaxY };

            auto AddAxisEvent = [](std::vector<double>& AEvents, double AValue, double AMaximum) {
                if (!std::isfinite(AValue) || AValue < -CET_RECTANGLE_FILL_POSITION_TOLERANCE || AValue > AMaximum + CET_RECTANGLE_FILL_POSITION_TOLERANCE) {
                    return;
                }
				const double Quantized = std::clamp(AValue, 0.0, AMaximum);
                for (double Existing : AEvents) {
                    if (std::abs(Existing - Quantized) <= CET_RECTANGLE_FILL_POSITION_TOLERANCE) {
                        return;
                    }
                }
                if (AEvents.size() < CET_RECTANGLE_FILL_MAX_AXIS_COORDINATES) {
                    AEvents.push_back(Quantized);
                }
                };

            CetClusterGeometryHelper Geometry;
            for (const TetItemTransform& Transform : ACtx.Candidate.Transforms) {
                if (Transform.OriginalId < 0 || Transform.OriginalId >= static_cast<int>(ACtx.OriginalItems.size())) {
                    continue;
                }
                const CetPath Child = Geometry.TransformContour(Geometry.GetIdentityContour(ACtx.OriginalItems[Transform.OriginalId]), Transform.RelativeRotation, Transform.RelativeX, Transform.RelativeY);
                double MinX = 0.0, MinY = 0.0, ChildMaxX = 0.0, ChildMaxY = 0.0;
                if (!Geometry.GetBounds(Child, MinX, MinY, ChildMaxX, ChildMaxY)) {
                    continue;
                }

                _AppendProbePosition(AOutPositions, ChildMaxX + ACtx.RequiredGap, MinY, ACtx.MaxX, ACtx.MaxY);
                _AppendProbePosition(AOutPositions, MinX - ACtx.FillerWidth - ACtx.RequiredGap, MinY, ACtx.MaxX, ACtx.MaxY);
                _AppendProbePosition(AOutPositions, MinX, ChildMaxY + ACtx.RequiredGap, ACtx.MaxX, ACtx.MaxY);
                _AppendProbePosition(AOutPositions, MinX, MinY - ACtx.FillerHeight - ACtx.RequiredGap, ACtx.MaxX, ACtx.MaxY);
                _AppendProbePosition(AOutPositions, ChildMaxX + ACtx.RequiredGap, ChildMaxY + ACtx.RequiredGap, ACtx.MaxX, ACtx.MaxY);

                AddAxisEvent(XEvents, ChildMaxX + ACtx.RequiredGap, ACtx.MaxX);
                AddAxisEvent(XEvents, MinX - ACtx.FillerWidth - ACtx.RequiredGap, ACtx.MaxX);
                AddAxisEvent(YEvents, ChildMaxY + ACtx.RequiredGap, ACtx.MaxY);
                AddAxisEvent(YEvents, MinY - ACtx.FillerHeight - ACtx.RequiredGap, ACtx.MaxY);

                for (const ClipperLib::IntPoint& ChildVertex : Child) {
                    for (const ClipperLib::IntPoint& FillerVertex : ACtx.RotatedFiller) {
                        AddAxisEvent(XEvents, static_cast<double>(ChildVertex.X - FillerVertex.X) + ACtx.FillerMinX + ACtx.RequiredGap, ACtx.MaxX);
                        AddAxisEvent(XEvents, static_cast<double>(ChildVertex.X - FillerVertex.X) + ACtx.FillerMinX - ACtx.RequiredGap, ACtx.MaxX);
                        AddAxisEvent(YEvents, static_cast<double>(ChildVertex.Y - FillerVertex.Y) + ACtx.FillerMinY + ACtx.RequiredGap, ACtx.MaxY);
                        AddAxisEvent(YEvents, static_cast<double>(ChildVertex.Y - FillerVertex.Y) + ACtx.FillerMinY - ACtx.RequiredGap, ACtx.MaxY);
                    }
                }
            }

            std::sort(XEvents.begin(), XEvents.end());
            std::sort(YEvents.begin(), YEvents.end());

            for (double Y : YEvents) {
                for (double X : XEvents) {
                    _AppendProbePosition(AOutPositions, X, Y, ACtx.MaxX, ACtx.MaxY);
                    if (AOutPositions.size() >= CET_RECTANGLE_FILL_MAX_PROBE_COUNT) {
                        return;
                    }
                }
            }
        }
        double CetRectangleFillClusterBuilder::_CalculatePlacementScore(const CetTNestItemVector& AOriginalItems, const TetClusterCandidate& ACandidate, double AFillerLeft, double AFillerTop, double AFillerRight, double AFillerBottom, double ARequiredGap) const
        {
            const double ContactTolerance = ARequiredGap + CET_RECTANGLE_FILL_POSITION_TOLERANCE;
            double Score = -(AFillerTop + AFillerLeft) / std::max(1.0, ACandidate.ClusterWidth + ACandidate.ClusterHeight);
            const double FillerCenterX = (AFillerLeft + AFillerRight) * 0.5;
            const double FillerCenterY = (AFillerTop + AFillerBottom) * 0.5;
            const double FillerSize = std::max(AFillerRight - AFillerLeft, AFillerBottom - AFillerTop);
            std::size_t NearbyCircleCount = 0;
            bool HasNonCircleFiller = false;
            CetClusterGeometryHelper Geometry;
            for (const TetItemTransform& Transform : ACandidate.Transforms){
                if (Transform.OriginalId < 0 || Transform.OriginalId >= static_cast<int>(AOriginalItems.size())){
                    continue;
                }
                const CetPath Child = Geometry.TransformContour(Geometry.GetIdentityContour(AOriginalItems[Transform.OriginalId]), Transform.RelativeRotation, Transform.RelativeX, Transform.RelativeY);
                double MinX = 0.0;
                double MinY = 0.0;
                double MaxX = 0.0;
                double MaxY = 0.0;
                if (!Geometry.GetBounds(Child, MinX, MinY, MaxX, MaxY)){
                    continue;
                }
                const double ChildWidth = MaxX - MinX;
                const double ChildHeight = MaxY - MinY;
                const bool IsCircleLike = Child.size() >= 12 && ChildWidth > 0.0 && ChildHeight > 0.0
                    && std::abs(ChildWidth - ChildHeight) <= std::max(1.0, std::max(ChildWidth, ChildHeight) * 0.02);
                if (!IsCircleLike) HasNonCircleFiller = true;
                const double CircleReach = std::min(ChildWidth, ChildHeight) * 0.5 + FillerSize + ARequiredGap;
                if (IsCircleLike && std::hypot(FillerCenterX - (MinX + MaxX) * 0.5, FillerCenterY - (MinY + MaxY) * 0.5) <= CircleReach) ++NearbyCircleCount;
                const bool VerticalOverlap = AFillerTop <= MaxY + CET_RECTANGLE_FILL_POSITION_TOLERANCE && AFillerBottom >= MinY - CET_RECTANGLE_FILL_POSITION_TOLERANCE;
                const bool HorizontalOverlap = AFillerLeft <= MaxX + CET_RECTANGLE_FILL_POSITION_TOLERANCE && AFillerRight >= MinX - CET_RECTANGLE_FILL_POSITION_TOLERANCE;
                const double HorizontalGap = std::max({ MinX - AFillerRight, AFillerLeft - MaxX, 0.0 });
                const double VerticalGap = std::max({ MinY - AFillerBottom, AFillerTop - MaxY, 0.0 });
                if (VerticalOverlap && HorizontalGap <= ContactTolerance){
                    Score += 3.0;
                }
                if (HorizontalOverlap && VerticalGap <= ContactTolerance){
                    Score += 3.0;
                }
            }
            if (NearbyCircleCount >= 3 && !HasNonCircleFiller) Score += 20.0;
            return Score;
        }

        void CetRectangleFillClusterBuilder::_AppendProbePosition(std::vector<std::pair<double, double>>& APositions, double AX, double AY, double AMaxX, double AMaxY) const
        {
            if (APositions.size() >= CET_RECTANGLE_FILL_MAX_PROBE_COUNT){
                return;
            }
            if (!std::isfinite(AX) || !std::isfinite(AY) || AX < -CET_RECTANGLE_FILL_POSITION_TOLERANCE || AY < -CET_RECTANGLE_FILL_POSITION_TOLERANCE || AX > AMaxX + CET_RECTANGLE_FILL_POSITION_TOLERANCE || AY > AMaxY + CET_RECTANGLE_FILL_POSITION_TOLERANCE){
                return;
            }

			const double QuantizedX = std::clamp(AX, 0.0, AMaxX);
			const double QuantizedY = std::clamp(AY, 0.0, AMaxY);
            for (const auto& Existing : APositions){
                if (std::abs(Existing.first - QuantizedX) <= CET_RECTANGLE_FILL_POSITION_TOLERANCE && std::abs(Existing.second - QuantizedY) <= CET_RECTANGLE_FILL_POSITION_TOLERANCE){
                    return;
                }
            }
            APositions.emplace_back(QuantizedX, QuantizedY);
        }

        std::vector<int> CetRectangleFillClusterBuilder::_PrepareFillerIndices(const TetRectangleFillContext& ACtx, const TetClusterCandidate& ACandidate, double ABaseAvailableArea) const
        {
            const double BaseAreaTolerance = std::max(1.0, ACandidate.ClusterWidth * ACandidate.ClusterHeight * 1e-9);
            std::vector<int> FillerIndices;
            FillerIndices.reserve(ACtx.OriginalItems.size());

            for (int Index = 0; Index < static_cast<int>(ACtx.OriginalItems.size()); ++Index) {
                if (ACtx.Used[Index] || _ContainsOriginalIndex(ACandidate, Index)) {
                    continue;
                }
                if (ACtx.Features[Index].Area <= 0.0 || ACtx.Features[Index].Width <= 0.0 || ACtx.Features[Index].Height <= 0.0) {
                    continue;
                }
                if (_GetFeatureArea(ACtx.OriginalItems[Index], ACtx.Features[Index]) > ABaseAvailableArea + BaseAreaTolerance) {
                    continue;
                }
                FillerIndices.push_back(Index);
            }

            std::stable_sort(FillerIndices.begin(), FillerIndices.end(), [&](int AFirstIndex, int ASecondIndex) {
                const double FirstArea = _GetFeatureArea(ACtx.OriginalItems[AFirstIndex], ACtx.Features[AFirstIndex]);
                const double SecondArea = _GetFeatureArea(ACtx.OriginalItems[ASecondIndex], ACtx.Features[ASecondIndex]);
                if (std::abs(FirstArea - SecondArea) > 1.0) {
                    return FirstArea > SecondArea;
                }
                return AFirstIndex < ASecondIndex;
                });

            if (FillerIndices.size() > CET_RECTANGLE_FILL_MAX_CANDIDATE_ITEMS) {
                constexpr std::size_t SmallCandidateCount = 16;
                const std::size_t LargeCandidateCount = CET_RECTANGLE_FILL_MAX_CANDIDATE_ITEMS - SmallCandidateCount;
                std::vector<int> BoundedIndices;
                BoundedIndices.reserve(CET_RECTANGLE_FILL_MAX_CANDIDATE_ITEMS);
                BoundedIndices.insert(BoundedIndices.end(), FillerIndices.begin(), FillerIndices.begin() + static_cast<std::vector<int>::difference_type>(LargeCandidateCount));
                BoundedIndices.insert(BoundedIndices.end(), FillerIndices.end() - static_cast<std::vector<int>::difference_type>(SmallCandidateCount), FillerIndices.end());
                FillerIndices = std::move(BoundedIndices);
            }
            return FillerIndices;
        }

        std::uint64_t CetRectangleFillClusterBuilder::_BuildFillerShapeSignature(const TetShapeFeature& AFeature, int AIndex) const
        {
            std::uint64_t Hash = 1469598103934665603ULL;
            const auto Mix = [&](std::uint64_t AValue) {
                Hash ^= AValue;
                Hash *= 1099511628211ULL;
                };

            Mix(static_cast<std::uint64_t>(AFeature.ShapeType));
            Mix(static_cast<std::uint64_t>(AFeature.NormalizedContour.size()));
            for (const ClipperLib::IntPoint& Point : AFeature.NormalizedContour) {
                Mix(static_cast<std::uint64_t>(Point.X));
                Mix(static_cast<std::uint64_t>(Point.Y));
            }
            if (AFeature.HasHoles) {
                Mix(static_cast<std::uint64_t>(AIndex));
            }
            return Hash;
        }

        bool CetRectangleFillClusterBuilder::_ContainsOriginalIndex(const TetClusterCandidate& ACandidate, int AOriginalIndex) const
        {
            return std::find(ACandidate.OriginalIndices.begin(), ACandidate.OriginalIndices.end(), AOriginalIndex) != ACandidate.OriginalIndices.end();
        }

        double CetRectangleFillClusterBuilder::_GetFeatureArea(const CetNestItem& AItem, const TetShapeFeature& AFeature) const
        {
            const double FeatureArea = std::abs(AFeature.Area);
            if (FeatureArea > 0.0 && std::isfinite(FeatureArea)){
                return FeatureArea;
            }
            return std::abs(static_cast<double>(AItem.area()));
        }

    }
}
