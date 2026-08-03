#include "pch.h"
#include "Nest2D_GapFillClusterBuilder.h"
#include "Nest2D_ClusterGeometryHelper.h"
#include "Nest2D_RotationUtils.h"
#include "NestUtils.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iostream>
#include <limits>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace ET {
    namespace NEST2DMANAGERLIB {

        struct TetGapFillCollisionItem
        {
            CetNestItem Item;
            double MinX = 0.0;
            double MinY = 0.0;
            double MaxX = 0.0;
            double MaxY = 0.0;
        };

        namespace {
            constexpr int CET_GAPFILL_GRID_PROBE_COUNT = 11;
            constexpr std::size_t CET_GAPFILL_MAX_PROBE_COUNT = 160;
            constexpr std::size_t CET_GAPFILL_VALID_PLACEMENT_COUNT = 24;
            constexpr std::size_t CET_GAPFILL_EXACT_PLACEMENT_COUNT = 6;

            std::string MakeGapFillClusterType(const std::string& ACurrentType)
            {
                const std::string Marker = "_GapFill";
                const std::size_t MarkerPos = ACurrentType.find(Marker);
                const std::string BaseType = MarkerPos == std::string::npos? ACurrentType: ACurrentType.substr(0, MarkerPos);
                int NextFillCount = 1;
                if (MarkerPos != std::string::npos){
                    const std::size_t NumberPos = MarkerPos + Marker.size();
                    if (NumberPos < ACurrentType.size()){
                        int CurrentFillCount = 0;
                        bool HasNumber = false;
                        for (std::size_t CharacterIndex = NumberPos; CharacterIndex < ACurrentType.size(); ++CharacterIndex){
                            const unsigned char Character = static_cast<unsigned char>(ACurrentType[CharacterIndex]);
                            if (!std::isdigit(Character)){
                                break;
                            }
                            HasNumber = true;
                            CurrentFillCount = CurrentFillCount * 10 + (ACurrentType[CharacterIndex] - '0');
                        }

                        if (HasNumber){
                            NextFillCount = CurrentFillCount + 1;
                        }
                    }
                }

                return BaseType + Marker + std::to_string(NextFillCount);
            }

            double GetFeatureArea(const TetShapeFeature& AFeature)
            {
                return std::max(0.0, AFeature.Area);
            }

            void AppendNormalizedContourKey(const CetPath& AContour, std::vector<long long>& AKey)
            {
                AKey.push_back(static_cast<long long>(AContour.size()));
                if (AContour.empty()){
                    return;
                }

                ClipperLib::cInt MinX = AContour.front().X;
                ClipperLib::cInt MinY = AContour.front().Y;
                for (const CetInpoint& Point : AContour){
                    MinX = std::min(MinX,Point.X);
                    MinY = std::min(MinY,Point.Y);
                }
                std::vector<long long> BestCoordinates;
                for (std::size_t StartIndex = 0; StartIndex < AContour.size(); ++StartIndex){
                    std::vector<long long> Coordinates;
                    Coordinates.reserve(AContour.size() * 2);
                    for (std::size_t PointOffset = 0; PointOffset < AContour.size(); ++PointOffset){
                        const CetInpoint& Point = AContour[(StartIndex + PointOffset) % AContour.size()];
                        Coordinates.push_back(static_cast<long long>(Point.X - MinX));
                        Coordinates.push_back(static_cast<long long>(Point.Y - MinY));
                    }
                    if (BestCoordinates.empty() || Coordinates < BestCoordinates){
                        BestCoordinates = std::move(Coordinates);
                    }
                }
                AKey.insert(AKey.end(),BestCoordinates.begin(),BestCoordinates.end());
            }

            std::vector<long long> BuildFillerGeometryKey(const CetNestItem& AItem, const TetShapeFeature& AFeature)
            {
                CetNestItem IdentityItem = AItem;
                IdentityItem.translation(libnest2d::Point(0,0));
                IdentityItem.rotation(libnest2d::Radians(0.0));
                IdentityItem.inflation(0);
                const auto Shape = IdentityItem.transformedShape();

                std::vector<long long> Key;
                Key.reserve(8 + Shape.Contour.size() * 2);
                Key.push_back(static_cast<long long>(AFeature.ShapeType));
                Key.push_back(static_cast<long long>(AFeature.VertexCount));
                Key.push_back(static_cast<long long>(std::llround(AFeature.Area)));
                AppendNormalizedContourKey(Shape.Contour,Key);
                Key.push_back(static_cast<long long>(Shape.Holes.size()));
                for (const CetPath& Hole : Shape.Holes){
                    AppendNormalizedContourKey(Hole,Key);
                }
                return Key;
            }

            bool GetContourBounds(const CetPath& AContour, double& AOutMinX, double& AOutMinY, double& AOutMaxX, double& AOutMaxY)
            {
                if (AContour.size() < 3){
                    return false;
                }
                AOutMinX = AOutMaxX = static_cast<double>(AContour.front().X);
                AOutMinY = AOutMaxY = static_cast<double>(AContour.front().Y);
                for (const CetInpoint& Point : AContour){
                    AOutMinX = std::min(AOutMinX,static_cast<double>(Point.X));
                    AOutMinY = std::min(AOutMinY,static_cast<double>(Point.Y));
                    AOutMaxX = std::max(AOutMaxX,static_cast<double>(Point.X));
                    AOutMaxY = std::max(AOutMaxY,static_cast<double>(Point.Y));
                }
                return AOutMaxX > AOutMinX && AOutMaxY > AOutMinY;
            }

            bool BuildBaseCollisionItems(const CetTNestItemVector& AOriginalItems, const TetClusterCandidate& ABaseCandidate, double ARequiredGap, std::vector<TetGapFillCollisionItem>& AOutItems)
            {
                AOutItems.clear();
                AOutItems.reserve(ABaseCandidate.Transforms.size());
                for (const TetItemTransform& Transform : ABaseCandidate.Transforms){
                    if (Transform.OriginalId < 0 || Transform.OriginalId >= static_cast<int>(AOriginalItems.size())){
                        return false;
                    }

                    CetNestItem Item = AOriginalItems[Transform.OriginalId];
                    Item.translation(libnest2d::Point(static_cast<ClipperLib::cInt>(std::llround(Transform.RelativeX)),static_cast<ClipperLib::cInt>(std::llround(Transform.RelativeY))));
                    Item.rotation(libnest2d::Radians(Transform.RelativeRotation));
                    Item.inflation(0);
                    if (ARequiredGap > 0.0){
                        const auto OriginalInflation = Item.inflation();
                        Item.inflation(static_cast<decltype(OriginalInflation)>(ARequiredGap));
                    }
                    const auto Shape = Item.transformedShape();
                    TetGapFillCollisionItem CollisionItem{ std::move(Item) };
                    if (!GetContourBounds(Shape.Contour,CollisionItem.MinX,CollisionItem.MinY,CollisionItem.MaxX,CollisionItem.MaxY)){
                        return false;
                    }
                    AOutItems.push_back(std::move(CollisionItem));
                }
                return !AOutItems.empty();
            }

            bool CanPlaceWithCollisionCache(const CetTNestItemVector& AOriginalItems, const CetClusterGeometryHelper& AGeometry, const TetClusterCandidate& ABaseCandidate, const std::vector<TetGapFillCollisionItem>& ABaseCollisionItems, const CetPath& ARotatedFiller, const TetItemTransform& AFillerTransform, double AFillerMinX, double AFillerMinY, double AFillerMaxX, double AFillerMaxY)
            {
                if (AFillerTransform.OriginalId < 0 || AFillerTransform.OriginalId >= static_cast<int>(AOriginalItems.size())){
                    return false;
                }

                CetNestItem FillerItem = AOriginalItems[AFillerTransform.OriginalId];
                FillerItem.translation(libnest2d::Point(static_cast<ClipperLib::cInt>(std::llround(AFillerTransform.RelativeX)),static_cast<ClipperLib::cInt>(std::llround(AFillerTransform.RelativeY))));
                FillerItem.rotation(libnest2d::Radians(AFillerTransform.RelativeRotation));
                FillerItem.inflation(0);
                for (const TetGapFillCollisionItem& BaseItem : ABaseCollisionItems){
                    if (AFillerMaxX < BaseItem.MinX || AFillerMinX > BaseItem.MaxX ||AFillerMaxY < BaseItem.MinY || AFillerMinY > BaseItem.MaxY){
                        continue;
                    }
                    if (CetNestItem::intersects(BaseItem.Item,FillerItem)){
                        return false;
                    }
                }

                const CetPath FillerContour = AGeometry.TransformContour(ARotatedFiller,0.0,AFillerTransform.RelativeX,AFillerTransform.RelativeY);
                const double AreaTolerance = std::max(16.0,ABaseCandidate.ProxyArea * 1e-10);
                return AGeometry.IsContourFullyContained(FillerContour,ABaseCandidate.ProxyContour,AreaTolerance);
            }

            void AddProbePosition(std::vector<std::pair<double, double>>& AProbes, double AX, double AY, double AMaxX, double AMaxY, double ATolerance)
            {
                if (AProbes.size() >= CET_GAPFILL_MAX_PROBE_COUNT || !std::isfinite(AX) || !std::isfinite(AY)){
                    return;
                }

                if (AX < -ATolerance || AY < -ATolerance || AX > AMaxX + ATolerance || AY > AMaxY + ATolerance){
                    return;
                }

                AX = std::clamp(AX, 0.0, AMaxX);
                AY = std::clamp(AY, 0.0, AMaxY);

                for (const auto& Probe : AProbes){
                    if (std::abs(Probe.first - AX) <= ATolerance && std::abs(Probe.second - AY) <= ATolerance){
                        return;
                    }
                }

                AProbes.emplace_back(AX, AY);
            }

            struct TetGapFillChildBounds
            {
                double MinX = 0.0;
                double MinY = 0.0;
                double MaxX = 0.0;
                double MaxY = 0.0;
                double CenterX = 0.0;
                double CenterY = 0.0;
            };

            std::vector<std::pair<double, double>> BuildProbePositions(const CetTNestItemVector& AOriginalItems, const TetClusterCandidate& ABaseCandidate, double AFillerWidth, double AFillerHeight, double ARequiredGap)
            {
                std::vector<std::pair<double, double>> Probes;

                const double MaxX = ABaseCandidate.ClusterWidth - AFillerWidth;
                const double MaxY = ABaseCandidate.ClusterHeight - AFillerHeight;
                if (MaxX < 0.0 || MaxY < 0.0){
                    return Probes;
                }

                const double ProbeTolerance = std::max(1.0, std::min(AFillerWidth, AFillerHeight) * 0.02);
                AddProbePosition(Probes,MaxX * 0.5,MaxY * 0.5,MaxX,MaxY,ProbeTolerance);
                AddProbePosition(Probes,0.0,0.0,MaxX,MaxY,ProbeTolerance);
                AddProbePosition(Probes,MaxX,0.0,MaxX,MaxY,ProbeTolerance);
                AddProbePosition(Probes,0.0,MaxY,MaxX,MaxY,ProbeTolerance);
                AddProbePosition(Probes,MaxX,MaxY,MaxX,MaxY,ProbeTolerance);

                CetClusterGeometryHelper Geometry;
                std::vector<TetGapFillChildBounds> ChildBounds;
                std::vector<double> XContactPositions;
                std::vector<double> YContactPositions;

                for (const TetItemTransform& Transform : ABaseCandidate.Transforms){
                    if (Transform.OriginalId < 0 || Transform.OriginalId >= static_cast<int>(AOriginalItems.size())){
                        continue;
                    }

                    const CetPath ChildContour = Geometry.TransformContour(Geometry.GetIdentityContour(AOriginalItems[Transform.OriginalId]),Transform.RelativeRotation,Transform.RelativeX,Transform.RelativeY);
                    TetGapFillChildBounds Bounds;
                    if (!Geometry.GetBounds(ChildContour,Bounds.MinX,Bounds.MinY,Bounds.MaxX,Bounds.MaxY)){
                        continue;
                    }
                    Bounds.CenterX = (Bounds.MinX + Bounds.MaxX) * 0.5;
                    Bounds.CenterY = (Bounds.MinY + Bounds.MaxY) * 0.5;
                    ChildBounds.push_back(Bounds);

                    AddProbePosition(Probes,Bounds.CenterX - AFillerWidth * 0.5,Bounds.CenterY - AFillerHeight * 0.5,MaxX,MaxY,ProbeTolerance);
                    XContactPositions.push_back(Bounds.MinX - AFillerWidth - ARequiredGap);
                    XContactPositions.push_back(Bounds.MaxX + ARequiredGap);
                    XContactPositions.push_back(Bounds.CenterX - AFillerWidth * 0.5);
                    YContactPositions.push_back(Bounds.MinY - AFillerHeight - ARequiredGap);
                    YContactPositions.push_back(Bounds.MaxY + ARequiredGap);
                    YContactPositions.push_back(Bounds.CenterY - AFillerHeight * 0.5);
                }

                const double FillerCenterOffsetX = AFillerWidth * 0.5;
                const double FillerCenterOffsetY = AFillerHeight * 0.5;
                for (std::size_t CenterIndex = 0; CenterIndex < ChildBounds.size(); ++CenterIndex){
                    std::vector<std::pair<double, std::size_t>> Neighbors;
                    for (std::size_t NeighborCenterIndex = 0; NeighborCenterIndex < ChildBounds.size(); ++NeighborCenterIndex){
                        if (CenterIndex == NeighborCenterIndex){
                            continue;
                        }
                        const double DeltaX = ChildBounds[CenterIndex].CenterX - ChildBounds[NeighborCenterIndex].CenterX;
                        const double DeltaY = ChildBounds[CenterIndex].CenterY - ChildBounds[NeighborCenterIndex].CenterY;
                        const double Distance = std::sqrt(DeltaX * DeltaX + DeltaY * DeltaY);
                        Neighbors.emplace_back(Distance,NeighborCenterIndex);
                    }
                    std::sort(Neighbors.begin(),Neighbors.end(),[](const auto& AFirstNeighbor, const auto& ASecondNeighbor){
                            return AFirstNeighbor.first < ASecondNeighbor.first;
                        });
                    if (Neighbors.size() > 2){
                        Neighbors.resize(2);
                    }

                    for (const auto& Neighbor : Neighbors){
                        const TetGapFillChildBounds& NeighborBounds = ChildBounds[Neighbor.second];
                        const double CenterX = (ChildBounds[CenterIndex].CenterX + NeighborBounds.CenterX) * 0.5;
                        const double CenterY = (ChildBounds[CenterIndex].CenterY + NeighborBounds.CenterY) * 0.5;
                        AddProbePosition(Probes,CenterX - FillerCenterOffsetX,CenterY - FillerCenterOffsetY,MaxX,MaxY,ProbeTolerance);
                    }

                    for (std::size_t FirstNeighborOffset = 0; FirstNeighborOffset < Neighbors.size(); ++FirstNeighborOffset){
                        for (std::size_t SecondNeighborOffset = FirstNeighborOffset + 1; SecondNeighborOffset < Neighbors.size(); ++SecondNeighborOffset){
                            const std::size_t FirstNeighborIndex = Neighbors[FirstNeighborOffset].second;
                            const std::size_t SecondNeighborIndex = Neighbors[SecondNeighborOffset].second;
                            const double CenterX = (ChildBounds[CenterIndex].CenterX + ChildBounds[FirstNeighborIndex].CenterX + ChildBounds[SecondNeighborIndex].CenterX) / 3.0;
                            const double CenterY = (ChildBounds[CenterIndex].CenterY + ChildBounds[FirstNeighborIndex].CenterY + ChildBounds[SecondNeighborIndex].CenterY) / 3.0;
                            AddProbePosition(Probes,CenterX - FillerCenterOffsetX,CenterY - FillerCenterOffsetY,MaxX,MaxY,ProbeTolerance);
                        }
                    }
                }

                const auto SortContactPositions = [](std::vector<double>& APositions, double ACenter){
                    std::stable_sort(APositions.begin(),APositions.end(),[ACenter](double AFirst, double ASecond){
                            return std::abs(AFirst - ACenter) < std::abs(ASecond - ACenter);
                        });
                    if (APositions.size() > 6){
                        APositions.resize(6);
                    }
                };
                SortContactPositions(XContactPositions,MaxX * 0.5);
                SortContactPositions(YContactPositions,MaxY * 0.5);
                for (double XPosition : XContactPositions){
                    for (double YPosition : YContactPositions){
                        AddProbePosition(Probes,XPosition,YPosition,MaxX,MaxY,ProbeTolerance);
                    }
                }

                for (int ProbeY = 0; ProbeY < CET_GAPFILL_GRID_PROBE_COUNT; ++ProbeY){
                    const double YRatio = static_cast<double>(ProbeY) / static_cast<double>(CET_GAPFILL_GRID_PROBE_COUNT - 1);
                    for (int ProbeX = 0; ProbeX < CET_GAPFILL_GRID_PROBE_COUNT; ++ProbeX){
                        const double XRatio = static_cast<double>(ProbeX) / static_cast<double>(CET_GAPFILL_GRID_PROBE_COUNT - 1);
                        AddProbePosition(Probes,MaxX * XRatio,MaxY * YRatio,MaxX,MaxY,ProbeTolerance);
                    }
                }

                return Probes;
            }
        }

        CetGapFillClusterBuilder::CetGapFillClusterBuilder() : CetCoreObject() {}
        CetGapFillClusterBuilder::~CetGapFillClusterBuilder() {}

        void CetGapFillClusterBuilder::BuildCandidates(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<TetClusterCandidate>& ABaseCandidates, const TetNestOptions& AOptions, std::vector<TetClusterCandidate>& AOutCandidates)
        {
            if (AOriginalItems.empty() || AFeatures.size() != AOriginalItems.size() || ABaseCandidates.empty()){
                return;
            }

            const std::size_t OldCandidateCount = AOutCandidates.size();

            for (const TetClusterCandidate& BaseCandidate : ABaseCandidates){
                TetClusterCandidate Candidate;
                if (!_BuildGapFillCandidate(AOriginalItems, AFeatures, BaseCandidate, AOptions, nullptr, Candidate)){
                    continue;
                }

                AOutCandidates.push_back(std::move(Candidate));
                std::cout << "[GAPFILL][CANDIDATE] BaseType = " << BaseCandidate.ClusterType << ", ChildCount = " << AOutCandidates.back().OriginalIndices.size() << ", Score = " << AOutCandidates.back().Score << std::endl;
            }

            std::cout << "[GAPFILL][BUILD CANDIDATES] BaseCandidateCount = " << ABaseCandidates.size() << ", NewCandidateCount = " << AOutCandidates.size() - OldCandidateCount << std::endl;
        }

        bool CetGapFillClusterBuilder::BuildCandidateForBase(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ABaseCandidate, const TetNestOptions& AOptions, const std::vector<bool>& AUsed, TetClusterCandidate& AOutCandidate)
        {
            AOutCandidate = TetClusterCandidate{};
            if (AOriginalItems.empty() || AFeatures.size() != AOriginalItems.size() || AUsed.size() != AOriginalItems.size()){
                return false;
            }
            return _BuildGapFillCandidate(AOriginalItems, AFeatures, ABaseCandidate, AOptions, &AUsed, AOutCandidate);
        }
        bool CetGapFillClusterBuilder::_BuildGapFillCandidate(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ABaseCandidate, const TetNestOptions& AOptions, const std::vector<bool>* AUsed, TetClusterCandidate& AOutCandidate)
        {
            AOutCandidate = TetClusterCandidate{};

            if (!_IsSupportedBaseCandidate(ABaseCandidate) || !ABaseCandidate.Valid || ABaseCandidate.ProxyContour.size() < 3){
                return false;
            }

            bool HasFiller = false;
            TetClusterCandidate CurrentCandidate = ABaseCandidate;
            const double RequiredGap = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));

            const std::size_t MaximumFillSteps = AOriginalItems.size() - std::min(AOriginalItems.size(),CurrentCandidate.OriginalIndices.size());
            for (std::size_t FillStep = 0; FillStep < MaximumFillSteps; ++FillStep){
                bool HasBestNext = false;
                TetClusterCandidate BestNextCandidate;
                std::set<std::vector<long long>> CheckedGeometryKeys;
                std::vector<TetGapFillCollisionItem> BaseCollisionItems;
                if (!BuildBaseCollisionItems(AOriginalItems,CurrentCandidate,RequiredGap,BaseCollisionItems)){
                    break;
                }

                for (int Index = 0; Index < static_cast<int>(AFeatures.size()); ++Index){
                    if (AUsed != nullptr){
                        if (Index >= static_cast<int>(AUsed->size()) || (*AUsed)[Index]){
                            continue;
                        }
                    }

                    if (_ContainsOriginalIndex(CurrentCandidate, Index)){
                        continue;
                    }

                    if (!_CanUseAsFiller(AFeatures[Index], CurrentCandidate)){
                        continue;
                    }

                    const std::vector<long long> GeometryKey = BuildFillerGeometryKey(AOriginalItems[Index],AFeatures[Index]);
                    if (!CheckedGeometryKeys.insert(GeometryKey).second){
                        continue;
                    }

                    TetClusterCandidate Candidate;
                    if (!_TryAddFiller(AOriginalItems,AFeatures,CurrentCandidate,BaseCollisionItems,Index,AOptions,Candidate)){
                        continue;
                    }

                    const bool AddsMoreArea = !HasBestNext || Candidate.RealArea > BestNextCandidate.RealArea + 1.0;
                    const bool SameAreaLessWaste = HasBestNext && std::abs(Candidate.RealArea - BestNextCandidate.RealArea) <= 1.0 && Candidate.ProxyWasteArea < BestNextCandidate.ProxyWasteArea - 1.0;
                    const bool SameGeometryBetterScore = HasBestNext && std::abs(Candidate.RealArea - BestNextCandidate.RealArea) <= 1.0 && std::abs(Candidate.ProxyWasteArea - BestNextCandidate.ProxyWasteArea) <= 1.0 && Candidate.Score > BestNextCandidate.Score;
                    if (AddsMoreArea || SameAreaLessWaste || SameGeometryBetterScore){
                        HasBestNext = true;
                        BestNextCandidate = std::move(Candidate);
                    }
                }

                if (!HasBestNext){
                    break;
                }

                HasFiller = true;
                CurrentCandidate = std::move(BestNextCandidate);
            }

            if (!HasFiller){
                return false;
            }

            AOutCandidate = std::move(CurrentCandidate);
            return true;
        }

        bool CetGapFillClusterBuilder::_TryAddFiller(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ABaseCandidate, const std::vector<TetGapFillCollisionItem>& ABaseCollisionItems, int AFillerIndex, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate)
        {
            AOutCandidate = TetClusterCandidate{};

            if (AFillerIndex < 0 || AFillerIndex >= static_cast<int>(AOriginalItems.size()) || AFillerIndex >= static_cast<int>(AFeatures.size())){
                return false;
            }

            const TetShapeFeature& FillerFeature = AFeatures[AFillerIndex];
            if (!_CanUseAsFiller(FillerFeature, ABaseCandidate)){
                return false;
            }

            CetClusterGeometryHelper Geometry;
            const double RequiredGap = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));
            const double DimensionTolerance = std::max(1.0, RequiredGap * 0.001);

            const std::vector<double> Rotations = CetRotationUtils::BuildAllowedRotations(AOptions.Rotations);
            if (ABaseCollisionItems.size() != ABaseCandidate.Transforms.size()){
                return false;
            }

            bool HasBest = false;
            TetClusterCandidate BestCandidate;
            std::set<std::vector<long long>> CheckedRotationKeys;

            for (double Rotation : Rotations){
                CetPath RotatedFiller = Geometry.TransformContour(Geometry.GetIdentityContour(AOriginalItems[AFillerIndex]),Rotation,0.0,0.0);
                std::vector<long long> RotationKey;
                AppendNormalizedContourKey(RotatedFiller,RotationKey);
                if (!FillerFeature.HasHoles && !CheckedRotationKeys.insert(RotationKey).second){
                    continue;
                }

                double FillerMinX = 0.0;
                double FillerMinY = 0.0;
                double FillerMaxX = 0.0;
                double FillerMaxY = 0.0;

                if (!Geometry.GetBounds(RotatedFiller, FillerMinX, FillerMinY, FillerMaxX, FillerMaxY)){
                    continue;
                }

                const double FillerWidth = FillerMaxX - FillerMinX;
                const double FillerHeight = FillerMaxY - FillerMinY;
                if (FillerWidth <= 0.0 || FillerHeight <= 0.0){
                    continue;
                }

                const double MaxX = ABaseCandidate.ClusterWidth - FillerWidth;
                const double MaxY = ABaseCandidate.ClusterHeight - FillerHeight;
                if (MaxX < 0.0 || MaxY < 0.0){
                    continue;
                }

                const std::vector<std::pair<double, double>> ProbePositions = BuildProbePositions(AOriginalItems,ABaseCandidate,FillerWidth,FillerHeight,RequiredGap);
                struct TetValidGapPlacement
                {
                    TetItemTransform Transform;
                    double Priority = 0.0;
                };
                std::vector<TetValidGapPlacement> ValidPlacements;
                ValidPlacements.reserve(ProbePositions.size());
                for (const auto& ProbePosition : ProbePositions){
                    const double ProbeXPosition = ProbePosition.first;
                    const double ProbeYPosition = ProbePosition.second;
                    TetItemTransform FillerTransform;
                    FillerTransform.OriginalId = AFillerIndex;
                    FillerTransform.RelativeRotation = Rotation;
                    FillerTransform.RelativeX = ProbeXPosition - FillerMinX;
                    FillerTransform.RelativeY = ProbeYPosition - FillerMinY;

                    if (!CanPlaceWithCollisionCache(AOriginalItems,Geometry,ABaseCandidate,ABaseCollisionItems,RotatedFiller,FillerTransform,ProbeXPosition,ProbeYPosition,ProbeXPosition + FillerWidth,ProbeYPosition + FillerHeight)){
                        continue;
                    }

                    const double FillerCenterX = ProbeXPosition + FillerWidth * 0.5;
                    const double FillerCenterY = ProbeYPosition + FillerHeight * 0.5;
                    const double DeltaX = FillerCenterX - ABaseCandidate.ClusterWidth * 0.5;
                    const double DeltaY = FillerCenterY - ABaseCandidate.ClusterHeight * 0.5;
                    const double Diagonal = std::max(1.0,std::hypot(ABaseCandidate.ClusterWidth,ABaseCandidate.ClusterHeight));
                    ValidPlacements.push_back({ FillerTransform,-std::hypot(DeltaX,DeltaY) / Diagonal });
                    const std::size_t ValidPlacementLimit = ABaseCandidate.Transforms.size() >= 24 ? 8 : CET_GAPFILL_VALID_PLACEMENT_COUNT;
                    if (ValidPlacements.size() >= ValidPlacementLimit){
                        break;
                    }
                }

                std::stable_sort(ValidPlacements.begin(),ValidPlacements.end(),[](const TetValidGapPlacement& AFirstPlacement, const TetValidGapPlacement& ASecondPlacement){
                        return AFirstPlacement.Priority > ASecondPlacement.Priority;
                    });
                const std::size_t ExactPlacementLimit = ABaseCandidate.Transforms.size() >= 24 ? 2 : ABaseCandidate.Transforms.size() >= 12 ? 3 : CET_GAPFILL_EXACT_PLACEMENT_COUNT;
                const std::size_t ExactPlacementCount = std::min(ExactPlacementLimit,ValidPlacements.size());
                for (std::size_t PlacementIndex = 0; PlacementIndex < ExactPlacementCount; ++PlacementIndex){
                    const TetItemTransform& FillerTransform = ValidPlacements[PlacementIndex].Transform;

                    TetClusterCandidate Candidate = ABaseCandidate;
                    Candidate.Valid = false;
                    Candidate.ClusterType = MakeGapFillClusterType(ABaseCandidate.ClusterType);
                    Candidate.OriginalIndices.push_back(AFillerIndex);
                    Candidate.Transforms.push_back(FillerTransform);
                    Candidate.Confidence = std::min(1.0, ABaseCandidate.Confidence + 0.05);

                    if (!Geometry.FinalizeCandidate(AOriginalItems, AOptions, Candidate)){
                        continue;
                    }

                    const double AreaTolerance = std::max(16.0,ABaseCandidate.ProxyArea * 1e-10);
                    if (Candidate.ClusterWidth > ABaseCandidate.ClusterWidth + DimensionTolerance ||Candidate.ClusterHeight > ABaseCandidate.ClusterHeight + DimensionTolerance ||Candidate.ProxyArea > ABaseCandidate.ProxyArea + AreaTolerance ||!Geometry.IsContourFullyContained(Candidate.ProxyContour,ABaseCandidate.ProxyContour,AreaTolerance)){
                        continue;
                    }

                    Candidate.Score = _CalculateScore(Candidate);

                    if (!HasBest || Candidate.Score > BestCandidate.Score){
                        HasBest = true;
                        BestCandidate = std::move(Candidate);
                    }
                }
            }

            if (!HasBest){
                return false;
            }

            AOutCandidate = std::move(BestCandidate);
            return true;
        }

        bool CetGapFillClusterBuilder::_IsSupportedBaseCandidate(const TetClusterCandidate& ACandidate)
        {
            return ACandidate.Valid && !ACandidate.BuilderName.empty() &&ACandidate.OriginalIndices.size() >= 2 && ACandidate.Transforms.size() == ACandidate.OriginalIndices.size() &&ACandidate.ProxyContour.size() >= 3 && ACandidate.ProxyArea > 0.0;
        }

        bool CetGapFillClusterBuilder::_CanUseAsFiller(const TetShapeFeature& AFeature, const TetClusterCandidate& ABaseCandidate)
        {
            if (AFeature.Width <= 0.0 || AFeature.Height <= 0.0 || AFeature.Area <= 0.0 ||ABaseCandidate.ClusterWidth <= 0.0 || ABaseCandidate.ClusterHeight <= 0.0 || ABaseCandidate.ProxyArea <= 0.0){
                return false;
            }

            const double AvailableArea = std::max(0.0,ABaseCandidate.ProxyArea - ABaseCandidate.OccupiedArea);
            const double AreaTolerance = std::max(16.0,ABaseCandidate.ProxyArea * 1e-10);
            const double FeatureLongSide = std::max(AFeature.Width,AFeature.Height);
            const double FeatureShortSide = std::min(AFeature.Width,AFeature.Height);
            const double ClusterLongSide = std::max(ABaseCandidate.ClusterWidth,ABaseCandidate.ClusterHeight);
            const double ClusterShortSide = std::min(ABaseCandidate.ClusterWidth,ABaseCandidate.ClusterHeight);
            return GetFeatureArea(AFeature) <= AvailableArea + AreaTolerance &&FeatureLongSide <= ClusterLongSide + 1.0 && FeatureShortSide <= ClusterShortSide + 1.0;
        }

        bool CetGapFillClusterBuilder::_ContainsOriginalIndex(const TetClusterCandidate& ACandidate, int AOriginalIndex)
        {
            return std::find(ACandidate.OriginalIndices.begin(), ACandidate.OriginalIndices.end(), AOriginalIndex) != ACandidate.OriginalIndices.end();
        }

        double CetGapFillClusterBuilder::_CalculateScore(const TetClusterCandidate& ACandidate)
        {
            if (!ACandidate.Valid || ACandidate.OriginalIndices.empty() || ACandidate.ProxyArea <= 0.0 || ACandidate.ClusterWidth <= 0.0 || ACandidate.ClusterHeight <= 0.0){
                return -std::numeric_limits<double>::infinity();
            }

            const double FillScore = ACandidate.FillRatio * 1500.0;
            const double ItemCountScore = static_cast<double>(ACandidate.OriginalIndices.size()) * 25.0;
            const double SavingScore = ACandidate.AreaSavingRatio * 500.0;
            const double WastePenalty = ACandidate.ProxyWasteRatio * 600.0;
            const double SizePenalty = (ACandidate.ClusterWidth + ACandidate.ClusterHeight) * 0.000001;
            return FillScore + ItemCountScore + SavingScore + ACandidate.Confidence - WastePenalty - SizePenalty;
        }

    }
}
