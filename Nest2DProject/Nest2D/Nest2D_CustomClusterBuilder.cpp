#include "pch.h"
#include "Nest2D_CustomClusterBuilder.h"
#include "Nest2D_ClusterGeometryHelper.h"
#include "Nest2D_RotationUtils.h"
#include "NestUtils.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace ET {
    namespace NEST2DMANAGERLIB {

        namespace {
            std::vector<long long> RotateSignature(const std::vector<long long>& ASignature, std::size_t AOffset, std::size_t AStride)
            {
                std::vector<long long> Result;
                Result.reserve(ASignature.size());
                const std::size_t ElementCount = ASignature.size() / AStride;
                for (std::size_t ElementOffset = 0; ElementOffset < ElementCount; ++ElementOffset){
                    const std::size_t SourceElement = (AOffset + ElementOffset) % ElementCount;
                    for (std::size_t Component = 0; Component < AStride; ++Component){
                        Result.push_back(ASignature[SourceElement * AStride + Component]);
                    }
                }
                return Result;
            }

            std::vector<long long> BuildContourSignature(CetPath AContour)
            {
                if (AContour.size() < 3){
                    return {};
                }

                ClipperLib::CleanPolygon(AContour, 1.0);
                if (AContour.size() < 3){
                    return {};
                }
                if (!ClipperLib::Orientation(AContour)){
                    std::reverse(AContour.begin(), AContour.end());
                }

                std::vector<long long> RawSignature;
                RawSignature.reserve(AContour.size() * 2);
                for (std::size_t Index = 0; Index < AContour.size(); ++Index){
                    const CetInpoint& Current = AContour[Index];
                    const CetInpoint& Next = AContour[(Index + 1) % AContour.size()];
                    const CetInpoint& AfterNext = AContour[(Index + 2) % AContour.size()];

                    const double EdgeX = static_cast<double>(Next.X - Current.X);
                    const double EdgeY = static_cast<double>(Next.Y - Current.Y);
                    const double NextEdgeX = static_cast<double>(AfterNext.X - Next.X);
                    const double NextEdgeY = static_cast<double>(AfterNext.Y - Next.Y);
                    const double Length = std::hypot(EdgeX, EdgeY);
                    const double TurnAngle = std::atan2(EdgeX * NextEdgeY - EdgeY * NextEdgeX,EdgeX * NextEdgeX + EdgeY * NextEdgeY);

                    RawSignature.push_back(static_cast<long long>(std::llround(Length)));
                    RawSignature.push_back(static_cast<long long>(std::llround(TurnAngle * 1000000.0)));
                }

                std::vector<long long> BestSignature;
                for (std::size_t Offset = 0; Offset < AContour.size(); ++Offset){
                    std::vector<long long> Candidate = RotateSignature(RawSignature, Offset, 2);
                    if (BestSignature.empty() || Candidate < BestSignature){
                        BestSignature = std::move(Candidate);
                    }
                }

                BestSignature.insert(BestSignature.begin(),static_cast<long long>(std::llround(std::abs(static_cast<double>(ClipperLib::Area(AContour))))));
                BestSignature.insert(BestSignature.begin(),static_cast<long long>(AContour.size()));
                return BestSignature;
            }

            bool BuildShapeKey(const CetNestItem& AItem, const TetShapeFeature& AFeature, TetCustomShapeKey& AOutKey)
            {
                CetNestItem IdentityItem = AItem;
                IdentityItem.translation(libnest2d::Point(0, 0));
                IdentityItem.rotation(libnest2d::Radians(0.0));
                IdentityItem.inflation(0);
                const auto Shape = IdentityItem.transformedShape();

                AOutKey = TetCustomShapeKey{};
                AOutKey.ShapeType = AFeature.ShapeType;
                AOutKey.HasHoles = !Shape.Holes.empty();
                AOutKey.OuterSignature = BuildContourSignature(Shape.Contour);
                if (AOutKey.OuterSignature.empty()){
                    return false;
                }

                AOutKey.HoleSignatures.reserve(Shape.Holes.size());
                for (const CetPath& Hole : Shape.Holes){
                    std::vector<long long> HoleSignature = BuildContourSignature(Hole);
                    if (HoleSignature.empty()){
                        return false;
                    }
                    AOutKey.HoleSignatures.push_back(std::move(HoleSignature));
                }
                std::sort(AOutKey.HoleSignatures.begin(), AOutKey.HoleSignatures.end());
                return true;
            }

            bool GetRotatedBounds(const CetClusterGeometryHelper& AGeometry, const CetNestItem& AItem, double ARotation, double& AOutMinX, double& AOutMinY, double& AOutMaxX, double& AOutMaxY)
            {
                const CetPath RotatedContour = AGeometry.TransformContour(AGeometry.GetIdentityContour(AItem),ARotation,0.0,0.0);
                return AGeometry.GetBounds(RotatedContour,AOutMinX,AOutMinY,AOutMaxX,AOutMaxY);
            }

            double GetLayoutGap(const TetNestOptions& AOptions)
            {
                const double RequiredGap = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));
                return RequiredGap + std::max(CET_CLUSTER_MIN_SAFETY_GAP, RequiredGap * 0.001);
            }

            bool BuildRotationPose(const CetClusterGeometryHelper& AGeometry, const CetNestItem& AItem, double ARotation, TetCustomRotationPose& AOutPose)
            {
                double MaxX = 0.0;
                double MaxY = 0.0;
                AOutPose = TetCustomRotationPose{};
                if (!GetRotatedBounds(AGeometry,AItem,ARotation,AOutPose.MinX,AOutPose.MinY,MaxX,MaxY)){
                    return false;
                }

                AOutPose.Rotation = ARotation;
                AOutPose.Width = MaxX - AOutPose.MinX;
                AOutPose.Height = MaxY - AOutPose.MinY;
                return AOutPose.Width > 0.0 && AOutPose.Height > 0.0;
            }

            std::vector<double> BuildDistinctBaseRotations(int ARotationCount)
            {
                const std::vector<double> AllowedRotations = CetRotationUtils::BuildAllowedRotations(ARotationCount);
                std::vector<double> Result;
                std::vector<double> CanonicalRotations;
                for (double Rotation : AllowedRotations){
                    double CanonicalRotation = std::fmod(CetRotationUtils::NormalizeAngle(Rotation),CET_CLUSTER_PI);
                    if (CanonicalRotation < 0.0){
                        CanonicalRotation += CET_CLUSTER_PI;
                    }

                    bool Duplicate = false;
                    for (double ExistingRotation : CanonicalRotations){
                        if (std::abs(CanonicalRotation - ExistingRotation) <= 1e-9){
                            Duplicate = true;
                            break;
                        }
                    }
                    if (!Duplicate){
                        Result.push_back(Rotation);
                        CanonicalRotations.push_back(CanonicalRotation);
                    }
                }
                if (Result.empty()){
                    Result.push_back(0.0);
                }
                return Result;
            }

            std::vector<std::size_t> BuildLayoutRowCounts(std::size_t AItemCount, double ACellWidth, double ACellHeight, double AColumnPitch, double ARowPitch, double ARowStaggerRatio, const TetNestOptions& AOptions)
            {
                std::vector<TetRowLayoutEstimate> Estimates;
                if (AItemCount == 0 || ACellWidth <= 0.0 || ACellHeight <= 0.0 || AColumnPitch <= 0.0 || ARowPitch <= 0.0){
                    return {};
                }

                const double BinWidth = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth));
                const double BinHeight = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinHeight));
                if (BinWidth <= 0.0 || BinHeight <= 0.0){
                    return {};
                }

                const bool QuarterTurnAllowed = CetRotationUtils::IsAllowedRotation(CET_CLUSTER_HALF_PI,AOptions.Rotations,1e-9);
                const double BoardAspectRatio = std::max(BinWidth,BinHeight) / std::max(1.0,std::min(BinWidth,BinHeight));
                Estimates.reserve(AItemCount);

                for (std::size_t RowCount = 1; RowCount <= AItemCount; ++RowCount){
                    const std::size_t ColumnCount = (AItemCount + RowCount - 1) / RowCount;
                    const double EstimatedWidth = ACellWidth + static_cast<double>(ColumnCount - 1) * AColumnPitch + (RowCount > 1 ? std::abs(ARowStaggerRatio) * AColumnPitch : 0.0);
                    const double EstimatedHeight = ACellHeight + static_cast<double>(RowCount - 1) * ARowPitch;
                    const bool FitsNormal = EstimatedWidth <= BinWidth && EstimatedHeight <= BinHeight;
                    const bool FitsRotated = QuarterTurnAllowed && EstimatedHeight <= BinWidth && EstimatedWidth <= BinHeight;
                    if (!FitsNormal && !FitsRotated){
                        continue;
                    }

                    const std::size_t CellCount = RowCount * ColumnCount;
                    const double EmptyCellRatio = static_cast<double>(CellCount - AItemCount) / static_cast<double>(CellCount);
                    const double LayoutAspectRatio = std::max(EstimatedWidth,EstimatedHeight) / std::max(1.0,std::min(EstimatedWidth,EstimatedHeight));
                    const double AspectPenalty = std::abs(std::log(std::max(1e-9,LayoutAspectRatio / BoardAspectRatio)));
                    Estimates.push_back({ RowCount, EmptyCellRatio * 4.0 + AspectPenalty });
                }

                std::stable_sort(Estimates.begin(),Estimates.end(),[](const TetRowLayoutEstimate& AFirst, const TetRowLayoutEstimate& ASecond){
                        if (std::abs(AFirst.Score - ASecond.Score) > 1e-9){
                            return AFirst.Score < ASecond.Score;
                        }
                        return AFirst.RowCount < ASecond.RowCount;
                    });

                const std::size_t ExactRowLimit = AItemCount <= 8 ? Estimates.size() : std::min<std::size_t>(6,Estimates.size());
                std::vector<std::size_t> Result;
                Result.reserve(ExactRowLimit + 2);
                for (std::size_t EstimateIndex = 0; EstimateIndex < ExactRowLimit; ++EstimateIndex){
                    Result.push_back(Estimates[EstimateIndex].RowCount);
                }
                const auto AppendExtremeRowCount = [&](std::size_t ARowCount) {
                    const auto EstimateIt = std::find_if(Estimates.begin(),Estimates.end(),[ARowCount](const TetRowLayoutEstimate& AEstimate){
                            return AEstimate.RowCount == ARowCount;
                        });
                    if (EstimateIt != Estimates.end() && std::find(Result.begin(),Result.end(),ARowCount) == Result.end()){
                        Result.push_back(ARowCount);
                    }
                    };
                AppendExtremeRowCount(1);
                AppendExtremeRowCount(AItemCount);
                std::sort(Result.begin(),Result.end());
                return Result;
            }

            std::vector<std::size_t> BuildCandidateChildCounts(std::size_t AItemCount)
            {
                if (AItemCount > CET_CUSTOM_SEARCH_MAX_CHILDREN){
                    // Search a small reusable template for large repeated orders.
                    // _BuildSameShapeClusterCandidates remaps the chosen geometry
                    // over the rest of the group without repeating this search.
                    return { 4, 2 };
                }
                const std::size_t MaxChildCount = std::min({ AItemCount, CET_CUSTOM_MAX_CLUSTER_CHILDREN, CET_CUSTOM_SEARCH_MAX_CHILDREN });
                if (MaxChildCount < 2){
                    return {};
                }

                std::vector<std::size_t> Result;
                const auto AddCount = [&](std::size_t ACount) {
                    if (ACount >= 2 && ACount <= MaxChildCount && std::find(Result.begin(), Result.end(), ACount) == Result.end()){
                        Result.push_back(ACount);
                    }
                };

                AddCount(MaxChildCount);
                for (std::size_t Count = 2; Count <= MaxChildCount; ++Count){
                    AddCount(Count);
                }
                AddCount(2);
                return Result;
            }
        }

        CetCustomClusterBuilder::CetCustomClusterBuilder() : CetCoreObject() {}
        CetCustomClusterBuilder::~CetCustomClusterBuilder() {}

        void CetCustomClusterBuilder::BuildCandidates(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, std::vector<TetClusterCandidate>& AOutCandidates)
        {
            if (AOriginalItems.size() != AFeatures.size() || AIndices.size() < 2){
                return;
            }

            std::map<TetCustomShapeKey, std::vector<int>> IndicesByShape;
            for (int Index : AIndices){
                if (Index < 0 || Index >= static_cast<int>(AFeatures.size()) || !_IsSupportedCustomShape(AFeatures[Index])){
                    continue;
                }

                TetCustomShapeKey ShapeKey;
                if (BuildShapeKey(AOriginalItems[Index], AFeatures[Index], ShapeKey)){
                    IndicesByShape[ShapeKey].push_back(Index);
                }
            }

            const std::size_t OldCandidateCount = AOutCandidates.size();
            for (auto& ShapeEntry : IndicesByShape){
                std::vector<int>& ShapeIndices = ShapeEntry.second;
                std::sort(ShapeIndices.begin(), ShapeIndices.end());
                ShapeIndices.erase(std::unique(ShapeIndices.begin(), ShapeIndices.end()), ShapeIndices.end());
                if (ShapeIndices.size() >= 2){
                    _BuildSameShapeClusterCandidates(AOriginalItems,AFeatures,ShapeIndices,AOptions,AOutCandidates);
                }
            }

            if (IndicesByShape.size() > 1 && AIndices.size() <= CET_CUSTOM_SEARCH_MAX_CHILDREN){
                TetClusterCandidate MixedCandidate;
                if (_BuildMixedShapeCandidate(AOriginalItems,AFeatures,AIndices,AOptions,MixedCandidate)){
                    std::cout << "[CUSTOM][MIXED CANDIDATE] ChildCount=" << MixedCandidate.OriginalIndices.size() << ", Type=" << MixedCandidate.ClusterType << ", Score=" << MixedCandidate.Score << std::endl;
                    AOutCandidates.push_back(std::move(MixedCandidate));
                }
            }

            std::cout << "[CUSTOM][BUILD CANDIDATES] IndexCount=" << AIndices.size() << ", ShapeGroupCount=" << IndicesByShape.size() << ", NewCandidateCount=" << AOutCandidates.size() - OldCandidateCount << std::endl;
        }

        bool CetCustomClusterBuilder::_BuildMixedShapeCandidateImpl(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate)
        {
            AOutCandidate = TetClusterCandidate{};
            if (AIndices.size() < 2 || AFeatures.size() != AOriginalItems.size()){
                return false;
            }

            CetClusterGeometryHelper Geometry;
            const double BoardWidth = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth));
            const double BoardHeight = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinHeight));
            const double LayoutGap = GetLayoutGap(AOptions);
            if (BoardWidth <= 0.0 || BoardHeight <= 0.0 || LayoutGap <= 0.0){
                return false;
            }

            const std::vector<double> AllowedRotations = CetRotationUtils::BuildAllowedRotations(AOptions.Rotations);
            if (AllowedRotations.empty()){
                return false;
            }

            std::map<int, int> GroupByIndex;
            std::map<TetCustomShapeKey, int> GroupIds;
            int NextGroupId = 0;
            for (int OriginalIndex : AIndices){
                TetCustomShapeKey ShapeKey;
                if (OriginalIndex >= 0 && OriginalIndex < static_cast<int>(AOriginalItems.size()) && BuildShapeKey(AOriginalItems[OriginalIndex], AFeatures[OriginalIndex], ShapeKey)){
                    auto GroupIt = GroupIds.find(ShapeKey);
                    if (GroupIt == GroupIds.end()){
                        GroupIt = GroupIds.emplace(std::move(ShapeKey), NextGroupId++).first;
                    }
                    GroupByIndex[OriginalIndex] = GroupIt->second;
                }
                else {
                    GroupByIndex[OriginalIndex] = NextGroupId++;
                }
            }

            std::vector<std::vector<int>> ShapeGroups;
            ShapeGroups.resize(static_cast<std::size_t>(NextGroupId));
            for (const auto& GroupEntry : GroupByIndex){
                if (GroupEntry.second >= 0 && GroupEntry.second < static_cast<int>(ShapeGroups.size())){
                    ShapeGroups[GroupEntry.second].push_back(GroupEntry.first);
                }
            }
            for (std::vector<int>& ShapeGroup : ShapeGroups){
                std::sort(ShapeGroup.begin(), ShapeGroup.end());
            }

            std::vector<int> Order = AIndices;
            TetClusterCandidate BestCandidate;
            bool HasBest = false;
            const double TargetWidthRatios[] = { 0.50, 0.80, 1.00 };

            const auto BuildTrialCounts = [](std::size_t APlacedCount) {
                const std::size_t MaxPlacedCount = std::min({ APlacedCount,CET_CUSTOM_MAX_CLUSTER_CHILDREN,CET_CUSTOM_SEARCH_MAX_CHILDREN });
                const std::size_t PreferredCounts[] = { 2, 4, 6, 8, 12, 16, 24, 32, 48, 64 };
                std::vector<std::size_t> Result;
                for (std::size_t Count : PreferredCounts){
                    if (Count <= MaxPlacedCount && std::find(Result.begin(), Result.end(), Count) == Result.end()){
                        Result.push_back(Count);
                    }
                }
                if (MaxPlacedCount >= 2 && std::find(Result.begin(), Result.end(), MaxPlacedCount) == Result.end()){
                    Result.push_back(MaxPlacedCount);
                }
                return Result;
            };

            const auto HasMultipleGroups = [&](const std::vector<int>& APlacedIndices, std::size_t ACount) {
                if (ACount < 2 || APlacedIndices.empty()){
                    return false;
                }
                const auto FirstGroupIt = GroupByIndex.find(APlacedIndices.front());
                if (FirstGroupIt == GroupByIndex.end()){
                    return false;
                }
                const int FirstGroup = FirstGroupIt->second;
                for (std::size_t Index = 1; Index < ACount && Index < APlacedIndices.size(); ++Index){
                    const auto GroupIt = GroupByIndex.find(APlacedIndices[Index]);
                    if (GroupIt != GroupByIndex.end() && GroupIt->second != FirstGroup){
                        return true;
                    }
                }
                return false;
            };

            const int OrderModes[] = { 0, 3 };
            for (int OrderMode : OrderModes){
                if (OrderMode == 3){
                    Order.clear();
                    std::size_t GroupOffset = 0;
                    bool AddedAny = true;
                    while (AddedAny){
                        AddedAny = false;
                        for (const std::vector<int>& ShapeGroup : ShapeGroups){
                            if (GroupOffset < ShapeGroup.size()){
                                Order.push_back(ShapeGroup[GroupOffset]);
                                AddedAny = true;
                            }
                        }
                        ++GroupOffset;
                    }
                }
                else {
                    Order = AIndices;
                    std::stable_sort(Order.begin(),Order.end(),[&](int AFirstIndex, int ASecondIndex){
                            const TetShapeFeature& FirstFeature = AFeatures[AFirstIndex];
                            const TetShapeFeature& SecondFeature = AFeatures[ASecondIndex];
                            if (OrderMode == 0 && std::abs(FirstFeature.Area - SecondFeature.Area) > 1.0){
                                return FirstFeature.Area > SecondFeature.Area;
                            }
                            if (OrderMode == 1 && std::abs(std::max(FirstFeature.Width,FirstFeature.Height) - std::max(SecondFeature.Width,SecondFeature.Height)) > 1.0){
                                return std::max(FirstFeature.Width,FirstFeature.Height) > std::max(SecondFeature.Width,SecondFeature.Height);
                            }
                            if (OrderMode == 2 && std::abs(FirstFeature.Width - SecondFeature.Width) > 1.0){
                                return FirstFeature.Width > SecondFeature.Width;
                            }
                            return AFirstIndex < ASecondIndex;
                        });
                }

                for (double TargetWidthRatio : TargetWidthRatios){
                    const double TargetWidth = BoardWidth * TargetWidthRatio;
                    if (TargetWidth <= 0.0){
                        continue;
                    }

                    std::vector<int> PlacedIndices;
                    std::vector<TetItemTransform> Transforms;
                    PlacedIndices.reserve(Order.size());
                    Transforms.reserve(Order.size());
                    double CurrentY = 0.0;
                    double CurrentX = 0.0;
                    double CurrentRowHeight = 0.0;

                    for (int OriginalIndex : Order){
                        if (OriginalIndex < 0 || OriginalIndex >= static_cast<int>(AOriginalItems.size())){
                            continue;
                        }

                        std::vector<TetCustomRotationPose> Poses;
                        Poses.reserve(AllowedRotations.size());
                        for (double Rotation : AllowedRotations){
                            TetCustomRotationPose Pose;
                            if (BuildRotationPose(Geometry,AOriginalItems[OriginalIndex],Rotation,Pose)){
                                Poses.push_back(Pose);
                            }
                        }
                        if (Poses.empty()){
                            continue;
                        }

                        const auto SelectPose = [&](bool AFitsCurrentRow, TetCustomRotationPose& AOutPose) {
                            bool HasPose = false;
                            for (const TetCustomRotationPose& Pose : Poses){
                                if (AFitsCurrentRow && CurrentX > 0.0 && CurrentX + Pose.Width > TargetWidth + 1.0){
                                    continue;
                                }
                                if (!HasPose || Pose.Height < AOutPose.Height - 1.0 ||(std::abs(Pose.Height - AOutPose.Height) <= 1.0 && Pose.Width < AOutPose.Width)){
                                    AOutPose = Pose;
                                    HasPose = true;
                                }
                            }
                            return HasPose;
                            };

                        TetCustomRotationPose Pose;
                        const bool FitsCurrentRow = SelectPose(true,Pose);
                        if (!FitsCurrentRow && CurrentX > 0.0){
                            CurrentY += CurrentRowHeight + LayoutGap;
                            CurrentX = 0.0;
                            CurrentRowHeight = 0.0;
                            if (!SelectPose(false,Pose)){
                                continue;
                            }
                        }
                        else if (!FitsCurrentRow){
                            continue;
                        }

                        if (CurrentY + Pose.Height > BoardHeight + 1.0){
                            break;
                        }

                        TetItemTransform Transform;
                        Transform.OriginalId = OriginalIndex;
                        Transform.RelativeRotation = Pose.Rotation;
                        Transform.RelativeX = CurrentX - Pose.MinX;
                        Transform.RelativeY = CurrentY - Pose.MinY;
                        PlacedIndices.push_back(OriginalIndex);
                        Transforms.push_back(Transform);
                        CurrentX += Pose.Width + LayoutGap;
                        CurrentRowHeight = std::max(CurrentRowHeight,Pose.Height);
                    }

                    const std::vector<std::size_t> TrialCounts = BuildTrialCounts(Transforms.size());
                    for (std::size_t TrialCount : TrialCounts){
                        if (!HasMultipleGroups(PlacedIndices,TrialCount)){
                            continue;
                        }

                        TetClusterCandidate Candidate;
                        Candidate.BuilderName = "CustomBuilder";
                        Candidate.ClusterType = "CustomMixedShelf_" + std::to_string(TrialCount) + "_W" + std::to_string(static_cast<int>(std::llround(TargetWidthRatio * 100.0))) + (OrderMode == 3 ? "_Interleave" : "");
                        Candidate.OriginalIndices.assign(PlacedIndices.begin(),PlacedIndices.begin() + static_cast<std::vector<int>::difference_type>(TrialCount));
                        Candidate.Transforms.assign(Transforms.begin(),Transforms.begin() + static_cast<std::vector<TetItemTransform>::difference_type>(TrialCount));
                        Candidate.Confidence = OrderMode == 3 ? 0.95 : 0.92;
                        if (!Geometry.HasValidTransformSpacing(AOriginalItems,AOptions,Candidate.Transforms) || !Geometry.FinalizeCandidate(AOriginalItems,AOptions,Candidate) || Candidate.AreaSavingRatio < -CET_CUSTOM_MAX_AREA_LOSS_RATIO){
                            continue;
                        }

                        Candidate.Score = _CalculateScore(Candidate,AOptions);
                        const double CandidateBoxArea = Candidate.ClusterWidth * Candidate.ClusterHeight;
                        const double BestBoxArea = HasBest ? BestCandidate.ClusterWidth * BestCandidate.ClusterHeight : std::numeric_limits<double>::max();
                        if (!HasBest || Candidate.Score > BestCandidate.Score || (std::abs(Candidate.Score - BestCandidate.Score) <= 1e-9 && CandidateBoxArea < BestBoxArea)){
                            HasBest = true;
                            BestCandidate = std::move(Candidate);
                        }
                    }
                }
            }

            if (!HasBest){
                return false;
            }

            AOutCandidate = std::move(BestCandidate);
            return true;
        }

        bool CetCustomClusterBuilder::_BuildMixedShapeCandidate(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate)
        {
            AOutCandidate = TetClusterCandidate{};
            if (AIndices.size() < 2 || AFeatures.size() != AOriginalItems.size()) return false;
            return _BuildMixedShapeCandidateImpl(AOriginalItems, AFeatures, AIndices, AOptions, AOutCandidate);
        }

        void CetCustomClusterBuilder::_BuildSameShapeClusterCandidates(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, std::vector<TetClusterCandidate>& AOutCandidates)
        {
            (void)AFeatures;
            if (AIndices.size() < 2){
                return;
            }

            std::size_t PreferredChildCount = 0;
            TetClusterCandidate FirstCandidate;
            _FindLargestBoardFitLayout(AOriginalItems,AIndices,AOptions,PreferredChildCount,FirstCandidate);

            if (PreferredChildCount < 2){
                std::cout << "[CUSTOM][REJECT] No valid complete layout. GroupCount=" << AIndices.size() << std::endl;
                return;
            }

            std::size_t GroupOffset = 0;
            while (GroupOffset + 1 < AIndices.size()){
                const std::size_t RemainingCount = AIndices.size() - GroupOffset;
                std::size_t TrialChildCount = std::min(PreferredChildCount,RemainingCount);
                bool HasCandidate = false;
                TetClusterCandidate BestCandidate;

                std::vector<int> RemainingIndices(AIndices.begin() + static_cast<std::vector<int>::difference_type>(GroupOffset),AIndices.end());
                if (GroupOffset == 0){
                    BestCandidate = FirstCandidate;
                    HasCandidate = true;
                }
                else if (RemainingCount >= PreferredChildCount){
                    std::vector<int> TrialIndices(RemainingIndices.begin(),RemainingIndices.begin() + static_cast<std::vector<int>::difference_type>(PreferredChildCount));
                    HasCandidate = _RemapCandidateIndices(FirstCandidate,TrialIndices,BestCandidate);
                }
                else {
                    HasCandidate = _FindLargestBoardFitLayout(AOriginalItems,RemainingIndices,AOptions,TrialChildCount,BestCandidate);
                }

                if (!HasCandidate){
                    break;
                }

                std::cout << "[CUSTOM][CANDIDATE] ChildCount=" << TrialChildCount << ", Type=" << BestCandidate.ClusterType << ", Score=" << BestCandidate.Score << std::endl;
                AOutCandidates.push_back(std::move(BestCandidate));
                GroupOffset += TrialChildCount;
            }
        }

        bool CetCustomClusterBuilder::_FindLargestBoardFitLayout(const CetTNestItemVector& AOriginalItems, const std::vector<int>& AIndices, const TetNestOptions& AOptions, std::size_t& AOutChildCount, TetClusterCandidate& AOutCandidate)
        {
            AOutChildCount = 0;
            AOutCandidate = TetClusterCandidate{};
            if (AIndices.size() < 2){
                return false;
            }

            bool HasBest = false;
            const std::vector<std::size_t> CandidateCounts = BuildCandidateChildCounts(AIndices.size());
            for (std::size_t TrialChildCount : CandidateCounts){
                std::vector<int> TrialIndices(
                    AIndices.begin(),
                    AIndices.begin() + static_cast<std::vector<int>::difference_type>(TrialChildCount));
                TetClusterCandidate Candidate;
                if (_BuildBestLayoutCandidate(AOriginalItems,TrialIndices,AOptions,Candidate)){
                    const bool BetterScore = !HasBest || Candidate.Score > AOutCandidate.Score + 1e-9;
                    const bool SameScoreMoreItems = HasBest &&
                        std::abs(Candidate.Score - AOutCandidate.Score) <= 1e-9 &&
                        TrialChildCount > AOutChildCount;
                    if (BetterScore || SameScoreMoreItems){
                        HasBest = true;
                        AOutChildCount = TrialChildCount;
                        AOutCandidate = std::move(Candidate);
                    }
                }
            }

            return HasBest && AOutChildCount >= 2;
        }

        bool CetCustomClusterBuilder::_PrepareEdgePairSearch(const TetEdgePairRequest& ARequest, TetEdgePairSearchContext& AOutContext)
        {
            if (ARequest.Indices.size() != 2){
                return false;
            }

            AOutContext.FirstIndex = ARequest.Indices[0];
            AOutContext.SecondIndex = ARequest.Indices[1];
            if (AOutContext.FirstIndex < 0 || AOutContext.SecondIndex < 0 || AOutContext.FirstIndex >= static_cast<int>(ARequest.OriginalItems.size()) || AOutContext.SecondIndex >= static_cast<int>(ARequest.OriginalItems.size())){
                return false;
            }

            CetClusterGeometryHelper Geometry;
            AOutContext.FirstContour = Geometry.GetIdentityContour(ARequest.OriginalItems[AOutContext.FirstIndex]);
            AOutContext.SecondContour = Geometry.GetIdentityContour(ARequest.OriginalItems[AOutContext.SecondIndex]);
            if (AOutContext.FirstContour.size() < 3 || AOutContext.SecondContour.size() < 3){
                return false;
            }

            AOutContext.AllowedRotations = CetRotationUtils::BuildAllowedRotations(ARequest.Options.Rotations);
            AOutContext.RequiredGap = GetLayoutGap(ARequest.Options);
            return true;
        }

        bool CetCustomClusterBuilder::_TryBuildEdgePairCandidate(const TetEdgePairRequest& ARequest, const TetEdgePairSearchContext& AContext, const TetEdgePairPlacement& APlacement, TetClusterCandidate& AOutCandidate)
        {
            AOutCandidate = TetClusterCandidate{};
            AOutCandidate.BuilderName = "CustomBuilder";
            AOutCandidate.ClusterType = "CustomEdgePair";
            AOutCandidate.OriginalIndices = ARequest.Indices;
            AOutCandidate.Confidence = 0.96;

            TetItemTransform FirstTransform;
            FirstTransform.OriginalId = AContext.FirstIndex;
            AOutCandidate.Transforms.push_back(FirstTransform);

            TetItemTransform SecondTransform;
            SecondTransform.OriginalId = AContext.SecondIndex;
            SecondTransform.RelativeRotation = APlacement.SecondRotation;
            SecondTransform.RelativeX = APlacement.BaseOffsetX + APlacement.NormalX * AContext.RequiredGap * APlacement.NormalDirection + APlacement.TangentX * APlacement.TangentShift;
            SecondTransform.RelativeY = APlacement.BaseOffsetY + APlacement.NormalY * AContext.RequiredGap * APlacement.NormalDirection + APlacement.TangentY * APlacement.TangentShift;
            AOutCandidate.Transforms.push_back(SecondTransform);

            CetClusterGeometryHelper Geometry;
            if (!Geometry.HasValidTransformSpacing(ARequest.OriginalItems,ARequest.Options,AOutCandidate.Transforms) || !Geometry.FinalizeCandidate(ARequest.OriginalItems,ARequest.Options,AOutCandidate) || AOutCandidate.AreaSavingRatio < -CET_CUSTOM_MAX_AREA_LOSS_RATIO){
                return false;
            }

            AOutCandidate.Score = _CalculateScore(AOutCandidate,ARequest.Options) + APlacement.LengthRatio * 40.0;
            return true;
        }

        bool CetCustomClusterBuilder::_SearchEdgePairCandidates(const TetEdgePairRequest& ARequest, const TetEdgePairSearchContext& AContext, TetEdgePairSearchResult& AOutResult)
        {
            constexpr double MIN_EDGE_LENGTH_RATIO = 0.80;
            constexpr double OPPOSITE_EDGE_COSINE = -0.9986295347545738; // cos(177 degrees)

            for (double SecondRotation : AContext.AllowedRotations){
                const double CosRotation = std::cos(SecondRotation);
                const double SinRotation = std::sin(SecondRotation);
                const auto RotatePoint = [&](const CetInpoint& APoint) {
                    return std::pair<double, double>{
                        static_cast<double>(APoint.X) * CosRotation - static_cast<double>(APoint.Y) * SinRotation,
                        static_cast<double>(APoint.X) * SinRotation + static_cast<double>(APoint.Y) * CosRotation
                    };
                };

                for (std::size_t FirstEdgeIndex = 0; FirstEdgeIndex < AContext.FirstContour.size(); ++FirstEdgeIndex){
                    const CetInpoint& FirstStart = AContext.FirstContour[FirstEdgeIndex];
                    const CetInpoint& FirstEnd = AContext.FirstContour[(FirstEdgeIndex + 1) % AContext.FirstContour.size()];
                    const double FirstDX = static_cast<double>(FirstEnd.X - FirstStart.X);
                    const double FirstDY = static_cast<double>(FirstEnd.Y - FirstStart.Y);
                    const double FirstLength = std::hypot(FirstDX,FirstDY);
                    if (FirstLength <= 0.0){
                        continue;
                    }

                    const double NormalX = -FirstDY / FirstLength;
                    const double NormalY = FirstDX / FirstLength;
                    for (std::size_t SecondEdgeIndex = 0; SecondEdgeIndex < AContext.SecondContour.size(); ++SecondEdgeIndex){
                        const auto RotatedSecondStart = RotatePoint(AContext.SecondContour[SecondEdgeIndex]);
                        const auto RotatedSecondEnd = RotatePoint(AContext.SecondContour[(SecondEdgeIndex + 1) % AContext.SecondContour.size()]);
                        const double SecondDX = RotatedSecondEnd.first - RotatedSecondStart.first;
                        const double SecondDY = RotatedSecondEnd.second - RotatedSecondStart.second;
                        const double SecondLength = std::hypot(SecondDX,SecondDY);
                        if (SecondLength <= 0.0){
                            continue;
                        }

                        const double LengthRatio = std::min(FirstLength,SecondLength) / std::max(FirstLength,SecondLength);
                        const double DirectionCosine = (FirstDX * SecondDX + FirstDY * SecondDY) / (FirstLength * SecondLength);
                        if (LengthRatio < MIN_EDGE_LENGTH_RATIO || DirectionCosine > OPPOSITE_EDGE_COSINE){
                            continue;
                        }

                        const double FirstMidX = (static_cast<double>(FirstStart.X) + static_cast<double>(FirstEnd.X)) * 0.5;
                        const double FirstMidY = (static_cast<double>(FirstStart.Y) + static_cast<double>(FirstEnd.Y)) * 0.5;
                        const double SecondMidX = (RotatedSecondStart.first + RotatedSecondEnd.first) * 0.5;
                        const double SecondMidY = (RotatedSecondStart.second + RotatedSecondEnd.second) * 0.5;
                        std::vector<std::pair<double, double>> BaseOffsets;
                        const auto AppendBaseOffset = [&](double AX, double AY) {
                            const auto Existing = std::find_if(BaseOffsets.begin(),BaseOffsets.end(),[&](const auto& AOffset) {
                                    return std::abs(AOffset.first - AX) <= 1.0 && std::abs(AOffset.second - AY) <= 1.0;
                                });
                            if (Existing == BaseOffsets.end()){
                                BaseOffsets.emplace_back(AX,AY);
                            }
                        };
                        AppendBaseOffset(FirstMidX - SecondMidX,FirstMidY - SecondMidY);
                        AppendBaseOffset(static_cast<double>(FirstStart.X) - RotatedSecondEnd.first,static_cast<double>(FirstStart.Y) - RotatedSecondEnd.second);
                        AppendBaseOffset(static_cast<double>(FirstEnd.X) - RotatedSecondStart.first,static_cast<double>(FirstEnd.Y) - RotatedSecondStart.second);
                        const double TangentX = FirstDX / FirstLength;
                        const double TangentY = FirstDY / FirstLength;
                        const double MatchedLength = std::min(FirstLength,SecondLength);
                        constexpr std::array<double, 5> TANGENT_SHIFT_RATIOS = {{ -0.25, -0.125, 0.0, 0.125, 0.25 }};

                        for (double NormalDirection : { -1.0, 1.0 }){
                            for (const auto& BaseOffset : BaseOffsets){
                                for (double TangentShiftRatio : TANGENT_SHIFT_RATIOS){
                                    TetEdgePairPlacement Placement;
                                    Placement.SecondRotation = SecondRotation;
                                    Placement.NormalX = NormalX;
                                    Placement.NormalY = NormalY;
                                    Placement.TangentX = TangentX;
                                    Placement.TangentY = TangentY;
                                    Placement.BaseOffsetX = BaseOffset.first;
                                    Placement.BaseOffsetY = BaseOffset.second;
                                    Placement.NormalDirection = NormalDirection;
                                    Placement.TangentShift = MatchedLength * TangentShiftRatio;
                                    Placement.LengthRatio = LengthRatio;
                                    TetClusterCandidate Candidate;
                                    if (!_TryBuildEdgePairCandidate(ARequest,AContext,Placement,Candidate)){
                                        continue;
                                    }

                                    const double CandidateBoxArea = Candidate.ClusterWidth * Candidate.ClusterHeight;
                                    const double BestBoxArea = AOutResult.HasCandidate ? AOutResult.BestCandidate.ClusterWidth * AOutResult.BestCandidate.ClusterHeight : std::numeric_limits<double>::max();
                                    if (!AOutResult.HasCandidate || CandidateBoxArea < BestBoxArea - 1.0 || (std::abs(CandidateBoxArea - BestBoxArea) <= 1.0 && Candidate.Score > AOutResult.BestCandidate.Score)){
                                        AOutResult.HasCandidate = true;
                                        AOutResult.BestCandidate = std::move(Candidate);
                                    }
                                }
                            }
                        }
                    }
                }
            }

            return AOutResult.HasCandidate;
        }

        bool CetCustomClusterBuilder::_BuildBestEdgePairCandidate(const TetEdgePairRequest& ARequest)
        {
            ARequest.OutCandidate = TetClusterCandidate{};

            TetEdgePairSearchContext SearchContext;
            if (!_PrepareEdgePairSearch(ARequest,SearchContext)){
                return false;
            }
            TetEdgePairSearchResult SearchResult;
            if (!_SearchEdgePairCandidates(ARequest,SearchContext,SearchResult)){
                return false;
            }
            ARequest.OutCandidate = std::move(SearchResult.BestCandidate);
            return true;
        }

        bool CetCustomClusterBuilder::_BuildCustomLayoutCandidate(const CustomLayoutCandidateRequest& ARequest, TetClusterCandidate& AOutCandidate)
        {
            AOutCandidate = TetClusterCandidate{};
            AOutCandidate.BuilderName = "CustomBuilder";
            AOutCandidate.ClusterType = "CustomLayout_" + std::to_string(ARequest.Indices.size()) + "_R" + std::to_string(ARequest.RowCount) + "_" + ARequest.Pattern.Name;
            AOutCandidate.OriginalIndices = ARequest.Indices;
            AOutCandidate.Confidence = ARequest.Pattern.AlternateHalfTurn ? 0.86 : 0.80;
            AOutCandidate.Transforms.reserve(ARequest.Indices.size());
            std::size_t ItemOffset = 0;
            for (std::size_t Row = 0; Row < ARequest.RowCount && ItemOffset < ARequest.Indices.size(); ++Row){
                const std::size_t RowItemCount = std::min(ARequest.ColumnCount, ARequest.Indices.size() - ItemOffset);
                double RowOffsetX = static_cast<double>(ARequest.ColumnCount - RowItemCount) * ARequest.ColumnPitch * 0.5;
                if ((Row % 2) != 0) RowOffsetX += ARequest.Pattern.RowStaggerRatio * ARequest.ColumnPitch;
                for (std::size_t Column = 0; Column < RowItemCount; ++Column){
                    const bool UseHalfTurn = ARequest.Pattern.AlternateHalfTurn && ((Row + Column) % 2) != 0;
                    const TetCustomRotationPose& Pose = UseHalfTurn ? ARequest.HalfTurnPose : ARequest.BasePose;
                    TetItemTransform Transform;
                    Transform.OriginalId = ARequest.Indices[ItemOffset];
                    Transform.RelativeRotation = Pose.Rotation;
                    Transform.RelativeX = RowOffsetX + static_cast<double>(Column) * ARequest.ColumnPitch + (ARequest.CellWidth - Pose.Width) * 0.5 - Pose.MinX;
                    Transform.RelativeY = static_cast<double>(Row) * ARequest.RowPitch + (ARequest.CellHeight - Pose.Height) * 0.5 - Pose.MinY;
                    AOutCandidate.Transforms.push_back(Transform);
                    ++ItemOffset;
                }
            }
            CetClusterGeometryHelper Geometry;
            if (!Geometry.HasValidTransformSpacing(ARequest.Items,ARequest.Options,AOutCandidate.Transforms) || !Geometry.FinalizeCandidate(ARequest.Items,ARequest.Options,AOutCandidate) || AOutCandidate.AreaSavingRatio < -CET_CUSTOM_MAX_AREA_LOSS_RATIO) return false;
            AOutCandidate.Score = _CalculateScore(AOutCandidate,ARequest.Options);
            return true;
        }

        bool CetCustomClusterBuilder::_BuildBestLayoutCandidate(const CetTNestItemVector& AOriginalItems, const std::vector<int>& AIndices, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate)
        {
            AOutCandidate = TetClusterCandidate{};
            if (AIndices.size() < 2 || AIndices.size() > CET_CUSTOM_MAX_CLUSTER_CHILDREN){
                return false;
            }
            for (int Index : AIndices){
                if (Index < 0 || Index >= static_cast<int>(AOriginalItems.size())){
                    return false;
                }
            }

            CetClusterGeometryHelper Geometry;
            const double LayoutGap = GetLayoutGap(AOptions);
            const std::vector<double> BaseRotations = BuildDistinctBaseRotations(AOptions.Rotations);
            const TetCustomLayoutPattern Patterns[] = {
                { "Uniform", 1.0, 1.0, 0.0, false },
                { "Alternating", 1.0, 1.0, 0.0, true },
                { "AlternatingCompactX", 0.82, 1.0, 0.0, true },
                { "AlternatingCompactY", 1.0, 0.82, 0.5, true },
                { "AlternatingCompactXY", 0.82, 0.82, 0.5, true },
                { "AlternatingTightX", 0.70, 1.0, 0.0, true },
                { "AlternatingTightY", 1.0, 0.70, 0.5, true }
            };
            bool HasBest = false;
            TetClusterCandidate BestCandidate;
            if (AIndices.size() == 2){
                TetClusterCandidate EdgePairCandidate;
                TetEdgePairRequest EdgePairRequestData{ AOriginalItems, AIndices, AOptions, EdgePairCandidate };
                if (_BuildBestEdgePairCandidate(EdgePairRequestData)){
                    HasBest = true;
                    BestCandidate = std::move(EdgePairCandidate);
                }
            }
            for (double BaseRotation : BaseRotations){
                TetCustomRotationPose BasePose;
                if (!BuildRotationPose(Geometry,AOriginalItems[AIndices.front()],BaseRotation,BasePose)){
                    continue;
                }

                TetCustomRotationPose HalfTurnPose;
                double HalfTurnRotation = 0.0;
                const bool HasHalfTurn = CetRotationUtils::SnapToAllowedRotation(BaseRotation + CET_CLUSTER_PI,AOptions.Rotations,HalfTurnRotation,1e-9) &&BuildRotationPose(Geometry,AOriginalItems[AIndices.front()],HalfTurnRotation,HalfTurnPose);

                for (const TetCustomLayoutPattern& Pattern : Patterns){
                    if (Pattern.AlternateHalfTurn && !HasHalfTurn){
                        continue;
                    }

                    const double CellWidth = Pattern.AlternateHalfTurn ? std::max(BasePose.Width,HalfTurnPose.Width) : BasePose.Width;
                    const double CellHeight = Pattern.AlternateHalfTurn ? std::max(BasePose.Height,HalfTurnPose.Height) : BasePose.Height;
                    const double ColumnPitch = CellWidth * Pattern.ColumnPitchRatio + LayoutGap;
                    const double RowPitch = CellHeight * Pattern.RowPitchRatio + LayoutGap;

                    const std::vector<std::size_t> RowCounts = BuildLayoutRowCounts(AIndices.size(),CellWidth,CellHeight,ColumnPitch,RowPitch,Pattern.RowStaggerRatio,AOptions);
                    for (std::size_t RowCount : RowCounts){
                        const std::size_t ColumnCount = (AIndices.size() + RowCount - 1) / RowCount;
                        TetClusterCandidate Candidate;
                        const CustomLayoutCandidateRequest Request{ AOriginalItems, AIndices, AOptions, Pattern, BasePose, HalfTurnPose, RowCount, ColumnCount, CellWidth, CellHeight, ColumnPitch, RowPitch };
                        if (!_BuildCustomLayoutCandidate(Request, Candidate)) continue;
                        const double CandidateBoxArea = Candidate.ClusterWidth * Candidate.ClusterHeight;
                        const double BestBoxArea = BestCandidate.ClusterWidth * BestCandidate.ClusterHeight;
                        if (!HasBest || Candidate.Score > BestCandidate.Score ||(std::abs(Candidate.Score - BestCandidate.Score) <= 1e-9 && CandidateBoxArea < BestBoxArea)){
                            HasBest = true;
                            BestCandidate = std::move(Candidate);
                        }
                    }
                }
            }

            if (!HasBest){
                return false;
            }
            if (BestCandidate.BoardSpanRatio > 0.50 && BestCandidate.BoundingFillRatio < 0.60){
                return false;
            }
            AOutCandidate = std::move(BestCandidate);
            return true;
        }

        bool CetCustomClusterBuilder::_RemapCandidateIndices(const TetClusterCandidate& ASourceCandidate, const std::vector<int>& AIndices, TetClusterCandidate& AOutCandidate)
        {
            AOutCandidate = TetClusterCandidate{};
            if (!ASourceCandidate.Valid || AIndices.size() != ASourceCandidate.Transforms.size() || AIndices.size() < 2){
                return false;
            }

            AOutCandidate = ASourceCandidate;
            AOutCandidate.OriginalIndices = AIndices;
            for (std::size_t IndexOffset = 0; IndexOffset < AIndices.size(); ++IndexOffset){
                AOutCandidate.Transforms[IndexOffset].OriginalId = AIndices[IndexOffset];
            }
            return true;
        }

        bool CetCustomClusterBuilder::_IsSupportedCustomShape(const TetShapeFeature& AFeature)
        {
            return (AFeature.ShapeType == MetShapeType::QuadrilateralLike ||AFeature.ShapeType == MetShapeType::ConvexPolygon ||AFeature.ShapeType == MetShapeType::ConcavePolygon) &&AFeature.VertexCount >= 3 && AFeature.Area > 0.0 && AFeature.Width > 0.0 && AFeature.Height > 0.0;
        }

        double CetCustomClusterBuilder::_CalculateScore(const TetClusterCandidate& ACandidate, const TetNestOptions& AOptions)
        {
            if (!ACandidate.Valid || ACandidate.OriginalIndices.size() < 2 ||ACandidate.ProxyArea <= 0.0 || ACandidate.ClusterWidth <= 0.0 || ACandidate.ClusterHeight <= 0.0){
                return -std::numeric_limits<double>::infinity();
            }

            (void)AOptions;
            const double FillScore = ACandidate.FillRatio * 700.0;
            const double SavingScore = ACandidate.AreaSavingRatio * 500.0;
            const double BoundingFillScore = ACandidate.BoundingFillRatio * 1600.0;
            const double ReuseScore = ACandidate.SheetReuseScore * 1000.0;
            const double CompactScore = ACandidate.CompactnessRatio * 500.0;
            const double ItemCountScore = std::sqrt(static_cast<double>(ACandidate.OriginalIndices.size())) * 35.0;
            const double AlternatingBonus = ACandidate.ClusterType.find("Alternating") != std::string::npos ? 12.0 : 0.0;
            const double WastePenalty = ACandidate.ProxyWasteRatio * 700.0;
            const double FragmentationPenalty = ACandidate.FragmentationRisk * 1200.0;
            const double LongSparsePenalty =
                std::clamp((ACandidate.BoardSpanRatio - 0.65) / 0.35, 0.0, 1.0) *
                std::clamp(1.0 - ACandidate.BoundingFillRatio, 0.0, 1.0) *
                std::clamp(1.0 - ACandidate.CompactnessRatio, 0.0, 1.0) *
                1800.0;
            const double SizePenalty = (ACandidate.ClusterWidth + ACandidate.ClusterHeight) * 0.000001;
            return FillScore + SavingScore + BoundingFillScore + ReuseScore + CompactScore +
                ItemCountScore + AlternatingBonus + ACandidate.Confidence -
                WastePenalty - FragmentationPenalty - LongSparsePenalty - SizePenalty;
        }

    }
}
