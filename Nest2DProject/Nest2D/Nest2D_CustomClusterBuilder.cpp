#include "pch.h"
#include "Nest2D_CustomClusterBuilder.h"
#include "Nest2D_ClusterGeometryHelper.h"
#include "Nest2D_RotationUtils.h"
#include "NestUtils.h"

#include <algorithm>
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
                return RequiredGap + std::max(2.0, RequiredGap * 0.001);
            }

            struct TetCustomRotationPose
            {
                double Rotation = 0.0;
                double MinX = 0.0;
                double MinY = 0.0;
                double Width = 0.0;
                double Height = 0.0;
            };

            struct TetCustomLayoutPattern
            {
                const char* Name = "";
                double ColumnPitchRatio = 1.0;
                double RowPitchRatio = 1.0;
                double RowStaggerRatio = 0.0;
                bool AlternateHalfTurn = false;
            };

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

            std::cout << "[CUSTOM][BUILD CANDIDATES] IndexCount=" << AIndices.size() << ", ShapeGroupCount=" << IndicesByShape.size() << ", NewCandidateCount=" << AOutCandidates.size() - OldCandidateCount << std::endl;
        }

        void CetCustomClusterBuilder::_BuildSameShapeClusterCandidates(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, std::vector<TetClusterCandidate>& AOutCandidates)
        {
            (void)AFeatures;
            if (AIndices.size() < 2){
                return;
            }

            CetClusterGeometryHelper Geometry;
            const std::size_t MaxChildCount = std::min(AIndices.size(),CET_CUSTOM_MAX_CLUSTER_CHILDREN);
            std::size_t PreferredChildCount = 0;
            TetClusterCandidate FirstCandidate;
            for (std::size_t TrialChildCount = MaxChildCount; TrialChildCount >= 2; --TrialChildCount){
                std::vector<int> TrialIndices(AIndices.begin(),AIndices.begin() + static_cast<std::vector<int>::difference_type>(TrialChildCount));
                TetClusterCandidate Candidate;
                if (_BuildBestLayoutCandidate(AOriginalItems,TrialIndices,AOptions,Candidate)){
                    const std::size_t RequiredCopies = std::min(CET_CLUSTER_TARGET_COPIES_PER_BOARD,AIndices.size() / TrialChildCount);
                    if (Geometry.CanPlaceCandidateCopiesOnBoard(Candidate,AOptions,RequiredCopies)){
                        PreferredChildCount = TrialChildCount;
                        FirstCandidate = std::move(Candidate);
                        break;
                    }
                }
            }

            if (PreferredChildCount < 2){
                std::cout << "[CUSTOM][REJECT] No valid complete layout. GroupCount=" << AIndices.size() << std::endl;
                return;
            }

            std::size_t GroupOffset = 0;
            while (GroupOffset + 1 < AIndices.size()){
                std::size_t TrialChildCount = std::min(PreferredChildCount,AIndices.size() - GroupOffset);
                bool HasCandidate = false;
                TetClusterCandidate BestCandidate;

                while (TrialChildCount >= 2){
                    std::vector<int> TrialIndices(AIndices.begin() + static_cast<std::vector<int>::difference_type>(GroupOffset),AIndices.begin() + static_cast<std::vector<int>::difference_type>(GroupOffset + TrialChildCount));
                    if (GroupOffset == 0 && TrialChildCount == PreferredChildCount){
                        BestCandidate = FirstCandidate;
                        HasCandidate = true;
                        break;
                    }
                    if (TrialChildCount == PreferredChildCount && _RemapCandidateIndices(FirstCandidate,TrialIndices,BestCandidate)){
                        HasCandidate = true;
                        break;
                    }
                    if (_BuildBestLayoutCandidate(AOriginalItems,TrialIndices,AOptions,BestCandidate)){
                        HasCandidate = true;
                        break;
                    }
                    --TrialChildCount;
                }

                if (!HasCandidate){
                    break;
                }

                std::cout << "[CUSTOM][CANDIDATE] ChildCount=" << TrialChildCount << ", Type=" << BestCandidate.ClusterType << ", Score=" << BestCandidate.Score << std::endl;
                AOutCandidates.push_back(std::move(BestCandidate));
                GroupOffset += TrialChildCount;
            }
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

                    for (std::size_t RowCount = 1; RowCount <= AIndices.size(); ++RowCount){
                        const std::size_t ColumnCount = (AIndices.size() + RowCount - 1) / RowCount;
                        TetClusterCandidate Candidate;
                        Candidate.BuilderName = "CustomBuilder";
                        Candidate.ClusterType = "CustomLayout_" + std::to_string(AIndices.size()) + "_R" + std::to_string(RowCount) + "_" + Pattern.Name;
                        Candidate.OriginalIndices = AIndices;
                        Candidate.Confidence = Pattern.AlternateHalfTurn ? 0.86 : 0.80;
                        Candidate.Transforms.reserve(AIndices.size());

                        std::size_t ItemOffset = 0;
                        for (std::size_t Row = 0; Row < RowCount && ItemOffset < AIndices.size(); ++Row){
                            const std::size_t RowItemCount = std::min(ColumnCount,AIndices.size() - ItemOffset);
                            double RowOffsetX = static_cast<double>(ColumnCount - RowItemCount) * ColumnPitch * 0.5;
                            if ((Row % 2) != 0){
                                RowOffsetX += Pattern.RowStaggerRatio * ColumnPitch;
                            }

                            for (std::size_t Column = 0; Column < RowItemCount; ++Column){
                                const bool UseHalfTurn = Pattern.AlternateHalfTurn && ((Row + Column) % 2) != 0;
                                const TetCustomRotationPose& Pose = UseHalfTurn ? HalfTurnPose : BasePose;
                                TetItemTransform Transform;
                                Transform.OriginalId = AIndices[ItemOffset];
                                Transform.RelativeRotation = Pose.Rotation;
                                Transform.RelativeX = RowOffsetX + static_cast<double>(Column) * ColumnPitch + (CellWidth - Pose.Width) * 0.5 - Pose.MinX;
                                Transform.RelativeY = static_cast<double>(Row) * RowPitch + (CellHeight - Pose.Height) * 0.5 - Pose.MinY;
                                Candidate.Transforms.push_back(Transform);
                                ++ItemOffset;
                            }
                        }

                        if (!Geometry.FinalizeCandidate(AOriginalItems,AOptions,Candidate) ||Candidate.AreaSavingRatio < -CET_CUSTOM_MAX_AREA_LOSS_RATIO){
                            continue;
                        }

                        Candidate.Score = _CalculateScore(Candidate,AOptions);
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

            const double BoundingArea = ACandidate.ClusterWidth * ACandidate.ClusterHeight;
            const double BoundingFillRatio = BoundingArea > 0.0 ? std::clamp(ACandidate.RealArea / BoundingArea,0.0,1.0) : 0.0;
            const double BinWidth = std::max(1.0,static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth)));
            const double BinHeight = std::max(1.0,static_cast<double>(NestUtils::ToNestCoord(AOptions.BinHeight)));
            const double BoardSpanRatio = std::max(ACandidate.ClusterWidth / BinWidth,ACandidate.ClusterHeight / BinHeight);
            const double FillScore = ACandidate.FillRatio * 900.0;
            const double SavingScore = ACandidate.AreaSavingRatio * 800.0;
            const double BoundingFillScore = BoundingFillRatio * 650.0;
            const double ItemCountScore = static_cast<double>(ACandidate.OriginalIndices.size()) * 18.0;
            const double LongSide = std::max(ACandidate.ClusterWidth, ACandidate.ClusterHeight);
            const double ShortSide = std::min(ACandidate.ClusterWidth, ACandidate.ClusterHeight);
            const double CompactScore = LongSide > 0.0 ? ShortSide / LongSide * 90.0 : 0.0;
            const double AlternatingBonus = ACandidate.ClusterType.find("Alternating") != std::string::npos ? 35.0 : 0.0;
            const double WastePenalty = ACandidate.ProxyWasteRatio * 180.0;
            const double BoardSpanPenalty = BoardSpanRatio * 220.0;
            const double SizePenalty = (ACandidate.ClusterWidth + ACandidate.ClusterHeight) * 0.000001;
            return FillScore + SavingScore + BoundingFillScore + ItemCountScore + CompactScore + AlternatingBonus + ACandidate.Confidence - WastePenalty - BoardSpanPenalty - SizePenalty;
        }

    }
}
