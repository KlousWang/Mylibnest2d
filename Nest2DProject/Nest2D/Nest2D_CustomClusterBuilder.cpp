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

            std::vector<TetCustomEdgeInfo> CollectLongestEdges(CetPath AContour)
            {
                std::vector<TetCustomEdgeInfo> Edges;
                if (AContour.size() < 3){
                    return Edges;
                }
                if (!ClipperLib::Orientation(AContour)){
                    std::reverse(AContour.begin(), AContour.end());
                }

                Edges.reserve(AContour.size());
                for (std::size_t Index = 0; Index < AContour.size(); ++Index){
                    const CetInpoint& Start = AContour[Index];
                    const CetInpoint& End = AContour[(Index + 1) % AContour.size()];
                    const double EdgeX = static_cast<double>(End.X - Start.X);
                    const double EdgeY = static_cast<double>(End.Y - Start.Y);
                    const double Length = std::hypot(EdgeX, EdgeY);
                    if (Length <= 0.0){
                        continue;
                    }
                    Edges.push_back({ Start, End, Length, std::atan2(EdgeY, EdgeX) });
                }

                std::stable_sort(Edges.begin(), Edges.end(), [](const TetCustomEdgeInfo& A, const TetCustomEdgeInfo& AOther) {
                    return A.Length > AOther.Length;
                    });
                if (Edges.size() > CET_CUSTOM_MAX_EDGE_CANDIDATES){
                    Edges.resize(CET_CUSTOM_MAX_EDGE_CANDIDATES);
                }
                return Edges;
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
            if (AIndices.size() < 2){
                return;
            }

            TetClusterCandidate BasePair;
            if (!_BuildBestPairCandidate(AOriginalItems,AIndices[0],AIndices[1],AOptions,BasePair)){
                std::cout << "[CUSTOM][REJECT] No valid representative pair. GroupCount=" << AIndices.size() << std::endl;
                return;
            }

            CetClusterGeometryHelper Geometry;
            std::size_t PreferredChildCount = std::min(AIndices.size(), CET_CUSTOM_MAX_CLUSTER_CHILDREN);
            PreferredChildCount -= PreferredChildCount % 2;
            TetClusterCandidate FirstCandidate;
            while (PreferredChildCount >= 2){
                std::vector<int> TrialIndices(AIndices.begin(),AIndices.begin() + static_cast<std::vector<int>::difference_type>(PreferredChildCount));
                TetClusterCandidate Candidate;
                if (_BuildClusterCandidateFromPair(AOriginalItems,BasePair,TrialIndices,AOptions,Candidate)){
                    const std::size_t RequiredCopies = std::min(CET_CLUSTER_TARGET_COPIES_PER_BOARD,AIndices.size() / PreferredChildCount);
                    if (Geometry.CanPlaceCandidateCopiesOnBoard(Candidate,AOptions,RequiredCopies)){
                        FirstCandidate = std::move(Candidate);
                        break;
                    }
                }
                PreferredChildCount -= 2;
            }

            if (PreferredChildCount < 2){
                return;
            }

            std::size_t GroupOffset = 0;
            while (GroupOffset + 1 < AIndices.size()){
                std::size_t TrialChildCount = std::min(PreferredChildCount,AIndices.size() - GroupOffset);
                TrialChildCount -= TrialChildCount % 2;
                bool HasCandidate = false;
                TetClusterCandidate BestCandidate;

                while (TrialChildCount >= 2){
                    if (GroupOffset == 0 && TrialChildCount == PreferredChildCount){
                        BestCandidate = std::move(FirstCandidate);
                        HasCandidate = true;
                        break;
                    }

                    std::vector<int> TrialIndices(AIndices.begin() + static_cast<std::vector<int>::difference_type>(GroupOffset),AIndices.begin() + static_cast<std::vector<int>::difference_type>(GroupOffset + TrialChildCount));
                    if (_BuildClusterCandidateFromPair(AOriginalItems,BasePair,TrialIndices,AOptions,BestCandidate)){
                        HasCandidate = true;
                        break;
                    }
                    TrialChildCount -= 2;
                }

                if (!HasCandidate){
                    break;
                }

                std::cout << "[CUSTOM][CANDIDATE] ChildCount=" << TrialChildCount << ", Type=" << BestCandidate.ClusterType << ", Score=" << BestCandidate.Score << std::endl;
                AOutCandidates.push_back(std::move(BestCandidate));
                GroupOffset += TrialChildCount;
            }
        }

        bool CetCustomClusterBuilder::_BuildBestPairCandidate(const CetTNestItemVector& AOriginalItems, int AFirstIndex, int ASecondIndex, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate)
        {
            AOutCandidate = TetClusterCandidate{};
            if (AFirstIndex < 0 || ASecondIndex < 0 || AFirstIndex == ASecondIndex ||AFirstIndex >= static_cast<int>(AOriginalItems.size()) || ASecondIndex >= static_cast<int>(AOriginalItems.size())){
                return false;
            }

            CetClusterGeometryHelper Geometry;
            const CetPath FirstContour = Geometry.GetIdentityContour(AOriginalItems[AFirstIndex]);
            const CetPath SecondContour = Geometry.GetIdentityContour(AOriginalItems[ASecondIndex]);
            const std::vector<TetCustomEdgeInfo> FirstEdges = CollectLongestEdges(FirstContour);
            const std::vector<TetCustomEdgeInfo> SecondEdges = CollectLongestEdges(SecondContour);
            const double LayoutGap = GetLayoutGap(AOptions);

            double FirstMinX = 0.0;
            double FirstMinY = 0.0;
            double FirstMaxX = 0.0;
            double FirstMaxY = 0.0;
            if (!Geometry.GetBounds(FirstContour,FirstMinX,FirstMinY,FirstMaxX,FirstMaxY)){
                return false;
            }

            bool HasBest = false;
            TetClusterCandidate BestCandidate;
            auto TryCandidate = [&](TetItemTransform AFirstTransform, TetItemTransform ASecondTransform, const std::string& AClusterType) {
                std::vector<TetItemTransform> Transforms = { AFirstTransform, ASecondTransform };
                if (!Geometry.HasValidTransformSpacing(AOriginalItems,AOptions,Transforms)){
                    return;
                }

                TetClusterCandidate Candidate;
                Candidate.BuilderName = "CustomBuilder";
                Candidate.ClusterType = AClusterType;
                Candidate.OriginalIndices = { AFirstIndex, ASecondIndex };
                Candidate.Transforms = std::move(Transforms);
                Candidate.Confidence = 0.72;
                if (!Geometry.FinalizeCandidate(AOriginalItems,AOptions,Candidate) ||Candidate.AreaSavingRatio < -CET_CUSTOM_MAX_AREA_LOSS_RATIO){
                    return;
                }

                Candidate.Score = _CalculateScore(Candidate);
                if (!HasBest || Candidate.Score > BestCandidate.Score){
                    HasBest = true;
                    BestCandidate = std::move(Candidate);
                }
                };

            TetItemTransform FirstTransform;
            FirstTransform.OriginalId = AFirstIndex;
            FirstTransform.RelativeX = -FirstMinX;
            FirstTransform.RelativeY = -FirstMinY;

            for (const TetCustomEdgeInfo& FirstEdge : FirstEdges){
                for (const TetCustomEdgeInfo& SecondEdge : SecondEdges){
                    const double LengthDenominator = std::max(FirstEdge.Length, SecondEdge.Length);
                    if (LengthDenominator <= 0.0 ||std::abs(FirstEdge.Length - SecondEdge.Length) / LengthDenominator > CET_CUSTOM_EDGE_LENGTH_TOLERANCE){
                        continue;
                    }

                    double SecondRotation = 0.0;
                    const double TargetRotation = FirstEdge.Angle + CET_CLUSTER_PI - SecondEdge.Angle;
                    if (!CetRotationUtils::SnapToAllowedRotation(TargetRotation,AOptions.Rotations,SecondRotation,CET_CUSTOM_EDGE_ANGLE_TOLERANCE)){
                        continue;
                    }

                    const double CosRotation = std::cos(SecondRotation);
                    const double SinRotation = std::sin(SecondRotation);
                    const double RotatedSecondStartX = static_cast<double>(SecondEdge.Start.X) * CosRotation - static_cast<double>(SecondEdge.Start.Y) * SinRotation;
                    const double RotatedSecondStartY = static_cast<double>(SecondEdge.Start.X) * SinRotation + static_cast<double>(SecondEdge.Start.Y) * CosRotation;
                    const double FirstEdgeX = static_cast<double>(FirstEdge.End.X - FirstEdge.Start.X);
                    const double FirstEdgeY = static_cast<double>(FirstEdge.End.Y - FirstEdge.Start.Y);
                    const double OutwardNormalX = FirstEdgeY / FirstEdge.Length;
                    const double OutwardNormalY = -FirstEdgeX / FirstEdge.Length;
                    const double DesiredSecondStartX = static_cast<double>(FirstEdge.End.X) - FirstMinX + OutwardNormalX * LayoutGap;
                    const double DesiredSecondStartY = static_cast<double>(FirstEdge.End.Y) - FirstMinY + OutwardNormalY * LayoutGap;

                    TetItemTransform SecondTransform;
                    SecondTransform.OriginalId = ASecondIndex;
                    SecondTransform.RelativeRotation = SecondRotation;
                    SecondTransform.RelativeX = DesiredSecondStartX - RotatedSecondStartX;
                    SecondTransform.RelativeY = DesiredSecondStartY - RotatedSecondStartY;
                    TryCandidate(FirstTransform,SecondTransform,"CustomEdgePair");
                }
            }

            const double FirstWidth = FirstMaxX - FirstMinX;
            const double FirstHeight = FirstMaxY - FirstMinY;
            const std::vector<double> Rotations = CetRotationUtils::BuildAllowedRotations(AOptions.Rotations);
            const double Alignments[] = { 0.0, 0.5, 1.0 };
            for (double SecondRotation : Rotations){
                double SecondMinX = 0.0;
                double SecondMinY = 0.0;
                double SecondMaxX = 0.0;
                double SecondMaxY = 0.0;
                if (!GetRotatedBounds(Geometry,AOriginalItems[ASecondIndex],SecondRotation,SecondMinX,SecondMinY,SecondMaxX,SecondMaxY)){
                    continue;
                }
                const double SecondWidth = SecondMaxX - SecondMinX;
                const double SecondHeight = SecondMaxY - SecondMinY;

                for (double Alignment : Alignments){
                    TetItemTransform HorizontalTransform;
                    HorizontalTransform.OriginalId = ASecondIndex;
                    HorizontalTransform.RelativeRotation = SecondRotation;
                    HorizontalTransform.RelativeX = FirstWidth + LayoutGap - SecondMinX;
                    HorizontalTransform.RelativeY = Alignment * (FirstHeight - SecondHeight) - SecondMinY;
                    TryCandidate(FirstTransform,HorizontalTransform,"CustomBoxPairHorizontal");

                    TetItemTransform VerticalTransform;
                    VerticalTransform.OriginalId = ASecondIndex;
                    VerticalTransform.RelativeRotation = SecondRotation;
                    VerticalTransform.RelativeX = Alignment * (FirstWidth - SecondWidth) - SecondMinX;
                    VerticalTransform.RelativeY = FirstHeight + LayoutGap - SecondMinY;
                    TryCandidate(FirstTransform,VerticalTransform,"CustomBoxPairVertical");
                }
            }

            if (!HasBest){
                return false;
            }
            AOutCandidate = std::move(BestCandidate);
            return true;
        }

        bool CetCustomClusterBuilder::_BuildClusterCandidateFromPair(const CetTNestItemVector& AOriginalItems, const TetClusterCandidate& ABasePair, const std::vector<int>& AIndices, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate)
        {
            AOutCandidate = TetClusterCandidate{};
            if (!ABasePair.Valid || ABasePair.Transforms.size() != 2 || AIndices.size() < 2 ||AIndices.size() > CET_CUSTOM_MAX_CLUSTER_CHILDREN || (AIndices.size() % 2) != 0){
                return false;
            }

            CetClusterGeometryHelper Geometry;
            const std::size_t PairCount = AIndices.size() / 2;
            const double CellGap = GetLayoutGap(AOptions);
            bool HasBest = false;
            TetClusterCandidate BestCandidate;

            for (std::size_t RowCount = 1; RowCount <= PairCount; ++RowCount){
                const std::size_t ColumnCount = (PairCount + RowCount - 1) / RowCount;
                TetClusterCandidate Candidate;
                Candidate.BuilderName = "CustomBuilder";
                Candidate.ClusterType = "CustomPairGrid_" + std::to_string(AIndices.size()) + "_R" + std::to_string(RowCount);
                Candidate.OriginalIndices = AIndices;
                Candidate.Confidence = 0.76;
                Candidate.Transforms.reserve(AIndices.size());

                bool LayoutValid = true;
                for (std::size_t PairIndex = 0; PairIndex < PairCount && LayoutValid; ++PairIndex){
                    const std::size_t Row = PairIndex / ColumnCount;
                    const std::size_t Column = PairIndex % ColumnCount;
                    const double CellOffsetX = static_cast<double>(Column) * (ABasePair.ClusterWidth + CellGap);
                    const double CellOffsetY = static_cast<double>(Row) * (ABasePair.ClusterHeight + CellGap);

                    for (std::size_t Role = 0; Role < 2; ++Role){
                        const TetItemTransform& ReferenceTransform = ABasePair.Transforms[Role];
                        const int OriginalIndex = AIndices[PairIndex * 2 + Role];

                        const CetPath ReferenceContour = Geometry.TransformContour(Geometry.GetIdentityContour(AOriginalItems[ReferenceTransform.OriginalId]),ReferenceTransform.RelativeRotation,ReferenceTransform.RelativeX,ReferenceTransform.RelativeY);
                        double ReferenceMinX = 0.0;
                        double ReferenceMinY = 0.0;
                        double ReferenceMaxX = 0.0;
                        double ReferenceMaxY = 0.0;
                        double CurrentMinX = 0.0;
                        double CurrentMinY = 0.0;
                        double CurrentMaxX = 0.0;
                        double CurrentMaxY = 0.0;
                        if (!Geometry.GetBounds(ReferenceContour,ReferenceMinX,ReferenceMinY,ReferenceMaxX,ReferenceMaxY) ||!GetRotatedBounds(Geometry,AOriginalItems[OriginalIndex],ReferenceTransform.RelativeRotation,CurrentMinX,CurrentMinY,CurrentMaxX,CurrentMaxY)){
                            LayoutValid = false;
                            break;
                        }

                        TetItemTransform Transform;
                        Transform.OriginalId = OriginalIndex;
                        Transform.RelativeRotation = ReferenceTransform.RelativeRotation;
                        Transform.RelativeX = CellOffsetX + ReferenceMinX - CurrentMinX;
                        Transform.RelativeY = CellOffsetY + ReferenceMinY - CurrentMinY;
                        Candidate.Transforms.push_back(Transform);
                    }
                }

                if (!LayoutValid || !Geometry.HasValidTransformSpacing(AOriginalItems,AOptions,Candidate.Transforms) ||!Geometry.FinalizeCandidate(AOriginalItems,AOptions,Candidate) ||Candidate.AreaSavingRatio < -CET_CUSTOM_MAX_AREA_LOSS_RATIO){
                    continue;
                }

                Candidate.Score = _CalculateScore(Candidate);
                if (!HasBest || Candidate.Score > BestCandidate.Score){
                    HasBest = true;
                    BestCandidate = std::move(Candidate);
                }
            }

            if (!HasBest){
                return false;
            }
            AOutCandidate = std::move(BestCandidate);
            return true;
        }

        bool CetCustomClusterBuilder::_IsSupportedCustomShape(const TetShapeFeature& AFeature)
        {
            return (AFeature.ShapeType == MetShapeType::QuadrilateralLike ||AFeature.ShapeType == MetShapeType::ConvexPolygon ||AFeature.ShapeType == MetShapeType::ConcavePolygon) &&AFeature.VertexCount >= 3 && AFeature.Area > 0.0 && AFeature.Width > 0.0 && AFeature.Height > 0.0;
        }

        double CetCustomClusterBuilder::_CalculateScore(const TetClusterCandidate& ACandidate)
        {
            if (!ACandidate.Valid || ACandidate.OriginalIndices.size() < 2 ||ACandidate.ProxyArea <= 0.0 || ACandidate.ClusterWidth <= 0.0 || ACandidate.ClusterHeight <= 0.0){
                return -std::numeric_limits<double>::infinity();
            }

            const double FillScore = ACandidate.FillRatio * 1050.0;
            const double SavingScore = ACandidate.AreaSavingRatio * 950.0;
            const double ItemCountScore = static_cast<double>(ACandidate.OriginalIndices.size()) * 45.0;
            const double LongSide = std::max(ACandidate.ClusterWidth, ACandidate.ClusterHeight);
            const double ShortSide = std::min(ACandidate.ClusterWidth, ACandidate.ClusterHeight);
            const double CompactScore = LongSide > 0.0 ? ShortSide / LongSide * 40.0 : 0.0;
            const double EdgePairBonus = ACandidate.ClusterType.find("EdgePair") != std::string::npos ? 90.0 : 0.0;
            const double WastePenalty = ACandidate.ProxyWasteRatio * 160.0;
            const double SizePenalty = (ACandidate.ClusterWidth + ACandidate.ClusterHeight) * 0.000001;
            return FillScore + SavingScore + ItemCountScore + CompactScore + EdgePairBonus + ACandidate.Confidence - WastePenalty - SizePenalty;
        }

    }
}
