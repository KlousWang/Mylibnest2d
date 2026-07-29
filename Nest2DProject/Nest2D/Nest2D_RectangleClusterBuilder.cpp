#include "pch.h"
#include "Nest2D_RectangleClusterBuilder.h"
#include "Nest2D_ClusterGeometryHelper.h"
#include "Nest2D_RotationUtils.h"
#include "NestUtils.h"
#include "Nest2D_PrivateDataType.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

namespace ET {
    namespace NEST2DMANAGERLIB {

        
        constexpr double CET_RECT_SIZE_TOLERANCE = 0.05;

        
        constexpr double CET_RECT_SQUARE_TOLERANCE = 0.01;

        
        
        constexpr double CET_RECT_MAX_AREA_LOSS_RATIO = 0.10;
        constexpr std::size_t CET_RECT_MAX_CLUSTER_CHILDREN = 32;
        namespace {

            bool NearlyEqual(double A, double AB, double ARelativeTolerance)
            {
                const double Denominator = std::max(1.0, std::max(std::abs(A), std::abs(AB)));
                return std::abs(A - AB) <= Denominator * ARelativeTolerance;
            }

            void GetCanonicalSides(const TetShapeFeature& AFeature, double& AOutShortSide, double& AOutLongSide)
            {
                AOutShortSide = std::min(AFeature.OrientedWidth, AFeature.OrientedHeight);
                AOutLongSide = std::max(AFeature.OrientedWidth, AFeature.OrientedHeight);
            }
            bool _MakePose(const CetNestItem& AItem, const TetShapeFeature& AFeature, bool ARotate90, const TetNestOptions& AOptions, const CetClusterGeometryHelper& AGeometry, TetRectanglePose& AOutPose)
            {
                AOutPose = TetRectanglePose{};

                






                const double TargetRotation = -AFeature.OrientedAngle + (ARotate90 ? CET_CLUSTER_HALF_PI : 0.0);
                if (!CetRotationUtils::SnapToNearestAllowedRotation(TargetRotation, AOptions.Rotations, AOutPose.Rotation)){
                    return false;
                }
                const CetPath Contour = AGeometry.TransformContour(AGeometry.GetIdentityContour(AItem), AOutPose.Rotation, 0.0, 0.0);
                double MaxX = 0.0;
                double MaxY = 0.0;
                if (!AGeometry.GetBounds(Contour, AOutPose.MinX, AOutPose.MinY, MaxX, MaxY)){ return false; }

                AOutPose.Width = MaxX - AOutPose.MinX;
                AOutPose.Height = MaxY - AOutPose.MinY;

                return AOutPose.Width > 0.0 && AOutPose.Height > 0.0;
            }

        }

        CetRectangleClusterBuilder::CetRectangleClusterBuilder() : CetCoreObject() {}
        CetRectangleClusterBuilder::~CetRectangleClusterBuilder() {}

        void CetRectangleClusterBuilder::BuildCandidates(const CetTNestItemVector& AItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, std::vector<TetClusterCandidate>& AOut)
        {
            if (AItems.size() != AFeatures.size()){
                std::cout << "[RECTANGLE][ERROR] " << "Feature count mismatch. Items=" << AItems.size() << ", Features=" << AFeatures.size() << std::endl;
                return;
            }

            if (AIndices.size() < 2){ return; }

            std::vector<int> ValidIndices;
            ValidIndices.reserve(AIndices.size());
            for (int Index : AIndices){
                if (Index < 0 || Index >= static_cast<int>(AFeatures.size())){ continue; }
                if (_IsValidRectangle(AFeatures[Index])){
                    ValidIndices.push_back(Index);
                }
            }

            std::sort(ValidIndices.begin(), ValidIndices.end());
            ValidIndices.erase(std::unique(ValidIndices.begin(), ValidIndices.end()), ValidIndices.end());
            if (ValidIndices.size() < 2){ return; }

            const std::size_t OldCandidateCount = AOut.size();
            std::vector<std::vector<int>> Groups;
            for (int Index : ValidIndices){
                bool AddedToGroup = false;
                for (std::vector<int>& Group : Groups){
                    if (!Group.empty() && _AreCompatible(AFeatures[Group.front()], AFeatures[Index])){
                        Group.push_back(Index);
                        AddedToGroup = true;
                        break;
                    }
                }
                if (!AddedToGroup){
                    Groups.push_back({ Index });
                }
            }

            for (const std::vector<int>& Group : Groups){
                if (Group.size() < 2){
                    continue;
                }

                CetClusterGeometryHelper Geometry;
                const std::size_t MaxChildCount = std::min(Group.size(), CET_RECT_MAX_CLUSTER_CHILDREN);
                std::size_t PreferredChildCount = 0;
                TetClusterCandidate FirstCandidate;
                for (std::size_t TrialChildCount = MaxChildCount; TrialChildCount >= 2; --TrialChildCount){
                    std::vector<int> ClusterIndices(Group.begin(),Group.begin() + static_cast<std::vector<int>::difference_type>(TrialChildCount));

                    TetClusterCandidate Candidate;
                    if (!_MakeGridCandidate(AItems, AFeatures, ClusterIndices, AOptions, Candidate)){
                        continue;
                    }
                    const std::size_t RequiredCopies = std::min(CET_CLUSTER_TARGET_COPIES_PER_BOARD,Group.size() / TrialChildCount);
                    if (!Geometry.CanPlaceCandidateCopiesOnBoard(Candidate, AOptions, RequiredCopies)){
                        continue;
                    }

                    PreferredChildCount = TrialChildCount;
                    FirstCandidate = std::move(Candidate);
                    break;
                }

                if (PreferredChildCount < 2){
                    continue;
                }

                std::size_t GroupOffset = 0;
                while (GroupOffset + 1 < Group.size()){
                    const std::size_t RemainingCount = Group.size() - GroupOffset;
                    std::size_t TrialChildCount = std::min(RemainingCount, PreferredChildCount);
                    std::size_t BestChildCount = 0;
                    TetClusterCandidate BestCandidate;

                    while (TrialChildCount >= 2){
                        std::vector<int> ClusterIndices(Group.begin() + static_cast<std::vector<int>::difference_type>(GroupOffset),Group.begin() + static_cast<std::vector<int>::difference_type>(GroupOffset + TrialChildCount));

                        if (GroupOffset == 0 && TrialChildCount == PreferredChildCount){
                            BestCandidate = std::move(FirstCandidate);
                            BestChildCount = TrialChildCount;
                            break;
                        }

                        TetClusterCandidate Candidate;
                        if (_MakeGridCandidate(AItems, AFeatures, ClusterIndices, AOptions, Candidate)){
                            BestCandidate = std::move(Candidate);
                            BestChildCount = TrialChildCount;
                            break;
                        }

                        --TrialChildCount;
                    }

                    if (BestChildCount < 2){
                        break;
                    }

                    GroupOffset += BestChildCount;
                    std::cout << "[RECTANGLE][CANDIDATE] ChildCount=" << BestCandidate.OriginalIndices.size() << ", Type=" << BestCandidate.ClusterType << ", Score=" << BestCandidate.Score << std::endl;
                    AOut.push_back(std::move(BestCandidate));
                }
            }

            std::cout << "[RECTANGLE][BUILD CANDIDATES] " << "IndexCount=" << ValidIndices.size() << ", GroupCount=" << Groups.size() << ", NewCandidateCount=" << AOut.size() - OldCandidateCount << std::endl;
        }
        bool CetRectangleClusterBuilder::_IsValidRectangle(const TetShapeFeature& AFeature)
        {
            return AFeature.ShapeType == MetShapeType::RectangleLike && AFeature.IsRotatedRectangle && !AFeature.HasHoles && AFeature.Area > 0.0 && AFeature.OrientedWidth > 0.0 && AFeature.OrientedHeight > 0.0;
        }

        bool CetRectangleClusterBuilder::_AreCompatible(const TetShapeFeature& AFeatureA, const TetShapeFeature& AFeatureB)
        {
            double ShortA = 0.0;
            double LongA = 0.0;
            double ShortB = 0.0;
            double LongB = 0.0;

            GetCanonicalSides(AFeatureA, ShortA, LongA);
            GetCanonicalSides(AFeatureB, ShortB, LongB);

            return NearlyEqual(ShortA, ShortB, CET_RECT_SIZE_TOLERANCE) && NearlyEqual(LongA, LongB, CET_RECT_SIZE_TOLERANCE);
        }

        bool CetRectangleClusterBuilder::_IsSquareLike(const TetShapeFeature& AFeature)
        {
            return NearlyEqual(AFeature.OrientedWidth, AFeature.OrientedHeight, CET_RECT_SQUARE_TOLERANCE);
        }

        bool CetRectangleClusterBuilder::_IsQuarterTurnAllowed(const TetNestOptions& AOptions)
        {
            constexpr double RotationTolerance = 1e-9;
            return CetRotationUtils::IsAllowedRotation(CET_CLUSTER_HALF_PI, AOptions.Rotations, RotationTolerance);
        }

        bool CetRectangleClusterBuilder::_MakePairCandidate(const CetTNestItemVector& AItems, const std::vector<TetShapeFeature>& AFeatures, int AIndexA, int AIndexB, bool AHorizontal, bool ARotateB90, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate)
        {
            AOutCandidate = TetClusterCandidate{};

            if (AIndexA < 0 || AIndexB < 0 || AIndexA == AIndexB || AIndexA >= static_cast<int>(AItems.size()) || AIndexB >= static_cast<int>(AItems.size()) || AIndexA >= static_cast<int>(AFeatures.size()) || AIndexB >= static_cast<int>(AFeatures.size())){ return false; }

            const TetShapeFeature& FeatureA = AFeatures[AIndexA];
            const TetShapeFeature& FeatureB = AFeatures[AIndexB];

            if (!_IsValidRectangle(FeatureA) || !_IsValidRectangle(FeatureB) || !_AreCompatible(FeatureA, FeatureB)){ return false; }

            CetClusterGeometryHelper Geometry;

            TetRectanglePose PoseA;
            TetRectanglePose PoseB;

            if (!_MakePose(AItems[AIndexA], FeatureA, false, AOptions, Geometry, PoseA) || !_MakePose(AItems[AIndexB], FeatureB, ARotateB90, AOptions, Geometry, PoseB)){ return false; }

            










            const double Gap = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));

            TetItemTransform TransformA;
            TransformA.OriginalId = AIndexA;
            TransformA.RelativeRotation = PoseA.Rotation;
            TransformA.RelativeX = -PoseA.MinX;
            TransformA.RelativeY = -PoseA.MinY;

            TetItemTransform TransformB;
            TransformB.OriginalId = AIndexB;
            TransformB.RelativeRotation = PoseB.Rotation;

            if (AHorizontal){
                






                TransformB.RelativeX = PoseA.Width + Gap - PoseB.MinX;
                TransformB.RelativeY = -PoseB.MinY;
            }
            else {
                










                TransformB.RelativeX = -PoseB.MinX;
                TransformB.RelativeY = PoseA.Height + Gap - PoseB.MinY;
            }

            AOutCandidate.BuilderName = "RectangleBuilder";
            AOutCandidate.ClusterType = AHorizontal ? "RectanglePairHorizontal" : "RectanglePairVertical";

            if (ARotateB90){
                AOutCandidate.ClusterType += "RotatedB90";
            }

            AOutCandidate.OriginalIndices = { AIndexA, AIndexB };
            AOutCandidate.Transforms = { TransformA, TransformB };

            



            AOutCandidate.Confidence = 0.60;

            










            if (!Geometry.FinalizeCandidate(AItems, AOptions, AOutCandidate)){ return false; }

            






            if (AOutCandidate.AreaSavingRatio < -CET_RECT_MAX_AREA_LOSS_RATIO){ return false; }

            return true;
        }

        bool CetRectangleClusterBuilder::_MakeGridCandidate(const CetTNestItemVector& AItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate)
        {
            AOutCandidate = TetClusterCandidate{};
            if (AItems.size() != AFeatures.size() || AIndices.size() < 2 ||AIndices.size() > CET_RECT_MAX_CLUSTER_CHILDREN){
                return false;
            }

            const int FirstIndex = AIndices.front();
            if (FirstIndex < 0 || FirstIndex >= static_cast<int>(AFeatures.size()) ||!_IsValidRectangle(AFeatures[FirstIndex])){
                return false;
            }

            CetClusterGeometryHelper Geometry;
            const bool AllowQuarterTurn = _IsQuarterTurnAllowed(AOptions);
            std::vector<TetRectanglePose> Poses;
            Poses.reserve(AIndices.size());
            double CellWidth = 0.0;
            double CellHeight = 0.0;

            for (int Index : AIndices){
                if (Index < 0 || Index >= static_cast<int>(AFeatures.size()) ||!_IsValidRectangle(AFeatures[Index]) ||!_AreCompatible(AFeatures[FirstIndex], AFeatures[Index])){
                    return false;
                }

                const TetShapeFeature& Feature = AFeatures[Index];
                const bool RotateLongSideHorizontal = AllowQuarterTurn && Feature.OrientedWidth < Feature.OrientedHeight;
                TetRectanglePose Pose;
                if (!_MakePose(AItems[Index], Feature, RotateLongSideHorizontal, AOptions, Geometry, Pose)){
                    if (!RotateLongSideHorizontal || !_MakePose(AItems[Index], Feature, false, AOptions, Geometry, Pose)){
                        return false;
                    }
                }

                CellWidth = std::max(CellWidth, Pose.Width);
                CellHeight = std::max(CellHeight, Pose.Height);
                Poses.push_back(Pose);
            }

            if (CellWidth <= 0.0 || CellHeight <= 0.0){
                return false;
            }

            const double RequiredGap = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));
            const double SafetyGap = RequiredGap > 0.0 ? std::max(10.0, RequiredGap * 0.001) : 0.0;
            const double CellGap = RequiredGap + SafetyGap;
            const double BinWidth = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth));
            const double BinHeight = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinHeight));
            const int ItemCount = static_cast<int>(AIndices.size());

            struct TetRectangleGridLayout
            {
                int Rows = 0;
                int Cols = 0;
                double Width = 0.0;
                double Height = 0.0;
                double Area = 0.0;
                double AspectPenalty = 0.0;
            };

            std::vector<TetRectangleGridLayout> Layouts;
            Layouts.reserve(AIndices.size());
            for (int Rows = 1; Rows <= ItemCount; ++Rows){
                const int Cols = (ItemCount + Rows - 1) / Rows;
                const double Width = static_cast<double>(Cols) * CellWidth + static_cast<double>(std::max(0, Cols - 1)) * CellGap;
                const double Height = static_cast<double>(Rows) * CellHeight + static_cast<double>(std::max(0, Rows - 1)) * CellGap;
                const bool FitsNormally = Width <= BinWidth && Height <= BinHeight;
                const bool FitsRotated = AllowQuarterTurn && Height <= BinWidth && Width <= BinHeight;
                if (!FitsNormally && !FitsRotated){
                    continue;
                }

                const double LongSide = std::max(Width, Height);
                const double ShortSide = std::min(Width, Height);
                Layouts.push_back({
                    Rows,
                    Cols,
                    Width,
                    Height,
                    Width * Height,
                    LongSide / std::max(1.0, ShortSide)
                    });
            }

            std::stable_sort(Layouts.begin(), Layouts.end(), [](const TetRectangleGridLayout& A, const TetRectangleGridLayout& AB) {
                if (std::abs(A.Area - AB.Area) > 1e-6){
                    return A.Area < AB.Area;
                }
                if (std::abs(A.AspectPenalty - AB.AspectPenalty) > 1e-6){
                    return A.AspectPenalty < AB.AspectPenalty;
                }
                return A.Rows < AB.Rows;
                });

            const std::size_t MaxLayoutChecks = std::min<std::size_t>(Layouts.size(), 3);
            for (std::size_t LayoutIndex = 0; LayoutIndex < MaxLayoutChecks; ++LayoutIndex){
                const TetRectangleGridLayout& Layout = Layouts[LayoutIndex];
                TetClusterCandidate Candidate;
                Candidate.BuilderName = "RectangleBuilder";
                Candidate.ClusterType = "RectangleGrid_" + std::to_string(AIndices.size()) + "_R" + std::to_string(Layout.Rows);
                Candidate.OriginalIndices = AIndices;
                Candidate.Confidence = 0.75;
                Candidate.Transforms.reserve(AIndices.size());

                for (int ItemOffset = 0; ItemOffset < ItemCount; ++ItemOffset){
                    const int Row = ItemOffset / Layout.Cols;
                    const int Col = ItemOffset % Layout.Cols;
                    const TetRectanglePose& Pose = Poses[static_cast<std::size_t>(ItemOffset)];
                    const double BaseX = static_cast<double>(Col) * (CellWidth + CellGap) + (CellWidth - Pose.Width) * 0.5;
                    const double BaseY = static_cast<double>(Row) * (CellHeight + CellGap) + (CellHeight - Pose.Height) * 0.5;

                    TetItemTransform Transform;
                    Transform.OriginalId = AIndices[static_cast<std::size_t>(ItemOffset)];
                    Transform.RelativeRotation = Pose.Rotation;
                    Transform.RelativeX = BaseX - Pose.MinX;
                    Transform.RelativeY = BaseY - Pose.MinY;
                    Candidate.Transforms.push_back(Transform);
                }

                if (!Geometry.FinalizeCandidate(AItems, AOptions, Candidate) ||Candidate.AreaSavingRatio < -CET_RECT_MAX_AREA_LOSS_RATIO){
                    continue;
                }

                Candidate.Score += 1800.0 + static_cast<double>(Candidate.OriginalIndices.size()) * 100.0;
                AOutCandidate = std::move(Candidate);
                return true;
            }

            return false;
        }

    }
}
