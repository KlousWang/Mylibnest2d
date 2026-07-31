#include "pch.h"
#include "Nest2D_TriangleClusterBuilder.h"
#include"Nest2D_ClusterGeometryHelper.h"
#include "Nest2D_ClusterMathUtils.h"
#include"Nest2D_RotationUtils.h"
#include"NestUtils.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <set>
#include <vector>

using namespace ClipperLib;
using namespace libnest2d;
namespace ET {
    namespace NEST2DMANAGERLIB {
        namespace {
            constexpr std::size_t CET_TRIANGLE_MAX_CLUSTER_CHILDREN = 32;
            constexpr std::size_t CET_GENERAL_TRIANGLE_MAX_CLUSTER_CHILDREN = 32;
            constexpr int CET_TRIANGLE_PAIR_PITCH_SEARCH_STEPS = 20;
        }

        CetTriangleClusterBuilder::CetTriangleClusterBuilder() : CetCoreObject()
        {
        }
        CetTriangleClusterBuilder::~CetTriangleClusterBuilder()
        {
        }
        bool CetTriangleClusterBuilder::TryMakeRightTrianglePair(const CetTNestItemVector& AOriginalItems, int AAIndex, int ABIndex, const TetNestOptions& AOptions, TetClusterBuildResult& AResult)
        {
            const auto& ItemA = AOriginalItems[AAIndex];
            const auto& ItemB = AOriginalItems[ABIndex];

            double WA = _GetItemWidth(ItemA);
            double HA = _GetItemHeight(ItemA);
            double WB = _GetItemWidth(ItemB);
            double HB = _GetItemHeight(ItemB);

            double AreaA = std::abs(static_cast<double>(ItemA.area()));
            double AreaB = std::abs(static_cast<double>(ItemB.area()));

            bool RightA = _IsRightTriangleLike(ItemA);
            bool RightB = _IsRightTriangleLike(ItemB);

            if (!RightA || !RightB){
                std::cout << "[CLUSTER][REJECT] not right triangle: "
                    << AAIndex << " + " << ABIndex
                    << ", A(W,H,Area)=(" << WA << "," << HA << "," << AreaA << ")"
                    << ", B(W,H,Area)=(" << WB << "," << HB << "," << AreaB << ")"
                    << ", A ratio=" << (WA * HA > 0.0 ? AreaA * 2.0 / (WA * HA) : 0.0)
                    << ", B ratio=" << (WB * HB > 0.0 ? AreaB * 2.0 / (WB * HB) : 0.0)
                    << std::endl;

                return false;
            }

            if (!_IsSameSizeTrianglePair(ItemA, ItemB)){
                std::cout << "[CLUSTER][REJECT] size mismatch: " << AAIndex << " + " << ABIndex << ", A(W,H)=(" << WA << "," << HA << ")" << ", B(W,H)=(" << WB << "," << HB << ")" << std::endl;

                return false;
            }
            double W = WA;
            double H = HA;
            if (W <= 0.0 || H <= 0.0){
                return false;
            }

            double AllowedHalfTurn = 0.0;

            if (!CetRotationUtils::SnapToAllowedRotation(CET_CLUSTER_PI, AOptions.Rotations, AllowedHalfTurn, 1e-9)){
                return false;
            }

            double InternalSpacing = static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing));
            double AxisGap = _CalcTrianglePairAxisGap(W, H, InternalSpacing);
            double ClusterW = W + AxisGap;
            double ClusterH = H + AxisGap;

            double BinW = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth));
            double BinH = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinHeight));
            if (ClusterW > BinW || ClusterH > BinH){
                std::cout << "[CLUSTER][REJECT] cluster bigger than bin: "
                    << AAIndex << " + " << ABIndex
                    << ", W = " << W
                    << ", H = " << H
                    << ", InternalSpacing = " << InternalSpacing
                    << ", AxisGap = " << AxisGap
                    << ", ClusterW = " << ClusterW
                    << ", ClusterH = " << ClusterH
                    << ", BinW = " << BinW
                    << ", BinH = " << BinH
                    << std::endl;

                return false;
            }
            auto ClusterItem = _MakeRectangleNestItem(ClusterW, ClusterH);
            const int PackedIndex = static_cast<int>(AResult.NestItems.size());
            AResult.NestItems.push_back(std::move(ClusterItem));

            TetMetaItem Meta;
            Meta.PackedItemIndex = PackedIndex;
            Meta.IsCluster = true;
            Meta.ClusterType = "RightTrianglePair";

            TetItemTransform TransformA;
            TransformA.OriginalId = AAIndex;
            TransformA.RelativeX = 0.0;
            TransformA.RelativeY = 0.0;
            TransformA.RelativeRotation = 0.0;
            Meta.TransformData.push_back(TransformA);

            TetItemTransform TransformB;
            TransformB.OriginalId = ABIndex;
            TransformB.RelativeX = ClusterW;
            TransformB.RelativeY = ClusterH;
            TransformB.RelativeRotation = AllowedHalfTurn;
            Meta.TransformData.push_back(TransformB);

            AResult.MetaItems.push_back(Meta);

            std::cout << "[CLUSTER] RightTrianglePair created: "
                << AAIndex << " + " << ABIndex
                << ", W = " << W
                << ", H = " << H
                << ", InternalSpacing = " << InternalSpacing
                << ", AxisGap = " << AxisGap
                << ", ClusterW = " << ClusterW
                << ", ClusterH = " << ClusterH
                << ", PackedIndex = " << PackedIndex
                << std::endl;

            return true;
        }
        void CetTriangleClusterBuilder::BuildCandidates(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, std::vector<TetClusterCandidate>& AOutCandidates) 
        {
            if (AOriginalItems.size() != AFeatures.size()){
                std::cout << "[TRIANGLE][ERROR] Feature count mismatch." << std::endl;
                return;
            }

            if (AIndices.size() < 2){ return; }

            const std::size_t OldCandidateCount = AOutCandidates.size();
            std::set<int> RightRectangleHandledIndices;
            _BuildRightTriangleRectangleClusterCandidates(AOriginalItems, AFeatures, AIndices, AOptions, AOutCandidates, RightRectangleHandledIndices);

            std::vector<std::vector<int>> Groups;
            for (int Index : AIndices){
                if (Index < 0 || Index >= static_cast<int>(AFeatures.size()) ||RightRectangleHandledIndices.find(Index) != RightRectangleHandledIndices.end()){
                    continue;
                }

                const TetShapeFeature& Feature = AFeatures[Index];
                if (Feature.ShapeType != MetShapeType::TriangleLike || Feature.HasHoles){
                    continue;
                }

                bool AddedToGroup = false;
                for (std::vector<int>& Group : Groups){
                    if (!Group.empty() && _AreCongruentTriangles(AFeatures[Group.front()], Feature)){
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
                _BuildAnyTriangleClusterCandidates(AOriginalItems, AFeatures, Group, AOptions, AOutCandidates);
            }

            std::cout << "[TRIANGLE][BUILD CANDIDATES] IndexCount=" << AIndices.size()
                << ", GroupCount=" << Groups.size()
                << ", RightRectangleHandled=" << RightRectangleHandledIndices.size()
                << ", NewCandidateCount=" << AOutCandidates.size() - OldCandidateCount << std::endl;

        }
        bool CetTriangleClusterBuilder::_IsRightTriangleLike(const CetNestItem& AItem)
        {
            double W = _GetItemWidth(AItem);
            double H = _GetItemHeight(AItem);
            if (W <= 0.0 || H <= 0.0){
                return false;
            }
            double BoxArea = std::abs(W * H);
            double ItemArea = std::abs(static_cast<double>(AItem.area()));
            if (ItemArea <= 0.0 || BoxArea <= 0.0){
                return false;
            }
            double Ratio = ItemArea * 2.0 / BoxArea;
            
            return std::abs(Ratio - 1.0) <= 0.08;
        }
        bool CetTriangleClusterBuilder::_IsSameSizeTrianglePair(const CetNestItem& AItem, const CetNestItem& ABItem)
        {
            double WA = _GetItemWidth(AItem);
            double HA = _GetItemHeight(AItem);
            double WB = _GetItemWidth(ABItem);
            double HB = _GetItemHeight(ABItem);
            bool SameDirection = _NearlyEqual(WA, WB, 0.05) && _NearlyEqual(HA, HB, 0.05);
            bool SwappedDirection = _NearlyEqual(WA, HB, 0.05) && _NearlyEqual(HA, WB, 0.05);
            return SameDirection || SwappedDirection;
        }
        bool CetTriangleClusterBuilder::_NearlyEqual(double A, double AB, double ARelTol)
        {
            return CetClusterMathUtils::NearlyEqual(A, AB, ARelTol);
        }  
        double CetTriangleClusterBuilder::_GetItemWidth(const CetNestItem& AItem)
        {
            return static_cast<double>(AItem.boundingBox().width());
        }
        double CetTriangleClusterBuilder::_GetItemHeight(const CetNestItem& AItem)
        {
            return static_cast<double>(AItem.boundingBox().height());
        }
        double CetTriangleClusterBuilder::_CalcTrianglePairAxisGap(double AW, double AH, double ASpacing)
        {
            if (AW <= 0.0 || AH <= 0.0 || ASpacing <= 0.0){
                return 0.0;
            }
            double AxisGap = ASpacing * std::sqrt(AW * AW + AH * AH) / (AW + AH);
            
            return std::ceil(AxisGap);
        }
        CetNestItem CetTriangleClusterBuilder::_MakeRectangleNestItem(double AWidth, double AHeight)
        {
            using namespace libnest2d;
            Path outerPoints;
            outerPoints.reserve(4);
            outerPoints.push_back(Point(0, 0));
            outerPoints.push_back(Point(static_cast<ClipperLib::cInt>(AWidth), 0));
            outerPoints.push_back(Point(static_cast<ClipperLib::cInt>(AWidth), static_cast<ClipperLib::cInt>(AHeight)));
            outerPoints.push_back(Point(0, static_cast<ClipperLib::cInt>(AHeight)));
            if (ClipperLib::Orientation(outerPoints) == false){
                std::reverse(outerPoints.begin(), outerPoints.end());
            }
            Paths holes;
            CetPolygonImpl poly(std::move(outerPoints), std::move(holes));
            return CetTNestItemVector::value_type(std::move(poly));
        }
        bool CetTriangleClusterBuilder::_BuildRightTrianglePairCandidate(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, int AAIndex, int ABIndex, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate)
        {
            if (AAIndex < 0 || ABIndex < 0 ||AAIndex >= static_cast<int>(AFeatures.size()) ||ABIndex >= static_cast<int>(AFeatures.size()) ||AAIndex == ABIndex){
                return false;
            }

            const TetShapeFeature& FA = AFeatures[AAIndex];
            const TetShapeFeature& FB = AFeatures[ABIndex];

            if (FA.TriangleAngleType != MetTriangleAngleType::Right ||FB.TriangleAngleType != MetTriangleAngleType::Right){
                return false;
            }

            if (!_AreCongruentTriangles(FA, FB)){
                return false;
            }

            double AllowedHalfTurn = 0.0;

            if (!CetRotationUtils::SnapToAllowedRotation(CET_CLUSTER_PI, AOptions.Rotations, AllowedHalfTurn, 1e-9)){
                return false;
            }

            const double Gap = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));

            AOutCandidate = TetClusterCandidate{};
            AOutCandidate.BuilderName = "TriangleBuilder";
            AOutCandidate.ClusterType = "RightTrianglePair";
            AOutCandidate.OriginalIndices = { AAIndex, ABIndex };

            TetItemTransform TA;
            TA.OriginalId = AAIndex;
            TA.RelativeX = -FA.MinX;
            TA.RelativeY = -FA.MinY;

            TetItemTransform TB;
            TB.OriginalId = ABIndex;
            TB.RelativeRotation = AllowedHalfTurn;
            
            TB.RelativeX = FA.Width + Gap - FB.MinX;
            TB.RelativeY = FA.Height + Gap - FB.MinY;

            AOutCandidate.Transforms = { TA, TB };
            AOutCandidate.Confidence = 1.0;

            CetClusterGeometryHelper Geometry;
            return Geometry.FinalizeCandidate(AOriginalItems, AOptions, AOutCandidate);
           
        }
        void CetTriangleClusterBuilder::_BuildRightTriangleRectangleClusterCandidates(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, std::vector<TetClusterCandidate>& AOutCandidates, std::set<int>& AOutHandledIndices)
        {
            std::vector<std::vector<int>> Groups;
            for (int Index : AIndices){
                if (Index < 0 || Index >= static_cast<int>(AFeatures.size())){
                    continue;
                }

                const TetShapeFeature& Feature = AFeatures[Index];
                if (Feature.ShapeType != MetShapeType::TriangleLike || Feature.TriangleAngleType != MetTriangleAngleType::Right || Feature.HasHoles){
                    continue;
                }

                bool AddedToGroup = false;
                for (std::vector<int>& Group : Groups){
                    if (!Group.empty() && _AreCongruentTriangles(AFeatures[Group.front()], Feature)){
                        Group.push_back(Index);
                        AddedToGroup = true;
                        break;
                    }
                }

                if (!AddedToGroup){
                    Groups.push_back({ Index });
                }
            }

            const std::size_t OldCandidateCount = AOutCandidates.size();
            for (const std::vector<int>& Group : Groups){
                if (Group.size() < 2){
                    continue;
                }

                CetClusterGeometryHelper Geometry;
                const std::size_t MaxChildCount = std::min(Group.size() - (Group.size() % 2),CET_TRIANGLE_MAX_CLUSTER_CHILDREN);
                std::size_t PreferredChildCount = 0;
                TetClusterCandidate FirstCandidate;
                for (std::size_t TrialCount = MaxChildCount; TrialCount >= 2; TrialCount -= 2){
                    std::vector<int> ClusterIndices(Group.begin(),Group.begin() + static_cast<std::vector<int>::difference_type>(TrialCount));

                    TetClusterCandidate Candidate;
                    if (!_BuildRightTriangleRectangleClusterCandidate(AOriginalItems, AFeatures, ClusterIndices, AOptions, Candidate)){
                        continue;
                    }

                    const std::size_t RequiredCopies = std::min(CET_CLUSTER_TARGET_COPIES_PER_BOARD,Group.size() / TrialCount);
                    if (!Geometry.CanPlaceCandidateCopiesOnBoard(Candidate, AOptions, RequiredCopies)){
                        continue;
                    }

                    PreferredChildCount = TrialCount;
                    FirstCandidate = std::move(Candidate);
                    break;
                }

                if (PreferredChildCount < 2){
                    continue;
                }

                std::size_t GroupOffset = 0;
                while (GroupOffset + 1 < Group.size()){
                    const std::size_t RemainingCount = Group.size() - GroupOffset;
                    std::size_t TrialCount = std::min(RemainingCount - (RemainingCount % 2),PreferredChildCount);
                    bool BuiltChunk = false;

                    while (TrialCount >= 2){
                        std::vector<int> ClusterIndices(Group.begin() + static_cast<std::vector<int>::difference_type>(GroupOffset),Group.begin() + static_cast<std::vector<int>::difference_type>(GroupOffset + TrialCount));

                        TetClusterCandidate Candidate;
                        const bool ReuseFirstCandidate = GroupOffset == 0 && TrialCount == PreferredChildCount;
                        if (ReuseFirstCandidate || _BuildRightTriangleRectangleClusterCandidate(AOriginalItems, AFeatures, ClusterIndices, AOptions, Candidate)){
                            if (ReuseFirstCandidate){
                                Candidate = std::move(FirstCandidate);
                            }
                            for (int OriginalIndex : Candidate.OriginalIndices){
                                AOutHandledIndices.insert(OriginalIndex);
                            }

                            AOutCandidates.push_back(std::move(Candidate));
                            std::cout << "[TRIANGLE][RECTANGLE][CANDIDATE] GroupSize=" << Group.size()
                                << ", Offset=" << GroupOffset
                                << ", UsedCount=" << AOutCandidates.back().OriginalIndices.size()
                                << ", Type=" << AOutCandidates.back().ClusterType
                                << ", Score=" << AOutCandidates.back().Score << std::endl;

                            GroupOffset += TrialCount;
                            BuiltChunk = true;
                            break;
                        }

                        TrialCount -= 2;
                    }

                    if (!BuiltChunk){
                        std::cout << "[TRIANGLE][RECTANGLE][REJECT] GroupSize=" << Group.size() << ", Offset=" << GroupOffset << ", RemainingCount=" << RemainingCount << std::endl;
                        break;
                    }
                }
            }

            std::cout << "[TRIANGLE][RECTANGLE][BUILD] GroupCount=" << Groups.size() << ", NewCandidateCount=" << AOutCandidates.size() - OldCandidateCount << ", HandledCount=" << AOutHandledIndices.size() << std::endl;
        }

        bool CetTriangleClusterBuilder::_BuildRightTriangleRectangleClusterCandidate(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate)
        {
            AOutCandidate = TetClusterCandidate{};
            if (AOriginalItems.size() != AFeatures.size() || AIndices.size() < 2 || (AIndices.size() % 2) != 0){
                return false;
            }

            const int FirstIndex = AIndices.front();
            if (FirstIndex < 0 || FirstIndex >= static_cast<int>(AFeatures.size())){
                return false;
            }

            const TetShapeFeature& BaseFeature = AFeatures[FirstIndex];
            if (BaseFeature.ShapeType != MetShapeType::TriangleLike || BaseFeature.TriangleAngleType != MetTriangleAngleType::Right || BaseFeature.HasHoles){
                return false;
            }

            for (int Index : AIndices){
                if (Index < 0 || Index >= static_cast<int>(AFeatures.size())){
                    return false;
                }
                const TetShapeFeature& Feature = AFeatures[Index];
                if (Feature.ShapeType != MetShapeType::TriangleLike || Feature.TriangleAngleType != MetTriangleAngleType::Right || Feature.HasHoles){
                    return false;
                }
                if (!_AreCongruentTriangles(BaseFeature, Feature)){
                    return false;
                }
            }

            double HalfTurn = 0.0;
            if (!CetRotationUtils::SnapToAllowedRotation(CET_CLUSTER_PI, AOptions.Rotations, HalfTurn, 1e-9)){
                return false;
            }

            CetClusterGeometryHelper Geometry;
            double BaseMinX = 0.0;
            double BaseMinY = 0.0;
            double BaseMaxX = 0.0;
            double BaseMaxY = 0.0;
            if (!Geometry.GetBounds(Geometry.GetIdentityContour(AOriginalItems[FirstIndex]), BaseMinX, BaseMinY, BaseMaxX, BaseMaxY)){
                return false;
            }

            const double BaseWidth = BaseMaxX - BaseMinX;
            const double BaseHeight = BaseMaxY - BaseMinY;
            if (BaseWidth <= 0.0 || BaseHeight <= 0.0){
                return false;
            }

            const double RequiredGap = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));
            const double SafetyGap = RequiredGap > 0.0 ? std::max(10.0, RequiredGap * 0.001) : 0.0;
            const double ValidationGap = RequiredGap + SafetyGap;
            const double AxisGap = _CalcTrianglePairAxisGap(BaseWidth, BaseHeight, ValidationGap);
            const double CellWidth = BaseWidth + AxisGap;
            const double CellHeight = BaseHeight + AxisGap;
            const double CellGap = ValidationGap;
            if (CellWidth <= 0.0 || CellHeight <= 0.0){
                return false;
            }

            const int PairCount = static_cast<int>(AIndices.size() / 2);
            struct TetRightTriangleRectangleLayout
            {
                int Rows = 0;
                int Cols = 0;
                double Width = 0.0;
                double Height = 0.0;
                double Area = 0.0;
                double AspectPenalty = 0.0;
            };

            std::vector<TetRightTriangleRectangleLayout> Layouts;
            Layouts.reserve(static_cast<std::size_t>(PairCount));
            const double BinWidth = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth));
            const double BinHeight = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinHeight));
            const bool QuarterTurnAllowed = CetRotationUtils::IsAllowedRotation(CET_CLUSTER_HALF_PI, AOptions.Rotations, 1e-9);

            for (int Rows = 1; Rows <= PairCount; ++Rows){
                const int Cols = (PairCount + Rows - 1) / Rows;
                const double Width = static_cast<double>(Cols) * CellWidth + static_cast<double>(std::max(0, Cols - 1)) * CellGap;
                const double Height = static_cast<double>(Rows) * CellHeight + static_cast<double>(std::max(0, Rows - 1)) * CellGap;
                if (Width <= 0.0 || Height <= 0.0){
                    continue;
                }

                const bool FitsNormally = Width <= BinWidth && Height <= BinHeight;
                const bool FitsRotated = QuarterTurnAllowed && Height <= BinWidth && Width <= BinHeight;
                if (!FitsNormally && !FitsRotated){
                    continue;
                }

                const double LongSide = std::max(Width, Height);
                const double ShortSide = std::min(Width, Height);
                TetRightTriangleRectangleLayout Layout;
                Layout.Rows = Rows;
                Layout.Cols = Cols;
                Layout.Width = Width;
                Layout.Height = Height;
                Layout.Area = Width * Height;
                Layout.AspectPenalty = LongSide > 0.0 ? LongSide / std::max(1.0, ShortSide) : std::numeric_limits<double>::infinity();
                Layouts.push_back(Layout);
            }

            if (Layouts.empty()){
                return false;
            }

            std::stable_sort(Layouts.begin(), Layouts.end(), [](const TetRightTriangleRectangleLayout& A, const TetRightTriangleRectangleLayout& AB) {
                if (std::abs(A.Area - AB.Area) > 1e-6){
                    return A.Area < AB.Area;
                }
                if (std::abs(A.AspectPenalty - AB.AspectPenalty) > 1e-6){
                    return A.AspectPenalty < AB.AspectPenalty;
                }
                return A.Rows < AB.Rows;
                });

            bool HasBest = false;
            TetClusterCandidate BestCandidate;
            double BestBoxArea = std::numeric_limits<double>::infinity();
            const std::size_t MaxLayoutChecks = std::min<std::size_t>(Layouts.size(), 6);
            for (std::size_t LayoutIndex = 0; LayoutIndex < MaxLayoutChecks; ++LayoutIndex){
                const TetRightTriangleRectangleLayout& Layout = Layouts[LayoutIndex];
                TetClusterCandidate Candidate;
                if (!_BuildRightTriangleRectangleLayoutCandidate(AOriginalItems, AFeatures, AIndices, AOptions, Layout.Rows, Layout.Cols, CellWidth, CellHeight, AxisGap, CellGap, HalfTurn, Candidate)){
                    continue;
                }

                Candidate.Score = _CalculateRightTriangleRectangleScore(Candidate, PairCount, Layout.Rows, Layout.Cols);
                const double CandidateBoxArea = Candidate.ClusterWidth * Candidate.ClusterHeight;
                const bool SmallerBox = CandidateBoxArea + 1.0 < BestBoxArea;
                const bool SameBoxBetterScore = std::abs(CandidateBoxArea - BestBoxArea) <= 1.0 && Candidate.Score > BestCandidate.Score;
                if (!HasBest || SmallerBox || SameBoxBetterScore){
                    HasBest = true;
                    BestBoxArea = CandidateBoxArea;
                    BestCandidate = std::move(Candidate);
                }
            }

            if (!HasBest){
                return false;
            }

            AOutCandidate = std::move(BestCandidate);
            return true;
        }

        bool CetTriangleClusterBuilder::_BuildRightTriangleRectangleLayoutCandidate(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, int ACellRows, int ACellCols, double ACellWidth, double ACellHeight, double AAxisGap, double ACellGap, double AHalfTurn, TetClusterCandidate& AOutCandidate)
        {
            (void)AFeatures;
            AOutCandidate = TetClusterCandidate{};
            if (ACellRows <= 0 || ACellCols <= 0 || ACellWidth <= 0.0 || ACellHeight <= 0.0 || AIndices.size() < 2 || (AIndices.size() % 2) != 0){
                return false;
            }

            CetClusterGeometryHelper Geometry;
            TetClusterCandidate Candidate;
            Candidate.BuilderName = "TriangleBuilder";
            Candidate.ClusterType = AIndices.size() == 2
                ? "RightTriangleRectanglePair"
                : "RightTriangleRectangleGrid_" + std::to_string(AIndices.size()) + "_R" + std::to_string(ACellRows);
            Candidate.OriginalIndices = AIndices;
            Candidate.Confidence = 1.25;
            Candidate.Transforms.reserve(AIndices.size());

            const int PairCount = static_cast<int>(AIndices.size() / 2);
            for (int PairIndex = 0; PairIndex < PairCount; ++PairIndex){
                const int AIndex = AIndices[static_cast<std::size_t>(PairIndex) * 2];
                const int BIndex = AIndices[static_cast<std::size_t>(PairIndex) * 2 + 1];
                if (AIndex < 0 || BIndex < 0 || AIndex >= static_cast<int>(AOriginalItems.size()) || BIndex >= static_cast<int>(AOriginalItems.size())){
                    return false;
                }

                const int Row = PairIndex / ACellCols;
                const int Col = PairIndex % ACellCols;
                if (Row >= ACellRows){
                    return false;
                }

                const double BaseX = static_cast<double>(Col) * (ACellWidth + ACellGap);
                const double BaseY = static_cast<double>(Row) * (ACellHeight + ACellGap);

                double AMinX = 0.0;
                double AMinY = 0.0;
                double AMaxX = 0.0;
                double AMaxY = 0.0;
                if (!Geometry.GetBounds(Geometry.GetIdentityContour(AOriginalItems[AIndex]), AMinX, AMinY, AMaxX, AMaxY)){
                    return false;
                }

                const CetPath RotatedB = Geometry.TransformContour(Geometry.GetIdentityContour(AOriginalItems[BIndex]), AHalfTurn, 0.0, 0.0);
                double BMinX = 0.0;
                double BMinY = 0.0;
                double BMaxX = 0.0;
                double BMaxY = 0.0;
                if (!Geometry.GetBounds(RotatedB, BMinX, BMinY, BMaxX, BMaxY)){
                    return false;
                }

                TetItemTransform TransformA;
                TransformA.OriginalId = AIndex;
                TransformA.RelativeRotation = 0.0;
                TransformA.RelativeX = BaseX - AMinX;
                TransformA.RelativeY = BaseY - AMinY;

                TetItemTransform TransformB;
                TransformB.OriginalId = BIndex;
                TransformB.RelativeRotation = AHalfTurn;
                TransformB.RelativeX = BaseX + AAxisGap - BMinX;
                TransformB.RelativeY = BaseY + AAxisGap - BMinY;

                Candidate.Transforms.push_back(TransformA);
                Candidate.Transforms.push_back(TransformB);
            }

            if (!Geometry.FinalizeCandidate(AOriginalItems, AOptions, Candidate)){
                return false;
            }

            AOutCandidate = std::move(Candidate);
            return true;
        }

        double CetTriangleClusterBuilder::_CalculateRightTriangleRectangleScore(const TetClusterCandidate& ACandidate, int APairCount, int ACellRows, int ACellCols)
        {
            if (!ACandidate.Valid || ACandidate.OriginalIndices.empty() || ACandidate.ProxyArea <= 0.0 || ACandidate.ClusterWidth <= 0.0 || ACandidate.ClusterHeight <= 0.0 || APairCount <= 0){
                return -std::numeric_limits<double>::infinity();
            }

            const double LongSide = std::max(ACandidate.ClusterWidth, ACandidate.ClusterHeight);
            const double ShortSide = std::min(ACandidate.ClusterWidth, ACandidate.ClusterHeight);
            const double CompactRatio = LongSide > 0.0 ? ShortSide / LongSide : 0.0;
            const double GridBalance = ACellRows > 0 && ACellCols > 0
                ? 1.0 / static_cast<double>(1 + std::abs(ACellRows - ACellCols))
                : 0.0;
            const double PerimeterPenalty = (ACandidate.ClusterWidth + ACandidate.ClusterHeight) * 0.000001;

            return 5000.0
                + ACandidate.FillRatio * 3000.0
                + ACandidate.AreaSavingRatio * 5000.0
                + static_cast<double>(ACandidate.OriginalIndices.size()) * 140.0
                + static_cast<double>(APairCount) * 180.0
                + CompactRatio * 20.0
                + GridBalance * 20.0
                + ACandidate.Confidence * 100.0
                - ACandidate.ProxyWasteRatio * 250.0
                - PerimeterPenalty;
        }
        bool CetTriangleClusterBuilder::_AreCongruentTriangles(const TetShapeFeature& AA, const TetShapeFeature& AB)
        {
            if (AA.ShapeType != MetShapeType::TriangleLike ||AB.ShapeType != MetShapeType::TriangleLike){
                return false;
            }
            for (int i = 0; i < 3; ++i) if (!_NearlyEqual(AA.TriangleSides[i], AB.TriangleSides[i], 0.03)) return false;
            return true;
        }
        void CetTriangleClusterBuilder::_BuildAnyTriangleClusterCandidates(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, std::vector<TetClusterCandidate>& AOutCandidates)
        {
            std::vector<int> RemainingIndices = AIndices;
            std::sort(RemainingIndices.begin(), RemainingIndices.end());
            RemainingIndices.erase(std::unique(RemainingIndices.begin(), RemainingIndices.end()), RemainingIndices.end());

            const std::size_t MaxPairCount = std::min((RemainingIndices.size() - (RemainingIndices.size() % 2)) / 2,CET_GENERAL_TRIANGLE_MAX_CLUSTER_CHILDREN / 2);
            if (MaxPairCount == 0){
                return;
            }

            CetClusterGeometryHelper Geometry;
            std::size_t PreferredPairCount = 0;
            TetClusterCandidate FirstCandidate;
            std::size_t FallbackPairCount = 0;
            TetClusterCandidate FallbackCandidate;
            for (std::size_t TrialPairCount = MaxPairCount; TrialPairCount >= 1; --TrialPairCount){
                const std::size_t ChildCount = TrialPairCount * 2;
                std::vector<int> ClusterIndices(RemainingIndices.begin(),RemainingIndices.begin() + static_cast<std::vector<int>::difference_type>(ChildCount));

                TetClusterCandidate Candidate;
                if (!_BuildAnyTriangleClusterCandidate(AOriginalItems, AFeatures, ClusterIndices, AOptions, Candidate)){
                    continue;
                }

                const std::size_t FullChunkCopies = RemainingIndices.size() / ChildCount;
                const std::size_t PracticalCopies = std::min(CET_CLUSTER_TARGET_COPIES_PER_BOARD,std::max(FullChunkCopies, RemainingIndices.size() > 2 ? std::size_t{ 2 } : std::size_t{ 1 }));
                if (FallbackPairCount == 0 && Geometry.CanPlaceCandidateCopiesOnBoard(Candidate, AOptions, PracticalCopies)){
                    FallbackPairCount = TrialPairCount;
                    FallbackCandidate = Candidate;
                }

                const std::size_t AllChunkCopies = (RemainingIndices.size() + ChildCount - 1) / ChildCount;
                if (Geometry.CanPlaceCandidateCopiesOnBoard(Candidate, AOptions, AllChunkCopies)){
                    PreferredPairCount = TrialPairCount;
                    FirstCandidate = std::move(Candidate);
                    break;
                }
            }

            if (PreferredPairCount == 0 && FallbackPairCount > 0){
                PreferredPairCount = FallbackPairCount;
                FirstCandidate = std::move(FallbackCandidate);
            }

            if (PreferredPairCount == 0){
                return;
            }

            std::size_t GroupOffset = 0;
            while (GroupOffset + 1 < RemainingIndices.size()){
                const std::size_t RemainingCount = RemainingIndices.size() - GroupOffset;
                std::size_t TrialPairCount = std::min(PreferredPairCount, RemainingCount / 2);
                TetClusterCandidate BestCandidate;
                std::size_t BestPairCount = 0;

                while (TrialPairCount >= 1){
                    const std::size_t ChildCount = TrialPairCount * 2;
                    std::vector<int> ClusterIndices(RemainingIndices.begin() + static_cast<std::vector<int>::difference_type>(GroupOffset),RemainingIndices.begin() + static_cast<std::vector<int>::difference_type>(GroupOffset + ChildCount));

                    if (GroupOffset == 0 && TrialPairCount == PreferredPairCount){
                        BestCandidate = std::move(FirstCandidate);
                        BestPairCount = TrialPairCount;
                        break;
                    }

                    TetClusterCandidate Candidate;
                    if (_BuildAnyTriangleClusterCandidate(AOriginalItems, AFeatures, ClusterIndices, AOptions, Candidate)){
                        BestCandidate = std::move(Candidate);
                        BestPairCount = TrialPairCount;
                        break;
                    }

                    --TrialPairCount;
                }

                if (BestPairCount == 0){
                    break;
                }

                GroupOffset += BestPairCount * 2;
                std::cout << "[TRIANGLE][GENERAL][CANDIDATE] ChildCount=" << BestCandidate.OriginalIndices.size()
                    << ", Type=" << BestCandidate.ClusterType
                    << ", Width=" << BestCandidate.ClusterWidth
                    << ", Height=" << BestCandidate.ClusterHeight
                    << ", Score=" << BestCandidate.Score << std::endl;
                AOutCandidates.push_back(std::move(BestCandidate));
            }
        }
        bool CetTriangleClusterBuilder::_BuildAnyTriangleClusterCandidate(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate)
        {
            AOutCandidate = TetClusterCandidate{};
            if (AOriginalItems.size() != AFeatures.size() || AIndices.size() < 2 ||(AIndices.size() % 2) != 0 || AIndices.size() > CET_GENERAL_TRIANGLE_MAX_CLUSTER_CHILDREN){
                return false;
            }

            const int FirstIndex = AIndices.front();
            if (FirstIndex < 0 || FirstIndex >= static_cast<int>(AFeatures.size())){
                return false;
            }

            const TetShapeFeature& BaseFeature = AFeatures[FirstIndex];
            for (int Index : AIndices){
                if (Index < 0 || Index >= static_cast<int>(AFeatures.size()) ||AFeatures[Index].HasHoles || !_AreCongruentTriangles(BaseFeature, AFeatures[Index])){
                    return false;
                }
            }

            std::vector<TetClusterCandidate> PairCandidates;
            PairCandidates.reserve(AIndices.size() / 2);
            double CellWidth = 0.0;
            double CellHeight = 0.0;
            TetClusterCandidate ReusablePairCandidate;
            int ReusableAIndex = -1;
            int ReusableBIndex = -1;
            auto HaveSameContour = [](const CetPath& A, const CetPath& AB) {
                if (A.size() != AB.size()){
                    return false;
                }
                for (std::size_t PointIndex = 0; PointIndex < A.size(); ++PointIndex){
                    if (A[PointIndex].X != AB[PointIndex].X || A[PointIndex].Y != AB[PointIndex].Y){
                        return false;
                    }
                }
                return true;
                };

            for (std::size_t PairOffset = 0; PairOffset < AIndices.size(); PairOffset += 2){
                const int AIndex = AIndices[PairOffset];
                const int BIndex = AIndices[PairOffset + 1];
                TetClusterCandidate PairCandidate;
                const bool CanReusePair = ReusableAIndex >= 0 && ReusableBIndex >= 0 &&
                    HaveSameContour(AFeatures[AIndex].NormalizedContour, AFeatures[ReusableAIndex].NormalizedContour) &&
                    HaveSameContour(AFeatures[BIndex].NormalizedContour, AFeatures[ReusableBIndex].NormalizedContour) &&
                    ReusablePairCandidate.Transforms.size() == 2;

                if (CanReusePair){
                    PairCandidate = ReusablePairCandidate;
                    PairCandidate.OriginalIndices = { AIndex, BIndex };
                    PairCandidate.Transforms[0].OriginalId = AIndex;
                    PairCandidate.Transforms[1].OriginalId = BIndex;
                }
                else {
                    if (!_BuildAnyTrianglePairCandidate(AOriginalItems,AFeatures,AIndex,BIndex,AOptions,PairCandidate)){
                        return false;
                    }

                    if (ReusableAIndex < 0){
                        ReusableAIndex = AIndex;
                        ReusableBIndex = BIndex;
                        ReusablePairCandidate = PairCandidate;
                    }
                }

                CellWidth = std::max(CellWidth, PairCandidate.ClusterWidth);
                CellHeight = std::max(CellHeight, PairCandidate.ClusterHeight);
                PairCandidates.push_back(std::move(PairCandidate));
            }

            if (CellWidth <= 0.0 || CellHeight <= 0.0){
                return false;
            }

            const double RequiredGap = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));
            const double SafetyGap = RequiredGap > 0.0 ? std::max(10.0, RequiredGap * 0.001) : 0.0;
            const double CellGap = RequiredGap + SafetyGap;
            const int PairCount = static_cast<int>(PairCandidates.size());
            const double BinWidth = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth));
            const double BinHeight = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinHeight));
            const bool QuarterTurnAllowed = CetRotationUtils::IsAllowedRotation(CET_CLUSTER_HALF_PI, AOptions.Rotations, 1e-9);
            CetClusterGeometryHelper Geometry;

            auto FindMinimumPairPitch = [&](bool AHorizontal, double AFallbackPitch) {
                if (PairCandidates.size() < 2 || AFallbackPitch <= 0.0){
                    return AFallbackPitch;
                }

                auto HasValidPitch = [&](double APitch) {
                    std::vector<TetItemTransform> ProbeTransforms = PairCandidates[0].Transforms;
                    ProbeTransforms.reserve(PairCandidates[0].Transforms.size() + PairCandidates[1].Transforms.size());
                    for (TetItemTransform Transform : PairCandidates[1].Transforms){
                        if (AHorizontal){
                            Transform.RelativeX += APitch;
                        }
                        else {
                            Transform.RelativeY += APitch;
                        }
                        ProbeTransforms.push_back(Transform);
                    }
                    return Geometry.HasValidTransformSpacing(AOriginalItems, AOptions, ProbeTransforms);
                    };

                if (!HasValidPitch(AFallbackPitch)){
                    return AFallbackPitch;
                }

                double InvalidPitch = 0.0;
                double ValidPitch = AFallbackPitch;
                for (int SearchIndex = 0; SearchIndex < CET_TRIANGLE_PAIR_PITCH_SEARCH_STEPS; ++SearchIndex){
                    const double TrialPitch = (InvalidPitch + ValidPitch) * 0.5;
                    if (HasValidPitch(TrialPitch)){
                        ValidPitch = TrialPitch;
                    }
                    else {
                        InvalidPitch = TrialPitch;
                    }
                }

                return std::min(AFallbackPitch, ValidPitch + SafetyGap);
                };

            const double PairPitchX = FindMinimumPairPitch(true, CellWidth + CellGap);
            const double PairPitchY = FindMinimumPairPitch(false, CellHeight + CellGap);

            struct TetGeneralTriangleLayout
            {
                int Rows = 0;
                int Cols = 0;
                double Width = 0.0;
                double Height = 0.0;
                double Area = 0.0;
                double AspectPenalty = 0.0;
            };

            std::vector<TetGeneralTriangleLayout> Layouts;
            Layouts.reserve(static_cast<std::size_t>(PairCount));
            for (int Rows = 1; Rows <= PairCount; ++Rows){
                const int Cols = (PairCount + Rows - 1) / Rows;
                const double Width = CellWidth + static_cast<double>(std::max(0, Cols - 1)) * PairPitchX;
                const double Height = CellHeight + static_cast<double>(std::max(0, Rows - 1)) * PairPitchY;
                const bool FitsNormally = Width <= BinWidth && Height <= BinHeight;
                const bool FitsRotated = QuarterTurnAllowed && Height <= BinWidth && Width <= BinHeight;
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

            std::stable_sort(Layouts.begin(), Layouts.end(), [](const TetGeneralTriangleLayout& A, const TetGeneralTriangleLayout& AB) {
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
                const TetGeneralTriangleLayout& Layout = Layouts[LayoutIndex];
                TetClusterCandidate Candidate;
                Candidate.BuilderName = "TriangleBuilder";
                Candidate.ClusterType = AIndices.size() == 2
                    ? "AnyTrianglePair"
                    : "AnyTriangleGrid_" + std::to_string(AIndices.size()) + "_R" + std::to_string(Layout.Rows);
                Candidate.OriginalIndices = AIndices;
                Candidate.Confidence = 1.0;
                Candidate.Transforms.reserve(AIndices.size());

                for (int PairIndex = 0; PairIndex < PairCount; ++PairIndex){
                    const TetClusterCandidate& PairCandidate = PairCandidates[static_cast<std::size_t>(PairIndex)];
                    const int Row = PairIndex / Layout.Cols;
                    const int Col = PairIndex % Layout.Cols;
                    const double BaseX = static_cast<double>(Col) * PairPitchX + (CellWidth - PairCandidate.ClusterWidth) * 0.5;
                    const double BaseY = static_cast<double>(Row) * PairPitchY + (CellHeight - PairCandidate.ClusterHeight) * 0.5;

                    for (const TetItemTransform& PairTransform : PairCandidate.Transforms){
                        TetItemTransform Transform = PairTransform;
                        Transform.RelativeX += BaseX;
                        Transform.RelativeY += BaseY;
                        Candidate.Transforms.push_back(Transform);
                    }
                }

                if (!Geometry.FinalizeCandidate(AOriginalItems, AOptions, Candidate, true)){
                    continue;
                }

                Candidate.Score += 2500.0 + static_cast<double>(Candidate.OriginalIndices.size()) * 120.0;
                AOutCandidate = std::move(Candidate);
                return true;
            }

            return false;
        }
        bool CetTriangleClusterBuilder::_BuildAnyTrianglePairCandidate(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, int AAIndex, int ABIndex, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate)
        {
            bool HasBest = false;
            TetClusterCandidate BestCandidate;
            



            for (int AEdgeIndex = 0; AEdgeIndex < 3; ++AEdgeIndex){
                for (int BEdgeIndex = 0; BEdgeIndex < 3; ++BEdgeIndex){
                    TetClusterCandidate Candidate;
                    if (!_TryBuildTriangleEdgePairCandidate(AOriginalItems, AFeatures, AAIndex, ABIndex, AEdgeIndex, BEdgeIndex, AOptions, Candidate)){
                        continue;
                    }
                    if (!HasBest || Candidate.Score > BestCandidate.Score){
                        HasBest = true;
                        BestCandidate = std::move(Candidate);
                    }
                }
            }
            if (!HasBest){
                TetClusterCandidate OppositeCandidate;
                if (_BuildOppositeTrianglePairCandidate(AOriginalItems, AFeatures, AAIndex, ABIndex, AOptions, OppositeCandidate)){
                    AOutCandidate = std::move(OppositeCandidate);
                    return true;
                }
                return false;
            }

            AOutCandidate = std::move(BestCandidate);
            return true;
        }
        bool CetTriangleClusterBuilder::_TryBuildTriangleEdgePairCandidate(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, int AAIndex, int ABIndex, int AEdgeIndex, int ABEdgeIndex, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate)
        {
            if (AAIndex < 0 || ABIndex < 0 || AAIndex == ABIndex ||
                AAIndex >= static_cast<int>(AOriginalItems.size()) || ABIndex >= static_cast<int>(AOriginalItems.size()) ||
                AAIndex >= static_cast<int>(AFeatures.size()) || ABIndex >= static_cast<int>(AFeatures.size())){
                return false;
            }
            const TetShapeFeature& FeatureA = AFeatures[AAIndex];
            const TetShapeFeature& FeatureB = AFeatures[ABIndex];
            if (FeatureA.ShapeType != MetShapeType::TriangleLike || FeatureB.ShapeType != MetShapeType::TriangleLike){
                return false;
            }
            if (!_AreCongruentTriangles(FeatureA, FeatureB)){
                return false;
            }

            CetClusterGeometryHelper Geometry;
            CetPath ContourA = Geometry.GetIdentityContour(AOriginalItems[AAIndex]);
            CetPath ContourB = Geometry.GetIdentityContour(AOriginalItems[ABIndex]);
            
            auto removeDuplicateStartEnd = [](CetPath& Apath) {
                if (Apath.size() > 3 && Apath.front().X == Apath.back().X && Apath.front().Y == Apath.back().Y){
                    Apath.pop_back();
                }
                };
            removeDuplicateStartEnd(ContourA);
            removeDuplicateStartEnd(ContourB);

            if (ContourA.size() != 3 || ContourB.size() != 3){
                return false;
            }
            TetTriangleEdgePose EdgeA;
            TetTriangleEdgePose EdgeB;
            if (!_GetTriangleEdgePose(ContourA, AEdgeIndex, EdgeA) || !_GetTriangleEdgePose(ContourB, ABEdgeIndex, EdgeB)){
                return false;
            }
            if (!_NearlyEqual(EdgeA.Length, EdgeB.Length, 0.03)){
                return false;
            }
            CetInpoint AThird;
            CetInpoint BThird;
            if (!_GetTriangleThirdPoint(ContourA, AEdgeIndex, AThird) || !_GetTriangleThirdPoint(ContourB, ABEdgeIndex, BThird)){
                return false;
            }

            const double ASide = _Cross(EdgeA.Start, EdgeA.End, AThird);
            if (std::abs(ASide) <= 1.0){
                return false;
            }

            
            const double TargetRotationB = EdgeA.Angle + CET_CLUSTER_PI - EdgeB.Angle;

            double RotationB = 0.0;

            if (!CetRotationUtils::SnapToAllowedRotation(TargetRotationB, AOptions.Rotations, RotationB, 0.0523598775598299)){
                return false;
            }

            double AMinX = 0.0, AMinY = 0.0, AMaxX = 0.0, AMaxY = 0.0;
            if (!Geometry.GetBounds(ContourA, AMinX, AMinY, AMaxX, AMaxY)){
                return false;
            }

            const double ATranslationX = -AMinX;
            const double ATranslationY = -AMinY;
            const double AStartX = static_cast<double>(EdgeA.Start.X) + ATranslationX;
            const double AStartY = static_cast<double>(EdgeA.Start.Y) + ATranslationY;
            const double AEndX = static_cast<double>(EdgeA.End.X) + ATranslationX;
            const double AEndY = static_cast<double>(EdgeA.End.Y) + ATranslationY;

            const double EdgeDX = AEndX - AStartX;
            const double EdgeDY = AEndY - AStartY;
            const double EdgeLen = std::sqrt(EdgeDX * EdgeDX + EdgeDY * EdgeDY);

            if (EdgeLen <= 0.0){
                return false;
            }

            const double UnitX = EdgeDX / EdgeLen;
            const double UnitY = EdgeDY / EdgeLen;
            const double RequiredGap = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));
            const double SafetyGap = RequiredGap > 0.0 ? std::max(10.0, RequiredGap * 0.001) : 0.0;
            const double Gap = RequiredGap + SafetyGap;

            const CetInpoint RotatedBStart = _RotatePoint(EdgeB.Start, RotationB);
            const CetInpoint RotatedBEnd = _RotatePoint(EdgeB.End, RotationB);
            const CetInpoint RotatedBThird = _RotatePoint(BThird, RotationB);

            const double AMidX = (AStartX + AEndX) * 0.5;
            const double AMidY = (AStartY + AEndY) * 0.5;
            const double BMidX = (static_cast<double>(RotatedBStart.X) + static_cast<double>(RotatedBEnd.X)) * 0.5;
            const double BMidY = (static_cast<double>(RotatedBStart.Y) + static_cast<double>(RotatedBEnd.Y)) * 0.5;

            std::vector<TetBaseOffset> BaseOffsets;
            BaseOffsets.push_back({ AMidX - BMidX, AMidY - BMidY });

            bool HasBest = false;
            TetClusterCandidate BestCandidate;
            int SideRejectCount = 0;
            int FinalizeRejectCount = 0;

            for (const auto& BaseOffset : BaseOffsets){
                
                const double BThirdBaseX = static_cast<double>(RotatedBThird.X) + BaseOffset.X;
                const double BThirdBaseY = static_cast<double>(RotatedBThird.Y) + BaseOffset.Y;
                const double BaseAPX = BThirdBaseX - AStartX;
                const double BaseAPY = BThirdBaseY - AStartY;
                const double BSideBase = EdgeDX * BaseAPY - EdgeDY * BaseAPX;

                
                if (ASide * BSideBase >= 0.0){
                    ++SideRejectCount;
                    continue;
                }

                



                const double OffsetSign = BSideBase > 0.0 ? 1.0 : -1.0;
                const double BTranslationX = BaseOffset.X + (-UnitY) * OffsetSign * Gap;
                const double BTranslationY = BaseOffset.Y + UnitX * OffsetSign * Gap;

                
                const double BThirdX = static_cast<double>(RotatedBThird.X) + BTranslationX;
                const double BThirdY = static_cast<double>(RotatedBThird.Y) + BTranslationY;
                const double APX = BThirdX - AStartX;
                const double APY = BThirdY - AStartY;
                const double BSide = EdgeDX * APY - EdgeDY * APX;

                if (ASide * BSide >= 0.0){
                    ++SideRejectCount;
                    continue;
                }

                TetClusterCandidate Candidate;
                Candidate.BuilderName = "TriangleBuilder";
                Candidate.ClusterType = "AnyTriangleEdgePair";
                Candidate.OriginalIndices = { AAIndex, ABIndex };

                TetItemTransform TransformA;
                TransformA.OriginalId = AAIndex;
                TransformA.RelativeX = ATranslationX;
                TransformA.RelativeY = ATranslationY;
                TransformA.RelativeRotation = 0.0;

                TetItemTransform TransformB;
                TransformB.OriginalId = ABIndex;
                TransformB.RelativeX = BTranslationX;
                TransformB.RelativeY = BTranslationY;
                TransformB.RelativeRotation = RotationB;

                Candidate.Transforms = { TransformA, TransformB };
                Candidate.Confidence = 0.90;

                if (!Geometry.FinalizeCandidate(AOriginalItems, AOptions, Candidate)){
                    ++FinalizeRejectCount;
                    continue;
                }

                const double LengthMatchRatio = std::min(EdgeA.Length, EdgeB.Length) / std::max(EdgeA.Length, EdgeB.Length);
                Candidate.Score += LengthMatchRatio * 50.0;

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
        bool CetTriangleClusterBuilder::_GetTriangleEdgePose(const CetPath& AContour, int AEdgeIndex, TetTriangleEdgePose& AOutEdge)
        {
            if (AContour.size() != 3){ return false; }
            if (AEdgeIndex < 0 || AEdgeIndex >= 3){ return false; }

            const int NextIndex = (AEdgeIndex + 1) % 3;

            AOutEdge = TetTriangleEdgePose{};
            AOutEdge.Start = AContour[AEdgeIndex];
            AOutEdge.End = AContour[NextIndex];

            const double DX = static_cast<double>(AOutEdge.End.X - AOutEdge.Start.X);
            const double DY = static_cast<double>(AOutEdge.End.Y - AOutEdge.Start.Y);
            AOutEdge.Length = std::sqrt(DX * DX + DY * DY);

            if (AOutEdge.Length <= 0.0){ return false; }

            AOutEdge.Angle = std::atan2(DY, DX);

            return true;
        }
        double CetTriangleClusterBuilder::_NormalizeAngle(double AAngle)
        {
            return CetRotationUtils::NormalizeAngle(AAngle);
        }
        CetInpoint CetTriangleClusterBuilder::_RotatePoint(const CetInpoint& APoint, double ARotation)
        {
            const double CosVal = std::cos(ARotation);
            const double SinVal = std::sin(ARotation);

            const double X = static_cast<double>(APoint.X);
            const double Y = static_cast<double>(APoint.Y);

            return CetInpoint(static_cast<ClipperLib::cInt>(std::llround(X * CosVal - Y * SinVal)),static_cast<ClipperLib::cInt>(std::llround(X * SinVal + Y * CosVal)));

        }
        bool CetTriangleClusterBuilder::_BuildOppositeTrianglePairCandidate(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, int AAIndex, int ABIndex, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate)
        {
            if (AAIndex < 0 || ABIndex < 0 || AAIndex == ABIndex ||
                AAIndex >= static_cast<int>(AOriginalItems.size()) ||
                ABIndex >= static_cast<int>(AOriginalItems.size()) ||
                AAIndex >= static_cast<int>(AFeatures.size()) ||
                ABIndex >= static_cast<int>(AFeatures.size())){
                return false;
            }

            const TetShapeFeature& FeatureA = AFeatures[AAIndex];
            const TetShapeFeature& FeatureB = AFeatures[ABIndex];

            if (FeatureA.ShapeType != MetShapeType::TriangleLike ||FeatureB.ShapeType != MetShapeType::TriangleLike){
                return false;
            }

            if (!_AreCongruentTriangles(FeatureA, FeatureB)){
                return false;
            }

            double AllowedHalfTurn = 0.0;

            if (!CetRotationUtils::SnapToAllowedRotation(CET_CLUSTER_PI, AOptions.Rotations, AllowedHalfTurn, 1e-9)){
                return false;
            }

            CetClusterGeometryHelper Geometry;

            CetPath PathA = Geometry.GetIdentityContour(AOriginalItems[AAIndex]);

            CetPath PathB = Geometry.TransformContour(Geometry.GetIdentityContour(AOriginalItems[ABIndex]), AllowedHalfTurn, 0.0, 0.0);

            double AMinX = 0.0, AMinY = 0.0, AMaxX = 0.0, AMaxY = 0.0;
            double BMinX = 0.0, BMinY = 0.0, BMaxX = 0.0, BMaxY = 0.0;

            if (!Geometry.GetBounds(PathA, AMinX, AMinY, AMaxX, AMaxY)) return false;
            if (!Geometry.GetBounds(PathB, BMinX, BMinY, BMaxX, BMaxY)) return false;

            const double RequiredGap = std::max(0.0, static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing)));
            const double SafetyGap = RequiredGap > 0.0 ? std::max(10.0, RequiredGap * 0.001) : 0.0;
            const double Gap = RequiredGap + SafetyGap;

            
            TetItemTransform TransformA;
            TransformA.OriginalId = AAIndex;
            TransformA.RelativeRotation = 0.0;
            TransformA.RelativeX = -AMinX;
            TransformA.RelativeY = -AMinY;

            
            TetItemTransform TransformB;
            TransformB.OriginalId = ABIndex;
            TransformB.RelativeRotation = AllowedHalfTurn;

            
            const double ACenterX = (AMaxX - AMinX) * 0.5;
            const double BWidth = BMaxX - BMinX;
            TransformB.RelativeX = ACenterX - BWidth * 0.5 - BMinX;
            TransformB.RelativeY = (AMaxY - AMinY) + Gap - BMinY;

            AOutCandidate = TetClusterCandidate{};
            AOutCandidate.BuilderName = "TriangleBuilder";
            AOutCandidate.ClusterType = "TriangleOppositePair";
            AOutCandidate.OriginalIndices = { AAIndex, ABIndex };
            AOutCandidate.Transforms = { TransformA, TransformB };
            AOutCandidate.Confidence = 0.75;

            if (!Geometry.FinalizeCandidate(AOriginalItems, AOptions, AOutCandidate)){
                return false;
            }

            return true;
        }
        
        double CetTriangleClusterBuilder::_Cross(const CetInpoint& AA, const CetInpoint& AB, CetInpoint& AP)
        {
            const double ABX = static_cast<double>(AB.X - AA.X);
            const double ABY = static_cast<double>(AB.Y - AA.Y);

            const double APX = static_cast<double>(AP.X - AA.X);
            const double APY = static_cast<double>(AP.Y - AA.Y);

            return ABX * APY - ABY * APX;
        }
        
        bool CetTriangleClusterBuilder::_GetTriangleThirdPoint(const CetPath& AContour, int AEdgeIndex, CetInpoint& AOutPoint)
        {
            if (AContour.size() != 3){return false;}
            if (AEdgeIndex < 0 || AEdgeIndex >= 3){return false;}
            const int ThirdIndex = (AEdgeIndex + 2) % 3;
            AOutPoint = AContour[ThirdIndex];
            return true;
        }
    }
}
