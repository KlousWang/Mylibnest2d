#include "pch.h"
#include "Nest2D_ShapeAnalyzer.h"
#include "Nest2D_ClusterMathUtils.h"
#include "Nest2D_RotationUtils.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

namespace ET {
    namespace NEST2DMANAGERLIB {

        namespace {
            TetCircleFitResult FitCircleCenterFromThreePoints(const ClipperLib::IntPoint &AFirstPoint, const ClipperLib::IntPoint &AMiddlePoint, const ClipperLib::IntPoint &ALastPoint)
            {
                TetCircleFitResult FitResult;
                const double FirstX = static_cast<double>(AFirstPoint.X);
                const double FirstY = static_cast<double>(AFirstPoint.Y);
                const double MiddleX = static_cast<double>(AMiddlePoint.X);
                const double MiddleY = static_cast<double>(AMiddlePoint.Y);
                const double LastX = static_cast<double>(ALastPoint.X);
                const double LastY = static_cast<double>(ALastPoint.Y);

                const double Denominator = 2.0 * (FirstX * (MiddleY - LastY) + MiddleX * (LastY - FirstY) + LastX * (FirstY - MiddleY));
                if (std::abs(Denominator) <= CET_SHAPE_EPSILON)
                    return FitResult;

                const double FirstSquared = FirstX * FirstX + FirstY * FirstY;
                const double MiddleSquared = MiddleX * MiddleX + MiddleY * MiddleY;
                const double LastSquared = LastX * LastX + LastY * LastY;

                FitResult.CenterX = (FirstSquared * (MiddleY - LastY) + MiddleSquared * (LastY - FirstY) + LastSquared * (FirstY - MiddleY)) / Denominator;
                FitResult.CenterY = (FirstSquared * (LastX - MiddleX) + MiddleSquared * (FirstX - LastX) + LastSquared * (MiddleX - FirstX)) / Denominator;
                FitResult.Valid = std::isfinite(FitResult.CenterX) && std::isfinite(FitResult.CenterY);
                return FitResult;
            }

            TetCircleFitResult FitCircleCenterFromChain(const CetPath &AChain)
            {
                if (AChain.size() < 3)
                    return TetCircleFitResult{};
                const std::size_t MiddleIndex = AChain.size() / 2;
                return FitCircleCenterFromThreePoints(AChain.front(), AChain[MiddleIndex], AChain.back());
            }

            CetPath BuildClosedContourChain(const CetPath &AContour, std::size_t AStartIndex, std::size_t AEndIndex)
            {
                CetPath Chain;
                if (AContour.empty())
                    return Chain;

                std::size_t CurrentIndex = AStartIndex % AContour.size();
                const std::size_t TargetIndex = AEndIndex % AContour.size();

                while (true) {
                    Chain.push_back(AContour[CurrentIndex]);
                    if (CurrentIndex == TargetIndex)
                        break;
                    CurrentIndex = (CurrentIndex + 1) % AContour.size();
                    if (Chain.size() > AContour.size()) {
                        Chain.clear();
                        break;
                    }
                }
                return Chain;
            }

            bool EvaluateArcChain(const CetPath &AChain, double ACenterX, double ACenterY, double &AOutAverageRadius, double &AOutAverageError, double &AOutMaxError)
            {
                AOutAverageRadius = 0.0;
                AOutAverageError = 1.0;
                AOutMaxError = 1.0;
                if (AChain.size() < 3)
                    return false;

                for (const ClipperLib::IntPoint &Point : AChain) {
                    AOutAverageRadius += std::hypot(static_cast<double>(Point.X) - ACenterX, static_cast<double>(Point.Y) - ACenterY);
                }
                AOutAverageRadius /= static_cast<double>(AChain.size());
                if (AOutAverageRadius <= CET_SHAPE_EPSILON)
                    return false;

                double SumError = 0.0;
                double MaxError = 0.0;
                for (const ClipperLib::IntPoint &Point : AChain) {
                    const double Radius = std::hypot(static_cast<double>(Point.X) - ACenterX, static_cast<double>(Point.Y) - ACenterY);
                    const double Error = std::abs(Radius - AOutAverageRadius) / std::max(1.0, AOutAverageRadius);
                    SumError += Error;
                    MaxError = std::max(MaxError, Error);
                }
                AOutAverageError = SumError / static_cast<double>(AChain.size());
                AOutMaxError = MaxError;
                return true;
            }

            void EvaluateArcChainPair(const CetPath &AContour, std::size_t AFirstEdgeIndex, std::size_t ASecondEdgeIndex, TetArcChainFitResult &ABestFitResult, CetPath &ABestOuterChain)
            {
                const std::size_t ContourSize = AContour.size();
                const CetPath FirstChain = BuildClosedContourChain(AContour, (AFirstEdgeIndex + 1) % ContourSize, ASecondEdgeIndex);
                const CetPath SecondChain = BuildClosedContourChain(AContour, (ASecondEdgeIndex + 1) % ContourSize, AFirstEdgeIndex);
                if (FirstChain.size() < 3 || SecondChain.size() < 3)
                    return;
                const TetCircleFitResult FirstCenterFit = FitCircleCenterFromChain(FirstChain);
                const TetCircleFitResult SecondCenterFit = FitCircleCenterFromChain(SecondChain);
                if (!FirstCenterFit.Valid || !SecondCenterFit.Valid)
                    return;
                const double CenterDistance = std::hypot(FirstCenterFit.CenterX - SecondCenterFit.CenterX, FirstCenterFit.CenterY - SecondCenterFit.CenterY);
                const double CenterX = (FirstCenterFit.CenterX + SecondCenterFit.CenterX) * 0.5;
                const double CenterY = (FirstCenterFit.CenterY + SecondCenterFit.CenterY) * 0.5;
                double FirstRadius = 0.0, FirstAverageError = 1.0, FirstMaxError = 1.0;
                double SecondRadius = 0.0, SecondAverageError = 1.0, SecondMaxError = 1.0;
                if (!EvaluateArcChain(FirstChain, CenterX, CenterY, FirstRadius, FirstAverageError, FirstMaxError) || !EvaluateArcChain(SecondChain, CenterX, CenterY, SecondRadius, SecondAverageError, SecondMaxError))
                    return;
                const double OuterRadius = std::max(FirstRadius, SecondRadius);
                const double InnerRadius = std::min(FirstRadius, SecondRadius);
                if (InnerRadius <= CET_SHAPE_EPSILON || OuterRadius <= InnerRadius || CenterDistance > OuterRadius * 0.10)
                    return;
                const double Thickness = OuterRadius - InnerRadius;
                if (Thickness < OuterRadius * 0.02 || Thickness > OuterRadius * 0.80)
                    return;
                const double AverageError = (FirstAverageError + SecondAverageError) * 0.5 + CenterDistance / std::max(1.0, OuterRadius);
                const double MaxError = std::max(FirstMaxError, SecondMaxError);
                if (AverageError > 0.08 || MaxError > 0.20 || (ABestFitResult.Valid && AverageError >= ABestFitResult.AverageError))
                    return;
                ABestFitResult.Valid = true;
                ABestFitResult.CenterX = CenterX;
                ABestFitResult.CenterY = CenterY;
                ABestFitResult.InnerRadius = InnerRadius;
                ABestFitResult.OuterRadius = OuterRadius;
                ABestFitResult.AverageError = AverageError;
                ABestFitResult.MaxError = MaxError;
                ABestOuterChain = FirstRadius >= SecondRadius ? FirstChain : SecondChain;
            }

            bool TryFitArcChains(const CetPath &AContour, TetArcChainFitResult &AOutFitResult, CetPath &AOutOuterChain)
            {
                AOutFitResult = TetArcChainFitResult{};
                AOutOuterChain.clear();
                const std::size_t ContourSize = AContour.size();
                if (ContourSize < 6)
                    return false;

                std::vector<std::pair<double, std::size_t>> EdgeLengths;
                EdgeLengths.reserve(ContourSize);
                for (std::size_t EdgeIndex = 0; EdgeIndex < ContourSize; ++EdgeIndex) {
                    const ClipperLib::IntPoint &StartPoint = AContour[EdgeIndex];
                    const ClipperLib::IntPoint &EndPoint = AContour[(EdgeIndex + 1) % ContourSize];
                    const double EdgeLength = std::hypot(static_cast<double>(EndPoint.X - StartPoint.X), static_cast<double>(EndPoint.Y - StartPoint.Y));
                    EdgeLengths.emplace_back(EdgeLength, EdgeIndex);
                }
                std::sort(EdgeLengths.begin(), EdgeLengths.end(), [](const auto &A, const auto &B) { return A.first > B.first; });

                TetArcChainFitResult BestFitResult;
                CetPath BestOuterChain;
                const std::size_t CandidateLimit = std::min<std::size_t>(8, EdgeLengths.size());

                for (std::size_t FirstEdgeOffset = 0; FirstEdgeOffset < CandidateLimit; ++FirstEdgeOffset) {
                    for (std::size_t SecondEdgeOffset = FirstEdgeOffset + 1; SecondEdgeOffset < CandidateLimit; ++SecondEdgeOffset) {
                        const std::size_t FirstEdgeIndex = EdgeLengths[FirstEdgeOffset].second;
                        const std::size_t SecondEdgeIndex = EdgeLengths[SecondEdgeOffset].second;
                        if (FirstEdgeIndex == SecondEdgeIndex)
                            continue;

                        EvaluateArcChainPair(AContour, FirstEdgeIndex, SecondEdgeIndex, BestFitResult, BestOuterChain);
                    }
                }

                if (!BestFitResult.Valid)
                    return false;
                AOutFitResult = BestFitResult;
                AOutOuterChain = std::move(BestOuterChain);
                return true;
            }

            TetAngleSpanResult FindMinimalAngleSpan(const CetPath &AChain, double ACenterX, double ACenterY)
            {
                TetAngleSpanResult Result;
                if (AChain.size() < 2)
                    return Result;

                std::vector<double> Angles;
                Angles.reserve(AChain.size());
                for (const ClipperLib::IntPoint &Point : AChain) {
                    Angles.push_back(CetRotationUtils::NormalizeAngle(std::atan2(static_cast<double>(Point.Y) - ACenterY, static_cast<double>(Point.X) - ACenterX)));
                }

                std::sort(Angles.begin(), Angles.end());
                Angles.erase(std::unique(Angles.begin(), Angles.end(), [](double A, double B) { return std::abs(A - B) <= 1e-9; }), Angles.end());
                if (Angles.size() < 2)
                    return Result;

                double LargestGap = -1.0;
                std::size_t LargestGapStartIndex = 0;
                for (std::size_t AngleIndex = 0; AngleIndex < Angles.size(); ++AngleIndex) {
                    const std::size_t NextAngleIndex = (AngleIndex + 1) % Angles.size();
                    const double CurrentAngle = Angles[AngleIndex];
                    const double NextAngle = NextAngleIndex == 0 ? Angles[NextAngleIndex] + CET_CLUSTER_TWO_PI : Angles[NextAngleIndex];
                    const double Gap = NextAngle - CurrentAngle;
                    if (Gap > LargestGap) {
                        LargestGap = Gap;
                        LargestGapStartIndex = AngleIndex;
                    }
                }

                const std::size_t SpanStartIndex = (LargestGapStartIndex + 1) % Angles.size();
                const double StartAngle = Angles[SpanStartIndex];
                const double EndAngle = Angles[LargestGapStartIndex] < StartAngle ? Angles[LargestGapStartIndex] + CET_CLUSTER_TWO_PI : Angles[LargestGapStartIndex];
                const double SweepAngle = EndAngle - StartAngle;

                if (SweepAngle < CET_GENERAL_ARC_MIN_SWEEP || SweepAngle > CET_GENERAL_ARC_MAX_SWEEP)
                    return Result;

                Result.Valid = true;
                Result.StartAngle = StartAngle;
                Result.EndAngle = EndAngle;
                Result.SweepAngle = SweepAngle;
                return Result;
            }

            bool TryAnalyzeGeneralThickArcFeature(const CetPath &AContour, TetShapeFeature &AFeature)
            {
                TetArcChainFitResult ChainFit;
                CetPath OuterChain;
                if (!TryFitArcChains(AContour, ChainFit, OuterChain))
                    return false;

                const TetAngleSpanResult AngleSpan = FindMinimalAngleSpan(OuterChain, ChainFit.CenterX, ChainFit.CenterY);
                if (!AngleSpan.Valid)
                    return false;

                const double ExpectedArea = 0.5 * AngleSpan.SweepAngle * (ChainFit.OuterRadius * ChainFit.OuterRadius - ChainFit.InnerRadius * ChainFit.InnerRadius);
                if (ExpectedArea <= CET_SHAPE_EPSILON)
                    return false;

                const double AreaError = std::abs(AFeature.Area - ExpectedArea) / std::max(1.0, ExpectedArea);
                if (AreaError > 0.22)
                    return false;

                const double ChordStartX = ChainFit.CenterX + ChainFit.OuterRadius * std::cos(AngleSpan.StartAngle);
                const double ChordStartY = ChainFit.CenterY + ChainFit.OuterRadius * std::sin(AngleSpan.StartAngle);
                const double ChordEndX = ChainFit.CenterX + ChainFit.OuterRadius * std::cos(AngleSpan.EndAngle);
                const double ChordEndY = ChainFit.CenterY + ChainFit.OuterRadius * std::sin(AngleSpan.EndAngle);
                const double ChordDeltaX = ChordEndX - ChordStartX;
                const double ChordDeltaY = ChordEndY - ChordStartY;
                const double ChordLength = std::hypot(ChordDeltaX, ChordDeltaY);
                if (ChordLength <= CET_SHAPE_EPSILON)
                    return false;

                const double MidAngle = AngleSpan.StartAngle + AngleSpan.SweepAngle * 0.5;
                const double MidArcX = ChainFit.CenterX + ChainFit.OuterRadius * std::cos(MidAngle);
                const double MidArcY = ChainFit.CenterY + ChainFit.OuterRadius * std::sin(MidAngle);
                const double MidCross = ChordDeltaX * (MidArcY - ChordStartY) - ChordDeltaY * (MidArcX - ChordStartX);
                const int BulgeSign = MidCross >= 0.0 ? 1 : -1;

                AFeature.ArcType = std::abs(AngleSpan.SweepAngle - CET_CLUSTER_PI) <= CET_GENERAL_ARC_SEMI_TOLERANCE ? MetArcType::SemiCircleLike : MetArcType::GeneralArcLike;
                AFeature.ArcCenter = ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(std::llround(ChainFit.CenterX)), static_cast<ClipperLib::cInt>(std::llround(ChainFit.CenterY)));
                AFeature.ArcChordStart = ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(std::llround(ChordStartX)), static_cast<ClipperLib::cInt>(std::llround(ChordStartY)));
                AFeature.ArcChordEnd = ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(std::llround(ChordEndX)), static_cast<ClipperLib::cInt>(std::llround(ChordEndY)));
                AFeature.ArcRadius = ChainFit.OuterRadius;
                AFeature.ArcChordLength = ChordLength;
                AFeature.ArcChordAngle = std::atan2(ChordDeltaY, ChordDeltaX);
                AFeature.ArcSweepAngle = AngleSpan.SweepAngle;
                AFeature.ArcBulgeSign = BulgeSign;
                AFeature.ArcFitError = std::max(ChainFit.AverageError, AreaError);

                std::cout << "[SHAPE][ARC][GENERAL_THICK] Index=" << AFeature.OriginalIndex << " Type=" << static_cast<int>(AFeature.ArcType) << " OuterRadius=" << AFeature.ArcRadius << " InnerRadius=" << ChainFit.InnerRadius << " Sweep=" << AFeature.ArcSweepAngle << " Chord=" << AFeature.ArcChordLength << " FitError=" << AFeature.ArcFitError << " BulgeSign=" << AFeature.ArcBulgeSign << std::endl;
                return true;
            }
        } // namespace

        CetShapeAnalyzer::CetShapeAnalyzer() : CetCoreObject() {}
        CetShapeAnalyzer::~CetShapeAnalyzer() {}

        std::vector<TetShapeFeature> CetShapeAnalyzer::AnalyzeALL(const CetTNestItemVector &AItems)
        {
            std::vector<TetShapeFeature> Features;
            Features.reserve(AItems.size());
            for (int i = 0; i < static_cast<int>(AItems.size()); ++i) {
                Features.push_back(_AnalyzeOne(AItems[i], i));
            }
            return Features;
        }

        TetShapeFeature CetShapeAnalyzer::_AnalyzeOne(const CetNestItem &AItem, int AOriginalIndex)
        {
            TetShapeFeature Feature;
            Feature.OriginalIndex = AOriginalIndex;

            CetNestItem Temp = AItem;
            Temp.translation(libnest2d::Point(0, 0));
            Temp.rotation(libnest2d::Radians(0.0));
            Temp.inflation(0);
            CetPath Contour = Temp.transformedShape().Contour;
            ClipperLib::Paths Holes = Temp.transformedShape().Holes;

            _NormalizePath(Contour);
            for (auto &Hole : Holes)
                _NormalizePath(Hole);
            Holes.erase(std::remove_if(Holes.begin(), Holes.end(), [](const CetPath &AH) { return AH.size() < 3; }), Holes.end());

            Feature.NormalizedContour = Contour;
            Feature.HasHoles = !Holes.empty();
            Feature.HoleCount = static_cast<int>(Holes.size());
            Feature.VertexCount = static_cast<int>(Contour.size());
            if (Contour.size() < 3)
                return Feature;

            auto MinX = Contour.front().X, MaxX = Contour.front().X;
            auto MinY = Contour.front().Y, MaxY = Contour.front().Y;
            for (const auto &P : Contour) {
                MinX = std::min(MinX, P.X);
                MaxX = std::max(MaxX, P.X);
                MinY = std::min(MinY, P.Y);
                MaxY = std::max(MaxY, P.Y);
            }

            Feature.MinX = static_cast<double>(MinX);
            Feature.MaxX = static_cast<double>(MaxX);
            Feature.MinY = static_cast<double>(MinY);
            Feature.MaxY = static_cast<double>(MaxY);
            Feature.Width = static_cast<double>(MaxX - MinX);
            Feature.Height = static_cast<double>(MaxY - MinY);

            double HoleArea = 0.0;
            for (const auto &H : Holes)
                HoleArea += std::abs(static_cast<double>(ClipperLib::Area(H)));
            Feature.Area = std::max(0.0, std::abs(static_cast<double>(ClipperLib::Area(Contour))) - HoleArea);
            Feature.BoxArea = Feature.Width * Feature.Height;
            Feature.FillRatio = Feature.BoxArea > CET_SHAPE_EPSILON ? Feature.Area / Feature.BoxArea : 0.0;

            const double ShortSide = std::min(Feature.Width, Feature.Height);
            Feature.AspectRatio = ShortSide > CET_SHAPE_EPSILON ? std::max(Feature.Width, Feature.Height) / ShortSide : 0.0;
            Feature.IsConvex = _IsConvex(Contour);

            double Perimeter = _CalculatePerimeter(Contour);
            for (const auto &H : Holes)
                Perimeter += _CalculatePerimeter(H);
            Feature.Circularity = Perimeter > CET_SHAPE_EPSILON ? 4.0 * CET_CLUSTER_PI * Feature.Area / (Perimeter * Perimeter) : 0.0;
            Feature.Circularity = std::clamp(Feature.Circularity, 0.0, 1.0);

            _AnalyzeTriangleFeature(Contour, Feature);
            _AnalyzeRectangleFeature(Contour, Feature);
            _AnalyzeArcFeature(Contour, Feature);
            _AnalyzeEllipseFeature(Contour, Feature);
            Feature.ShapeType = _ClassifyShape(Feature);

            std::cout << "[SHAPE] Index=" << Feature.OriginalIndex << " Type=" << static_cast<int>(Feature.ShapeType) << " Vertices=" << Feature.VertexCount << " W=" << Feature.Width << " H=" << Feature.Height << " Fill=" << Feature.FillRatio << std::endl;
            return Feature;
        }

        void CetShapeAnalyzer::_AnalyzeTriangleFeature(const CetPath &AContour, TetShapeFeature &AFeature)
        {
            if (AContour.size() != 3)
                return;

            const double L0 = _Distance(AContour[0], AContour[1]);
            const double L1 = _Distance(AContour[1], AContour[2]);
            const double L2 = _Distance(AContour[2], AContour[0]);

            std::array<double, 3> SortedSides = {L0, L1, L2};
            std::sort(SortedSides.begin(), SortedSides.end());
            AFeature.TriangleSides = SortedSides;

            constexpr double SideTolerance = 0.02;
            const bool Equal01 = CetClusterMathUtils::NearlyEqual(SortedSides[0], SortedSides[1], SideTolerance);
            const bool Equal12 = CetClusterMathUtils::NearlyEqual(SortedSides[1], SortedSides[2], SideTolerance);

            if (Equal01 && Equal12) {
                AFeature.TriangleSideType = MetTriangleSideType::Equilateral;
            } else if (Equal01 || Equal12 || CetClusterMathUtils::NearlyEqual(SortedSides[0], SortedSides[2], SideTolerance)) {
                AFeature.TriangleSideType = MetTriangleSideType::Isosceles;
            } else {
                AFeature.TriangleSideType = MetTriangleSideType::Scalene;
            }

            const double A2 = SortedSides[0] * SortedSides[0];
            const double B2 = SortedSides[1] * SortedSides[1];
            const double C2 = SortedSides[2] * SortedSides[2];
            const double Sum = A2 + B2;
            const double Difference = C2 - Sum;

            constexpr double AngleTolerance = 0.03;
            if (std::abs(Difference) <= std::max(1.0, Sum) * AngleTolerance) {
                AFeature.TriangleAngleType = MetTriangleAngleType::Right;
            } else if (Difference > 0.0) {
                AFeature.TriangleAngleType = MetTriangleAngleType::Obtuse;
            } else {
                AFeature.TriangleAngleType = MetTriangleAngleType::Acute;
            }

            for (std::size_t i = 0; i < AContour.size(); ++i) {
                const auto &Previous = AContour[(i + AContour.size() - 1) % AContour.size()];
                const auto &Current = AContour[i];
                const auto &Next = AContour[(i + 1) % AContour.size()];
                AFeature.TriangleAngles[i] = _AngleAtVertex(Previous, Current, Next);
            }
        }

        void CetShapeAnalyzer::_AnalyzeRectangleFeature(const CetPath &AContour, TetShapeFeature &AFeature)
        {
            if (AContour.size() != 4 || !AFeature.IsConvex)
                return;

            std::array<double, 4> EdgeLengths{};
            std::array<long double, 4> EdgeX{};
            std::array<long double, 4> EdgeY{};

            for (std::size_t i = 0; i < 4; ++i) {
                const auto &A = AContour[i];
                const auto &B = AContour[(i + 1) % 4];
                EdgeX[i] = static_cast<long double>(B.X) - static_cast<long double>(A.X);
                EdgeY[i] = static_cast<long double>(B.Y) - static_cast<long double>(A.Y);
                EdgeLengths[i] = std::sqrt(static_cast<double>(EdgeX[i] * EdgeX[i] + EdgeY[i] * EdgeY[i]));
                if (EdgeLengths[i] <= CET_SHAPE_EPSILON)
                    return;
            }

            constexpr double OrthogonalTolerance = 0.035;
            constexpr double LengthTolerance = 0.02;

            for (std::size_t i = 0; i < 4; ++i) {
                const std::size_t Next = (i + 1) % 4;
                const long double Dot = EdgeX[i] * EdgeX[Next] + EdgeY[i] * EdgeY[Next];
                const long double Denominator = static_cast<long double>(EdgeLengths[i] * EdgeLengths[Next]);
                const double NormalizedDot = Denominator > 0.0L ? std::abs(static_cast<double>(Dot / Denominator)) : 1.0;
                if (NormalizedDot > OrthogonalTolerance)
                    return;
            }

            if (!CetClusterMathUtils::NearlyEqual(EdgeLengths[0], EdgeLengths[2], LengthTolerance) || !CetClusterMathUtils::NearlyEqual(EdgeLengths[1], EdgeLengths[3], LengthTolerance)) {
                return;
            }

            AFeature.IsRotatedRectangle = true;
            AFeature.OrientedWidth = EdgeLengths[0];
            AFeature.OrientedHeight = EdgeLengths[1];
            AFeature.OrientedAngle = std::atan2(static_cast<double>(EdgeY[0]), static_cast<double>(EdgeX[0]));
            AFeature.OrientedBoxArea = AFeature.OrientedWidth * AFeature.OrientedHeight;
            AFeature.OrientedFillRatio = AFeature.OrientedBoxArea > 0.0 ? AFeature.Area / AFeature.OrientedBoxArea : 0.0;
        }

        void CetShapeAnalyzer::_AnalyzeArcFeature(const CetPath &AContour, TetShapeFeature &AFeature)
        {
            AFeature.ArcType = MetArcType::None;
            AFeature.ArcFitError = 1.0;
            AFeature.ArcBulgeSign = 0;

            if (AFeature.HasHoles || AContour.size() < 5)
                return;

            if (!AFeature.IsConvex) {
                _AnalyzeThickArcFeature(AContour, AFeature);
                return;
            }

            _AnalyzeSolidArcFeature(AContour, AFeature);
        }

        void CetShapeAnalyzer::_AnalyzeEllipseFeature(const CetPath &AContour, TetShapeFeature &AFeature)
        {
            AFeature.EllipseMajorAxis = 0.0;
            AFeature.EllipseMinorAxis = 0.0;
            AFeature.EllipseAngle = 0.0;
            AFeature.EllipseFitError = 1.0;

            if (AFeature.HasHoles || !AFeature.IsConvex || AContour.size() < 8)
                return;
            if (AFeature.Width <= CET_SHAPE_EPSILON || AFeature.Height <= CET_SHAPE_EPSILON)
                return;

            const double CenterX = (AFeature.MinX + AFeature.MaxX) * 0.5;
            const double CenterY = (AFeature.MinY + AFeature.MaxY) * 0.5;
            const double RadiusX = AFeature.Width * 0.5;
            const double RadiusY = AFeature.Height * 0.5;

            if (RadiusX <= CET_SHAPE_EPSILON || RadiusY <= CET_SHAPE_EPSILON)
                return;

            double SumError = 0.0;
            double MaxError = 0.0;
            for (const ClipperLib::IntPoint &Point : AContour) {
                const double X = (static_cast<double>(Point.X) - CenterX) / RadiusX;
                const double Y = (static_cast<double>(Point.Y) - CenterY) / RadiusY;
                const double Error = std::abs((X * X + Y * Y) - 1.0);
                SumError += Error;
                MaxError = std::max(MaxError, Error);
            }

            const double AverageError = SumError / static_cast<double>(AContour.size());
            if (AverageError > 0.12 || MaxError > 0.35)
                return;

            const double MajorAxis = std::max(AFeature.Width, AFeature.Height);
            const double MinorAxis = std::min(AFeature.Width, AFeature.Height);
            if (MinorAxis <= CET_SHAPE_EPSILON)
                return;

            AFeature.EllipseMajorAxis = MajorAxis;
            AFeature.EllipseMinorAxis = MinorAxis;
            AFeature.EllipseAngle = AFeature.Width >= AFeature.Height ? 0.0 : CET_CLUSTER_HALF_PI;
            AFeature.EllipseFitError = AverageError;

            std::cout << "[SHAPE][ELLIPSE] Index=" << AFeature.OriginalIndex << " Major=" << AFeature.EllipseMajorAxis << " Minor=" << AFeature.EllipseMinorAxis << " FitError=" << AFeature.EllipseFitError << std::endl;
        }

        MetShapeType CetShapeAnalyzer::_ClassifyShape(const TetShapeFeature &AFeature)
        {
            if (AFeature.VertexCount < 3)
                return MetShapeType::Unknown;
            if (AFeature.VertexCount == 3)
                return MetShapeType::TriangleLike;
            if (AFeature.IsRotatedRectangle)
                return MetShapeType::RectangleLike;
            if (!AFeature.HasHoles && AFeature.ArcType != MetArcType::None)
                return MetShapeType::ArcLike;
            if (!AFeature.HasHoles && AFeature.IsConvex && AFeature.Circularity >= 0.88 && AFeature.AspectRatio <= 1.08)
                return MetShapeType::CircleLike;
            if (!AFeature.HasHoles && AFeature.IsConvex && AFeature.EllipseFitError <= 0.12 && AFeature.AspectRatio > 1.08)
                return MetShapeType::EllipseLike;
            if (AFeature.VertexCount == 4 && AFeature.IsConvex)
                return MetShapeType::QuadrilateralLike;
            return AFeature.IsConvex ? MetShapeType::ConvexPolygon : MetShapeType::ConcavePolygon;
        }

        double CetShapeAnalyzer::_Distance(const ClipperLib::IntPoint &AA, const ClipperLib::IntPoint &AB) { return std::hypot(static_cast<double>(AB.X - AA.X), static_cast<double>(AB.Y - AA.Y)); }

        double CetShapeAnalyzer::_AngleAtVertex(const ClipperLib::IntPoint &APeacture, const ClipperLib::IntPoint &ACurrent, const ClipperLib::IntPoint &ANext)
        {
            const double AX = static_cast<double>(APeacture.X - ACurrent.X), AY = static_cast<double>(APeacture.Y - ACurrent.Y);
            const double BX = static_cast<double>(ANext.X - ACurrent.X), BY = static_cast<double>(ANext.Y - ACurrent.Y);
            const double Den = std::hypot(AX, AY) * std::hypot(BX, BY);
            if (Den <= CET_SHAPE_EPSILON)
                return 0.0;
            return std::acos(std::clamp((AX * BX + AY * BY) / Den, -1.0, 1.0));
        }

        double CetShapeAnalyzer::_CalculatePerimeter(const CetPath &AContour)
        {
            const std::size_t Count = AContour.size();
            if (Count < 2)
                return 0.0;

            long double Perimeter = 0.0L;
            for (std::size_t i = 0; i < Count; ++i) {
                const std::size_t NextIndex = (i + 1) % Count;
                const ClipperLib::IntPoint &Current = AContour.at(i);
                const ClipperLib::IntPoint &Next = AContour.at(NextIndex);
                const long double DX = static_cast<long double>(Next.X) - static_cast<long double>(Current.X);
                const long double DY = static_cast<long double>(Next.Y) - static_cast<long double>(Current.Y);
                Perimeter += std::sqrt(DX * DX + DY * DY);
            }
            return static_cast<double>(Perimeter);
        }

        bool CetShapeAnalyzer::_IsConvex(const CetPath &AContour)
        {
            const std::size_t Count = AContour.size();
            if (Count < 3)
                return false;

            bool HasPositiveCross = false;
            bool HasNegativeCross = false;
            for (std::size_t i = 0; i < Count; ++i) {
                const ClipperLib::IntPoint &A = AContour[i];
                const ClipperLib::IntPoint &B = AContour[(i + 1) % Count];
                const ClipperLib::IntPoint &C = AContour[(i + 2) % Count];
                const long double ABX = static_cast<long double>(B.X) - static_cast<long double>(A.X);
                const long double ABY = static_cast<long double>(B.Y) - static_cast<long double>(A.Y);
                const long double BCX = static_cast<long double>(C.X) - static_cast<long double>(B.X);
                const long double BCY = static_cast<long double>(C.Y) - static_cast<long double>(B.Y);
                const long double Cross = ABX * BCY - ABY * BCX;

                if (Cross > 0.0L) {
                    HasPositiveCross = true;
                } else if (Cross < 0.0L) {
                    HasNegativeCross = true;
                }
                if (HasPositiveCross && HasNegativeCross)
                    return false;
            }
            return HasPositiveCross || HasNegativeCross;
        }

        void CetShapeAnalyzer::_NormalizePath(CetPath &APath)
        {
            if (APath.empty())
                return;

            CetPath Clean;
            Clean.reserve(APath.size());
            for (const auto &Point : APath) {
                if (!Clean.empty() && Clean.back().X == Point.X && Clean.back().Y == Point.Y)
                    continue;
                Clean.push_back(Point);
            }

            if (Clean.size() >= 2 && Clean.front().X == Clean.back().X && Clean.front().Y == Clean.back().Y)
                Clean.pop_back();

            bool Changed = true;
            while (Changed && Clean.size() > 3) {
                Changed = false;
                CetPath Result;
                const std::size_t Count = Clean.size();
                for (std::size_t i = 0; i < Count; ++i) {
                    const auto &A = Clean[(i + Count - 1) % Count];
                    const auto &B = Clean[i];
                    const auto &C = Clean[(i + 1) % Count];
                    const long double Cross = static_cast<long double>(B.X - A.X) * static_cast<long double>(C.Y - B.Y) - static_cast<long double>(B.Y - A.Y) * static_cast<long double>(C.X - B.X);
                    if (Cross == 0.0L) {
                        Changed = true;
                        continue;
                    }
                    Result.push_back(B);
                }
                if (Result.size() < 3)
                    break;
                Clean = std::move(Result);
            }
            APath = std::move(Clean);
        }

        bool CetShapeAnalyzer::_AnalyzeThickArcFeature(const CetPath &AContour, TetShapeFeature &AFeature)
        {
            if (AFeature.HasHoles || AFeature.IsConvex || AContour.size() < 5)
                return false;
            if (AFeature.Width <= CET_SHAPE_EPSILON || AFeature.Height <= CET_SHAPE_EPSILON || AFeature.Area <= CET_SHAPE_EPSILON)
                return false;
            if (TryAnalyzeGeneralThickArcFeature(AContour, AFeature))
                return true;
            if (AFeature.AspectRatio < 1.40 || AFeature.AspectRatio > 2.60)
                return false;
            if (AFeature.FillRatio < 0.10 || AFeature.FillRatio > 0.55)
                return false;

            TetArcCandidateLocal BestCandidate;
            const double MinX = AFeature.MinX;
            const double MaxX = AFeature.MaxX;
            const double MinY = AFeature.MinY;
            const double MaxY = AFeature.MaxY;
            const double Width = AFeature.Width;
            const double Height = AFeature.Height;
            const double CenterX = (MinX + MaxX) * 0.5;
            const double CenterY = (MinY + MaxY) * 0.5;

            if (Width >= Height * 1.40) {
                const double OuterRadius = Width * 0.5;
                _EvaluateThickArcCandidate(AContour, AFeature, TetThickArcTestInput{CenterX, MinY, OuterRadius, true, 1, 0.0, ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(std::llround(CenterX - OuterRadius)), static_cast<ClipperLib::cInt>(std::llround(MinY))), ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(std::llround(CenterX + OuterRadius)), static_cast<ClipperLib::cInt>(std::llround(MinY)))}, BestCandidate);

                _EvaluateThickArcCandidate(AContour, AFeature, TetThickArcTestInput{CenterX, MaxY, OuterRadius, true, -1, 0.0, ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(std::llround(CenterX - OuterRadius)), static_cast<ClipperLib::cInt>(std::llround(MaxY))), ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(std::llround(CenterX + OuterRadius)), static_cast<ClipperLib::cInt>(std::llround(MaxY)))}, BestCandidate);
            }

            if (Height >= Width * 1.40) {
                const double OuterRadius = Height * 0.5;
                _EvaluateThickArcCandidate(AContour, AFeature, TetThickArcTestInput{MinX, CenterY, OuterRadius, false, 1, CET_CLUSTER_HALF_PI, ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(std::llround(MinX)), static_cast<ClipperLib::cInt>(std::llround(CenterY - OuterRadius))), ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(std::llround(MinX)), static_cast<ClipperLib::cInt>(std::llround(CenterY + OuterRadius)))}, BestCandidate);

                _EvaluateThickArcCandidate(AContour, AFeature, TetThickArcTestInput{MaxX, CenterY, OuterRadius, false, -1, CET_CLUSTER_HALF_PI, ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(std::llround(MaxX)), static_cast<ClipperLib::cInt>(std::llround(CenterY - OuterRadius))), ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(std::llround(MaxX)), static_cast<ClipperLib::cInt>(std::llround(CenterY + OuterRadius)))}, BestCandidate);
            }

            if (!BestCandidate.Valid)
                return false;

            AFeature.ArcType = MetArcType::SemiCircleLike;
            AFeature.ArcCenter = ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(std::llround(BestCandidate.CenterX)), static_cast<ClipperLib::cInt>(std::llround(BestCandidate.CenterY)));
            AFeature.ArcRadius = BestCandidate.OuterRadius;
            AFeature.ArcChordLength = BestCandidate.OuterRadius * 2.0;
            AFeature.ArcChordAngle = BestCandidate.ChordAngle;
            AFeature.ArcSweepAngle = CET_CLUSTER_PI;
            AFeature.ArcBulgeSign = BestCandidate.BulgeSign;
            AFeature.ArcFitError = BestCandidate.FitError;
            AFeature.ArcChordStart = BestCandidate.ChordStart;
            AFeature.ArcChordEnd = BestCandidate.ChordEnd;

            std::cout << "[SHAPE][ARC][THICK] Index=" << AFeature.OriginalIndex << " OuterRadius=" << AFeature.ArcRadius << " InnerRadius=" << BestCandidate.InnerRadius << " Chord=" << AFeature.ArcChordLength << " FitError=" << AFeature.ArcFitError << " BulgeSign=" << AFeature.ArcBulgeSign << std::endl;

            return true;
        }

        void CetShapeAnalyzer::_EvaluateThickArcCandidate(const CetPath &AContour, const TetShapeFeature &AFeature, const TetThickArcTestInput &AInput, TetArcCandidateLocal &AInOutBestCandidate)
        {
            if (AInput.OuterRadius <= CET_SHAPE_EPSILON)
                return;

            const double InnerRadiusSquared = AInput.OuterRadius * AInput.OuterRadius - 2.0 * AFeature.Area / CET_CLUSTER_PI;
            if (InnerRadiusSquared <= CET_SHAPE_EPSILON || InnerRadiusSquared >= AInput.OuterRadius * AInput.OuterRadius)
                return;

            const double InnerRadius = std::sqrt(InnerRadiusSquared);
            const double Thickness = AInput.OuterRadius - InnerRadius;
            if (Thickness <= CET_SHAPE_EPSILON || Thickness < AInput.OuterRadius * 0.02 || Thickness > AInput.OuterRadius * 0.80)
                return;

            int OuterPointCount = 0;
            int InnerPointCount = 0;
            int BadSideCount = 0;
            double SumError = 0.0;
            double MaxError = 0.0;

            const double SideTolerance = std::max(1.0, AInput.OuterRadius) * 0.02;
            const std::size_t Count = AContour.size();

            for (const auto &Point : AContour) {
                const double PX = static_cast<double>(Point.X);
                const double PY = static_cast<double>(Point.Y);

                double SideValue = AInput.Horizontal ? AInput.SideSign * (PY - AInput.CenterY) : AInput.SideSign * (PX - AInput.CenterX);
                if (SideValue < -SideTolerance)
                    ++BadSideCount;

                const double DistanceToCenter = std::hypot(PX - AInput.CenterX, PY - AInput.CenterY);
                const double OuterError = std::abs(DistanceToCenter - AInput.OuterRadius);
                const double InnerError = std::abs(DistanceToCenter - InnerRadius);
                const double CurrentError = std::min(OuterError, InnerError) / std::max(1.0, AInput.OuterRadius);

                if (OuterError <= InnerError) {
                    ++OuterPointCount;
                } else {
                    ++InnerPointCount;
                }

                SumError += CurrentError;
                MaxError = std::max(MaxError, CurrentError);
            }

            if (OuterPointCount < 3 || InnerPointCount < 3)
                return;

            const int MaxBadSideCount = static_cast<int>(std::ceil(static_cast<double>(Count) * 0.10));
            if (BadSideCount > MaxBadSideCount)
                return;

            const double AverageError = SumError / static_cast<double>(Count);
            if (AverageError > 0.05 || MaxError > 0.15)
                return;

            if (!AInOutBestCandidate.Valid || AverageError < AInOutBestCandidate.FitError) {
                AInOutBestCandidate.Valid = true;
                AInOutBestCandidate.CenterX = AInput.CenterX;
                AInOutBestCandidate.CenterY = AInput.CenterY;
                AInOutBestCandidate.OuterRadius = AInput.OuterRadius;
                AInOutBestCandidate.InnerRadius = InnerRadius;
                AInOutBestCandidate.FitError = AverageError;
                AInOutBestCandidate.ChordAngle = AInput.ChordAngle;
                AInOutBestCandidate.BulgeSign = AInput.SideSign;
                AInOutBestCandidate.ChordStart = AInput.ChordStart;
                AInOutBestCandidate.ChordEnd = AInput.ChordEnd;
            }
        }

        bool CetShapeAnalyzer::_FindLongestContourEdge(const CetPath &AContour, std::size_t &AOutStartIndex, std::size_t &AOutEndIndex, double &AOutLength)
        {
            AOutStartIndex = 0;
            AOutEndIndex = AContour.size() > 1 ? 1 : 0;
            AOutLength = 0.0;
            for (std::size_t Index = 0; Index < AContour.size(); ++Index) {
                const std::size_t NextIndex = (Index + 1) % AContour.size();
                const double Length = _Distance(AContour[Index], AContour[NextIndex]);
                if (Length > AOutLength) {
                    AOutLength = Length;
                    AOutStartIndex = Index;
                    AOutEndIndex = NextIndex;
                }
            }
            return AOutLength > CET_SHAPE_EPSILON;
        }

        bool CetShapeAnalyzer::_AnalyzeSolidArcFeature(const CetPath &AContour, TetShapeFeature &AFeature)
        {
            if (AFeature.HasHoles || !AFeature.IsConvex || AContour.size() < 5)
                return false;

            const std::size_t Count = AContour.size();
            std::size_t ChordStartIndex = 0;
            std::size_t ChordEndIndex = 1;
            double MaxEdgeLength = 0.0;

            if (!_FindLongestContourEdge(AContour, ChordStartIndex, ChordEndIndex, MaxEdgeLength))
                return false;

            const ClipperLib::IntPoint &ChordStart = AContour[ChordStartIndex];
            const ClipperLib::IntPoint &ChordEnd = AContour[ChordEndIndex];
            const double ChordDX = static_cast<double>(ChordEnd.X - ChordStart.X);
            const double ChordDY = static_cast<double>(ChordEnd.Y - ChordStart.Y);
            const double ChordLength = std::hypot(ChordDX, ChordDY);

            if (ChordLength <= CET_SHAPE_EPSILON)
                return false;

            const double Radius = ChordLength * 0.5;
            if (Radius <= CET_SHAPE_EPSILON)
                return false;

            double SecondMaxEdgeLength = 0.0;
            for (std::size_t i = 0; i < Count; ++i) {
                const std::size_t NextIndex = (i + 1) % Count;
                if (i == ChordStartIndex)
                    continue;
                const double Length = _Distance(AContour[i], AContour[NextIndex]);
                SecondMaxEdgeLength = std::max(SecondMaxEdgeLength, Length);
            }

            if (SecondMaxEdgeLength > 0.0 && ChordLength < SecondMaxEdgeLength * 1.8)
                return false;

            const double CenterX = (static_cast<double>(ChordStart.X) + static_cast<double>(ChordEnd.X)) * 0.5;
            const double CenterY = (static_cast<double>(ChordStart.Y) + static_cast<double>(ChordEnd.Y)) * 0.5;

            int PositiveSideCount = 0;
            int NegativeSideCount = 0;
            double MaxRadiusError = 0.0;
            double SumRadiusError = 0.0;
            int ArcPointCount = 0;

            for (std::size_t i = 0; i < Count; ++i) {
                if (i == ChordStartIndex || i == ChordEndIndex)
                    continue;

                const ClipperLib::IntPoint &Point = AContour[i];
                const double PX = static_cast<double>(Point.X);
                const double PY = static_cast<double>(Point.Y);

                const double CrossValue = ChordDX * (PY - static_cast<double>(ChordStart.Y)) - ChordDY * (PX - static_cast<double>(ChordStart.X));
                const double CrossTolerance = std::max(1.0, ChordLength) * 0.001;

                if (CrossValue > CrossTolerance) {
                    ++PositiveSideCount;
                } else if (CrossValue < -CrossTolerance) {
                    ++NegativeSideCount;
                }

                const double DistanceToCenter = std::hypot(PX - CenterX, PY - CenterY);
                const double RadiusError = std::abs(DistanceToCenter - Radius) / std::max(1.0, Radius);

                MaxRadiusError = std::max(MaxRadiusError, RadiusError);
                SumRadiusError += RadiusError;
                ++ArcPointCount;
            }

            if (ArcPointCount <= 0)
                return false;
            if (PositiveSideCount > 0 && NegativeSideCount > 0)
                return false;

            const int BulgeSign = PositiveSideCount >= NegativeSideCount ? 1 : -1;
            const double AverageRadiusError = SumRadiusError / static_cast<double>(ArcPointCount);

            if (AverageRadiusError > 0.12 || MaxRadiusError > 0.25)
                return false;

            const double ExpectedArea = 0.5 * CET_CLUSTER_PI * Radius * Radius;
            if (ExpectedArea <= CET_SHAPE_EPSILON)
                return false;

            const double AreaError = std::abs(AFeature.Area - ExpectedArea) / std::max(1.0, ExpectedArea);
            if (AreaError > 0.25)
                return false;

            AFeature.ArcType = MetArcType::SemiCircleLike;
            AFeature.ArcChordStart = ChordStart;
            AFeature.ArcChordEnd = ChordEnd;
            AFeature.ArcCenter = ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(std::llround(CenterX)), static_cast<ClipperLib::cInt>(std::llround(CenterY)));
            AFeature.ArcChordLength = ChordLength;
            AFeature.ArcRadius = Radius;
            AFeature.ArcChordAngle = std::atan2(ChordDY, ChordDX);
            AFeature.ArcSweepAngle = CET_CLUSTER_PI;
            AFeature.ArcBulgeSign = BulgeSign;
            AFeature.ArcFitError = std::max(AverageRadiusError, AreaError);

            std::cout << "[SHAPE][ARC] Index=" << AFeature.OriginalIndex << " Radius=" << AFeature.ArcRadius << " Chord=" << AFeature.ArcChordLength << " FitError=" << AFeature.ArcFitError << " BulgeSign=" << AFeature.ArcBulgeSign << std::endl;

            return true;
        }

        /*
        MetShapeType CetShapeAnalyzer::_ClassifyShape(const TetShapeFeature& AFeature, bool AHasHoles)
        {
            if (AFeature.VertexCount == 3) return MetShapeType::TriangleLike;
            if (AFeature.VertexCount == 4 && AFeature.IsConvex)
                return AFeature.FillRatio >= 0.95 ? MetShapeType::RectangleLike : MetShapeType::QuadrilateralLike;
            if (!AHasHoles && AFeature.IsConvex && AFeature.Circularity >= 0.90 && AFeature.AspectRatio <= 1.10)
                return MetShapeType::CircleLike;
            if (!AHasHoles && AFeature.IsConvex && AFeature.Circularity >= 0.65 && AFeature.AspectRatio > 1.10)
                return MetShapeType::EllipseLike;
            return AFeature.IsConvex ? MetShapeType::ConvexPolygon : MetShapeType::ConcavePolygon;
        }
        */

    } // namespace NEST2DMANAGERLIB
} // namespace ET
