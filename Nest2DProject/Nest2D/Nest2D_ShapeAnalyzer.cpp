#include "pch.h"
#include "Nest2D_ShapeAnalyzer.h"

#include <cmath>
#include <algorithm>
#include <iostream>
namespace ET {
	namespace NEST2DMANAGERLIB {
		constexpr double CET_SHAPE_PI = 3.14159265358979323846;
		constexpr double CET_SHAPE_EPSILON = 1e-9;
     
		CetShapeAnalyzer::CetShapeAnalyzer() :CetCoreObject()
		{
		}
		CetShapeAnalyzer::~CetShapeAnalyzer()
		{
		}
		std::vector<TetShapeFeature> CetShapeAnalyzer::AnalyzeALL(const CetTNestItemVector& AItems)
		{
			std::vector<TetShapeFeature> Features;
			Features.reserve(AItems.size());
			for (int i = 0; i < static_cast<int>(AItems.size()); ++i) {
				TetShapeFeature Feature = _AnalyzeOne(AItems[i], i);
				Features.push_back(std::move(Feature));
			}

			return Features;
		}
        TetShapeFeature CetShapeAnalyzer::_AnalyzeOne(const CetNestItem& AItem, int AOriginalIndex)
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
            for (auto& Hole : Holes) _NormalizePath(Hole);
            Holes.erase(std::remove_if(Holes.begin(), Holes.end(),
                [](const CetPath& H) { return H.size() < 3; }), Holes.end());

            Feature.NormalizedContour = Contour;
            Feature.HasHoles = !Holes.empty();
            Feature.HoleCount = static_cast<int>(Holes.size());
            Feature.VertexCount = static_cast<int>(Contour.size());
            if (Contour.size() < 3) return Feature;

            auto MinX = Contour.front().X, MaxX = Contour.front().X;
            auto MinY = Contour.front().Y, MaxY = Contour.front().Y;
            for (const auto& P : Contour) {
                MinX = std::min(MinX, P.X); MaxX = std::max(MaxX, P.X);
                MinY = std::min(MinY, P.Y); MaxY = std::max(MaxY, P.Y);
            }
            Feature.MinX = static_cast<double>(MinX); Feature.MaxX = static_cast<double>(MaxX);
            Feature.MinY = static_cast<double>(MinY); Feature.MaxY = static_cast<double>(MaxY);
            Feature.Width = static_cast<double>(MaxX - MinX);
            Feature.Height = static_cast<double>(MaxY - MinY);

            double HoleArea = 0.0;
            for (const auto& H : Holes) HoleArea += std::abs(static_cast<double>(ClipperLib::Area(H)));
            Feature.Area = std::max(0.0, std::abs(static_cast<double>(ClipperLib::Area(Contour))) - HoleArea);
            Feature.BoxArea = Feature.Width * Feature.Height;
            Feature.FillRatio = Feature.BoxArea > CET_SHAPE_EPSILON ? Feature.Area / Feature.BoxArea : 0.0;
            const double ShortSide = std::min(Feature.Width, Feature.Height);
            Feature.AspectRatio = ShortSide > CET_SHAPE_EPSILON ? std::max(Feature.Width, Feature.Height) / ShortSide : 0.0;
            Feature.IsConvex = _IsConvex(Contour);

            double Perimeter = _CalculatePerimeter(Contour);
            for (const auto& H : Holes) Perimeter += _CalculatePerimeter(H);
            Feature.Circularity = Perimeter > CET_SHAPE_EPSILON ? 4.0 * CET_SHAPE_PI * Feature.Area / (Perimeter * Perimeter) : 0.0;
            Feature.Circularity = std::clamp(Feature.Circularity, 0.0, 1.0);

            _AnalyzeTriangleFeature(Contour, Feature);
            _AnalyzeRectangleFeature(Contour, Feature);
            _AnalyzeArcFeature(Contour, Feature);
            _AnalyzeEllipseFeature(Contour, Feature);
            Feature.ShapeType = _ClassifyShape(Feature);

            std::cout << "[SHAPE] Index=" << Feature.OriginalIndex
                << " Type=" << static_cast<int>(Feature.ShapeType)
                << " Vertices=" << Feature.VertexCount
                << " W=" << Feature.Width << " H=" << Feature.Height
                << " Fill=" << Feature.FillRatio << std::endl;
            return Feature;
        }

        void CetShapeAnalyzer::_AnalyzeTriangleFeature(const CetPath& AContour, TetShapeFeature& AFeature)
        {
            if (AContour.size() != 3) { return; }

            const double L0 = _Distance(AContour[0], AContour[1]);
            const double L1 = _Distance(AContour[1], AContour[2]);
            const double L2 = _Distance(AContour[2], AContour[0]);

            AFeature.TriangleSides = { L0, L1, L2 };
            std::array<double, 3> SortedSides = AFeature.TriangleSides;
            std::sort(SortedSides.begin(), SortedSides.end());
            AFeature.TriangleSides = SortedSides;

            const auto NearlyEqual = [](double A, double B, double Tolerance) {
                const double Denominator = std::max(1.0, std::max(std::abs(A), std::abs(B)));
                return std::abs(A - B) <= Denominator * Tolerance;
                };

            constexpr double SideTolerance = 0.02;
            const bool Equal01 = NearlyEqual(SortedSides[0], SortedSides[1], SideTolerance);
            const bool Equal12 = NearlyEqual(SortedSides[1], SortedSides[2], SideTolerance);

            if (Equal01 && Equal12) {
                AFeature.TriangleSideType = MetTriangleSideType::Equilateral;
            }
            else if (Equal01 || Equal12 || NearlyEqual(SortedSides[0], SortedSides[2], SideTolerance)) {
                AFeature.TriangleSideType = MetTriangleSideType::Isosceles;
            }
            else {
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
            }
            else if (Difference > 0.0) {
                AFeature.TriangleAngleType = MetTriangleAngleType::Obtuse;
            }
            else {
                AFeature.TriangleAngleType = MetTriangleAngleType::Acute;
            }

            for (std::size_t i = 0; i < AContour.size(); ++i) {
                const auto& Previous = AContour[(i + AContour.size() - 1) % AContour.size()];
                const auto& Current = AContour[i];
                const auto& Next = AContour[(i + 1) % AContour.size()];
                AFeature.TriangleAngles[i] = _AngleAtVertex(Previous, Current, Next);
            }
        }

        void CetShapeAnalyzer::_AnalyzeRectangleFeature(const CetPath& AContour, TetShapeFeature& AFeature)
        {
            if (AContour.size() != 4 || !AFeature.IsConvex) { return; }

            std::array<double, 4> EdgeLengths{};
            std::array<long double, 4> EdgeX{};
            std::array<long double, 4> EdgeY{};

            for (std::size_t i = 0; i < 4; ++i) {
                const auto& A = AContour[i];
                const auto& B = AContour[(i + 1) % 4];

                EdgeX[i] = static_cast<long double>(B.X) - static_cast<long double>(A.X);
                EdgeY[i] = static_cast<long double>(B.Y) - static_cast<long double>(A.Y);
                EdgeLengths[i] = std::sqrt(static_cast<double>(EdgeX[i] * EdgeX[i] + EdgeY[i] * EdgeY[i]));

                if (EdgeLengths[i] <= CET_SHAPE_EPSILON) { return; }
            }

            constexpr double OrthogonalTolerance = 0.035;
            constexpr double LengthTolerance = 0.02;

            for (std::size_t i = 0; i < 4; ++i) {
                const std::size_t Next = (i + 1) % 4;
                const long double Dot = EdgeX[i] * EdgeX[Next] + EdgeY[i] * EdgeY[Next];
                const long double Denominator = static_cast<long double>(EdgeLengths[i] * EdgeLengths[Next]);
                const double NormalizedDot = Denominator > 0.0L ? std::abs(static_cast<double>(Dot / Denominator)) : 1.0;

                if (NormalizedDot > OrthogonalTolerance) { return; }
            }

            auto NearlyEqual = [](double A, double B, double Tolerance) {
                const double Denominator = std::max(1.0, std::max(std::abs(A), std::abs(B)));
                return std::abs(A - B) <= Denominator * Tolerance;
                };

            if (!NearlyEqual(EdgeLengths[0], EdgeLengths[2], LengthTolerance) || !NearlyEqual(EdgeLengths[1], EdgeLengths[3], LengthTolerance)) { return; }

            AFeature.IsRotatedRectangle = true;
            AFeature.OrientedWidth = EdgeLengths[0];
            AFeature.OrientedHeight = EdgeLengths[1];
            AFeature.OrientedAngle = std::atan2(static_cast<double>(EdgeY[0]), static_cast<double>(EdgeX[0]));
            AFeature.OrientedBoxArea = AFeature.OrientedWidth * AFeature.OrientedHeight;
            AFeature.OrientedFillRatio = AFeature.OrientedBoxArea > 0.0 ? AFeature.Area / AFeature.OrientedBoxArea : 0.0;
        }


        void CetShapeAnalyzer::_AnalyzeArcFeature(const CetPath& AContour, TetShapeFeature& AFeature)
        {
            AFeature.ArcType = MetArcType::None;
            AFeature.ArcFitError = 1.0;
            AFeature.ArcBulgeSign = 0;

            if (AFeature.HasHoles) {
                return;
            }

            if (AContour.size() < 5) {
                return;
            }
            /*
             * 厚度半圆弧通常是凹多边形。
             * 例如 radius=20, thickness=4, start=0, end=180。
             */
            if (!AFeature.IsConvex) {
                _AnalyzeThickArcFeature(AContour, AFeature);
                return;
            }

            /*
             * 凸图形继续走原来的实心半圆识别逻辑。
             */
            _AnalyzeSolidArcFeature(AContour, AFeature);
        }

        void CetShapeAnalyzer::_AnalyzeEllipseFeature(const CetPath& AContour, TetShapeFeature& AFeature)
        {
            AFeature.EllipseMajorAxis = 0.0;
            AFeature.EllipseMinorAxis = 0.0;
            AFeature.EllipseAngle = 0.0;
            AFeature.EllipseFitError = 1.0;

            if (AFeature.HasHoles) {
                return;
            }

            if (!AFeature.IsConvex) {
                return;
            }

            // 顶点太少时不认为是椭圆。
            // CAD 中圆/椭圆一般会离散成较多点。
            if (AContour.size() < 8) {
                return;
            }

            if (AFeature.Width <= CET_SHAPE_EPSILON ||
                AFeature.Height <= CET_SHAPE_EPSILON) {
                return;
            }

            const double CenterX =
                (AFeature.MinX + AFeature.MaxX) * 0.5;

            const double CenterY =
                (AFeature.MinY + AFeature.MaxY) * 0.5;

            const double RadiusX = AFeature.Width * 0.5;
            const double RadiusY = AFeature.Height * 0.5;

            if (RadiusX <= CET_SHAPE_EPSILON ||
                RadiusY <= CET_SHAPE_EPSILON) {
                return;
            }

            double SumError = 0.0;
            double MaxError = 0.0;

            for (const ClipperLib::IntPoint& Point : AContour) {
                const double X =
                    (static_cast<double>(Point.X) - CenterX) / RadiusX;

                const double Y =
                    (static_cast<double>(Point.Y) - CenterY) / RadiusY;

                const double EquationValue = X * X + Y * Y;

                const double Error =
                    std::abs(EquationValue - 1.0);

                SumError += Error;
                MaxError = std::max(MaxError, Error);
            }

            const double AverageError =
                SumError / static_cast<double>(AContour.size());

            // 第一阶段只做轴向椭圆识别。
            // 旋转椭圆暂时可以退化为 ConvexPolygon / Single。
            if (AverageError > 0.12 || MaxError > 0.35) {
                return;
            }

            const double MajorAxis =
                std::max(AFeature.Width, AFeature.Height);

            const double MinorAxis =
                std::min(AFeature.Width, AFeature.Height);

            if (MinorAxis <= CET_SHAPE_EPSILON) {
                return;
            }

            AFeature.EllipseMajorAxis = MajorAxis;
            AFeature.EllipseMinorAxis = MinorAxis;

            AFeature.EllipseAngle =
                AFeature.Width >= AFeature.Height ? 0.0 : CET_SHAPE_PI * 0.5;

            AFeature.EllipseFitError = AverageError;

            std::cout << "[SHAPE][ELLIPSE] Index=" << AFeature.OriginalIndex
                << " Major=" << AFeature.EllipseMajorAxis
                << " Minor=" << AFeature.EllipseMinorAxis
                << " FitError=" << AFeature.EllipseFitError
                << std::endl;
        }

        MetShapeType CetShapeAnalyzer::_ClassifyShape(const TetShapeFeature& AFeature)
        {
            if (AFeature.VertexCount < 3) return MetShapeType::Unknown;
            if (AFeature.VertexCount == 3) return MetShapeType::TriangleLike;
            if (AFeature.IsRotatedRectangle) return MetShapeType::RectangleLike;
            if (!AFeature.HasHoles && AFeature.ArcType != MetArcType::None) return MetShapeType::ArcLike;
            if (!AFeature.HasHoles && AFeature.IsConvex && AFeature.Circularity >= 0.88 && AFeature.AspectRatio <= 1.12)
                return MetShapeType::CircleLike;
            if (!AFeature.HasHoles && AFeature.IsConvex && AFeature.EllipseFitError <= 0.12 && AFeature.AspectRatio > 1.08)
                return MetShapeType::EllipseLike;
            if (AFeature.VertexCount == 4 && AFeature.IsConvex) return MetShapeType::QuadrilateralLike;
            return AFeature.IsConvex ? MetShapeType::ConvexPolygon : MetShapeType::ConcavePolygon;
        }

        double CetShapeAnalyzer::_Distance(const ClipperLib::IntPoint& AA, const ClipperLib::IntPoint& AB)
        {
            return std::hypot(static_cast<double>(AB.X - AA.X), static_cast<double>(AB.Y - AA.Y));
        }

        double CetShapeAnalyzer::_AngleAtVertex(const ClipperLib::IntPoint& APeacture, const ClipperLib::IntPoint& ACurrent, const ClipperLib::IntPoint& ANext)
        {
            const double AX = static_cast<double>(APeacture.X - ACurrent.X), AY = static_cast<double>(APeacture.Y - ACurrent.Y);
            const double BX = static_cast<double>(ANext.X - ACurrent.X), BY = static_cast<double>(ANext.Y - ACurrent.Y);
            const double Den = std::hypot(AX, AY) * std::hypot(BX, BY);
            if (Den <= CET_SHAPE_EPSILON) return 0.0;
            return std::acos(std::clamp((AX * BX + AY * BY) / Den, -1.0, 1.0));
        }
	
        double CetShapeAnalyzer::_CalculatePerimeter(const CetPath& AContour)
        {
            const std::size_t Count = AContour.size();
            if (Count < 2) {
                return 0.0;
            }
            long double Perimeter = 0.0L;
            for (std::size_t i = 0; i < Count; ++i) {
                /*
                 * 使用取模让最后一个点连接回第一个点。
                 *
                 * 例如Count=3：
                 * i=0，NextIndex=1
                 * i=1，NextIndex=2
                 * i=2，NextIndex=0
                 */
                const std::size_t NextIndex =(i + 1) % Count;

                /*
                 * 调试阶段使用at()。
                 * 如果以后再次发生越界，能够更容易暴露问题。
                 */
                const ClipperLib::IntPoint& Current =AContour.at(i);
                const ClipperLib::IntPoint& Next =AContour.at(NextIndex);
                const long double DX =static_cast<long double>(Next.X) -static_cast<long double>(Current.X);
                const long double DY =static_cast<long double>(Next.Y) -static_cast<long double>(Current.Y);
                Perimeter +=std::sqrt(DX * DX + DY * DY);
            }

            return static_cast<double>(Perimeter);
        }
		//判断是否是凸多边形
		bool CetShapeAnalyzer::_IsConvex(const CetPath& AContour)
		{
			const std::size_t Count = AContour.size();
			if (Count < 3) {
				return false;
			}
			bool HasPositiveCross = false;
			bool HasNegativeCross = false;
			for (std::size_t i = 0; i < Count; ++i) {
				const ClipperLib::IntPoint& A =AContour[i];
				const ClipperLib::IntPoint& B =AContour[(i + 1) % Count];
				const ClipperLib::IntPoint& C =AContour[(i + 2) % Count];
				const long double ABX =static_cast<long double>(B.X) -static_cast<long double>(A.X);
				const long double ABY =static_cast<long double>(B.Y) -static_cast<long double>(A.Y);
				const long double BCX =static_cast<long double>(C.X) -static_cast<long double>(B.X);
			    const long double BCY =static_cast<long double>(C.Y) -static_cast<long double>(B.Y);
				const long double Cross =ABX * BCY -ABY * BCX;
				//共线点不影响凸性判断
				if (Cross > 0.0l) {
					HasPositiveCross = true;
				}
				else if(Cross<0.0l){
					HasNegativeCross = true;
				}
				//同时出现正交叉和负交叉证明缺陷存在
				if (HasPositiveCross && HasNegativeCross)return false;

			}
			return HasPositiveCross || HasNegativeCross;
		}
        void CetShapeAnalyzer::_NormalizePath(CetPath& APath)
        {
            if (APath.empty()) return;
            // 删除连续重复点
            CetPath Clean;
            Clean.reserve(APath.size());
            for (const auto& Point : APath) {
                if (!Clean.empty() && Clean.back().X == Point.X && Clean.back().Y == Point.Y) continue;
                Clean.push_back(Point);
            }
            // 删除首尾重复闭合点
            if (Clean.size() >= 2 && Clean.front().X == Clean.back().X && Clean.front().Y == Clean.back().Y)
                Clean.pop_back();
            // 删除共线的多余中间点（循环处理直到没有点被删除）
            bool Changed = true;
            while (Changed && Clean.size() > 3) {
                Changed = false;
                CetPath Result;
                const std::size_t Count = Clean.size();
                for (std::size_t i = 0; i < Count; ++i) {
                    const auto& A = Clean[(i + Count - 1) % Count];
                    const auto& B = Clean[i];
                    const auto& C = Clean[(i + 1) % Count];
                    const long double Cross = static_cast<long double>(B.X - A.X) * static_cast<long double>(C.Y - B.Y) - static_cast<long double>(B.Y - A.Y) 
                        * static_cast<long double>(C.X - B.X);
                    if (Cross == 0.0L) { Changed = true; continue; }
                    Result.push_back(B);
                }
                if (Result.size() < 3) break;
                Clean = std::move(Result);
            }
            APath = std::move(Clean);
        }
        bool CetShapeAnalyzer::_AnalyzeThickArcFeature(const CetPath& AContour, TetShapeFeature& AFeature)
        {
            if (AFeature.HasHoles) { return false; }
            if (AFeature.IsConvex) { return false; }
            if (AContour.size() < 5) { return false; }

            if (AFeature.Width <= CET_SHAPE_EPSILON || AFeature.Height <= CET_SHAPE_EPSILON || AFeature.Area <= CET_SHAPE_EPSILON) { return false; }
 
            if (AFeature.AspectRatio < 1.40 || AFeature.AspectRatio > 2.60) { return false; }
            if (AFeature.FillRatio < 0.10 || AFeature.FillRatio > 0.55) { return false; }
            const std::size_t Count = AContour.size();

            TetArcCandidateLocal BestCandidate;

            auto TryCandidate = [&](double ACenterX, double ACenterY, double AOuterRadius, bool AHorizontal, int ASideSign, double AChordAngle, ClipperLib::IntPoint AChordStart, ClipperLib::IntPoint AChordEnd) {
                if (AOuterRadius <= CET_SHAPE_EPSILON) { return; }

                /*
                 * 半圆环面积：
                 * Area = 0.5 * pi * (OuterR^2 - InnerR^2)
                 * 这里通过当前轮廓面积反推出 InnerRadius。
                 */
                const double InnerRadiusSquared = AOuterRadius * AOuterRadius - 2.0 * AFeature.Area / CET_SHAPE_PI;

                if (InnerRadiusSquared <= CET_SHAPE_EPSILON || InnerRadiusSquared >= AOuterRadius * AOuterRadius) { return; }

                const double InnerRadius = std::sqrt(InnerRadiusSquared);
                const double Thickness = AOuterRadius - InnerRadius;

                if (Thickness <= CET_SHAPE_EPSILON) { return; }

                /*
                 * 厚度不能太薄，也不能太厚。
                 * 太薄可能是离散误差；
                 * 太厚可能不是半圆弧，而是普通凹多边形。
                 */
                if (Thickness < AOuterRadius * 0.02 || Thickness > AOuterRadius * 0.80) { return; }

                int OuterPointCount = 0;
                int InnerPointCount = 0;
                int BadSideCount = 0;

                double SumError = 0.0;
                double MaxError = 0.0;

                const double SideTolerance = std::max(1.0, AOuterRadius) * 0.02;

                for (const auto& Point : AContour) {
                    const double PX = static_cast<double>(Point.X);
                    const double PY = static_cast<double>(Point.Y);

                    /*
                     * 半圆弧点应该基本位于圆心的一侧。
                     *
                     * 水平半圆：
                     *   上半圆：PY >= CenterY
                     *   下半圆：PY <= CenterY
                     *
                     * 垂直半圆：
                     *   右半圆：PX >= CenterX
                     *   左半圆：PX <= CenterX
                     */
                    double SideValue = 0.0;

                    if (AHorizontal) {
                        SideValue = ASideSign * (PY - ACenterY);
                    }
                    else {
                        SideValue = ASideSign * (PX - ACenterX);
                    }

                    if (SideValue < -SideTolerance) { ++BadSideCount; }

                    const double DistanceToCenter = std::hypot(PX - ACenterX, PY - ACenterY);
                    const double OuterError = std::abs(DistanceToCenter - AOuterRadius);
                    const double InnerError = std::abs(DistanceToCenter - InnerRadius);
                    const double CurrentError = std::min(OuterError, InnerError) / std::max(1.0, AOuterRadius);

                    if (OuterError <= InnerError) {
                        ++OuterPointCount;
                    }
                    else {
                        ++InnerPointCount;
                    }

                    SumError += CurrentError;
                    MaxError = std::max(MaxError, CurrentError);
                }

                if (OuterPointCount < 3 || InnerPointCount < 3) { return; }

                const int MaxBadSideCount = static_cast<int>(std::ceil(static_cast<double>(Count) * 0.10));

                if (BadSideCount > MaxBadSideCount) { return; }

                const double AverageError = SumError / static_cast<double>(Count);

                /*
                 * 当前测试数据 FitError 大概是 0.00016，
                 * 所以这里 0.05 已经比较宽松。
                 */
                if (AverageError > 0.05 || MaxError > 0.15) { return; }

                if (!BestCandidate.Valid || AverageError < BestCandidate.FitError) {
                    BestCandidate.Valid = true;
                    BestCandidate.CenterX = ACenterX;
                    BestCandidate.CenterY = ACenterY;
                    BestCandidate.OuterRadius = AOuterRadius;
                    BestCandidate.InnerRadius = InnerRadius;
                    BestCandidate.FitError = AverageError;
                    BestCandidate.ChordAngle = AChordAngle;
                    BestCandidate.BulgeSign = ASideSign;
                    BestCandidate.ChordStart = AChordStart;
                    BestCandidate.ChordEnd = AChordEnd;
                }
                };

            const double MinX = AFeature.MinX;
            const double MaxX = AFeature.MaxX;
            const double MinY = AFeature.MinY;
            const double MaxY = AFeature.MaxY;
            const double Width = AFeature.Width;
            const double Height = AFeature.Height;
            const double CenterX = (MinX + MaxX) * 0.5;
            const double CenterY = (MinY + MaxY) * 0.5;

            /*
             * 水平方向半圆弧：
             * Width 约等于 2 * OuterRadius
             * Height 约等于 OuterRadius
             */
            if (Width >= Height * 1.40) {
                const double OuterRadius = Width * 0.5;
                // 上半圆：圆心在下边界附近。
                TryCandidate(CenterX, MinY, OuterRadius, true, 1, 0.0,
                    ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(std::llround(CenterX - OuterRadius)), static_cast<ClipperLib::cInt>(std::llround(MinY))),
                    ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(std::llround(CenterX + OuterRadius)), static_cast<ClipperLib::cInt>(std::llround(MinY)))
                );
                // 下半圆：圆心在上边界附近。
                TryCandidate(CenterX, MaxY, OuterRadius, true, -1, 0.0,
                    ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(std::llround(CenterX - OuterRadius)), static_cast<ClipperLib::cInt>(std::llround(MaxY))),
                    ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(std::llround(CenterX + OuterRadius)), static_cast<ClipperLib::cInt>(std::llround(MaxY)))
                );
            }

            /*
             * 垂直方向半圆弧：
             * Height 约等于 2 * OuterRadius
             * Width 约等于 OuterRadius
             */
            if (Height >= Width * 1.40) {
                const double OuterRadius = Height * 0.5;

                // 右半圆：圆心在左边界附近。
                TryCandidate(MinX, CenterY, OuterRadius, false, 1, CET_SHAPE_PI * 0.5,
                    ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(std::llround(MinX)), static_cast<ClipperLib::cInt>(std::llround(CenterY - OuterRadius))),
                    ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(std::llround(MinX)), static_cast<ClipperLib::cInt>(std::llround(CenterY + OuterRadius)))
                );
                // 左半圆：圆心在右边界附近。
                TryCandidate(MaxX, CenterY, OuterRadius, false, -1, CET_SHAPE_PI * 0.5,
                    ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(std::llround(MaxX)), static_cast<ClipperLib::cInt>(std::llround(CenterY - OuterRadius))),
                    ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(std::llround(MaxX)), static_cast<ClipperLib::cInt>(std::llround(CenterY + OuterRadius)))
                );
            }
            if (!BestCandidate.Valid) { return false; }
            AFeature.ArcType = MetArcType::SemiCircleLike;
            AFeature.ArcCenter = ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(std::llround(BestCandidate.CenterX)), static_cast<ClipperLib::cInt>(std::llround(BestCandidate.CenterY)));
            AFeature.ArcRadius = BestCandidate.OuterRadius;
            AFeature.ArcChordLength = BestCandidate.OuterRadius * 2.0;
            AFeature.ArcChordAngle = BestCandidate.ChordAngle;
            AFeature.ArcSweepAngle = CET_SHAPE_PI;
            AFeature.ArcBulgeSign = BestCandidate.BulgeSign;
            AFeature.ArcFitError = BestCandidate.FitError;
            AFeature.ArcChordStart = BestCandidate.ChordStart;
            AFeature.ArcChordEnd = BestCandidate.ChordEnd;

            std::cout << "[SHAPE][ARC][THICK] Index=" << AFeature.OriginalIndex << " OuterRadius=" << AFeature.ArcRadius << " InnerRadius=" << BestCandidate.InnerRadius << " Chord=" << AFeature.ArcChordLength << " FitError=" << AFeature.ArcFitError << " BulgeSign=" << AFeature.ArcBulgeSign << std::endl;

            return true;
        }

        bool CetShapeAnalyzer::_AnalyzeSolidArcFeature(const CetPath& AContour, TetShapeFeature& AFeature)
        {
            if (AFeature.HasHoles) { return false; }
            if (!AFeature.IsConvex) { return false; }
            if (AContour.size() < 5) { return false; }

            const std::size_t Count = AContour.size();

            // 1. 找最长边，作为半圆的直径弦候选。
            std::size_t ChordStartIndex = 0;
            std::size_t ChordEndIndex = 1;
            double MaxEdgeLength = 0.0;

            for (std::size_t i = 0; i < Count; ++i) {
                const std::size_t NextIndex = (i + 1) % Count;
                const double Length = _Distance(AContour[i], AContour[NextIndex]);
                if (Length > MaxEdgeLength) {
                    MaxEdgeLength = Length;
                    ChordStartIndex = i;
                    ChordEndIndex = NextIndex;
                }
            }

            if (MaxEdgeLength <= CET_SHAPE_EPSILON) { return false; }

            const ClipperLib::IntPoint& ChordStart = AContour[ChordStartIndex];
            const ClipperLib::IntPoint& ChordEnd = AContour[ChordEndIndex];

            const double ChordDX = static_cast<double>(ChordEnd.X - ChordStart.X);
            const double ChordDY = static_cast<double>(ChordEnd.Y - ChordStart.Y);
            const double ChordLength = std::hypot(ChordDX, ChordDY);

            if (ChordLength <= CET_SHAPE_EPSILON) { return false; }

            const double Radius = ChordLength * 0.5;

            if (Radius <= CET_SHAPE_EPSILON) { return false; }

            // 2. 半圆的直径边应该是轮廓里非常明显的最长边。
            // 如果最长边并不明显，说明它更可能是普通多边形或椭圆离散边。
            double SecondMaxEdgeLength = 0.0;

            for (std::size_t i = 0; i < Count; ++i) {
                const std::size_t NextIndex = (i + 1) % Count;
                if (i == ChordStartIndex) { continue; }
                const double Length = _Distance(AContour[i], AContour[NextIndex]);
                SecondMaxEdgeLength = std::max(SecondMaxEdgeLength, Length);
            }

            if (SecondMaxEdgeLength > 0.0 && ChordLength < SecondMaxEdgeLength * 1.8) { return false; }

            // 3. 半圆中心近似为直径中点。
            const double CenterX = (static_cast<double>(ChordStart.X) + static_cast<double>(ChordEnd.X)) * 0.5;
            const double CenterY = (static_cast<double>(ChordStart.Y) + static_cast<double>(ChordEnd.Y)) * 0.5;

            int PositiveSideCount = 0;
            int NegativeSideCount = 0;

            double MaxRadiusError = 0.0;
            double SumRadiusError = 0.0;
            int ArcPointCount = 0;

            for (std::size_t i = 0; i < Count; ++i) {
                if (i == ChordStartIndex || i == ChordEndIndex) { continue; }

                const ClipperLib::IntPoint& Point = AContour[i];
                const double PX = static_cast<double>(Point.X);
                const double PY = static_cast<double>(Point.Y);

                const double CrossValue = ChordDX * (PY - static_cast<double>(ChordStart.Y)) - ChordDY * (PX - static_cast<double>(ChordStart.X));
                const double CrossTolerance = std::max(1.0, ChordLength) * 0.001;

                if (CrossValue > CrossTolerance) {
                    ++PositiveSideCount;
                }
                else if (CrossValue < -CrossTolerance) {
                    ++NegativeSideCount;
                }

                const double DistanceToCenter = std::hypot(PX - CenterX, PY - CenterY);
                const double RadiusError = std::abs(DistanceToCenter - Radius) / std::max(1.0, Radius);

                MaxRadiusError = std::max(MaxRadiusError, RadiusError);
                SumRadiusError += RadiusError;
                ++ArcPointCount;
            }

            if (ArcPointCount <= 0) { return false; }

            // 圆弧点必须基本在直径边同一侧。
            if (PositiveSideCount > 0 && NegativeSideCount > 0) { return false; }

            const int BulgeSign = PositiveSideCount >= NegativeSideCount ? 1 : -1;
            const double AverageRadiusError = SumRadiusError / static_cast<double>(ArcPointCount);

            // 第一阶段容差可以稍微宽一些，避免 CAD 离散误差导致识别失败。
            if (AverageRadiusError > 0.12 || MaxRadiusError > 0.25) { return false; }

            // 4. 面积接近半圆面积。
            const double ExpectedArea = 0.5 * CET_SHAPE_PI * Radius * Radius;

            if (ExpectedArea <= CET_SHAPE_EPSILON) { return false; }

            const double AreaError = std::abs(AFeature.Area - ExpectedArea) / std::max(1.0, ExpectedArea);

            if (AreaError > 0.25) { return false; }

            AFeature.ArcType = MetArcType::SemiCircleLike;
            AFeature.ArcChordStart = ChordStart;
            AFeature.ArcChordEnd = ChordEnd;

            AFeature.ArcCenter = ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(std::llround(CenterX)), static_cast<ClipperLib::cInt>(std::llround(CenterY)));

            AFeature.ArcChordLength = ChordLength;
            AFeature.ArcRadius = Radius;
            AFeature.ArcChordAngle = std::atan2(ChordDY, ChordDX);
            AFeature.ArcSweepAngle = CET_SHAPE_PI;
            AFeature.ArcBulgeSign = BulgeSign;

            AFeature.ArcFitError = std::max(AverageRadiusError, AreaError);

            std::cout << "[SHAPE][ARC] Index=" << AFeature.OriginalIndex << " Radius=" << AFeature.ArcRadius << " Chord=" << AFeature.ArcChordLength << " FitError=" << AFeature.ArcFitError << " BulgeSign=" << AFeature.ArcBulgeSign << std::endl;

            return true;
        }
      /*  MetShapeType CetShapeAnalyzer::_ClassifyShape(const TetShapeFeature& AFeature, bool AHasHoles)
        {
            if (AFeature.VertexCount == 3) return MetShapeType::TriangleLike;
            if (AFeature.VertexCount == 4 && AFeature.IsConvex)
                return AFeature.FillRatio >= 0.95 ? MetShapeType::RectangleLike : MetShapeType::QuadrilateralLike;
            if (!AHasHoles && AFeature.IsConvex && AFeature.Circularity >= 0.90 && AFeature.AspectRatio <= 1.10)
                return MetShapeType::CircleLike;
            if (!AHasHoles && AFeature.IsConvex && AFeature.Circularity >= 0.65 && AFeature.AspectRatio > 1.10)
                return MetShapeType::EllipseLike;
            return AFeature.IsConvex ? MetShapeType::ConvexPolygon : MetShapeType::ConcavePolygon;
        }*/
	}
}
