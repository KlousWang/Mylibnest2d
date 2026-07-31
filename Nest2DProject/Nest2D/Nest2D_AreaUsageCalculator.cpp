#include "pch.h"
#include "Nest2D_AreaUsageCalculator.h"
#include "NestUtils.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

    ClipperLib::Path TransformContour(const std::vector<TetNestPoint>& APoints, const TetNestPolygon& AItem)
    {
        ClipperLib::Path Result;
        if (APoints.size() < 3){
            return Result;
        }

        Result.reserve(APoints.size());
        const double CosAngle = std::cos(AItem.Out_angle);
        const double SinAngle = std::sin(AItem.Out_angle);
        const double TranslationX = static_cast<double>(NestUtils::ToNestCoord(AItem.Out_x));
        const double TranslationY = static_cast<double>(NestUtils::ToNestCoord(AItem.Out_y));

        for (const TetNestPoint& Point : APoints){
            const double X = static_cast<double>(NestUtils::ToNestCoord(Point.X));
            const double Y = static_cast<double>(NestUtils::ToNestCoord(Point.Y));
            Result.emplace_back(
                static_cast<ClipperLib::cInt>(std::llround(X * CosAngle - Y * SinAngle + TranslationX)),
                static_cast<ClipperLib::cInt>(std::llround(X * SinAngle + Y * CosAngle + TranslationY)));
        }

        return Result;
    }

    bool AppendTransformedItemContours(const TetNestPolygon& AItem, ClipperLib::Paths& AContours)
    {
        ClipperLib::Path Outer = TransformContour(AItem.Vertices, AItem);
        if (Outer.size() < 3){
            return false;
        }
        if (!ClipperLib::Orientation(Outer)){
            std::reverse(Outer.begin(), Outer.end());
        }
        AContours.push_back(std::move(Outer));

        for (const auto& HolePoints : AItem.Holes){
            ClipperLib::Path Hole = TransformContour(HolePoints, AItem);
            if (Hole.size() < 3){
                continue;
            }
            if (ClipperLib::Orientation(Hole)){
                std::reverse(Hole.begin(), Hole.end());
            }
            AContours.push_back(std::move(Hole));
        }

        return true;
    }

    double CalculateUnionArea(const ClipperLib::Paths& AContours)
    {
        if (AContours.empty()){
            return 0.0;
        }

        ClipperLib::Clipper Clipper;
        if (!Clipper.AddPaths(AContours, ClipperLib::ptSubject, true)){
            return 0.0;
        }

        ClipperLib::Paths UnionContours;
        if (!Clipper.Execute(ClipperLib::ctUnion, UnionContours, ClipperLib::pftNonZero, ClipperLib::pftNonZero)){
            return 0.0;
        }

        double AreaInNestCoords = 0.0;
        for (const auto& Contour : UnionContours){
            AreaInNestCoords += static_cast<double>(ClipperLib::Area(Contour));
        }

        const long double Scale = NestUtils::NestScale();
        const long double Area = std::abs(static_cast<long double>(AreaInNestCoords)) / (Scale * Scale);
        return std::isfinite(static_cast<double>(Area)) ? static_cast<double>(Area) : 0.0;
    }

    double CalculateSpacingAdjustedArea(const ClipperLib::Paths& AContours, double ASpacing)
    {
        if (ASpacing <= 0.0 || AContours.empty()){
            return 0.0;
        }

        // Match libnest2d: each part is expanded by half the requested clearance.
        const auto SpacingCoord = static_cast<double>(NestUtils::ToNestCoord(ASpacing));
        const auto Inflation = static_cast<ClipperLib::cInt>(std::ceil(SpacingCoord * 0.5));
        if (Inflation <= 0){
            return 0.0;
        }

        ClipperLib::Paths OffsetContours;
        ClipperLib::ClipperOffset Offset;
        Offset.AddPaths(AContours, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
        Offset.Execute(OffsetContours, static_cast<double>(Inflation));
        ClipperLib::CleanPolygons(OffsetContours, 1.0);
        return CalculateUnionArea(OffsetContours);
    }
}

namespace ET {
    namespace NEST2DMANAGERLIB {

        CetAreaUsageCalculator::CetAreaUsageCalculator() : CetCoreObject()
        {
        }

        CetAreaUsageCalculator::~CetAreaUsageCalculator()
        {
        }

        double CetAreaUsageCalculator::CalcPointArea(const std::vector<TetNestPoint>& APoints)
        {
            if (APoints.size() < 3){
                return 0.0;
            }

            double Area = 0.0;

            for (std::size_t i = 0, j = APoints.size() - 1; i < APoints.size(); j = i++){
                Area += APoints[j].X * APoints[i].Y - APoints[i].X * APoints[j].Y;
            }

            return Area * 0.5;
        }

        double CetAreaUsageCalculator::CalcNetArea(const std::vector<TetNestPoint>& AOuter, const std::vector<std::vector<TetNestPoint>>& AHoles)
        {
            double Area = std::abs(CalcPointArea(AOuter));

            for (const auto& Hole : AHoles){
                Area -= std::abs(CalcPointArea(Hole));
            }

            return std::max(0.0, Area);
        }

        double CetAreaUsageCalculator::CalcPolygonArea(const TetNestPolygon& APoly)
        {
            return CalcNetArea(APoly.Vertices, APoly.Holes);
        }

        double CetAreaUsageCalculator::CalcBoardArea(const TetNestOptions& AOptions)
        {
            if (AOptions.Board.Enabled && AOptions.Board.Vertices.size() >= 3){
                return CalcNetArea(AOptions.Board.Vertices, AOptions.Board.Holes);
            }

            return std::max(0.0, AOptions.BinWidth) * std::max(0.0, AOptions.BinHeight);
        }

        std::vector<TetBoardUsageResult> CetAreaUsageCalculator::CalculateBoardUsages(const std::vector<TetNestPolygon>& AItems, const TetNestOptions& AOptions, int AUsedBins)
        {
            std::vector<TetBoardUsageResult> Results;

            if (AUsedBins <= 0){
                return Results;
            }

            const double BoardArea = CalcBoardArea(AOptions);
            Results.resize(static_cast<std::size_t>(AUsedBins));
            std::vector<ClipperLib::Paths> BinContours(static_cast<std::size_t>(AUsedBins));

            for (int Bin = 0; Bin < AUsedBins; ++Bin){
                Results[static_cast<std::size_t>(Bin)].BinId = Bin;
                Results[static_cast<std::size_t>(Bin)].BoardArea = BoardArea;
            }

            for (const auto& Item : AItems){
                if (Item.Out_bin < 0 || Item.Out_bin >= AUsedBins){
                    continue;
                }

                TetBoardUsageResult& Usage = Results[static_cast<std::size_t>(Item.Out_bin)];
                Usage.UsedArea += CalcPolygonArea(Item);
                Usage.PartCount += 1;
                AppendTransformedItemContours(Item, BinContours[static_cast<std::size_t>(Item.Out_bin)]);
            }

            for (std::size_t Bin = 0; Bin < Results.size(); ++Bin){
                TetBoardUsageResult& Usage = Results[Bin];
                Usage.PureArea = Usage.UsedArea;
                const double SpacingAdjustedArea = CalculateSpacingAdjustedArea(BinContours[Bin], AOptions.Spacing);
                if (SpacingAdjustedArea > 0.0){
                    Usage.UsedArea = SpacingAdjustedArea;
                }
                if (Usage.BoardArea > 0.0){
                    Usage.UsagePercent = (Usage.UsedArea / Usage.BoardArea) * 100.0;
                    Usage.PureUsagePercnt = (Usage.PureArea / Usage.BoardArea) * 100.0;
                }
            }

            return Results;
        }
    }
}
