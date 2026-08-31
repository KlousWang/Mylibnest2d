#include "pch.h"
#include "Nest2D_BoardUtils.h"
#include "Nest2D_RotationUtils.h"
#include "NestUtils.h"
#include <cmath>

// using namespace libnest2d;
ET::NEST2DMANAGERLIB::CetNest2DBoardUtils::CetNest2DBoardUtils() {}
ET::NEST2DMANAGERLIB::CetNest2DBoardUtils::~CetNest2DBoardUtils() {}
bool ET::NEST2DMANAGERLIB::CetNest2DBoardUtils::FitsBin(double AWidth, double AHeight, const TetNestOptions &AOptions)
{
    if (AWidth <= 0.0 || AHeight <= 0.0) {
        return false;
    }
    const double BinWidth = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth));
    const double BinHeight = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinHeight));
    if (BinWidth <= 0.0 || BinHeight <= 0.0) {
        return false;
    }
    const bool FitsNormally = AWidth <= BinWidth && AHeight <= BinHeight;
    const bool QuarterTurnAllowed = CetRotationUtils::IsAllowedRotation(CET_CLUSTER_HALF_PI, AOptions.Rotations, 1e-9);
    const bool FitsAfterRotation = QuarterTurnAllowed && AHeight <= BinWidth && AWidth <= BinHeight;
    return FitsNormally || FitsAfterRotation;
}
TetBoardBounds ET::NEST2DMANAGERLIB::CetNest2DBoardUtils::CalcBoardBoundsLocal(const TetNestBoard &ABoard)
{
    TetBoardBounds B;
    if (!ABoard.Enabled || ABoard.Vertices.size() < 3) {
        return B;
    }
    B.MinX = B.MaxX = ABoard.Vertices[0].X;
    B.MinY = B.MaxY = ABoard.Vertices[0].Y;
    for (const auto &P : ABoard.Vertices) {
        B.MinX = std::min(B.MinX, P.X);
        B.MaxX = std::max(B.MaxX, P.X);
        B.MinY = std::min(B.MinY, P.Y);
        B.MaxY = std::max(B.MaxY, P.Y);
    }
    B.Width = B.MaxX - B.MinX;
    B.Height = B.MaxY - B.MinY;
    B.Valid = B.Width > 0.0 && B.Height > 0.0;
    return B;
}
CetPath ET::NEST2DMANAGERLIB::CetNest2DBoardUtils::BuildPathFromPoints(const std::vector<TetNestPoint> &APoints, double AOffsetX, double AOffsetY, bool AWantOuter)
{
    CetPath Result;
    Result.reserve(APoints.size());
    for (const auto &P : APoints) {
        Result.push_back(Point(NestUtils::ToNestCoord(P.X - AOffsetX), NestUtils::ToNestCoord(P.Y - AOffsetY)));
    }
    if (Result.size() >= 3) {
        bool IsCCW = ClipperLib::Orientation(Result);
        if (AWantOuter && !IsCCW) {
            std::reverse(Result.begin(), Result.end());
        }
        if (!AWantOuter && IsCCW) {
            std::reverse(Result.begin(), Result.end());
        }
    }
    return Result;
}
CetPolygonImpl ET::NEST2DMANAGERLIB::CetNest2DBoardUtils::BuildBinPolygonFromOptions(const TetNestOptions &AOptions, double &AOutBinWidth, double &AOutBinHeight)
{
    if (AOptions.Board.Enabled && AOptions.Board.Vertices.size() >= 3) {
        TetBoardBounds Bounds = CalcBoardBoundsLocal(AOptions.Board);
        if (!Bounds.Valid) {
            throw std::runtime_error("Invalid custom board bounds.");
        }
        AOutBinWidth = Bounds.Width;
        AOutBinHeight = Bounds.Height;
        Path Outer = BuildPathFromPoints(AOptions.Board.Vertices, Bounds.MinX, Bounds.MinY, true);
        Paths Holes;
        Holes.reserve(AOptions.Board.Holes.size());
        for (const auto &Hole : AOptions.Board.Holes) {
            if (Hole.size() < 3) {
                continue;
            }
            Holes.push_back(BuildPathFromPoints(Hole, Bounds.MinX, Bounds.MinY, false));
        }
        return CetPolygonImpl(std::move(Outer), std::move(Holes));
    }
    AOutBinWidth = AOptions.BinWidth;
    AOutBinHeight = AOptions.BinHeight;
    Path Outer;
    Outer.push_back(Point(NestUtils::ToNestCoord(0.0), NestUtils::ToNestCoord(0.0)));
    Outer.push_back(Point(NestUtils::ToNestCoord(AOptions.BinWidth), NestUtils::ToNestCoord(0.0)));
    Outer.push_back(Point(NestUtils::ToNestCoord(AOptions.BinWidth), NestUtils::ToNestCoord(AOptions.BinHeight)));
    Outer.push_back(Point(NestUtils::ToNestCoord(0.0), NestUtils::ToNestCoord(AOptions.BinHeight)));
    if (ClipperLib::Orientation(Outer) == false) {
        std::reverse(Outer.begin(), Outer.end());
    }
    Paths Holes;
    return CetPolygonImpl(std::move(Outer), std::move(Holes));
}
CetPolygonImpl ET::NEST2DMANAGERLIB::CetNest2DBoardUtils::BuildRectangleBinPolygon(double ABinWidth, double ABinHeight)
{
    const auto Width = NestUtils::ToNestCoord(ABinWidth);
    const auto Height = NestUtils::ToNestCoord(ABinHeight);
    ClipperLib::Path Contour;
    Contour.reserve(4);
    Contour.push_back(ClipperLib::IntPoint(0, 0));
    Contour.push_back(ClipperLib::IntPoint(Width, 0));
    Contour.push_back(ClipperLib::IntPoint(Width, Height));
    Contour.push_back(ClipperLib::IntPoint(0, Height));
    if (!ClipperLib::Orientation(Contour)) {
        std::reverse(Contour.begin(), Contour.end());
    }
    ClipperLib::Paths Holes;
    return CetPolygonImpl(std::move(Contour), std::move(Holes));
}
bool ET::NEST2DMANAGERLIB::CetNest2DBoardUtils::BuildBoardPath(const std::vector<TetNestPoint>& V, bool Outer, CetPath& Out)
{ Out.clear(); for (const auto& P:V) Out.push_back({NestUtils::ToNestCoord(P.X),NestUtils::ToNestCoord(P.Y)}); ClipperLib::CleanPolygon(Out,1.0); if(Out.size()<3||std::abs(ClipperLib::Area(Out))<=0.0)return false; if(ClipperLib::Orientation(Out)!=Outer)std::reverse(Out.begin(),Out.end()); return true; }
bool ET::NEST2DMANAGERLIB::CetNest2DBoardUtils::BuildBoardSubjectContours(const TetNestOptions& O, ClipperLib::Paths& Out)
{ Out.clear(); if(O.Board.Enabled&&O.Board.Vertices.size()>=3){CetPath P;if(!BuildBoardPath(O.Board.Vertices,true,P))return false;Out.push_back(std::move(P));for(const auto& H:O.Board.Holes){CetPath Q;if(!BuildBoardPath(H,false,Q))return false;Out.push_back(std::move(Q));}return true;} if(O.BinWidth<=0.0||O.BinHeight<=0.0)return false;std::vector<TetNestPoint> R{{0,0},{O.BinWidth,0},{O.BinWidth,O.BinHeight},{0,O.BinHeight}};CetPath P;if(!BuildBoardPath(R,true,P))return false;Out.push_back(std::move(P));return true; }
bool ET::NEST2DMANAGERLIB::CetNest2DBoardUtils::BuildPlacedReservedContours(const CetTNestItemVector& Items,int Bin,double Spacing,ClipperLib::Paths& Out,libnest2d::Coord Extra)
{ Out.clear();const auto Half=static_cast<libnest2d::Coord>(std::ceil(static_cast<double>(NestUtils::ToNestCoord(std::max(0.0,Spacing)))*0.5));for(const auto& Source:Items){if(Source.binId()!=Bin)continue;CetNestItem Item=Source;Item.inflation(Half+std::max<libnest2d::Coord>(0,Extra));CetPolygonImpl Shape=Item.transformedShape();ClipperLib::CleanPolygon(Shape.Contour,1.0);if(Shape.Contour.size()<3||std::abs(ClipperLib::Area(Shape.Contour))<=0.0)return false;if(!ClipperLib::Orientation(Shape.Contour))std::reverse(Shape.Contour.begin(),Shape.Contour.end());Out.push_back(std::move(Shape.Contour));for(CetPath Hole:Shape.Holes){ClipperLib::CleanPolygon(Hole,1.0);if(Hole.size()<3||std::abs(ClipperLib::Area(Hole))<=0.0)return false;if(ClipperLib::Orientation(Hole))std::reverse(Hole.begin(),Hole.end());Out.push_back(std::move(Hole));}}return true; }
