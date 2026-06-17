#include "pch.h"
#include "Nest2D_BoardUtils.h"
#include"NestUtils.h"
//using namespace libnest2d;
ET::NEST2DMANAGERLIB::CetNest2DBoardUtils::CetNest2DBoardUtils()
{
}

ET::NEST2DMANAGERLIB::CetNest2DBoardUtils::~CetNest2DBoardUtils()
{
}
TetBoardBounds ET::NEST2DMANAGERLIB::CetNest2DBoardUtils:: CalcBoardBoundsLocal(const TetNestBoard& ABoard)
{
	TetBoardBounds B;

	if (!ABoard.Enabled || ABoard.Vertices.size() < 3) {
		return B;
	}

	B.MinX = B.MaxX = ABoard.Vertices[0].X;
	B.MinY = B.MaxY = ABoard.Vertices[0].Y;

	for (const auto& P : ABoard.Vertices) {
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
Path ET::NEST2DMANAGERLIB::CetNest2DBoardUtils::BuildPathFromPoints(const std::vector<TetNestPoint>& APoints, double AOffsetX, double AOffsetY, bool AWantOuter)
{
	Path Result;
	Result.reserve(APoints.size());

	for (const auto& P : APoints) {
		Result.push_back(Point(
			NestUtils::ToNestCoord(P.X - AOffsetX),
			NestUtils::ToNestCoord(P.Y - AOffsetY)
		));
	}

	if (Result.size() >= 3) {
		bool IsCCW = ClipperLib::Orientation(Result);

		// 外轮廓保持 CCW
		if (AWantOuter && !IsCCW) {
			std::reverse(Result.begin(), Result.end());
		}

		// 洞保持 CW
		if (!AWantOuter && IsCCW) {
			std::reverse(Result.begin(), Result.end());
		}
	}

	return Result;
}
PolygonImpl ET::NEST2DMANAGERLIB::CetNest2DBoardUtils::BuildBinPolygonFromOptions(const TetNestOptions& AOptions, double& AOutBinWidth, double& AOutBinHeight)
{
	if (AOptions.Board.Enabled && AOptions.Board.Vertices.size() >= 3) {
		TetBoardBounds Bounds = CalcBoardBoundsLocal(AOptions.Board);

		if (!Bounds.Valid) {
			throw std::runtime_error("Invalid custom board bounds.");
		}
		AOutBinWidth = Bounds.Width;
		AOutBinHeight = Bounds.Height;
		Path Outer = BuildPathFromPoints(
			AOptions.Board.Vertices,
			Bounds.MinX,
			Bounds.MinY,
			true
		);
		Paths Holes;
		Holes.reserve(AOptions.Board.Holes.size());

		for (const auto& Hole : AOptions.Board.Holes) {
			if (Hole.size() < 3) {
				continue;
			}

			Holes.push_back(BuildPathFromPoints(Hole, Bounds.MinX, Bounds.MinY, false));
		}

		return PolygonImpl(std::move(Outer), std::move(Holes));
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

	return PolygonImpl(std::move(Outer), std::move(Holes));
}
