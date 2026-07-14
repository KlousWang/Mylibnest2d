#include "pch.h"
#include "NestGeometryUtils.h"
#include <algorithm>
#include <cmath>

constexpr double PI = 3.14159265358979323846;

CetNestGeometryUtils::CetNestGeometryUtils()
{
}

CetNestGeometryUtils::~CetNestGeometryUtils()
{
}

void CetNestGeometryUtils::OffsetVertices(CetVertices& AVerts,double AX,double AY)
{
    for (auto& V : AVerts) {
        V.first += AX;
        V.second += AY;
    }
}

double CetNestGeometryUtils::CalcSignedArea(const CetVertices& AVerts)
{
    if (AVerts.size() < 3) {
        return 0.0;
    }
    double Area = 0.0;
    for (size_t i = 0; i < AVerts.size(); ++i) {
        const auto& P1 = AVerts[i];
        const auto& P2 = AVerts[(i + 1) % AVerts.size()];

        Area += P1.first * P2.second - P2.first * P1.second;
    }
    return Area * 0.5;
}

void CetNestGeometryUtils::NormalizeContourDirection(CetVertices& AVerts,bool AIsHole)
{
    if (AVerts.size() < 3) {
        return;
    }

    double Area = CalcSignedArea(AVerts);

    // 外轮廓建议逆时针，孔洞建议顺时针
    if (!AIsHole && Area < 0.0) {
        std::reverse(AVerts.begin(), AVerts.end());
    }

    if (AIsHole && Area > 0.0) {
        std::reverse(AVerts.begin(), AVerts.end());
    }
}

CetVertices CetNestGeometryUtils::MakeTriangleBySidesVertices(double ASideA,double ASideB,double ASideC)
{
    constexpr double EPS = 1e-9;
    CetVertices Verts;
    double a = ASideA;
    double b = ASideB;
    double c = ASideC;
    if (a <= 0.0 || b <= 0.0 || c <= 0.0) {
        return Verts;
    }
    if (a + b <= c + EPS ||
        a + c <= b + EPS ||
        b + c <= a + EPS) {
        return Verts;
    }
    double x = (b * b + c * c - a * a) / (2.0 * c);
    double y2 = b * b - x * x;
    if (y2 < 0.0 && y2 > -EPS) {
        y2 = 0.0;
    }
    if (y2 <= 0.0) {
        return Verts;
    }
    double y = std::sqrt(y2);
    Verts = {
        {0.0, 0.0},
        {c, 0.0},
        {x, y}
    };
    double minX = Verts[0].first;
    double minY = Verts[0].second;
    for (const auto& V : Verts) {
        if (V.first < minX) {
            minX = V.first;
        }
        if (V.second < minY) {
            minY = V.second;
        }
    }
    if (minX < 0.0 || minY < 0.0) {
        for (auto& V : Verts) {
            V.first -= minX;
            V.second -= minY;
        }
    }

    return Verts;
}

CetVertices CetNestGeometryUtils::MakeCircleVertices(double ACX,double ACY,double ARadius,int ASegments,bool AIsHole)
{
    CetVertices Verts;

    if (ARadius <= 0.0) {
        return Verts;
    }

    if (ASegments < 4) {
        ASegments = 4;
    }

    Verts.reserve(ASegments);

    double UseRadius = ARadius;

    if (!AIsHole) {
        UseRadius = ARadius / std::cos(PI / ASegments);
    }

    if (!AIsHole) {
        for (int i = 0; i < ASegments; ++i) {
            double Angle = 2.0 * PI * i / ASegments;

            Verts.emplace_back(ACX + UseRadius * std::cos(Angle),ACY + UseRadius * std::sin(Angle));
        }
    }
    else {
        for (int i = ASegments - 1; i >= 0; --i) {
            double Angle = 2.0 * PI * i / ASegments;

            Verts.emplace_back(ACX + UseRadius * std::cos(Angle),ACY + UseRadius * std::sin(Angle));
        }
    }

    return Verts;
}

CetVertices CetNestGeometryUtils::MakeRegularPolygonVertices(double ACX,double ACY,int ASideCount,double ASideLength,bool AIsHole)
{
    CetVertices Verts;

    if (ASideCount < 3 || ASideLength <= 0.0) {
        return Verts;
    }

    double Radius = ASideLength / (2.0 * std::sin(PI / ASideCount));

    Verts.reserve(ASideCount);

    if (!AIsHole) {
        for (int i = 0; i < ASideCount; ++i) {
            double Angle = 2.0 * PI * i / ASideCount;

            Verts.emplace_back(
                ACX + Radius * std::cos(Angle),
                ACY + Radius * std::sin(Angle)
            );
        }
    }
    else {
        for (int i = ASideCount - 1; i >= 0; --i) {
            double Angle = 2.0 * PI * i / ASideCount;

            Verts.emplace_back(
                ACX + Radius * std::cos(Angle),
                ACY + Radius * std::sin(Angle)
            );
        }
    }

    return Verts;
}

int CetNestGeometryUtils::CalcCircleSegmentsByTolerance(double ARadius,double ATolerance,int AMinSegments,int AMaxSegments)
{
    if (ARadius <= 0.0) {
        return AMinSegments;
    }

    if (ATolerance <= 0.0) {
        return AMaxSegments;
    }

    double value = ARadius / (ARadius + ATolerance);

    if (value < -1.0) {
        value = -1.0;
    }

    if (value > 1.0) {
        value = 1.0;
    }

    int segments = static_cast<int>(
        std::ceil(PI / std::acos(value))
        );

    if (segments < AMinSegments) {
        segments = AMinSegments;
    }

    if (segments > AMaxSegments) {
        segments = AMaxSegments;
    }

    return segments;
}

int CetNestGeometryUtils::CalcCircleSegmentsAuto(double ARadius,bool AHasOtherItems,double AMinOtherItemSize,double AToleranceRatio)
{
    if (!AHasOtherItems) {
        return 4;
    }

    if (AMinOtherItemSize <= 0.0) {
        return 16;
    }

    if (AToleranceRatio <= 0.0) {
        AToleranceRatio = 0.1;
    }

    double tolerance = AMinOtherItemSize * AToleranceRatio;

    if (tolerance < 0.2) {
        tolerance = 0.2;
    }

    if (tolerance > 3.0) {
        tolerance = 3.0;
    }

    return CalcCircleSegmentsByTolerance(ARadius,tolerance,32,32);
}

CetVertices CetNestGeometryUtils::MakeArcVertices(const TetArcData& AArcData)
{
     CetVertices Verts;
    if (AArcData.Radius <= 0.0) {
        return Verts;
    }
    int Segments = AArcData.Segments;
    if (Segments < 1) {
        Segments = 1;
    }
    constexpr double PI = 3.14159265358979323846;
    double StartRad = AArcData.StartAngle * PI / 180.0;
    double EndRad = AArcData.EndAngle * PI / 180.0;
    double Delta = EndRad - StartRad;
    Verts.reserve(Segments + 1);
    for (int i = 0; i <= Segments; ++i) {
        double T = static_cast<double>(i) / static_cast<double>(Segments);
        double Angle = StartRad + Delta * T;

        double X = AArcData.CenterX + AArcData.Radius * std::cos(Angle);
        double Y = AArcData.CenterY + AArcData.Radius * std::sin(Angle);

        Verts.emplace_back(X, Y);
    }

    return Verts;
}

CetVertices CetNestGeometryUtils::MakeEllipseVertices(double ACX, double ACY, double ARadiusX, double ARadiusY, int ASegments, bool AIsHole, double ARotationAngle)
{
	CetVertices Verts;

	if (ARadiusX <= 0.0 || ARadiusY <= 0.0) {
		return Verts;
	}

	if (ASegments < 4) {
		ASegments = 4;
	}
	Verts.reserve(ASegments);
	// 外轮廓使用外切多边形，避免近似后的多边形侵入真实椭圆
	double UseRadiusX = ARadiusX;
	double UseRadiusY = ARadiusY;
	if (!AIsHole) {
		double Scale = 1.0 / std::cos(PI / ASegments);
		UseRadiusX *= Scale;
		UseRadiusY *= Scale;
	}
	double RotationRad = ARotationAngle * PI / 180.0;
	double CosRotation = std::cos(RotationRad);
	double SinRotation = std::sin(RotationRad);
	auto AddVertex = [&](int AIndex) {
		double Angle = 2.0 * PI * static_cast<double>(AIndex) / static_cast<double>(ASegments);
		// 未旋转椭圆上的局部坐标
		double LocalX = UseRadiusX * std::cos(Angle);
		double LocalY = UseRadiusY * std::sin(Angle);
		// 绕椭圆中心旋转
		double X =ACX +LocalX * CosRotation -LocalY * SinRotation;
		double Y =ACY +LocalX * SinRotation +LocalY * CosRotation;
		Verts.emplace_back(X, Y);
		};
	if (!AIsHole) {
		// 外轮廓逆时针
		for (int i = 0; i < ASegments; ++i) {
			AddVertex(i);
		}
	}
	else {
		// 孔洞顺时针
		for (int i = ASegments - 1; i >= 0; --i) {
			AddVertex(i);
		}
	}

	return Verts;
}
