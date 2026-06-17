#pragma once
#include"NestTestData_DataType.h"
class CetNestGeometryUtils
{
public:
	CetNestGeometryUtils();
	~CetNestGeometryUtils();

public:
	void OffsetVertices(CetVertices& AVerts, double AX, double AY);
	double CalcSignedArea(const CetVertices& AVerts);
	void NormalizeContourDirection(CetVertices& AVerts, bool AIsHole);
	CetVertices MakeTriangleBySidesVertices(double ASideA,double ASideB,double ASideC);
	CetVertices MakeCircleVertices(double ACX,double ACY,double ARadius,int ASegments,bool AIsHole);
	CetVertices MakeRegularPolygonVertices(double ACX,double ACY,int ASideCount,double ASideLength,bool AIsHole);
	int CalcCircleSegmentsByTolerance(double ARadius,double ATolerance,int AMinSegments = 4,int AMaxSegments = 32);
	int CalcCircleSegmentsAuto(double ARadius,bool AHasOtherItems,double AMinOtherItemSize,double AToleranceRatio);
};

