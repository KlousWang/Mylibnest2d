#include "pch.h"
#include "NestTestDataAPI.h"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <cmath>
#include <iomanip>
#include <iostream>


constexpr double PI = 3.14159265358979323846;

namespace ET {
    namespace NESTTESTDATALIB {

        CetNestTestDataAPI::CetNestTestDataAPI():CetCoreObject()
        {
            std::cout << "CetNestTestDataAPI Constructor" << std::endl;
        }

        CetNestTestDataAPI::~CetNestTestDataAPI()
        {
        }

        static void OffsetVertices(CetVertices& AVerts, double AX, double AY)
        {
            for (auto& V : AVerts) {
                V.first += AX;
                V.second += AY;
            }
        }

        static double CalcSignedArea(const CetVertices& AVerts)
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

        static void NormalizeContourDirection(CetVertices& AVerts, bool AIsHole)
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
        static CetVertices MakeTriangleBySidesVertices(double ASideA,double ASideB,double ASideC)
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
        static CetVertices MakeCircleVertices( double ACX, double ACY, double ARadius, int ASegments, bool AIsHole )
        {
            CetVertices Verts;

            if (ARadius <= 0.0) {
                return Verts;
            }

            if (ASegments < 4) {
                ASegments = 4;
            }

            Verts.reserve(ASegments);

            // 外轮廓圆继续用外切多边形，避免排版时圆被缩小
            double UseRadius = ARadius;

            if (!AIsHole) {
                UseRadius = ARadius / std::cos(PI / ASegments);
            }

            if (!AIsHole) {
                // 外轮廓：逆时针
                for (int i = 0; i < ASegments; ++i) {
                    double Angle = 2.0 * PI * i / ASegments;

                    Verts.emplace_back(
                        ACX + UseRadius * std::cos(Angle),
                        ACY + UseRadius * std::sin(Angle)
                    );
                }
            }
            else {
                // 孔洞：顺时针
                for (int i = ASegments - 1; i >= 0; --i) {
                    double Angle = 2.0 * PI * i / ASegments;

                    Verts.emplace_back(
                        ACX + UseRadius * std::cos(Angle),
                        ACY + UseRadius * std::sin(Angle)
                    );
                }
            }

            return Verts;
        }
        static CetVertices MakeRegularPolygonVertices(double ACX,double ACY, int ASideCount, double ASideLength,bool AIsHole )
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
        static CetVertices ReadProfileVertices(bool AIsHole,std::string* AProfileName)
        {
            if (AProfileName) {
                *AProfileName = "Polygon";
            }
            int ShapeType = 0;

            std::cout << std::endl;
            std::cout << "Please select profile type:" << std::endl;
            std::cout << "1. Triangle by 3 sides" << std::endl;
            std::cout << "2. Circle by radius" << std::endl;
            std::cout << "3. Custom polygon by vertices" << std::endl;
            std::cout << "4. Regular polygon by side length" << std::endl;
            std::cout << "Please enter: ";
            std::cin >> ShapeType;

            if (ShapeType == 1) {
                if (AProfileName) {
                    *AProfileName = "Triangle";
                }
                double A = 0.0;
                double B = 0.0;
                double C = 0.0;
                double OffsetX = 0.0;
                double OffsetY = 0.0;

                std::cout << "Please enter side A: ";
                std::cin >> A;

                std::cout << "Please enter side B: ";
                std::cin >> B;

                std::cout << "Please enter side C: ";
                std::cin >> C;

                std::cout << "Please enter offset X: ";
                std::cin >> OffsetX;

                std::cout << "Please enter offset Y: ";
                std::cin >> OffsetY;

                CetVertices Verts = MakeTriangleBySidesVertices(A, B, C);

                OffsetVertices(Verts, OffsetX, OffsetY);
                NormalizeContourDirection(Verts, AIsHole);

                return Verts;
            }

            if (ShapeType == 2) {
                if (AProfileName) {
                    *AProfileName = "Circle";
                }
                double CX = 0.0;
                double CY = 0.0;
                double Radius = 0.0;
                int Segments = 32;

                std::cout << "Please enter center X: ";
                std::cin >> CX;

                std::cout << "Please enter center Y: ";
                std::cin >> CY;

                std::cout << "Please enter radius: ";
                std::cin >> Radius;

                std::cout << "Please enter segments, for example 32: ";
                std::cin >> Segments;

                CetVertices Verts = MakeCircleVertices(
                    CX,
                    CY,
                    Radius,
                    Segments,
                    AIsHole
                );

                NormalizeContourDirection(Verts, AIsHole);

                return Verts;
            }

            if (ShapeType == 3) {
                if (AProfileName) {
                    *AProfileName = "Polygon";
                }
                int PointCount = 0;

                std::cout << "Please enter point count: ";
                std::cin >> PointCount;

                CetVertices Verts;

                if (PointCount < 3) {
                    return Verts;
                }

                Verts.reserve(PointCount);

                for (int i = 0; i < PointCount; ++i) {
                    double X = 0.0;
                    double Y = 0.0;

                    std::cout << "Point " << i + 1 << " X: ";
                    std::cin >> X;

                    std::cout << "Point " << i + 1 << " Y: ";
                    std::cin >> Y;

                    Verts.emplace_back(X, Y);
                }

                NormalizeContourDirection(Verts, AIsHole);

                return Verts;
            }

            if (ShapeType == 4) {
                if (AProfileName) {
                    *AProfileName = "RegularPolygon";
                }

                int SideCount = 0;
                double SideLength = 0.0;
                double CX = 0.0;
                double CY = 0.0;

                std::cout << "Please enter side count: ";
                std::cin >> SideCount;

                std::cout << "Please enter side length: ";
                std::cin >> SideLength;

                std::cout << "Please enter center X: ";
                std::cin >> CX;

                std::cout << "Please enter center Y: ";
                std::cin >> CY;

                CetVertices Verts = MakeRegularPolygonVertices(
                    CX,
                    CY,
                    SideCount,
                    SideLength,
                    AIsHole
                );

                NormalizeContourDirection(Verts, AIsHole);

                return Verts;
            }

            return CetVertices();
        }
        static int CalcCircleSegmentsByTolerance( double ARadius,double ATolerance,int AMinSegments = 4,int AMaxSegments = 32)
        {
            if (ARadius <= 0.0) {
                return AMinSegments;
            }

            if (ATolerance <= 0.0) {
                return AMaxSegments;
            }

            double value = ARadius / (ARadius + ATolerance);

            if (value < -1.0) value = -1.0;
            if (value > 1.0) value = 1.0;

            int segments = static_cast<int>( std::ceil(PI / std::acos(value)));

            if (segments < AMinSegments) {
                segments = AMinSegments;
            }

            if (segments > AMaxSegments) {
                segments = AMaxSegments;
            }

            return segments;
        }
        static int CalcCircleSegmentsAuto( double ARadius,bool AHasOtherItems, double AMinOtherItemSize, double AToleranceRatio = 0.1)
        {
            // 没有其他图形，粗略一点就行
            if (!AHasOtherItems) {
                return 4;
            }
            // 有其他图形，但是没拿到尺寸，给一个平衡值
            if (AMinOtherItemSize <= 0.0) {
                return 16;
            }
            if (AToleranceRatio <= 0.0) {
                AToleranceRatio = 0.1;
            }
            // 允许圆形多边形比真实圆多占“最小零件尺寸的 10%”
            double tolerance = AMinOtherItemSize * AToleranceRatio;

            // 防止 tolerance 太小导致 segments 太大
            if (tolerance < 0.2) {
                tolerance = 0.2;
            }

            // 防止 tolerance 太大导致圆太粗
            if (tolerance > 3.0) {
                tolerance = 3.0;
            }

            return CalcCircleSegmentsByTolerance( ARadius,tolerance, 4,32);
        }
        int CetNestTestDataAPI::Init( double ABinWidth, double ABinHeight,double ASpacing, int ARotations )
        {
            
            m_BinWidth = ABinWidth;
            m_BinHeight = ABinHeight;
            m_Bpacing = ASpacing;
            m_Rotations = ARotations;

            std::cout << "[DLL] Init called. Bin = "
                << m_BinWidth
                << " x "
                << m_BinHeight
                << ", spacing = "
                << m_Bpacing
                << ", rotations = "
                << m_Rotations
                << std::endl;

            return 0;
        }

        void CetNestTestDataAPI::AddPolygon( const TetPolygonData& APoly)
        {
            m_Polygons.push_back(APoly);

            std::cout << "[DLL] AddPolygon called. Current polygon count = "
                << m_Polygons.size()
                << std::endl;
        }

        void CetNestTestDataAPI::AddPolygon(int AId,const std::string& AName,CetVertices&& AVertices)
        {
            TetPolygonData Poly;

            Poly.Id = AId;
            Poly.Name = AName;
            Poly.Vertices = std::move(AVertices);

            
            m_Polygons.emplace_back(std::move(Poly));

            std::cout << "[DLL] AddPolygon move called. Current polygon count = "
                << m_Polygons.size()
                << std::endl;
        }

        void CetNestTestDataAPI::AddRectangle( int AId,double AW,double AH)
        {
            std::cout << "[DLL] AddRectangle called. Id = " << AId<< ", width = " << AW<< ", height = " << AH<< std::endl;

            AddPolygon( AId, "Rectangle",
                {
                    {0, 0},
                    {AW, 0},
                    {AW, AH},
                    {0, AH}
                }
            );
        }

        void CetNestTestDataAPI::AddTriangle( int AId,double ABase,double AHeight, bool ARightAngle )
        {
            std::cout << "[DLL] AddTriangle called. Id = " << AId << ", base = " << ABase << ", height = " << AHeight << ", rightAngle = " << ARightAngle<< std::endl;

            if (ARightAngle) {
                AddPolygon( AId, "Triangle",
                    {
                        {0, 0},
                        {ABase, 0},
                        {0, AHeight}
                    }
                );
            }
            else {
                AddPolygon( AId, "Triangle",
                    {
                        {0, 0},
                        {ABase, 0},
                        {ABase / 2.0, AHeight}
                    });
            }

            std::cout << "[DLL] After AddTriangle, real polygon count = " << m_Polygons.size() << std::endl;
        }

        void CetNestTestDataAPI::AddTriangleBySides(int AId, double ASideA, double ASideB, double ASideC)
        {
            constexpr double EPS = 1e-9;
            double a = ASideA;
            double b = ASideB;
            double c = ASideC;

            if (a <= 0.0 || b <= 0.0 || c <= 0.0)return;
            if (a + b <= c +EPS|| a + c <= b +EPS|| b + c <= a+EPS)return;
            double x = (b * b + c * c - a * a) / (2.0 * c);
            double y2 = b * b - x * x;
            if (y2 < 0.0 && y2 > -EPS) {
                y2 = 0.0;
            }
            if (y2 <= 0.0)return;

            double y = std::sqrt(y2);

            CetVertices Verts = {
                {0.0,0.0},
                {c,0.0},
                {x,y}
            };
            double minX = std::min({ Verts[0].first, Verts[1].first, Verts[2].first });
            double minY = std::min({ Verts[0].second, Verts[1].second, Verts[2].second });
            if (minX < 0.0 || minY < 0.0) {
                for (auto& V : Verts) {
                    V.first -= minX;
                    V.second -= minY;
                }
            }
            AddPolygon(AId, "Triangle", std::move(Verts));

        }

        void CetNestTestDataAPI::AddCircle(int AId,double ARadius,bool AHasOtherItems,double AMinOtherItemSize, double AToleranceRatio)
        {
            if (ARadius <= 0.0) {
                return;
            }

            // ASegments <= 0 表示自动计算
           
             int  ASegments = CalcCircleSegmentsAuto(  ARadius,AHasOtherItems, AMinOtherItemSize , AToleranceRatio);
           
            if (ASegments < 4) {
                ASegments = 4;
            }

            std::cout << "[DLL] AddCircle called. Id = "
                << AId
                << ", radius = "
                << ARadius
                << ", segments = "
                << ASegments
                << std::endl;

            std::vector<std::pair<double, double>> Verts;
            Verts.reserve(ASegments);

            // 外切圆：ARadius 是真实圆半径，多边形包住这个圆
            double vertexRadius = ARadius / std::cos(PI / ASegments);

            for (int i = 0; i < ASegments; ++i) {
                double Angle = 2.0 * PI * i / ASegments;

                Verts.emplace_back( vertexRadius * std::cos(Angle), vertexRadius * std::sin(Angle) );
            }

            AddPolygon(AId, "Circle", std::move(Verts));
        }

        void CetNestTestDataAPI::AddLShape( int AId,double AW,double AH, double ACutW,double ACutH )
        {
            std::cout << "[DLL] AddLShape called. Id = "
                << AId
                << std::endl;

            AddPolygon( AId, "LShape",{
                    {0, 0},
                    {AW, 0},
                    {AW, ACutH},
                    {ACutW, ACutH},
                    {ACutW, AH},
                    {0, AH}
                }
            );
        }

        void CetNestTestDataAPI::AddPolygonWithHoles(int AId, const std::string& AName, CetVertices&& AOuter, std::vector<CetVertices>&& AHoles)
        {
            if (AOuter.size() < 3) {
                return;
            }

            TetPolygonData Poly;

            Poly.Id = AId;
            Poly.Name = AName;
            Poly.Vertices = std::move(AOuter);
            Poly.Holes = std::move(AHoles);
     

            m_Polygons.emplace_back(std::move(Poly));

            std::cout << "[DLL] AddPolygonWithHoles called. Current polygon count = "
                << m_Polygons.size()
                << std::endl;
        }

        void CetNestTestDataAPI::AddCustomShapeWithHolesByInput(int AId)
        {
            std::cout << std::endl;
            std::cout << "Create outer profile:" << std::endl;

            std::string OuterName = "Polygon";
            CetVertices Outer = ReadProfileVertices(false, &OuterName);

            if (Outer.size() < 3) {
                std::cout << "[DLL] Invalid outer profile." << std::endl;
                return;
            }

            int HoleCount = 0;

            std::cout << "Please enter hole count: ";
            std::cin >> HoleCount;

            std::vector<CetVertices> Holes;
            std::vector<std::string> HoleNames;
            
            Holes.reserve(HoleCount);
            HoleNames.reserve(HoleCount);

            for (int i = 0; i < HoleCount; ++i) {
                std::cout << std::endl;
                std::cout << "Create hole " << i + 1 << ":" << std::endl;

                std::string HoleName = "Polygon";
                CetVertices Hole = ReadProfileVertices(true, &HoleName);

                if (Hole.size() < 3) {
                    std::cout << "[DLL] Invalid hole, skipped." << std::endl;
                    continue;
                }

                Holes.push_back(std::move(Hole));
                HoleNames.push_back(HoleName);
            }

            AddPolygonWithHoles(
                AId,
                OuterName,
                std::move(Outer),
                std::move(Holes)
            );
        }

        size_t CetNestTestDataAPI::PolygonCount() const
        {
            std::cout << "[DLL] PolygonCount called. real count = "<< m_Polygons.size() << std::endl;

            return m_Polygons.size();
        }

        void CetNestTestDataAPI::ClearPolygons()
        {
            std::cout << "[DLL] ClearPolygons called. Old count = " << m_Polygons.size() << std::endl;

            m_Polygons.clear();
        }

        std::string CetNestTestDataAPI::ToString() const
        {
            std::ostringstream Oss;

            Oss << std::fixed << std::setprecision(4);

            Oss << "BIN " << m_BinWidth<< " " << m_BinHeight<< "\n";

            Oss << "SPACING " << m_Bpacing << "\n";

            Oss << "ROTATIONS " << m_Rotations << "\n";

            for (const auto& Poly : m_Polygons) {
                if (Poly.Vertices.empty()) {
                    continue;
                }

                Oss << "\nPOLY "<< Poly.Id << " "<< Poly.Vertices.size()<< " "<< Poly.Name<< "\n";

                for (const auto& V : Poly.Vertices) {
                    Oss << V.first<< " "<< V.second << "\n";
                }
                for (std::size_t i = 0; i < Poly.Holes.size(); ++i) {
                    const auto& Hole = Poly.Holes[i];

                    if (Hole.size() < 3) {
                        continue;
                    }

                    //std::string HoleName = "Polygon";

                    //if (i < Poly.HoleNames.size()) {
                    //    HoleName = Poly.HoleNames[i];
                    //}

                    Oss << "HOLE " << Hole.size()  << "\n";

                    for (const auto& V : Hole) {
                        Oss << V.first << " " << V.second << "\n";
                    }
                }

            }

            return Oss.str();
        }

        bool CetNestTestDataAPI::SaveToFile(const std::string& AFilePath) const
        {
            std::cout << "[DLL] SaveToFile called. File = "
                << AFilePath
                << ", polygon count = "
                << m_Polygons.size()
                << std::endl;

            std::ofstream Out(AFilePath.c_str());

            if (!Out.is_open()) {
                std::cout << "[DLL] Failed to open output file: " << AFilePath<< std::endl;

                return false;
            }

            std::string Content = ToString();

            Out << Content;

            if (!Out.good()) {
                std::cout << "[DLL] Write file failed: "<< AFilePath<< std::endl;

                return false;
            }

            std::cout << "[DLL] Save file success: "<< AFilePath<< std::endl;

            return true;
        }

        static void _ConvexHull(CetVertices& APoints, CetVertices& AHull )
        {
            AHull.clear();
            if (APoints.size() <= 2) {
                AHull = APoints;
                return;
            }
            std::sort(APoints.begin(), APoints.end());
            AHull.reserve(APoints.size() + 1);

            for (const auto& Point : APoints) {
                while (AHull.size() >= 2) {
                    auto& A = AHull[AHull.size() - 2];
                    auto& B = AHull.back();

                    double Cross =
                        (B.first - A.first) * (Point.second - A.second)
                        - (B.second - A.second) * (Point.first - A.first);

                    if (Cross <= 0) {
                        AHull.pop_back();
                    }
                    else {
                        break;
                    }
                }

                AHull.push_back(Point);
            }
            size_t LowerSize = AHull.size();
            for (int i = static_cast<int>(APoints.size()) - 2; i >= 0; --i) {
                const auto& Point = APoints[i];

                while (AHull.size() > LowerSize) {
                    auto& A = AHull[AHull.size() - 2];
                    auto& B = AHull.back();

                    double Cross =
                        (B.first - A.first) * (Point.second - A.second)
                        - (B.second - A.second) * (Point.first - A.first);

                    if (Cross <= 0) {
                        AHull.pop_back();
                    }
                    else {
                        break;
                    }
                }

                AHull.push_back(Point);
            }

            if (AHull.size() > 1) {
                AHull.pop_back();
            }
        }

        void CetNestTestDataAPI::GenerateRandomConvexPolygons(int ACount, int AMinVertices,int AMaxVertices,double AMaxWidth,double AMaxHeight,unsigned ASeed )
        {
            std::mt19937 Rng(ASeed);

            std::uniform_int_distribution<int> VertDist(AMinVertices, AMaxVertices);

            std::uniform_real_distribution<double> CoordX( 0.0,AMaxWidth );

            std::uniform_real_distribution<double> CoordY(0.0,AMaxHeight);

            int NextId = static_cast<int>(m_Polygons.size()) + 1;
            // 很重要：提前给最终结果容器预留空间
            m_Polygons.reserve(m_Polygons.size() + ACount);
            int MaxGenCount = AMaxVertices * 2 + 2;

            CetVertices Points;
            Points.reserve(MaxGenCount);

            for (int i = 0; i < ACount; ++i) {
                int TargetVerts = VertDist(Rng);

                CetVertices Vertices;
                Vertices.reserve(MaxGenCount);

                for (int Attempt = 0; Attempt < 100; ++Attempt) {
                    Points.clear();

                    int GenCount = TargetVerts * 2 + 2;

                    for (int j = 0; j < GenCount; ++j) {
                        Points.emplace_back( CoordX(Rng), CoordY(Rng) );
                    }

                    _ConvexHull( Points, Vertices );

                    if (Vertices.size() >= static_cast<size_t>(AMinVertices) &&Vertices.size() <= static_cast<size_t>(AMaxVertices)) {
                        break;
                    }
                }

                AddPolygon( NextId++, "ConvexPolygon",std::move(Vertices) );
            }
        }

        void CetNestTestDataAPI::GenerateRandomTriangles(int ACount, double AMaxWidth,double AMaxHeight,unsigned ASeed)
        {
            std::mt19937 Rng(ASeed);

            std::uniform_real_distribution<double> CoordX( 0.0, AMaxWidth);

            std::uniform_real_distribution<double> CoordY( 0.0, AMaxHeight);

            int NextId = static_cast<int>(m_Polygons.size()) + 1;

            for (int i = 0; i < ACount; ++i) {
                AddPolygon( NextId++,"Triangle", {
                        {CoordX(Rng), CoordY(Rng)},
                        {CoordX(Rng), CoordY(Rng)},
                        {CoordX(Rng), CoordY(Rng)}
                    } );
            }
        }

    }
}