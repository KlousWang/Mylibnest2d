#include "pch.h"
#include "NestTestDataAPI.h"
#include "NestGeometryUtils.h"
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
           
             int  ASegments = m_GeometryUtils.CalcCircleSegmentsAuto(  ARadius,AHasOtherItems, AMinOtherItemSize , AToleranceRatio);
           
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
            CetVertices Outer = m_ProfileInputUtils.ReadProfileVertices(false, &OuterName);

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
                CetVertices Hole = m_ProfileInputUtils.ReadProfileVertices(true, &HoleName);

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

        void CetNestTestDataAPI::AddBoard(CetVertices&& AVertices)
        {
            std::cout << "[DLL] AddBoard called. point count = "
                << AVertices.size()
                << std::endl;

            if (AVertices.size() < 3) {
                std::cout << "[DLL] Invalid board. point count must be >= 3." << std::endl;
                return;
            }

            m_GeometryUtils.NormalizeContourDirection(AVertices, false);

            m_Board.Enabled = true;
            m_Board.Vertices = std::move(AVertices);
            m_Board.Holes.clear();
        }

        void CetNestTestDataAPI::AddBoardWithHoles(CetVertices&& AOuter, std::vector<CetVertices>&& AHoles)
        {
            std::cout << "[DLL] AddBoardWithHoles called. outer point count = "
                << AOuter.size()
                << ", hole count = "
                << AHoles.size()
                << std::endl;

            if (AOuter.size() < 3) {
                std::cout << "[DLL] Invalid board outer. point count must be >= 3." << std::endl;
                return;
            }

            m_GeometryUtils.NormalizeContourDirection(AOuter, false);

            for (auto& Hole : AHoles) {
                if (Hole.size() >= 3) {
                    m_GeometryUtils.NormalizeContourDirection(Hole, true);
                }
            }

            m_Board.Enabled = true;
            m_Board.Vertices = std::move(AOuter);
            m_Board.Holes = std::move(AHoles);
        }

        void CetNestTestDataAPI::ClearBoard()
        {
            std::cout << "[DLL] ClearBoard called." << std::endl;

            m_Board.Enabled = false;
            m_Board.Vertices.clear();
            m_Board.Holes.clear();
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
            ClearBoard();
        }

        std::string CetNestTestDataAPI::ToString() const
        {
            std::ostringstream Oss;

            Oss << std::fixed << std::setprecision(4);

            Oss << "BIN " << m_BinWidth<< " " << m_BinHeight<< "\n";
            if (m_Board.Enabled && m_Board.Vertices.size() >= 3) {
                Oss << "BOARD " << m_Board.Vertices.size() << "\n";
                for (const auto& P : m_Board.Vertices) {
                    Oss << P.first << " " << P.second << "\n";
                }
                for (const auto& Hole : m_Board.Holes) {
                    if (Hole.size() < 3) {
                        continue;
                    }
                    Oss << "BOARD_HOLE " << Hole.size() << "\n";
                    for (const auto& P : Hole) {
                        Oss << P.first << " " << P.second << "\n";
                    }
                }
            }
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
            std::cout << "[DLL] SaveToFile called. File = "<< AFilePath<< ", polygon count = "<< m_Polygons.size()<< std::endl;

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