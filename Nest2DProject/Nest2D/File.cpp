#include "pch.h"
#include "File.h"
#include "Nest2D_DataConst.h"
#include "Nest2D_SelfFunction.h"
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include<iomanip>
#include<algorithm>
#include<vector>

namespace ET {
    namespace NEST2DMANAGERLIB {
        
        struct TetCircleExportInfo
        {
            bool Valid = false;

            TetNestPoint CenterLocal;
            TetNestPoint CenterWorld;

            double Radius = 0.0;        // 真实圆半径
            double VertexRadius = 0.0;  // 外切多边形顶点半径
            int Segments = 0;
        };
        static bool _IsCircleItem(const TetNestPolygon& AItem)
        {
            return AItem.Name == "Circle";
        }
        static TetNestPoint _ApplyTransform(const TetNestPoint& APt, double AX, double AY, double AAngle)
        {
            double cosA = std::cos(AAngle);
            double sinA = std::sin(AAngle);

            TetNestPoint Result;
            Result.X = AX + APt.X * cosA - APt.Y * sinA;
            Result.Y = AY + APt.X * sinA + APt.Y * cosA;

            return Result;
        }    
        static void _WriteLocalVertices(std::ofstream& AOut, const std::vector<TetNestPoint>& APoints )
        {
            AOut << "LOCAL_VERTICES " << APoints.size() << "\n";

            for (const auto& pt : APoints) {
                AOut << pt.X << " " << pt.Y << "\n";
            }
        }
        static void _WriteWorldVertices( std::ofstream& AOut, const std::vector<TetNestPoint>& APoints, double AX,double AY, double AAngle )
        {
            AOut << "WORLD_VERTICES " << APoints.size() << "\n";

            for (const auto& pt : APoints) {
                TetNestPoint worldPt = _ApplyTransform(pt, AX, AY, AAngle);
                AOut << worldPt.X << " " << worldPt.Y << "\n";
            }
        }
        static  bool _CalcCircleInfoFromPolygon(const TetNestPolygon& AItem, TetCircleExportInfo& AInfo)
        {
            AInfo = TetCircleExportInfo();
            const std::vector<TetNestPoint>& pts = AItem.Vertices;
            if (pts.size() < 4) {
                return false;
            }

            int segments = static_cast<int>(pts.size());
            double cx = 0.0;
            double cy = 0.0;
            for (const auto& pt : pts) {
                cx += pt.X;
                cy += pt.Y;
            }
            cx /= static_cast<double>(segments);
            cy /= static_cast<double>(segments);

            double sumR = 0.0;

            for (const auto& pt : pts) {
                double dx = pt.X - cx;
                double dy = pt.Y - cy;
                sumR += std::sqrt(dx * dx + dy * dy);
            }

            double vertexRadius = sumR / static_cast<double>(segments);

            if (vertexRadius <= 0.0) {
                return false;
            }
            constexpr double PI = 3.14159265358979323846;
            double realRadius = vertexRadius * std::cos(PI / static_cast<double>(segments));
            if (realRadius <= 0.0) {
                return false;
            }
            AInfo.Valid = true;
            AInfo.CenterLocal.X = cx;
            AInfo.CenterLocal.Y = cy;
            AInfo.Radius = realRadius;
            AInfo.VertexRadius = vertexRadius;
            AInfo.Segments = segments;

            AInfo.CenterWorld = _ApplyTransform(AInfo.CenterLocal, AItem.Out_x, AItem.Out_y, AItem.Out_angle);

            return true;
        }
        static bool _CompareNestItemPosition( const TetNestPolygon* A, const TetNestPolygon* B )
        {
            if (A->Out_y != B->Out_y) {
                return A->Out_y < B->Out_y;
            }

            if (A->Out_x != B->Out_x) {
                return A->Out_x < B->Out_x;
            }

            return A->Id < B->Id;
        }
        CetFile::CetFile():CetCoreObject()
        {
            std::cout << "CetFile Constructor" << std::endl;
        }

        CetFile::~CetFile()
        {
        }

        int CetFile::LoadNestCaseFromFile(const std::string& AFilePath, TetNestOptions& AOptions, std::vector<TetNestPolygon>& AItems, std::string* AErrorMessage) 
        {
            std::cout << "[DLL] sizeof(TetNestOptions) = "
                << sizeof(TetNestOptions)
                << std::endl;

            std::cout << "[DLL] offsetof(Board) = "
                << offsetof(TetNestOptions, Board)
                << std::endl;

            std::cout << "[DLL] &AOptions = "
                << &AOptions
                << ", &AOptions.Board = "
                << &AOptions.Board
                << ", &AOptions.Board.Vertices = "
                << &AOptions.Board.Vertices
                << std::endl;

            if (AErrorMessage) {
                AErrorMessage->clear();
            }
            std::ifstream fin(AFilePath);
            if (!fin.is_open()) {
                if (AErrorMessage) {
                    *AErrorMessage = "Failed to open file: " + AFilePath;
                }
                return NEST2D_ERR_FILE_OPEN_FAILED;
            }
            AItems.clear();
            std::string token;
            while (fin >> token) {
                if (token.empty()) {
                    continue;
                }
                // 支持整行注释
                if (token[0] == '#') {
                    std::string dummy;
                    std::getline(fin, dummy);
                    continue;
                }
                if (token == "BIN") {
                    if (!(fin >> AOptions.BinWidth >> AOptions.BinHeight)) {
                        if (AErrorMessage) {
                            *AErrorMessage = "Invalid BIN format. Expected: BIN width height";
                        }
                        return NEST2D_ERR_FILE_BIN_FORMAT;
                    }
                    if (AOptions.BinWidth <= 0 || AOptions.BinHeight <= 0) {
                        if (AErrorMessage) {
                            *AErrorMessage = "Invalid BIN size.";
                        }
                        return NEST2D_ERR_FILE_BIN_SIZE;
                    }
                }
                else if (token == "BOARD") {
                    std::size_t pointCount = 0;

                    if (!(fin >> pointCount)) {
                        if (AErrorMessage) {
                            *AErrorMessage = "Invalid BOARD format. Expected: BOARD pointCount";
                        }
                        return NEST2D_ERR_FILE_UNKNOWN_TOKEN;
                    }

                    if (pointCount < 3) {
                        if (AErrorMessage) {
                            *AErrorMessage = "Board point count must be >= 3.";
                        }
                        return NEST2D_ERR_FILE_UNKNOWN_TOKEN;
                    }

                    AOptions.Board.Enabled = true;
                    AOptions.Board.Vertices.clear();
                    AOptions.Board.Holes.clear();
                    AOptions.Board.Vertices.reserve(pointCount);

                    for (std::size_t i = 0; i < pointCount; ++i) {
                        TetNestPoint pt;

                        if (!(fin >> pt.X >> pt.Y)) {
                            if (AErrorMessage) {
                                *AErrorMessage = "Invalid board coordinate.";
                            }
                            return NEST2D_ERR_FILE_UNKNOWN_TOKEN;
                        }

                        if (!std::isfinite(pt.X) || !std::isfinite(pt.Y)) {
                            if (AErrorMessage) {
                                *AErrorMessage = "Board coordinate is not finite.";
                            }
                            return NEST2D_ERR_FILE_UNKNOWN_TOKEN;
                        }

                        AOptions.Board.Vertices.push_back(pt);
                    }
                }
                else if (token == "BOARD_HOLE") {
                    std::size_t pointCount = 0;

                    if (!(fin >> pointCount)) {
                        if (AErrorMessage) {
                            *AErrorMessage = "Invalid BOARD_HOLE format. Expected: BOARD_HOLE pointCount";
                        }
                        return NEST2D_ERR_FILE_UNKNOWN_TOKEN;
                    }

                    if (pointCount < 3) {
                        if (AErrorMessage) {
                            *AErrorMessage = "Board hole point count must be >= 3.";
                        }
                        return NEST2D_ERR_FILE_UNKNOWN_TOKEN;
                    }

                    AOptions.Board.Enabled = true;

                    std::vector<TetNestPoint> hole;
                    hole.reserve(pointCount);

                    for (std::size_t i = 0; i < pointCount; ++i) {
                        TetNestPoint pt;

                        if (!(fin >> pt.X >> pt.Y)) {
                            if (AErrorMessage) {
                                *AErrorMessage = "Invalid board hole coordinate.";
                            }
                            return NEST2D_ERR_FILE_UNKNOWN_TOKEN;
                        }

                        if (!std::isfinite(pt.X) || !std::isfinite(pt.Y)) {
                            if (AErrorMessage) {
                                *AErrorMessage = "Board hole coordinate is not finite.";
                            }
                            return NEST2D_ERR_FILE_UNKNOWN_TOKEN;
                        }

                        hole.push_back(pt);
                    }

                    AOptions.Board.Holes.push_back(std::move(hole));
                }
                else if (token == "SPACING") {
                    if (!(fin >> AOptions.Spacing)) {
                        if (AErrorMessage) {
                            *AErrorMessage = "Invalid SPACING format. Expected: SPACING value";
                        }
                        return NEST2D_ERR_FILE_SPACING_FMT;
                    }
                    if (AOptions.Spacing < 0) {
                        if (AErrorMessage) {
                            *AErrorMessage = "Invalid SPACING.";
                        }
                        return NEST2D_ERR_FILE_SPACING_VAL;
                    }
                }
                else if (token == "ROTATIONS") {
                    if (!(fin >> AOptions.Rotations)) {
                        if (AErrorMessage) {
                            *AErrorMessage = "Invalid ROTATIONS format. Expected: ROTATIONS count";
                        }
                        return NEST2D_ERR_FILE_ROTATIONS_FMT;
                    }
                    if (AOptions.Rotations <= 0) {
                        if (AErrorMessage) {
                            *AErrorMessage = "Invalid ROTATIONS.";
                        }
                        return NEST2D_ERR_FILE_ROTATIONS_VAL;
                    }
                }
                else if (token == "POLY") {
                    TetNestPolygon poly;
                    std::size_t pointCount = 0;
                    std::string headerRest;
                    std::getline(fin, headerRest);
                    std::istringstream headerStream(headerRest);
                    if (!(headerStream >> poly.Id >> pointCount)) {
                        if (AErrorMessage) {
                            *AErrorMessage = "Invalid POLY format. Expected: POLY id pointCount [name]";
                        }
                        return NEST2D_ERR_FILE_POLY_FMT;
                    }
                    std::string shapeName;
                    if (headerStream >> shapeName) {
                        poly.Name = shapeName;
                    }
                    else {
                        poly.Name = "Polygon";
                    }

                    if (pointCount < 3) {
                        if (AErrorMessage) {
                            *AErrorMessage = "Polygon point count must be >= 3.";
                        }
                        return NEST2D_ERR_FILE_POLY_PTS_FEW;
                    }

                    poly.Vertices.clear();
                    poly.Vertices.reserve(pointCount);

                    for (std::size_t i = 0; i < pointCount; ++i) {
                        TetNestPoint pt;

                        if (!(fin >> pt.X >> pt.Y)) {
                            if (AErrorMessage) {
                                *AErrorMessage = "Invalid polygon coordinate.";
                            }
                            return NEST2D_ERR_FILE_POLY_COORD_FMT;
                        }

                        if (!std::isfinite(pt.X) || !std::isfinite(pt.Y)) {
                            if (AErrorMessage) {
                                *AErrorMessage = "Polygon coordinate is not finite.";
                            }
                            return NEST2D_ERR_FILE_POLY_COORD_INF;
                        }

                        poly.Vertices.push_back(pt);
                    }

                    poly.Out_bin = -1;
                    poly.Out_x = 0.0;
                    poly.Out_y = 0.0;
                    poly.Out_angle = 0.0;

                    AItems.push_back(poly);
                }
                else if (token == "HOLE") {
                    if (AItems.empty()) {
                        if (AErrorMessage) {
                            *AErrorMessage = "HOLE appears before any POLY.";
                        }
                        return NEST2D_ERR_FILE_HOLE_NO_POLY;
                    }
                    std::size_t pointCount = 0;
                    if (!(fin >> pointCount)) {
                        if (AErrorMessage) {
                            *AErrorMessage = "Invalid HOLE format. Expected: HOLE pointCount";
                        }
                        return NEST2D_ERR_FILE_HOLE_FMT;
                    }
                    if (pointCount < 3) {
                        if (AErrorMessage) {
                            *AErrorMessage = "Hole point count must be >= 3.";
                        }
                        return NEST2D_ERR_FILE_HOLE_PTS_FEW;
                    }
                    std::vector<TetNestPoint> hole;
                    hole.reserve(pointCount);
                    for (std::size_t i = 0; i < pointCount; ++i) {
                        TetNestPoint pt;
                        if (!(fin >> pt.X >> pt.Y)) {
                            if (AErrorMessage) {
                                *AErrorMessage = "Invalid hole coordinate.";
                            }
                            return NEST2D_ERR_FILE_HOLE_COORD_FMT;
                        }
                        if (!std::isfinite(pt.X) || !std::isfinite(pt.Y)) {
                            if (AErrorMessage) {
                                *AErrorMessage = "Hole coordinate is not finite.";
                            }
                            return NEST2D_ERR_FILE_HOLE_COORD_INF;
                        }
                        hole.push_back(pt);
                    }
                    AItems.back().Holes.push_back(hole);
                }
                else {
                    if (AErrorMessage) {
                        *AErrorMessage = "Unknown token: " + token;
                    }
                    return NEST2D_ERR_FILE_UNKNOWN_TOKEN;
                }
            }
            if (AItems.empty()) {
                if (AErrorMessage) {
                    *AErrorMessage = "No polygon loaded.";
                }
                return NEST2D_ERR_FILE_NO_POLYGON;
            }
            if (AOptions.BinWidth <= 0 || AOptions.BinHeight <= 0) {
                if (AErrorMessage) {
                    *AErrorMessage = "BIN is missing or invalid.";
                }
                return NEST2D_ERR_FILE_MISSING_BIN;
            }
            return 0;
        }
        int CetFile::SaveNestResultToFile(const std::string& AFilePath, const TetNestOptions& AOptions, const std::vector<TetNestPolygon>& AItems, int AUsedBins)
        {
            if (AFilePath.empty()) {
                return NEST2D_ERR_EXPORT_NO_PATH;
            }
            if (AItems.empty() || AUsedBins <= 0) {
                return NEST2D_ERR_EXPORT_EMPTY_ITEMS;
            }
            std::ofstream Out(AFilePath.c_str());
            if (!Out.is_open()) {
                return NEST2D_ERR_FILE_OPEN_FAILED;
            }
            Out << std::fixed << std::setprecision(4);

            Out << "NEST_RESULT 1\n";
            Out << "UNIT mm\n";
            Out << "BIN " << AOptions.BinWidth << " " << AOptions.BinHeight << "\n";
            if (AOptions.Board.Enabled && AOptions.Board.Vertices.size() >= 3) {
                Out << "BOARD_SHAPE " << AOptions.Board.Vertices.size() << "\n";

                for (const auto& P : AOptions.Board.Vertices) {
                    Out << P.X << " " << P.Y << "\n";
                }

                for (const auto& Hole : AOptions.Board.Holes) {
                    if (Hole.size() < 3) {
                        continue;
                    }

                    Out << "BOARD_SHAPE_HOLE " << Hole.size() << "\n";

                    for (const auto& P : Hole) {
                        Out << P.X << " " << P.Y << "\n";
                    }
                }
            }
            Out << "SPACING " << AOptions.Spacing << "\n";
            Out << "USED_BINS " << AUsedBins << "\n";
            for (int currentBin = 0; currentBin < AUsedBins; ++currentBin) {

                std::vector<const TetNestPolygon*> BinItems;

                for (const auto& item : AItems) {
                    if (item.Out_bin == currentBin) {
                        BinItems.push_back(&item);
                    }
                }
                std::sort(BinItems.begin(),BinItems.end(), _CompareNestItemPosition );
                Out << "\nBOARD " << currentBin << "\n";
                Out << "ITEM_COUNT " << BinItems.size() << "\n";

                for (const TetNestPolygon* item : BinItems) {

                    Out << "\nITEM " << item->Id << " " << item->Name << "\n";

                    Out << "TRANSFORM "
                        << item->Out_x << " "
                        << item->Out_y << " "
                        << std::setprecision(6) << item->Out_angle << " "
                        << std::setprecision(4) << Nest2DUtils->RadToDeg(item->Out_angle)
                        << "\n";

                    Out << "PROFILE\n";

                    if (_IsCircleItem(*item)) {

                        TetCircleExportInfo circleInfo;

                        if (_CalcCircleInfoFromPolygon(*item, circleInfo)) {
                            Out << "OUTER CIRCLE\n";
                            Out << "CENTER_LOCAL " << circleInfo.CenterLocal.X << " "<< circleInfo.CenterLocal.Y << "\n";
                            Out << "CENTER_WORLD "<< circleInfo.CenterWorld.X << " " << circleInfo.CenterWorld.Y << "\n";
                            Out << "RADIUS "<< circleInfo.Radius << "\n";
                    
                        }
                        else {
                            // 如果反推失败，就退回普通多边形导出
                            Out << "OUTER POLYGON\n";
                            _WriteLocalVertices(Out, item->Vertices);
                            _WriteWorldVertices(
                                Out,
                                item->Vertices,
                                item->Out_x,
                                item->Out_y,
                                item->Out_angle
                            );
                        }
                    }
                    else {
                        Out << "OUTER POLYGON\n";
                        _WriteLocalVertices(Out, item->Vertices);
                        _WriteWorldVertices(
                            Out,
                            item->Vertices,
                            item->Out_x,
                            item->Out_y,
                            item->Out_angle
                        );
                    }

                    Out << "HOLE_COUNT " << item->Holes.size() << "\n";

                    for (std::size_t h = 0; h < item->Holes.size(); ++h) {
                        Out << "HOLE " << h + 1 << " POLYGON\n";

                        _WriteLocalVertices(Out, item->Holes[h]);
                        _WriteWorldVertices(
                            Out,
                            item->Holes[h],
                            item->Out_x,
                            item->Out_y,
                            item->Out_angle
                        );
                    }

                    Out << "END_PROFILE\n";
                    Out << "END_ITEM\n";
                }

                Out << "END_BOARD\n";
            }

            if (!Out.good()) {
                return NEST2D_ERR_FILE_OPEN_FAILED;
            }

            return 0;
        }
    }
}