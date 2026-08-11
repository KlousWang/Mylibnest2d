#include "pch.h"
#include "Nest2D_GeometryUtils.h"
#include"Nest2D_PrivateDataType.h"

namespace {
    double CalcContourArea(const std::vector<TetNestPoint>& APoints)
    {
        if (APoints.size() < 3) return 0.0;
        double TwiceArea = 0.0;
        for (std::size_t Index = 0; Index < APoints.size(); ++Index) {
            const TetNestPoint& Current = APoints[Index];
            const TetNestPoint& Next = APoints[(Index + 1) % APoints.size()];
            TwiceArea += Current.X * Next.Y - Next.X * Current.Y;
        }
        return std::abs(TwiceArea) * 0.5;
    }

    double CalcPolygonNetArea(const TetNestPolygon& APolygon)
    {
        double Result = CalcContourArea(APolygon.Vertices);
        for (const std::vector<TetNestPoint>& Hole : APolygon.Holes) {
            Result -= CalcContourArea(Hole);
        }
        return std::max(0.0, Result);
    }

    int CalcAreaBand(double AArea)
    {
        return static_cast<int>(std::floor(std::log2(std::max(1.0, AArea))));
    }
}

ET::NEST2DMANAGERLIB::CetGeometryUtils::CetGeometryUtils()
{
}

ET::NEST2DMANAGERLIB::CetGeometryUtils::~CetGeometryUtils()
{
}

TetNestPoint ET::NEST2DMANAGERLIB::CetGeometryUtils::TransformPoint(const TetNestPoint& AP, double AX, double AY, double AAngle)
{
    TetNestPoint R;

    double C = std::cos(AAngle);
    double S = std::sin(AAngle);

    R.X = AP.X * C - AP.Y * S + AX;
    R.Y = AP.X * S + AP.Y * C + AY;

    return R;
}

double ET::NEST2DMANAGERLIB::CetGeometryUtils::RadToDeg(double ARad)
{
    return ARad * 180.0 / CET_CLUSTER_PI;
}

bool ET::NEST2DMANAGERLIB::CetGeometryUtils::PointInPolygon(const TetNestPoint& AP, const std::vector<TetNestPoint>& APolygon)
{
    bool Inside = false;
    size_t Count = APolygon.size();

    if (Count < 3){
        return false;
    }

    for (size_t i = 0, j = Count - 1; i < Count; j = i++){
        const auto& Pi = APolygon[i];
        const auto& Pj = APolygon[j];

        bool Intersect =
            ((Pi.Y > AP.Y) != (Pj.Y > AP.Y)) &&
            (AP.X < (Pj.X - Pi.X) * (AP.Y - Pi.Y) / (Pj.Y - Pi.Y + 1e-12) + Pi.X);

        if (Intersect){
            Inside = !Inside;
        }
    }

    return Inside;
}

bool ET::NEST2DMANAGERLIB::CetGeometryUtils::IsPointInsideBoard(const TetNestPoint& AP, const TetNestBoard& ABoard)
{
    if (!PointInPolygon(AP, ABoard.Vertices)){
        return false;
    }

    for (const auto& Hole : ABoard.Holes){
        if (PointInPolygon(AP, Hole)){
            return false;
        }
    }

    return true;
}

void ET::NEST2DMANAGERLIB::CetGeometryUtils::ValidateItemsInsideBoard(std::vector<TetNestPolygon>& AItems, const TetNestBoard& ABoard)
{
    if (!ABoard.Enabled || ABoard.Vertices.size() < 3){
        return;
    }

    for (auto& Item : AItems){
        if (Item.Out_bin < 0){
            continue;
        }

        bool Valid = true;

        for (const auto& P : Item.Vertices){
            TetNestPoint TP = TransformPoint(P,Item.Out_x,Item.Out_y,Item.Out_angle);

            if (!IsPointInsideBoard(TP, ABoard)){
                Valid = false;
                break;
            }
        }

        if (!Valid){
            Item.Out_bin = -1;
            Item.Out_x = 0.0;
            Item.Out_y = 0.0;
            Item.Out_angle = 0.0;
        }
    }
}

double ET::NEST2DMANAGERLIB::CetGeometryUtils::CalcPolygonBoundingBoxArea(const TetNestPolygon& APoly)
{
    if (APoly.Vertices.empty()) return 0.0;

    double minX = APoly.Vertices[0].X, maxX = minX;
    double minY = APoly.Vertices[0].Y, maxY = minY;

    for (const auto& pt : APoly.Vertices){
        if (pt.X < minX) minX = pt.X;
        if (pt.X > maxX) maxX = pt.X;
        if (pt.Y < minY) minY = pt.Y;
        if (pt.Y > maxY) maxY = pt.Y;
    }
    return (maxX - minX) * (maxY - minY);
}

bool ET::NEST2DMANAGERLIB::CetGeometryUtils::ComparePolygonAreaDesc(const TetNestPolygon& ADataa, const TetNestPolygon& ADAtab)
{
    const double AreaA = CalcPolygonNetArea(ADataa);
    const double AreaB = CalcPolygonNetArea(ADAtab);
    const int AreaBandA = CalcAreaBand(AreaA);
    const int AreaBandB = CalcAreaBand(AreaB);
    if (AreaBandA != AreaBandB) return AreaBandA > AreaBandB;

    const double BoundingBoxAreaA = CalcPolygonBoundingBoxArea(ADataa);
    const double BoundingBoxAreaB = CalcPolygonBoundingBoxArea(ADAtab);
    const double DensityA = AreaA / std::max(1.0, BoundingBoxAreaA);
    const double DensityB = AreaB / std::max(1.0, BoundingBoxAreaB);
    if (std::abs(DensityA - DensityB) > 1e-9) return DensityA > DensityB;
    if (std::abs(AreaA - AreaB) > 1e-9) return AreaA > AreaB;
    return BoundingBoxAreaA > BoundingBoxAreaB;
}
