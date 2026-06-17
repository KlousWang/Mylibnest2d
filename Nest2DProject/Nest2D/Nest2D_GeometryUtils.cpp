#include "pch.h"
#include "Nest2D_GeometryUtils.h"

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
    constexpr double PI = 3.14159265358979323846;
    return ARad * 180.0 / PI;
}

bool ET::NEST2DMANAGERLIB::CetGeometryUtils::PointInPolygon(const TetNestPoint& AP, const std::vector<TetNestPoint>& APolygon)
{
    bool Inside = false;
    size_t Count = APolygon.size();

    if (Count < 3) {
        return false;
    }

    for (size_t i = 0, j = Count - 1; i < Count; j = i++) {
        const auto& Pi = APolygon[i];
        const auto& Pj = APolygon[j];

        bool Intersect =
            ((Pi.Y > AP.Y) != (Pj.Y > AP.Y)) &&
            (AP.X < (Pj.X - Pi.X) * (AP.Y - Pi.Y) / (Pj.Y - Pi.Y + 1e-12) + Pi.X);

        if (Intersect) {
            Inside = !Inside;
        }
    }

    return Inside;
}

bool ET::NEST2DMANAGERLIB::CetGeometryUtils::IsPointInsideBoard(const TetNestPoint& AP, const TetNestBoard& ABoard)
{
    if (!PointInPolygon(AP, ABoard.Vertices)) {
        return false;
    }

    for (const auto& Hole : ABoard.Holes) {
        if (PointInPolygon(AP, Hole)) {
            return false;
        }
    }

    return true;
}

void ET::NEST2DMANAGERLIB::CetGeometryUtils::ValidateItemsInsideBoard(std::vector<TetNestPolygon>& AItems, const TetNestBoard& ABoard)
{
    if (!ABoard.Enabled || ABoard.Vertices.size() < 3) {
        return;
    }

    for (auto& Item : AItems) {
        if (Item.Out_bin < 0) {
            continue;
        }

        bool Valid = true;

        for (const auto& P : Item.Vertices) {
            TetNestPoint TP = TransformPoint(
                P,
                Item.Out_x,
                Item.Out_y,
                Item.Out_angle
            );

            if (!IsPointInsideBoard(TP, ABoard)) {
                Valid = false;
                break;
            }
        }

        if (!Valid) {
            Item.Out_bin = -1;
            Item.Out_x = 0.0;
            Item.Out_y = 0.0;
            Item.Out_angle = 0.0;
        }
    }
}
