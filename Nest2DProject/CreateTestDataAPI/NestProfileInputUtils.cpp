#include "pch.h"
#include "NestProfileInputUtils.h"
#include <iostream>

CetNestProfileInputUtils::CetNestProfileInputUtils()
{
    _InitProfileMenu();
}
CetNestProfileInputUtils::~CetNestProfileInputUtils()
{
}
void CetNestProfileInputUtils::_InitProfileMenu()
{
    m_ProfileMenuItems[PROFILE_TRIANGLE_BY_3_SIDES] = {"Triangle by 3 sides",&CetNestProfileInputUtils::ReadTriangleProfile};
    m_ProfileMenuItems[PROFILE_CIRCLE_BY_RADIUS] = {"Circle by radius",&CetNestProfileInputUtils::ReadCircleProfile};

    m_ProfileMenuItems[PROFILE_CUSTOM_POLYGON] = {"Custom polygon by vertices",&CetNestProfileInputUtils::ReadCustomPolygonProfile};

    m_ProfileMenuItems[PROFILE_REGULAR_POLYGON] = {"Regular polygon by side length",&CetNestProfileInputUtils::ReadRegularPolygonProfile};
}
void CetNestProfileInputUtils::_PrintProfileMenu() const
{
    std::cout << std::endl;
    std::cout << "Please select profile type:" << std::endl;

    for (const auto& Item : m_ProfileMenuItems) {
        std::cout << Item.first
            << ". "
            << Item.second.Description
            << std::endl;
    }

    std::cout << "Please enter: ";
}
int CetNestProfileInputUtils::_ReadChoice() const
{
    int Choice = 0;
    std::cin >> Choice;
    return Choice;
}
CetVertices CetNestProfileInputUtils::_ExecuteProfileMenuItem(int AChoice,bool AIsHole,std::string* AProfileName)
{
    auto It = m_ProfileMenuItems.find(AChoice);

    if (It == m_ProfileMenuItems.end()) {
        std::cout << "Invalid profile type." << std::endl;
        return CetVertices();
    }

    if (It->second.Func == nullptr) {
        std::cout << "Profile function is empty." << std::endl;
        return CetVertices();
    }

    return (this->*(It->second.Func))(AIsHole,AProfileName);
}
CetVertices CetNestProfileInputUtils::ReadProfileVertices(bool AIsHole,std::string* AProfileName)
{
    if (AProfileName) {
        *AProfileName = "Polygon";
    }

    _PrintProfileMenu();

    int ShapeType = _ReadChoice();

    return _ExecuteProfileMenuItem(ShapeType,AIsHole,AProfileName);
}
CetVertices CetNestProfileInputUtils::ReadTriangleProfile(bool AIsHole,std::string* AProfileName)
{
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

    CetVertices Verts =
        m_GeometryUtils.MakeTriangleBySidesVertices(A, B, C);

    m_GeometryUtils.OffsetVertices(Verts, OffsetX, OffsetY);
    m_GeometryUtils.NormalizeContourDirection(Verts, AIsHole);

    return Verts;
}
CetVertices CetNestProfileInputUtils::ReadCircleProfile(bool AIsHole,std::string* AProfileName)
{
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

    CetVertices Verts =
        m_GeometryUtils.MakeCircleVertices(
            CX,
            CY,
            Radius,
            Segments,
            AIsHole
        );

    m_GeometryUtils.NormalizeContourDirection(Verts, AIsHole);

    return Verts;
}
CetVertices CetNestProfileInputUtils::ReadCustomPolygonProfile(bool AIsHole,std::string* AProfileName)
{
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

    m_GeometryUtils.NormalizeContourDirection(Verts, AIsHole);

    return Verts;
}
CetVertices CetNestProfileInputUtils::ReadRegularPolygonProfile(bool AIsHole,std::string* AProfileName)
{
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

    CetVertices Verts =
        m_GeometryUtils.MakeRegularPolygonVertices(
            CX,
            CY,
            SideCount,
            SideLength,
            AIsHole
        );

    m_GeometryUtils.NormalizeContourDirection(Verts, AIsHole);

    return Verts;
}