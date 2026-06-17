#pragma once

#include "NestTestData_DataType.h"
#include "NestGeometryUtils.h"

#include <string>
#include<map>

class CetNestProfileInputUtils
{
public:
    CetNestProfileInputUtils();
    ~CetNestProfileInputUtils();

public:
    CetVertices ReadProfileVertices(bool AIsHole,std::string* AProfileName);

protected:
    enum TetProfileMenuChoice
    {
        PROFILE_TRIANGLE_BY_3_SIDES = 1,
        PROFILE_CIRCLE_BY_RADIUS = 2,
        PROFILE_CUSTOM_POLYGON = 3,
        PROFILE_REGULAR_POLYGON = 4
    };
    using CetProFileFunc = CetVertices(CetNestProfileInputUtils::*)(bool, std::string*);
    struct TetProfileMenuItem
    {
        std::string Description;
        CetProFileFunc Func = nullptr;
    };
    using CetProfileMenuMap = std::map<int, TetProfileMenuItem>;

protected:
    void _InitProfileMenu();

    void _PrintProfileMenu() const;

    int _ReadChoice() const;

    CetVertices _ExecuteProfileMenuItem(int AChoice,bool AIsHole,std::string* AProfileName);

    CetVertices ReadTriangleProfile(bool AIsHole,std::string* AProfileName);

    CetVertices ReadCircleProfile(bool AIsHole,std::string* AProfileName);

    CetVertices ReadCustomPolygonProfile(bool AIsHole,std::string* AProfileName);

    CetVertices ReadRegularPolygonProfile(bool AIsHole,std::string* AProfileName);

protected:
    CetProfileMenuMap m_ProfileMenuItems;
    CetNestGeometryUtils m_GeometryUtils;
};
