#include "pch.h"
#include "Nest2D_SvgUtils.h"

#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <cstddef>

ET::NEST2DMANAGERLIB::CetSvgUtils::CetSvgUtils()
{
}

ET::NEST2DMANAGERLIB::CetSvgUtils::~CetSvgUtils()
{
}

std::string ET::NEST2DMANAGERLIB::CetSvgUtils::MakeBoardSvgPath(const TetNestBoard& ABoard, double ASvgHeight)
{
    if (!ABoard.Enabled || ABoard.Vertices.size() < 3) {
        return "";
    }

    std::ostringstream ss;

    ss << "<path d=\"";

    ss << "M "
        << ABoard.Vertices[0].X
        << ","
        << (ASvgHeight - ABoard.Vertices[0].Y)
        << " ";

    for (size_t i = 1; i < ABoard.Vertices.size(); ++i) {
        ss << "L "
            << ABoard.Vertices[i].X
            << ","
            << (ASvgHeight - ABoard.Vertices[i].Y)
            << " ";
    }
    ss << "z\" style=\"fill:#1b2a3a;fill-opacity:0.18;stroke:none;stroke-width:2px;\"/>\n";

    return ss.str();
}

void ET::NEST2DMANAGERLIB::CetSvgUtils::InsertTextBeforeSvgEnd(const std::string& AFilePath, const std::string& AText)
{
    if (AText.empty()) {
        return;
    }

    std::ifstream fin(AFilePath.c_str(), std::ios::in | std::ios::binary);

    if (!fin.is_open()) {
        std::cout << "[SVG][WARN] cannot reopen svg: " << AFilePath << std::endl;
        return;
    }

    std::stringstream buffer;
    buffer << fin.rdbuf();
    std::string content = buffer.str();
    fin.close();

    size_t pos = content.rfind("</svg>");

    if (pos == std::string::npos) {
        std::cout << "[SVG][WARN] cannot find </svg> in: " << AFilePath << std::endl;
        return;
    }

    content.insert(pos, AText);

    std::ofstream fout(AFilePath.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);

    if (!fout.is_open()) {
        std::cout << "[SVG][WARN] cannot write svg: " << AFilePath << std::endl;
        return;
    }

    fout << content;
    fout.close();
}
