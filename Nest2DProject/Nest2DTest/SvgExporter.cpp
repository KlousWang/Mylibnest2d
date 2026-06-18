#include "SvgExporter.h"

bool CetSvgExporter::Prepare(TetNestOptions& AOptions)
{
    char ExportSvg = 'n';

    std::cout << "Export SVG? y/n: ";
    std::cin >> ExportSvg;

    if (ExportSvg != 'y' && ExportSvg != 'Y') {
        AOptions.ExportSvg = false;
        return true;
    }

    std::string SvgPath;

    std::cout << "Please enter SVG out path, for example Result.svg: ";
    std::cin >> SvgPath;

    if (SvgPath.empty()) {
        std::cout << "SVG path is empty." << std::endl;
        return false;
    }

    AOptions.ExportSvg = true;
    AOptions.SvgPath = SvgPath;

    return true;
}
