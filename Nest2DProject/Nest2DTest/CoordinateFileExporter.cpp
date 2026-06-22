#include "CoordinateFileExporter.h"
#include "Nest2DTest_SelfFunction.h"
#include "Nest2D_DataType.h"
namespace ET {
    namespace NEST2DTESTAPP {
        int CetCoordinateFileExporter::Export(const TetNestOptions& AOptions, const std::vector<TetNestPolygon>& AItems, const TetNestResult& AResult)
        {
            {
                char ExportFile = 'n';

                std::cout << "Export coordinate file? y/n: ";
                std::cin >> ExportFile;

                if (ExportFile != 'y' && ExportFile != 'Y') {
                    return 0;
                }

                std::string FilePath;

                std::cout << "Please enter File out path, for example Result.txt: ";
                std::cin >> FilePath;

                if (FilePath.empty()) {
                    std::cout << "File path is empty." << std::endl;
                    return -1;
                }

                int SaveCode =
                    Nest2DUtils->SaveCoordinatesFile(
                        FilePath,
                        AOptions,
                        AItems,
                        AResult.UsedBins
                    );

                std::cout << "SaveCoordinatesFile result = "
                    << SaveCode
                    << std::endl;

                return SaveCode;
            }
        }
    }
}
