#include "NestExportManager.h"
#include <vector>
#include <memory>

CetNestExportManager::CetNestExportManager()
{
    m_Exporters.emplace_back(new CetSvgExporter());
    m_Exporters.emplace_back(new ET::NEST2DTESTAPP::CetCoordinateFileExporter());
}

CetNestExportManager::~CetNestExportManager()
{
}

bool CetNestExportManager::PrepareAll(TetNestOptions& AOptions)
{
	for (auto& Exporter : m_Exporters) {
		if (!Exporter->Prepare(AOptions)) {
			return false;
		}
	}

	return true;
}


int CetNestExportManager::ExportAll(const TetNestOptions& AOptions, const std::vector<TetNestPolygon>& AItems, const TetNestResult& AResult)
{
    int ResultCode = 0;

    for (auto& Exporter : m_Exporters) {
        int Code = Exporter->Export(AOptions, AItems, AResult);

        if (Code != 0) {
            ResultCode = Code;
        }
    }

    return ResultCode;
}