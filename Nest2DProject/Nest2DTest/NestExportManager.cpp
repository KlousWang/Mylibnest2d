#include "NestExportManager.h"
#include <vector>
#include <memory>
#include <iostream>

CetNestExportManager::CetNestExportManager()
{
    auto* SvgExporter =static_cast<CetNestExporterBase*>(ET::CORE::CetCoreObject::CreateIns("SvgExporter"));
    if (SvgExporter) {
        m_Exporters.emplace_back(std::unique_ptr<CetNestExporterBase>(SvgExporter));
    }
    else {
        std::cout << "Create SvgExporter failed." << std::endl;
    }
    auto* CoordinateExporter =static_cast<CetNestExporterBase*>(ET::CORE::CetCoreObject::CreateIns("CoordinateFileExporter"));
    if (CoordinateExporter) {
        m_Exporters.emplace_back(std::unique_ptr<CetNestExporterBase>(CoordinateExporter));
    }
    else {
        std::cout << "Create CoordinateFileExporter failed." << std::endl;
    }
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

int CetNestExportManager::ExportAll(
    const TetNestOptions& AOptions,
    const std::vector<TetNestPolygon>& AItems,
    const TetNestResult& AResult
)
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