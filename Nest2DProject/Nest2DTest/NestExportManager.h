#pragma once

#include "Nest2D_DataType.h"
#include "NestExporterBase.h"
#include "SvgExporter.h"
#include "CoordinateFileExporter.h"
#include"EtTechCore_Object.h"
#include <vector>
#include <memory>
class CetNestExportManager : public ET::CORE::CetCoreObject
{
	Inherit_Invoke_Hook(CetNestExportManager)
public:
	CetNestExportManager();
	virtual ~CetNestExportManager();
	bool PrepareAll(TetNestOptions& AOptions);
	int ExportAll(const TetNestOptions& AOptions, const std::vector<TetNestPolygon>& AItems, const TetNestResult& AResult);
private:
	std::vector<std::unique_ptr<CetNestExporterBase>> m_Exporters;
};

