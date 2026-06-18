#include "NestExporterBase.h"

CetNestExporterBase::CetNestExporterBase()
{
}

CetNestExporterBase::~CetNestExporterBase()
{
}

bool CetNestExporterBase::Prepare(TetNestOptions& AOptions)
{
	return true;
}

int CetNestExporterBase::Export(const TetNestOptions& AOptions, const std::vector<TetNestPolygon>& AItems, const TetNestResult& AResult)
{
	return 0;
}
