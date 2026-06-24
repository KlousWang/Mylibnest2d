#pragma once
#include "EtTechCore_Object.h"
#include"Nest2D_DataType.h"
#include<vector>
class CetNestExporterBase:public ET::CORE::CetCoreObject
{
    Inherit_Invoke_Hook(CetNestExporterBase)
public:
    CetNestExporterBase();
    virtual ~CetNestExporterBase();
    virtual bool Prepare(TetNestOptions& AOptions);
    virtual int Export(const TetNestOptions& AOptions, const std::vector<TetNestPolygon>& AItems, const TetNestResult& AResult);
};
