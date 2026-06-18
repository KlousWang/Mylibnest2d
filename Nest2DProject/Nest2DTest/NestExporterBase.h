#pragma once
#include"Nest2D_DataType.h"
#include<vector>
class CetNestExporterBase
{
public:
    CetNestExporterBase();
    virtual ~CetNestExporterBase();

    virtual bool Prepare(TetNestOptions& AOptions);

    virtual int Export(const TetNestOptions& AOptions, const std::vector<TetNestPolygon>& AItems, const TetNestResult& AResult);
};
