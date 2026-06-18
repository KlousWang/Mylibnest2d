#pragma once
#include"NestExporterBase.h"
#include<iostream>
class CetSvgExporter : public CetNestExporterBase
{
public:
    bool Prepare(TetNestOptions& AOptions) override;
};

