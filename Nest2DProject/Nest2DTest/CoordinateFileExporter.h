#pragma once
#include"Nest2D_DataType.h"
#include<iostream>
#include"NestExporterBase.h"
namespace ET {
    namespace NEST2DTESTAPP {
        class CetCoordinateFileExporter : public CetNestExporterBase
        {
        public:
            int Export(const TetNestOptions& AOptions, const std::vector<TetNestPolygon>& AItems, const TetNestResult& AResult) override;

        };
    }
}