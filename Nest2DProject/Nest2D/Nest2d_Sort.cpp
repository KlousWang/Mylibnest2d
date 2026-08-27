#include "pch.h"
#include "Nest2D_Sort.h"
#include "Nest2D_SelfFunction.h"


namespace ET {
    namespace NEST2DMANAGERLIB {
        CetSort::CetSort() : CetCoreObject() {}

        CetSort::~CetSort() {}

        void CetSort::Sort(std::vector<TetNestPolygon> &AItems)
        {
            std::sort(AItems.begin(), AItems.end(), [&](const TetNestPolygon &ADataa, const TetNestPolygon &ADatab) { return Nest2DUtils->Nest2DGeometryUtils->ComparePolygonAreaDesc(ADataa, ADatab); });
        }

    } // namespace NEST2DMANAGERLIB
} // namespace ET
