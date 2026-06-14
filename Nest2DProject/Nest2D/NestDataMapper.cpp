#include "pch.h"
#include "NestDataMapper.h"
#include"NestUtils.h"
#include<algorithm>
#include<cmath>
//#include<libnest2d/backends/clipper/geometries.hpp>
//#include <libnest2d/libnest2d.hpp>

using namespace ClipperLib;
using namespace libnest2d;
using TNestCoord = decltype(libnest2d::mm(0.0));

namespace ET {
    namespace NEST2DMANAGERLIB {
        CetNestDataMapper::CetNestDataMapper():CetCoreObject()
        {
        }

        CetNestDataMapper::~CetNestDataMapper()
        {
        }

        void CetNestDataMapper::BuildNestItems(const std::vector<TetNestPolygon>& AItems, CetTNestItemVector& ANestItems)
        {
            std::cout << "this is build NestItems" << std::endl;
            using namespace libnest2d;
            ANestItems.clear();
            ANestItems.reserve(AItems.size());
            //¹¹½¨ÍâÂÖÀª
            for (const auto& item : AItems) {
                Path outerPoints;
                outerPoints.reserve(item.Vertices.size());

                for (const auto& pt : item.Vertices) {
                    outerPoints.push_back(
                        Point(
                            NestUtils::ToNestCoord(pt.X),
                            NestUtils::ToNestCoord(pt.Y)
                        )
                    );
                }

                if (ClipperLib::Orientation(outerPoints) == false) {
                    std::reverse(outerPoints.begin(), outerPoints.end());
                }

                Paths holes;
                holes.reserve(item.Holes.size());

                for (const auto& holePts : item.Holes) {
                    Path innerPoints;
                    innerPoints.reserve(holePts.size());

                    for (const auto& pt : holePts) {
                        innerPoints.push_back(
                            Point(
                                NestUtils::ToNestCoord(pt.X),
                                NestUtils::ToNestCoord(pt.Y)
                            )
                        );
                    }

                    if (ClipperLib::Orientation(innerPoints) == true) {
                        std::reverse(innerPoints.begin(), innerPoints.end());
                    }

                    holes.push_back(std::move(innerPoints));
                }

                PolygonImpl poly(std::move(outerPoints), std::move(holes));

                ANestItems.emplace_back(std::move(poly));
            }
        }

        void CetNestDataMapper::ApplyResults(const CetTNestItemVector& nestItems, std::vector<TetNestPolygon>& AItems)
        {
            double invScale = 1.0 / libnest2d::mm();

            const size_t nestCount = nestItems.size();
            const size_t itemCount = AItems.size();

            if (nestCount != itemCount) {
                std::cout << "[ERROR] ApplyResults size mismatch: nestItems = "
                    << nestCount
                    << ", AItems = "
                    << itemCount
                    << std::endl;
            }

            const size_t count = std::min(nestCount, itemCount);

            for (size_t i = 0; i < count; ++i) {
                auto tr = nestItems[i].translation();

                AItems[i].Out_bin = static_cast<int>(nestItems[i].binId());
                AItems[i].Out_x = static_cast<double>(tr.X) * invScale;
                AItems[i].Out_y = static_cast<double>(tr.Y) * invScale;
                AItems[i].Out_angle = nestItems[i].rotation();
            }
            for (size_t i = count; i < itemCount; ++i) {
                AItems[i].Out_bin = -1;
                AItems[i].Out_x = 0.0;
                AItems[i].Out_y = 0.0;
                AItems[i].Out_angle = 0.0;
            }
        }
    }
}