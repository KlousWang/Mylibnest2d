#include "pch.h"
#include "ExportPhoto.h"
#include"NestUtils.h"
#include"Nest2D_DataConst.h"
#include"Nest2D_PrivateDataType.h"
#include"Nest2D_SelfFunction.h"
//#include<libnest2d/backends/clipper/geometries.hpp>
//#include<libnest2d/libnest2d.hpp>
#include <libnest2d/utils/svgtools.hpp>
//#include"EtTechCore_Object.h"

using namespace libnest2d;
using namespace ClipperLib;
namespace ET {
	namespace NEST2DMANAGERLIB {
		CetExportPhoto::CetExportPhoto() :CetCoreObject()
		{
		}
		CetExportPhoto::~CetExportPhoto()
		{
		}
		int CetExportPhoto::ExportSvg(const std::vector<TetNestPolygon>& AItems, const TetNestOptions& AOptions, int AUsedBins) 
		{
            std::cout << "[SVG] Board.Enabled = "
                << AOptions.Board.Enabled
                << ", Board.Vertices.size = "
                << AOptions.Board.Vertices.size()
                << std::endl;

            for (size_t i = 0; i < AOptions.Board.Vertices.size(); ++i) {
                std::cout << "[SVG] Board Pt " << i
                    << " = "
                    << AOptions.Board.Vertices[i].X
                    << ", "
                    << AOptions.Board.Vertices[i].Y
                    << std::endl;
            }
			std::cout << "[DLL]this is export svg1" << std::endl;
			if (AItems.empty() || AUsedBins <= 0) return NEST2D_ERR_EXPORT_EMPTY_ITEMS;
			if (AOptions.SvgPath.empty()) return NEST2D_ERR_EXPORT_NO_PATH;

			Box binSize(NestUtils::ToNestCoord(AOptions.BinWidth), NestUtils::ToNestCoord(AOptions.BinHeight));

			using SvgWriter = svg::SVGWriter<PolygonImpl>;
			SvgWriter::Config conf;
			conf.mm_in_coord_units = mm();

			// 处理文件后缀
			std::string basePath = AOptions.SvgPath;
			size_t extPos = basePath.find(".svg");
			if (extPos != std::string::npos) {
				basePath = basePath.substr(0, extPos);
			}

			// 遍历每一块使用到的板材，单独生成一个 SVG (解决堆叠和单文件输出问题)
            for (int currentBin = 0; currentBin < AUsedBins; ++currentBin) {

                SvgWriter svgw(conf);
                svgw.setSize(binSize);

                std::vector<libnest2d::Item> currentBinItems;

                for (const auto& item : AItems) {
                    if (item.Out_bin == currentBin) {
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

                        if (outerPoints.size() < 3) {
                            continue;
                        }

                        if (ClipperLib::Orientation(outerPoints) == false) {
                            std::reverse(outerPoints.begin(), outerPoints.end());
                        }
                        Paths holes;
                        holes.reserve(item.Holes.size());
                        for (const auto& holePts : item.Holes) {
                            if (holePts.size() < 3) {
                                continue;
                            }
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
                        libnest2d::Item svgItem(poly);
                        svgItem.rotation(item.Out_angle);
                        svgItem.translation(Point(NestUtils::ToNestCoord(item.Out_x),NestUtils::ToNestCoord(item.Out_y)));
                        svgItem.binId(item.Out_bin);
                        currentBinItems.push_back(std::move(svgItem));
                    }
                }
                // 关键：空板材不导出，防止 SVGWriter 内部访问空 vector
                if (currentBinItems.empty()) {
                    std::cout << "[WARN] Skip empty bin: " << currentBin << std::endl;
                    continue;
                }
                libnest2d::_PackGroup<libnest2d::PolygonImpl> pgrp(1);
                for (auto& svgItem : currentBinItems) {
                    pgrp[0].emplace_back(svgItem);
                }
                svgw.writePackGroup(pgrp);
                std::string finalPath;
                if(AUsedBins>1)  finalPath = basePath + "_" + std::to_string(currentBin);
                else finalPath = basePath;
                svgw.save(finalPath);
                std::string realSvgPath = finalPath;
                std::ifstream testFile(realSvgPath.c_str(), std::ios::in | std::ios::binary);
                if (!testFile.is_open()) {
                    realSvgPath = finalPath + ".svg";
                }
                else {testFile.close();}
                std::cout << "[SVG] realSvgPath = " << realSvgPath << std::endl;
                if (AOptions.Board.Enabled && AOptions.Board.Vertices.size() >= 3) {
                    std::string boardPath = Nest2DUtils->MakeBoardSvgPath(AOptions.Board,AOptions.BinHeight);
                   Nest2DUtils->InsertTextBeforeSvgEnd(realSvgPath, boardPath);
                }
            }

			return 0;
		}

		int CetExportPhoto::ExportSvgItems(CetTNestItemVector& ANestItems, const TetNestOptions& AOptions, int AUsedBins)
		{
			//std::cout << "[DLL]this is export svg2" << std::endl;
			if (ANestItems.empty() || AUsedBins <= 0) return NEST2D_ERR_EXPORT_EMPTY_ITEMS;
			if (AOptions.SvgPath.empty()) return NEST2D_ERR_EXPORT_NO_PATH;
			const auto binWidth = NestUtils::ToNestCoord(AOptions.BinWidth);
			const auto binHeight = NestUtils::ToNestCoord(AOptions.BinHeight);

			Box binSize(binWidth, binHeight);

			using SvgWriter = svg::SVGWriter<PolygonImpl>;

			SvgWriter::Config conf;
			conf.mm_in_coord_units = mm();

			SvgWriter svgw(conf);
			svgw.setSize(binSize);

			libnest2d::_PackGroup<libnest2d::PolygonImpl> pgrp(AUsedBins);

			for (auto& item : ANestItems) {
				int binId = static_cast<int>(item.binId());
				if (binId >= 0 && binId < AUsedBins) {
					pgrp[static_cast<size_t>(binId)].emplace_back(item);
				}
			}
			std::string basePath = AOptions.SvgPath;
			if (basePath.size() >= 4 && basePath.substr(basePath.size() - 4) == ".svg") {
				basePath = basePath.substr(0, basePath.size() - 4);
			}

			svgw.writePackGroup(pgrp);
			svgw.save(basePath);

			return 0;
		}
        int CetExportPhoto::ExportSvgPackGroup(const CetPackGround& PackGroup,const TetNestOptions& AOptions)
        {
            if (PackGroup.empty()) {
                return NEST2D_ERR_EXPORT_EMPTY_ITEMS;
            }

            if (AOptions.SvgPath.empty()) {
                return NEST2D_ERR_EXPORT_NO_PATH;
            }

            const auto binWidth = NestUtils::ToNestCoord(AOptions.BinWidth);
            const auto binHeight = NestUtils::ToNestCoord(AOptions.BinHeight);

            Box binSize(binWidth, binHeight);

            using SvgWriter = svg::SVGWriter<PolygonImpl>;

            SvgWriter::Config conf;
            conf.mm_in_coord_units = mm();

            std::string basePath = AOptions.SvgPath;
            if (basePath.size() >= 4 && basePath.substr(basePath.size() - 4) == ".svg") {
                basePath = basePath.substr(0, basePath.size() - 4);
            }

            for (std::size_t binIndex = 0; binIndex < PackGroup.size(); ++binIndex)
            {
                if (PackGroup[binIndex].empty()) {
                    std::cout << "[SVG][Filler] Skip empty bin: "
                        << binIndex << std::endl;
                    continue;
                }

                SvgWriter svgw(conf);
                svgw.setSize(binSize);

                libnest2d::_PackGroup<libnest2d::PolygonImpl> singleBinGroup(1);

                for (const auto& itemRef : PackGroup[binIndex])
                {
                    singleBinGroup[0].emplace_back(itemRef);
                }

                svgw.writePackGroup(singleBinGroup);

                std::string finalPath;

                if (PackGroup.size() > 1) {
                    finalPath = basePath + "_" + std::to_string(binIndex);
                }
                else {
                    finalPath = basePath;
                }

                svgw.save(finalPath);

                std::cout << "[SVG][Filler] saved bin "
                    << binIndex
                    << " to "
                    << finalPath
                    << ".svg"
                    << std::endl;
            }

            return Nest2D_Success;
        }
	}
}

