#include "pch.h"
#include "ExportPhoto.h"
#include"NestUtils.h"
#include"Nest2D_DataConst.h"
#include"Nest2D_PrivateDataType.h"
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

				// 【保命容器】：防止 SVG 渲染时 Item 被销毁导致野指针崩溃
				std::vector<libnest2d::Item> currentBinItems;

				for (const auto& item : AItems) {
					// 只处理排入当前板材的零件
					if (item.Out_bin == currentBin) {

						Path outerPoints;
						outerPoints.reserve(item.Vertices.size());
						for (const auto& pt : item.Vertices) {
							outerPoints.push_back(Point(NestUtils::ToNestCoord(pt.X), NestUtils::ToNestCoord(pt.Y)));
						}

						// 【关键修复】：导出时也必须清洗方向，否则 SVG 重建的几何中心依旧会错乱飞走！
						if (ClipperLib::Orientation(outerPoints) == false) {
							std::reverse(outerPoints.begin(), outerPoints.end());
						}

						Paths holes;
						holes.reserve(item.Holes.size());
						for (const auto& holePts : item.Holes) {
							Path innerPoints;
							innerPoints.reserve(holePts.size());
							for (const auto& pt : holePts) {
								innerPoints.push_back(Point(NestUtils::ToNestCoord(pt.X), NestUtils::ToNestCoord(pt.Y)));
							}

							if (ClipperLib::Orientation(innerPoints) == true) {
								std::reverse(innerPoints.begin(), innerPoints.end());
							}
							holes.push_back(innerPoints);
						}

						// 构建多边形并转为 SVG Item
						PolygonImpl poly(outerPoints, holes);
						libnest2d::Item svgItem(poly);

						// 应用算法已经算好的排版结果 (旋转 + 平移)
						svgItem.rotation(item.Out_angle);
						svgItem.translation(Point(NestUtils::ToNestCoord(item.Out_x), NestUtils::ToNestCoord(item.Out_y)));
						svgItem.binId(item.Out_bin);

						// 存入保命容器
						currentBinItems.push_back(svgItem);
					}
				}
				libnest2d::_PackGroup<libnest2d::PolygonImpl> pgrp(1);
				for (auto& svgItem : currentBinItems) {
					pgrp[0].emplace_back(svgItem);
				}
				// 写入并保存为多个文件：比如 Result_FromFile_0.svg, Result_FromFile_1.svg ...
				svgw.writePackGroup(pgrp);
				std::string finalPath = basePath + "_" + std::to_string(currentBin);
				svgw.save(finalPath);
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

	}
}

