#pragma once
#include <vector>
#include <string>
#include <fstream>
#include <unordered_map>
#include <tuple>
#include <iomanip>
#include "scalarField.h" // ContourData
#include "../src/utils/Vector.h" // Vec3


// ...
// Vec3哈希与等号重载（仅用于导出OBJ时的点去重）
namespace std {
    template <>
    struct hash<alice2::Vec3> {
        size_t operator()(const alice2::Vec3& v) const {
            // 量化到小数点后6位，防止浮点误差导致重复点
            auto quant = [](float x) { return std::lround(x * 1e6f); };
            size_t h1 = std::hash<int>()(quant(v.x));
            size_t h2 = std::hash<int>()(quant(v.y));
            size_t h3 = std::hash<int>()(quant(v.z));
            return h1 ^ (h2 << 1) ^ (h3 << 2);
        }
    };
    template <>
    struct equal_to<alice2::Vec3> {
        bool operator()(const alice2::Vec3& a, const alice2::Vec3& b) const {
            return std::abs(a.x - b.x) < 1e-6f &&
                   std::abs(a.y - b.y) < 1e-6f &&
                   std::abs(a.z - b.z) < 1e-6f;
        }
    };
}

// 导出塔楼等高线到OBJ
// 参数：
//   layers: 每层的线段集合（m_towerContours）
//   heights: 每层的高度（m_towerLevels）
inline bool exportContoursToOBJ(const std::vector<std::vector<std::pair<alice2::Vec3, alice2::Vec3>>>& layers,
                               const std::vector<float>& heights,
                               const std::string& filename) {
    using alice2::Vec3;
    std::ofstream ofs(filename);
    if (!ofs.is_open()) return false;

    ofs << "# Exported SDF Tower contours\n";
    ofs << std::setprecision(8);

    // 顶点去重表
    std::unordered_map<Vec3, int> vertexMap;
    std::vector<Vec3> vertices;
    std::vector<std::pair<int, int>> lines;

    // 收集所有线段
    for (size_t i = 0; i < layers.size(); ++i) {
        float z = (i < heights.size()) ? heights[i] : 0.0f;
        for (const auto& seg : layers[i]) {
            Vec3 v0 = seg.first;
            Vec3 v1 = seg.second;
            v0.z = z;
            v1.z = z;

            int idx0, idx1;
            auto it0 = vertexMap.find(v0);
            if (it0 == vertexMap.end()) {
                idx0 = (int)vertices.size();
                vertexMap[v0] = idx0;
                vertices.push_back(v0);
            } else {
                idx0 = it0->second;
            }
            auto it1 = vertexMap.find(v1);
            if (it1 == vertexMap.end()) {
                idx1 = (int)vertices.size();
                vertexMap[v1] = idx1;
                vertices.push_back(v1);
            } else {
                idx1 = it1->second;
            }
            lines.emplace_back(idx0, idx1);
        }
    }

    // 写入顶点
    for (const auto& v : vertices) {
        ofs << "v " << v.x << " " << v.y << " " << v.z << "\n";
    }
    // 写入线段
    for (const auto& l : lines) {
        ofs << "l " << (l.first + 1) << " " << (l.second + 1) << "\n";
    }
    ofs.close();
    return true;
}