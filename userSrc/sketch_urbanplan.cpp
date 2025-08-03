#include "../include/alice2.h"
#include <algorithm>
#include <unordered_map>
#include <fstream>
#include <sstream>
#include <iostream>
#include <utility>
#include <filesystem>
#include <array>
#include <vector>
#include <random>
#include <string>
#include <ctime>
#include <cmath>
#include <cfloat>
#include <set>
#include <limits>
#include "ColorGrowth.h"
#include "EllipseExpansion.h"

static std::mt19937 globalRng;
using namespace alice2;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

constexpr float mergeEPS = 5.0f;
constexpr float connectRadius = 20.0f;
constexpr float minEdge = 15.0f;
constexpr float angEPSdeg = 15.0f;
// 调大 sigma
constexpr float sigma = 30.0f;
constexpr int   gridN = 120;
Vec3 cpt{ -48.f, -18.f, 0.f };

inline Vec3 z2A(const Vec3& v) { return Vec3(v.x, v.y, v.z); }
inline float lerp(float a, float b, float t) { return a + (b - a) * t; }
inline float deg(float r) { return r * 57.2957795f; }

static void hsv2rgb(float h, float s, float v, float& r, float& g, float& b)
{
    h = fmodf(h, 1.f) * 6.f; int i = int(h);
    float f = h - i, p = v * (1 - s), q = v * (1 - f * s), t = v * (1 - (1 - f) * s);
    switch (i) {
    case 0: r = v; g = t; b = p; break;
    case 1: r = q; g = v; b = p; break;
    case 2: r = p; g = v; b = t; break;
    case 3: r = p; g = q; b = v; break;
    case 4: r = t; g = p; b = v; break;
    default:r = v; g = p; b = q; break;
    }
}

std::vector<Vec3> loadTerrainPoints(const std::string& filename, int stepX = 3, int stepY = 3)
{
    std::vector<Vec3> pts;
    std::ifstream in(filename);
    if (!in.is_open()) {
        //std::cerr << "Cannot open " << filename << "\n";
        return pts;
    }
    std::string line;
    std::vector<std::vector<Vec3>> rows;
    float lastY = std::numeric_limits<float>::max();
    std::vector<Vec3> currentRow;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::string xstr, ystr;
        if (std::getline(ss, xstr, ',') && std::getline(ss, ystr, ',')) {
            float x = std::stof(xstr);
            float y = std::stof(ystr);
            if (currentRow.empty() || std::fabs(y - lastY) < 1e-4) {
                currentRow.emplace_back(x, y, 0.0f);
                lastY = y;
            }
            else {
                rows.push_back(currentRow);
                currentRow.clear();
                currentRow.emplace_back(x, y, 0.0f);
                lastY = y;
            }
        }
    }
    if (!currentRow.empty()) rows.push_back(currentRow);
    for (int rowIdx = 0; rowIdx < (int)rows.size(); rowIdx += stepY) {
        const auto& row = rows[rowIdx];
        for (int colIdx = 0; colIdx < (int)row.size(); colIdx += stepX) {
            pts.push_back(row[colIdx]);
        }
    }
    //std::cout << "[Debug] Loaded sparse terrain points: " << pts.size() << std::endl;
    return pts;
}

void rescaleToBox(std::vector<Vec3>& pts, float boxHalfSize = 98.0f)
{
    if (pts.empty()) return;
    Vec3 minPt = pts[0], maxPt = pts[0];
    for (const auto& p : pts) {
        minPt.x = std::min(minPt.x, p.x);
        minPt.y = std::min(minPt.y, p.y);
        minPt.z = std::min(minPt.z, p.z);
        maxPt.x = std::max(maxPt.x, p.x);
        maxPt.y = std::max(maxPt.y, p.y);
        maxPt.z = std::max(maxPt.z, p.z);
    }
    Vec3 center = (minPt + maxPt) * 0.5f;
    Vec3 size = maxPt - minPt;
    float maxDim = std::max({ size.x, size.y, size.z });
    float scale = (maxDim > 1e-6f) ? (boxHalfSize / (maxDim * 0.5f)) : 1.0f;
    for (auto& p : pts) {
        p = (p - center) * scale;
    }
}

static void loadYellowLinesFromCSV(
    const std::string& filename,
    std::vector<std::pair<Vec3, Vec3>>& out)
{
    out.clear();
    std::ifstream in(filename);
    if (!in.is_open()) {
        //std::cerr << "Cannot open " << filename << "\n";
        return;
    }
    
    std::string line;
    std::vector<Vec3> currentRidge;
    std::vector<std::vector<Vec3>> ridges;
    
    while (std::getline(in, line)) {
        // Trim whitespace
        line.erase(0, line.find_first_not_of(" \t\r\n"));
        line.erase(line.find_last_not_of(" \t\r\n") + 1);
        
        if (line.empty()) {
            // Empty line marks the end of a ridge
            if (currentRidge.size() >= 2) {
                ridges.push_back(currentRidge);
                //std::cout << "[Debug] Completed ridge with " << currentRidge.size() << " points" << std::endl;
            }
            currentRidge.clear();
            continue;
        }
        
        // Parse coordinate line
        std::stringstream ss(line);
        float x, y, z;
        char comma;
        
        if (ss >> x >> comma >> y >> comma >> z) {
            currentRidge.emplace_back(x, y, z);
        } else {
            //std::cerr << "[Warning] Failed to parse line: " << line << std::endl;
        }
    }
    
    // Don't forget the last ridge
    if (currentRidge.size() >= 2) {
        ridges.push_back(currentRidge);
        //std::cout << "[Debug] Completed final ridge with " << currentRidge.size() << " points" << std::endl;
    }
    
    //std::cout << "[Debug] Loaded " << ridges.size() << " ridges from " << filename << std::endl;
    
    // Create line segments for each ridge separately
    for (size_t ridgeIdx = 0; ridgeIdx < ridges.size(); ++ridgeIdx) {
        const auto& ridge = ridges[ridgeIdx];
        //std::cout << "[Debug] Processing ridge " << ridgeIdx << " with " << ridge.size() << " points" << std::endl;
        
                 // Sequential connection with strict distance check
         for (size_t i = 0; i + 1 < ridge.size(); ++i) {
             auto a = ridge[i];
             auto b = ridge[i + 1];
             
             // Check distance before connecting
             float distance = std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
             if (distance > 20.0f) {  // Skip connections longer than 20 units
                 //std::cout << "[Debug] Skipping long connection in ridge " << ridgeIdx 
                         // << ": " << distance << " units between points " << i << " and " << (i+1) << std::endl;
                 continue;
             }
             
             // Apply transformations
             a.x *= 2.3f;  a.y *= 2.3f;
             b.x *= 2.3f;  b.y *= 2.3f;
             a.x -= 5.0f; a.y -= 5.0f;
             b.x -= 5.0f; b.y -= 5.0f;
             
             out.emplace_back(a, b);
         }
    }
    
    //std::cout << "[Debug] Created " << out.size() << " line segments" << std::endl;
}

// ===================== 类型声明区 =====================
struct Snapshot {
    std::vector<std::vector<Vec3>> curves;
    std::vector<std::pair<Vec3, Vec3>> rays;
    std::vector<GPoint> points;
    std::vector<GLine> lines;
};

// ===================== 类声明区 =====================
class SplineNet {
public:
    std::vector<std::pair<Vec3, Vec3>> rays;
    std::vector<Vec3> seed1;
    std::vector<std::vector<Vec3>> curves;
    std::vector<std::vector<Vec3>> contour0Lines;
    std::vector<std::vector<Vec3>> contour1Lines;
    std::vector<std::vector<Vec3>> contour7Lines;
    void build(int samples = 60);
    void drawRays(Renderer& renderer) const;
    void drawCurves(Renderer& renderer) const;
    void drawSeeds(Renderer& renderer) const;
    void computeContourLines(const Vec3& seed0, const std::vector<Vec3>& seed1, int gridRes, int nLevels, float spacing);
    void drawContourLines(Renderer& renderer) const;
    std::vector<Vec3> computeCircleIntersections() const;
private:
    static float lengthSquared(const Vec3& v);
    static Vec3 normalize(const Vec3& v);
    static float dot(const Vec3& a, const Vec3& b);
};

class PointCloudNet {
public:
    std::vector<Vec3> points;
    std::vector<Vec3> curvePoints;
    std::vector<std::pair<Vec3, Vec3>> segments;
    bool loadFromFile(const std::string& filename);
    void computeBoundingBoxAndRescale();
    void drawCurve(Renderer& renderer) const;
    void scaleFromTopLeftCorner(float scaleFactor = 0.869f);
};

struct MarkSet {
    std::vector<Vec3> merged;
    std::vector<Vec3> intPts;
    static bool segIntersect2D(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d, Vec3& out);
    void build(const SplineNet& net);
    void draw(Renderer& renderer) const;
};

class VectorField {
    float cell, minV, maxV;
    float scalar[gridN][gridN];
public:
    void compute(const std::vector<Vec3>& src, const std::vector<float>& w);
    void draw(Renderer& renderer) const;
};

class ParcelSketch : public ISketch {
    PointCloudNet pointCloud;
    std::vector<Vec3> terrainPoints;
    uint32_t currentSeed;
    std::string cmdLog;
    std::unordered_map<uint32_t, Snapshot> snapshots;
    std::vector<std::pair<Vec3, Vec3>> yellowLines;
    int stage = 0;
    bool showField = false;
    bool showEllipseMode = false;
    bool showTriangleOnly = false;
    SplineNet net;
    MarkSet marks;
    VectorField vField;
    ColorGrowth growth;
    EllipseExpansion ellipses;
    bool showContours = false;
    bool countRequested = false;
    size_t prevGrowthCount = 0;
public:
    static constexpr int SMOOTH_PASSES = 2;
    ParcelSketch();
    void setup(void) override;
    void update(float) override;
    void draw(Renderer& renderer, Camera& camera) override;
    bool onKeyPress(unsigned char k, int x, int y) override;
    std::string getName() const override;
    void rebuild();
    void next();
    void recordSnapshot();
    void loadSnapshot(uint32_t seed);
};

// ===================== 实现体区 =====================
// SplineNet 实现
void SplineNet::build(int samples) {
    rays.clear(); seed1.clear(); curves.clear();
    std::vector<Vec3> borderCenters = {
        Vec3(0, -100, cpt.z), Vec3(0, 100, cpt.z), Vec3(-100, 0, cpt.z), Vec3(100, 0, cpt.z)
    };
    seed1.emplace_back(65.0f, 8.0f, 0.0f); rays.emplace_back(cpt, seed1[0]);
    seed1.emplace_back(-55.0f, 45.0f, 0.0f); rays.emplace_back(cpt, seed1[1]);
    std::vector<Vec3> circle;
    for (int i = 0; i <= samples; ++i) {
        float t = float(i) / samples;
        float angle = t * 2.0f * M_PI;
        circle.emplace_back(cpt.x + 65.0f * std::cos(angle), cpt.y + 65.0f * std::sin(angle), cpt.z);
    }
    curves.push_back(std::move(circle));
}
void SplineNet::drawRays(Renderer& renderer) const {
    if (rays.size() >= 2) {
        renderer.drawLine(z2A(rays[0].first), z2A(rays[0].second), Vec3(0,0,0), 1.0f);
        renderer.drawLine(z2A(rays[1].first), z2A(rays[1].second), Vec3(0,0,0), 1.0f);
    }
}
void SplineNet::drawCurves(Renderer& renderer) const {
    if (!curves.empty()) {
        const auto& pts = curves[0];
        for (size_t i = 1; i < pts.size(); ++i) {
            renderer.drawLine(pts[i-1], pts[i], Vec3(0,0,0), 1.0f);
        }
    }
}
void SplineNet::drawSeeds(Renderer& renderer) const {
    for (const auto& pt : seed1) {
        renderer.drawPoint(pt, Vec3(1,0,0), 5.0f);
    }
}
void SplineNet::computeContourLines(const Vec3& seed0, const std::vector<Vec3>& seed1, int gridRes, int nLevels, float spacing) {
    contour0Lines.clear();
    contour1Lines.clear();
    struct Circle { Vec3 c; float r; };
    // 采样范围
    float minX = -100.f, maxX = 100.f;
    float minY = -100.f, maxY = 100.f;
    float dx = (maxX - minX) / float(gridRes - 1);
    float dy = (maxY - minY) / float(gridRes - 1);

    // 圆参数
   
    std::vector<Circle> circles;
    circles.push_back({ seed0, 10.f });
    for (const auto& s : seed1) circles.push_back({ s, 10.f });

    // 构建归一化距离场
    std::vector<std::vector<float>> D(gridRes, std::vector<float>(gridRes));
    for (int i = 0; i < gridRes; ++i) {
        float x = minX + i * dx;
        for (int j = 0; j < gridRes; ++j) {
            float y = minY + j * dy;
            float minNormR = 1e9f;
            for (const auto& cir : circles) {
                float dx0 = x - cir.c.x;
                float dy0 = y - cir.c.y;
                float r_norm = std::sqrt(dx0 * dx0 + dy0 * dy0) / cir.r;
                if (r_norm < minNormR) minNormR = r_norm;
            }
            D[i][j] = minNormR;
        }
    }

    // --- 辅助函数：判断点是否相等（容差） ---
    auto ptEqual = [](const Vec3& a, const Vec3& b) {
        return (a - b).length() < 1e-4f;
    };
    // --- marching squares ---
    for (int k = 1; k <= nLevels; ++k) {
        float iso = spacing * k;
        std::vector<std::pair<Vec3, Vec3>> segments;
        for (int i = 0; i < gridRes - 1; ++i) {
            float x0 = minX + i * dx;
            float x1 = x0 + dx;
            for (int j = 0; j < gridRes - 1; ++j) {
                float y0 = minY + j * dy;
                float y1 = y0 + dy;
                float v0 = D[i][j];
                float v1 = D[i + 1][j];
                float v2 = D[i + 1][j + 1];
                float v3 = D[i][j + 1];
                int m = (v0 > iso) | ((v1 > iso) << 1) | ((v2 > iso) << 2) | ((v3 > iso) << 3);
                if (m == 0 || m == 15) continue;
                auto interp = [](float va, float vb, float a, float b, float iso) {
                    return a + (iso - va) / (vb - va) * (b - a);
                };
                Vec3 p[4];
                bool has[4] = {false, false, false, false};
                if ((m & 1) != (m & 2)) { p[0] = Vec3(interp(v0, v1, x0, x1, iso), y0, 0); has[0]=true; }
                if ((m & 2) != (m & 4)) { p[1] = Vec3(x1, interp(v1, v2, y0, y1, iso), 0); has[1]=true; }
                if ((m & 8) != (m & 4)) { p[2] = Vec3(interp(v3, v2, x0, x1, iso), y1, 0); has[2]=true; }
                if ((m & 1) != (m & 8)) { p[3] = Vec3(x0, interp(v0, v3, y0, y1, iso), 0); has[3]=true; }
                auto addSeg = [&](int a, int b) {
                    if (has[a] && has[b]) segments.emplace_back(p[a], p[b]);
                };
                switch (m) {
                case 1: case 14: addSeg(0, 3); break;
                case 2: case 13: addSeg(0, 1); break;
                case 3: case 12: addSeg(1, 3); break;
                case 4: case 11: addSeg(1, 2); break;
                case 5: addSeg(0, 1); addSeg(3, 2); break;
                case 6: case 9: addSeg(0, 2); break;
                case 7: case 8: addSeg(2, 3); break;
                }
            }
        }
        // --- 拼接 segments 为 polyline ---
        std::vector<std::vector<Vec3>> polylines;
        std::vector<bool> used(segments.size(), false);
        for (size_t i = 0; i < segments.size(); ++i) {
            if (used[i]) continue;
            std::vector<Vec3> poly;
            poly.push_back(segments[i].first);
            poly.push_back(segments[i].second);
            used[i] = true;
            bool extended = true;
            while (extended) {
                extended = false;
                for (size_t j = 0; j < segments.size(); ++j) {
                    if (used[j]) continue;
                    if (ptEqual(poly.back(), segments[j].first)) {
                        poly.push_back(segments[j].second);
                        used[j] = true;
                        extended = true;
                        break;
                    } else if (ptEqual(poly.back(), segments[j].second)) {
                        poly.push_back(segments[j].first);
                        used[j] = true;
                        extended = true;
                        break;
                    } else if (ptEqual(poly.front(), segments[j].first)) {
                        poly.insert(poly.begin(), segments[j].second);
                        used[j] = true;
                        extended = true;
                        break;
                    } else if (ptEqual(poly.front(), segments[j].second)) {
                        poly.insert(poly.begin(), segments[j].first);
                        used[j] = true;
                        extended = true;
                        break;
                    }
                }
            }
            polylines.push_back(poly);
        }
        if (k % 2 == 1) contour0Lines.insert(contour0Lines.end(), polylines.begin(), polylines.end());
        else            contour1Lines.insert(contour1Lines.end(), polylines.begin(), polylines.end());
    }
}   

void SplineNet::drawContourLines(Renderer& renderer) const {
    static const Vec3  contourColor{1,1,1};
    static constexpr float contourWidth = 1.5f;
    auto drawGroup = [&](const std::vector<std::vector<Vec3>>& groups) {
        for (const auto& seg : groups) {
            if (seg.size() >= 2) {
                for (size_t i = 1; i < seg.size(); ++i) {
                    renderer.drawLine(seg[i-1], seg[i], contourColor, contourWidth);
                }
            }
        }
    };
    drawGroup(contour0Lines);
    drawGroup(contour1Lines);
}


std::vector<Vec3> SplineNet::computeCircleIntersections() const {
    return seed1;
}
float SplineNet::lengthSquared(const Vec3& v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
}
Vec3 SplineNet::normalize(const Vec3& v) {
    float len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (len < 1e-6f) return Vec3(0, 0, 0);
    return Vec3(v.x / len, v.y / len, v.z / len);
}
float SplineNet::dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
// PointCloudNet 实现
bool PointCloudNet::loadFromFile(const std::string& filename) {
    points.clear(); curvePoints.clear();
    std::ifstream inFile(filename);
    if (!inFile.is_open()) { std::cerr << "Failed to open file: " << filename << std::endl; return false; }
    std::string line;
    while (std::getline(inFile, line)) {
        std::istringstream iss(line);
        std::string xStr, yStr, zStr;
        if (std::getline(iss, xStr, ',') && std::getline(iss, yStr, ',') && std::getline(iss, zStr, ',')) {
            float x = std::stof(xStr); float y = std::stof(yStr); float z = std::stof(zStr);
            points.push_back(Vec3(x, y, z));
        }
    }
    computeBoundingBoxAndRescale();
    return true;
}
void PointCloudNet::computeBoundingBoxAndRescale() {
    if (points.empty()) return;
    Vec3 minPt = points[0], maxPt = points[0];
    for (const auto& pt : points) {
        minPt.x = std::min(minPt.x, pt.x); minPt.y = std::min(minPt.y, pt.y); minPt.z = std::min(minPt.z, pt.z);
        maxPt.x = std::max(maxPt.x, pt.x); maxPt.y = std::max(maxPt.y, pt.y); maxPt.z = std::max(maxPt.z, pt.z);
    }
    Vec3 center = (minPt + maxPt) * 0.5f; Vec3 size = maxPt - minPt;
    float maxDim = std::max({ size.x, size.y, size.z }); float scale = (maxDim > 1e-6f) ? (100.0f / (maxDim * 0.5f)) : 1.0f;
    curvePoints.clear();
    for (const auto& pt : points) {
        Vec3 shifted = pt - center; Vec3 scaled = shifted * scale; curvePoints.push_back(scaled);
    }
}
void PointCloudNet::drawCurve(Renderer& renderer) const {
    if (curvePoints.size() >= 2) {
        for (size_t i = 1; i < curvePoints.size(); ++i) {
            renderer.drawLine(curvePoints[i-1], curvePoints[i], Vec3(0.2f, 0.7f, 0.2f), 3.0f);
        }
    }
}
void PointCloudNet::scaleFromTopLeftCorner(float scaleFactor) {
    Vec3 corner(-100.0f, 100.0f, 0.0f);
    for (auto& pt : curvePoints) {
        Vec3 relative = pt - corner; relative *= scaleFactor; pt = corner + relative;
    }
    segments.clear();
    if (curvePoints.size() >= 2) {
        segments.reserve(curvePoints.size() - 1);
        for (size_t i = 1; i < curvePoints.size(); ++i) {
            segments.emplace_back(curvePoints[i - 1], curvePoints[i]);
        }
    }
}
// MarkSet 实现
bool MarkSet::segIntersect2D(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d, Vec3& out) {
    float x1 = a.x, y1 = a.y, x2 = b.x, y2 = b.y;
    float x3 = c.x, y3 = c.y, x4 = d.x, y4 = d.y;
    float den = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
    if (fabs(den) < 1e-6f) return false;
    float ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / den;
    float ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / den;
    if (ua < 0 || ua > 1 || ub < 0 || ub > 1) return false;
    out = Vec3(x1 + ua * (x2 - x1), y1 + ua * (y2 - y1), 0);
    return true;
}
void MarkSet::build(const SplineNet& net) {
    struct Raw { Vec3 p; };
    std::vector<Raw> raw;
    for (size_t i = 0; i < net.rays.size(); ++i) {
        for (size_t j = i + 1; j < net.rays.size(); ++j) {
            Vec3 P;
            if (segIntersect2D(net.rays[i].first, net.rays[i].second, net.rays[j].first, net.rays[j].second, P)) {
                raw.push_back({ P });
            }
        }
    }
    for (auto& ray : net.rays) {
        for (auto& C : net.curves) {
            for (size_t k = 0; k + 1 < C.size(); ++k) {
                Vec3 P;
                if (segIntersect2D(ray.first, ray.second, C[k], C[k + 1], P)) {
                    raw.push_back({ P });
                }
            }
        }
    }
    struct Cluster { Vec3 ip; };
    std::vector<Cluster> cls;
    for (auto& m : raw) {
        bool fit = false;
        for (auto& c : cls) {
            if ((m.p - c.ip).length() < mergeEPS) {
                c.ip = (c.ip + m.p) * 0.5f; fit = true; break;
            }
        }
        if (!fit) { cls.push_back({ m.p }); }
    }
    merged.clear(); intPts.clear();
    for (auto& c : cls) {
        if (c.ip.x == 50.0f || c.ip.y == 50.0f) continue;
        merged.push_back(c.ip); intPts.push_back(c.ip);
    }
}
void MarkSet::draw(Renderer& renderer) const {
    for (const auto& p : intPts) {
        renderer.drawPoint(p, Vec3(0,0,0), 1.4f);
    }
}

// VectorField 实现
void VectorField::compute(const std::vector<Vec3>& src, const std::vector<float>& w) {
    float half = 50.f; cell = (half * 2) / (gridN - 1);
    minV = 1e9f; maxV = -1e9f;
    for (int i = 0; i < gridN; ++i)for (int j = 0; j < gridN; ++j) {
        float x = -half + i * cell, y = -half + j * cell, v = 0;
        for (size_t k = 0; k < src.size(); ++k) {
            float d = sqrtf((x - src[k].x) * (x - src[k].x) + (y - src[k].y) * (y - src[k].y));
            v += w[k] * 0.3f * expf(-(d * d) / (sigma * sigma));
        }
        scalar[i][j] = v; minV = fminf(minV, v); maxV = fmaxf(maxV, v);
    }
}
void VectorField::draw(Renderer& renderer) const {
    float half = 50.f;
    for (int i = 0; i < gridN - 1; ++i) {
        for (int j = 0; j < gridN - 1; ++j) {
            renderer.drawLine(Vec3(-half + i * cell, -half + j * cell, 0), Vec3(-half + (i + 1) * cell, -half + j * cell, 0), Vec3(0,0,0), 1.0f);
            renderer.drawLine(Vec3(-half + (i + 1) * cell, -half + j * cell, 0), Vec3(-half + (i + 1) * cell, -half + (j + 1) * cell, 0), Vec3(0,0,0), 1.0f);
            renderer.drawLine(Vec3(-half + (i + 1) * cell, -half + (j + 1) * cell, 0), Vec3(-half + i * cell, -half + (j + 1) * cell, 0), Vec3(0,0,0), 1.0f);
            renderer.drawLine(Vec3(-half + i * cell, -half + (j + 1) * cell, 0), Vec3(-half + i * cell, -half + j * cell, 0), Vec3(0,0,0), 1.0f);
        }
    }
}

// ParcelSketch 实现
ParcelSketch::ParcelSketch() : currentSeed(static_cast<uint32_t>(std::time(nullptr))) {
    globalRng.seed(currentSeed);
    cmdLog.clear();
    rebuild();
}
void ParcelSketch::setup(void) { rebuild(); }
void ParcelSketch::update(float) { /* 可留空 */ }
void ParcelSketch::rebuild() {
    pointCloud.loadFromFile("Land Border.txt");
    pointCloud.scaleFromTopLeftCorner(0.896f);
    terrainPoints = loadTerrainPoints("02_Terrain Points.txt");
    rescaleToBox(terrainPoints, 100.0f);
    auto& segs = pointCloud.segments;
    if (!segs.empty()) {
        segs.emplace_back(segs.back().second, segs.front().first);
    }
    ellipses.setPointSegments(pointCloud.segments);
    growth.setBoundarySegments(pointCloud.segments);
    globalRng.seed(currentSeed);
    showEllipseMode = false;
    loadYellowLinesFromCSV("smoothed_main_ridges.csv", yellowLines);
    std::cerr << "[Debug] scene.yellowLines=" << yellowLines.size() << std::endl;
    net.build();
    marks.build(net);
    growth.clear();
    growth.setPlacementMode(ColorGrowth::PlacementMode::Contour);
    growth.addSeed(0, cpt);
    for (auto& z : net.seed1) {
        growth.addSeed(1, z);
    }
    // 只用 net.seed1 作为高斯包，调大 nLevels 和调小 spacing
    net.computeContourLines(cpt, net.seed1, 120, 30, 0.45f);
    growth.setContour0Lines(net.contour0Lines);
    ellipses.setContour1Lines(net.contour1Lines);
    stage = 0;
}
void ParcelSketch::next() {
    if (showEllipseMode) return;
    stage = (stage + 1) % 3;
    if (stage == 1) {
        for (auto& z : net.seed1) {
            growth.addSeed(1, z);
        }
    }
}
void ParcelSketch::draw(Renderer& renderer, Camera& camera) {
     // Draw yellowLines in yellow - NO BOUNDARY RESTRICTIONS
     glColor3f(0, 0, 0);
     glLineWidth(2.0f);
     
     // Disable any clipping or boundary restrictions for yellow lines
     glDisable(GL_CLIP_PLANE0);
     glDisable(GL_CLIP_PLANE1);
     glDisable(GL_CLIP_PLANE2);
     glDisable(GL_CLIP_PLANE3);
     glDisable(GL_CLIP_PLANE4);
     glDisable(GL_CLIP_PLANE5);
     
     // Simple direct drawing of yellow lines - no complex ridge reconstruction
     // This should avoid any connection issues
     for (const auto& seg : yellowLines) {
         glBegin(GL_LINES);
         glVertex3f(seg.first.x, seg.first.y, seg.first.z);
         glVertex3f(seg.second.x, seg.second.y, seg.second.z);
         glEnd();
     }
     
     // Debug: Print some yellow line coordinates to check for issues
     if (!yellowLines.empty()) {
         //std::cout << "[Debug] Yellow lines count: " << yellowLines.size() << std::endl;
         //std::cout << "[Debug] First few segments:" << std::endl;
         for (size_t i = 0; i < std::min(size_t(5), yellowLines.size()); ++i) {
             const auto& seg = yellowLines[i];
         }
     }
    if (showTriangleOnly) {
        ellipses.drawVectorTriangles();
        return;
    }
    if (showEllipseMode) {
        net.drawContourLines(renderer);
       
        ellipses.update();
        ellipses.draw();
        return;
    }
    if (showField) {
        std::vector<Vec3> src; std::vector<float> w;
        src.push_back(cpt); w.push_back(0.3f);
        if (stage >= 1) for (auto& p : marks.merged) src.push_back(p), w.push_back(0.18f);
        vField.compute(src, w);
        vField.draw(renderer);
        return;
    }
    net.drawContourLines(renderer);
    if (stage == 1) {
        net.drawSeeds(renderer);
    }
    size_t before = prevGrowthCount;
    growth.update();
    size_t after = growth.getPoints().size();
    if (countRequested && before == after) {
        //std::cout << "Total points after full-expand: " << after << std::endl;
        countRequested = false;
    }
    prevGrowthCount = after;
    growth.draw();
    if (stage == 2) {
        std::vector<GPoint> ctr;
        growth.recolorNear(marks.intPts, 1.0f);
    }
}
bool ParcelSketch::onKeyPress(unsigned char k, int x, int y) {
    switch (k) {
    case 'v': case 'V': showField = !showField; break;
    case 'r': case 'R': currentSeed = static_cast<uint32_t>(std::time(nullptr)); rebuild(); cmdLog += 'r'; break;
    case '1': growth.expandTo50(); cmdLog += '1'; break;
    case '2': growth.expandFull(); countRequested = true; prevGrowthCount = growth.getPoints().size(); cmdLog += '2'; break;
    case 'a': case 'A': if (showEllipseMode) { ellipses.cornerSmooth(SMOOTH_PASSES); cmdLog += 'a'; } break;
    case 'q': case 'Q': if (showEllipseMode) { ellipses.initCircleSeeds(); cmdLog += 'q'; } break;
    case 'w': case 'W': if (showEllipseMode) { ellipses.circleCornerSmooth(SMOOTH_PASSES); cmdLog += 'w'; } break;
    case 't': case 'T': showTriangleOnly = !showTriangleOnly; ellipses.toggleVectorTriangle(); cmdLog += 't'; break;
    case 'z': { showEllipseMode = true; cmdLog += 'z'; std::vector<std::pair<Vec3, Vec3>> sl; for (auto& C : net.curves) { for (size_t i = 0; i + 1 < C.size(); ++i) { sl.emplace_back(C[i], C[i + 1]); } } const auto gpts = growth.getPoints(); std::vector<std::pair<Vec3, Vec3>> el; ellipses.init(cpt, marks.intPts, gpts, sl, net.rays, el, yellowLines); ellipses.setContour1Lines(net.contour1Lines); break; }
    case 's': case 'S': if (showEllipseMode) { ellipses.startDeform(); cmdLog += 's'; } break;
    case 'c': case 'C': { cmdLog += 'c'; growth.removeNearNetwork(yellowLines, 2.6f); growth.removeNearNetwork(pointCloud.segments, 2.6f); recordSnapshot(); break; }
    case 'm': case 'M': growth.removePointsNearPurple(6.0f); cmdLog += 'm'; break;
    case 'o': case 'O': if (showEllipseMode) { ellipses.toggleOffset(); cmdLog += 'o'; } break;
    //case '0': std::cout << "ReplaySeed=" << currentSeed << " cmds=" << cmdLog << std::endl; break;
    //case 'l': case 'L': { uint32_t seed; std::cout << "请输入 Seed 并回车："; if (std::cin >> seed) { loadSnapshot(seed); } else { std::cin.clear(); std::string dummy; std::getline(std::cin, dummy); std::cerr << "Seed 输入失败，取消恢复\n"; } break; }
    default: break;
    }
    return true;
}
std::string ParcelSketch::getName() const { return "ParcelSketch"; }
void ParcelSketch::recordSnapshot() {
    Snapshot s;
    s.curves = net.curves;
    s.rays = net.rays;
    s.points = growth.getPoints();
    s.lines = growth.getLines();
    snapshots[currentSeed] = std::move(s);
}
void ParcelSketch::loadSnapshot(uint32_t seed) {
    auto it = snapshots.find(seed);
    if (it == snapshots.end()) {
        std::cerr << "No snapshot for seed " << seed << std::endl;
        return;
    }
    const Snapshot& s = it->second;
    net.curves = s.curves;
    net.rays = s.rays;
    growth.loadState(s.points, s.lines);
    showEllipseMode = false;
    stage = 2;
}

ALICE2_REGISTER_SKETCH_AUTO(ParcelSketch)
