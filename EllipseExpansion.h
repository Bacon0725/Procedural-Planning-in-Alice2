#pragma once

#define _USE_MATH_DEFINES
#include <vector>
#include <array>
#include <utility>
#include <cmath>
#include <ctime>
#include <algorithm>
#include "../include/alice2.h"
#include "ColorGrowth.h"  // 包含 ColorType和GPoint

// 确保M_PI常量可用
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace alice2 {

    // 计算点到线段的距离
    inline float distPointSegment(
        const Vec3& P,
        const Vec3& A,
        const Vec3& B)
    {
        Vec3 AB = B - A;
        float len2 = AB.x * AB.x + AB.y * AB.y;
        if (len2 < 1e-6f) return (P - A).length();
        float t = ((P.x - A.x) * AB.x + (P.y - A.y) * AB.y) / len2;
        t = std::fmax(0.0f, std::fmin(1.0f, t));
        Vec3 H = A + AB * t;
        return (P - H).length();
    }

    // 椭圆边界结构
    struct EPoint {
        Vec3                  pos;        // 圆心
        Vec3                  rep;        // 表示椭圆短轴方向
        ColorType                col;        // 颜色
        std::vector<Vec3>     boundary;   // 边界点
        std::vector<bool>        frozen;     // 冻结标志
        std::vector<Vec3> regionCenters;
        float                    maxRadius;  // 最大半径
    };

    // 椭圆扩散算法
    class EllipseExpansion {
    public:
        // 公开成员变量
        std::vector<EPoint>                           ellipses;
        std::vector<std::pair<Vec3, Vec3>>       splineLines, rayLines, edgeLines;
        std::vector<std::pair<Vec3, Vec3>>       yellowLines;                           // 黄色线条存储
        std::vector<std::vector<Vec3>>             contour0Lines;
        std::vector<std::vector<Vec3>>             contour1Lines;
        std::vector<std::pair<Vec3, Vec3>> pointSegments;

        // 定义椭圆的一个分割线
        std::vector<std::pair<Vec3, Vec3>> abLines;

        const int                                     STEPS = 35;
        const float                                   SAME_DIST = 1.5;
        const float                                   DIFF_DIST = 1.5f;
        const std::array<float, 6>                    growRates = { 0.1f,0.1f,0.1f,0.1f,0.1f,0.1f };
        bool                                          showOffset = false;
        bool                                          deforming = false;
        bool                                          fillMode = false;
        double                                        lastTime = 0.0;
        double lastTimeCircles = 0.0;   // 小圆扩散
        std::vector<std::vector<Vec3>>             evenContourLines;
        bool    showVectorTriangle = false;      // 按 t 键显示 vector+三角形
        float   vectorTriHeight = 2.f;       // 三角形的"高"
        float   vectorBaseWidth = 0.8f;       // 三角形的"底边宽度"
        static inline Vec3 normalize(const Vec3& v) {
            float len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
            if (len > 1e-6f) {
                return Vec3{ v.x / len, v.y / len, v.z / len };
            }
            return v;  // 返回原向量
        }

        Vec3                      seed0_;        // 种子点
        std::vector<Vec3>         seeds_;        // 从net.build() 获取的种子点 seed1
        static constexpr int CIRCLE_STEPS = 25;    // 小圆 25 步

        struct EpDrawData {
            std::vector<std::pair<Vec3, Vec3>> lines;
            std::vector<Vec3> midpoints;
        };
        std::vector<EpDrawData> drawDataPerEllipse;

        // 初始化设置
        void init(
            const Vec3& seed0,
            const std::vector<Vec3>& seed1,
            const std::vector<GPoint>& growthPts,
            const std::vector<std::pair<Vec3, Vec3>>& _splineLines,
            const std::vector<std::pair<Vec3, Vec3>>& _rayLines,
            const std::vector<std::pair<Vec3, Vec3>>& _edgeLines,
            const std::vector<std::pair<Vec3, Vec3>>& yellowLinesParam  // 黄色线条
        )
        {
            ellipses.clear();
            splineLines = _splineLines;
            rayLines = _rayLines;
            edgeLines = _edgeLines;
            yellowLines = yellowLinesParam;  // 存储黄色线条
            deforming = false;
            fillMode = false;

            // 需要小圆扩散的椭圆点  
            static const std::array<Vec3, 3> specialPts = {
                Vec3{-48.f, -18.f, 0.f},
                Vec3{ 65.f,   8.f, 0.f},
                Vec3{-55.f,  45.f, 0.f}
            };
            constexpr float eps = 1e-3f;

            for (auto& gp : growthPts) {
                Vec3 p( gp.pos.x, gp.pos.y, gp.pos.z );

                // 判断是否为特殊点
                bool isSpecial = false;
                for (auto& sp : specialPts) {
                    if (fabs(p.x - sp.x) < eps && fabs(p.y - sp.y) < eps) {
                        isSpecial = true;
                        break;
                    }
                }

                EPoint ep;
                ep.pos = p;
                ep.col = gp.col;
                ep.frozen.assign(STEPS, false);
                ep.boundary.resize(STEPS);

                if (isSpecial) {
                    // 特殊点，半径 0.4 的圆  
                    ep.maxRadius = 0.8f;
                    for (int k = 0; k < STEPS; ++k) {
                        float theta = 2 * M_PI * k / STEPS;
                        ep.boundary[k] = p + Vec3{
                            0.8f * std::cos(theta),
                            0.8f * std::sin(theta),
                            0.0f
                        };
                    }
                }
                else {
                    // 普通点，椭圆初始化  
                    // 1) 找到 nearest contour0Lines 线段方向
                    Vec3 tangent{ 1,0,0 };
                    float bestD = 1e6f;
                    for (auto& contour : contour0Lines) {
                        for (size_t i = 0; i + 1 < contour.size(); i += 2) {
                            float d = distPointSegment(p, contour[i], contour[i + 1]);
                            if (d < bestD) {
                                bestD = d;
                                tangent = contour[i + 1] - contour[i];
                            }
                        }
                    }
                    float tl = std::hypot(tangent.x, tangent.y);
                    if (tl > 1e-6f) tangent.x /= tl, tangent.y /= tl;

                    // 2) 计算椭圆轴
                    Vec3 major = tangent * 0.8f;
                    Vec3 minor = Vec3(-tangent.y, tangent.x, 0) * 1.6f;

                    ep.rep = minor;
                    ep.maxRadius = 15.0f;

                    // 3) 计算椭圆边界
                    for (int k = 0; k < STEPS; ++k) {
                        float theta = 2 * M_PI * k / STEPS;
                        ep.boundary[k] = p
                            + major * std::cos(theta)
                            + minor * std::sin(theta);
                    }
                }

                ellipses.push_back(ep);
            }
        }

        void toggleVectorTriangle() {
            showVectorTriangle = !showVectorTriangle;
        }

        // 绘制每个 ellipse的短轴方向，绘制+三角形
        void drawVectorTriangles() {
            if (!showVectorTriangle) return;

            // 排斥点，直接定义三个固定点
            std::vector<Vec3> repulsionPoints;
            repulsionPoints.emplace_back(-45.f, 15.f, 0.f);
            repulsionPoints.emplace_back(19.2046f, 4.86243f, 0.f);
            repulsionPoints.emplace_back(-8.00135f, -38.4425f, 0.f);

            const float H = vectorTriHeight;
            const float W = vectorBaseWidth;

            for (auto& ep : ellipses) {
                const auto& C = ep.pos;

                // 1) 计算排斥力的反方向
                Vec3 rep{ 0,0,0 };
                for (auto& S : repulsionPoints) {
                    Vec3 d{ C.x - S.x, C.y - S.y, 0 };
                    float d2 = d.x * d.x + d.y * d.y;
                    if (d2 > 1e-6f) {
                        rep.x += d.x / d2;
                        rep.y += d.y / d2;
                    }
                }

                // 2) 归一化方向向量
                float len = std::sqrt(rep.x * rep.x + rep.y * rep.y);
                if (len < 1e-6f) continue;
                Vec3 dir{ rep.x / len, rep.y / len, 0.0f };

                // 3) 计算三角形的顶点和底边
                Vec3 apex{ C.x + dir.x * H, C.y + dir.y * H, C.z };
                Vec3 perp{ -dir.y, dir.x, 0 };
                Vec3 b1{ C.x + perp.x * (W * 0.5f), C.y + perp.y * (W * 0.5f), C.z };
                Vec3 b2{ C.x - perp.x * (W * 0.5f), C.y - perp.y * (W * 0.5f), C.z };

                // 4) 绘制向量线
                glColor3f(0, 0, 0);
                glLineWidth(2.0f);
                glBegin(GL_LINES);
                glVertex3f(C.x, C.y, C.z);
                glVertex3f(apex.x, apex.y, apex.z);
                glEnd();

                // 5) 绘制三角形
                glColor3f(0, 0, 0);
                glBegin(GL_TRIANGLES);
                glVertex3f(apex.x, apex.y, apex.z);
                glVertex3f(b1.x, b1.y, b1.z);
                glVertex3f(b2.x, b2.y, b2.z);
                glEnd();
            }
        }

        // 切换变形模式
        void startDeform()
        {
            deforming = true;
            lastTime = std::clock() / double(CLOCKS_PER_SEC);
        }

        // 角点平滑处理
        void cornerSmooth(int passes)
        {
            const int N = STEPS;
            for (int it = 0; it < passes; ++it) {
                for (auto& ep : ellipses) {
                    auto bak = ep.boundary;
                    for (int k = 0; k < N; ++k) {
                        int km = (k - 1 + N) % N, kp = (k + 1) % N;
                        ep.boundary[k] = bak[k] * 0.75f
                            + bak[km] * 0.125f
                            + bak[kp] * 0.125f;
                    }
                }
            }
            fillMode = true;
        }

        // 设置偶数等高线
        void setContour1Lines(const std::vector<std::vector<Vec3>>& evens)
        {
            contour1Lines = evens;
        }
        // 新增：设置 contour0Lines
        void setContour0Lines(const std::vector<std::vector<Vec3>>& odds)
        {
            contour0Lines = odds;
        }

        void setPointSegments(const std::vector<std::pair<Vec3, Vec3>>& segs) {
            pointSegments = segs;
        }

        bool segmentsIntersect(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d, Vec3& intersection) {
            // 2D平面直线段相交
            float s1_x = b.x - a.x;
            float s1_y = b.y - a.y;
            float s2_x = d.x - c.x;
            float s2_y = d.y - c.y;

            float denom = (-s2_x * s1_y + s1_x * s2_y);
            if (fabs(denom) < 1e-8f) return false; // 平行

            float s = (-s1_y * (a.x - c.x) + s1_x * (a.y - c.y)) / denom;
            float t = (s2_x * (a.y - c.y) - s2_y * (a.x - c.x)) / denom;
            if (s >= 0 && s <= 1 && t >= 0 && t <= 1) {
                intersection.x = a.x + (t * s1_x);
                intersection.y = a.y + (t * s1_y);
                intersection.z = a.z; // 保持z
                return true;
            }
            return false;
        }

        Vec3 polygonCentroid(const std::vector<Vec3>& poly) {
            double A = 0, Cx = 0, Cy = 0;
            int n = poly.size();
            for (int i = 0; i < n; ++i) {
                const auto& p0 = poly[i];
                const auto& p1 = poly[(i + 1) % n];
                double cross = p0.x * p1.y - p1.x * p0.y;
                A += cross;
                Cx += (p0.x + p1.x) * cross;
                Cy += (p0.y + p1.y) * cross;
            }
            A *= 0.5;
            if (std::abs(A) < 1e-7) return Vec3(0, 0, 0);
            Cx /= (6 * A);
            Cy /= (6 * A);
            return Vec3(Cx, Cy, 0);
        }

        // 分割椭圆，并计算区域中心
        void splitAllEllipsesWithContourAndVector() {
            abLines.clear();
            for (auto& ep : ellipses) {
                ep.regionCenters.clear();

                Vec3 center = ep.pos;
                Vec3 dir = normalize(ep.rep);           // 短轴方向
                Vec3 orth = Vec3(-dir.y, dir.x, 0.0f); // 正交方向

                // 1) 沿 orth方向从 center穿过 boundary 找到交点 A、B
                std::vector<Vec3> orthoInters;
                std::vector<int>     orthoIdxs;
                for (int i = 0; i < ep.boundary.size(); ++i) {
                    Vec3 a = ep.boundary[i];
                    Vec3 b = ep.boundary[(i + 1) % ep.boundary.size()];
                    Vec3 o0 = center - orth * 2000.f;
                    Vec3 o1 = center + orth * 2000.f;
                    Vec3 interP;
                    if (segmentsIntersect(a, b, o0, o1, interP)) {
                        orthoInters.push_back(interP);
                        orthoIdxs.push_back(i + 1);
                    }
                }
                auto get2closest = [&](std::vector<Vec3>& pts, std::vector<int>& idxs, const Vec3& ref) {
                    if (pts.size() > 2) {
                        std::vector<std::pair<float, int>> dist_idx;
                        for (int i = 0; i < pts.size(); ++i) {
                            float d2 = (pts[i].x - ref.x) * (pts[i].x - ref.x)
                                + (pts[i].y - ref.y) * (pts[i].y - ref.y);
                            dist_idx.emplace_back(d2, i);
                        }
                        std::sort(dist_idx.begin(), dist_idx.end());
                        pts = { pts[dist_idx[0].second], pts[dist_idx[1].second] };
                        idxs = { idxs[dist_idx[0].second], idxs[dist_idx[1].second] };
                    }
                    };
                get2closest(orthoInters, orthoIdxs, center);
                if (orthoInters.size() != 2) continue;

                // 2) 计算 A、B 的中点 center2
                Vec3 center2 = (orthoInters[0] + orthoInters[1]) * 0.5f;

                // 3) 找到距离 center2，方向为 dir，正交方向为 orth，穿过 boundary 的交点 C、D
                std::vector<Vec3> dir2Inters;
                std::vector<int>     dir2Idxs;
                for (int i = 0; i < ep.boundary.size(); ++i) {
                    Vec3 a = ep.boundary[i];
                    Vec3 b = ep.boundary[(i + 1) % ep.boundary.size()];
                    Vec3 d0 = center2 - dir * 2000.f;
                    Vec3 d1 = center2 + dir * 2000.f;
                    Vec3 interP;
                    if (segmentsIntersect(a, b, d0, d1, interP)) {
                        dir2Inters.push_back(interP);
                        dir2Idxs.push_back(i + 1);
                    }
                }
                get2closest(dir2Inters, dir2Idxs, center2);
                if (dir2Inters.size() != 2) continue;

                // 找到 orthoInters 和 get2closest(...) 之后，保证 orthoInters.size()==2
                abLines.emplace_back(orthoInters[0], orthoInters[1]);

                // 4) 将四个交点插入到 boundary，得到 newBoundary 和对应的索引 cutIdxs
                std::vector<std::pair<int, Vec3>> allInserts;
                for (int i = 0; i < 2; ++i) allInserts.emplace_back(orthoIdxs[i], orthoInters[i]);
                for (int i = 0; i < 2; ++i) allInserts.emplace_back(dir2Idxs[i], dir2Inters[i]);
                std::sort(allInserts.begin(), allInserts.end(),
                    [](auto& a, auto& b) { return a.first < b.first; });

                std::vector<Vec3> newBoundary = ep.boundary;
                std::vector<int>     cutIdxs;
                int insertCount = 0;
                for (auto& pr : allInserts) {
                    newBoundary.insert(newBoundary.begin() + pr.first + insertCount,
                        pr.second);
                    cutIdxs.push_back(pr.first + insertCount);
                    ++insertCount;
                }

                // 5) 用四个交点将 newBoundary 分成 4 组，每组先加 center2，再加边界顶点，计算质心
                int sz = newBoundary.size();
                std::sort(cutIdxs.begin(), cutIdxs.end());
                for (int s = 0; s < 4; ++s) {
                    int i0 = cutIdxs[s];
                    int i1 = cutIdxs[(s + 1) % 4];

                    // 构建区域多边形
                    std::vector<Vec3> region;
                    region.push_back(center2);  // 关键：将 center2 作为第一个顶点

                    // 从 boundary 上从 i0 (含) 到 i1 (含) 的顶点依次加入
                    int k = i0;
                    do {
                        region.push_back(newBoundary[k]);
                        k = (k + 1) % sz;
                    } while (k != (i1 + 1) % sz);

                    if (region.size() >= 3) {
                        // 1) 直接计算质心
                        Vec3 c = polygonCentroid(region);

                        // 2) 过滤：距离原椭圆边界 < 1.0f 的不要
                        bool tooClose = false;
                        for (int j = 0, n = (int)ep.boundary.size(); j < n; ++j) {
                            const Vec3& A = ep.boundary[j];
                            const Vec3& B = ep.boundary[(j + 1) % n];
                            if (distPointSegment(c, A, B) < 1.0f) {
                                tooClose = true;
                                break;
                            }
                        }

                        // 3) 只有距离 boundary 1 以上的才加入
                        if (!tooClose) {
                            ep.regionCenters.push_back(c);
                        }
                    }
                }
            }
        }

        struct CPoint {
            Vec3                  pos;       // 圆心
            std::vector<Vec3>     boundary;  // 圆上的顶点
            std::vector<bool>        frozen;    // 每个顶点是否被冻结
            float                    maxRadius; // 最大扩散半径，当前设置，可以继续扩展
        };
        std::vector<CPoint> circles;            // 存储所有小圆

        // 1) 从 regionCenters 创建小圆种子点，并初始化小圆时间
        inline void initCircleSeeds() {
            circles.clear();
            for (auto& ep : ellipses) {
                for (auto& c : ep.regionCenters) {
                    CPoint cp;
                    cp.pos = c;
                    cp.maxRadius = 5.0f;
                    cp.boundary.resize(CIRCLE_STEPS);
                    cp.frozen.assign(CIRCLE_STEPS, false);
                    // 初始化圆周上的 CIRCLE_STEPS 个点
                    for (int i = 0; i < CIRCLE_STEPS; ++i) {
                        float theta = 2 * M_PI * i / CIRCLE_STEPS;
                        cp.boundary[i] = {
                            c.x + std::cos(theta) * 0.8f,
                            c.y + std::sin(theta) * 0.8f,
                            c.z
                        };
                    }
                    circles.push_back(cp);
                }
            }
            deforming = true;
            lastTimeCircles = std::clock() / double(CLOCKS_PER_SEC);
        }

        inline void updateCircles() {
            if (!deforming) return;
            double now = std::clock() / double(CLOCKS_PER_SEC);
            float dt = float(now - lastTimeCircles);
            lastTimeCircles = now;

            for (auto& cp : circles) {
                for (int k = 0; k < STEPS; ++k) {
                    if (cp.frozen[k]) continue;
                    Vec3 P = cp.boundary[k];

                    // 限制：只对距离圆心 <= maxRadius (8) 的点进行扩散/膨胀
                    float r = (P - cp.pos).length();
                    if (r > cp.maxRadius) continue;

                    // 碰撞检测
                    bool freeze = false;

                    // 2.1 撞到椭圆边界
                    for (auto& ep : ellipses) {
                        for (int j = 0; j + 1 < (int)ep.boundary.size(); ++j) {
                            if (distPointSegment(P, ep.boundary[j], ep.boundary[j + 1]) < 0.5f) {
                                freeze = true; break;
                            }
                        }
                        if (freeze) break;
                    }
                    if (freeze) { cp.frozen[k] = true; continue; }

                    // 2.2 圆与圆碰撞
                    for (auto& other : circles) {
                        if (&other == &cp) continue;
                        for (auto& Q : other.boundary) {
                            if ((P - Q).length() < 0.8f) {
                                freeze = true; break;
                            }
                        }
                        if (freeze) break;
                    }
                    if (freeze) { cp.frozen[k] = true; continue; }

                    // 2.3 点线段检测
                    for (auto& seg : pointSegments) {
                        if (distPointSegment(P, seg.first, seg.second) < DIFF_DIST) {
                            freeze = true; break;
                        }
                    }
                    if (freeze) { cp.frozen[k] = true; continue; }

                    // 2.4 偶数等高线，contour0检测，阈值 0.8
                    for (auto& contour : contour1Lines) {
                        for (size_t j = 0; j + 1 < contour.size(); j += 2) {
                            if (distPointSegment(P, contour[j], contour[j + 1]) < 0.8f) {
                                freeze = true;
                                break;
                            }
                        }
                        if (freeze) break;
                    }
                    if (freeze) { cp.frozen[k] = true; continue; }

                    // 2.x 对 AB 分割线段检测，阈值 0.8f
                    for (auto& seg : abLines) {
                        if (distPointSegment(P, seg.first, seg.second) < 0.5f) {
                            freeze = true;
                            break;
                        }
                    }
                    if (freeze) { cp.frozen[k] = true; continue; }

                    // 扩散移动
                    Vec3 dir = P - cp.pos;
                    Vec3 cand = cp.pos + dir * (1.0f + growRates[int(ColorType::BLUE)] * dt);
                    // 越界处理
                    if (cand.x < -100 || cand.x > 100 || cand.y < -100 || cand.y > 100) {
                        cp.frozen[k] = true;
                    }
                    else {
                        cp.boundary[k] = cand;
                    }
                }
            }
        }

        inline void drawCircles() {
            // 圆边界
            glColor3f(1, 0, 1);
            for (auto& cp : circles) {
                glBegin(GL_LINE_LOOP);
                for (int i = 0; i < CIRCLE_STEPS; ++i) {
                    auto& p = cp.boundary[i];
                    glVertex3f(p.x, p.y, p.z);
                }
                glEnd();
            }
            // 顶点
            glColor3f(1, 1, 1);
            for (auto& cp : circles) {
                for (int i = 0; i < CIRCLE_STEPS; ++i) {
                    // 这里需要实现drawCircle函数或者用其他方式绘制点
                    glPointSize(3.0f);
                    glBegin(GL_POINTS);
                    glVertex3f(cp.boundary[i].x, cp.boundary[i].y, cp.boundary[i].z);
                    glEnd();
                }
            }
        }

        inline void circleCornerSmooth(int passes)
        {
            for (int it = 0; it < passes; ++it) {
                for (auto& cp : circles) {
                    auto bak = cp.boundary;           // 备份当前边界
                    int N = (int)bak.size();          // 动态取当前大小
                    for (int k = 0; k < N; ++k) {
                        int km = (k - 1 + N) % N;
                        int kp = (k + 1) % N;
                        cp.boundary[k] = bak[k] * 0.75f
                            + bak[km] * 0.125f
                            + bak[kp] * 0.125f;
                    }
                }
            }
        }

        // 每帧更新
        void update()
        {
            if (!deforming) return;
            double now = std::clock() / double(CLOCKS_PER_SEC);
            float dt = float(now - lastTime);
            lastTime = now;

            // 硬编码特殊椭圆中心点豁免检测
            static const Vec3 exemptPts[3] = {
                { -48.f, -18.f, 0.f },
                {  65.f,   8.f, 0.f },
                { -55.f,  45.f, 0.f }
            };
            const float eps = 1e-3f;

            for (auto& ep : ellipses) {
                // 1) 判断当前椭圆中心是否为特殊豁免点
                bool isExemptCenter = false;
                for (int i = 0; i < 3; ++i) {
                    if (fabs(ep.pos.x - exemptPts[i].x) < eps &&
                        fabs(ep.pos.y - exemptPts[i].y) < eps) {
                        isExemptCenter = true;
                        break;
                    }
                }

                // 2) 所有的"特殊椭圆中心点"逻辑：far away from [-100,-100]
                float dx0 = ep.pos.x + 100.0f, dy0 = ep.pos.y + 100.0f;
                bool skipDetect = (dx0 * dx0 + dy0 * dy0 <= 0.0f);

                for (int k = 0; k < STEPS; ++k) {
                    if (ep.frozen[k]) continue;
                    Vec3 oldP = ep.boundary[k];




                  // —— 3) yellowLines 冻结检测 —— 
                   bool freeze = false;
                   //// 只有在非免检区域且非免疫中心时才做 yellowLines 检测
                    if (!skipDetect && !isExemptCenter) {
                    for (auto& seg : yellowLines) {
                    if (distPointSegment(oldP, seg.first, seg.second) < 1.8f) {
                    freeze = true;
                    break;
                    }
                  }
                 }
                 if (freeze) {
                 ep.frozen[k] = true;
                 continue;
                 }



                 // 4) 偶数层等高线检测 
                //std::cout << "[Debug] contour1Lines.size() = " << contour1Lines.size() << std::endl;
                for (auto& contour : contour1Lines) {
                  for (size_t j = 0; j + 1 < contour.size(); j += 2) {
                  if (distPointSegment(oldP, contour[j], contour[j + 1]) <= 0.8f) {
                  freeze = true; break;
                    }
                  }
                  if (freeze) break;
                 }
                 if (freeze) { ep.frozen[k] = true; continue; }


                  //5) 点云线段检测
                  for (auto& seg : pointSegments) {
                    if (distPointSegment(oldP, seg.first, seg.second) < 1.0f) {
                   freeze = true;
                  break;}
                 }
                 if (freeze) {
                 ep.frozen[k] = true;
                 continue;
                 } 

                    // 6) 椭圆间碰撞检测
                    bool stop = false;
                    for (auto& o : ellipses) {
                        if (&o == &ep) continue;
                        float thresh = (o.col == ep.col ? SAME_DIST : DIFF_DIST);
                        float cd = (ep.pos - o.pos).length();
                        if (cd > ep.maxRadius + o.maxRadius + thresh) continue;
                        for (auto& r : o.boundary) {
                            if ((oldP - r).length() < thresh) {
                                stop = true;
                                break;
                            }
                        }
                        if (stop) break;
                    }
                    if (stop) {
                        ep.frozen[k] = true;
                        continue;
                    }

                    // 7) 扩散移动
                    Vec3 dir = oldP - ep.pos;
                    Vec3 cand = ep.pos + dir * (1.0f + growRates[int(ep.col)] * dt);
                    if (cand.x < -100 || cand.x > 100 || cand.y < -100 || cand.y > 100) {
                        ep.frozen[k] = true;
                    }
                    else {
                        ep.boundary[k] = cand;
                    }
                }
            }

            // 更新小圆
            updateCircles();
        }

        // 切换偏移显示
        void toggleOffset()
        {
            showOffset = !showOffset;
        }

        // 绘制所有椭圆
        void draw()
        {
            for (auto& ep : ellipses) {
                if (fillMode) {
                    // 颜色填充
                    float r, g, b;
                    switch (ep.col) {
                    case BLUE:   r = 1.f; g = 1.f; b = 1.f; break;
                    case RED:    r = 1.f; g = 1.f; b = 1.f; break;
                    case PURPLE: r = 1.f; g = 1.f; b = 1.f; break;
                    case GREEN:  r = 1.f; g = 1.f; b = 1.f; break;
                    case YELLOW: r = 1.f; g = 1.f; b = 1.f; break;
                    case ORANGE: r = 1.f; g = 1.f; b = 1.f; break;
                    //case BLUE:   r = 0.220f; g = 0.906f; b = 0.890f; break;
                    //case RED:    r = 0.557f; g = 0.000f; b = 0.000f; break;
                    //case PURPLE: r = 0.506f; g = 0.000f; b = 0.800f; break;
                    //case GREEN:  r = 0.500f; g = 1.000f; b = 0.500f; break;
                    //case YELLOW: r = 1.000f; g = 0.839f; b = 0.200f; break;
                    //case ORANGE: r = 0.980f; g = 0.325f; b = 0.024f; break;
                    }
                   glColor3f(r, g, b);
                    glBegin(GL_TRIANGLE_FAN);
                    glVertex3f(ep.pos.x, ep.pos.y, ep.pos.z);
                    for (auto& p : ep.boundary) glVertex3f(p.x, p.y,p.z);
                    glVertex3f(ep.boundary[0].x, ep.boundary[0].y, ep.boundary[0].z);
                    glEnd();

                    // 内偏移填充
                    if (showOffset) {
                        std::vector<Vec3> innerPts;
                        innerPts.reserve(ep.boundary.size());
                        for (auto& p : ep.boundary) {
                            Vec3 dir = p - ep.pos;
                            float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
                            if (len > 1e-6f) dir *= (1.0f / len);
                            innerPts.push_back(p - dir * 0.2f);
                        }
                        glColor3f(1, 1, 1);
                        glBegin(GL_TRIANGLE_FAN);
                        glVertex3f(ep.pos.x, ep.pos.y, ep.pos.z);
                        for (auto& ip : innerPts) glVertex3f(ip.x, ip.y, ip.z);
                        glVertex3f(innerPts[0].x, innerPts[0].y, innerPts[0].z);
                        glEnd();
                    }
                }

                // 绘制边界线条
                if (!showOffset) {
                    glColor3f(1, 1, 1);
                    glBegin(GL_LINE_LOOP);
                    for (auto& p : ep.boundary) glVertex3f(p.x, p.y, p.z);
                    glEnd();
                }
               
            }

            if (showVectorTriangle) {
                // 只绘制三角形，不绘制其他内容
                drawVectorTriangles();
                return;
            }

            drawCircles();
        }
    };

} // namespace alice2