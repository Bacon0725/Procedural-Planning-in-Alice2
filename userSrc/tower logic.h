#include "../include/alice2.h"
#include "scalarField.h"
#include <vector>
#include <GL/gl.h>
#include <cmath>
#include <utility> // for std::pair in structured bindings
#include <functional> // for std::function

namespace alice2 {

// 封装三种模式下的叠层逻辑
class ScalarFieldLayers {
public:
    // 定义SDF形状生成函数的类型
    using ShapeGenerator = std::function<void(ScalarField2D&, float)>;
    
    inline ScalarFieldLayers(int numLevels, float levelHeight,
                             const Vec3& minCorner, const Vec3& maxCorner,
                             int resX, int resY)
      : m_field(minCorner, maxCorner, resX, resY)
      , m_numLevels(numLevels)
      , m_levelHeight(levelHeight)
    {
        m_simpleLayers .resize(m_numLevels, m_field);
        m_towerContours.resize(m_numLevels);
        m_towerLevels.reserve(m_numLevels);
        for(int i = 0; i < m_numLevels; ++i)
            m_towerLevels.push_back(i * m_levelHeight);
    }

    // 生成基础场（使用自定义形状生成器）
    inline void generateField(
        const ShapeGenerator& shapeGen1,
        const ShapeGenerator& shapeGen2,
        const ShapeGenerator& shapeGen3,
        int booleanMode = 0    // 默认平滑并集
    )
    {
        m_field.clear_field();
        ScalarField2D f1 = m_field;  shapeGen1(f1, 0.0f);
        ScalarField2D f2 = m_field;  shapeGen2(f2, 0.0f);
        ScalarField2D f3 = m_field;  shapeGen3(f3, 0.0f);
        m_field = f1;
        switch(booleanMode) {
            case 1: m_field.boolean_union(f2);     break;
            case 2: m_field.boolean_subtract(f2);  break;
            case 3: m_field.boolean_intersect(f2); break;
            default: m_field.boolean_smin(f2, 2.0f); break;
        }
        switch(booleanMode) {
            case 1: m_field.boolean_union(f3);     break;
            case 2: m_field.boolean_subtract(f3);  break;
            case 3: m_field.boolean_intersect(f3); break;
            default: m_field.boolean_smin(f3, 2.0f); break;
        }
    }

    // Simple 叠层：按 timeStep 时间间隔，基于自定义形状生成器，重建每一层
    // time：当前动画时刻  
    // shapeGen1, shapeGen2, shapeGen3：三个形状生成器函数
    // timeStep：每层与上一层的时间偏移
    inline void generateSimpleLayers(float time,
                                 const ShapeGenerator& shapeGen1,
                                 const ShapeGenerator& shapeGen2,
                                 const ShapeGenerator& shapeGen3,
                                 int booleanMode,
                                 float timeStep = 0.1f)
{
    auto [min_bb, max_bb] = m_field.get_bounds();
    auto [res_x, res_y] = m_field.get_resolution();

    for(int i = 0; i < m_numLevels; ++i) {
        float t = time - i * timeStep;
        ScalarField2D layer(min_bb, max_bb, res_x, res_y);
        shapeGen1(layer, t);

        ScalarField2D tmp = layer;
        shapeGen2(tmp, t);
        switch (booleanMode) {
            case 1: layer.boolean_union(tmp); break;
            case 2: layer.boolean_subtract(tmp); break;
            case 3: layer.boolean_intersect(tmp); break;
            default: layer.boolean_smin(tmp, 2.0f); break;
        }
        ScalarField2D tmp3 = layer;
        shapeGen3(tmp3, t);
        switch (booleanMode) {
            case 1: layer.boolean_union(tmp3); break;
            case 2: layer.boolean_subtract(tmp3); break;
            case 3: layer.boolean_intersect(tmp3); break;
            default: layer.boolean_smin(tmp3, 2.0f); break;
        }
        m_simpleLayers[i] = std::move(layer);
    }
}

    // Contour 叠层：线性插值后提取等高线线段
    inline void generateContourLayers(const ShapeGenerator& shapeGen1, const ShapeGenerator& shapeGen2, const ShapeGenerator& shapeGen3)
    {
        ScalarField2D f1 = m_field, f2 = m_field, f3 = m_field;
        f1.clear_field(); shapeGen1(f1, 0.0f);
        f2.clear_field(); shapeGen2(f2, 0.0f);
        f3.clear_field(); shapeGen3(f3, 0.0f);

        for(int i = 0; i < m_numLevels; ++i) {
            float wt = float(i) / float(m_numLevels - 1);
            ScalarField2D tmp = f1;  tmp.interpolate(f2, wt);
            tmp.interpolate(f3, wt * 0.5f);
            ContourData cd = tmp.get_contours(0.0f);
            m_towerContours[i] = cd.line_segments;
        }
    }

    // 平面基础绘制（点云 + 零等高线）
    inline void drawBase(Renderer& renderer) const
    {
        m_field.draw_points(renderer, 2);
        renderer.setColor(Vec3(1.0f,1.0f,1.0f));
        m_field.drawIsocontours(renderer, 0.0f);
    }

    // 叠层绘制：mode=1(Simple) 或 2(Contour)
    inline void drawLayers(Renderer& renderer, int mode) const
    {
        if(mode == 1) {
            for(int i = 0; i < m_numLevels; ++i) {
                float alpha = 1.0f - float(i)/m_numLevels;
                renderer.setColor(Vec3(alpha,alpha,alpha));
                glPushMatrix();
                glTranslatef(80.0f, 0.0f, i * m_levelHeight);
                m_simpleLayers[i].drawIsocontours(renderer, 0.0f);
                glPopMatrix();
            }
        }
        else if(mode == 2) {
            renderer.setColor(Vec3(0.5f,0.8f,1.0f));
            Vec3 offset(80,0,0);
            for(int i = 0; i < m_numLevels; ++i) {
                float z = m_towerLevels[i];
                for(auto &seg : m_towerContours[i]) {
                    Vec3 a(seg.first.x, seg.first.y, z);
                    Vec3 b(seg.second.x, seg.second.y, z);
                    renderer.drawLine(a + offset, b + offset);
                }
            }
        }
    }

    const std::vector<std::vector<std::pair<Vec3,Vec3>>>& getTowerContours() const { return m_towerContours; }
    const std::vector<float>& getTowerLevels() const { return m_towerLevels; }
    const ScalarField2D& getSimpleLayer(int i) const { return m_simpleLayers[i]; }

private:
    ScalarField2D                                 m_field;
    std::vector<ScalarField2D>                    m_simpleLayers;
    std::vector<std::vector<std::pair<Vec3,Vec3>>> m_towerContours;
    std::vector<float>                            m_towerLevels;
    int                                           m_numLevels;
    float                                         m_levelHeight;
};

} // namespace alice2 