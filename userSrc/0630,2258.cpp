// sketch_sdf_interactive_refactored.cpp - 重构后的交互式SDF示例
// 使用shape.h头文件，将形状和运动逻辑分离

#include "../include/alice2.h"
#include "scalarField.h"
#include "shape.h"
#include "tower logic.h"
#include "ExportContoursOBJ.h"
#include "../src/sketches/SketchRegistry.h"
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <unordered_map>
#include <tuple>
#include <iomanip>

using namespace alice2;

class InteractiveSDFSketch : public ISketch {
private:
    ScalarField2D m_field;
    
    // 形状管理器（处理所有形状和运动逻辑）
    std::unique_ptr<ShapeManager> m_shapeManager;
    
    // 显示控制
    bool m_show_contours;
    bool m_show_centers;
    
    // 塔楼模式
    int m_tower_mode;        // 0=普通模式, 1=Simple叠层, 2=Contour叠层
    std::unique_ptr<ScalarFieldLayers> m_tower_layers;

    bool m_paused = false;
    // 轨迹线相关
    bool m_showTrail = false;
    std::vector<Vec3> m_trailMain;
    std::vector<Vec3> m_trailSecondary;

public:
    InteractiveSDFSketch()
        : m_field(Vec3(-50, -50, 0), Vec3(50, 50, 0), 200, 200)
        , m_shapeManager(std::make_unique<ShapeManager>())
        , m_show_contours(true)
        , m_show_centers(true)
        , m_tower_mode(0)
    {
        // 初始化塔楼层
        m_tower_layers = std::make_unique<ScalarFieldLayers>(
        40, 5.0f, Vec3(-50, -50, 0), Vec3(50, 50, 0), 200, 200
        );
        // BBB.cpp风格轨迹参数
        auto& mainMotion = m_shapeManager->getMotion();
        mainMotion.orbit_speed = 1.2f;
        mainMotion.compSpeed = 1.0f;
        mainMotion.compMin = 0.0f;
        mainMotion.compMax = 1.5f;
        mainMotion.compAngle = 0.0f;
        mainMotion.compress = true;
        auto& secMotion = m_shapeManager->getSecondaryMotion();
        secMotion.orbit_speed = 1.1f;
        secMotion.compSpeed = 0.7f;
        secMotion.compMin = 0.0f;
        secMotion.compMax = 1.5f;
        secMotion.compAngle = 0.0f;
        secMotion.compress = true;
    }

    ~InteractiveSDFSketch() override = default;

    std::string getName() const override {
        return "Interactive SDF Sketch (Refactored)";
    }

    std::string getDescription() const override {
        return "Interactive signed distance fields with shapes, animations, boolean operations and tower layers (using shape.h)";
    }

    std::string getAuthor() const override {
        return "alice2";
    }


    void setup() override {
        scene().setBackgroundColor(Vec3(0.1f, 0.1f, 0.15f));
        scene().setShowGrid(false);
        scene().setShowAxes(true);
        scene().setAxesLength(10.0f);
        generateField();
    }

    void update(float deltaTime) override {
        float dt = m_paused ? 0.0f : deltaTime;
        // 强制主/次形状 compress 始终为 true，实现自动变轨
        m_shapeManager->getMotion().compress = true;
        m_shapeManager->getSecondaryMotion().compress = true;
        // 每帧累加 compAngle，轨道压缩因子随时间变化
        m_shapeManager->getMotion().compAngle += m_shapeManager->getMotion().compSpeed * dt;
        m_shapeManager->getSecondaryMotion().compAngle += m_shapeManager->getSecondaryMotion().compSpeed * dt;
        m_shapeManager->updateTime(dt);
        m_shapeManager->updateMotion(dt);
        // 每帧都刷新主平面
        m_shapeManager->generateField(m_field);
        // 轨迹记录
        if (m_showTrail) {
            if (m_shapeManager->getMotion().enable_orbit)
                m_trailMain.push_back(m_shapeManager->getMainShape().center);
            if (m_shapeManager->getSecondaryMotion().enable_orbit)
                m_trailSecondary.push_back(m_shapeManager->getSecondaryShape().center);
        }
        if (m_tower_mode > 0) {
            auto shapeGen1 = m_shapeManager->createMainShapeGeneratorCrossOrbit();
            auto shapeGen2 = m_shapeManager->createSecondaryShapeGeneratorCrossOrbit();
            auto shapeGen3 = MotionController::createAnimatedShapeGenerator(
                m_shapeManager->getThirdShape(), m_shapeManager->getThirdMotion());
            if (m_tower_mode == 1) {
                m_tower_layers->generateSimpleLayers(
                    m_shapeManager->getTime(),
                    shapeGen1,
                    shapeGen2,
                    shapeGen3,
                    static_cast<int>(m_shapeManager->getBooleanMode()),
                    0.15f
                );
            }
            else if (m_tower_mode == 2) {
                m_tower_layers->generateContourLayers(shapeGen1, shapeGen2, shapeGen3);
            }
        }
    }

    void draw(Renderer& renderer, Camera& camera) override {
        // 轨迹线
        if (m_showTrail) {
            if (m_trailMain.size() > 1) {
                renderer.setColor(Vec3(1.0f, 0.5f, 0.2f));
                for (size_t i = 1; i < m_trailMain.size(); ++i) {
                    renderer.drawLine(m_trailMain[i-1], m_trailMain[i], Vec3(1.0f, 0.5f, 0.2f), 2.0f);
                }
            }
            if (m_trailSecondary.size() > 1) {
                renderer.setColor(Vec3(0.2f, 0.8f, 1.0f));
                for (size_t i = 1; i < m_trailSecondary.size(); ++i) {
                    renderer.drawLine(m_trailSecondary[i-1], m_trailSecondary[i], Vec3(0.2f, 0.8f, 1.0f), 2.0f);
                }
            }
        }
        // 始终绘制主SDF场和中心点（不受7模式影响）
        m_field.draw_points(renderer, 2);
        if (m_show_contours) {
            renderer.setColor(Vec3(1.0f, 1.0f, 1.0f));
            m_field.drawIsocontours(renderer, 0.0f);
        }
        if (m_show_centers) {
            renderer.setColor(Vec3(1.0f, 0.0f, 0.0f));
            renderer.drawPoint(m_shapeManager->getMainShape().center, Vec3(1.0f, 0.0f, 0.0f), 5.0f);
            if (m_shapeManager->getBooleanMode() != BooleanMode::None) {
                renderer.setColor(Vec3(0.0f, 1.0f, 0.0f));
                renderer.drawPoint(m_shapeManager->getSecondaryShape().center, Vec3(0.0f, 1.0f, 0.0f), 5.0f);
            }
        }
        // 叠层显示（7/8模式）
        if (m_tower_mode > 0) {
            m_tower_layers->drawBase(renderer);
            m_tower_layers->drawLayers(renderer, m_tower_mode);
        }
        // UI信息
        renderer.setColor(Vec3(1.0f, 1.0f, 0.0f));
        renderer.drawString("Mode: " + getModeName(), 10, 30);
        renderer.drawString("Secondary Motion: " + m_shapeManager->getSecondaryMotionStatus(), 10, 150);
        renderer.drawString("Main Shape: " + m_shapeManager->getMainShapeName(), 10, 50);
        renderer.drawString("Secondary Shape: " + m_shapeManager->getSecondaryShapeName(), 10, 70);
        renderer.drawString("Boolean Mode: " + m_shapeManager->getBooleanModeName(), 10, 90);
        renderer.drawString("FPS: " + std::to_string(Application::getInstance()->getFPS()), 10, 110);
        auto& mainMotion = m_shapeManager->getMotion();
        auto& secMotion = m_shapeManager->getSecondaryMotion();
        renderer.drawString("Main Motion: " + m_shapeManager->getMotionStatus(), 10, 130);
        renderer.drawString("Secondary Motion: " + m_shapeManager->getSecondaryMotionStatus(), 10, 150);
        renderer.drawString("Shape: 1=Circle, 2=Square, 3=Hexagon", 10, 170);
        renderer.drawString("Main Motion: Q=Rotate CW, W=Rotate CCW, Z=Stop Rotate, E=Oscillate, R=Orbit", 10, 190);
        renderer.drawString("Secondary Motion: A=Rotate CW, S=Rotate CCW, X=Stop Rotate, D=Oscillate, F=Orbit", 10, 210);
        renderer.drawString("Boolean: B=Toggle, 4/5/6=Union/Subtract/Intersect", 10, 230);
        renderer.drawString("Display: C=Contours, D=Centers, F=Grid", 10, 250);
        if (m_tower_mode > 0) {
            renderer.drawString("Tower Mode: " + getTowerModeName(m_tower_mode), 10, 270);
            renderer.drawString("Press 0 to return to normal mode", 10, 290);
            renderer.drawString("Current shapes: " + m_shapeManager->getMainShapeName() + " + " + m_shapeManager->getSecondaryShapeName(), 10, 310);
        }
        renderer.drawString("Tower: 0=Normal, 7=Simple, 8=Contour", 10, 330);
        renderer.drawString(std::string("Paused: ") + (m_paused ? "ON" : "OFF") + " (L to toggle)", 10, 370);
        renderer.drawString("M: show/hide orbit trail", 10, 390);
    }

    void cleanup() override {}

    bool onKeyPress(unsigned char key, int x, int y) override {
        switch (key) {
            // 模式切换
            case '0':
                m_tower_mode = 0;
                return true;
            case 't':
            case 'T':
                m_tower_mode = 1;
                return true;
            case '*':
                m_tower_mode = 2;
                return true;
                
            // 形状切换（在普通模式和塔楼模式下都有效）
            case '1':
                m_shapeManager->setMainShapeType(ShapeType::Circle);
                generateField();
                return true;
            case '2':
                m_shapeManager->setMainShapeType(ShapeType::Square);
                generateField();
                return true;
            case '3':
                m_shapeManager->setMainShapeType(ShapeType::Hexagon);
                generateField();
                return true;
                
            // 次要形状切换
            case '4':
           
                m_shapeManager->setSecondaryShapeType(ShapeType::Circle);
                generateField();
                return true;
            case '5':
           
                m_shapeManager->setSecondaryShapeType(ShapeType::Square);
                generateField();
                return true;
            case '6':
           
                m_shapeManager->setSecondaryShapeType(ShapeType::Hexagon);
                generateField();
                return true;
                
            // 主形状运动控制
            case 'q': case 'Q': 
                m_shapeManager->getMotion().enable_rotate = true;
                m_shapeManager->getMotion().rotate_clockwise = true;  // 顺时针
                return true;
            case 'w': case 'W': 
                m_shapeManager->getMotion().enable_rotate = true;
                m_shapeManager->getMotion().rotate_clockwise = false; // 逆时针
                return true;
            case 'e': case 'E': m_shapeManager->getMotion().enable_oscillate = !m_shapeManager->getMotion().enable_oscillate; return true;
            case 'r': case 'R': 
                m_shapeManager->getMotion().enable_orbit = !m_shapeManager->getMotion().enable_orbit;
                m_shapeManager->setUseCrossOrbit(m_shapeManager->getMotion().enable_orbit || m_shapeManager->getSecondaryMotion().enable_orbit);
                return true;
           
            
            // 次要形状运动控制
            case 'a': case 'A': 
                m_shapeManager->getSecondaryMotion().enable_rotate = true;
                m_shapeManager->getSecondaryMotion().rotate_clockwise = true;  // 顺时针
                return true;
            case 's': case 'S': 
                m_shapeManager->getSecondaryMotion().enable_rotate = true;
                m_shapeManager->getSecondaryMotion().rotate_clockwise = false; // 逆时针
                return true;
            case 'd': case 'D': m_shapeManager->getSecondaryMotion().enable_oscillate = !m_shapeManager->getSecondaryMotion().enable_oscillate; return true;
            case 'f': case 'F': 
                m_shapeManager->getSecondaryMotion().enable_orbit = !m_shapeManager->getSecondaryMotion().enable_orbit;
                m_shapeManager->setUseCrossOrbit(m_shapeManager->getMotion().enable_orbit || m_shapeManager->getSecondaryMotion().enable_orbit);
                return true;
           

            // 布尔运算（仅在普通模式下有效）
            case 'b':
            case 'B':
                if (m_tower_mode == 0) {
                    cycleBooleanMode();
                    generateField();
                }
                return true;
            case '7':
                if (m_tower_mode == 0) {
                    m_shapeManager->setBooleanMode(BooleanMode::Union);
                    generateField();
                }
                return true;
            case '8':
                if (m_tower_mode == 0) {
                    m_shapeManager->setBooleanMode(BooleanMode::Subtract);
                    generateField();
                }
                return true;

            // 显示控制（仅在普通模式下有效）
            case 'c':
            case 'C':
                if (m_tower_mode == 0) {
                    m_show_contours = !m_show_contours;
                }
                return true;

                case 'o':
            case 'O':
                if (m_tower_mode > 0 && m_tower_layers) {
                    std::string filename = "tower_export.obj";
                    if (m_tower_mode == 1) {
                        // 直接导出当前已生成的simple layers
                        std::vector<std::vector<std::pair<Vec3, Vec3>>> allContours;
                        for (int i = 0; i < m_tower_layers->getTowerLevels().size(); ++i) {
                            ContourData cd = m_tower_layers->getSimpleLayer(i).get_contours(0.0f);
                            allContours.push_back(cd.line_segments);
                        }
                        if (exportContoursToOBJ(allContours, m_tower_layers->getTowerLevels(), filename)) {
                            std::cout << "Exported tower to " << filename << std::endl;
                        } else {
                            std::cout << "Failed to export tower OBJ." << std::endl;
                        }
                    } else if (m_tower_mode == 2) {
                        // 直接导出当前已生成的contour layers
                        if (exportContoursToOBJ(m_tower_layers->getTowerContours(), m_tower_layers->getTowerLevels(), filename)) {
                            std::cout << "Exported tower to " << filename << std::endl;
                        } else {
                            std::cout << "Failed to export tower OBJ." << std::endl;
                        }
                    }
                }
                return true;

            case 'z': case 'Z':
                m_paused = !m_paused;
                return true;

            case 'm': case 'M':
                m_showTrail = !m_showTrail;
                if (!m_showTrail) { m_trailMain.clear(); m_trailSecondary.clear(); }
                return true;

            // 图形三形状切换
            case 'y':
                m_shapeManager->setThirdShapeType(ShapeType::Circle);
                generateField();
                return true;
            case 'u':
                m_shapeManager->setThirdShapeType(ShapeType::Square);
                generateField();
                return true;
            case 'i':
                m_shapeManager->setThirdShapeType(ShapeType::Hexagon);
                generateField();
                return true;
            // 图形三运动控制
            case 'h':
                m_shapeManager->getThirdMotion().enable_rotate = true;
                m_shapeManager->getThirdMotion().rotate_clockwise = true;
                return true;
            case 'j':
                m_shapeManager->getThirdMotion().enable_rotate = true;
                m_shapeManager->getThirdMotion().rotate_clockwise = false;
                return true;
            case 'k':
                m_shapeManager->getThirdMotion().enable_oscillate = !m_shapeManager->getThirdMotion().enable_oscillate;
                return true;
            case 'l':
                m_shapeManager->getThirdMotion().enable_orbit = !m_shapeManager->getThirdMotion().enable_orbit;
                return true;

        }
        return false;
    }

private:
    void generateField() {
        // 始终刷新主平面
        m_shapeManager->generateField(m_field);
        if (m_tower_mode > 0) {
            // 叠层模式：生成塔楼层
            auto shapeGen1 = m_shapeManager->createMainShapeGeneratorCrossOrbit();
            auto shapeGen2 = m_shapeManager->createSecondaryShapeGeneratorCrossOrbit();
            auto shapeGen3 = MotionController::createAnimatedShapeGenerator(
                m_shapeManager->getThirdShape(), m_shapeManager->getThirdMotion());
            m_tower_layers->generateField(
                shapeGen1,
                shapeGen2,
                shapeGen3,
                static_cast<int>(m_shapeManager->getBooleanMode())
            );
        }
    }
    
    // 循环切换布尔模式
    void cycleBooleanMode() {
        auto& booleanMode = m_shapeManager->getBooleanMode();
        int currentMode = static_cast<int>(booleanMode);
        currentMode = (currentMode + 1) % 4;
        booleanMode = static_cast<BooleanMode>(currentMode);
    }
    
    std::string getModeName() {
        switch (m_tower_mode) {
            case 0: return "Normal";
            case 1: return "Tower Simple";
            case 2: return "Tower Contour";
            default: return "Unknown";
        }
    }
    
    std::string getTowerModeName(int mode) {
        switch (mode) {
            case 1: return "Simple Layers";
            case 2: return "Contour Layers";
            default: return "Unknown";
        }
    }
};

// 注册该 Sketch
ALICE2_REGISTER_SKETCH_AUTO(InteractiveSDFSketch) 