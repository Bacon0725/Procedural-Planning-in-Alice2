#pragma once

#include "scalarField.h"
#include <cmath>
#include <vector>
#include <functional>

// 定义圆周率常量
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 形状类型枚举
enum class ShapeType {
    Circle = 0,
    Square = 1,
    Hexagon = 2
};

// 布尔运算模式枚举
enum class BooleanMode {
    None = 0,
    Union = 1,
    Subtract = 2,
    Intersect = 3
};

// 运动类型枚举
enum class MotionType {
    None = 0,
    Rotate = 1,         // 自转
    Oscillate = 2,      // 振荡（size/半径/边长）
    Orbit = 3           // 椭圆轨道公转
};

// 形状参数结构体
struct ShapeParams {
    ShapeType type = ShapeType::Circle;
    Vec3 center = Vec3(0, 0, 0);
    Vec3 origin_center = Vec3(0, 0, 0);
    float size = 15.0f;
    float base_size = 15.0f; // 新增，振荡基准
    float angle = 0.0f;
};

// 运动参数结构体
struct MotionParams {
    bool enable_rotate = false;
    bool enable_oscillate = false;
    bool enable_orbit = false;
    float rotate_speed = 1.0f;
    bool rotate_clockwise = true;  // true=顺时针, false=逆时针
    float osc_amplitude = 5.0f;
    float osc_frequency = 1.0f;
    float orbit_radius = 15.0f;
    float orbit_speed = 1.0f;
    float orbit_eccentricity = 1.0f;
    // BBB.cpp 特殊轨迹参数
    bool compress = false;
    float compAngle = 0.0f;
    float compSpeed = 1.0f;
    float compMin = 0.0f;
    float compMax = 1.5f;
};

// 形状生成器类
class ShapeGenerator {
public:
    // 应用形状到SDF场
    static void applyShape(ScalarField2D& field, const ShapeParams& params) {
        switch (params.type) {
            case ShapeType::Circle:
                field.apply_scalar_circle(params.center, params.size);
                break;
            case ShapeType::Square:
                field.apply_scalar_rect(params.center, Vec3(params.size, params.size, 0), params.angle);
                break;
            case ShapeType::Hexagon:
                applyHexagon(field, params.center, params.size, params.angle);
                break;
        }
    }
    
    // 应用六边形
    static void applyHexagon(ScalarField2D& field, const Vec3& center, float radius, float angle) {
        std::vector<Vec3> vertices;
        for (int i = 0; i < 6; ++i) {
            float a = angle + i * M_PI / 3.0f;
            float x = center.x + radius * std::cos(a);
            float y = center.y + radius * std::sin(a);
            vertices.emplace_back(x, y, 0.0f);
        }
        field.apply_scalar_polygon(vertices);
    }
    
    // 获取形状名称
    static std::string getShapeName(ShapeType type) {
        switch (type) {
            case ShapeType::Circle: return "Circle";
            case ShapeType::Square: return "Square";
            case ShapeType::Hexagon: return "Hexagon";
            default: return "Unknown";
        }
    }
};

// 运动控制器类
class MotionController {
public:
    // 更新形状参数（应用运动）
    static void updateShapeParams(ShapeParams& params, const MotionParams& motion, float time, float deltaTime, bool isSnapshot = false, bool isSecondary = false) {
        // 轨道运动
        if (motion.enable_orbit) {
            float angle = motion.orbit_speed * time;
            float comp = 1.0f;
            float compAngle = motion.compAngle;
            if (motion.compress) {
                compAngle += motion.compSpeed * deltaTime;
                float t = (std::sin(compAngle) + 1.0f) * 0.5f;
                comp = motion.compMin + t * (motion.compMax - motion.compMin);
            }
            params.center.x = params.origin_center.x + motion.orbit_radius * std::cos(angle);
            params.center.y = params.origin_center.y - std::sin(angle) * motion.orbit_radius * comp;
            params.center.z = 0.0f;
        } else {
            params.center = params.origin_center;
        }
        // 振荡
        if (motion.enable_oscillate) {
            float phase = isSecondary ? (time * motion.osc_frequency + M_PI) : (time * motion.osc_frequency);
            params.size = params.base_size + std::sin(phase) * motion.osc_amplitude;
        } else {
            params.size = params.base_size;
        }
        // 自转
        if (motion.enable_rotate) {
            float rotation_direction = motion.rotate_clockwise ? 1.0f : -1.0f;
            if (isSnapshot) {
                params.angle = params.angle + rotation_direction * motion.rotate_speed * time * 0.2f;
            } else {
                params.angle += rotation_direction * motion.rotate_speed * deltaTime;
            }
        }
    }
    
    // 更新形状参数（应用交叉orbit运动）
    static void updateShapeParamsWithCrossOrbit(ShapeParams& mainParams, ShapeParams& secondaryParams, 
                                               const MotionParams& mainMotion, const MotionParams& secondaryMotion, 
                                               float time, float deltaTime, bool isSnapshot = false) {
        // 主形状的orbit：围绕 (0,0,0) 旋转
        if (mainMotion.enable_orbit) {
            float angle = mainMotion.orbit_speed * time;
            float comp = 1.0f;
            float compAngle = mainMotion.compAngle;
            if (mainMotion.compress) {
                compAngle += mainMotion.compSpeed * deltaTime;
                float t = (std::sin(compAngle) + 1.0f) * 0.5f;
                comp = mainMotion.compMin + t * (mainMotion.compMax - mainMotion.compMin);
            }
            mainParams.center.x = 0.0f + mainMotion.orbit_radius * std::cos(angle);
            mainParams.center.y = 0.0f - std::sin(angle) * mainMotion.orbit_radius * comp;
            mainParams.center.z = 0.0f;
        } else {
            mainParams.center = mainParams.origin_center;
        }
        // 次形状的orbit：围绕 (0,0,0) 旋转
        if (secondaryMotion.enable_orbit) {
            float angle = secondaryMotion.orbit_speed * time;
            float comp = 1.0f;
            float compAngle = secondaryMotion.compAngle;
            if (secondaryMotion.compress) {
                compAngle += secondaryMotion.compSpeed * deltaTime;
                float t = (std::sin(compAngle) + 1.0f) * 0.5f;
                comp = secondaryMotion.compMin + t * (secondaryMotion.compMax - secondaryMotion.compMin);
            }
            secondaryParams.center.x = 0.0f + secondaryMotion.orbit_radius * std::cos(angle);
            secondaryParams.center.y = 0.0f - std::sin(angle) * secondaryMotion.orbit_radius * comp;
            secondaryParams.center.z = 0.0f;
        } else {
            secondaryParams.center = secondaryParams.origin_center;
        }
        // 振荡（使用各自的基准点）
        if (mainMotion.enable_oscillate) {
            mainParams.size = mainParams.base_size + std::sin(time * mainMotion.osc_frequency) * mainMotion.osc_amplitude;
        } else {
            mainParams.size = mainParams.base_size;
        }
        if (secondaryMotion.enable_oscillate) {
            secondaryParams.size = secondaryParams.base_size + std::sin(time * secondaryMotion.osc_frequency + M_PI) * secondaryMotion.osc_amplitude;
        } else {
            secondaryParams.size = secondaryParams.base_size;
        }
        // 自转（使用各自的基准点）
        if (mainMotion.enable_rotate) {
            float rotation_direction = mainMotion.rotate_clockwise ? 1.0f : -1.0f;
            if (isSnapshot) {
                mainParams.angle = mainParams.angle + rotation_direction * mainMotion.rotate_speed * time * 0.2f;
            } else {
                mainParams.angle += rotation_direction * mainMotion.rotate_speed * deltaTime;
            }
        }
        if (secondaryMotion.enable_rotate) {
            float rotation_direction = secondaryMotion.rotate_clockwise ? 1.0f : -1.0f;
            if (isSnapshot) {
                secondaryParams.angle = secondaryParams.angle + rotation_direction * secondaryMotion.rotate_speed * time * 0.2f;
            } else {
                secondaryParams.angle += rotation_direction * secondaryMotion.rotate_speed * deltaTime;
            }
        }
    }
    
    // 创建动画形状生成器（用于塔楼模式）
    static std::function<void(ScalarField2D&, float)> createAnimatedShapeGenerator(
        const ShapeParams& baseParams, 
        const MotionParams& motion) {
        return [baseParams, motion](ScalarField2D& field, float t) {
            ShapeParams animatedParams = baseParams;
            // 轨道运动
            if (motion.enable_orbit) {
                animatedParams.center.x = baseParams.origin_center.x + motion.orbit_radius * std::cos(motion.orbit_speed * t);
                animatedParams.center.y = baseParams.origin_center.y + motion.orbit_radius * std::sin(motion.orbit_speed * t) * motion.orbit_eccentricity;
            } else {
                animatedParams.center = baseParams.origin_center;
            }
            // 振荡
            if (motion.enable_oscillate) {
                animatedParams.size = baseParams.base_size + std::sin(t * motion.osc_frequency) * motion.osc_amplitude;
            } else {
                animatedParams.size = baseParams.base_size;
            }
            // 自转
            if (motion.enable_rotate) {
                float rotation_direction = motion.rotate_clockwise ? 1.0f : -1.0f;
                animatedParams.angle = baseParams.angle + rotation_direction * motion.rotate_speed * t * 0.2f;
            }
            ShapeGenerator::applyShape(field, animatedParams);
        };
    }
    
    // 获取运动状态字符串
    static std::string getMotionStatus(const MotionParams& motion) {
        std::string status;
        if (motion.enable_rotate) {
            status += "Rotate(" + std::string(motion.rotate_clockwise ? "CW" : "CCW") + ") ";
        }
        if (motion.enable_oscillate) status += "Oscillate ";
        if (motion.enable_orbit) status += "Orbit ";
        if (status.empty()) return "OFF";
        return status;
    }
};

// 布尔运算处理器类
class BooleanProcessor {
public:
    // 执行布尔运算
    static void processBoolean(ScalarField2D& result, 
                              const ScalarField2D& field1, 
                              const ScalarField2D& field2, 
                              BooleanMode mode) {
        result = field1;
        
        switch (mode) {
            case BooleanMode::Union:
                result.boolean_union(field2);
                break;
            case BooleanMode::Subtract:
                result.boolean_subtract(field2);
                break;
            case BooleanMode::Intersect:
                result.boolean_intersect(field2);
                break;
            case BooleanMode::None:
            default:
                // 不做任何操作，保持field1
                break;
        }
    }
    
    // 生成带布尔运算的SDF场
    static void generateFieldWithBoolean(ScalarField2D& result,
                                        const ShapeParams& shape1,
                                        const ShapeParams& shape2,
                                        const ShapeParams& shape3, // 新增
                                        BooleanMode booleanMode,
                                        const Vec3& bounds_min,
                                        const Vec3& bounds_max,
                                        int res_x, int res_y) {
        result.clear_field();
        if (booleanMode == BooleanMode::None) {
            // 只显示主形状
            ShapeGenerator::applyShape(result, shape1);
        } else {
            // 先主次布尔，再与三布尔
            ScalarField2D main_field(bounds_min, bounds_max, res_x, res_y);
            ScalarField2D secondary_field(bounds_min, bounds_max, res_x, res_y);
            ScalarField2D third_field(bounds_min, bounds_max, res_x, res_y);
            ShapeGenerator::applyShape(main_field, shape1);
            ShapeGenerator::applyShape(secondary_field, shape2);
            ShapeGenerator::applyShape(third_field, shape3);
            ScalarField2D temp_field = main_field;
            processBoolean(temp_field, main_field, secondary_field, booleanMode);
            processBoolean(result, temp_field, third_field, booleanMode);
        }
    }
    
    // 获取布尔模式名称
    static std::string getBooleanModeName(BooleanMode mode) {
        switch (mode) {
            case BooleanMode::None: return "None";
            case BooleanMode::Union: return "Union";
            case BooleanMode::Subtract: return "Subtract";
            case BooleanMode::Intersect: return "Intersect";
            default: return "Unknown";
        }
    }
};

// 形状管理器类（整合所有功能）
class ShapeManager {
private:
    ShapeParams m_mainShape;
    ShapeParams m_secondaryShape;
    ShapeParams m_thirdShape; // 新增：第三图形
    MotionParams m_motion;
    MotionParams m_secondaryMotion;
    MotionParams m_thirdMotion; // 新增：第三图形运动
    BooleanMode m_booleanMode;
    float m_time;
    bool m_use_cross_orbit;  // 是否使用交叉orbit模式
    
public:
    ShapeManager() 
        : m_time(0.0f)
        , m_booleanMode(BooleanMode::None)
        , m_use_cross_orbit(false) {
        // 初始化主形状
        m_mainShape.type = ShapeType::Circle;
        m_mainShape.center = Vec3(0, 6, 0);
        m_mainShape.origin_center = m_mainShape.center;
        m_mainShape.size = 15.0f;
        m_mainShape.base_size = 15.0f;
        m_mainShape.angle = 0.0f;
        // 初始化次要形状
        m_secondaryShape.type = ShapeType::Square;
        m_secondaryShape.center = Vec3(-8, -8, 0);
        m_secondaryShape.origin_center = m_secondaryShape.center;
        m_secondaryShape.size = 15.0f;
        m_secondaryShape.base_size = 15.0f;
        m_secondaryShape.angle = 0.0f;
        // 初始化第三图形
        m_thirdShape.type = ShapeType::Circle;
        m_thirdShape.center = Vec3(8, -8, 0);
        m_thirdShape.origin_center = m_thirdShape.center;
        m_thirdShape.size = 15.0f;
        m_thirdShape.base_size = 15.0f;
        m_thirdShape.angle = 0.0f;
        // 初始化运动参数
        m_motion.rotate_speed = 1.0f;
        m_motion.osc_amplitude = 5.0f;
        m_motion.osc_frequency = 1.0f;
        m_motion.orbit_radius = 10.0f;
        m_motion.orbit_speed = 1.0f;
        m_motion.orbit_eccentricity = 1.0f;
        m_secondaryMotion.rotate_speed = 1.0f;
        m_secondaryMotion.osc_amplitude = 5.0f;
        m_secondaryMotion.osc_frequency = 1.0f;
        m_secondaryMotion.orbit_radius = 18.0f;
        m_secondaryMotion.orbit_speed = 1.0f;
        m_secondaryMotion.orbit_eccentricity = 1.0f;
        // 第三图形运动参数，放大缩小幅度介于主/次之间
        m_thirdMotion.rotate_speed = 1.0f;
        m_thirdMotion.osc_amplitude = 5.0f; // 可根据需要调整为主/次之间
        m_thirdMotion.osc_frequency = 1.0f;
        m_thirdMotion.orbit_radius = 14.0f;
        m_thirdMotion.orbit_speed = 1.0f;
        m_thirdMotion.orbit_eccentricity = 1.0f;
    }
    
    // 更新时间
    void updateTime(float deltaTime) {
        m_time += deltaTime;
    }
    
    // 更新运动
    void updateMotion(float deltaTime) {
        if (m_use_cross_orbit) {
            // 使用交叉orbit模式
            MotionController::updateShapeParamsWithCrossOrbit(m_mainShape, m_secondaryShape, m_motion, m_secondaryMotion, m_time, deltaTime);
        } else {
            // 使用普通模式
            MotionController::updateShapeParams(m_mainShape, m_motion, m_time, deltaTime, false, false);
            MotionController::updateShapeParams(m_secondaryShape, m_secondaryMotion, m_time, deltaTime, false, true);
        }
        // 无论什么模式，第三图形都要单独更新
        MotionController::updateShapeParams(m_thirdShape, m_thirdMotion, m_time, deltaTime, false, false);
    }
    
    // 生成SDF场
    void generateField(ScalarField2D& field) {
        BooleanProcessor::generateFieldWithBoolean(
            field, m_mainShape, m_secondaryShape, m_thirdShape, m_booleanMode,
            field.get_bounds().first, field.get_bounds().second,
            field.get_resolution().first, field.get_resolution().second
        );
    }
    
    // 创建形状生成器（用于塔楼模式）
    std::function<void(ScalarField2D&, float)> createMainShapeGenerator() {
        return MotionController::createAnimatedShapeGenerator(m_mainShape, m_motion);
    }
    
    std::function<void(ScalarField2D&, float)> createSecondaryShapeGenerator() {
        return MotionController::createAnimatedShapeGenerator(m_secondaryShape, m_secondaryMotion);
    }
    
    // 创建交叉orbit动画形状生成器（用于塔楼模式）
    std::function<void(ScalarField2D&, float)> createMainShapeGeneratorCrossOrbit() {
        return [this](ScalarField2D& field, float t) {
            ShapeParams mainParams = m_mainShape;
            ShapeParams secondaryParams = m_secondaryShape;
            MotionController::updateShapeParamsWithCrossOrbit(
                mainParams, secondaryParams, m_motion, m_secondaryMotion, t, 0.016f, true
            );
            ShapeGenerator::applyShape(field, mainParams);
        };
    }
    std::function<void(ScalarField2D&, float)> createSecondaryShapeGeneratorCrossOrbit() {
        return [this](ScalarField2D& field, float t) {
            ShapeParams mainParams = m_mainShape;
            ShapeParams secondaryParams = m_secondaryShape;
            MotionController::updateShapeParamsWithCrossOrbit(
                mainParams, secondaryParams, m_motion, m_secondaryMotion, t, 0.016f, true
            );
            ShapeGenerator::applyShape(field, secondaryParams);
        };
    }
    
    // Getters and Setters
    ShapeParams& getMainShape() { return m_mainShape; }
    ShapeParams& getSecondaryShape() { return m_secondaryShape; }
    ShapeParams& getThirdShape() { return m_thirdShape; } // 新增
    MotionParams& getMotion() { return m_motion; }
    MotionParams& getSecondaryMotion() { return m_secondaryMotion; }
    MotionParams& getThirdMotion() { return m_thirdMotion; } // 新增
    BooleanMode& getBooleanMode() { return m_booleanMode; }
    float getTime() const { return m_time; }
    
    // 便捷的设置方法
    void setMainShapeType(ShapeType type) { m_mainShape.type = type; }
    void setSecondaryShapeType(ShapeType type) { m_secondaryShape.type = type; }
    void setThirdShapeType(ShapeType type) { m_thirdShape.type = type; } // 新增
    void setMotionType(MotionType type) { /* 已废弃，无type字段 */ }
    void setSecondaryMotionType(MotionType type) { /* 已废弃，无type字段 */ }
    void setThirdMotionType(MotionType type) { /* 已废弃，无type字段 */ } // 新增
    void setBooleanMode(BooleanMode mode) { m_booleanMode = mode; }
    
    // 获取状态信息
    std::string getMainShapeName() const { return ShapeGenerator::getShapeName(m_mainShape.type); }
    std::string getSecondaryShapeName() const { return ShapeGenerator::getShapeName(m_secondaryShape.type); }
    std::string getThirdShapeName() const { return ShapeGenerator::getShapeName(m_thirdShape.type); } // 新增
    std::string getMotionStatus() const { return MotionController::getMotionStatus(m_motion); }
    std::string getSecondaryMotionStatus() const { return MotionController::getMotionStatus(m_secondaryMotion); }
    std::string getThirdMotionStatus() const { return MotionController::getMotionStatus(m_thirdMotion); } // 新增
    std::string getBooleanModeName() const { return BooleanProcessor::getBooleanModeName(m_booleanMode); }
    
    // 交叉orbit模式控制
    void setUseCrossOrbit(bool use) { m_use_cross_orbit = use; }
    bool getUseCrossOrbit() const { return m_use_cross_orbit; }
};
