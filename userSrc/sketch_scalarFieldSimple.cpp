// 0629_4_layers.cpp

#include "../include/alice2.h"
#include "scalarField.h"
#include "../src/sketches/SketchRegistry.h"
#include <cmath>
#include <vector>
#include <GL/gl.h>    // for glPushMatrix, glPopMatrix, glTranslatef

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

using namespace alice2;

class ScalarFieldRectanglesLayersSketch : public ISketch {
private:
    static constexpr int    LAYER_COUNT   = 40;     // Number of stacked layers
    static constexpr float  LAYER_HEIGHT  = 1.0f;   // Vertical spacing between layers
    static constexpr float  TIME_OFFSET   = 0.05f;  // Time step between successive layers

    ScalarField2D                m_baseField;      // Base scalar field for point drawing
    std::vector<ScalarField2D>   m_layers;         // One field per layer
    float                        m_time;           // Animation time

    // Rectangle parameters
    Vec3                         m_rect1_center, m_rect1_size;
    Vec3                         m_rect2_center, m_rect2_size;
    Vec3                         m_rect3_center, m_rect3_size;

public:
    ScalarFieldRectanglesLayersSketch()
      : m_baseField(Vec3(-50, -50, 0), Vec3(50, 50, 0), 200, 200)
      , m_time(0.0f)
      , m_rect1_center( Vec3(  0,   0, 0) ), m_rect1_size( Vec3(20,15,0) )
      , m_rect2_center( Vec3( 15,  10, 0) ), m_rect2_size( Vec3(15,10,0) )
      , m_rect3_center( Vec3(-15, -10, 0) ), m_rect3_size( Vec3(15,10,0) )
    {
        m_layers.reserve(LAYER_COUNT);
        for(int i = 0; i < LAYER_COUNT; ++i) {
            m_layers.emplace_back(m_baseField);  // copy base grid
        }
    }

    std::string getName() const override {
        return "SDF Boundary Stack (40 Layers)";
    }
    std::string getDescription() const override {
        return "Stacks 40 successive SDF isocontours vertically";
    }
    std::string getAuthor() const override {
        return "alice2 & ChatGPT";
    }

    void setup() override {
        scene().setBackgroundColor(Vec3(0.1f,0.1f,0.15f));
        scene().setShowGrid(false);
        scene().setShowAxes(true);
        scene().setAxesLength(10.0f);

        // initial fill of layers
        for(int i = 0; i < LAYER_COUNT; ++i) {
            generateLayerField(m_time + i * TIME_OFFSET, m_layers[i]);
        }
    }

    void update(float deltaTime) override {
        m_time += deltaTime;

        // regenerate all layers at staggered times
        for(int i = 0; i < LAYER_COUNT; ++i) {
            float t = m_time + i * TIME_OFFSET;
            generateLayerField(t, m_layers[i]);
        }
    }

    void draw(Renderer& renderer, Camera& camera) override {
        // draw the dynamic SDF point field at the base
        m_baseField.draw_points(renderer, 2);

        // draw stacked isocontours with OpenGL matrix stack
        for(int i = 0; i < LAYER_COUNT; ++i) {
            float alpha = 1.0f - float(i) / float(LAYER_COUNT);
            renderer.setColor(Vec3(alpha,alpha,alpha));  // fade older layers

            // push model matrix, translate in Z, draw, pop
            glPushMatrix();
            glTranslatef(0.0f, 0.0f, i * LAYER_HEIGHT);
            m_layers[i].drawIsocontours(renderer, /*isoValue=*/0.0f);
            glPopMatrix();
        }

        // draw UI info
        renderer.setColor(Vec3(1,1,0));
        renderer.drawString("Time: " + std::to_string(m_time).substr(0,5), 10, 30);
        renderer.drawString("Layers: " + std::to_string(LAYER_COUNT), 10, 50);
        renderer.drawString("Spacing: " + std::to_string(LAYER_HEIGHT), 10, 70);
        renderer.drawString("Use F to toggle grid, G to toggle axes", 10, 90);
    }

    bool onKeyPress(unsigned char key, int x, int y) override {
        switch(key) {
            case 'f': case 'F':
                scene().setShowGrid(!scene().getShowGrid());
                return true;
            case 'g': case 'G':
                scene().setShowAxes(!scene().getShowAxes());
                return true;
        }
        return false;
    }

private:
    // Regenerate one layer's field at time t
    void generateLayerField(float t, ScalarField2D& field) {
        field.clear_field();

        // animated rectangle 1
        float r1w = 20.0f + std::sin(t * 1.5f) * 5.0f;
        float r1h = 15.0f + std::cos(t * 1.2f) * 3.0f;
        float a1  = t * 0.2f;

        // rectangles 2 & 3 share same rotation
        float a23 = -t * 0.3f;

        // build SDF: smooth-union of three rotating rectangles
        ScalarField2D tmp = field;
        field.apply_scalar_rect(m_rect1_center, Vec3(r1w, r1h,0), a1);
        tmp.apply_scalar_rect(m_rect2_center, m_rect2_size, a23);
        field.boolean_smin(tmp, 2.0f);
        tmp = field;
        tmp.apply_scalar_rect(m_rect3_center, m_rect3_size, a23);
        field.boolean_smin(tmp, 2.0f); 
    }
};

// register sketch
ALICE2_REGISTER_SKETCH_AUTO(ScalarFieldRectanglesLayersSketch);
