/* Assignment 2: Flash Animation
 * CSCI 4471: Computer Graphics
 * Seamus MacInnes
 * A00415673
 * Nov 05 2019
 */

#include <OpenGP/GL/Application.h>
#include <OpenGP/external/LodePNG/lodepng.cpp>
#include "Triangle/Triangle.h"

using namespace OpenGP;
#define POINTSIZE 10.0f

const int width = 640, height = 640;
const float SpeedFactor = 1;
typedef Eigen::Transform<float, 3, Eigen::Affine> Transform;

static const char* fb_vshader =
#include "fb_vshader.glsl"
;
static const char* fb_fshader =
#include "fb_fshader.glsl"
;
static const char* quad_vshader =
#include "quad_vshader.glsl"
;
static const char* quad_fshader =
#include "quad_fshader.glsl"
;
static const char* line_vshader =
#include "line_vshader.glsl"
;
static const char* line_fshader =
#include "line_fshader.glsl"
;

static Triangle tri1;
static Triangle tri2;

void init();
void quadInit(std::unique_ptr<GPUMesh>& quad);
void loadTexture(std::unique_ptr<RGBA8Texture>& texture, const char* filename);
void drawScene(float timeCount);

static std::unique_ptr<GPUMesh> quad;

static std::unique_ptr<Shader> quadShader;
static std::unique_ptr<Shader> fbShader;

static std::unique_ptr<Shader> lineShader;
static std::unique_ptr<GPUMesh> line;
static std::vector<Vec2> controlPoints;

static std::unique_ptr<RGBA8Texture> cat;
static std::unique_ptr<RGBA8Texture> night;

static std::unique_ptr<Framebuffer> fb;
static std::unique_ptr<RGBA8Texture> c_buf;

int main(int, char**) {

    // Hold mouse position and selected control point
    Vec2 position = Vec2(0,0);
    Vec2 *selection = nullptr;

    Application app;
    init();

    // initialize framebuffer
    fb = std::unique_ptr<Framebuffer>(new Framebuffer());

    // initialize color buffer texture, and allocate memory
    c_buf = std::unique_ptr<RGBA8Texture>(new RGBA8Texture());
    c_buf->allocate(width, height);

    // attach color texture to framebuffer
    fb->attach_color_texture(*c_buf);

    // CREATE WINDOW
    Window& window = app.create_window([&](Window&) {
        glViewport(0, 0, width, height);
        glPointSize(POINTSIZE);

        fb->bind();
        glClear(GL_COLOR_BUFFER_BIT);
        drawScene(float(glfwGetTime()));
        fb->unbind();

        fbShader->bind();
        c_buf->bind();
        glActiveTexture(GL_TEXTURE0);
        fbShader->set_uniform("tex", 0);
        fbShader->set_uniform("tex_width", float(width));
        fbShader->set_uniform("tex_height", float(height));
        quad->set_attributes(*fbShader);
        quad->draw();
        c_buf->unbind();
        fbShader->unbind();

        // DRAW LINE THROUGH CONTROL POINTS
        lineShader->bind();
        lineShader->set_uniform("selection", -1);
        line->set_attributes(*lineShader);
        line->set_mode(GL_LINE_STRIP);
        line->draw();

        // DRAW CONTROL POINTS
        if(selection!=nullptr)
            lineShader->set_uniform("selection",
                                    int(selection-&controlPoints[0]));
        line->set_mode(GL_POINTS);
        line->draw();
        lineShader->unbind();

    });

    window.set_title("Assignment 2: Flash Animation - A00415673");
    window.set_size(width, height);

    // CONTROLLING BEZIER WITH MOUSE CURSOR

    // Mouse movement callback
    window.add_listener<MouseMoveEvent>([&](const MouseMoveEvent &m){
        // Mouse position in clip coordinates
        Vec2 p = 2.0f*(Vec2(m.position.x()/width,-m.position.y()/height)
                       - Vec2(0.5f,-0.5f));
        if( selection && (p-position).norm() > 0.0f) {
            // Make selected control points move with cursor
            selection->x() = p.x();
            selection->y() = p.y();
            line->set_vbo<Vec2>("vposition", controlPoints);
        }
        position = p;
    });

    // Mouse click callback
    window.add_listener<MouseButtonEvent>([&](const MouseButtonEvent &e){
        // Mouse selection case
        if( e.button == GLFW_MOUSE_BUTTON_LEFT && !e.released) {
            selection = nullptr;
            for(auto&& v : controlPoints) {
                if ( (v-position).norm() < POINTSIZE/std::min(width,height) ) {
                    selection = &v;
                    break;
                }
            }
        }
        // Mouse release case
        if( e.button == GLFW_MOUSE_BUTTON_LEFT && e.released) {
            if(selection) {
                selection->x() = position.x();
                selection->y() = position.y();
                selection = nullptr;
                line->set_vbo<Vec2>("vposition", controlPoints);
            }
        }
    });

    return app.run();
}

void init() {
    glClearColor(1, 1, 1, /*solid*/1.0);

    fbShader = std::unique_ptr<Shader>(new Shader());
    fbShader->verbose = true;
    fbShader->add_vshader_from_source(fb_vshader);
    fbShader->add_fshader_from_source(fb_fshader);
    fbShader->link();

    quadShader = std::unique_ptr<Shader>(new Shader());
    quadShader->verbose = true;
    quadShader->add_vshader_from_source(quad_vshader);
    quadShader->add_fshader_from_source(quad_fshader);
    quadShader->link();

    lineShader = std::unique_ptr<Shader>(new Shader());
    lineShader->verbose = true;
    lineShader->add_vshader_from_source(line_vshader);
    lineShader->add_fshader_from_source(line_fshader);
    lineShader->link();

    // initial control points for bezier animation path
    controlPoints = std::vector<Vec2>();
    controlPoints.push_back(Vec2(-0.7f,-0.2f));
    controlPoints.push_back(Vec2(-0.3f, 0.2f));
    controlPoints.push_back(Vec2( 0.3f, 0.5f));
    controlPoints.push_back(Vec2( 0.7f, 0.0f));

    quadInit(quad);

    loadTexture(cat, "nyancat.png");
    loadTexture(night, "night.png");

    tri1.init();
    tri2.init();

    line = std::unique_ptr<GPUMesh>(new GPUMesh());
    line->set_vbo<Vec2>("vposition", controlPoints);
    std::vector<unsigned int> indices = {0,1,2,3};
    line->set_triangles(indices);
}

void quadInit(std::unique_ptr<GPUMesh>& quad) {
    quad = std::unique_ptr<GPUMesh>(new GPUMesh());
    std::vector<Vec3> quad_vposition = {
        Vec3(-1, -1, 0),
        Vec3(-1,  1, 0),
        Vec3(1, -1, 0),
        Vec3(1,  1, 0)
    };
    quad->set_vbo<Vec3>("vposition", quad_vposition);
    std::vector<unsigned int> quad_triangle_indices = {
        0, 2, 1, 1, 2, 3
    };
    quad->set_triangles(quad_triangle_indices);
    std::vector<Vec2> quad_vtexcoord = {
        Vec2(0, 0),
        Vec2(0,  1),
        Vec2(1, 0),
        Vec2(1,  1)
    };
    quad->set_vtexcoord(quad_vtexcoord);
}

void loadTexture(std::unique_ptr<RGBA8Texture>& texture, const char* filename) {
    std::vector<unsigned char> image; //the raw pixels
    unsigned width, height;
    //decode
    unsigned error = lodepng::decode(image, width, height, filename);
    //if there's an error, display it
    if (error) std::cout << "decoder error " << error << ": "
                         << lodepng_error_text(error) << std::endl;
    // unfortunately they are upside down...lets fix that
    unsigned char* row = new unsigned char[4 * width];
    for (unsigned i = 0; i < height / 2; ++i) {
        memcpy(row, &image[4 * i * width],
                4 * width * sizeof(unsigned char));
        memcpy(&image[4 * i * width],&image[image.size() - 4 * (i + 1) * width],
                4 * width * sizeof(unsigned char));
        memcpy(&image[image.size() - 4 * (i + 1) * width], row,
                4 * width * sizeof(unsigned char));
    }
    delete[] row;

    texture = std::unique_ptr<RGBA8Texture>(new RGBA8Texture());
    texture->upload_raw(int(width), int(height), &image[0]);
}

void drawScene(float timeCount)
{
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    float t = timeCount * SpeedFactor;

    //--- THE BACKGROUND TEXTURE
    Transform TRS = Transform::Identity();
    quadShader->bind();
    quadShader->set_uniform("M", TRS.matrix());
    // Make texture unit 0 active
    glActiveTexture(GL_TEXTURE0);
    // Bind the texture to the active unit for drawing
    night->bind();
    // Set the shader's texture uniform to the index of the texture unit we have
    // bound the texture to
    quadShader->set_uniform("tex", 0);
    quad->set_attributes(*quadShader);
    quad->draw();
    night->unbind();

    //--- THE FLYING NYAN CAT
    float xcord = 0.7f * std::cos(t);
    float ycord = 0.7f * std::sin(t);
    TRS *= Eigen::Translation3f(xcord, ycord, 0);
    TRS *= Eigen::AngleAxisf(float(double(t) + M_PI / 2), Eigen::Vector3f::UnitZ());
    TRS *= Eigen::AlignedScaling3f(0.2f, 0.2f, 1);
    quadShader->bind();
    quadShader->set_uniform("M", TRS.matrix());
    glActiveTexture(GL_TEXTURE0);
    cat->bind();
    quadShader->set_uniform("tex", 0);
    quad->set_attributes(*quadShader);
    quad->draw();
    cat->unbind();
    quadShader->unbind();

    glDisable(GL_BLEND);

    //--- THE TRIANGLES
    tri1.draw(t,controlPoints);
    //tri1.draw(t+1,controlPoints);

}
