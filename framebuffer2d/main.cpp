#include <OpenGP/GL/Application.h>
#include <OpenGP/external/LodePNG/lodepng.cpp>
#include "Triangle/Triangle.h"

using namespace OpenGP;

const int width = 640, height = 640;
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

static Triangle tri1;
static Triangle tri2;

const float SpeedFactor = 1;
void init();
void quadInit(std::unique_ptr<GPUMesh>& quad);
void loadTexture(std::unique_ptr<RGBA8Texture>& texture, const char* filename);
void drawScene(float timeCount);

static std::unique_ptr<GPUMesh> quad;

static std::unique_ptr<Shader> quadShader;
static std::unique_ptr<Shader> fbShader;

static std::unique_ptr<RGBA8Texture> cat;
static std::unique_ptr<RGBA8Texture> night;

/// TODO: declare Framebuffer and color buffer texture

static std::unique_ptr<Framebuffer> fb;
static std::unique_ptr<RGBA8Texture> c_buf;

int main(int, char**) {

    Application app;
    init();

    /// TODO: initialize framebuffer
    fb = std::unique_ptr<Framebuffer>(new Framebuffer());

    /// TODO: initialize color buffer texture, and allocate memory
    c_buf = std::unique_ptr<RGBA8Texture>(new RGBA8Texture());
    c_buf->allocate(width, height);

    /// TODO: attach color texture to framebuffer
    fb->attach_color_texture(*c_buf);

    Window& window = app.create_window([&](Window&) {
        glViewport(0, 0, width, height);

        /// TODO: First draw the scene onto framebuffer
        /// bind and then unbind framebuffer
        fb->bind();
        glClear(GL_COLOR_BUFFER_BIT);
        drawScene(float(glfwGetTime()));
        fb->unbind();

        /// Render to Window, uncomment the lines and do TODOs
        glViewport(0, 0, width, height);
        glClear(GL_COLOR_BUFFER_BIT);
        fbShader->bind();
        // TODO: Bind texture and set uniforms
        glActiveTexture(GL_TEXTURE0);
        c_buf->bind();
        fbShader->set_uniform("tex", 0);
        fbShader->set_uniform("tex_width", float(width));
        fbShader->set_uniform("tex_height", float(height));
        quad->set_attributes(*fbShader);
        quad->draw();

        c_buf->unbind();
        fbShader->unbind();
    });
    window.set_title("FrameBuffer");
    window.set_size(width, height);

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

    quadInit(quad);

    loadTexture(cat, "nyancat.png");
    loadTexture(night, "night.png");

    tri1.init();
    tri2.init();
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
    // Used snippet from https://raw.githubusercontent.com/lvandeve/lodepng/master/examples/example_decode.cpp
    std::vector<unsigned char> image; //the raw pixels
    unsigned width, height;
    //decode
    unsigned error = lodepng::decode(image, width, height, filename);
    //if there's an error, display it
    if (error) std::cout << "decoder error " << error << ": " << lodepng_error_text(error) << std::endl;
    //the pixels are now in the vector "image", 4 bytes per pixel, ordered RGBARGBA..., use it as texture, draw it, ...

    // unfortunately they are upside down...lets fix that
    unsigned char* row = new unsigned char[4 * width];
    for (unsigned i = 0; i < height / 2; ++i) {
        memcpy(row, &image[4 * i * width], 4 * width * sizeof(unsigned char));
        memcpy(&image[4 * i * width], &image[image.size() - 4 * (i + 1) * width], 4 * width * sizeof(unsigned char));
        memcpy(&image[image.size() - 4 * (i + 1) * width], row, 4 * width * sizeof(unsigned char));
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

    // the background
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

    float xcord = 0.7f * std::cos(t);
    float ycord = 0.7f * std::sin(t);
    TRS *= Eigen::Translation3f(xcord, ycord, 0);
    TRS *= Eigen::AngleAxisf(float(double(t) + M_PI / 2), Eigen::Vector3f::UnitZ());
    TRS *= Eigen::AlignedScaling3f(0.2f, 0.2f, 1);

    quadShader->bind();
    quadShader->set_uniform("M", TRS.matrix());
    // Make texture unit 0 active
    glActiveTexture(GL_TEXTURE0);
    // Bind the texture to the active unit for drawing
    cat->bind();
    // Set the shader's texture uniform to the index of the texture unit we have
    // bound the texture to
    quadShader->set_uniform("tex", 0);
    quad->set_attributes(*quadShader);
    quad->draw();
    cat->unbind();
    quadShader->unbind();

    glDisable(GL_BLEND);

    tri1.draw(t);
    tri1.draw(t+1);
}
