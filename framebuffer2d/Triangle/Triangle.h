/* Assignment 2: Flash Animation
 * CSCI 4471: Computer Graphics
 * Seamus MacInnes
 * A00415673
 * Nov 05 2019
 */

#pragma once
#include "icg_common.h"

class Triangle{
private:
    GLuint _vao; ///< vertex array object
    GLuint _pid; ///< GLSL shader program ID
    GLuint _vbo; ///< memory buffer
public:
    void init(){
        ///--- Compile the shaders
        /// DON'T KNOW WHY NEED ABSOLUTE PATHS BUT JUST DO
        // C:/Users/smaci/Documents/Classes/CSCI_4471/CGLab/framebuffer2d/Triangle
        // C:/Users/Seamus Macinnes/Documents/CGLab/framebuffer2d/Triangle
        _pid = OpenGP::load_shaders(
            "C:/Users/Seamus Macinnes/Documents/CGLab/framebuffer2d/Triangle/vshader.glsl",
            "C:/Users/Seamus Macinnes/Documents/CGLab/framebuffer2d/Triangle/fshader.glsl");

        if(!_pid) exit(EXIT_FAILURE);
        glUseProgram(_pid);

        ///--- Vertex one vertex Array
        glGenVertexArrays(1, &_vao);
        glBindVertexArray(_vao);

        ///--- Vertex Buffer
        const GLfloat vpoint[] = { /*V1*/-1.0f, -1.0f, 0.0f,
                                   /*V2*/ 1.0f, -1.0f, 0.0f,
                                   /*V3*/ 0.0f,  1.0f, 0.0f};
        glGenBuffers(1, &_vbo);
        glBindBuffer(GL_ARRAY_BUFFER, _vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vpoint), vpoint, GL_STATIC_DRAW);

        ///--- vpoint shader attribute
        GLuint position = unsigned(glGetAttribLocation(_pid, "vpoint"));
        glEnableVertexAttribArray(position); /// Enable it
        glVertexAttribPointer(position, 3, GL_FLOAT,
                              DONT_NORMALIZE, ZERO_STRIDE, ZERO_BUFFER_OFFSET);

        ///--- to avoid the current object being polluted
        glBindVertexArray(0);
        glUseProgram(0);
    }

    void cleanup(){
        glBindVertexArray(0);
        glUseProgram(0);
        glDeleteBuffers(1, &_vbo);
        glDeleteProgram(_pid);
        glDeleteVertexArrays(1, &_vao);
    }

    void draw(float t,const vector<vec2>& controlPoints){
        glUseProgram(_pid);
        glBindVertexArray(_vao);

        // FIRST LEVEL OF TRANSFORMATION HIERARCHY

        // center translations around bottom left point of triangle
        mat4 T1 = Eigen::Affine3f(Eigen::Translation3f(1,1, 0)).matrix();
        // scale matrix down to size
        mat4 S = mat4::Identity();
        S(0,0) = .25f;
        S(1,1) = -.15f;
        // rotate according to time for flapping affect
        mat4 R = Eigen::Affine3f(Eigen::AngleAxisf(.3f*std::sin(5*t),
                                                   vec3::UnitZ())).matrix();
        // pass resulting matrix to shader
        mat4 M = R*S*T1;
        GLuint M_id = unsigned(glGetUniformLocation(_pid, "M"));
        glUniformMatrix4fv(int(M_id), 1, GL_FALSE, M.data());

        // pass time to shader for bezier curve animation path
        float unused;
        float SPEED_UP_FACTOR = 0.5;
        GLuint t_id = unsigned(glGetUniformLocation(_pid, "time"));
        glUniform1f(int(t_id), modf(t*SPEED_UP_FACTOR,&unused));

        // control points to shader for bezier curve animation path
        vec2 cpts[] = {controlPoints.at(0),
                       controlPoints.at(1),
                       controlPoints.at(2),
                       controlPoints.at(3)};
        GLuint c_id = unsigned(glGetUniformLocation(_pid, "cpts"));
        glUniform2fv(int(c_id), 4, reinterpret_cast<GLfloat *>(cpts));

        ///--- Draw
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glBindVertexArray(0);
        glUseProgram(0);
    }
};
