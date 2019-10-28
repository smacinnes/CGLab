#include "icg_common.h"
#include <OpenGP/GL/Application.h>

using namespace std;
using namespace OpenGP;

/// Vertex position of the triangle
const GLfloat vpoint[] = {
       -1.0f, -1.0f, 0.0f,
       1.0f, -1.0f, 0.0f,
       0.0f,  1.0f, 0.0f,};

void init(){
    ///--- Sets background color
    //glClearColor(/*dark blue*/ .937,.937,.937, /*solid*/1.0 );
    glClearColor(0.0f, 0.0f, 0.4f, 1.0f);

    ///@TODO--- Compile the shaders
    char vshader_location[] = "../../../glsl/vshader.glsl";
    char fshader_location[] = "../external/OpenGP/GL/fshader.glsl";
    GLuint pID = OpenGP::load_shaders(vshader_location,fshader_location);
    if(!pID) exit(EXIT_FAILURE);
    glUseProgram(pID);




    ///@TODO--- Setup vertex array
    /// vertex arrays wrap buffers & attributes together
    /// creating it is mandatory in newer OpenGL versions

    GLuint VertexArrayID;
    glGenVertexArrays(1, &VertexArrayID);
    glBindVertexArray(VertexArrayID);

/*
    GLuint vpoint_id = GLuint(glGetAttribLocation(pID,"vpoint"));
    glEnableVertexAttribArray(vpoint_id);
    glVertexAttribPointer(vpoint_id, 3, GL_FLOAT,
                          GL_FALSE,
                          OpenGP::ZERO_STRIDE,OpenGP::ZERO_BUFFER_OFFSET);
*/

    ///@TODO--- Generate memory for vertexbuffer and bind it

    GLuint vertexbuffer;
    glGenBuffers(1, &vertexbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vpoint), vpoint, GL_STATIC_DRAW);

    /// @TODO---Pass the vertex positions to OpenGL

    ///@TODO--- Creates Vertex Attribute to store Vertex Positions
//*
    glVertexAttribPointer(0, 3, GL_FLOAT,GL_FALSE,
                          OpenGP::ZERO_STRIDE,OpenGP::ZERO_BUFFER_OFFSET);
//*/
}


void display(){
    //glClear(GL_COLOR_BUFFER_BIT);

    ///@TODO--Issue draw command


    glEnableVertexAttribArray(0);
    //glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    /*glVertexAttribPointer(
       0,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
       3,                  // size
       GL_FLOAT,           // type
       GL_FALSE,           // normalized?
       0,                  // stride
       (void*)0            // array buffer offset
    );*/

    glDrawArrays(GL_TRIANGLES, 0, 3);
    glDisableVertexAttribArray(0);


}

int main(int, char**){

    GLFWwindow* window;
    glewExperimental = true; // for core profile

    if (!glfwInit())
    {
        fprintf(stderr, "Failed to initialize GLFW\n");
        return -1;
    }

    // glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); // We want OpenGL 3.3
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    // We don't want the old OpenGL
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Open a window and create its OpenGL context
    int wResolution = 1024;
    int hResolution = 768;
    window = glfwCreateWindow(wResolution, hResolution, "GLSL", NULL, NULL);
    if (window == NULL) {
        fprintf(stderr, "Failed to open GLFW window. "
                        "If you have an Intel GPU, "
                        "they are not 3.3 compatible. "
                        "Try the 2.1 version of the tutorials.\n");
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window); // Initialize GLEW

    if (glewInit() != GLEW_OK) {
        fprintf(stderr, "Failed to initialize GLEW\n");
        return -1;
    }
    init();

    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
    do {
        // Clear the screen.
        glClear(GL_COLOR_BUFFER_BIT);

        // draw things
        display();

        // Swap buffers
        glfwSwapBuffers(window);
        glfwPollEvents();

        // Check if the ESC key was pressed or the window was closed
    } while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
             glfwWindowShouldClose(window) == 0);

    ////Another option for window creation:
    /// one can use OpenGP Application class to create windows as follows.
    //
    //Application app;

    //// Here we ask the application to spawn a new window
    ////
    //// Notice that we keep only a reference to the window object, as it is
    //// owned by the application and can not be copied/moved
    //init();
    //Window& window = app.create_window([](Window&) {  display();});

    //

    //// The window reference can be used to manipulate the window properties
    //window.set_title("GLSL Example");
    //window.set_size(800, 600);

    //// Finally, give control of the main loop to OpenGP by running the app
    //return app.run();
}
