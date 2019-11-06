R"(
#version 330 core
in vec3 vposition;
in vec2 vtexcoord;

uniform mat4 M;
uniform mat4 V;
uniform mat4 P;

out vec2 uv;


void main() {
    uv = vtexcoord;
    // TODO: Calculate vertical displacement h given uv
	float h=0;    	       
    
    // Multiply model, view, and projection matrices in the correct order
		
    vec4 displacement = M * vec4(vposition + vec3(0, h, 0), 1.0);
    gl_Position = P * V * displacement;
}
)"
