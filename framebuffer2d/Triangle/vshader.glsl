#version 330 core
in vec3 vpoint;

const vec3 COLORS[3] = vec3[](
    vec3(1.0,0.0,0.0),
    vec3(0.0,1.0,0.0),
    vec3(0.0,0.0,1.0));

// (x,y,degrees,scale)
// (x,y) passed each frame
// (degre,scale) remain constant, edit here to change animation
vec4 PTS[4] = vec4[](
    vec4(-1,-1,  0, .5),
    vec4(-1, 1, 30, .8),
    vec4( 1,-1, 60,1.1),
    vec4( 1, 1, 90,1.4));

uniform mat4 M;         // primary transformation matrix
uniform float time;     // [0,1] where on path to draw triangle
uniform vec2[] cpts;    // bezier control points

out vec3 fcolor;

// De Casteljau's algorithm to find position on bezier curve
vec4 bezier(vec4 p0, vec4 p1, vec4 p2, vec4 p3, float t){
    vec4 p01 = mix(p0, p1, t);
    vec4 p12 = mix(p1, p2, t);
    vec4 p23 = mix(p2, p3, t);

    vec4 p012 = mix(p01, p12, t);
    vec4 p123 = mix(p12, p23, t);

    return mix(p012, p123, t);
}

void main() {

    // update PTS with control point locations
    PTS[0].xy = cpts[0].xy;
    PTS[1].xy = cpts[1].xy;
    PTS[2].xy = cpts[2].xy;
    PTS[3].xy = cpts[3].xy;

    /// find position on bezier curve
    vec4 result = bezier(PTS[0],PTS[1],PTS[2],PTS[3],time);

    ///--- Translation
    mat4 T = mat4(1);
    T[3][0] = result[0];
    T[3][1] = result[1];

    ///--- Rotation
    mat3 R = mat3(1);
    float alpha = radians(result[2]);
    R[0][0] =  cos(alpha);
    R[0][1] =  sin(alpha);
    R[1][0] = -sin(alpha);
    R[1][1] =  cos(alpha);

    ///--- Scale
    mat3 S = mat3(result[3]);

    ///--- Put it all together
    gl_Position = T*mat4(S)*mat4(R)*M*vec4(vpoint, 1.0);

    // interpolate colours
    fcolor = COLORS[gl_VertexID];
}
