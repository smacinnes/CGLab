R"(
#version 330 core
uniform sampler2D noiseTex;

in vec2 uv;

out vec4 color;

void main() {
    // Texture size in pixels
    ivec2 size = textureSize(noiseTex, 0);

    /// TODO: Calculate surface normal N
    /// HINT: Use textureOffset(,,) to read height at uv + pixelwise offset
    vec3 A = textureOffset(noiseTex, uv, ivec2(1,0)).xyz;
    vec3 B = textureOffset(noiseTex, uv, ivec2(-1,0)).xyz;
    vec3 C = textureOffset(noiseTex, uv, ivec2(0,1)).xyz;
    vec3 D = textureOffset(noiseTex, uv, ivec2(0,-1)).xyz;
    vec3 N = normalize( cross(normalize(A-B), normalize(C-D)) );

    // Sample height from texture and normalize to [0,1]
    vec3 c = vec3((texture(noiseTex, uv).r + 1.0f)/2.0f);

    // (Optional): Visualize normals as RGB vector

    color = vec4(1,1,1,1);
}
)"
