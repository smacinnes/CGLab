R"(
#version 330 core
uniform sampler2D noiseTex;

uniform sampler2D grass;
uniform sampler2D rock;
uniform sampler2D sand;
uniform sampler2D snow;
uniform sampler2D water;

// The camera position
uniform vec3 viewPos;

in vec2 uv;
// Fragment position in world space coordinates
in vec3 fragPos;

out vec4 color;

void main() {

    // Texture size in pixels
    ivec2 size = textureSize(noiseTex, 0);

    /// TODO: Calculate surface normal N
    /// HINT: Use textureOffset(,,) to read height at uv + pixelwise offset
    /// HINT: Account for texture x,y dimensions in world space coordinates (default f_width=f_height=5)

    // temp colouring before textures are applied
    vec3 N  = vec3(0,0,1);
    if (texture2D(noiseTex,uv).r > 0.0f){
        vec3 A = vec3(uv.x +1.0/size.x, uv.y, textureOffset(noiseTex, uv, ivec2(1,0)));
        vec3 B = vec3(uv.x -1.0/size.x, uv.y, textureOffset(noiseTex, uv, ivec2(-1,0)));
        vec3 C = vec3(uv.x, uv.y +1.0/size.y, textureOffset(noiseTex, uv, ivec2(0,1)));
        vec3 D = vec3(uv.x, uv.y -1.0/size.y, textureOffset(noiseTex, uv, ivec2(0,-1)));
        N = normalize( cross(normalize(A-B), normalize(C-D)) );
    }
    /// TODO: Texture according to height and slope
    /// HINT: Read noiseTex for height at uv
    // options for texture are water,sand,grass,rock,snow
    float waterLevel = 0.0f;    // all below this is water
    float sandLevel = 0.02f;    // sand can only be below this
    float sandThresh = 0.1f;    // sand only if not this steep
    float snowLevel = 0.6f;     // snow can only be above this
    float rockThresh = 0.9f;    // rock if steeper than this

    float height = texture2D(noiseTex,uv).r;
    float incline = abs(N.z);

    int tiles = 32; // make nxn grid of the images
    vec2 pixel = tiles*uv - int(tiles*uv);
    // some tweaking to do - ex repeating textures
    if (height <= waterLevel) {
        color = texture(water,pixel);
    } else if (incline >= rockThresh) {
        color = texture(rock,pixel);
    } else if (height >= snowLevel) {
        color = texture(snow,pixel);
    } else if (height <= sandLevel && incline <= sandThresh) {
        color = texture(sand,pixel);
    } else {
        color = texture(grass,pixel);
    }

    /// TODO: Calculate ambient, diffuse, and specular lighting
    /// HINT: max(,) dot(,) reflect(,) normalize()

    vec3 L = normalize(vec3(5,-5,9)-fragPos);
    vec3 R = reflect(-L,N);  // both normalized already
    vec3 V = normalize(viewPos-fragPos);
    float alpha = 2;

    // make dependant on texture?
    vec4 ambient = vec4(0.8f,0.8f,0.8f,1);
    vec4 diffuse = vec4(0.8f,0.8f,0.8f,1)*max(dot(L,N),0);
    vec4 specular = vec4(0.5f,0.5f,0.5f,1)*pow(max(dot(R,V),0),alpha);

    color *= ambient + diffuse + specular;
    color = min(color,1);
    //color = vec4((N + vec3(1.0)) / 2.0f,1);
}
)"
