#version 440
layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(std140, binding = 0) uniform Scene {
    mat4 viewProjection;
    mat4 model;
    vec4 color;
    vec4 options; // x: line pass, y: current simulation step
} scene;
layout(location = 0) out vec3 worldNormal;
layout(location = 1) out vec2 pathData;
void main()
{
    gl_Position = scene.viewProjection * scene.model * vec4(position, 1.0);
    worldNormal = transpose(inverse(mat3(scene.model))) * normal;
    pathData = normal.xy;
}
