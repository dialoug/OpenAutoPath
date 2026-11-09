#version 440
layout(location = 0) in vec3 worldNormal;
layout(location = 1) in vec2 pathData;
layout(std140, binding = 0) uniform Scene {
    mat4 viewProjection;
    mat4 model;
    vec4 color;
    vec4 options;
} scene;
layout(location = 0) out vec4 fragColor;
void main()
{
    if (scene.options.x > 0.5) {
        fragColor = pathData.x >= scene.options.y + 0.5
            ? vec4(0.55, 0.59, 0.66, 0.30)
            : (pathData.y > 0.5 ? vec4(0.25, 0.72, 1.0, 0.82) : vec4(0.97, 0.82, 0.18, 0.90));
        fragColor.a *= scene.options.z;
    } else {
        vec3 normal = normalize(worldNormal);
        float lighting = 0.30 + 0.70 * max(dot(normal, normalize(vec3(0.35, 0.45, 1.0))), 0.0);
        fragColor = vec4(scene.color.rgb * lighting, scene.color.a);
    }
}
