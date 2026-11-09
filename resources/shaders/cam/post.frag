#version 440
// Same FXAA and screen-space AO policy as the OpenGL simulator.
layout(location = 0) in vec2 vUv;
layout(binding = 1) uniform sampler2D uColorTexture;
layout(binding = 2) uniform sampler2D uDepthTexture;
layout(std140, binding = 0) uniform Post {
    vec4 sizeClip; // texel size, near, far
    vec4 options; // orthographic, AO enabled
} post;
layout(location = 0) out vec4 fragColor;
float linearDepth(float depth) {
    float nearPlane = post.sizeClip.z;
    float farPlane = post.sizeClip.w;
    if (post.options.x > 0.5) {
        return mix(
            nearPlane, farPlane, depth);
    }
    float ndc = depth * 2.0 - 1.0;
    return
        (2.0 * nearPlane * farPlane) /
        (farPlane + nearPlane -
         ndc * (farPlane - nearPlane));
}

float luminance(vec3 color) {
    return dot(color, vec3(0.299, 0.587, 0.114));
}

vec4 fxaa(vec2 uv) {
    vec3 rgbNorthWest = texture(
        uColorTexture,
        clamp(uv + vec2(-1.0, -1.0) * post.sizeClip.xy,
              vec2(0.0), vec2(1.0))).rgb;
    vec3 rgbNorthEast = texture(
        uColorTexture,
        clamp(uv + vec2(1.0, -1.0) * post.sizeClip.xy,
              vec2(0.0), vec2(1.0))).rgb;
    vec3 rgbSouthWest = texture(
        uColorTexture,
        clamp(uv + vec2(-1.0, 1.0) * post.sizeClip.xy,
              vec2(0.0), vec2(1.0))).rgb;
    vec3 rgbSouthEast = texture(
        uColorTexture,
        clamp(uv + vec2(1.0, 1.0) * post.sizeClip.xy,
              vec2(0.0), vec2(1.0))).rgb;
    vec4 center = texture(uColorTexture, uv);
    float lumaNorthWest = luminance(rgbNorthWest);
    float lumaNorthEast = luminance(rgbNorthEast);
    float lumaSouthWest = luminance(rgbSouthWest);
    float lumaSouthEast = luminance(rgbSouthEast);
    float lumaCenter = luminance(center.rgb);
    float lumaMinimum = min(
        lumaCenter,
        min(min(lumaNorthWest, lumaNorthEast),
            min(lumaSouthWest, lumaSouthEast)));
    float lumaMaximum = max(
        lumaCenter,
        max(max(lumaNorthWest, lumaNorthEast),
            max(lumaSouthWest, lumaSouthEast)));
    vec2 direction = vec2(
        -((lumaNorthWest + lumaNorthEast) -
          (lumaSouthWest + lumaSouthEast)),
        (lumaNorthWest + lumaSouthWest) -
          (lumaNorthEast + lumaSouthEast));
    float directionReduce = max(
        0.0078125,
        0.03125 * 0.25 *
            (lumaNorthWest + lumaNorthEast +
             lumaSouthWest + lumaSouthEast));
    float reciprocalMinimum = 1.0 /
        (min(abs(direction.x), abs(direction.y)) +
         directionReduce);
    direction = clamp(
        direction * reciprocalMinimum,
        vec2(-8.0), vec2(8.0)) * post.sizeClip.xy;
    vec3 rgbA = 0.5 * (
        texture(uColorTexture, clamp(
            uv + direction * (1.0 / 3.0 - 0.5),
            vec2(0.0), vec2(1.0))).rgb +
        texture(uColorTexture, clamp(
            uv + direction * (2.0 / 3.0 - 0.5),
            vec2(0.0), vec2(1.0))).rgb);
    vec3 rgbB = rgbA * 0.5 + 0.25 * (
        texture(uColorTexture, clamp(
            uv + direction * -0.5,
            vec2(0.0), vec2(1.0))).rgb +
        texture(uColorTexture, clamp(
            uv + direction * 0.5,
            vec2(0.0), vec2(1.0))).rgb);
    float lumaB = luminance(rgbB);
    return vec4(
        lumaB < lumaMinimum || lumaB > lumaMaximum
            ? rgbA : rgbB,
        center.a);
}

void main() {
    vec4 color = fxaa(vUv);
    if (post.options.y < 0.5) {
        fragColor = color;
        return;
    }
    float rawDepth =
        texture(uDepthTexture, vUv).r;
    if (rawDepth >= 0.999999) {
        fragColor = color;
        return;
    }

    float centerDepth =
        linearDepth(rawDepth);
    vec2 directions[8] = vec2[](
        vec2(1.0, 0.0),
        vec2(-1.0, 0.0),
        vec2(0.0, 1.0),
        vec2(0.0, -1.0),
        vec2(0.707, 0.707),
        vec2(-0.707, 0.707),
        vec2(0.707, -0.707),
        vec2(-0.707, -0.707));
    float occlusion = 0.0;
    for (int radiusIndex = 0;
         radiusIndex < 3; ++radiusIndex) {
        float radius =
            1.5 + float(radiusIndex) * 2.0;
        for (int index = 0;
             index < 8; ++index) {
            vec2 sampleUv = clamp(
                vUv +
                    directions[index] *
                    post.sizeClip.xy * radius,
                vec2(0.0), vec2(1.0));
            float sampleDepth = linearDepth(
                texture(
                    uDepthTexture,
                    sampleUv).r);
            float gap =
                centerDepth - sampleDepth;
            float bias =
                max(0.0005,
                    centerDepth * 0.0008);
            float rangeWeight =
                1.0 - smoothstep(
                    centerDepth * 0.02,
                    centerDepth * 0.16,
                    abs(gap));
            occlusion +=
                step(bias, gap) *
                rangeWeight;
        }
    }
    float ambient = clamp(
        1.0 - 0.55 *
            occlusion / 24.0,
        0.45, 1.0);
    fragColor = vec4(
        color.rgb * ambient,
        color.a);
}
