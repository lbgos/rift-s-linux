// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#version 450
layout(binding = 0) uniform sampler2D source;
layout(push_constant) uniform Frame {
    vec4 bounds;
    uint srgb;
} frame;
layout(location = 0) in vec2 uv;
layout(location = 0) out vec4 color;
void main() {
    vec3 rgb = texture(source, uv).rgb;
    // Sampling an sRGB image decodes it; the UNORM swapchain expects encoded
    // values.
    if (frame.srgb != 0u)
        rgb = mix(12.92 * rgb, 1.055 * pow(max(rgb, vec3(0)), vec3(1.0 / 2.4)) - 0.055,
                  greaterThan(rgb, vec3(0.0031308)));
    color = vec4(rgb, 1.0);
}
