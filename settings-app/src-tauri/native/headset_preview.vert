// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#version 450
// One triangle covering the viewport; uv maps the viewport onto the overlay
// texture bounds (u/v min to max, possibly flipped).
layout(push_constant) uniform Frame {
    vec4 bounds;
    uint srgb;
} frame;
layout(location = 0) out vec2 uv;
void main() {
    vec2 corner = vec2((gl_VertexIndex << 1) & 2, gl_VertexIndex & 2);
    uv = mix(frame.bounds.xy, frame.bounds.zw, corner);
    gl_Position = vec4(corner * 2.0 - 1.0, 0.0, 1.0);
}
