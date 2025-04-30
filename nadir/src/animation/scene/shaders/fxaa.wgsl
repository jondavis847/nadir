@group(0) @binding(0)
var fxaa_texture: texture_2d<f32>;

@group(0) @binding(1)
var fxaa_sampler: sampler;

struct FxaaUniform {
    pixel: vec2<f32>,  // This is 1.0 / screen_size
};
@group(0) @binding(2)
var<uniform> fxaa: FxaaUniform;

fn luma(color: vec3<f32>) -> f32 {
    return dot(color, vec3(0.299, 0.587, 0.114));
}

struct VertexOutput {
    @builtin(position) pos: vec4<f32>,
    @location(0) uv: vec2<f32>,
};

@vertex
fn vs_main(@builtin(vertex_index) index: u32) -> VertexOutput {
    var positions = array<vec2<f32>, 3>(
        vec2(-1.0, -1.0),
        vec2( 3.0, -1.0),
        vec2(-1.0,  3.0)
    );

    var uvs = array<vec2<f32>, 3>(
        vec2(0.0, 0.0),
        vec2(2.0, 0.0),
        vec2(0.0, 2.0)
    );

    var out: VertexOutput;
    out.pos = vec4(positions[index], 0.0, 1.0);
    out.uv = uvs[index];
    return out;
}

@fragment
fn fs_main(@location(0) uv: vec2<f32>) -> @location(0) vec4<f32> {
    let pixel = fxaa.pixel;

    let rgbNW = textureSample(fxaa_texture, fxaa_sampler, uv + vec2(-1.0, -1.0) * pixel).rgb;
    let rgbNE = textureSample(fxaa_texture, fxaa_sampler, uv + vec2( 1.0, -1.0) * pixel).rgb;
    let rgbSW = textureSample(fxaa_texture, fxaa_sampler, uv + vec2(-1.0,  1.0) * pixel).rgb;
    let rgbSE = textureSample(fxaa_texture, fxaa_sampler, uv + vec2( 1.0,  1.0) * pixel).rgb;
    let rgbM  = textureSample(fxaa_texture, fxaa_sampler, uv).rgb;

    let lumaNW = luma(rgbNW);
    let lumaNE = luma(rgbNE);
    let lumaSW = luma(rgbSW);
    let lumaSE = luma(rgbSE);
    let lumaM  = luma(rgbM);

    let lumaMin = min(lumaM, min(min(lumaNW, lumaNE), min(lumaSW, lumaSE)));
    let lumaMax = max(lumaM, max(max(lumaNW, lumaNE), max(lumaSW, lumaSE)));

    var dir = vec2(
        -((lumaNW + lumaNE) - (lumaSW + lumaSE)),
         ((lumaNW + lumaSW) - (lumaNE + lumaSE))
    );

    let reduce_min: f32 = 1.0 / 128.0;
    let reduce_mul: f32 = 1.0 / 8.0;
    let span_max:  f32 = 8.0;

    let dir_reduce = max((lumaNW + lumaNE + lumaSW + lumaSE) * (0.25 * reduce_mul), reduce_min);
    let rcp_dir_min = 1.0 / (min(abs(dir.x), abs(dir.y)) + dir_reduce);

    dir = clamp(
        dir * rcp_dir_min,
        vec2(-span_max),
        vec2(span_max)
    ) * pixel;

    let rgbA = 0.5 * (
        textureSample(fxaa_texture, fxaa_sampler, uv + dir * (1.0 / 3.0 - 0.5)).rgb +
        textureSample(fxaa_texture, fxaa_sampler, uv + dir * (2.0 / 3.0 - 0.5)).rgb
    );

    let rgbB = rgbA * 0.5 + 0.25 * (
        textureSample(fxaa_texture, fxaa_sampler, uv + dir * -0.5).rgb +
        textureSample(fxaa_texture, fxaa_sampler, uv + dir * 0.5).rgb
    );

    let lumaB = luma(rgbB);

    var final_rgb: vec3<f32>;

    if (lumaB < lumaMin || lumaB > lumaMax) {
        final_rgb = rgbA;
    } else {
        final_rgb = rgbB;
    }

    return vec4(final_rgb, 1.0);
}
