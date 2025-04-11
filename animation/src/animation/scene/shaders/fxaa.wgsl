@group(0) @binding(0) var fxaa_texture: texture_2d<f32>;
@group(0) @binding(1) var fxaa_sampler: sampler;

struct FxaaUniform {
    reciprocal_screen_size: vec2<f32>,
    subpix_shift: f32,
    subpix_trim: f32,
    reduce_min: f32,
    reduce_mul: f32,
};
@group(0) @binding(2) var<uniform> fxaa_uniform: FxaaUniform;

struct VertexOutput {
    @builtin(position) pos: vec4<f32>,
    @location(0) uv: vec2<f32>,
};

fn luma(color: vec3<f32>) -> f32 {
    return dot(color, vec3<f32>(0.299, 0.587, 0.114));
}

@vertex
fn vs_main(@builtin(vertex_index) vertex_index: u32) -> VertexOutput {
    var positions = array<vec2<f32>, 3>(
        vec2<f32>(-1.0, -1.0),
        vec2<f32>( 3.0, -1.0),
        vec2<f32>(-1.0,  3.0)
    );

    var uvs = array<vec2<f32>, 3>(
        vec2<f32>(0.0, 0.0),
        vec2<f32>(2.0, 0.0),
        vec2<f32>(0.0, 2.0)
    );

    var out: VertexOutput;
    out.pos = vec4<f32>(positions[vertex_index], 0.0, 1.0);
    out.uv = uvs[vertex_index];
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    // Unpack uniform parameters.
    let texel = fxaa_uniform.reciprocal_screen_size;
    let subpix_shift = fxaa_uniform.subpix_shift;
    let subpix_trim  = fxaa_uniform.subpix_trim;
    let reduce_min   = fxaa_uniform.reduce_min;
    let reduce_mul   = fxaa_uniform.reduce_mul;

    let uv = in.uv;

    // Center and cardinal samples.
    let rgbM = textureSample(fxaa_texture, fxaa_sampler, uv).rgb;
    let lumaM = luma(rgbM);
    let lumaN = luma(textureSample(fxaa_texture, fxaa_sampler, uv + vec2<f32>(0.0, -texel.y)).rgb);
    let lumaS = luma(textureSample(fxaa_texture, fxaa_sampler, uv + vec2<f32>(0.0,  texel.y)).rgb);
    let lumaW = luma(textureSample(fxaa_texture, fxaa_sampler, uv + vec2<f32>(-texel.x, 0.0)).rgb);
    let lumaE = luma(textureSample(fxaa_texture, fxaa_sampler, uv + vec2<f32>( texel.x, 0.0)).rgb);

    // Diagonal samples.
    let lumaNW = luma(textureSample(fxaa_texture, fxaa_sampler, uv + vec2<f32>(-texel.x, -texel.y)).rgb);
    let lumaNE = luma(textureSample(fxaa_texture, fxaa_sampler, uv + vec2<f32>( texel.x, -texel.y)).rgb);
    let lumaSW = luma(textureSample(fxaa_texture, fxaa_sampler, uv + vec2<f32>(-texel.x,  texel.y)).rgb);
    let lumaSE = luma(textureSample(fxaa_texture, fxaa_sampler, uv + vec2<f32>( texel.x,  texel.y)).rgb);

    // Determine luma range.
    let lumaMin = min(lumaM, min(min(lumaN, lumaS), min(lumaW, lumaE)));
    let lumaMax = max(lumaM, max(max(lumaN, lumaS), max(lumaW, lumaE)));
    let range = lumaMax - lumaMin;

    // If the contrast is low, bypass FXAA.
    if (range < max(reduce_min, lumaMax * reduce_mul)) {
        return vec4<f32>(rgbM, 1.0);
    }

    // Average of the diagonal samples.
    let lumaCross = (lumaNW + lumaNE + lumaSW + lumaSE);

    // Determine edge orientation using the absolute difference of horizontal vs. vertical gradients.
    let gradientHorizontal = abs(lumaW - lumaE);
    let gradientVertical   = abs(lumaN - lumaS);
    let isHorizontal = gradientHorizontal >= gradientVertical;

    // Calculate gradients from the center.
    let gradientN = abs(lumaN - lumaM);
    let gradientS = abs(lumaS - lumaM);
    let gradientE = abs(lumaE - lumaM);
    let gradientW = abs(lumaW - lumaM);

    let pixelOffset = -0.5 * texel;
    var offset = vec2<f32>(0.0, 0.0);

    if (isHorizontal) {
        if (gradientW < gradientE) {
            offset.x = -1.0;
        } else {
            offset.x = 1.0;
        }
    } else {
        if (gradientN < gradientS) {
            offset.y = -1.0;
        } else {
            offset.y = 1.0;
        }
    }

    // Compute a sub-pixel offset based on the average of the diagonal samples.
    let subpix = clamp((lumaCross * 0.25 - lumaM) * subpix_shift, -subpix_trim, subpix_trim);

    // Adjust the offset.
    if (isHorizontal) {
        offset.x = offset.x * texel.x;
        offset.y = subpix;
    } else {
        offset.x = subpix;
        offset.y = offset.y * texel.y;
    }

    // Sample two sets of pixels along the edge.
    let rgbA = 0.5 * (
        textureSample(fxaa_texture, fxaa_sampler, uv + pixelOffset + offset).rgb +
        textureSample(fxaa_texture, fxaa_sampler, uv - pixelOffset + offset).rgb
    );

    let rgbB = rgbA * 0.5 + 0.25 * (
        textureSample(fxaa_texture, fxaa_sampler, uv + offset).rgb +
        textureSample(fxaa_texture, fxaa_sampler, uv - offset).rgb
    );

    let lumaB = luma(rgbB);

    // Choose the final color based on whether rgbB is within the local luma range.
    var final_rgb: vec3<f32>;
    if (lumaB < lumaMin || lumaB > lumaMax) {
        final_rgb = rgbA;
    } else {
        final_rgb = rgbB;
    }

    return vec4<f32>(final_rgb, 1.0);
}
