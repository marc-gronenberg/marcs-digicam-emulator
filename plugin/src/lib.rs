//! Marcs Digicam Emulator: the look of 2000s CCD compact cameras for PhotoCraft (plug-in ABI v1).
//!
//! Per band (with a 48 px overlap, for the spatial stages):
//!
//! 1. the document to linear sRGB (Input Space), the camera's exposure compensation;
//! 2. sensor noise in the raw channels (shot noise from the pixel's full well, read noise, both
//!    growing with ISO), carried into the scene through the camera's white balance and colour
//!    matrix (`raw_to_scene`), so a strong matrix (CMYG sensors) makes coloured noise;
//! 3. the camera's LUT (`model/digicam_model.py`): sensor, clip at full well, white balance,
//!    colour matrix, tone curve and the maker's style, sRGB-encoded out;
//! 4. CCD blooming: charge from clipped pixels spilling up and down the column (a vertical
//!    streak, its length growing with how far the highlight is blown);
//! 5. purple fringing around clipped highlights;
//! 6. the camera's noise reduction (chroma blurred, luma softened at high ISO) and sharpening
//!    (unsharp mask on luma, with its halos), both on the encoded image as the cameras did;
//! 7. Strength mixes the result with the original in linear light.
//!
//! `no_std`, no imports, no allocator.
#![no_std]

use digicam_common::blur::blur;
use digicam_common::{Aligned, ERR_ARGS, ERR_MEMORY, ERR_MODE, HAS_ALPHA, TN, alloc, clamp01, json_num, json_str, lerp_tab};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    core::arch::wasm32::unreachable()
}

const MANIFEST: &str = r#"{
  "id": "io.marcs.digicam",
  "name": "Marcs Digicam Emulator…",
  "version": "0.1.0",
  "kind": "filter",
  "author": "Marc Gronenberg",
  "description": "The look of 2000s CCD compact cameras: sensor colour, hard digital highlights, sensor noise by ISO, noise reduction, oversharpening, blooming and purple fringing. RGB and Grayscale.",
  "params": {
    "camera":     {"type": "choice", "options": ["Pocket 2002", "Prosumer 2003", "Compact 2004", "Compact 2005", "Compact 2006", "Compact 2007", "Ultracompact 2008", "Prosumer 2009"], "default": "Compact 2004"},
    "inputSpace": {"type": "choice", "options": ["sRGB", "Display P3", "Adobe RGB"], "default": "sRGB"},
    "exposure":   {"type": "number", "min": -2, "max": 2, "default": 0, "unit": "EV"},
    "iso":        {"type": "choice", "options": ["100", "200", "400", "800", "1600"], "default": "100"},
    "strength":   {"type": "number", "min": 0, "max": 100, "default": 100},
    "noise":      {"type": "number", "min": 0, "max": 200, "default": 100},
    "noiseReduction": {"type": "number", "min": 0, "max": 200, "default": 100},
    "sharpening": {"type": "number", "min": 0, "max": 200, "default": 100},
    "blooming":   {"type": "number", "min": 0, "max": 200, "default": 100},
    "purpleFringing": {"type": "number", "min": 0, "max": 200, "default": 100},
    "seed":       {"type": "int", "min": 0, "max": 9999, "default": 1}
  },
  "overlap": 48,
  "area": "content"
}"#;

const OVERLAP: usize = 48;

#[unsafe(no_mangle)]
pub extern "C" fn pc_abi_version() -> u32 {
    1
}

#[unsafe(no_mangle)]
pub extern "C" fn pc_manifest() -> u64 {
    digicam_common::pack_manifest(MANIFEST)
}

#[unsafe(no_mangle)]
pub extern "C" fn pc_alloc(size: u32) -> u32 {
    alloc(size)
}

// ----------------------------------------------------------------------------- static data

const N: usize = 37;
const STOPS_LO: f32 = -8.0;
const STOPS_HI: f32 = 5.0;
const INV_SPAN: f32 = 1.0 / (STOPS_HI - STOPS_LO);
const LUT_LEN: usize = N * N * N * 3;
const LOG2_018: f32 = -2.473_931_2;
/// `log2(linear) * INV_SPAN + LOG_OFF` is the shaper coordinate.
const LOG_OFF: f32 = -(LOG2_018 + STOPS_LO) * INV_SPAN;

/// A camera's LUT: `u16` (little-endian), `[b][g][r][rgb]`, sRGB-encoded output.
type Lut = Aligned<[u8; LUT_LEN * 2]>;
macro_rules! include_lut {
    ($key:literal) => {
        &Aligned(*include_bytes!(concat!("../../lut/", $key, ".lut")))
    };
}

/// What the plug-in needs per camera besides its LUT (generated, see `cameras.rs`).
pub struct Camera {
    name: &'static str,
    lut: &'static Lut,
    /// raw colour channels: 3 (RGB) or 4 (Ye, Cy, Mg, G)
    channels: usize,
    /// linear scene (grey 0.18) -> raw signal as a fraction of full well
    scene_to_raw: [[f32; 3]; 4],
    /// raw noise -> linear scene, through white balance and colour matrix
    raw_to_scene: [[f32; 4]; 3],
    /// electrons at base ISO
    full_well: f32,
    read_noise: f32,
    sharpen: f32,
    sharpen_radius: f32,
    nr: f32,
    bloom: f32,
    fringe: f32,
}

include!("cameras.rs");

/// `u8base[256] | odec[TN+1] | enc[TN+1] | log2m[257] | u16base[65536]` as `f32` (make_tables.py).
const TABLES_LEN: usize = 256 + 2 * (TN + 1) + 257 + 65536;
static TABLE_BYTES: &Aligned<[u8; TABLES_LEN * 4]> = &Aligned(*include_bytes!("../../lut/tables.bin"));
const ODEC: usize = 256;
const ENC: usize = 256 + (TN + 1);
const LOG2M: usize = 256 + 2 * (TN + 1);
/// `adec[TN+1] | aenc[TN+1]`: Adobe RGB (1998) transfer curve (gamma 563/256) and its inverse.
static ADOBE_BYTES: &Aligned<[u8; 2 * (TN + 1) * 4]> = &Aligned(*include_bytes!("../../lut/adobe.bin"));
/// Linear Display P3 / Adobe RGB (D65) -> linear sRGB and back (rows sum to 1: white stays white).
const P3_IN: [f32; 9] = [1.2249402, -0.2249402, 0.0, -0.0420571, 1.0420571, 0.0, -0.0196375, -0.0786360, 1.0982736];
const P3_OUT: [f32; 9] = [0.8224620, 0.1775380, 0.0, 0.0331943, 0.9668057, 0.0, 0.0170826, 0.0723974, 0.9105200];
const ADOBE_IN: [f32; 9] = [1.3983558, -0.3983558, 0.0, 0.0, 1.0, 0.0, 0.0, -0.0429290, 1.0429290];
const ADOBE_OUT: [f32; 9] = [0.7151256, 0.2848744, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0411619, 0.9588381];
/// White Gaussian noise, `[y][x][4]` signed bytes, unit spread = 32.
const NT: usize = 1024;
static NOISE_BYTES: &Aligned<[u8; NT * NT * 4]> = &Aligned(*include_bytes!("../../lut/noise1024.bin"));

#[inline(always)]
fn tables() -> *const f32 {
    TABLE_BYTES.0.as_ptr() as *const f32
}

// ----------------------------------------------------------------------------- LUT lookup

const DR: usize = 3;
const DG: usize = N * 3;
const DB: usize = N * N * 3;

/// Tetrahedral interpolation in the cell at `p0` with fractions (fr, fg, fb); sRGB-encoded rgb.
///
/// # Safety
/// `p0` is a cell's first corner, so that all 8 corners lie inside the LUT.
#[inline(always)]
unsafe fn tetra(p0: *const u16, fr: f32, fg: f32, fb: f32) -> (f32, f32, f32) {
    let (a, b, w1, w2, w3) = if fr > fg {
        if fg > fb {
            (DR, DR + DG, fr - fg, fg - fb, fb)
        } else if fr > fb {
            (DR, DR + DB, fr - fb, fb - fg, fg)
        } else {
            (DB, DR + DB, fb - fr, fr - fg, fg)
        }
    } else if fb > fg {
        (DB, DG + DB, fb - fg, fg - fr, fr)
    } else if fb > fr {
        (DG, DG + DB, fg - fb, fb - fr, fr)
    } else {
        (DG, DR + DG, fg - fr, fr - fb, fb)
    };
    // the u16 scale folded into the weights
    const S: f32 = 1.0 / 65535.0;
    let (w1, w2, w3) = (w1 * S, w2 * S, w3 * S);
    let w0 = S - (w1 + w2 + w3);
    unsafe {
        let (pa, pb, pc) = (p0.add(a), p0.add(b), p0.add(DR + DG + DB));
        let c = |k: usize| w0 * *p0.add(k) as f32 + w1 * *pa.add(k) as f32 + w2 * *pb.add(k) as f32 + w3 * *pc.add(k) as f32;
        (c(0), c(1), c(2))
    }
}

/// The LUT cell and fraction along one axis for a shaper coordinate (clamped to 0..1).
#[inline(always)]
fn cell(u: f32) -> (usize, f32) {
    const S: f32 = (N - 1) as f32;
    let p = clamp01(u) * S;
    let i = p as usize;
    let i = if i < N - 1 { i } else { N - 2 };
    (i, p - i as f32)
}

/// log2(v) for v > 0 from the float's exponent and a 257-entry mantissa table (|error| < 2e-6).
#[inline(always)]
fn log2_fast(v: f32) -> f32 {
    let v = if v > 1e-30 { v } else { 1e-30 };
    let bits = v.to_bits();
    let e = ((bits >> 23) & 0xff) as i32 - 127;
    let m = bits & 0x007f_ffff;
    let i = (m >> 15) as usize; // top 8 mantissa bits
    let f = (m & 0x7fff) as f32 * (1.0 / 32768.0);
    unsafe {
        let t = tables().add(LOG2M);
        let a = *t.add(i);
        e as f32 + a + (*t.add(i + 1) - a) * f
    }
}

/// The camera's rendering of a linear scene colour (grey 0.18), sRGB-encoded.
#[inline(always)]
fn render(lut: *const u16, r: f32, g: f32, b: f32) -> (f32, f32, f32) {
    let u = |v: f32| log2_fast(v) * INV_SPAN + LOG_OFF;
    let ((ir, fr), (ig, fg), (ib, fb)) = (cell(u(r)), cell(u(g)), cell(u(b)));
    // SAFETY: ir, ig, ib <= N-2, so every corner index stays below LUT_LEN.
    unsafe { tetra(lut.add(ib * DB + ig * DG + ir * DR), fr, fg, fb) }
}

#[inline(always)]
fn mat(m: &[f32; 9], r: f32, g: f32, b: f32) -> (f32, f32, f32) {
    (m[0] * r + m[1] * g + m[2] * b, m[3] * r + m[4] * g + m[5] * b, m[6] * r + m[7] * g + m[8] * b)
}

/// sqrt(x) for x > 0 from the reciprocal square root bit trick and one Newton step (error under
/// 0.2 %, plenty for noise amplitudes; core has no stable square root in `no_std`).
#[inline(always)]
fn sqrt(x: f32) -> f32 {
    if x <= 0.0 {
        return 0.0;
    }
    let y = f32::from_bits(0x5f37_59df - (x.to_bits() >> 1));
    x * y * (1.5 - 0.5 * x * y * y)
}

#[inline(always)]
fn exp2(x: f32) -> f32 {
    // 2^x for the exposure slider (|x| <= 2): integer part by exponent, fraction by a cubic
    let i = if x < 0.0 { x as i32 - 1 } else { x as i32 };
    let f = x - i as f32;
    let p = 1.0 + f * (0.695_976_1 + f * (0.226_131_1 + f * 0.078_024_5));
    f32::from_bits(((i + 127) as u32) << 23) * p
}

// ----------------------------------------------------------------------------- the camera stage

/// How the document's pixels become linear sRGB.
struct Doc {
    linear: bool,
    grey: bool,
    ident: bool,
    n: usize,
    dec: *const f32,
    m_in: [f32; 9],
    /// exposure compensation, as a factor
    ev: f32,
}

/// The sensor and its noise, for one band.
struct Sensor {
    lut: *const u16,
    s: [[f32; 3]; 4],
    a: [[f32; 4]; 3],
    /// shot noise variance per unit of signal (full well shrinking with ISO)
    inv_fw: f32,
    read_var: f32,
    /// noise amount / tile spread
    amp: f32,
    noise: *const i8,
    ox: usize,
    oy: usize,
    x0: i32,
    y0: i32,
}

/// Steps 1-3 for every pixel of the band: the scene, its raw signal (and how far the most blown
/// channel is over full well, in stops, into `blown`), the sensor noise carried into the scene,
/// the camera's LUT into `pr`, `pg`, `pb`. `K` raw channels, `NOISE` on or off. Returns whether
/// any pixel is blown.
///
/// # Safety
/// `buf` holds `w × h × doc.n` samples, the planes `w × h` floats each.
#[inline(always)]
unsafe fn camera<const K: usize, const NOISE: bool>(buf: *mut f32, w: usize, h: usize, doc: &Doc, sen: &Sensor, planes: (*mut f32, *mut f32, *mut f32, *mut f32)) -> bool {
    let (pr, pg, pb, blown) = planes;
    let s = &sen.s;
    let a = &sen.a;
    let mut any = false;
    for y in 0..h {
        // the noise tile row for this document row (negative rows of the edge overlap wrap)
        let row = unsafe { sen.noise.add((((sen.y0 + y as i32) as usize).wrapping_add(sen.oy) % NT) * NT * 4) };
        let mut nx = ((sen.x0 as usize).wrapping_add(sen.ox)) % NT;
        for x in 0..w {
            let i = y * w + x;
            unsafe {
                let p = buf.add(i * doc.n);
                let lin = |v: f32| if doc.linear { if v > 0.0 { v } else { 0.0 } } else { lerp_tab(doc.dec, v) };
                let (dr, dg, db) = if doc.grey { let l = lin(*p); (l, l, l) } else { (lin(*p), lin(*p.add(1)), lin(*p.add(2))) };
                let (r, g, b) = if doc.ident { (dr, dg, db) } else { mat(&doc.m_in, dr, dg, db) };
                let (mut r, mut g, mut b) = (if r > 0.0 { r * doc.ev } else { 0.0 }, if g > 0.0 { g * doc.ev } else { 0.0 }, if b > 0.0 { b * doc.ev } else { 0.0 });
                let raw = |k: usize| s[k][0] * r + s[k][1] * g + s[k][2] * b;
                let (r0, r1, r2) = (raw(0), raw(1), raw(2));
                let r3 = if K == 4 { raw(3) } else { 0.0 };
                let mut top = if r0 > r1 { r0 } else { r1 };
                top = if r2 > top { r2 } else { top };
                if K == 4 && r3 > top {
                    top = r3;
                }
                *blown.add(i) = if top > 1.0 {
                    any = true;
                    log2_fast(top)
                } else {
                    0.0
                };
                if NOISE {
                    let q = row.add(nx * 4);
                    let sd = |v: f32, k: usize| {
                        let sig = if v > 0.0 { if v < 1.0 { v } else { 1.0 } } else { 0.0 };
                        *q.add(k) as f32 * sqrt(sig * sen.inv_fw + sen.read_var) * sen.amp
                    };
                    let (d0, d1, d2) = (sd(r0, 0), sd(r1, 1), sd(r2, 2));
                    let d3 = if K == 4 { sd(r3, 3) } else { 0.0 };
                    r += a[0][0] * d0 + a[0][1] * d1 + a[0][2] * d2 + if K == 4 { a[0][3] * d3 } else { 0.0 };
                    g += a[1][0] * d0 + a[1][1] * d1 + a[1][2] * d2 + if K == 4 { a[1][3] * d3 } else { 0.0 };
                    b += a[2][0] * d0 + a[2][1] * d1 + a[2][2] * d2 + if K == 4 { a[2][3] * d3 } else { 0.0 };
                    nx = if nx + 1 < NT { nx + 1 } else { 0 };
                }
                let (or, og, ob) = render(sen.lut, r, g, b);
                *pr.add(i) = or;
                *pg.add(i) = og;
                *pb.add(i) = ob;
            }
        }
    }
    any
}

// ----------------------------------------------------------------------------- the filter

/// # Safety
/// The host passes `buf..buf+len` and `params..params+params_len` from `pc_alloc`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn pc_filter(buf: *mut f32, len: u32, width: u32, height: u32, channels: u32, format: u32, params: *const u8, params_len: u32) -> i32 {
    let (w, h, n) = (width as usize, height as usize, channels as usize);
    if n == 0 || w == 0 || h == 0 || (len as usize) < w * h * n * 4 {
        return ERR_ARGS;
    }
    let alpha = format & HAS_ALPHA != 0;
    let colour = if alpha { n - 1 } else { n };
    let depth = format & 0xff;
    let grey = colour == 1;
    if !(grey || (colour == 3 && (format >> 16) & 0xff == 3)) {
        return ERR_MODE;
    }
    // SAFETY: see the function's contract.
    let json = unsafe { core::slice::from_raw_parts(params, params_len as usize) };
    let g = |k: &str, d: f32| json_num(json, k).unwrap_or(d);
    let cam = json_str(json, "camera").and_then(|v| CAMERAS.iter().find(|c| c.name.as_bytes() == v)).unwrap_or(&CAMERAS[2]);
    let lut = cam.lut.0.as_ptr() as *const u16;
    let iso_gain = match json_str(json, "iso") {
        Some(b"200") => 2.0,
        Some(b"400") => 4.0,
        Some(b"800") => 8.0,
        Some(b"1600") => 16.0,
        _ => 1.0,
    };
    let ev = exp2(g("exposure", 0.0).clamp(-2.0, 2.0));
    let strength = clamp01(g("strength", 100.0) / 100.0);
    let noise_amt = g("noise", 100.0).clamp(0.0, 200.0) / 100.0;
    let nr_amt = g("noiseReduction", 100.0).clamp(0.0, 200.0) / 100.0 * cam.nr * sqrt(iso_gain);
    let sharpen_amt = g("sharpening", 100.0).clamp(0.0, 200.0) / 100.0 * cam.sharpen;
    let bloom_amt = g("blooming", 100.0).clamp(0.0, 200.0) / 100.0 * cam.bloom;
    let fringe_amt = g("purpleFringing", 100.0).clamp(0.0, 200.0) / 100.0 * cam.fringe;
    let seed = g("seed", 1.0) as u32;
    let mix = |s: u32| -> u32 {
        let mut h = s.wrapping_mul(0x9e37_79b9) ^ 0x85eb_ca6b;
        h ^= h >> 13;
        h = h.wrapping_mul(0xc2b2_ae35);
        h ^ (h >> 16)
    };
    let (ox, oy) = (mix(seed) as usize % NT, mix(seed ^ 0x5bd1_e995) as usize % NT);
    let (x0, y0) = (g("x", 0.0) as i32, g("y", 0.0) as i32);
    let ov = (g("overlap", OVERLAP as f32) as usize).min(w / 2).min(h / 2);

    // the document's transfer curve and primaries
    let space = if grey { &b"sRGB"[..] } else { json_str(json, "inputSpace").unwrap_or(b"sRGB") };
    let (ident, m_in, m_out) = match space {
        b"Display P3" => (false, P3_IN, P3_OUT),
        b"Adobe RGB" => (false, ADOBE_IN, ADOBE_OUT),
        _ => (true, [0.0; 9], [0.0; 9]),
    };
    let t = tables();
    let a = ADOBE_BYTES.0.as_ptr() as *const f32;
    // SAFETY: offsets inside the static tables
    let (odec, enc) = unsafe { (t.add(ODEC), t.add(ENC)) };
    let (dec, denc) = unsafe { if space == b"Adobe RGB" { (a, a.add(TN + 1)) } else { (odec, enc) } };
    let linear_doc = depth == 32;

    // planes: the camera's output r, g, b (sRGB-encoded), the blown-highlight map, two scratch
    let area = w * h;
    let mem = alloc((area * 4 * 6) as u32) as usize;
    if mem == 0 {
        return ERR_MEMORY;
    }
    let plane = |i: usize| (mem + area * 4 * i) as *mut f32;
    let (pr, pg, pb, blown, s1, s2) = (plane(0), plane(1), plane(2), plane(3), plane(4), plane(5));

    // ---------------------------------------------------------------- 1-3: noise and the camera
    let doc = Doc { linear: linear_doc, grey, ident, n, dec, m_in, ev };
    let sensor = Sensor {
        lut,
        s: cam.scene_to_raw,
        a: cam.raw_to_scene,
        inv_fw: iso_gain / cam.full_well,
        read_var: {
            // read noise in electrons: a part before the amplifier (stays) and one after it
            // (shrinks with the gain); `read_noise` is roughly their total at base ISO
            let (pre, post) = (cam.read_noise * 0.25, cam.read_noise / iso_gain);
            let sigma = sqrt(pre * pre + post * post) * iso_gain / cam.full_well;
            sigma * sigma
        },
        amp: noise_amt * (1.0 / 32.0),
        noise: NOISE_BYTES.0.as_ptr() as *const i8,
        ox,
        oy,
        x0,
        y0,
    };
    let planes = (pr, pg, pb, blown);
    // SAFETY: buf holds w × h × n samples, the planes w × h floats each
    let any_blown = unsafe {
        match (cam.channels, noise_amt > 0.0) {
            (4, true) => camera::<4, true>(buf, w, h, &doc, &sensor, planes),
            (4, false) => camera::<4, false>(buf, w, h, &doc, &sensor, planes),
            (_, true) => camera::<3, true>(buf, w, h, &doc, &sensor, planes),
            (_, false) => camera::<3, false>(buf, w, h, &doc, &sensor, planes),
        }
    };

    unsafe {
        // ---------------------------------------------------------------- 4: blooming
        // charge spilling along the column, both ways, falling off over `reach` pixels; reach
        // and amount are kept inside the overlap so bands meet without seams
        if bloom_amt > 0.0 && any_blown {
            // reach <= 10 px: falls below 1 % within the 48 px overlap
            let reach = 4.0 + 4.0 * bloom_amt;
            let reach = if reach > 10.0 { 10.0 } else { reach };
            let keep = 1.0 - 1.0 / reach;
            for x in 0..w {
                let mut acc = 0.0f32;
                for y in 0..h {
                    acc = acc * keep + *blown.add(y * w + x);
                    *s1.add(y * w + x) = acc;
                }
                acc = 0.0;
                for y in (0..h).rev() {
                    let e = *blown.add(y * w + x);
                    acc = acc * keep + e;
                    let streak = (*s1.add(y * w + x) + acc - e) * (0.08 * bloom_amt);
                    let s = if streak < 1.0 { streak } else { 1.0 };
                    let i = y * w + x;
                    *pr.add(i) += (1.0 - *pr.add(i)) * s;
                    *pg.add(i) += (1.0 - *pg.add(i)) * s;
                    *pb.add(i) += (1.0 - *pb.add(i)) * s;
                }
            }
        }

        // ---------------------------------------------------------------- 5: purple fringing
        if fringe_amt > 0.0 && any_blown {
            for i in 0..area {
                let e = *blown.add(i);
                *s1.add(i) = if e > 1.0 { 1.0 } else { e };
            }
            blur(s1, s2, w, h, 2);
            for i in 0..area {
                let e = *blown.add(i);
                let edge = *s1.add(i) - if e > 1.0 { 1.0 } else { e }; // the dark side of the edge
                if edge > 0.0 {
                    let f = edge * fringe_amt * 0.9;
                    *pr.add(i) += f * 0.45;
                    *pg.add(i) -= f * 0.08;
                    *pb.add(i) += f * 0.85;
                }
            }
        }

        // ---------------------------------------------------------------- 6: noise reduction, sharpening
        // to Y'CbCr (BT.601 on the encoded values): pr <- Y, pg <- Cb, pb <- Cr
        for i in 0..area {
            let (r, gg, b) = (*pr.add(i), *pg.add(i), *pb.add(i));
            let y = 0.299 * r + 0.587 * gg + 0.114 * b;
            *pr.add(i) = y;
            *pg.add(i) = b - y;
            *pb.add(i) = r - y;
        }
        // chroma blurred (the cameras' main weapon against colour noise)
        let rc = (nr_amt * 2.5 + 0.5) as usize;
        let rc = if rc > 12 { 12 } else { rc };
        blur(pg, s2, w, h, rc);
        blur(pb, s2, w, h, rc);
        // luma, against one blur of itself: softened towards it at higher noise reduction (the
        // "watercolour"), then sharpened away from it (unsharp mask, with its halos)
        let wl = (nr_amt - 0.6) * 0.35;
        let wl = if wl < 0.0 { 0.0 } else if wl > 0.75 { 0.75 } else { wl };
        if wl > 0.0 || sharpen_amt > 0.0 {
            for i in 0..area {
                *s1.add(i) = *pr.add(i);
            }
            let rs = (cam.sharpen_radius + 0.5) as usize;
            blur(s1, s2, w, h, if rs < 1 { 1 } else { rs });
            let k = sharpen_amt * 1.6;
            for i in 0..area {
                let b = *s1.add(i);
                let y = *pr.add(i);
                let y = y + (b - y) * wl;
                let d = y - b;
                // a small threshold keeps flat areas from sharpening their own noise as much
                let d = if d > 0.004 { d - 0.004 } else if d < -0.004 { d + 0.004 } else { 0.0 };
                *pr.add(i) = y + d * k;
            }
        }

        // ---------------------------------------------------------------- 7: back, Strength, write
        for y in ov..h - ov {
            for x in ov..w - ov {
                let i = y * w + x;
                let p = buf.add(i * n);
                if alpha && *p.add(n - 1) <= 0.0 {
                    continue;
                }
                let (yy, cb, cr) = (*pr.add(i), *pg.add(i), *pb.add(i));
                let er = yy + cr;
                let eb = yy + cb;
                let eg = (yy - 0.299 * er - 0.114 * eb) / 0.587;
                // camera output (sRGB-encoded) -> linear sRGB
                let (cr_, cg_, cb_) = (lerp_tab(odec, er), lerp_tab(odec, eg), lerp_tab(odec, eb));
                let lin = |v: f32| if linear_doc { if v > 0.0 { v } else { 0.0 } } else { lerp_tab(dec, v) };
                if grey {
                    let o = lin(*p);
                    let c = 0.2126 * cr_ + 0.7152 * cg_ + 0.0722 * cb_;
                    let v = o + (c - o) * strength;
                    *p = if linear_doc { v } else { lerp_tab(denc, v) };
                } else {
                    let (or, og, ob) = (lin(*p), lin(*p.add(1)), lin(*p.add(2)));
                    // the camera's output is sRGB; into the document's primaries
                    let (tr, tg, tb) = if ident { (cr_, cg_, cb_) } else { mat(&m_out, cr_, cg_, cb_) };
                    let (vr, vg, vb) = (or + (tr - or) * strength, og + (tg - og) * strength, ob + (tb - ob) * strength);
                    if linear_doc {
                        *p = vr;
                        *p.add(1) = vg;
                        *p.add(2) = vb;
                    } else {
                        *p = lerp_tab(denc, vr);
                        *p.add(1) = lerp_tab(denc, vg);
                        *p.add(2) = lerp_tab(denc, vb);
                    }
                }
            }
        }
    }
    0
}
