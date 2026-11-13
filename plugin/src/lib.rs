//! Marcs Digicam Emulator: the look of 2000s CCD compact cameras for PhotoCraft (plug-in ABI v1).
//!
//! Per band (with a 16 px overlap, for the spatial stages):
//!
//! 0. Resolution "Camera" (the default): the band is rendered at about the camera's megapixels,
//!    each camera pixel the mean, in linear light, of an f × f block of the document grid (f a
//!    whole number from the document's and the camera's size, 1-4), and scaled back up at the
//!    end; the noise grain is then the camera's pixel, fine detail goes as with a real 5 MP file,
//!    and it runs about f² times faster. "Document" renders every document pixel;
//! 1. the document to linear sRGB (Input Space), the camera's exposure compensation;
//! 2. sensor noise in the raw channels (shot noise from the pixel's full well, read noise, both
//!    growing with ISO), carried into the scene through the camera's white balance and colour
//!    matrix (`raw_to_scene`), so a strong matrix (CMYG sensors) makes coloured noise;
//! 3. the camera's LUT (`model/digicam_model.py`): sensor, clip at full well, white balance,
//!    colour matrix, tone curve and the maker's style, sRGB-encoded out, kept as Y'CbCr;
//! 4. CCD blooming: charge from clipped pixels spilling up and down the column (a vertical
//!    streak, its length growing with how far the highlight is blown), only in columns with one;
//! 5. purple fringing around clipped highlights, only around them;
//! 6. the camera's noise reduction (chroma blurred, luma softened at high ISO) and sharpening
//!    (unsharp mask on luma, with its halos), both on the encoded image as the cameras did;
//! 7. Strength mixes the result with the original in linear light.
//!
//! `no_std`, no imports, no allocator.
#![no_std]

use digicam_common::blur::{blur, blur_rect, blur1};
use digicam_common::{Aligned, ERR_ARGS, ERR_MEMORY, ERR_MODE, HAS_ALPHA, TN, alloc, clamp01, json_num, json_str, lerp_tab};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    core::arch::wasm32::unreachable()
}

const MANIFEST: &str = r#"{
  "id": "io.marcs.digicam",
  "name": "Marcs Digicam Emulator…",
  "version": "0.3.0",
  "kind": "filter",
  "author": "Marc Gronenberg",
  "description": "The look of 2000s CCD compact cameras: sensor colour, hard digital highlights, sensor noise by ISO, noise reduction, oversharpening, blooming and purple fringing, at the camera's resolution. RGB and Grayscale.",
  "params": {
    "camera":     {"type": "choice", "options": ["Pocket 2002", "Prosumer 2003", "Compact 2004", "Compact 2005", "Compact 2006", "Compact 2007", "Ultracompact 2008", "Prosumer 2009"], "default": "Compact 2004"},
    "resolution": {"type": "choice", "options": ["Camera", "Low", "Document"], "default": "Camera"},
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
  "overlap": 16,
  "area": "content"
}"#;

const OVERLAP: usize = 16;

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
    /// the sensor's size (Resolution "Camera")
    megapixels: f32,
    /// electrons at base ISO
    full_well: f32,
    sharpen: f32,
    sharpen_radius: f32,
    nr: f32,
    bloom: f32,
    fringe: f32,
}

include!("cameras.rs");

/// Read noise in electrons at base ISO, the same for every camera (the model has no per-camera value).
const READ_NOISE: f32 = 10.0;

/// `odec[TN+1] | enc[TN+1] | log2m[257]` as `f32` (make_tables.py).
const TABLES_LEN: usize = 2 * (TN + 1) + 257;
static TABLE_BYTES: &Aligned<[u8; TABLES_LEN * 4]> = &Aligned(*include_bytes!("../../lut/tables.bin"));
const ODEC: usize = 0;
const ENC: usize = TN + 1;
const LOG2M: usize = 2 * (TN + 1);
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
/// The LUT is the camera's, as floats 0..1 (`pc_filter` converts it once per band).
///
/// # Safety
/// `p0` is a cell's first corner, so that all 8 corners lie inside the LUT.
#[inline(always)]
unsafe fn tetra(p0: *const f32, fr: f32, fg: f32, fb: f32) -> (f32, f32, f32) {
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
    let w0 = 1.0 - (w1 + w2 + w3);
    unsafe {
        let (pa, pb, pc) = (p0.add(a), p0.add(b), p0.add(DR + DG + DB));
        let c = |k: usize| w0 * *p0.add(k) + w1 * *pa.add(k) + w2 * *pb.add(k) + w3 * *pc.add(k);
        (c(0), c(1), c(2))
    }
}

/// The float exponents the shaper table covers: the LUT's 13 stops around 0.18 lie in 2^-11 .. 2^3.
const SH_E0: i32 = -11;
/// 14 exponents × the top 8 mantissa bits.
const SH_LEN: usize = 14 * 256;

/// The shaper table: entry `k` is the LUT position (0..N-1, unclamped) of the float with exponent
/// `SH_E0 + k / 256` and top mantissa bits `k % 256`, i.e. `log2_fast` and the shaper in one step
/// (`SH_LEN + 1` floats, the last one for the interpolation).
///
/// # Safety
/// `dst` holds `SH_LEN + 1` floats.
unsafe fn build_shaper(dst: *mut f32) {
    const S: f32 = (N - 1) as f32;
    for k in 0..=SH_LEN {
        let e = SH_E0 + (k / 256) as i32;
        // SAFETY: k % 256 < 257 entries of log2m; k <= SH_LEN
        unsafe { *dst.add(k) = ((e as f32 + *tables().add(LOG2M + k % 256)) * INV_SPAN + LOG_OFF) * S };
    }
}

/// The LUT cell and fraction along one axis for a linear value (outside the LUT's range: its edge).
#[inline(always)]
fn cell(sh: *const f32, v: f32) -> (usize, f32) {
    const S: f32 = (N - 1) as f32;
    let v = if v > 1e-30 { v } else { 1e-30 };
    let bits = v.to_bits();
    let k = (bits >> 15) as i32 - ((127 + SH_E0) << 8);
    let p = if k < 0 {
        0.0
    } else if k >= SH_LEN as i32 {
        S
    } else {
        let f = (bits & 0x7fff) as f32 * (1.0 / 32768.0);
        // SAFETY: 0 <= k < SH_LEN
        unsafe {
            let a = *sh.add(k as usize);
            a + (*sh.add(k as usize + 1) - a) * f
        }
    };
    let p = if p > 0.0 { if p < S { p } else { S } } else { 0.0 };
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
fn render(lut: *const f32, sh: *const f32, r: f32, g: f32, b: f32) -> (f32, f32, f32) {
    let ((ir, fr), (ig, fg), (ib, fb)) = (cell(sh, r), cell(sh, g), cell(sh, b));
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
    /// 8-bit documents: the decode per level (exact, and cheaper than `dec`); null otherwise
    dec8: *const f32,
    m_in: [f32; 9],
    /// exposure compensation, as a factor
    ev: f32,
}

impl Doc {
    /// A document sample as linear light (in the document's primaries).
    ///
    /// # Safety
    /// `dec` (and `dec8`, if not null) are the tables `pc_filter` set up.
    #[inline(always)]
    unsafe fn lin(&self, v: f32) -> f32 {
        if self.linear {
            if v > 0.0 { v } else { 0.0 }
        } else if !self.dec8.is_null() {
            unsafe { *self.dec8.add(((v * 255.0 + 0.5) as u32).min(255) as usize) }
        } else {
            unsafe { lerp_tab(self.dec, v) }
        }
    }

    /// Whether the pixel at `p` has a channel at the top of an 8/16-bit document (clipped there).
    ///
    /// # Safety
    /// `p` points at a pixel of the band.
    #[inline(always)]
    unsafe fn clipped(&self, p: *const f32) -> bool {
        unsafe { !self.linear && (*p >= 0.999 || (!self.grey && (*p.add(1) >= 0.999 || *p.add(2) >= 0.999))) }
    }
}

/// The sums of a full `F × F` block of document pixels from `p0` (rows `pitch` samples apart,
/// pixels `n`), and its count of clipped pixels, added in row order like the general loop in
/// `pc_filter`, so the result is the same to the bit; unrolled for the interpreter.
///
/// # Safety
/// `p0` reaches `(F - 1) × pitch + F × n` samples.
#[inline(always)]
unsafe fn block_sum<const F: usize>(p0: *const f32, pitch: usize, n: usize, grey: bool, doc: &Doc) -> (f32, f32, f32, f32) {
    let (mut sr, mut sg, mut sb, mut sc) = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
    for dy in 0..F {
        for dx in 0..F {
            unsafe {
                let p = p0.add(dy * pitch + dx * n);
                sr += *p;
                if !grey {
                    sg += *p.add(1);
                    sb += *p.add(2);
                }
                if doc.clipped(p) {
                    sc += 1.0;
                }
            }
        }
    }
    (sr, sg, sb, sc)
}

/// The sensor and its noise, for one band.
struct Sensor {
    /// the camera's LUT as floats
    lut: *const f32,
    /// the shaper table (`build_shaper`)
    sh: *const f32,
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

/// What the camera stage reads and writes, `w × h` each: (with `LOW`) its input, the camera
/// pixels as linear sRGB and the share of their document pixels clipped in the source; the
/// camera's output as Y'CbCr (BT.601 on the sRGB-encoded values), the blown-highlight map, a flag
/// per column with a blown pixel, and (with `KEEP`) the document's own linear values for the
/// Strength mix.
struct Planes {
    inp: [*mut f32; 4],
    y: *mut f32,
    cb: *mut f32,
    cr: *mut f32,
    blown: *mut f32,
    cols: *mut u8,
    keep: [*mut f32; 3],
}

/// Steps 1-3 for every pixel of the band: the scene, its raw signal (and how far the most blown
/// channel is over full well, in stops, into `blown`), the sensor noise carried into the scene,
/// the camera's LUT, as Y'CbCr. `K` raw channels, `NOISE` on or off, `KEEP` the document's linear
/// values, `LOW` the input from `pl.inp` (camera pixels) instead of the document in `buf`.
/// Returns the bounding box of the blown pixels (x0, x1, y0, y1; inclusive), if any.
///
/// # Safety
/// `buf` holds `w × h × doc.n` samples (unless `LOW`), the planes `w × h` values each.
#[inline(always)]
unsafe fn camera<const K: usize, const NOISE: bool, const KEEP: bool, const LOW: bool>(buf: *mut f32, w: usize, h: usize, doc: &Doc, sen: &Sensor, pl: &Planes) -> Option<(usize, usize, usize, usize)> {
    let s = &sen.s;
    let a = &sen.a;
    let (mut bx0, mut bx1, mut by0, mut by1) = (usize::MAX, 0, usize::MAX, 0);
    for y in 0..h {
        // the noise tile row for this document row (negative rows of the edge overlap wrap)
        let row = unsafe { sen.noise.add((((sen.y0 + y as i32) as usize).wrapping_add(sen.oy) % NT) * NT * 4) };
        let mut nx = ((sen.x0 as usize).wrapping_add(sen.ox)) % NT;
        for x in 0..w {
            let i = y * w + x;
            unsafe {
                // the scene in linear sRGB, and whether the source was clipped here
                let (r, g, b, clipped) = if LOW {
                    (*pl.inp[0].add(i), *pl.inp[1].add(i), *pl.inp[2].add(i), *pl.inp[3].add(i) >= 0.5)
                } else {
                    let p = buf.add(i * doc.n);
                    let (dr, dg, db) = if doc.grey { let l = doc.lin(*p); (l, l, l) } else { (doc.lin(*p), doc.lin(*p.add(1)), doc.lin(*p.add(2))) };
                    if KEEP {
                        *pl.keep[0].add(i) = dr;
                        if !doc.grey {
                            *pl.keep[1].add(i) = dg;
                            *pl.keep[2].add(i) = db;
                        }
                    }
                    let (r, g, b) = if doc.ident { (dr, dg, db) } else { mat(&doc.m_in, dr, dg, db) };
                    (r, g, b, doc.clipped(p))
                };
                let (mut r, mut g, mut b) = (if r > 0.0 { r * doc.ev } else { 0.0 }, if g > 0.0 { g * doc.ev } else { 0.0 }, if b > 0.0 { b * doc.ev } else { 0.0 });
                let raw = |k: usize| s[k][0] * r + s[k][1] * g + s[k][2] * b;
                let (r0, r1, r2) = (raw(0), raw(1), raw(2));
                let r3 = if K == 4 { raw(3) } else { 0.0 };
                let mut top = if r0 > r1 { r0 } else { r1 };
                top = if r2 > top { r2 } else { top };
                if K == 4 && r3 > top {
                    top = r3;
                }
                // an 8/16-bit channel at its top was already clipped in the source (its white sits
                // below full well in every camera): one stop over at Exposure 0, so it blooms and fringes
                if clipped {
                    let t = 2.0 * doc.ev;
                    top = if t > top { t } else { top };
                }
                *pl.blown.add(i) = if top > 1.0 {
                    *pl.cols.add(x) = 1;
                    bx0 = if x < bx0 { x } else { bx0 };
                    bx1 = if x > bx1 { x } else { bx1 };
                    by0 = if y < by0 { y } else { by0 };
                    by1 = y;
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
                let (or, og, ob) = render(sen.lut, sen.sh, r, g, b);
                let yy = 0.299 * or + 0.587 * og + 0.114 * ob;
                *pl.y.add(i) = yy;
                *pl.cb.add(i) = ob - yy;
                *pl.cr.add(i) = or - yy;
            }
        }
    }
    if by0 == usize::MAX { None } else { Some((bx0, bx1, by0, by1)) }
}

/// Chroma at half resolution, the way the cameras' JPEGs stored it (4:2:0) and blurred it: the
/// `w × h` planes `cb` and `cr` are averaged over 2x2 blocks into `hb` and `hr`, blurred with
/// radius `r` there and interpolated back. `tmp` is scratch of a half plane's size, `cols` of
/// `3 × w` words (each column's two half-plane columns and its fraction, worked out once).
/// `px`, `py` are the parities of the planes' first column and row in the image, so the 2x2
/// blocks are the same ones in every band.
///
/// # Safety
/// `cb`, `cr` hold `w × h` floats, `hb`, `hr`, `tmp` at least `(w/2 + 1) × (h/2 + 1)`.
#[allow(clippy::too_many_arguments)]
unsafe fn chroma_half(cb: *mut f32, cr: *mut f32, hb: *mut f32, hr: *mut f32, tmp: *mut f32, cols: *mut u32, w: usize, h: usize, px: usize, py: usize, r: usize) {
    let (hw, hh) = ((w + px).div_ceil(2), (h + py).div_ceil(2));
    // the plane's two pixels of half-plane pixel `i` (repeated at the plane's edges)
    let pair = |i: usize, p: usize, len: usize| {
        let a = (2 * i) as isize - p as isize;
        let lo = if a < 0 { 0 } else { a as usize };
        let hi = if a + 1 < len as isize { (a + 1) as usize } else { len - 1 };
        (if lo < len { lo } else { len - 1 }, hi)
    };
    unsafe {
        for y in 0..hh {
            let (y0, y1) = pair(y, py, h);
            for x in 0..hw {
                let (x0, x1) = pair(x, px, w);
                let (a, b, c, d) = (y0 * w + x0, y0 * w + x1, y1 * w + x0, y1 * w + x1);
                *hb.add(y * hw + x) = (*cb.add(a) + *cb.add(b) + *cb.add(c) + *cb.add(d)) * 0.25;
                *hr.add(y * hw + x) = (*cr.add(a) + *cr.add(b) + *cr.add(c) + *cr.add(d)) * 0.25;
            }
        }
        blur(hb, tmp, hw, hh, r);
        blur(hr, tmp, hw, hh, r);
        // back up by bilinear interpolation between half-plane centres
        let (cix, cjx, ctx) = (cols, cols.add(w), cols.add(2 * w) as *mut f32);
        for x in 0..w {
            let fx = ((x + px) as f32 - 0.5) * 0.5;
            let fx = if fx < 0.0 { 0.0 } else { fx };
            let ix = fx as usize;
            let (ix, jx, tx) = if ix + 1 < hw { (ix, ix + 1, fx - ix as f32) } else { (hw - 1, hw - 1, 0.0) };
            *cix.add(x) = ix as u32;
            *cjx.add(x) = jx as u32;
            *ctx.add(x) = tx;
        }
        for y in 0..h {
            let fy = ((y + py) as f32 - 0.5) * 0.5;
            let fy = if fy < 0.0 { 0.0 } else { fy };
            let iy = fy as usize;
            let (iy, jy, ty) = if iy + 1 < hh { (iy, iy + 1, fy - iy as f32) } else { (hh - 1, hh - 1, 0.0) };
            let (bi, bj, ri, rj) = (hb.add(iy * hw), hb.add(jy * hw), hr.add(iy * hw), hr.add(jy * hw));
            for x in 0..w {
                let (ix, jx, tx) = (*cix.add(x) as usize, *cjx.add(x) as usize, *ctx.add(x));
                let up = |i: *const f32, j: *const f32| {
                    let (a, b) = (*i.add(ix), *i.add(jx));
                    let (cc, d) = (*j.add(ix), *j.add(jx));
                    let top = a + (b - a) * tx;
                    let bot = cc + (d - cc) * tx;
                    top + (bot - top) * ty
                };
                *cb.add(y * w + x) = up(bi, bj);
                *cr.add(y * w + x) = up(ri, rj);
            }
        }
    }
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
    let iso_gain = match json_str(json, "iso") {
        Some(b"200") => 2.0,
        Some(b"400") => 4.0,
        Some(b"800") => 8.0,
        Some(b"1600") => 16.0,
        _ => 1.0,
    };
    let ev = exp2(g("exposure", 0.0).clamp(-2.0, 2.0));
    let strength = clamp01(g("strength", 100.0) / 100.0);
    if strength <= 0.0 {
        return 0; // the original, untouched
    }
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

    // Resolution "Camera": render at about the camera's megapixels, on f × f blocks of the
    // document grid (whole blocks of document coordinates, so every band makes the same ones);
    // "Low" one block size coarser, for speed
    let res = json_str(json, "resolution");
    let f: usize = if res == Some(b"Document") {
        1
    } else {
        let px = g("canvasWidth", 0.0) * g("canvasHeight", 0.0);
        let r = sqrt(px / (cam.megapixels * 1e6));
        let fc = ((r + 0.5) as usize).clamp(1, 4);
        if res == Some(b"Low") { (fc + 1).min(4) } else { fc }
    };
    let fi = f as i32;
    let (bx0, by0) = (x0.div_euclid(fi), y0.div_euclid(fi));
    // the planes' size: the band's camera pixels
    let (pw, ph) = if f == 1 { (w, h) } else { (((x0 + w as i32 - 1).div_euclid(fi) - bx0 + 1) as usize, ((y0 + h as i32 - 1).div_euclid(fi) - by0 + 1) as usize) };

    // planes (pw × ph): the camera's output as Y'CbCr, the blown-highlight map, two scratch; for
    // a Strength below 100 at f = 1 the document's linear values; at f > 1 the camera pixels'
    // input. Then per-column scratch and the tables.
    let mixing = strength < 1.0;
    let keep = mixing && f == 1;
    let area = pw * ph;
    let kn = if !keep { 0 } else if grey { 1 } else { 3 };
    let ln = if f > 1 { 4 } else { 0 };
    // each plane with a little room beyond pw × ph: s1 holds both half-resolution chroma planes,
    // which on tiny planes can be a few values more than one plane
    let ps = area + 2 * (pw + ph) + 8;
    let planes_len = ps * 4 * (6 + kn + ln);
    let tail_len = pw * 12 + w * 40 + 256 * 4 + (SH_LEN + 1) * 4 + LUT_LEN * 4 + pw;
    let mem = alloc((planes_len + tail_len) as u32) as usize;
    if mem == 0 {
        return ERR_MEMORY;
    }
    let plane = |i: usize| (mem + ps * 4 * i) as *mut f32;
    let (py, pcb, pcr, blown, s1, s2) = (plane(0), plane(1), plane(2), plane(3), plane(4), plane(5));
    let mut at = mem + planes_len;
    let mut take = |bytes: usize| {
        let p = at;
        at += bytes;
        p
    };
    let cols = take(pw * 12) as *mut u32; // 3 × pw words for the chroma upsampling
    let upc = take(w * 12) as *mut u32; // 3 × w words for the upscaling (f > 1)
    let rows = take(w * 24) as *mut f32; // two camera rows at document width, 3 values each (f > 1)
    let dec8 = take(256 * 4) as *mut f32;
    let sh = take((SH_LEN + 1) * 4) as *mut f32;
    let lut = take(LUT_LEN * 4) as *mut f32;
    let blown_cols = take(pw) as *mut u8; // pw flags (zeroed by memory.grow)
    // SAFETY: allocated above; the static LUT holds LUT_LEN u16
    unsafe {
        build_shaper(sh);
        // the camera's u16 LUT as floats, once per band: the per-pixel lookup then skips 12
        // conversions (the LUT is ~150k values, a band ~300k pixels)
        let src = cam.lut.0.as_ptr() as *const u16;
        for j in 0..LUT_LEN {
            *lut.add(j) = *src.add(j) as f32 * (1.0 / 65535.0);
        }
    }
    let k = |j: usize| if j < kn { plane(6 + j) } else { core::ptr::null_mut() };
    let inp = |j: usize| if j < ln { plane(6 + kn + j) } else { core::ptr::null_mut() };
    let pl = Planes { inp: [inp(0), inp(1), inp(2), inp(3)], y: py, cb: pcb, cr: pcr, blown, cols: blown_cols, keep: [k(0), k(1), k(2)] };
    let dec8 = if depth == 8 {
        for l in 0..256 {
            // SAFETY: 256 floats allocated above
            unsafe { *dec8.add(l) = lerp_tab(dec, l as f32 / 255.0) };
        }
        dec8 as *const f32
    } else {
        core::ptr::null()
    };
    let doc = Doc { linear: linear_doc, grey, ident, n, dec, dec8, m_in, ev };

    // ---------------------------------------------------------------- 0: camera pixels (f > 1)
    if f > 1 {
        let [ir, ig, ib, ic] = pl.inp;
        // each camera column's first and last document column in the band (`upc` is free until
        // the upscaling)
        let (xa, xb) = (upc, unsafe { upc.add(pw) });
        // the band's document pixels (first, last; band-relative) of block `b` along one axis
        let span = |b: i32, o: i32, len: usize| -> (usize, usize) {
            let lo = if b * fi > o { b * fi } else { o };
            let hi = if b * fi + fi - 1 < o + len as i32 - 1 { b * fi + fi - 1 } else { o + len as i32 - 1 };
            ((lo - o) as usize, (hi - o) as usize)
        };
        // SAFETY: buf holds w × h × n samples, the input planes pw × ph floats, upc 3 × w words
        // (pw <= w)
        unsafe {
            for lx in 0..pw {
                let (a, b) = span(bx0 + lx as i32, x0, w);
                *xa.add(lx) = a as u32;
                *xb.add(lx) = b as u32;
            }
            // per camera pixel the sums of the document's values (encoded: decoded once per block,
            // a little darker on fine bright detail than light would average, and faster) and of
            // its clipped pixels, gathered row by row as the camera pixel's own locals; then the
            // mean, in sRGB primaries
            for ly in 0..ph {
                let (ya, yb) = span(by0 + ly as i32, y0, h);
                let cy = (yb - ya + 1) as f32;
                for lx in 0..pw {
                    let (xa_, xb_) = (*xa.add(lx) as usize, *xb.add(lx) as usize);
                    let p0 = buf.add((ya * w + xa_) * n);
                    let full = xb_ - xa_ + 1 == f && yb - ya + 1 == f;
                    let (sr, sg, sb, sc) = match (full, f) {
                        (true, 2) => block_sum::<2>(p0, w * n, n, grey, &doc),
                        (true, 3) => block_sum::<3>(p0, w * n, n, grey, &doc),
                        (true, _) => block_sum::<4>(p0, w * n, n, grey, &doc),
                        _ => {
                            let (mut sr, mut sg, mut sb, mut sc) = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
                            for y in ya..=yb {
                                let mut p = buf.add((y * w + xa_) * n);
                                for _ in xa_..=xb_ {
                                    sr += *p;
                                    if !grey {
                                        sg += *p.add(1);
                                        sb += *p.add(2);
                                    }
                                    if doc.clipped(p) {
                                        sc += 1.0;
                                    }
                                    p = p.add(n);
                                }
                            }
                            (sr, sg, sb, sc)
                        }
                    };
                    let inv = 1.0 / (cy * (xb_ - xa_ + 1) as f32);
                    let l = ly * pw + lx;
                    let r = doc.lin(sr * inv);
                    let (gg, bb) = if grey { (r, r) } else { (doc.lin(sg * inv), doc.lin(sb * inv)) };
                    let (r, gg, bb) = if ident { (r, gg, bb) } else { mat(&m_in, r, gg, bb) };
                    *ir.add(l) = r;
                    *ig.add(l) = gg;
                    *ib.add(l) = bb;
                    *ic.add(l) = sc * inv;
                }
            }
        }
    }

    // ---------------------------------------------------------------- 1-3: noise and the camera
    let (sx0, sy0) = if f == 1 { (x0, y0) } else { (bx0, by0) };
    let sensor = Sensor {
        lut,
        sh,
        s: cam.scene_to_raw,
        a: cam.raw_to_scene,
        inv_fw: iso_gain / cam.full_well,
        read_var: {
            // read noise in electrons: a part before the amplifier (stays) and one after it
            // (shrinks with the gain); `READ_NOISE` is roughly their total at base ISO
            let (pre, post) = (READ_NOISE * 0.25, READ_NOISE / iso_gain);
            let sigma = sqrt(pre * pre + post * post) * iso_gain / cam.full_well;
            sigma * sigma
        },
        amp: noise_amt * (1.0 / 32.0),
        noise: NOISE_BYTES.0.as_ptr() as *const i8,
        ox,
        oy,
        x0: sx0,
        y0: sy0,
    };
    // SAFETY: buf holds w × h × n samples, the planes pw × ph values each
    // every pixel takes the full path: sensor noise makes each one different, so a colour cache
    // (as in Marcs Film Emulator) misses too often to pay (tried: 30-50 % slower on photos)
    macro_rules! run {
        ($noise:literal, $keep:literal, $low:literal) => {
            if cam.channels == 4 {
                camera::<4, $noise, $keep, $low>(buf, pw, ph, &doc, &sensor, &pl)
            } else {
                camera::<3, $noise, $keep, $low>(buf, pw, ph, &doc, &sensor, &pl)
            }
        };
    }
    let bbox = unsafe {
        match (noise_amt > 0.0, keep, f > 1) {
            (true, false, false) => run!(true, false, false),
            (false, false, false) => run!(false, false, false),
            (true, true, _) => run!(true, true, false),
            (false, true, _) => run!(false, true, false),
            (true, false, true) => run!(true, false, true),
            (false, false, true) => run!(false, false, true),
        }
    };

    unsafe {
        if let Some((ax0, ax1, ay0, ay1)) = bbox {
            // ---------------------------------------------------------------- 4: blooming
            // charge spilling along the column, both ways, falling off over `reach` pixels; reach
            // and amount are kept inside the overlap so bands meet without seams. Only columns
            // with a blown pixel get any (the others would add exactly nothing). In Y'CbCr, mixing
            // r, g, b towards white by `s` is Y towards 1 and Cb, Cr towards 0.
            if bloom_amt > 0.0 {
                // reach <= 3.5 document px: falls below 1 % within the 16 px overlap; in camera
                // pixels 1/f of it, with the spill per pixel f times as strong
                let reach_doc = 1.5 + 1.5 * bloom_amt;
                let reach_doc = if reach_doc > 3.5 { 3.5 } else { reach_doc };
                let reach = reach_doc / f as f32;
                let reach = if reach > 1.0 { reach } else { 1.0 };
                // twice the spill per pixel of a 7 px reach, so the streak is as bright at the light
                let gain = 0.16 * bloom_amt * reach_doc / reach;
                let keep_ = 1.0 - 1.0 / reach;
                for x in ax0..=ax1 {
                    if *blown_cols.add(x) == 0 {
                        continue;
                    }
                    let mut acc = 0.0f32;
                    for y in 0..ph {
                        acc = acc * keep_ + *blown.add(y * pw + x);
                        *s1.add(y * pw + x) = acc;
                    }
                    acc = 0.0;
                    for y in (0..ph).rev() {
                        let e = *blown.add(y * pw + x);
                        acc = acc * keep_ + e;
                        let streak = (*s1.add(y * pw + x) + acc - e) * gain;
                        let s = if streak < 1.0 { streak } else { 1.0 };
                        let i = y * pw + x;
                        *py.add(i) += (1.0 - *py.add(i)) * s;
                        *pcb.add(i) *= 1.0 - s;
                        *pcr.add(i) *= 1.0 - s;
                    }
                }
            }

            // ---------------------------------------------------------------- 5: purple fringing
            // on the blown pixels' bounding box plus a margin beyond the blur's reach (4 px)
            if fringe_amt > 0.0 {
                const M: usize = 6;
                let (xa, xb) = (ax0.saturating_sub(M), (ax1 + M).min(pw - 1));
                let (ya, yb) = (ay0.saturating_sub(M), (ay1 + M).min(ph - 1));
                for y in ya..=yb {
                    for x in xa..=xb {
                        let e = *blown.add(y * pw + x);
                        *s1.add(y * pw + x) = if e > 1.0 { 1.0 } else { e };
                    }
                }
                // 2 document px wide; a camera pixel is wider already
                blur_rect(s1.add(ya * pw + xa), s2.add(ya * pw + xa), pw, xb - xa + 1, yb - ya + 1, if f == 1 { 2 } else { 1 });
                // the fringe's colour (r +0.45, g -0.08, b +0.85 per unit) in Y'CbCr
                const FR: f32 = 0.45;
                const FG: f32 = -0.08;
                const FB: f32 = 0.85;
                const FY: f32 = 0.299 * FR + 0.587 * FG + 0.114 * FB;
                for y in ya..=yb {
                    for x in xa..=xb {
                        let i = y * pw + x;
                        let e = *blown.add(i);
                        let edge = *s1.add(i) - if e > 1.0 { 1.0 } else { e }; // the dark side of the edge
                        if edge > 0.0 {
                            let f = edge * fringe_amt * 0.9;
                            *py.add(i) += f * FY;
                            *pcb.add(i) += f * (FB - FY);
                            *pcr.add(i) += f * (FR - FY);
                        }
                    }
                }
            }
        }

        // ---------------------------------------------------------------- 6: noise reduction, sharpening
        // chroma at half resolution and blurred (the cameras' main weapon against colour noise);
        // the radius in document px, so at f > 1 in camera pixels 1/f of it, within the overlap
        let rc = (nr_amt * 1.25 + 0.5) as usize;
        let rc = if rc > 6 { 6 } else { rc };
        let rc = if f == 1 { rc } else { (rc as f32 / f as f32 + 0.5) as usize };
        // reach 4 rc + 2 camera px within the overlap (16 px: up to 3 at f = 1, 1 at f = 2)
        let rc = rc.min((ov / f).saturating_sub(4) / 4);
        // the 2x2 blocks on the image's grid (planes start at x0, y0 or the block bx0, by0)
        let (gx, gy) = if f == 1 { (x0, y0) } else { (bx0, by0) };
        let (px_, py_) = ((gx & 1) as usize, (gy & 1) as usize);
        let half = (pw + px_).div_ceil(2) * (ph + py_).div_ceil(2);
        chroma_half(pcb, pcr, s1, s1.add(half), s2, cols, pw, ph, px_, py_, rc);
        // luma, against one blur of itself: softened towards it at higher noise reduction (the
        // "watercolour"), then sharpened away from it (unsharp mask, with its halos)
        let wl = (nr_amt - 0.6) * 0.35;
        let wl = if wl < 0.0 { 0.0 } else if wl > 0.75 { 0.75 } else { wl };
        if wl > 0.0 || sharpen_amt > 0.0 {
            let rs = (cam.sharpen_radius + 0.5) as usize;
            if rs <= 1 {
                blur1(py, s1, s2, pw, ph);
            } else {
                for i in 0..area {
                    *s1.add(i) = *py.add(i);
                }
                blur(s1, s2, pw, ph, rs);
            }
            let k = sharpen_amt * 1.6;
            for i in 0..area {
                let b = *s1.add(i);
                let y = *py.add(i);
                let y = y + (b - y) * wl;
                let d = y - b;
                // a small threshold keeps flat areas from sharpening their own noise as much
                let d = if d > 0.004 { d - 0.004 } else if d < -0.004 { d + 0.004 } else { 0.0 };
                *py.add(i) = y + d * k;
            }
        }

        // ---------------------------------------------------------------- 7: back, Strength, write
        let direct = !linear_doc && ident && !grey && !mixing;
        let [k0, k1, k2] = pl.keep;
        // one pixel: the camera's Y'CbCr (sRGB-encoded) into the document, mixed by Strength
        // with the original (from the planes at `i` with `keep`, else decoded from `p`)
        let put = |p: *mut f32, i: usize, yy: f32, cb: f32, cr: f32| {
            let er = yy + cr;
            let eb = yy + cb;
            let eg = (yy - 0.299 * er - 0.114 * eb) / 0.587;
            if direct {
                // the camera's output is sRGB-encoded, like the document (the host clamps 8/16-bit
                // results to 0..1 itself)
                *p = er;
                *p.add(1) = eg;
                *p.add(2) = eb;
                return;
            }
            // camera output (sRGB-encoded) -> linear sRGB
            let (cr_, cg_, cb_) = (lerp_tab(odec, er), lerp_tab(odec, eg), lerp_tab(odec, eb));
            if grey {
                let c = 0.2126 * cr_ + 0.7152 * cg_ + 0.0722 * cb_;
                let v = if mixing {
                    let o = if keep { *k0.add(i) } else { doc.lin(*p) };
                    o + (c - o) * strength
                } else {
                    c
                };
                *p = if linear_doc { v } else { lerp_tab(denc, v) };
            } else {
                // the camera's output is sRGB; into the document's primaries
                let (tr, tg, tb) = if ident { (cr_, cg_, cb_) } else { mat(&m_out, cr_, cg_, cb_) };
                let (vr, vg, vb) = if mixing {
                    let (or, og, ob) = if keep { (*k0.add(i), *k1.add(i), *k2.add(i)) } else { (doc.lin(*p), doc.lin(*p.add(1)), doc.lin(*p.add(2))) };
                    (or + (tr - or) * strength, og + (tg - og) * strength, ob + (tb - ob) * strength)
                } else {
                    (tr, tg, tb)
                };
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
        };
        if f == 1 {
            for y in ov..h - ov {
                for x in ov..w - ov {
                    let i = y * w + x;
                    let p = buf.add(i * n);
                    if alpha && *p.add(n - 1) <= 0.0 {
                        continue;
                    }
                    put(p, i, *py.add(i), *pcb.add(i), *pcr.add(i));
                }
            }
        } else {
            // the camera's result per camera pixel in the form the document takes: sRGB-encoded
            // (direct), else linear light in the document's primaries (grey: its luminance)
            for i in 0..area {
                let (yy, cb, cr) = (*py.add(i), *pcb.add(i), *pcr.add(i));
                let er = yy + cr;
                let eb = yy + cb;
                let eg = (yy - 0.299 * er - 0.114 * eb) / 0.587;
                let (a, b, c) = if direct {
                    (er, eg, eb)
                } else {
                    let (lr, lg, lb) = (lerp_tab(odec, er), lerp_tab(odec, eg), lerp_tab(odec, eb));
                    if grey {
                        (0.2126 * lr + 0.7152 * lg + 0.0722 * lb, 0.0, 0.0)
                    } else if ident {
                        (lr, lg, lb)
                    } else {
                        mat(&m_out, lr, lg, lb)
                    }
                };
                *py.add(i) = a;
                *pcb.add(i) = b;
                *pcr.add(i) = c;
            }
            // scaled back up, bilinear between camera-pixel centres: along x once per camera row
            // (two rows kept, at document width), along y per document pixel
            let inv_f = 1.0 / f as f32;
            let at_ = |d: i32, b0: i32, len: usize| -> (usize, usize, f32) {
                let u = (d as f32 + 0.5) * inv_f - 0.5 - b0 as f32;
                let u = if u > 0.0 { u } else { 0.0 };
                let i = u as usize;
                if i + 1 < len { (i, i + 1, u - i as f32) } else { (len - 1, len - 1, 0.0) }
            };
            let (uix, ujx, utx) = (upc, upc.add(w), upc.add(2 * w) as *mut f32);
            for x in ov..w - ov {
                let (i, j, t) = at_(x0 + x as i32, bx0, pw);
                *uix.add(x) = i as u32;
                *ujx.add(x) = j as u32;
                *utx.add(x) = t;
            }
            let nc = if grey { 1 } else { 3 };
            let planes3 = [py, pcb, pcr];
            // a camera row along x into `dst` ([x][c], document width)
            let widen = |ly: usize, dst: *mut f32| {
                for x in ov..w - ov {
                    let (ix, jx, tx) = (*uix.add(x) as usize, *ujx.add(x) as usize, *utx.add(x));
                    for (c, q) in planes3.iter().enumerate().take(nc) {
                        let (a, b) = (*q.add(ly * pw + ix), *q.add(ly * pw + jx));
                        *dst.add(x * 3 + c) = a + (b - a) * tx;
                    }
                }
            };
            let (mut ra, mut rb) = (rows, rows.add(w * 3));
            let (mut la, mut lb) = (usize::MAX, usize::MAX);
            for y in ov..h - ov {
                let (iy, jy, ty) = at_(y0 + y as i32, by0, ph);
                if la != iy {
                    if lb == iy {
                        core::mem::swap(&mut ra, &mut rb);
                        la = iy;
                        lb = usize::MAX;
                    } else {
                        widen(iy, ra);
                        la = iy;
                    }
                }
                if lb != jy {
                    widen(jy, rb);
                    lb = jy;
                }
                for x in ov..w - ov {
                    let p = buf.add((y * w + x) * n);
                    if alpha && *p.add(n - 1) <= 0.0 {
                        continue;
                    }
                    let v = |c: usize| {
                        let a = *ra.add(x * 3 + c);
                        a + (*rb.add(x * 3 + c) - a) * ty
                    };
                    if direct {
                        // the host clamps 8/16-bit results to 0..1 itself
                        *p = v(0);
                        *p.add(1) = v(1);
                        *p.add(2) = v(2);
                    } else if grey {
                        let c = v(0);
                        let c = if mixing { let o = doc.lin(*p); o + (c - o) * strength } else { c };
                        *p = if linear_doc { c } else { lerp_tab(denc, c) };
                    } else {
                        let (tr, tg, tb) = (v(0), v(1), v(2));
                        let (vr, vg, vb) = if mixing {
                            let (or, og, ob) = (doc.lin(*p), doc.lin(*p.add(1)), doc.lin(*p.add(2)));
                            (or + (tr - or) * strength, og + (tg - og) * strength, ob + (tb - ob) * strength)
                        } else {
                            (tr, tg, tb)
                        };
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
    }
    0
}
