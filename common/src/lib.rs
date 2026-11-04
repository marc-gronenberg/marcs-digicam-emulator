//! The plug-in's building blocks (`plugin/` uses them): the PhotoCraft plug-in ABI v1 glue,
//! the no-allocation JSON parameter reader, the table lookups and the blurs. `no_std`, no imports.
//!
//! The plug-in keeps its own exported `pc_abi_version`, `pc_manifest`, `pc_alloc`, `pc_filter`
//! and panic handler (they must live in the cdylib itself); they call in here.
#![cfg_attr(not(test), no_std)]

pub mod blur;

/// Error codes `pc_filter` returns.
pub const ERR_ARGS: i32 = 1;
pub const ERR_MODE: i32 = 2;
pub const ERR_MEMORY: i32 = 3;

/// `format` bit: the last channel is alpha.
pub const HAS_ALPHA: u32 = 1 << 8;

/// Entries of the 0..1 lookup tables (`lerp_tab`): `TN + 1` floats, v = i / TN.
pub const TN: usize = 4096;

/// 4-byte aligned static bytes, so `include_bytes!` data can be read as `f32`.
#[repr(C, align(4))]
pub struct Aligned<T: ?Sized>(pub T);

/// `pc_manifest`'s packed return: length in the high word, pointer in the low word.
#[inline(always)]
pub fn pack_manifest(m: &str) -> u64 {
    ((m.len() as u64) << 32) | m.as_ptr() as u64
}

/// `pc_alloc`: fresh pages from `memory.grow` (each band runs in a fresh instance, nothing is
/// freed). 0 if the host's memory limit is reached.
#[cfg(target_arch = "wasm32")]
#[inline(always)]
pub fn alloc(size: u32) -> u32 {
    let pages = (size as usize).div_ceil(65536).max(1);
    let old = core::arch::wasm32::memory_grow(0, pages);
    if old == usize::MAX { 0 } else { (old * 65536) as u32 }
}

#[inline(always)]
pub fn clamp01(x: f32) -> f32 {
    if x > 0.0 { if x < 1.0 { x } else { 1.0 } } else { 0.0 }
}

/// Linear interpolation in a `TN + 1`-entry table over 0..1 (the input is clamped).
///
/// # Safety
/// `tab` points at TN + 1 floats.
#[inline(always)]
pub unsafe fn lerp_tab(tab: *const f32, v: f32) -> f32 {
    let x = clamp01(v) * TN as f32;
    let i = x as usize;
    let i = if i < TN { i } else { TN - 1 };
    let f = x - i as f32;
    unsafe {
        let a = *tab.add(i);
        a + (*tab.add(i + 1) - a) * f
    }
}

// ----------------------------------------------------------------------------- parameters

/// The byte index just after `"key":` (and any blanks), or None if the key is not there.
fn after_key(json: &[u8], key: &str) -> Option<usize> {
    let k = key.as_bytes();
    let mut i = 0;
    while i + k.len() + 2 <= json.len() {
        if json[i] == b'"' && json[i + 1..].starts_with(k) && json.get(i + 1 + k.len()) == Some(&b'"') {
            let mut j = i + 2 + k.len();
            while j < json.len() && matches!(json[j], b' ' | b':' | b'\n' | b'\t' | b'\r') {
                j += 1;
            }
            return Some(j);
        }
        i += 1;
    }
    None
}

/// Finds `"key"` in the JSON and parses the number after its colon (no allocation).
pub fn json_num(json: &[u8], key: &str) -> Option<f32> {
    parse_num(&json[after_key(json, key)?..])
}

/// The string value of `"key"` in the JSON, if it is a string (no escapes needed for our labels).
pub fn json_str<'a>(json: &'a [u8], key: &str) -> Option<&'a [u8]> {
    let j = after_key(json, key)?;
    if json.get(j) != Some(&b'"') {
        return None;
    }
    let start = j + 1;
    let mut e = start;
    while e < json.len() && json[e] != b'"' {
        e += 1;
    }
    Some(&json[start..e])
}

/// A JSON number (sign, digits, fraction, exponent) at the start of `s`.
pub fn parse_num(s: &[u8]) -> Option<f32> {
    let mut i = 0;
    let neg = s.first() == Some(&b'-');
    if neg {
        i += 1;
    }
    let mut v = 0.0f64;
    let mut any = false;
    while i < s.len() && s[i].is_ascii_digit() {
        v = v * 10.0 + (s[i] - b'0') as f64;
        i += 1;
        any = true;
    }
    if i < s.len() && s[i] == b'.' {
        i += 1;
        let mut scale = 0.1;
        while i < s.len() && s[i].is_ascii_digit() {
            v += (s[i] - b'0') as f64 * scale;
            scale *= 0.1;
            i += 1;
            any = true;
        }
    }
    if i < s.len() && (s[i] == b'e' || s[i] == b'E') {
        i += 1;
        let eneg = s.get(i) == Some(&b'-');
        if eneg || s.get(i) == Some(&b'+') {
            i += 1;
        }
        let mut e = 0i32;
        while i < s.len() && s[i].is_ascii_digit() {
            e = e * 10 + (s[i] - b'0') as i32;
            i += 1;
        }
        let mut p = 1.0f64;
        for _ in 0..e.min(300) {
            p *= 10.0;
        }
        v = if eneg { v / p } else { v * p };
    }
    if !any {
        return None;
    }
    Some(if neg { -v as f32 } else { v as f32 })
}
