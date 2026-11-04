//! Box blurs for the plug-in's spatial stages (noise reduction, sharpening, purple fringing).
//! Two box passes each way (a tent, close enough to a Gaussian for these stages, at two thirds
//! of the cost of three).

/// One running-sum box blur pass of half-width `r` (edges repeat), along rows (`stride` 1,
/// `line_step` = row length) or columns (`stride` = row length, `line_step` 1).
///
/// # Safety
/// `src` and `dst` hold `lines × len` values laid out as described.
pub unsafe fn box_pass(src: *const f32, dst: *mut f32, lines: usize, len: usize, line_step: usize, stride: usize, r: usize) {
    let inv = 1.0 / (2 * r + 1) as f32;
    let last = len - 1;
    for l in 0..lines {
        unsafe {
            let s = src.add(l * line_step);
            let d = dst.add(l * line_step);
            let mut acc = 0.0f32;
            for i in 0..=2 * r {
                let i = if i < r { 0 } else if i - r > last { last } else { i - r };
                acc += *s.add(i * stride);
            }
            for i in 0..len {
                *d.add(i * stride) = acc * inv;
                let next = if i + r < last { *s.add((i + r + 1) * stride) } else { *s.add(last * stride) };
                let prev = if i >= r { *s.add((i - r) * stride) } else { *s };
                acc += next - prev;
            }
        }
    }
}

/// A Gaussian-like blur (two box passes each way) of the `w × h` plane `a`, in place; `tmp`
/// holds `w × h` floats of scratch. Radius 0 leaves `a` as it is.
///
/// # Safety
/// `a` and `tmp` hold `w × h` floats.
pub unsafe fn blur(a: *mut f32, tmp: *mut f32, w: usize, h: usize, r: usize) {
    if r == 0 {
        return;
    }
    for _ in 0..2 {
        unsafe {
            box_pass(a, tmp, h, w, w, 1, r);
            box_pass(tmp, a, w, h, 1, w, r);
        }
    }
}
