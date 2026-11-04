//! Box blurs for the plug-in's spatial stages (noise reduction, sharpening, purple fringing).
//! Two box passes each way (a tent, close enough to a Gaussian for these stages, at two thirds
//! of the cost of three).

/// One running-sum box blur pass of half-width `r` (edges repeat), along rows (`stride` 1,
/// `line_step` = row pitch) or columns (`stride` = row pitch, `line_step` 1).
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
    unsafe { blur_rect(a, tmp, w, w, h, r) }
}

/// `blur` of a `w × h` rectangle inside a plane whose rows are `pitch` floats apart (`a` and
/// `tmp` point at the rectangle's first value in their planes). Its edges repeat, as a plane's.
///
/// # Safety
/// `a` and `tmp` reach `(h - 1) × pitch + w` floats.
pub unsafe fn blur_rect(a: *mut f32, tmp: *mut f32, pitch: usize, w: usize, h: usize, r: usize) {
    if r == 0 {
        return;
    }
    for _ in 0..2 {
        unsafe {
            box_pass(a, tmp, h, w, pitch, 1, r);
            box_pass(tmp, a, w, h, 1, pitch, r);
        }
    }
}

/// `blur` with radius 1 (two 3-tap box passes each way) as one 5-tap pass each way, from `src`
/// into `dst` (`tmp` scratch, all `w × h`): the same result up to float rounding, in half the
/// passes and without a copy.
///
/// # Safety
/// `src`, `dst` and `tmp` hold `w × h` floats; `dst` and `tmp` do not overlap `src` or each other.
pub unsafe fn blur1(src: *const f32, dst: *mut f32, tmp: *mut f32, w: usize, h: usize) {
    const N: f32 = 1.0 / 9.0;
    unsafe {
        // along the rows: src -> tmp
        for y in 0..h {
            let (s, d) = (src.add(y * w), tmp.add(y * w));
            if w < 5 {
                for x in 0..w {
                    *d.add(x) = tent_edge(s, w, 1, x);
                }
                continue;
            }
            *d = tent_edge(s, w, 1, 0);
            *d.add(1) = tent_edge(s, w, 1, 1);
            for x in 2..w - 2 {
                let p = s.add(x);
                *d.add(x) = (*p.sub(2) + *p.add(2) + 2.0 * (*p.sub(1) + *p.add(1)) + 3.0 * *p) * N;
            }
            *d.add(w - 2) = tent_edge(s, w, 1, w - 2);
            *d.add(w - 1) = tent_edge(s, w, 1, w - 1);
        }
        // along the columns, row by row: tmp -> dst
        for y in 0..h {
            let d = dst.add(y * w);
            if y < 2 || y + 2 >= h {
                for x in 0..w {
                    *d.add(x) = tent_edge(tmp.add(x), h, w, y);
                }
                continue;
            }
            let (r0, r1, r2, r3, r4) = (tmp.add((y - 2) * w), tmp.add((y - 1) * w), tmp.add(y * w), tmp.add((y + 1) * w), tmp.add((y + 2) * w));
            for x in 0..w {
                *d.add(x) = (*r0.add(x) + *r4.add(x) + 2.0 * (*r1.add(x) + *r3.add(x)) + 3.0 * *r2.add(x)) * N;
            }
        }
    }
}

/// Two 3-tap box passes (edges repeat) at position `i` of a line of `len` values `stride` apart.
///
/// # Safety
/// `s` reaches `(len - 1) × stride` floats.
#[inline(always)]
unsafe fn tent_edge(s: *const f32, len: usize, stride: usize, i: usize) -> f32 {
    let last = len as isize - 1;
    let at = |j: isize| unsafe { *s.add(j.clamp(0, last) as usize * stride) };
    // the first pass's value at a clamped position (the second pass repeats its edges too)
    let b = |j: isize| {
        let j = j.clamp(0, last);
        (at(j - 1) + at(j) + at(j + 1)) * (1.0 / 3.0)
    };
    let i = i as isize;
    (b(i - 1) + b(i) + b(i + 1)) * (1.0 / 3.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `blur1` matches `blur` with radius 1, edges included, down to planes of one pixel.
    #[test]
    fn blur1_is_blur_with_radius_1() {
        let mut seed = 1u32;
        for (w, h) in [(1, 1), (2, 3), (3, 3), (4, 7), (5, 5), (6, 2), (17, 9), (40, 33)] {
            let src: Vec<f32> = (0..w * h)
                .map(|_| {
                    seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                    (seed >> 8) as f32 / (1 << 24) as f32
                })
                .collect();
            let mut want = src.clone();
            let mut tmp = vec![0.0; w * h];
            unsafe { blur(want.as_mut_ptr(), tmp.as_mut_ptr(), w, h, 1) };
            let mut got = vec![0.0; w * h];
            unsafe { blur1(src.as_ptr(), got.as_mut_ptr(), tmp.as_mut_ptr(), w, h) };
            let worst = want.iter().zip(&got).map(|(a, b)| (a - b).abs()).fold(0.0f32, f32::max);
            assert!(worst < 1e-5, "{w} x {h}: off by {worst}");
        }
    }
}
