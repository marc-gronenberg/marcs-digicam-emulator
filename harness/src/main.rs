//! Test host for PhotoCraft filter plug-ins, mirroring PhotoCraft's `crates/plugins` host:
//! wasmi 2.0 with fuel metering and strict limits, no imports, a fresh instance per band,
//! tile-row bands (256 px) as wide as fit in 4 MiB, the manifest's overlap with edge pixels
//! repeated, `_image` geometry in the params, fuel = 50 M + 4,000 per sample, 60 s wall clock.
//!
//! harness <plugin.wasm> <in.png|jpg> <out.png> [params-json] [depth 8|16|32] [--grey] [--threads N]
use std::time::{Duration, Instant};

use wasmi::{Config, EnforcedLimits, Engine, Linker, Module, Store, StoreLimits, StoreLimitsBuilder, TypedFunc};

const TILE: i32 = 256;
const FUEL_BASE: u64 = 50_000_000;
const FUEL_PER_SAMPLE: u64 = 4_000;
const MAX_MEM: usize = 512 << 20;
const BAND_BYTES: usize = 4 << 20;

struct Host {
    engine: Engine,
    module: Module,
}

impl Host {
    /// Compiles the plug-in with PhotoCraft's engine settings; panics if it imports anything.
    fn load(bytes: &[u8]) -> Host {
        let mut cfg = Config::default();
        cfg.consume_fuel(true).enforced_limits(EnforcedLimits::strict()).set_max_recursion_depth(1024);
        let engine = Engine::new(&cfg);
        let module = Module::new(&engine, bytes).expect("module");
        if let Some(i) = module.imports().next() {
            panic!("module imports {}.{} (not allowed)", i.module(), i.name());
        }
        Host { engine, module }
    }

    fn store(&self) -> (Store<StoreLimits>, wasmi::Instance, wasmi::Memory) {
        let limits = StoreLimitsBuilder::new().memory_size(MAX_MEM).memories(1).tables(4).table_elements(100_000).instances(1).build();
        let mut store = Store::new(&self.engine, limits);
        store.limiter(|l| l);
        store.set_fuel(20_000_000).unwrap();
        let inst = Linker::<StoreLimits>::new(&self.engine).instantiate_and_start(&mut store, &self.module).expect("instantiate");
        let mem = inst.get_memory(&store, "memory").expect("memory export");
        (store, inst, mem)
    }

    /// Returns fuel used.
    fn filter(&self, data: &mut [f32], w: u32, h: u32, ch: u32, fmt: u32, params: &[u8]) -> Result<u64, String> {
        let (mut store, inst, mem) = self.store();
        let alloc: TypedFunc<i32, i32> = inst.get_typed_func(&store, "pc_alloc").map_err(|e| e.to_string())?;
        type Args = (i32, i32, i32, i32, i32, i32, i32, i32);
        let filter: TypedFunc<Args, i32> = inst.get_typed_func(&store, "pc_filter").map_err(|e| e.to_string())?;
        store.set_fuel(FUEL_BASE).unwrap();
        let bytes = data.len() * 4;
        let ptr = alloc.call(&mut store, bytes as i32).map_err(|e| e.to_string())? as usize;
        let pptr = alloc.call(&mut store, params.len().max(1) as i32).map_err(|e| e.to_string())? as usize;
        if ptr == 0 || pptr == 0 {
            return Err("pc_alloc returned 0".into());
        }
        {
            let m = mem.data_mut(&mut store);
            for (d, v) in m[ptr..ptr + bytes].chunks_exact_mut(4).zip(data.iter()) {
                d.copy_from_slice(&v.to_le_bytes());
            }
            m[pptr..pptr + params.len()].copy_from_slice(params);
        }
        let budget = FUEL_BASE + data.len() as u64 * FUEL_PER_SAMPLE;
        store.set_fuel(budget).unwrap();
        let rc = filter
            .call(&mut store, (ptr as i32, bytes as i32, w as i32, h as i32, ch as i32, fmt as i32, pptr as i32, params.len() as i32))
            .map_err(|e| format!("trap: {e}"))?;
        if rc != 0 {
            return Err(format!("pc_filter returned error code {rc}"));
        }
        let used = budget - store.get_fuel().unwrap();
        let m = mem.data(&store);
        for (v, s) in data.iter_mut().zip(m[ptr..ptr + bytes].chunks_exact(4)) {
            *v = f32::from_le_bytes([s[0], s[1], s[2], s[3]]);
        }
        Ok(used)
    }
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let threads: usize = a.iter().position(|s| s == "--threads").and_then(|i| a.get(i + 1)).and_then(|s| s.parse().ok()).unwrap_or(2);
    let grey = a.iter().any(|s| s == "--grey");
    let depth: u32 = a.get(5).and_then(|s| s.parse().ok()).unwrap_or(8);
    let user: serde_json::Value = serde_json::from_str(a.get(4).map(String::as_str).unwrap_or("{}")).expect("params json");

    let bytes = std::fs::read(&a[1]).expect("wasm");
    let t = Instant::now();
    let host = Host::load(&bytes);
    let (mut store, inst, mem) = host.store();
    let ver: TypedFunc<(), i32> = inst.get_typed_func(&store, "pc_abi_version").unwrap();
    assert_eq!(ver.call(&mut store, ()).unwrap(), 1, "abi version");
    let man: TypedFunc<(), i64> = inst.get_typed_func(&store, "pc_manifest").unwrap();
    let packed = man.call(&mut store, ()).unwrap() as u64;
    let (mp, ml) = ((packed & 0xffff_ffff) as usize, (packed >> 32) as usize);
    let manifest: serde_json::Value = serde_json::from_slice(&mem.data(&store)[mp..mp + ml]).expect("manifest json");
    println!("loaded {:?} v{} ({} bytes, {:?})", manifest["name"].as_str().unwrap(), manifest["version"].as_str().unwrap(), bytes.len(), t.elapsed());
    let ov = manifest["overlap"].as_i64().unwrap_or(0) as i32;

    // Resolve params like Manifest::resolve_params: defaults, clamp numbers, round ints.
    let mut params = serde_json::Map::new();
    for (k, spec) in manifest["params"].as_object().unwrap() {
        let v = user.get(k).cloned().unwrap_or_else(|| spec["default"].clone());
        let v = match spec["type"].as_str().unwrap() {
            "number" => serde_json::json!(v.as_f64().unwrap().clamp(spec["min"].as_f64().unwrap(), spec["max"].as_f64().unwrap())),
            "int" => serde_json::json!((v.as_f64().unwrap().round() as i64).clamp(spec["min"].as_i64().unwrap(), spec["max"].as_i64().unwrap())),
            _ => v,
        };
        params.insert(k.clone(), v);
    }
    println!("params {}", serde_json::Value::Object(params.clone()));

    // Image -> f32 surface (sRGB for 8/16-bit, linear for 32-bit), RGBA or GA.
    let img = image::open(&a[2]).expect("image").to_rgba32f();
    let (w, h) = (img.width() as i32, img.height() as i32);
    let n: usize = if grey { 2 } else { 4 };
    let lin = |v: f32| if v <= 0.04045 { v / 12.92 } else { ((v + 0.055) / 1.055).powf(2.4) };
    let q = |v: f32| match depth {
        8 => (v * 255.0).round() / 255.0,
        16 => (v * 65535.0).round() / 65535.0,
        _ => lin(v),
    };
    let mut surf = Vec::with_capacity((w * h) as usize * n);
    for p in img.pixels() {
        let [r, g, b, al] = p.0;
        if grey {
            surf.extend_from_slice(&[q(0.2126 * r + 0.7152 * g + 0.0722 * b), al]);
        } else {
            surf.extend_from_slice(&[q(r), q(g), q(b), al]);
        }
    }
    let fmt = depth | (1 << 8) | ((if grey { 1 } else { 3 }) << 16);

    // Bands: one tile row tall, as many tiles across as fit in BAND_BYTES.
    let tile_bytes = (TILE * TILE) as usize * n * 4;
    let across = (BAND_BYTES.min(MAX_MEM / 2) / tile_bytes).clamp(1, 1024) as i32;
    let mut bands = vec![];
    let mut ty = 0;
    while ty < h {
        let mut tx = 0;
        while tx < w {
            bands.push((tx, ty, (tx + across * TILE).min(w), (ty + TILE).min(h)));
            tx += across * TILE;
        }
        ty += TILE;
    }
    let deadline = Instant::now() + Duration::from_secs(60);
    let t = Instant::now();
    let surf_ref = &surf;
    let params_ref = &params;
    let host_ref = &host;
    let results: Vec<(usize, Vec<f32>, u64)> = std::thread::scope(|s| {
        let mut hs = vec![];
        for tid in 0..threads {
            let bands = &bands;
            hs.push(s.spawn(move || {
                let mut out = vec![];
                for (bi, &(x0, y0, x1, y1)) in bands.iter().enumerate().filter(|(i, _)| i % threads == tid) {
                    assert!(Instant::now() < deadline, "time budget exceeded");
                    let (ix0, iy0, ix1, iy1) = (x0 - ov, y0 - ov, x1 + ov, y1 + ov);
                    let (iw, ih) = ((ix1 - ix0) as usize, (iy1 - iy0) as usize);
                    let mut data = Vec::with_capacity(iw * ih * n);
                    for y in iy0..iy1 {
                        let sy = y.clamp(0, h - 1);
                        for x in ix0..ix1 {
                            let sx = x.clamp(0, w - 1);
                            let i = (sy * w + sx) as usize * n;
                            data.extend_from_slice(&surf_ref[i..i + n]);
                        }
                    }
                    let mut p = params_ref.clone();
                    p.insert(
                        "_image".into(),
                        serde_json::json!({"x": ix0, "y": iy0, "width": iw, "height": ih, "overlap": ov,
                            "canvasWidth": w, "canvasHeight": h, "mode": if grey {"grayscale"} else {"rgb"},
                            "depth": depth, "alpha": true, "channels": n}),
                    );
                    let pb = serde_json::to_vec(&serde_json::Value::Object(p)).unwrap();
                    let used = host_ref.filter(&mut data, iw as u32, ih as u32, n as u32, fmt, &pb).unwrap_or_else(|e| panic!("band {bi}: {e}"));
                    out.push((bi, data, used / (iw * ih * n) as u64));
                }
                out
            }));
        }
        hs.into_iter().flat_map(|h| h.join().unwrap()).collect()
    });
    let dt = t.elapsed();
    let max_fuel = results.iter().map(|r| r.2).max().unwrap_or(0);
    let avg_fuel = results.iter().map(|r| r.2).sum::<u64>() as f64 / results.len().max(1) as f64;
    println!(
        "{w}x{h} ({:.1} MP), {} bands, {threads} threads: {:?}  (~{:.1} s per 24 MP on {threads} cores); fuel/sample avg {avg_fuel:.0} max {max_fuel} of {FUEL_PER_SAMPLE}",
        (w * h) as f64 / 1e6,
        bands.len(),
        dt,
        dt.as_secs_f64() * 24e6 / (w * h) as f64
    );

    // Write back the inner rectangles, clamp like the host, save as 8-bit sRGB PNG.
    let mut res = surf.clone();
    for (bi, data, _) in results {
        let (x0, y0, x1, y1) = bands[bi];
        let iw = (x1 - x0 + 2 * ov) as usize;
        for y in y0..y1 {
            for x in x0..x1 {
                let si = ((y - y0 + ov) as usize * iw + (x - x0 + ov) as usize) * n;
                let di = (y * w + x) as usize * n;
                for c in 0..n {
                    let mut v = data[si + c];
                    if !v.is_finite() {
                        v = surf[di + c];
                    }
                    if depth != 32 {
                        v = v.clamp(0.0, 1.0);
                    }
                    res[di + c] = v;
                }
            }
        }
    }
    let enc = |v: f32| if v <= 0.0031308 { v * 12.92 } else { 1.055 * v.max(0.0).powf(1.0 / 2.4) - 0.055 };
    let c8 = |v: f32| ((if depth == 32 { enc(v) } else { v }).clamp(0.0, 1.0) * 255.0 + 0.5) as u8;
    let mut o = image::RgbaImage::new(w as u32, h as u32);
    for (i, p) in o.pixels_mut().enumerate() {
        let s = &res[i * n..i * n + n];
        p.0 = if grey { [c8(s[0]), c8(s[0]), c8(s[0]), 255] } else { [c8(s[0]), c8(s[1]), c8(s[2]), 255] };
    }
    o.save(&a[3]).expect("save");
}

/// Checks on the shipped plug-ins (`dist/`), run by `build.sh`: `cd harness && cargo test --release`.
#[cfg(test)]
mod tests {
    use super::*;

    const W: usize = 256;
    const H: usize = 4;
    const RGBA: u32 = (1 << 8) | (3 << 16);
    // The ramp tests run the film without overlap (`_image.overlap` 0), so every pixel counts.

    fn host(name: &str) -> Host {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../dist/");
        Host::load(&std::fs::read(format!("{path}{name}")).expect("plug-in in dist/ (run build.sh first)"))
    }

    /// A grey ramp, one level per column, `top` at the right edge; RGBA, opaque.
    fn ramp(top: f32) -> Vec<f32> {
        let mut v = Vec::with_capacity(W * H * 4);
        for _ in 0..H {
            for x in 0..W {
                let l = x as f32 / (W - 1) as f32 * top;
                v.extend_from_slice(&[l, l, l, 1.0]);
            }
        }
        v
    }

    /// Runs a band of `w × h` RGBA pixels whose outer `ov` pixels are overlap, with the host's
    /// `_image` geometry added to `params` (a JSON object).
    fn run_band(host: &Host, data: &mut [f32], w: usize, h: usize, ov: usize, depth: u32, params: &str) {
        let p = format!(r#"{{"_image":{{"x":0,"y":0,"width":{w},"height":{h},"overlap":{ov}}}{}{}"#, if params.len() > 2 { "," } else { "" }, &params[1..]);
        host.filter(data, w as u32, h as u32, 4, depth | RGBA, p.as_bytes()).expect("filter");
    }

    /// The ramp band, no overlap.
    fn run(host: &Host, data: &mut [f32], depth: u32, params: &str) {
        run_band(host, data, W, H, 0, depth, params);
    }

    /// A band of `w × h` with `ov` overlap: dark grey with a white disc in the middle (a highlight
    /// for the halation to glow around), 8-bit levels.
    fn disc(w: usize, h: usize) -> Vec<f32> {
        let mut v = Vec::with_capacity(w * h * 4);
        for y in 0..h {
            for x in 0..w {
                let (dx, dy) = (x as f32 - w as f32 / 2.0, y as f32 - h as f32 / 2.0);
                let l = if dx * dx + dy * dy < 400.0 { 1.0 } else { 51.0 / 255.0 };
                v.extend_from_slice(&[l, l * 0.9, l * 0.8, 1.0]);
            }
        }
        v.iter().map(|v: &f32| (v * 255.0).round() / 255.0).collect()
    }

    fn inner(data: &[f32], w: usize, h: usize, ov: usize) -> Vec<f32> {
        let mut out = vec![];
        for y in ov..h - ov {
            out.extend_from_slice(&data[(y * w + ov) * 4..(y * w + w - ov) * 4]);
        }
        out
    }

    fn luma(p: &[f32]) -> f32 {
        0.2126 * p[0] + 0.7152 * p[1] + 0.0722 * p[2]
    }

    const CAMERAS: [&str; 8] = ["Pocket 2002", "Prosumer 2003", "Compact 2004", "Compact 2005", "Compact 2006", "Compact 2007", "Ultracompact 2008", "Prosumer 2009"];

    fn flat(w: usize, h: usize, v: f32) -> Vec<f32> {
        (0..w * h).flat_map(|_| [v, v, v, 1.0]).collect()
    }

    fn std_luma(data: &[f32]) -> f32 {
        let l: Vec<f32> = data.chunks(4).map(luma).collect();
        let m = l.iter().sum::<f32>() / l.len() as f32;
        (l.iter().map(|v| (v - m) * (v - m)).sum::<f32>() / l.len() as f32).sqrt()
    }

    #[test]
    fn strength_0_returns_the_original() {
        let cam = host("marcs-digicam-emulator.wasm");
        for depth in [8, 16] {
            let orig = ramp(1.0);
            let mut out = orig.clone();
            run(&cam, &mut out, depth, r#"{"strength":0,"iso":"1600"}"#);
            let worst = orig.iter().zip(&out).map(|(a, b)| (a - b).abs() * 255.0).fold(0.0f32, f32::max);
            assert!(worst <= 1.0, "{depth}-bit: original changed by {worst} levels");
        }
    }

    #[test]
    fn every_camera_runs_at_every_depth_and_iso() {
        let cam = host("marcs-digicam-emulator.wasm");
        for name in CAMERAS {
            for iso in ["100", "1600"] {
                for depth in [8, 16, 32] {
                    let mut out = ramp(1.0);
                    run(&cam, &mut out, depth, &format!(r#"{{"camera":"{name}","iso":"{iso}"}}"#));
                    assert!(out.iter().all(|v| v.is_finite()), "{name} ISO {iso} {depth}-bit: non-finite output");
                    assert!(luma(&out[(W - 1) * 4..]) > luma(&out[W * 2..]), "{name} ISO {iso} {depth}-bit: white not brighter than mid-grey");
                }
            }
        }
    }

    /// Without noise and spatial stages the camera's tone curve is a monotonic grey scale.
    #[test]
    fn grey_ramp_is_monotonic_without_noise() {
        let cam = host("marcs-digicam-emulator.wasm");
        for name in CAMERAS {
            let mut out = ramp(1.0);
            run(&cam, &mut out, 8, &format!(r#"{{"camera":"{name}","noise":0,"sharpening":0,"noiseReduction":0}}"#));
            let row: Vec<f32> = out[..W * 4].chunks(4).map(luma).collect();
            for x in 1..W {
                assert!(row[x] + 0.5 / 255.0 >= row[x - 1], "{name}: luminance falls at level {x}: {} -> {}", row[x - 1], row[x]);
            }
            // diffuse white (+2.5 stops) is near, not always at, clip: cameras with 3+ stops of
            // headroom keep it a little below 1
            assert!(row[W - 1] > 0.9, "{name}: white is {}", row[W - 1]);
        }
    }

    /// Sensor noise grows with ISO (measured on flat grey with noise reduction and sharpening off).
    #[test]
    fn noise_grows_with_iso() {
        let cam = host("marcs-digicam-emulator.wasm");
        let (w, h) = (128, 64);
        let mut last = 0.0;
        for iso in ["100", "400", "1600"] {
            let mut out = flat(w, h, 0.46);
            run_band(&cam, &mut out, w, h, 0, 8, &format!(r#"{{"camera":"Compact 2004","iso":"{iso}","noiseReduction":0,"sharpening":0}}"#));
            let sd = std_luma(&out) * 255.0;
            assert!(sd > last * 1.3, "ISO {iso}: noise {sd} levels, not clearly above {last}");
            last = sd;
        }
        let mut quiet = flat(w, h, 0.46);
        run_band(&cam, &mut quiet, w, h, 0, 8, r#"{"camera":"Compact 2004","noise":0,"noiseReduction":0,"sharpening":0}"#);
        assert!(std_luma(&quiet) * 255.0 < 0.01, "noise 0 still has noise");
    }

    /// A blown highlight turns white and blooms along the column, not along the row.
    #[test]
    fn blown_highlight_blooms_vertically() {
        let cam = host("marcs-digicam-emulator.wasm");
        let (w, h, ov) = (448, 256, 48);
        let mut out = disc(w, h);
        run_band(&cam, &mut out, w, h, ov, 8, r#"{"camera":"Pocket 2002","exposure":2,"blooming":200,"purpleFringing":0,"noise":0,"sharpening":0,"noiseReduction":0}"#);
        let px = |x: usize, y: usize| luma(&out[(y * w + x) * 4..]);
        let (cx, cy) = (w / 2, h / 2);
        assert!(px(cx, cy) > 0.97, "the blown disc is {}", px(cx, cy));
        // the disc's radius is 20 px: 8 px beyond its edge, above vs. beside it
        let (above, beside) = (px(cx, cy - 28), px(cx + 28, cy));
        assert!(above > beside + 0.05, "no vertical bloom: above {above}, beside {beside}");
    }
}
