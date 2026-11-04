"""The camera model of Marcs Digicam Emulator: what a 2000s CCD compact did to the light.

Per camera:

  scene linear sRGB
    -> smooth, non-negative scene spectrum under daylight (sigmoid-polynomial reflectance x 6504 K)
    -> raw signal of each colour channel through the sensor's spectral sensitivity: colour filter
       x silicon quantum efficiency x IR-cut filter. Primary RGB filters or complementary
       Ye/Cy/Mg/G filters, MODELLED from the physics of pigment and dye filters (no datasheet
       data: the CCD datasheets of these cameras are no longer published)
    -> exposure: 18 % grey at 2^-headroom of full well in the most sensitive channel
    -> per-channel clip at full well (the hard digital highlight: a channel that clips first
       shifts the hue of what is brighter)
    -> white balance on grey, then every channel clipped where the least amplified one
       saturates (a fully blown highlight is white)
    -> colour correction matrix, fitted by least squares from the sensor to sRGB (3x3 for RGB,
       3x4 for CMYG; white kept white). Where the sensor's sensitivities are not a linear
       combination of the eye's, the fit leaves hue errors: the sensor's own colour.
    -> clip to the output range, tone curve (grey target, highlight knee ending at clip, black
       crush, contrast), the maker's style in Oklab (saturation, hue twists of memory colours)
    -> sRGB output, baked into a 37^3 LUT over 13 stops of scene log exposure

Also written, for the plug-in: the scene->raw matrix (for clipping masks and signal-dependent
noise), the raw->output matrix (to put raw noise into the scene the LUT sees), the pixel's full
well, and the style of the spatial stages, as plugin/src/cameras.rs.

Run:  python3 digicam_model.py [camera ...] [--preview]
"""

from __future__ import annotations

import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "lut")
RUST = os.path.join(HERE, "..", "plugin", "src", "cameras.rs")

LAM = np.arange(380.0, 781.0, 5.0)
DL = 5.0

# ----------------------------------------------------------------------------- colour basics


def g2(lam, mu, s1, s2):
    s = np.where(lam < mu, s1, s2)
    return np.exp(-0.5 * ((lam - mu) / s) ** 2)


def logistic(lam, c, w):
    return 1.0 / (1.0 + np.exp(-(lam - c) / w))


def planck(lam_nm, t):
    lam = lam_nm * 1e-9
    h, c, k = 6.62607015e-34, 2.99792458e8, 1.380649e-23
    return 1.0 / (lam**5 * (np.exp(h * c / (lam * k * t)) - 1.0))


# CIE 1931 2-degree observer, multi-lobe analytic fit (Wyman, Sloan & Shirley 2013)
XBAR = 1.056 * g2(LAM, 599.8, 37.9, 31.0) + 0.362 * g2(LAM, 442.0, 16.0, 26.7) - 0.065 * g2(LAM, 501.1, 20.4, 26.2)
YBAR = 0.821 * g2(LAM, 568.8, 46.9, 40.5) + 0.286 * g2(LAM, 530.9, 16.3, 31.1)
ZBAR = 1.217 * g2(LAM, 437.0, 11.8, 36.0) + 0.681 * g2(LAM, 459.0, 26.0, 13.8)
CMF = np.stack([XBAR, YBAR, ZBAR])

SRGB_TO_XYZ = np.array([[0.4124564, 0.3575761, 0.1804375], [0.2126729, 0.7151522, 0.0721750], [0.0193339, 0.1191920, 0.9503041]])
D65_WHITE = SRGB_TO_XYZ @ np.ones(3)
BRADFORD = np.array([[0.8951, 0.2664, -0.1614], [-0.7502, 1.7135, 0.0367], [0.0389, -0.0685, 1.0296]])


def bradford(src_white, dst_white):
    s, d = BRADFORD @ src_white, BRADFORD @ dst_white
    return np.linalg.inv(BRADFORD) @ np.diag(d / s) @ BRADFORD


def white_of(spd):
    return (CMF * spd).sum(axis=1) * DL


def srgb_decode(v):
    return np.where(v <= 0.04045, v / 12.92, ((v + 0.055) / 1.055) ** 2.4)


def srgb_encode(v):
    v = np.clip(v, 0, 1)
    return np.where(v <= 0.0031308, v * 12.92, 1.055 * v ** (1 / 2.4) - 0.055)


_M1 = np.array([[0.4122214708, 0.5363325363, 0.0514459929], [0.2119034982, 0.6806995451, 0.1073969566], [0.0883024619, 0.2817188376, 0.6299787005]])
_M2 = np.array([[0.2104542553, 0.7936177850, -0.0040720468], [1.9779984951, -2.4285922050, 0.4505937099], [0.0259040371, 0.7827717662, -0.8086757660]])


def oklab(c):
    return np.cbrt(np.maximum(c @ _M1.T, 0.0)) @ _M2.T


def oklab_inv(lab):
    return ((lab @ np.linalg.inv(_M2).T) ** 3) @ np.linalg.inv(_M1).T


# ----------------------------------------------------------------------------- spectral upsampling

T_NORM = (LAM - LAM[0]) / (LAM[-1] - LAM[0])
T_POW = np.stack([np.ones_like(T_NORM), T_NORM, T_NORM**2])


def _sig(z):
    return 0.5 + 0.5 * z / np.sqrt(1.0 + z * z)


def _dsig(z):
    return 0.5 / (1.0 + z * z) ** 1.5


class Upsampler:
    """Smooth reflectance (sigmoid of a quadratic) x illuminant, matched exactly to each colour
    (as in Marcs Film Emulator)."""

    LEVEL = 0.7

    def __init__(self, illum_k):
        il = planck(LAM, illum_k)
        self.illum = il / ((CMF[1] * il).sum() * DL)
        self.cmf_w = (CMF * self.illum).T * DL
        self.rgb_to_xyz = bradford(D65_WHITE, white_of(self.illum)) @ SRGB_TO_XYZ

    def fit(self, rgb01, iters=200):
        n = rgb01.shape[0]
        target = rgb01 @ self.rgb_to_xyz.T
        c = np.zeros((n, 3))
        lam = np.full(n, 1e-3)

        def err(cc):
            z = cc @ T_POW
            return _sig(z) @ self.cmf_w - target, z

        e, z = err(c)
        cost = (e * e).sum(1)
        for _ in range(iters):
            ds = _dsig(z)
            jac = np.einsum("nl,il,lk->nki", ds, T_POW, self.cmf_w)
            jtj = np.einsum("nki,nkj->nij", jac, jac)
            jte = np.einsum("nki,nk->ni", jac, e)
            a = jtj + lam[:, None, None] * (np.einsum("nii->ni", jtj)[:, :, None] * np.eye(3)[None] + 1e-9 * np.eye(3)[None])
            step = np.linalg.solve(a, -jte[:, :, None])[:, :, 0]
            en, zn = err(c + step)
            cn = (en * en).sum(1)
            ok = cn < cost
            c[ok] += step[ok]
            e[ok], z[ok], cost[ok] = en[ok], zn[ok], cn[ok]
            lam = np.clip(np.where(ok, lam * 0.3, lam * 8.0), 1e-9, 1e9)
            if cost.max() < 1e-14:
                break
        return c

    def spectra(self, rgb_lin):
        rgb = np.maximum(rgb_lin, 0.0)
        m = np.maximum(rgb.max(1, keepdims=True), 1e-12)
        c = self.fit(np.clip(rgb / m, 0, 1) * self.LEVEL)
        return (m / self.LEVEL) * _sig(c @ T_POW) * self.illum[None, :]


UP = Upsampler(6504.0)

# ----------------------------------------------------------------------------- sensors (modelled)


def silicon(peak):
    """Quantum efficiency of a front-illuminated interline CCD with microlenses: low in the blue
    (absorbed in the gate and the surface), highest in the green-yellow, falling into the red as
    the photodiode gets too shallow for long wavelengths."""
    return 0.55 * g2(LAM, peak, 120.0, 170.0)


def ir_cut(c, w):
    """The IR-cut (hot-mirror plus absorptive glass) in front of the sensor, and its UV edge."""
    return (1.0 - logistic(LAM, c, w)) * logistic(LAM, 395.0, 6.0)


SENSORS = {
    # Primary pigment filters. Early: broad, overlapping green (more cross-talk, softer colour);
    # later: a little narrower and the red edge a little further out.
    "rgb_early": dict(cfa="rgb", b=(458, 30, 40), g=(530, 46, 48), r=(588, 13), leak=0.05, qe=560, ir=(652, 9)),
    "rgb_mid": dict(cfa="rgb", b=(455, 27, 36), g=(532, 41, 43), r=(592, 11), leak=0.04, qe=565, ir=(655, 8)),
    "rgb_late": dict(cfa="rgb", b=(452, 25, 33), g=(534, 38, 40), r=(596, 10), leak=0.03, qe=570, ir=(658, 7)),
    # Complementary dye filters (Ye, Cy, Mg, G): twice the light per filter, colour by subtraction
    "cmyg": dict(cfa="cmyg", ye=(497, 12), cy=(582, 12), mg=(540, 36, 0.9), g=(530, 44, 46), leak=0.03, qe=555, ir=(650, 9)),
}


def sensitivities(key):
    """Spectral sensitivities (K x L) of a sensor: filter x silicon x IR-cut."""
    s = SENSORS[key]
    base = silicon(s["qe"]) * ir_cut(*s["ir"])
    lk = s["leak"]
    if s["cfa"] == "rgb":
        b = 0.85 * g2(LAM, *s["b"]) + lk
        g = 0.90 * g2(LAM, *s["g"]) + lk
        r = 0.92 * logistic(LAM, *s["r"]) + lk
        filt = [r, g, b]
    else:
        ye = 0.95 * logistic(LAM, *s["ye"]) + lk
        cy = 0.95 * (1.0 - logistic(LAM, *s["cy"])) + lk
        mu, sd, depth = s["mg"]
        mg = 1.0 - depth * g2(LAM, mu, sd, sd)
        g = 0.90 * g2(LAM, *s["g"]) + lk
        filt = [ye, cy, mg, g]
    return np.stack(filt) * base[None, :]


# ----------------------------------------------------------------------------- cameras

# Neutral names (no trademarks): an era and a class. The style values are model assumptions set
# by analogy to how the makers of the time rendered (warm or cool white balance, saturation,
# contrast, memory-colour twists, how hard they sharpened and denoised), not measured.
# pixel: pixel pitch in um (full well ~ 2000 e-/um^2, read noise 10 e- for all, READ_NOISE in the plug-in).
# hue: (Oklab hue centre deg, half-width deg, hue shift deg, chroma factor).
CAMERAS = {
    "pocket2002": dict(label="Pocket 2002", sensor="cmyg", pixel=3.4, headroom=2.6, wb=(1.03, 1.0, 0.95), sat=0.95,
                       contrast=0.18, grey=0.47, black=0.004, hue=[(60, 30, 4, 1.0)],
                       sharpen=1.2, sharpen_r=1.0, nr=0.3, bloom=0.8, fringe=0.6),
    "prosumer2003": dict(label="Prosumer 2003", sensor="cmyg", pixel=3.4, headroom=3.0, wb=(1.01, 1.0, 0.99), sat=1.0,
                         contrast=0.12, grey=0.47, black=0.003, hue=[],
                         sharpen=0.8, sharpen_r=1.0, nr=0.3, bloom=0.5, fringe=0.4),
    "compact2004": dict(label="Compact 2004", sensor="rgb_early", pixel=2.7, headroom=3.0, wb=(1.03, 1.0, 0.97), sat=1.1,
                        contrast=0.2, grey=0.47, black=0.004, hue=[(55, 25, 3, 1.05), (25, 18, 0, 1.12)],
                        sharpen=1.0, sharpen_r=1.0, nr=0.4, bloom=0.45, fringe=0.5),
    "compact2005": dict(label="Compact 2005", sensor="rgb_mid", pixel=2.3, headroom=2.8, wb=(0.98, 1.0, 1.03), sat=1.15,
                        contrast=0.3, grey=0.46, black=0.006, hue=[(345, 30, -6, 1.08), (250, 30, 4, 1.05)],
                        sharpen=1.4, sharpen_r=0.9, nr=0.5, bloom=0.45, fringe=0.6),
    "compact2006": dict(label="Compact 2006", sensor="rgb_mid", pixel=2.8, headroom=3.3, wb=(1.0, 1.0, 1.0), sat=1.25,
                        contrast=0.25, grey=0.47, black=0.004, hue=[(140, 30, -5, 1.12), (245, 30, 6, 1.1)],
                        sharpen=0.9, sharpen_r=1.0, nr=0.5, bloom=0.4, fringe=0.4),
    "compact2007": dict(label="Compact 2007", sensor="rgb_mid", pixel=2.2, headroom=2.9, wb=(1.0, 1.0, 0.99), sat=1.0,
                        contrast=0.15, grey=0.47, black=0.003, hue=[(110, 30, 4, 1.0)],
                        sharpen=1.1, sharpen_r=1.0, nr=0.6, bloom=0.4, fringe=0.5),
    "ultracompact2008": dict(label="Ultracompact 2008", sensor="rgb_late", pixel=1.75, headroom=2.7, wb=(1.0, 1.0, 1.01), sat=1.15,
                             contrast=0.3, grey=0.46, black=0.007, hue=[(250, 30, 5, 1.08)],
                             sharpen=1.5, sharpen_r=0.9, nr=1.0, bloom=0.35, fringe=1.0),
    "prosumer2009": dict(label="Prosumer 2009", sensor="rgb_late", pixel=2.0, headroom=3.2, wb=(1.01, 1.0, 0.99), sat=1.05,
                         contrast=0.2, grey=0.47, black=0.003, hue=[(55, 25, 2, 1.0)],
                         sharpen=0.8, sharpen_r=1.0, nr=0.5, bloom=0.3, fringe=0.4),
}


def training_colours():
    """sRGB colours the colour matrix is fitted on: an encoded 7^3 grid plus extra neutrals."""
    v = srgb_decode(np.linspace(0.08, 0.92, 7))
    rr, gg, bb = np.meshgrid(v, v, v, indexing="ij")
    rgb = np.stack([rr.ravel(), gg.ravel(), bb.ravel()], 1)
    grey = np.repeat(srgb_decode(np.linspace(0.1, 0.9, 9))[:, None], 3, 1)
    return np.concatenate([rgb, grey, grey])


def fit_matrix(raw, target):
    """3 x K matrix from white-balanced raw to linear sRGB, least squares, rows summing to 1 (so
    a white-balanced grey stays grey)."""
    k = raw.shape[1]
    m = np.zeros((3, k))
    a = raw[:, :-1] - raw[:, -1:]
    for c in range(3):
        x, *_ = np.linalg.lstsq(a, target[:, c] - raw[:, -1], rcond=None)
        m[c, :-1] = x
        m[c, -1] = 1.0 - x.sum()
    return m


class Camera:
    def __init__(self, key):
        self.key, self.cfg = key, CAMERAS[key]
        self.sens = sensitivities(self.cfg["sensor"])
        h = self.cfg["headroom"]
        grey_raw = self._raw_unscaled(np.full((1, 3), 0.18))[0]
        self.exposure = 2.0 ** -h / grey_raw.max()  # grey at 2^-h of full well in the hottest channel
        grey_raw = grey_raw * self.exposure
        self.wb = 2.0 ** -h / grey_raw  # grey balanced in every raw channel
        self.bias = np.array(self.cfg["wb"])  # the maker's warm/cool lean, on the output RGB
        # colour matrix fitted without clipping, on colours well below it
        cols = training_colours()
        raw = self._raw_unscaled(cols * 0.18 / 0.5) * self.exposure * self.wb[None, :]
        self.matrix = fit_matrix(raw, cols * 0.18 / 0.5 * 2.0 ** -h / 0.18)
        # scene (linear sRGB, grey 0.18) -> raw, linearised over the training colours
        s, *_ = np.linalg.lstsq(cols, self._raw_unscaled(cols) * self.exposure, rcond=None)
        self.scene_to_raw = s.T  # K x 3
        self._fit_tone()

    def _raw_unscaled(self, rgb_lin):
        return (UP.spectra(rgb_lin) @ self.sens.T) * DL

    def raw(self, rgb_lin):
        """Raw signal (fraction of full well) per channel, unclipped."""
        return self._raw_unscaled(rgb_lin) * self.exposure

    # tone ---------------------------------------------------------------------
    KNEE = 3.0  # sharpness of the highlight knee: digital, so short

    def _fit_tone(self):
        """Tone curve over linear output x (grey at 2^-h, clip at 1): a power with a knee,
        normalised to reach exactly 1 at clip (the digital highlight). Grey lands within a few
        hundredths of the target."""
        h, g = self.cfg["headroom"], float(srgb_decode(np.array(self.cfg["grey"])))
        n = self.KNEE

        def shape(s):
            t = g * 2.0**s
            return t / (1.0 + t**n) ** (1.0 / n)

        top = shape(h)
        self._shape = lambda s: shape(s) / top

    def tone(self, x):
        h = self.cfg["headroom"]
        b = self.cfg["black"]
        x = np.clip(x, 0.0, 1.0)
        x = np.maximum(x - b * 2.0**-h, 0.0) / (1.0 - b * 2.0**-h)
        s = np.log2(np.maximum(x, 1e-9)) + h  # stops above grey
        ylin = np.where(x > 0, self._shape(s), 0.0)
        y = srgb_encode(np.clip(ylin, 0, 1))
        a = self.cfg["contrast"]
        return np.clip(y + a * y * (1 - y) * (2 * y - 1) * 2.0, 0, 1)

    # style --------------------------------------------------------------------
    def style(self, lin):
        lab = oklab(np.clip(lin, 0, None))
        c = np.hypot(lab[:, 1], lab[:, 2])
        h = np.degrees(np.arctan2(lab[:, 2], lab[:, 1])) % 360
        dh = np.zeros_like(h)
        cm = np.ones_like(h)
        for centre, half, shift, chroma in self.cfg["hue"]:
            d = (h - centre + 180) % 360 - 180
            w = np.clip(1 - np.abs(d) / half, 0, 1)
            w = w * w * (3 - 2 * w)
            dh += w * shift
            cm *= 1 + w * (chroma - 1)
        lim = 0.33
        c2 = c * cm * self.cfg["sat"]
        c2 = np.where(c2 > c, lim * np.tanh(c2 / lim) * np.maximum(c, 1e-9) / np.maximum(lim * np.tanh(c / lim), 1e-9), c2)
        hr = np.radians(h + dh)
        out = oklab_inv(np.stack([lab[:, 0], c2 * np.cos(hr), c2 * np.sin(hr)], 1))
        return np.clip(out, 0, 1)

    # the whole chain ------------------------------------------------------------
    def simulate(self, rgb_lin, chunk=20000):
        out = np.empty((len(rgb_lin), 3))
        for i in range(0, len(rgb_lin), chunk):
            raw = np.minimum(self.raw(rgb_lin[i : i + chunk]), 1.0)  # full well
            # after white balance every channel is clipped where the least amplified one saturates,
            # as the cameras did: a blown highlight turns white instead of a matrix-made hue
            wbd = np.minimum(raw * self.wb[None, :], self.wb.min())
            lin = np.clip(wbd @ self.matrix.T * self.bias[None, :], 0.0, 1.0)
            y = self.tone(lin)
            out[i : i + chunk] = self.style(srgb_decode(y))
        return out  # linear sRGB


# ----------------------------------------------------------------------------- LUT and Rust

LUT_N = 37
STOPS_LO, STOPS_HI = -8.0, 5.0  # must match the plug-in


def shaper_inv(u):
    return 0.18 * 2.0 ** (STOPS_LO + u * (STOPS_HI - STOPS_LO))


def bake(cam):
    v = shaper_inv(np.linspace(0, 1, LUT_N))
    bb, gg, rr = np.meshgrid(v, v, v, indexing="ij")
    return cam.simulate(np.stack([rr.ravel(), gg.ravel(), bb.ravel()], 1)).reshape(LUT_N, LUT_N, LUT_N, 3)


def write_lut(lut, path):
    """u16 little-endian, [b][g][r][rgb], sRGB-encoded."""
    np.round(srgb_encode(lut) * 65535).astype("<u2").tofile(path)


def rust_table(cams):
    """plugin/src/cameras.rs: per camera what the plug-in needs besides its LUT."""
    f = lambda v: f"{float(v):.6e}"
    rows = []
    for key, cam in cams.items():
        cfg = cam.cfg
        k = cam.sens.shape[0]
        s = np.zeros((4, 3))
        s[:k] = cam.scene_to_raw
        a = np.zeros((3, 4))
        a[:, :k] = cam.matrix * cam.wb[None, :] / (2.0 ** -cfg["headroom"] / 0.18)  # raw noise -> scene
        fw = 2000.0 * cfg["pixel"] ** 2  # full well, electrons
        rows.append(
            f'    Camera {{ name: "{cfg["label"]}", lut: include_lut!("{key}"), channels: {k},\n'
            f"        scene_to_raw: [{', '.join('[' + ', '.join(f(x) for x in r) + ']' for r in s)}],\n"
            f"        raw_to_scene: [{', '.join('[' + ', '.join(f(x) for x in r) + ']' for r in a)}],\n"
            f"        full_well: {fw:.1f}, sharpen: {cfg['sharpen']}, sharpen_radius: {cfg['sharpen_r']},\n"
            f"        nr: {cfg['nr']}, bloom: {cfg['bloom']}, fringe: {cfg['fringe']} }},"
        )
    body = "\n".join(rows)
    return f"""// Generated by model/digicam_model.py; do not edit by hand.
// Per camera: its LUT, the scene->raw matrix (raw = fraction of full well at the camera's
// exposure; rows beyond `channels` are zero), the raw->scene matrix that carries raw noise into
// the scene the LUT sees (white balance and colour matrix included), the pixel's full well in
// electrons at base ISO, and the style of the spatial stages.

pub const CAMERAS: [Camera; {len(cams)}] = [
{body}
];
"""


def report(cam):
    stops = np.arange(-7, 5)
    out = srgb_encode(cam.simulate(np.repeat((0.18 * 2.0**stops)[:, None], 3, 1)))
    print(f"{cam.key}: wb {np.round(cam.wb, 3)}, matrix rows\n{np.round(cam.matrix, 3)}")
    print("  grey scale (sRGB):", " ".join(f"{s:+d}:{o[1]:.2f}" for s, o in zip(stops, out)))


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    keys = [a for a in sys.argv[1:] if not a.startswith("--")] or list(CAMERAS)
    cams = {}
    for key in CAMERAS:
        cams[key] = Camera(key)
    for key in keys:
        report(cams[key])
        write_lut(bake(cams[key]), os.path.join(OUT, f"{key}.lut"))
        print("wrote", key, flush=True)
    with open(RUST, "w") as fh:
        fh.write(rust_table(cams))
    print("wrote", RUST)
