"""Static data for the plug-ins (so no instance has to compute it per band):

  lut/tables.bin    little-endian f32 (the film plug-in):
                      u8base[256]   shaper coordinate of each 8-bit sRGB level (exposure 0)
                      odec[4097]    sRGB -> linear (v = i/4096)
                      enc[4097]     linear -> sRGB (v = i/4096)
                      log2m[257]    log2(1 + i/256), for log2 from a float's exponent and mantissa
                      u16base[65536] shaper coordinate of each 16-bit sRGB level (exposure 0)
  lut/adobe.bin     little-endian f32: adec[4097] (Adobe RGB (1998) -> linear, v = i/4096),
                    aenc[4097] (linear -> Adobe RGB); gamma 563/256
  lut/noise1024.bin 1024 x 1024 x 4 signed bytes: white Gaussian sensor noise (unit spread = 32)

Run:  python3 make_tables.py
"""

import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "lut")
STOPS_LO, STOPS_HI = -8.0, 5.0  # must match digicam_model.py and the plug-in
GRAIN_SCALE = 32.0


def srgb_decode(v):
    return np.where(v <= 0.04045, v / 12.92, ((v + 0.055) / 1.055) ** 2.4)


def srgb_encode(v):
    v = np.clip(v, 0, 1)
    return np.where(v <= 0.0031308, v * 12.92, 1.055 * v ** (1 / 2.4) - 0.055)


def shaper(lin):
    return (np.log2(np.maximum(lin, 1e-12) / 0.18) - STOPS_LO) / (STOPS_HI - STOPS_LO)


def tables():
    v8 = np.arange(256) / 255.0
    v = np.arange(4097) / 4096.0
    srgb = [srgb_decode(v), srgb_encode(v)]
    parts = [shaper(srgb_decode(v8))] + srgb + [np.log2(1 + np.arange(257) / 256), shaper(srgb_decode(np.arange(65536) / 65535.0))]
    data = np.concatenate(parts).astype("<f4")
    path = os.path.join(OUT, "tables.bin")
    data.tofile(path)
    return path, data.size


def adobe():
    v = np.arange(4097) / 4096.0
    g = 563.0 / 256.0
    data = np.concatenate([v**g, v ** (1 / g)]).astype("<f4")
    path = os.path.join(OUT, "adobe.bin")
    data.tofile(path)
    return path


def noise():
    """White Gaussian noise for the sensor: 1024 x 1024 x 4 signed bytes (one independent value
    per raw channel, up to four for CMYG), unit spread = 32. Sensor noise is per pixel, so
    unlike film grain it has no clumping; the tile wraps seamlessly by construction."""
    rng = np.random.default_rng(2002)
    n = rng.standard_normal((1024, 1024, 4))
    n /= n.std()
    q = np.clip(np.round(n * GRAIN_SCALE), -127, 127).astype(np.int8)
    path = os.path.join(OUT, "noise1024.bin")
    q.tofile(path)  # [y][x][channel]
    return path


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    p, n = tables()
    print("wrote", p, n, "floats")
    print("wrote", adobe())
    print("wrote", noise())
