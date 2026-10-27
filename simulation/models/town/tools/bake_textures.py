#!/usr/bin/env python3
"""Bake photorealistic composite textures for the vo_town Gazebo world.

Sources: CC0 photo textures from ambientCG (extracted in ./ext).
Outputs: JPEG albedo + normal maps into models/vo_town/<model>/textures/.
"""
import math
import os
import random

import numpy as np
from PIL import Image, ImageDraw, ImageEnhance, ImageFilter

EXT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ext")
MODELS = "/mnt/disk2/sprind/baza/src/simulation_tools/models/vo_town"

FLAT_NORMAL = (128, 128, 255)


def load(asset, kind="Color"):
    return Image.open(f"{EXT}/{asset}/{asset}_1K-JPG_{kind}.jpg").convert("RGB")


def tile(img, out_w, out_h, px_per_m, phys_m):
    """Tile a photo (covering phys_m meters) to an out_w x out_h canvas."""
    tw = max(8, int(round(px_per_m * phys_m)))
    t = img.resize((tw, tw), Image.LANCZOS)
    canvas = Image.new("RGB", (out_w, out_h))
    for y in range(0, out_h, tw):
        for x in range(0, out_w, tw):
            canvas.paste(t, (x, y))
    return canvas


def brightness_variation(img, seed, strength=0.16, cells=9):
    """Low-frequency multiplicative brightness noise to break tiling repetition."""
    rng = np.random.default_rng(seed)
    small = rng.uniform(1.0 - strength, 1.0 + strength, (cells, cells)).astype(np.float32)
    field = Image.fromarray((small * 127).astype(np.uint8)).resize(img.size, Image.BICUBIC)
    field = field.filter(ImageFilter.GaussianBlur(img.size[0] // 20))
    arr = np.asarray(img).astype(np.float32) * (np.asarray(field)[..., None] / 127.0)
    return Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))


def add_stains(img, seed, n, px_per_m, max_r_m=1.6, dark=0.55):
    rng = random.Random(seed)
    overlay = Image.new("L", img.size, 0)
    d = ImageDraw.Draw(overlay)
    for _ in range(n):
        r = rng.uniform(0.4, max_r_m) * px_per_m
        x, y = rng.uniform(0, img.size[0]), rng.uniform(0, img.size[1])
        d.ellipse([x - r, y - r * rng.uniform(0.5, 1), x + r, y + r * rng.uniform(0.5, 1)],
                  fill=int(255 * rng.uniform(0.25, 0.6)))
    overlay = overlay.filter(ImageFilter.GaussianBlur(px_per_m * 0.35))
    black = Image.new("RGB", img.size, (int(30 * dark),) * 3)
    return Image.composite(black, img, overlay.point(lambda p: int(p * 0.6)))


def save(img, model, name, q=88, transpose=False, is_normal=False):
    """Save with a globally unique filename (ogre2 caches textures by URI basename).

    transpose=True remaps a texture baked width=X/height=Y to gz box-top UVs
    (image width maps to world Y). Normal maps get their R/G channels swapped
    to keep tangent directions consistent after the transpose.
    """
    if transpose:
        img = img.transpose(Image.TRANSPOSE)
        if is_normal:
            r, g, b = img.split()
            img = Image.merge("RGB", (g, r, b))
    path = f"{MODELS}/{model}/textures"
    os.makedirs(path, exist_ok=True)
    img.save(f"{path}/{model}_{name}.jpg", quality=q)
    print(f"  {model}/textures/{model}_{name}.jpg  {img.size}")


def white_line(d, xy, wear=210):
    d.rectangle(xy, fill=(wear, wear, max(0, wear - 8)))


# ---------------------------------------------------------------- roads
def bake_road_straight(variant, seed):
    """8 m wide (image W) x 22 m long (image H). Lane lines run along H."""
    ppm = 48
    W, H = 8 * ppm, 22 * ppm
    img = tile(load("Asphalt026C"), W, H, ppm, 3.0)
    img = brightness_variation(img, seed, 0.10)
    img = add_stains(img, seed + 1, 10, ppm, 1.8)
    d = ImageDraw.Draw(img)
    cx = W // 2
    lw = int(0.14 * ppm)
    # solid edge lines at +/-3.3 m
    for ex in (cx - int(3.3 * ppm), cx + int(3.3 * ppm)):
        white_line(d, [ex - lw // 2, 0, ex + lw // 2, H], 195)
    # dashed center line: 3 m dash / 3.5 m gap
    y = -seed % 5 * ppm
    while y < H:
        white_line(d, [cx - lw // 2, max(0, y), cx + lw // 2, min(H, y + 3 * ppm)], 205)
        y += int(6.5 * ppm)
    rng = random.Random(seed + 2)
    # tar crack lines
    for _ in range(6):
        x0, y0 = rng.uniform(0, W), rng.uniform(0, H)
        pts = [(x0, y0)]
        for _ in range(5):
            x0 += rng.uniform(-ppm, ppm); y0 += rng.uniform(0.4 * ppm, 2 * ppm)
            pts.append((x0, y0))
        d.line(pts, fill=(28, 27, 26), width=max(2, ppm // 16))
    if variant == "b":  # manhole cover
        mx, my, r = cx + int(1.7 * ppm), H // 3, int(0.4 * ppm)
        d.ellipse([mx - r, my - r, mx + r, my + r], fill=(52, 50, 48), outline=(25, 25, 25), width=3)
        d.ellipse([mx - r + 6, my - r + 6, mx + r - 6, my + r - 6], outline=(35, 34, 33), width=2)
    nrm = tile(load("Asphalt026C", "NormalGL"), W, H, ppm, 3.0)
    m = f"road_straight_{variant}"
    save(img, m, "road_color", transpose=True)
    save(nrm, m, "road_normal", q=80, transpose=True, is_normal=True)
    bake_sidewalk_strip(m, seed + 10)


def bake_sidewalk_strip(model, seed):
    """2.4 m (W) x 22 m (H) paving strip with curb edge."""
    ppm = 48
    W, H = int(2.4 * ppm), 22 * ppm
    img = tile(load("PavingStones128"), W, H, ppm, 2.0)
    img = brightness_variation(img, seed, 0.12)
    img = add_stains(img, seed + 1, 5, ppm, 0.9, dark=0.4)
    d = ImageDraw.Draw(img)
    d.rectangle([0, 0, int(0.12 * ppm), H], fill=(150, 148, 144))  # curb stone edge
    nrm = tile(load("PavingStones128", "NormalGL"), W, H, ppm, 2.0)
    save(img, model, "sidewalk_color", transpose=True)
    save(nrm, model, "sidewalk_normal", q=80, transpose=True, is_normal=True)


def bake_intersection():
    """8 x 8 m with zebra crosswalks on four sides + stop lines."""
    ppm = 48
    W = H = 8 * ppm
    img = tile(load("Asphalt026C"), W, H, ppm, 3.0)
    img = brightness_variation(img, 77, 0.10)
    img = add_stains(img, 78, 6, ppm, 1.4)
    d = ImageDraw.Draw(img)
    band0, band1 = int(0.6 * ppm), int(2.6 * ppm)  # crosswalk band from edge
    sw, gap = int(0.45 * ppm), int(0.45 * ppm)
    for k in range(4):  # 0=N,1=S,2=W,3=E
        p = int(2.9 * ppm)
        while p < W - int(2.9 * ppm):
            if k == 0:
                d.rectangle([p, band0, p + sw, band1], fill=(200, 200, 194))
            elif k == 1:
                d.rectangle([p, H - band1, p + sw, H - band0], fill=(196, 196, 190))
            elif k == 2:
                d.rectangle([band0, p, band1, p + sw], fill=(203, 203, 197))
            else:
                d.rectangle([W - band1, p, W - band0, p + sw], fill=(198, 198, 192))
            p += sw + gap
    nrm = tile(load("Asphalt026C", "NormalGL"), W, H, ppm, 3.0)
    save(img, "road_intersection", "intersection_color", transpose=True)
    save(nrm, "road_intersection", "intersection_normal", q=80, transpose=True, is_normal=True)
    # 2.4 m corner pads
    pad = tile(load("PavingStones128"), int(2.4 * ppm), int(2.4 * ppm), ppm, 2.0)
    pad = brightness_variation(pad, 5, 0.1)
    save(pad, "road_intersection", "pad_color")


def bake_parking():
    """12 m (W) x 18 m (H) parking lot, two rows of stalls."""
    ppm = 48
    W, H = 12 * ppm, 18 * ppm
    img = tile(load("Asphalt026C"), W, H, ppm, 3.0)
    img = brightness_variation(img, 33, 0.12)
    img = add_stains(img, 34, 12, ppm, 1.2)
    d = ImageDraw.Draw(img)
    lw = max(3, int(0.1 * ppm))
    for rx in (0, W - int(5 * ppm)):  # two stall rows: left & right, stalls 2.6 m pitch
        y = int(0.8 * ppm)
        while y < H - int(0.8 * ppm):
            white_line(d, [rx, y, rx + int(5 * ppm), y + lw], 185)
            y += int(2.6 * ppm)
        white_line(d, [rx + (int(5 * ppm) if rx == 0 else 0) - lw // 2, int(0.8 * ppm),
                       rx + (int(5 * ppm) if rx == 0 else 0) + lw // 2, y - int(2.6 * ppm) + lw], 185)
    nrm = tile(load("Asphalt026C", "NormalGL"), W, H, ppm, 3.0)
    save(img, "parking_lot", "parking_color", transpose=True)
    save(nrm, "parking_lot", "parking_normal", q=80, transpose=True, is_normal=True)


# ---------------------------------------------------------------- ground
def bake_ground(variant, seed):
    """100 x 100 m grass quadrant, 3072 px (~3 cm/px)."""
    S, ppm = 3072, 3072 / 100.0
    img = tile(load("Grass004"), S, S, ppm, 2.5)
    img = brightness_variation(img, seed, 0.20, cells=13)
    rng = random.Random(seed)
    dirt = load("Ground037")
    # organic dirt/bare patches
    for _ in range(24):
        r = int(rng.uniform(1.5, 7) * ppm)
        patch = dirt.resize((2 * r, 2 * r), Image.LANCZOS)
        mask = Image.new("L", (2 * r, 2 * r), 0)
        md = ImageDraw.Draw(mask)
        md.ellipse([r * 0.1, r * 0.25, r * 1.9, r * 1.75], fill=235)
        mask = mask.filter(ImageFilter.GaussianBlur(r * 0.12))
        x, y = rng.randint(0, S - 2 * r), rng.randint(0, S - 2 * r)
        img.paste(patch, (x, y), mask)
    # faint mowing stripes
    stripes = Image.new("L", (S, S), 127)
    sd = ImageDraw.Draw(stripes)
    sw = int(rng.uniform(4, 7) * ppm)
    for i, x in enumerate(range(0, S, sw)):
        sd.rectangle([x, 0, x + sw, S], fill=127 + (6 if i % 2 else -6))
    ang = rng.choice([0, 90, 45, -45])
    stripes = stripes.rotate(ang, expand=False, fillcolor=127)
    arr = np.asarray(img).astype(np.float32) * (np.asarray(stripes)[..., None] / 127.0)
    img = Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))
    # worn footpath across the quadrant
    path = Image.new("L", (S, S), 0)
    pd = ImageDraw.Draw(path)
    x0, y0 = rng.randint(0, S), 0
    pts = [(x0, y0)]
    while pts[-1][1] < S:
        pts.append((pts[-1][0] + rng.randint(-150, 150), pts[-1][1] + rng.randint(200, 400)))
    pd.line(pts, fill=160, width=int(0.8 * ppm))
    path = path.filter(ImageFilter.GaussianBlur(6))
    dirt_full = tile(dirt, S, S, ppm, 2.0)
    img = Image.composite(dirt_full, img, path)
    # small weed/clover spots + per-pixel speckle so nearby features stay unique
    d = ImageDraw.Draw(img, "RGBA")
    for _ in range(140):
        r = rng.uniform(0.25, 1.1) * ppm
        x, y = rng.uniform(0, S), rng.uniform(0, S)
        shade = rng.choice([(40, 60, 25, 90), (120, 130, 60, 80), (90, 70, 40, 95),
                            (55, 85, 35, 90), (150, 145, 120, 70)])
        d.ellipse([x - r, y - r * rng.uniform(0.5, 1), x + r, y + r], fill=shade)
    arr = np.asarray(img).astype(np.float32)
    arr += np.random.default_rng(seed).normal(0, 7.5, arr.shape)
    img = Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))
    m = f"ground_{variant}"
    save(img, m, "ground_color", q=85)
    nrm = tile(load("Grass004", "NormalGL"), S, S, ppm, 2.5)
    save(nrm, m, "ground_normal", q=75)


# ---------------------------------------------------------------- facades
GLASS_TINTS = [(96, 118, 138), (82, 104, 126), (110, 126, 140), (70, 88, 108),
               (120, 132, 138), (88, 96, 104), (60, 80, 100), (130, 140, 148)]


def draw_window(d, x0, y0, x1, y1, rng, frame=(228, 226, 220)):
    d.rectangle([x0 - 3, y0 - 3, x1 + 3, y1 + 3], fill=frame)
    base = rng.choice(GLASS_TINTS)
    if rng.random() < 0.25:  # curtains / blinds
        base = tuple(min(255, c + 70) for c in base)
    h = y1 - y0
    for i in range(4):  # vertical sky-reflection gradient
        f = 1.15 - 0.11 * i
        d.rectangle([x0, y0 + i * h // 4, x1, y0 + (i + 1) * h // 4],
                    fill=tuple(int(min(255, c * f)) for c in base))
    mx = (x0 + x1) // 2
    d.line([mx, y0, mx, y1], fill=frame, width=2)
    d.rectangle([x0 - 4, y1 + 3, x1 + 4, y1 + 7], fill=(150, 148, 142))  # sill


def bake_house_facade(model, base_asset, phys_w, phys_h, floors, cols, seed,
                      tint=None, door_col=(72, 52, 38)):
    ppm = 72
    W, H = int(phys_w * ppm), int(phys_h * ppm)
    img = tile(load(base_asset), W, H, ppm, 2.0)
    if tint:
        arr = np.asarray(img).astype(np.float32)
        arr = arr * 0.55 + np.array(tint, np.float32) * 0.45
        img = Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))
    img = brightness_variation(img, seed, 0.08)
    d = ImageDraw.Draw(img)
    rng = random.Random(seed)
    ww, wh = int(1.1 * ppm), int(1.35 * ppm)
    floor_h = H / floors
    for f in range(floors):
        wy = H - int((f + 0.72) * floor_h)
        slots = list(range(cols))
        for c in slots:
            wx = int((c + 0.5) * W / cols - ww / 2)
            if f == 0 and c == cols // 2:  # door
                d.rectangle([wx - 4, wy - int(0.35 * ppm) - 4, wx + ww + 4, H], fill=(210, 206, 198))
                d.rectangle([wx, wy - int(0.35 * ppm), wx + ww, H - 4], fill=door_col)
                d.line([wx + ww // 2, wy - int(0.35 * ppm), wx + ww // 2, H - 4],
                       fill=(30, 24, 18), width=2)
                continue
            draw_window(d, wx, wy, wx + ww, wy + wh, rng)
    # grime at base, soffit shadow at top
    d.rectangle([0, H - int(0.12 * ppm), W, H], fill=(90, 86, 80))
    sh = np.asarray(img).astype(np.float32)
    fade = np.linspace(0.7, 1.0, int(0.3 * ppm))[:, None, None]
    sh[: int(0.3 * ppm)] *= fade
    img = Image.fromarray(np.clip(sh, 0, 255).astype(np.uint8))
    save(img, model, "facade_color")
    nrm = tile(load(base_asset, "NormalGL"), W, H, ppm, 2.0)
    save(nrm, model, "facade_normal", q=80)


def bake_photo_facade(model, asset, crop=1.0, reps_x=1, reps_y=1, out_w=1024, out_h=1024):
    img = load(asset)
    if crop < 1.0:
        s = int(img.size[0] * crop)
        img = img.crop((0, img.size[1] - s, s, img.size[1]))
    cell_w, cell_h = out_w // reps_x, out_h // reps_y
    cell = img.resize((cell_w, cell_h), Image.LANCZOS)
    canvas = Image.new("RGB", (out_w, out_h))
    for y in range(reps_y):
        for x in range(reps_x):
            canvas.paste(cell, (x * cell_w, y * cell_h))
    save(canvas, model, "facade_color")
    n = load(asset, "NormalGL")
    if crop < 1.0:
        s = int(n.size[0] * crop)
        n = n.crop((0, n.size[1] - s, s, n.size[1]))
    ncell = n.resize((cell_w, cell_h), Image.LANCZOS)
    ncan = Image.new("RGB", (out_w, out_h))
    for y in range(reps_y):
        for x in range(reps_x):
            ncan.paste(ncell, (x * cell_w, y * cell_h))
    save(ncan, model, "facade_normal", q=80)


def bake_storefront(model, phys_w, phys_h, seed):
    """Shop row: brick upper floor, glass storefronts + awnings below."""
    ppm = 72
    W, H = int(phys_w * ppm), int(phys_h * ppm)
    img = tile(load("Bricks090"), W, H, ppm, 2.0)
    img = brightness_variation(img, seed, 0.07)
    d = ImageDraw.Draw(img)
    rng = random.Random(seed)
    n_shops = max(2, int(phys_w // 5))
    shop_w = W / n_shops
    awning_cols = [(160, 44, 40), (32, 84, 130), (36, 110, 60), (150, 110, 30), (90, 60, 120)]
    gh0, gh1 = H - int(2.6 * ppm), H - int(0.15 * ppm)
    for s in range(n_shops):
        x0, x1 = int(s * shop_w + 0.3 * ppm), int((s + 1) * shop_w - 0.3 * ppm)
        d.rectangle([x0 - 5, gh0 - 5, x1 + 5, gh1], fill=(60, 58, 54))
        for i in range(4):
            f = 1.2 - 0.14 * i
            gg = tuple(int(min(255, c * f)) for c in (76, 96, 112))
            d.rectangle([x0, gh0 + i * (gh1 - gh0) // 4, x1, gh0 + (i + 1) * (gh1 - gh0) // 4], fill=gg)
        for mx in range(x0 + (x1 - x0) // 3, x1, (x1 - x0) // 3):
            d.line([mx, gh0, mx, gh1], fill=(50, 48, 45), width=3)
        ac = awning_cols[(s + seed) % len(awning_cols)]
        ay = gh0 - int(0.55 * ppm)
        for st in range(x0 - 8, x1 + 8, int(0.35 * ppm)):  # striped awning
            c = ac if (st // int(0.35 * ppm)) % 2 == 0 else tuple(min(255, v + 80) for v in ac)
            d.rectangle([st, ay, st + int(0.35 * ppm), gh0 - 6], fill=c)
        # sign band
        d.rectangle([x0, ay - int(0.5 * ppm), x1, ay - 4], fill=tuple(int(v * 0.5) for v in ac))
        for tx in range(x0 + 10, x1 - 20, int(0.5 * ppm)):  # fake lettering
            d.rectangle([tx, ay - int(0.4 * ppm), tx + int(0.28 * ppm), ay - int(0.14 * ppm)],
                        fill=(235, 232, 225))
    # upper floor windows
    ww, wh = int(1.0 * ppm), int(1.2 * ppm)
    wy = int(0.5 * ppm)
    for c in range(n_shops * 2):
        wx = int((c + 0.5) * W / (n_shops * 2) - ww / 2)
        draw_window(d, wx, wy, wx + ww, wy + wh, rng)
    save(img, model, "facade_color")
    nrm = tile(load("Bricks090", "NormalGL"), W, H, ppm, 2.0)
    save(nrm, model, "facade_normal", q=80)


def bake_warehouse_facade(model, phys_w, phys_h, seed):
    ppm = 64
    W, H = int(phys_w * ppm), int(phys_h * ppm)
    img = tile(load("CorrugatedSteel005"), W, H, ppm, 2.0)
    img = brightness_variation(img, seed, 0.09)
    d = ImageDraw.Draw(img)
    # two big roller doors + person door + high windows
    for dx in (0.22, 0.62):
        x0, y0 = int(dx * W), H - int(4.2 * ppm)
        x1, y1 = x0 + int(4.5 * ppm), H - int(0.1 * ppm)
        d.rectangle([x0 - 6, y0 - 6, x1 + 6, y1], fill=(70, 72, 76))
        for yy in range(y0, y1, int(0.35 * ppm)):
            d.rectangle([x0, yy, x1, yy + int(0.3 * ppm)], fill=(120, 124, 130))
    x0 = int(0.88 * W)
    d.rectangle([x0, H - int(2.2 * ppm), x0 + int(1.0 * ppm), H - int(0.1 * ppm)], fill=(60, 70, 90))
    for c in range(6):
        wx = int((c + 0.5) * W / 6 - 0.9 * ppm / 2)
        d.rectangle([wx, int(0.6 * ppm), wx + int(0.9 * ppm), int(1.3 * ppm)], fill=(96, 116, 134))
    d.rectangle([0, H - int(0.5 * ppm), W, H], fill=(88, 86, 82))  # concrete plinth
    save(img, model, "facade_color")
    nrm = tile(load("CorrugatedSteel005", "NormalGL"), W, H, ppm, 2.0)
    save(nrm, model, "facade_normal", q=80)


def bake_civic_facade(model, phys_w, phys_h, seed):
    ppm = 72
    W, H = int(phys_w * ppm), int(phys_h * ppm)
    img = tile(load("Concrete034"), W, H, ppm, 2.0)
    arr = np.asarray(img).astype(np.float32) * np.array([1.06, 1.02, 0.95])
    img = Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))
    d = ImageDraw.Draw(img)
    rng = random.Random(seed)
    ww, wh = int(1.3 * ppm), int(2.6 * ppm)  # tall arched-ish windows
    for c in range(int(phys_w // 2.6)):
        n = int(phys_w // 2.6)
        wx = int((c + 0.5) * W / n - ww / 2)
        wy = int(0.18 * H)
        draw_window(d, wx, wy, wx + ww, wy + wh, rng, frame=(200, 196, 186))
        d.rectangle([wx - 8, wy + wh + 10, wx + ww + 8, wy + wh + 16], fill=(165, 160, 150))
    # cornice + banner
    d.rectangle([0, int(0.06 * H), W, int(0.06 * H) + int(0.12 * ppm)], fill=(170, 165, 155))
    d.rectangle([int(0.4 * W), int(0.62 * H), int(0.6 * W), H - int(0.05 * ppm)], fill=(120, 30, 30))
    save(img, model, "facade_color")
    nrm = tile(load("Concrete034", "NormalGL"), W, H, ppm, 2.0)
    save(nrm, model, "facade_normal", q=80)


# ---------------------------------------------------------------- roofs
def bake_flat_roof(model, phys_w, phys_l, seed, helipad=False, name="roof_color"):
    ppm = 40
    W, H = int(phys_w * ppm), int(phys_l * ppm)
    img = tile(load("Concrete034"), W, H, ppm, 2.0)
    arr = np.asarray(img).astype(np.float32) * 0.82  # weathered gray
    img = Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))
    img = brightness_variation(img, seed, 0.14)
    img = add_stains(img, seed + 1, 8, ppm, 2.0, dark=0.5)
    d = ImageDraw.Draw(img)
    rng = random.Random(seed)
    b = int(0.5 * ppm)  # parapet border
    d.rectangle([0, 0, W, b], fill=(148, 144, 138)); d.rectangle([0, H - b, W, H], fill=(148, 144, 138))
    d.rectangle([0, 0, b, H], fill=(148, 144, 138)); d.rectangle([W - b, 0, W, H], fill=(148, 144, 138))
    for _ in range(rng.randint(2, 4)):  # AC units with shadow
        aw, al = int(rng.uniform(1.5, 2.5) * ppm), int(rng.uniform(2, 3.5) * ppm)
        for _try in range(20):
            x, y = rng.randint(b * 2, max(b * 2 + 1, W - aw - b * 2)), rng.randint(b * 2, max(b * 2 + 1, H - al - b * 2))
            if not helipad or math.hypot(x + aw / 2 - W / 2, y + al / 2 - H / 2) > min(W, H) * 0.42:
                break
        d.rectangle([x + 6, y + 8, x + aw + 6, y + al + 8], fill=(60, 60, 58))
        d.rectangle([x, y, x + aw, y + al], fill=(176, 178, 180), outline=(110, 112, 114), width=3)
        for fx in (0.3, 0.7):
            cxx, cyy, r = x + aw * fx, y + al * 0.5, aw * 0.22
            d.ellipse([cxx - r, cyy - r, cxx + r, cyy + r], fill=(90, 92, 94), outline=(60, 60, 60), width=2)
    for _ in range(rng.randint(3, 6)):  # vents & pipes
        for _try in range(20):
            x, y = rng.randint(b * 2, W - b * 2), rng.randint(b * 2, H - b * 2)
            if not helipad or math.hypot(x - W / 2, y - H / 2) > min(W, H) * 0.42:
                break
        r = int(rng.uniform(0.15, 0.3) * ppm)
        d.ellipse([x - r, y - r, x + r, y + r], fill=(70, 68, 66), outline=(40, 40, 40), width=2)
    if helipad:
        cx, cy, r = W // 2, H // 2, int(min(W, H) * 0.32)
        d.ellipse([cx - r, cy - r, cx + r, cy + r], outline=(220, 190, 40), width=int(0.25 * ppm))
        hw, hh, t = int(r * 0.5), int(r * 0.62), int(0.22 * ppm)
        d.rectangle([cx - hw, cy - hh, cx - hw + t, cy + hh], fill=(220, 190, 40))
        d.rectangle([cx + hw - t, cy - hh, cx + hw, cy + hh], fill=(220, 190, 40))
        d.rectangle([cx - hw, cy - t // 2, cx + hw, cy + t // 2], fill=(220, 190, 40))
    save(img, model, name, transpose=True)


def bake_tile_roof(model, phys_w, phys_l, seed, darken=1.0):
    """Fake-pitch tiled roof: ridge along the long (H) axis + slope shading."""
    ppm = 40
    W, H = int(phys_w * ppm), int(phys_l * ppm)
    img = tile(load("RoofingTiles003"), W, H, ppm, 1.6)
    arr = np.asarray(img).astype(np.float32) * darken
    # slope shading: sunlit west half, shaded east half
    shade = np.concatenate([np.linspace(1.12, 0.98, W // 2), np.linspace(0.86, 0.72, W - W // 2)])
    arr *= shade[None, :, None]
    img = Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))
    d = ImageDraw.Draw(img)
    d.rectangle([W // 2 - int(0.1 * ppm), 0, W // 2 + int(0.1 * ppm), H], fill=(96, 60, 46))  # ridge caps
    for y in range(0, H, int(0.45 * ppm)):
        d.ellipse([W // 2 - int(0.09 * ppm), y, W // 2 + int(0.09 * ppm), y + int(0.4 * ppm)],
                  outline=(70, 44, 34), width=2)
    rng = random.Random(seed)
    if rng.random() < 0.8:  # chimney
        cx, cy = int(W * rng.uniform(0.6, 0.8)), int(H * rng.uniform(0.2, 0.8))
        cw = int(0.5 * ppm)
        d.rectangle([cx + 4, cy + 6, cx + cw + 4, cy + cw + 6], fill=(50, 40, 34))
        d.rectangle([cx, cy, cx + cw, cy + cw], fill=(140, 90, 70), outline=(90, 55, 42), width=3)
    save(img, model, "roof_color", transpose=True)
    nrm = tile(load("RoofingTiles003", "NormalGL"), W, H, ppm, 1.6)
    save(nrm, model, "roof_normal", q=80, transpose=True, is_normal=True)


def bake_metal_roof(model, phys_w, phys_l, seed):
    ppm = 40
    W, H = int(phys_w * ppm), int(phys_l * ppm)
    img = tile(load("MetalPlates006"), W, H, ppm, 2.0)
    img = brightness_variation(img, seed, 0.15)
    img = add_stains(img, seed + 1, 10, ppm, 2.5, dark=0.6)  # rust-ish stains
    d = ImageDraw.Draw(img)
    for x in range(0, W, int(1.0 * ppm)):  # standing seams
        d.line([x, 0, x, H], fill=(105, 106, 108), width=2)
    for c in range(3):  # skylight strips
        x = int((c + 0.5) * W / 3)
        d.rectangle([x - int(0.4 * ppm), int(0.15 * H), x + int(0.4 * ppm), int(0.85 * H)],
                    fill=(150, 165, 175), outline=(90, 92, 94), width=3)
    save(img, model, "roof_color", transpose=True)


# ---------------------------------------------------------------- props
def bake_tree():
    img = load("Grass004").resize((512, 512), Image.LANCZOS)
    arr = np.asarray(img).astype(np.float32)
    arr = arr * np.array([0.55, 0.85, 0.45])  # deep leafy green
    rng = np.random.default_rng(7)
    arr *= rng.uniform(0.45, 1.35, (512, 512, 1))
    # leaf-clump scale variation
    clumps = rng.uniform(0.55, 1.45, (24, 24, 1)).astype(np.float32)
    clumps = np.asarray(Image.fromarray((clumps[..., 0] * 127).astype(np.uint8))
                        .resize((512, 512), Image.BICUBIC))[..., None] / 127.0
    arr *= clumps
    img = Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8)).filter(ImageFilter.GaussianBlur(0.8))
    img = ImageEnhance.Contrast(img).enhance(1.45)
    save(img, "tree", "canopy_color")
    bark = load("Bark006").resize((256, 512), Image.LANCZOS)
    save(bark, "tree", "bark_color")


def bake_signs():
    for model, bg, fg in [("street_sign", (18, 70, 150), (240, 240, 240)),
                          ("street_sign_yellow", (225, 180, 20), (30, 30, 30))]:
        img = Image.new("RGB", (256, 192), bg)
        d = ImageDraw.Draw(img)
        d.rectangle([6, 6, 249, 185], outline=fg, width=6)
        rng = random.Random(sum(bg))
        for r in range(3):  # fake text bars
            y = 34 + r * 48
            x = 24
            while x < 210:
                w = rng.randint(14, 42)
                d.rectangle([x, y, x + w, y + 24], fill=fg)
                x += w + 12
        save(img, model, "sign_color")


if __name__ == "__main__":
    os.makedirs(MODELS, exist_ok=True)
    print("== roads ==")
    bake_road_straight("a", 101)
    bake_road_straight("b", 202)
    bake_intersection()
    bake_parking()
    print("== ground ==")
    for v, s in zip("abcd", (11, 22, 33, 44)):
        bake_ground(v, s)
    print("== facades ==")
    bake_house_facade("house_brick_a", "Bricks059", 10, 5.5, 1, 4, seed=1)
    bake_house_facade("house_brick_b", "Bricks090", 9, 6.5, 2, 3, seed=2)
    bake_house_facade("house_plaster", "Plaster003", 10, 5.0, 1, 4, seed=3,
                      tint=(214, 196, 160))
    bake_house_facade("house_two_story", "Plaster003", 11, 7.5, 2, 4, seed=4,
                      tint=(196, 170, 150), door_col=(46, 60, 84))
    bake_storefront("shop_row", 20, 6.5, seed=5)
    bake_photo_facade("midrise_office", "Facade006", reps_x=2, reps_y=2)
    bake_photo_facade("midrise_dark", "Facade012", crop=0.55, reps_x=1, reps_y=1)
    bake_photo_facade("tower_glass", "Facade001", reps_x=2, reps_y=3)
    bake_warehouse_facade("warehouse", 24, 9, seed=6)
    bake_civic_facade("civic_hall", 16, 8, seed=7)
    print("== roofs ==")
    bake_tile_roof("house_brick_a", 8, 10, seed=11)
    bake_tile_roof("house_brick_b", 9, 9, seed=12, darken=0.75)
    bake_tile_roof("house_plaster", 8, 10, seed=13, darken=1.1)
    bake_tile_roof("house_two_story", 9, 11, seed=14, darken=0.6)
    bake_flat_roof("shop_row", 20, 8, seed=15)
    bake_flat_roof("midrise_office", 16, 20, seed=16)
    bake_flat_roof("midrise_dark", 14, 14, seed=17)
    bake_flat_roof("tower_glass", 18, 18, seed=18, helipad=True)
    bake_metal_roof("warehouse", 24, 18, seed=19)
    bake_flat_roof("civic_hall", 16, 12, seed=20)
    print("== props ==")
    bake_tree()
    bake_signs()
    print("done")
