import numpy as np

KT = [2, 7, 14, 18, 14, 7, 2]
M = (1 << 32) - 1

def lcg_image(w, h, nframes=2):
    st = 0xC0FFEE
    px = []
    for _ in range(nframes * w * h):
        st = (st * 1664525 + 1013904223) & M
        px.append((st >> 8) & 0xFF)
    return np.array(px, dtype=np.int64).reshape(nframes, h, w)

def hpass_exact(img):
    """Horizontal pass, unrounded: value = true_result * 64. BORDER_REPLICATE."""
    H, W = img.shape
    rep = np.pad(img, 3, mode='edge')
    he = np.zeros((H, W), dtype=np.int64)
    for i in range(H):
        acc = np.zeros(W, dtype=np.int64)
        for j in range(7):
            acc += KT[j] * rep[i + 3, j:j + W]
        he[i] = acc
    return he

def gauss(img, double_round=True):
    H, W = img.shape
    he = hpass_exact(img)
    h8 = (he + 32) >> 6                      # hardware: horizontal rounds to 8 bit
    src = h8 if double_round else he
    vr = np.pad(src, ((3, 3), (0, 0)), mode='edge')   # 只在行方向做上/下边界复制
    scale = 64 if double_round else 64 * 64  # single-round keeps 64x precision
    out = np.zeros((H, W), dtype=np.int64)
    for c in range(W):
        acc = np.zeros(H, dtype=np.int64)
        for j in range(7):
            acc += KT[j] * vr[j:j + H, c]
        out[:, c] = acc
    return (out + scale // 2) >> 6 if double_round else (out + scale // 2) // scale

for W, H, expect in [(128, 128, [169, 135, 105, 89, 88, 94, 107, 126, 146, 156]),
                     (64, 48, [149])]:
    img = lcg_image(W, H)[0]
    g2 = gauss(img, True)
    g1 = gauss(img, False)
    n = len(expect)
    print(f"[{W}x{H}] python double-round  row0 col0..{n-1}: {g2[0, :n].tolist()}")
    print(f"[{W}x{H}] ModelSim TB golden                   : {expect}")
    print(f"[{W}x{H}] MATCH: {g2[0, :n].tolist() == expect}")
    d = np.abs(g2 - g1)
    print(f"[{W}x{H}] single-round (OpenCV style) vs HW: diff px "
          f"{int((d > 0).sum())}/{W*H} ({100*(d > 0).mean():.2f}%), max |delta| = {int(d.max())}\n")
