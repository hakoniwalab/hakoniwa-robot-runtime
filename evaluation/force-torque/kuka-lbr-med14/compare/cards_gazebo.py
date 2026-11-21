"""Measurement cards: the KUKA real recording against Hakoniwa, Gazebo, and Gazebo + Hakoniwa's vibration model.

    python3 compare/cards_gazebo.py ja|en      (after data/fetch.py and compare/run_gazebo.py)

Hakoniwa: runs/harness-t14.npz (Sensor PRO with the fitted ATI Gamma noise and vibration T14, seeds 1-20).
Gazebo: compare/out/gazebo*, see run_gazebo.py.
Calculations follow the published cards (cards_kuka.py): 3 Hz high-pass, 0.5-5.5 s, Welch 512, 5-20 Hz, 0.3 s windows.
"""
import json
import sys
from pathlib import Path

import numpy as np
from scipy.signal import butter, sosfiltfilt, spectrogram, welch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import kukadata as K  # noqa: E402
import cards_common as cc  # noqa: E402
from cards_common import L, INK, MUTED, BODY, C1, C2, arrow, sketch_axes, notes, card, result_title, plt  # noqa: E402
from matplotlib.patches import FancyBboxPatch  # noqa: E402

cc.LANG[0] = sys.argv[1] if len(sys.argv) > 1 else "ja"
OUT = K.OUT / "cards"
OUT.mkdir(parents=True, exist_ok=True)
AX = K.AX
NAMES = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
REAL_C, HK_C, GZ_C = "#222222", "#1f6feb", "#e8710a"

tw, real = K.real()
hk = [(t, x) for t, x, _ in K.runs("harness-t14")]
gz = [(t, x) for t, x, _ in K.gazebo("gazebo")]
gzv = [(t, x) for t, x, _ in K.gazebo("gazebo-vibration")]
tq, q = K.joint_angle()
tr_rest, real_rest = K.real_rest()
hk_rest = [(t, x) for t, x, _ in K.runs("rest-harness-t14")]
gz_rest = [(t, x) for t, x, _ in K.gazebo("gazebo-rest")]
gzv_rest = [(t, x) for t, x, _ in K.gazebo("gazebo-vibration-rest")]


def lp(x, fs, fc, order=2, kind="low"):
    return sosfiltfilt(butter(order, fc, kind, fs=fs, output="sos"), x, axis=0)


def fs_of(t):
    return 1 / np.median(np.diff(t))


def env(t, x, k, win=0.3, step=0.15):
    fs = fs_of(t)
    y = lp(lp(x[:, k], fs, 3.0, 4, "high"), fs, [5, 20], 4, "band")
    c = np.arange(0.15, 5.75, step)
    return c, np.array([y[(t >= a - win / 2) & (t < a + win / 2)].std() for a in c])


def band(t, x, lo, hi, t0=0.5, t1=5.5):
    fs = fs_of(t)
    m = (t > t0) & (t < t1)
    y = lp(x[m] - x[m].mean(0), fs, 3.0, 4, "high")
    f, p = welch(y, fs=fs, nperseg=min(512, m.sum()), axis=0)
    s = (f >= lo) & (f < hi)
    return np.sqrt(p[s].sum(0) * (f[1] - f[0]))


def corr(t, x):
    cw, e = env(t, x, 2, 0.3, 0.3)
    m = (cw > 0.5) & (cw < 5.3)
    return np.corrcoef(e[m], np.abs(np.cos(np.interp(cw, tq, q)))[m])[0, 1]


def save(fig, name):
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    p = OUT / f"{name}-{cc.LANG[0]}.png"
    fig.savefig(p, dpi=110)
    plt.close(fig)
    print(p)


def box(ax, xy, w, h, text, fc, ec=INK, size=8.3, color=INK):
    ax.add_patch(FancyBboxPatch(xy, w, h, boxstyle="round,pad=0.008,rounding_size=0.02", fc=fc, ec=ec, lw=1.1))
    ax.text(xy[0] + w / 2, xy[1] + h / 2, text, ha="center", va="center", fontsize=size, color=color)


def pipeline(ax):
    """How the four columns are made: one truth, swapped sensor implementations."""
    sketch_axes(ax, xlim=(0, 1), ylim=(-0.62, 0.55), title=L("比べ方", "How it is compared"))
    box(ax, (0.03, 0.33), 0.94, 0.16, L("MuJoCo：KUKA iiwa 14 が実機と同じ角度で回る\n→ 手首の真の力（雑音なし）を 1 回だけ計算", "MuJoCo: iiwa 14 follows the recorded angle\n→ the true wrench, computed once"), "#eef1f6", size=7.8)
    rows = [(0.13, L("① 箱庭", "① Hakoniwa"), HK_C, L("振動モデル", "vibration model"), L("センサーPRO", "Sensor PRO")),
            (-0.07, L("② Gazebo", "② Gazebo"), GZ_C, None, "gz-sensors"),
            (-0.27, L("③ Gazebo＋箱庭の振動", "③ Gazebo + Hakoniwa vibration"), GZ_C, L("振動モデル", "vibration model"), "gz-sensors")]
    for y, name, col, vib, sensor in rows:
        ax.text(0.03, y + 0.115, name, fontsize=8.6, color=col, fontweight="bold", va="bottom")
        if vib:
            box(ax, (0.30, y), 0.30, 0.09, vib, "#e7effd", ec=HK_C, size=7.8 if cc.LANG[0] == "ja" else 6.9)
        else:
            ax.text(0.45, y + 0.045, L("（振動なし）", "(no vibration)"), fontsize=7.8, color=MUTED, ha="center", va="center")
        arrow(ax, (0.20, y + 0.045), (0.29, y + 0.045), color=MUTED, scale=9)
        ax.text(0.03, y + 0.045, L("真の力", "truth"), fontsize=7.8, color=MUTED, va="center")
        arrow(ax, (0.61, y + 0.045), (0.67, y + 0.045), color=MUTED, scale=9)
        box(ax, (0.68, y), 0.29, 0.09, sensor, "#e7effd" if sensor == L("センサーPRO", "Sensor PRO") else "#fdeedd", ec=col, size=7.8)
    ax.text(0.02, -0.40, L("雑音の大きさは、実機の静止時から合わせた同じ値を\n箱庭（JSON）と Gazebo（SDF）に書いた（軸ごと）",
                           "The same per-axis noise sigma, fitted to the real arm at rest,\nis written for Hakoniwa (JSON) and Gazebo (SDF)"), fontsize=8, color=MUTED, va="top", linespacing=1.5)
    ax.text(0.02, -0.53, L("実機：KUKA LBR Med 14 ＋ ATI Gamma（約 700 Hz、公開データ）", "Real: KUKA LBR Med 14 + ATI Gamma (about 700 Hz, public data)"), fontsize=8, color=MUTED, va="top")


def legend_handles():
    from matplotlib.patches import Patch
    return [Patch(color=HK_C, label=L("① 箱庭", "① Hakoniwa")), Patch(color=GZ_C, label=L("② Gazebo", "② Gazebo")),
            Patch(fc="white", ec=GZ_C, hatch="////", label=L("③ Gazebo＋箱庭の振動", "③ Gazebo + Hakoniwa vibration"))]


# ------------------------------------------------------------------ 1. how and gravity
fig, left, r1 = card(L("比べ方と重力：同じ真の力を、3 通りのセンサーに通す", "Setup and gravity: one true wrench through three sensor set-ups"), right=1)
pipeline(left)
fs_r = 697.84
for k, c in ((0, "Fx"), (2, "Fz")):
    r1.plot(tw, lp(real[:, k], fs_r, 1.0), color=REAL_C, lw=2.2, label=L(f"実機 {c}", f"real {c}"))
    t, x = hk[0]
    r1.plot(t, lp(x[:, k], fs_of(t), 1.0), color=HK_C, lw=1.4, label=L(f"① 箱庭 {c}", f"① Hakoniwa {c}"))
    t, x = gz[0]
    r1.plot(t, lp(x[:, k], fs_of(t), 1.0), color=GZ_C, lw=1.4, ls="--", label=L(f"② Gazebo {c}", f"② Gazebo {c}"))
grid = np.arange(0.3, 6.0, 0.01)
def rms(t, x, k):
    return np.sqrt(np.mean((np.interp(grid, t, lp(x[:, k], fs_of(t), 1.0)) - np.interp(grid, tw, lp(real[:, k], fs_r, 1.0))) ** 2))
r1.text(0.03, 0.96, L("実機との差（RMS）：Fx {:.2f} N、Fz {:.2f} N ― ①②③ で同じ\n（3 つとも同じ真の力を使うので、重力の分は一致する）".format(rms(*hk[0], 0), rms(*hk[0], 2)),
                      "difference from real (RMS): Fx {:.2f} N, Fz {:.2f} N, the same for ①②③\n(all three use the same truth, so the gravity part agrees)".format(rms(*hk[0], 0), rms(*hk[0], 2))),
        transform=r1.transAxes, ha="left", va="top", fontsize=8.5, color=INK, linespacing=1.5)
r1.set_xlim(0, 6.2); r1.set_xlabel(L("時間 [s]", "time [s]")); r1.set_ylabel(L("力 [N]（1 Hz 以下だけ）", "force [N] (below 1 Hz only)"))
r1.grid(alpha=0.3); r1.legend(fontsize=7.8, loc="lower left", ncol=2)
result_title(r1, L("腕を回したときの重力の分（揺れを除いた値）", "Gravity as the wrist turns (below 1 Hz)"))
save(fig, "card-ftv-gz-setup")

# ------------------------------------------------------------------ 2. band ratios
fig, left, r1, r2 = card(L("動いている間の揺れ：6 軸の大きさを実機と比べる", "Vibration while moving: six axes against the real arm"), right=2)
sketch_axes(left, xlim=(0, 1), ylim=(-0.62, 0.55), title=L("測ったこと", "What was measured"))
notes(left, L(["動いている間（0.5–5.5 s）の揺れの大きさを、",
               "周波数の帯ごとに、実機に対する比で示す（1 ＝ 一致）",
               "",
               "5–20 Hz：腕のたわみの揺れ。力制御のフィルタ",
               "（10–20 Hz）を通っても残る、制御に効く帯",
               "20–150 Hz：減速機の歯車の揺れ",
               "",
               "棒の高さは 20 回（乱数の種 1–20）の中央値。",
               "黒い縦線は 20 回の最小〜最大",
               "",
               "Gazebo の雑音は、実機の静止時から合わせた値",
               "（箱庭と同じ値）。足りない分は揺れそのもの"],
              ["The vibration while moving (0.5–5.5 s), per frequency band,",
               "as a ratio to the real recording (1 = equal)",
               "",
               "5–20 Hz: arm flex. It survives a force-control filter",
               "(10–20 Hz), so control sees it",
               "20–150 Hz: the gear ripple",
               "",
               "Bars: median of 20 runs (seeds 1–20).",
               "Black lines: min–max of the 20 runs",
               "",
               "Gazebo's noise is fitted to the real arm at rest (the same",
               "values as Hakoniwa). What is missing is the vibration itself"]), y=0.5, size=8.6, color=INK)
for ax, (lo, hi) in ((r1, (5, 20)), (r2, (20, 150))):
    br = band(tw, real, lo, hi)
    xs = np.arange(6)
    ax.axhspan(0.5, 2.0, color="#eaf4ec", zorder=0)
    ax.axhline(1, color=REAL_C, lw=1.2)
    for i, (runs, fc, ec, hatch) in enumerate(((hk, HK_C, HK_C, None), (gz, GZ_C, GZ_C, None), (gzv, "white", GZ_C, "////"))):
        r = np.array([band(t, x, lo, hi) / br for t, x in runs])
        v = np.median(r, 0)
        xx = xs + (i - 1) * 0.27
        ax.bar(xx, v, 0.27, color=fc, ec=ec, hatch=hatch, lw=1.0, zorder=2)
        ax.vlines(xx, r.min(0), r.max(0), color="#333", lw=0.8, zorder=3)
        for a, b in zip(xx, v):
            ax.text(a, min(b, 2.9) + 0.06, f"{b:.2f}", ha="center", fontsize=6.6, rotation=90, va="bottom")
    ax.set_xticks(xs, NAMES)
    ax.set_ylim(0, 3.2)
    ax.text(5.45, 1.93, L("0.5–2 倍", "0.5–2×"), fontsize=8, color="#1a7f37", ha="right")
    ax.set_ylabel(L("実機に対する比（1 ＝ 一致）", "ratio to real (1 = equal)"))
    result_title(ax, L(f"{lo}–{hi} Hz の揺れ", f"{lo}–{hi} Hz vibration"))
r1.legend(handles=legend_handles(), fontsize=7.8, loc="upper left")
save(fig, "card-ftv-gz-bands")

# ------------------------------------------------------------------ 3. spectrograms
fig, left, r1 = card(L("揺れを時間と周波数で見る（Fx）", "The vibration in time and frequency (Fx)"), right=1)
sketch_axes(left, xlim=(0, 1), ylim=(-0.62, 0.55), title=L("グラフの見方", "How to read it"))
notes(left, L(["横軸は時間、縦軸は周波数。色が明るいほど強い揺れ",
               "3 枚とも同じ色の目盛り",
               "",
               "実機：動き出すと 5–20 Hz（腕のたわみ）が強まり、",
               "回転の速さに沿って歯車の線が現れ、止まると消える",
               "",
               "箱庭：乱数の種 1–20 のうち、5–20 Hz の大きさが",
               "中央値に一番近い 1 回",
               "",
               "Gazebo：同じ雑音の設定。動いても止まっても",
               "同じ一様な雑音だけ"],
              ["Horizontal: time. Vertical: frequency. Brighter = stronger",
               "The three panels share one colour scale",
               "",
               "Real: when the arm moves, 5–20 Hz (arm flex) grows, gear",
               "lines follow the speed, and they vanish at the stop",
               "",
               "Hakoniwa: of seeds 1–20, the run whose 5–20 Hz level is",
               "closest to the median",
               "",
               "Gazebo: the same noise settings. The same flat noise",
               "whether the arm moves or not"]), y=0.5, size=8.6, color=INK)
pos = r1.get_position()
r1.remove()
br = band(tw, real, 5, 20)[0]
ratios = [band(t, x, 5, 20)[0] / br for t, x in hk]
pick = int(np.argmin(np.abs(np.array(ratios) - np.median(ratios))))
panels = [(L("実機", "Real"), REAL_C, tw, real), (L("① 箱庭", "① Hakoniwa"), HK_C, *hk[pick]), (L("② Gazebo", "② Gazebo"), GZ_C, *gz[pick])]
gap = 0.012
x0 = pos.x0 - 0.04
width = (pos.width + 0.02 - 2 * gap) / 3
ims = []
specs = []
for name, col, t, x in panels:
    fs = fs_of(t)
    y = lp(x[:, 0] - x[:, 0].mean(), fs, 3.0, 4, "high")
    f, tt, p = spectrogram(y, fs=fs, nperseg=256, noverlap=240)
    specs.append((f, tt + t[0], 10 * np.log10(p + 1e-14)))
top = np.percentile(specs[0][2][specs[0][0] <= 150], 99.5)
for i, ((name, col, t, x), (f, tt, db)) in enumerate(zip(panels, specs)):
    ax = fig.add_axes([x0 + i * (width + gap), pos.y0, width, pos.height])
    m = f <= 150
    ims.append(ax.pcolormesh(tt, f[m], db[m], shading="auto", cmap="magma", vmin=top - 45, vmax=top))
    ax.set_title(name, fontsize=10.5, color=col, loc="left", fontweight="bold")
    ax.set_xlabel(L("時間 [s]", "time [s]"))
    ax.set_xlim(0, 6.2)
    if i == 0:
        ax.set_ylabel(L("周波数 [Hz]", "frequency [Hz]"))
    else:
        ax.set_yticklabels([])
    for a in (0.28, 5.2):
        ax.axvline(a, color="white", lw=0.6, ls=":")
cb = fig.colorbar(ims[0], ax=fig.axes[-3:], fraction=0.025, pad=0.01)
cb.set_label(L("強さ [dB]", "power [dB]"), fontsize=8)
fig.suptitle(L("揺れを時間と周波数で見る（Fx）", "The vibration in time and frequency (Fx)"), fontsize=13.5, color=INK, x=0.02, ha="left", fontweight="bold")
p = OUT / f"card-ftv-gz-spectrogram-{cc.LANG[0]}.png"
fig.savefig(p, dpi=110)
plt.close(fig)
print(p, "pick seed", pick + 1)

# ------------------------------------------------------------------ 4. direction
fig, left, r1 = card(L("揺れの向き：腕の角度とともに変わる揺れを、Gazebo は出せない", "Direction: the vibration changes with the arm's angle; Gazebo cannot show it"), right=1)
sketch_axes(left, xlim=(0, 1), ylim=(-0.62, 0.55), title=L("測ったこと", "What was measured"))
notes(left, L(["Fz の 5–20 Hz の揺れの大きさを 0.3 秒ごとに測り、",
               "関節の角度 θ（青い点線は |cos θ|）と並べた",
               "",
               "実機では、腕の揺れの向きが世界に対して決まっている。",
               "センサーが回ると、Fz に出る分が |cos θ| に沿って変わる",
               "",
               "帯は 20 回の最小〜最大、線は中央値",
               "③ Gazebo＋箱庭の振動は ① とほぼ重なる"],
              ["The Fz 5–20 Hz vibration in 0.3 s windows, against the",
               "joint angle θ (blue dotted: |cos θ|)",
               "",
               "On the real arm the vibration has a fixed direction in the",
               "world; as the sensor turns, its share in Fz follows |cos θ|",
               "",
               "Bands: min–max of 20 runs; lines: the median",
               "③ Gazebo + Hakoniwa vibration lies on top of ①"]), y=0.5, size=8.6, color=INK)
c, e = env(tw, real, 2)
for runs, col, name in ((hk, HK_C, L("① 箱庭", "① Hakoniwa")), (gz, GZ_C, L("② Gazebo", "② Gazebo"))):
    n = np.array([env(t, x, 2)[1] for t, x in runs])
    r1.fill_between(c, n.min(0), n.max(0), color=col, alpha=0.18, lw=0)
    r1.plot(c, np.median(n, 0), color=col, lw=1.8, label=name)
r1.plot(c, e, color=REAL_C, lw=2.2, label=L("実機", "real"))
cos = np.abs(np.cos(np.interp(c, tq, q)))
r1b = r1.twinx()
r1b.plot(c, cos, color=C1, ls=":", lw=1.3)
r1b.set_ylim(0, 1.6)
r1b.set_ylabel("|cos θ|", color=C1)
r1.set_xlim(0, 5.8); r1.set_ylim(0, 0.32)
r1.set_xlabel(L("時間 [s]", "time [s]")); r1.set_ylabel(L("Fz の 5–20 Hz の揺れ [N]", "Fz vibration, 5–20 Hz [N]")); r1.grid(alpha=0.3)
cr = corr(tw, real)
ch, cg, cv = (np.median([corr(t, x) for t, x in runs]) for runs in (hk, gz, gzv))
r1.text(0.99, 0.97, L(f"|cos θ| との相関（1 回ごと、中央値）：実機 {cr:.2f}、① {ch:.2f}、② {cg:.2f}、③ {cv:.2f}",
                      f"correlation with |cos θ| (per run, median): real {cr:.2f}, ① {ch:.2f}, ② {cg:.2f}, ③ {cv:.2f}"),
        transform=r1.transAxes, ha="right", va="top", fontsize=8.5, color=INK)
r1.legend(fontsize=8, loc="upper left", bbox_to_anchor=(0, 0.9))
result_title(r1, L("Fz の揺れの大きさ（0.3 秒ごと）と、関節の角度", "Fz vibration (0.3 s windows) against the joint angle"))
save(fig, "card-ftv-gz-direction")
print("corr", round(cr, 3), round(ch, 3), round(cg, 3), round(cv, 3))

# ------------------------------------------------------------------ 5. at rest
def rest_spec(t, x, settle=1.0):
    m = t >= settle if t[-1] > 2 * settle else np.ones(len(t), bool)
    y = x[m] - x[m].mean(0)
    return welch(y, fs=fs_of(t), nperseg=128, detrend="linear", axis=0)


def peak_over_floor(f, p):
    floor = np.median(p[(f >= 40) & (f <= 300)], 0)
    return np.median(p[(f >= 9) & (f < 16)], 0) / floor, floor


fig, left, r1, r2 = card(L("止まっているとき：保持中の腕の揺れ（9–16 Hz）と雑音", "At rest: the held arm's vibration (9–16 Hz) and the noise"), right=2)
sketch_axes(left, xlim=(0, 1), ylim=(-0.62, 0.55), title=L("測ったこと", "What was measured"))
notes(left, L(["腕を止めたまま（関節を保持）の手首の値",
               "実機：静止時の記録（約 1 秒）",
               "箱庭・Gazebo：同じ姿勢で 10 秒（最初の 1 秒は除く）、",
               "乱数の種 1–6",
               "",
               "左：Fz の周波数ごとの強さ（実機は太線、他は 6 回の中央値）",
               "右：9–16 Hz の強さを、40–300 Hz の雑音の床で割った値",
               "（1 ＝ 山なし。対数目盛り）",
               "",
               "実機は止まっていても約 9.5 Hz で揺れる（保持中の腕）。",
               "箱庭は振動モデルの「止まっているときの揺れ」で再現。",
               "Gazebo は白い雑音だけなので、山が出ない"],
              ["The wrist with the arm held still (joints holding)",
               "Real: the rest recording (about 1 s)",
               "Hakoniwa, Gazebo: the same pose for 10 s (first 1 s left out),",
               "seeds 1–6",
               "",
               "Left: Fz by frequency (real thick, others the median of 6)",
               "Right: the 9–16 Hz level over the 40–300 Hz noise floor",
               "(1 = no peak; log scale)",
               "",
               "The real arm vibrates at about 9.5 Hz even when held.",
               "Hakoniwa: the vibration model's level at rest.",
               "Gazebo has white noise only, so no peak"]), y=0.5, size=8.6, color=INK)
f, p = rest_spec(tr_rest, real_rest)
r1.semilogy(f, np.sqrt(p[:, 2]), color=REAL_C, lw=2.2, label=L("実機", "real"))
for runs_, col, name, ls in ((hk_rest, HK_C, L("① 箱庭", "① Hakoniwa"), "-"), (gz_rest, GZ_C, L("② Gazebo", "② Gazebo"), "-"),
                            (gzv_rest, GZ_C, L("③ Gazebo＋箱庭の振動", "③ Gazebo + Hakoniwa vibration"), ":")):
    P = np.array([rest_spec(t, x)[1][:, 2] for t, x in runs_])
    fr = rest_spec(*runs_[0])[0]
    r1.semilogy(fr, np.sqrt(np.median(P, 0)), color=col, lw=1.5, ls=ls, label=name)
r1.set_xlim(0, 120); r1.grid(alpha=0.3, which="both")
r1.set_xlabel(L("周波数 [Hz]", "frequency [Hz]")); r1.set_ylabel(L("Fz の強さ [N/√Hz]", "Fz amplitude density [N/√Hz]"))
r1.legend(fontsize=7.8, loc="upper right")
result_title(r1, L("Fz の周波数ごとの強さ（止まっているとき）", "Fz by frequency, at rest"))
qr, _ = peak_over_floor(f, p)
xs = np.arange(6)
r2.bar(xs - 0.3, qr, 0.2, color=REAL_C, label=L("実機", "real"))
for i, (runs_, fc, ec, hatch, name) in enumerate(((hk_rest, HK_C, HK_C, None, "①"), (gz_rest, GZ_C, GZ_C, None, "②"),
                                                   (gzv_rest, "white", GZ_C, "////", "③"))):
    v = np.median([peak_over_floor(*rest_spec(t, x))[0] for t, x in runs_], 0)
    r2.bar(xs - 0.1 + 0.2 * i, v, 0.2, color=fc, ec=ec, hatch=hatch, lw=1.0)
    print("rest 9-16/floor", name, np.round(v, 1))
print("rest 9-16/floor real", np.round(qr, 1))
r2.set_yscale("log"); r2.axhline(1, color=MUTED, lw=1, ls="--")
r2.set_xticks(xs, NAMES); r2.set_ylabel(L("9–16 Hz ÷ 雑音の床（1 ＝ 山なし）", "9–16 Hz over the noise floor (1 = no peak)"))
r2.legend(handles=[__import__("matplotlib.patches").patches.Patch(color=REAL_C, label=L("実機", "real"))] + legend_handles(), fontsize=7.4, loc="upper right")
result_title(r2, L("9–16 Hz の山の高さ（6 軸）", "The 9–16 Hz peak, six axes"))
save(fig, "card-ftv-gz-rest")
