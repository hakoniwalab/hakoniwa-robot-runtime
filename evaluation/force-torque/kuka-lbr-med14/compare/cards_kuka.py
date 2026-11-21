"""Cards of the KUKA real-data page (force-torque vibration). python3 compare/cards_kuka.py ja|en

Real: data/zenodo-11096791 (data/fetch.py). Model before (T12) and now (T14): 20 harness runs each
(runs/harness-t12.npz, runs/harness-t14.npz). Hakoniwa robot runs (iiwa 14 through the robot runtime, T14): 6 runs,
runs/robot-t14.npz. Writes compare/out/cards/.
"""
import sys
from pathlib import Path
import numpy as np
from scipy.signal import welch, butter, sosfiltfilt
sys.path.insert(0, str(Path(__file__).resolve().parent))
import kukadata as K  # noqa: E402
import cards_common as cc  # noqa: E402
from cards_common import L, INK, MUTED, BODY, C1, C2, arrow, sketch_axes, notes, card, result_title, plt  # noqa: E402
from matplotlib.patches import Arc, Rectangle, Circle  # noqa: E402

cc.LANG[0] = sys.argv[1]
OUT = K.OUT / "cards"
OUT.mkdir(parents=True, exist_ok=True)
AX = K.AX
REAL_C, OLD_C, HK_C = "#222222", "#9a9a9a", "#d9480f"

tw, real = K.real()
t12 = K.runs("harness-t12")            # model before (20 runs, harness)
t14 = K.runs("harness-t14")            # model now (20 runs, harness)
hk = [(t, x, None) for t, x, _ in K.runs("robot-t14")]   # Hakoniwa robot runs (6)
tq, q = t14[0][0], np.abs(t14[0][2])

def save(fig, name):
    fig.tight_layout(rect=(0, 0, 1, 0.93)); p = OUT / f"{name}-{cc.LANG[0]}.png"; fig.savefig(p, dpi=110); plt.close(fig); print(p)

def lp(x, fs, fc, order=2, kind="low"):
    return sosfiltfilt(butter(order, fc, kind, fs=fs, output="sos"), x, axis=0)

def env(t, x, k, win=0.3, step=0.15):
    """5-20 Hz amplitude in short windows."""
    fs = 1 / np.median(np.diff(t)); y = lp(lp(x[:, k], fs, 3.0, 4, "high"), fs, [5, 20], 4, "band")
    c = np.arange(0.15, 5.75, step); return c, np.array([y[(t >= a - win / 2) & (t < a + win / 2)].std() for a in c])

def band(t, x, t0=0.5, t1=5.5):
    fs = 1 / np.median(np.diff(t)); m = (t > t0) & (t < t1); y = x[m] - x[m].mean(0)
    y = lp(y, fs, 3.0, 4, "high"); f, p = welch(y, fs=fs, nperseg=min(512, m.sum()), axis=0)
    s = (f >= 5) & (f < 20); return np.sqrt(p[s].sum(0) * (f[1] - f[0]))

def arm(ax, theta_deg, alpha=1.0, z_axis=True, label=None):
    """The turning link seen along the joint axis, sensor and tool at its end."""
    th = np.radians(theta_deg); d = np.array([np.cos(th), np.sin(th)]); n = np.array([-d[1], d[0]])
    tip = 0.30 * d
    ax.plot([0, tip[0]], [0, tip[1]], color=BODY, lw=9, solid_capstyle="round", alpha=alpha, zorder=2)
    s = tip + 0.03 * d
    ax.add_patch(Rectangle(s - 0.035 * n - 0.0 * d, 0.025, 0.07, angle=theta_deg, fc=C2, ec="none", alpha=alpha, zorder=3))
    ax.add_patch(Rectangle(s + 0.025 * d - 0.03 * n, 0.07, 0.06, angle=theta_deg, fc="#7c8aa3", ec="none", alpha=alpha, zorder=3))
    if z_axis:
        a0 = s + 0.12 * d
        arrow(ax, tuple(a0), tuple(a0 + 0.12 * d), color=C1, lw=1.6, scale=12)
        ax.text(*(a0 + 0.06 * d + 0.11 * n), L("センサー z", "sensor z"), color=C1, fontsize=8.5, ha="center", va="center")
    if label:
        ax.text(*(tip + 0.18 * d - 0.12 * n), label, fontsize=8.5, color=INK, ha="center")

def pivot(ax):
    ax.add_patch(Circle((0, 0), 0.035, fc="white", ec=INK, lw=1.5, zorder=4))
    ax.add_patch(Circle((0, 0), 0.008, fc=INK, ec="none", zorder=5))
    ax.add_patch(Rectangle((-0.09, -0.16), 0.18, 0.12, fc=BODY, ec="none", zorder=1))

# ------------------------------------------------------------------ 1. motion and gravity
fig, left, r1, r2 = card(L("動きと重力：関節を 1 本、約 5 秒で 180° 回す", "Motion and gravity: one joint turns 180° in about 5 s"), right=2)
sketch_axes(left, xlim=(-0.5, 0.55), ylim=(-0.62, 0.55))
pivot(left); arm(left, 0, label="0°"); arm(left, 90, alpha=0.35, z_axis=False, label="90°")
left.add_patch(Arc((0, 0), 0.24, 0.24, theta1=5, theta2=175, color=INK, lw=1.4))
arrow(left, (-0.115, 0.04), (-0.12, 0.01), color=INK, scale=12)
left.text(0, 0.15, L("180° 回す", "turn 180°"), ha="center", fontsize=9, color=INK)
arrow(left, (-0.42, 0.35), (-0.42, 0.22), color=MUTED, scale=12); left.text(-0.40, 0.29, L("重力", "gravity"), fontsize=8.5, color=MUTED)
notes(left, L(["関節の軸は水平（紙面に垂直）。何にも触れない",
               "実機：KUKA LBR Med 14 ＋ ATI Gamma（約 700 Hz）、道具 0.93 kg",
               "箱庭：KUKA iiwa 14（MuJoCo Menagerie）を箱庭のアームとして起動し、",
               "記録と同じ角度の推移で回す。力覚はセンサーPRO ＋ 振動モデル"],
              ["The joint axis is horizontal (into the page). Nothing is touched",
               "Real: KUKA LBR Med 14 + ATI Gamma (about 700 Hz), 0.93 kg tool",
               "Hakoniwa: KUKA iiwa 14 (MuJoCo Menagerie) runs as a Hakoniwa arm and",
               "follows the recorded angle. F/T: Sensor PRO + vibration model"]), y=-0.24)
r1.plot(tq, np.degrees(q), color=INK)
for a, b, lab in ((0, 1, L("加速", "accel")), (1, 4.4, L("巡航", "cruise")), (4.4, 5.2, L("減速", "decel")), (5.2, 6.2, L("停止", "stop"))):
    r1.axvspan(a, b, color="#eef1f6" if lab in ("加速", "減速", "accel", "decel") else "white", zorder=0); r1.text((a + b) / 2, 190, lab, ha="center", fontsize=8.5, color=MUTED)
r1.set_ylim(-5, 205); r1.set_xlim(0, 6.2); r1.set_xlabel(L("時間 [s]", "time [s]")); r1.set_ylabel(L("関節の角度 θ [°]", "joint angle θ [°]")); r1.grid(alpha=0.3)
result_title(r1, L("回した角度（実機の記録から）", "Joint angle (from the recording)"))
fs_r = 697.84
for k, c in ((0, "Fx"), (2, "Fz")):
    r2.plot(tw, lp(real[:, k], fs_r, 1.0), color=REAL_C, lw=1.6, label=L(f"実機 {c}", f"real {c}"))
    t, x, _ = hk[0]; r2.plot(t, lp(x[:, k], 1000, 1.0), color=HK_C, lw=1.4, ls="--", label=L(f"箱庭 {c}", f"Hakoniwa {c}"))
r2.set_xlim(0, 6.2); r2.set_xlabel(L("時間 [s]", "time [s]")); r2.set_ylabel(L("力 [N]（1 Hz 以下だけ）", "force [N] (below 1 Hz only)")); r2.grid(alpha=0.3); r2.legend(fontsize=8, loc="lower left")
r2.text(0.03, 0.95, L("差の RMS：Fx 0.42 N、Fz 0.54 N", "RMS difference: Fx 0.42 N, Fz 0.54 N"), transform=r2.transAxes, ha="left", va="top", fontsize=8.5, color=INK)
result_title(r2, L("重力の向きの変化（揺れを除いた値）", "Gravity as the wrist turns (below 1 Hz)"))
save(fig, "card-ftv-kuka-motion")

# ------------------------------------------------------------------ 2. world-fixed direction
fig, left, r1 = card(L("揺れの向き：上下方向の揺れは、世界に対して向きが決まっていた", "Direction: the Fz vibration is fixed in the world, not in the sensor"), right=1)
sketch_axes(left, xlim=(-0.5, 0.55), ylim=(-0.62, 0.55), title=L("仕組み", "Mechanism"))
pivot(left); arm(left, 0, label="0°"); arm(left, 90, alpha=0.45, label="90°")
for y in (0.0, 0.33):
    x0, yy = (0.47, -0.08) if y == 0.0 else (0.13, 0.33)
    left.annotate("", xy=(x0 + 0.07, yy), xytext=(x0 - 0.07, yy), arrowprops=dict(arrowstyle="<->", color=C2, lw=2))
left.text(0.47, -0.15, L("揺れ", "vibration"), color=C2, fontsize=8.5, ha="center")
notes(left, L(["揺れ（赤）は、腕の回る面の中で水平のまま",
               "0°：センサー z と同じ向き → Fz に全部出る",
               "90°：センサー z と直角 → Fz にはほぼ出ない",
               "→ Fz の揺れは |cos θ| に沿う",
               "モデル：モードごとに向きを「センサー」か「世界」で選ぶ"],
              ["The vibration (red) stays horizontal in the plane of the turn",
               "0°: along sensor z → all of it shows in Fz",
               "90°: across sensor z → almost none in Fz",
               "→ the Fz vibration follows |cos θ|",
               "Model: each mode is fixed in the sensor or in the world"]), y=-0.22)
c, e = env(tw, real, 2)
o = np.array([env(t, x, 2)[1] for t, x, _ in t12]); n = np.array([env(t, x, 2)[1] for t, x, _ in hk])
r1.fill_between(c, np.percentile(o, 5, 0), np.percentile(o, 95, 0), color=OLD_C, alpha=0.2, lw=0)
r1.plot(c, np.median(o, 0), color=OLD_C, lw=1.5, label=L("前のモデル（向きはセンサーに固定）", "model before (fixed in the sensor)"))
r1.fill_between(c, n.min(0), n.max(0), color=HK_C, alpha=0.2, lw=0)
r1.plot(c, np.median(n, 0), color=HK_C, lw=1.8, label=L("今：箱庭 6 回の中央値（帯は最小〜最大）", "now: Hakoniwa, median of 6 runs (band: min–max)"))
r1.plot(c, e, color=REAL_C, lw=2, label=L("実機", "real"))
cos = np.abs(np.cos(np.interp(c, tq, q)))
r1b = r1.twinx(); r1b.plot(c, cos, color=C1, ls=":", lw=1.3); r1b.set_ylim(0, 1.6); r1b.set_ylabel("|cos θ|", color=C1)
r1.set_xlim(0, 5.8); r1.set_ylim(0, 0.32); r1.set_xlabel(L("時間 [s]", "time [s]")); r1.set_ylabel(L("Fz の 5–20 Hz の揺れ [N]", "Fz vibration, 5–20 Hz [N]")); r1.grid(alpha=0.3)
def corr(t, x):
    """Correlation of the Fz 5-20 Hz amplitude (0.3 s windows, 0.5-5.3 s) with |cos θ|."""
    cw, e = env(t, x, 2, 0.3, 0.3); m = (cw > 0.5) & (cw < 5.3); return np.corrcoef(e[m], np.abs(np.cos(np.interp(cw, tq, q)))[m])[0, 1]
cr, ch, co = corr(tw, real), np.median([corr(t, x) for t, x, _ in hk]), np.median([corr(t, x) for t, x, _ in t12])
r1.text(0.99, 0.97, L(f"|cos θ| との相関（1 回ごと、中央値）：実機 {cr:.2f}、箱庭 {ch:.2f}、前 {co:.2f}",
                      f"correlation with |cos θ| (per run, median): real {cr:.2f}, Hakoniwa {ch:.2f}, before {co:.2f}"),
        transform=r1.transAxes, ha="right", va="top", fontsize=8.5, color=INK)
r1.legend(fontsize=8, loc="lower left")
result_title(r1, L("Fz の揺れの大きさ（0.3 秒ごと）と、関節の角度", "Fz vibration (0.3 s windows) against the joint angle"))
save(fig, "card-ftv-kuka-direction")
print("corr", cr, ch, co)

# ------------------------------------------------------------------ 3. ratios per axis
fig, left, r1 = card(L("制御に効く 5–20 Hz：6 軸の揺れの大きさを実機と比べる", "The 5–20 Hz band that control sees: vibration on six axes against the real arm"), right=1)
sketch_axes(left, xlim=(-0.5, 0.55), ylim=(-0.62, 0.55))
left.text(-0.48, 0.42, L("力制御は値を 10–20 Hz のフィルタに通して使う。\n残る揺れの大部分は 20 Hz 以下（腕のしなり）",
                         "Force control passes the signal through a 10–20 Hz filter.\nMost of the vibration left is below 20 Hz (arm flex)"),
          fontsize=9, color=INK, va="top", linespacing=1.6)
notes(left, L(["動いている間（0.5–5.5 s）の 5–20 Hz の揺れの大きさを、",
               "実機に対する比で比べる（1 がぴったり）",
               "灰：前のモデル（検証用プログラム 20 回の中央値）",
               "橙：今のモデル（検証用プログラム 20 回の中央値）",
               "赤：今のモデルを箱庭で動かした 6 回の中央値"],
              ["5–20 Hz vibration while moving (0.5–5.5 s),",
               "as a ratio to the real recording (1 = equal)",
               "grey: model before (median of 20 harness runs)",
               "orange: model now (median of 20 harness runs)",
               "red: model now, run in Hakoniwa (median of 6 runs)"]), y=0.18)
br = band(tw, real)
sets = [(t12, OLD_C, L("前のモデル", "before")), (t14, "#f08c4a", L("今（検証用 20 回）", "now (harness, 20)")), (hk, HK_C, L("今（箱庭 6 回）", "now (Hakoniwa, 6)"))]
xs = np.arange(6)
r1.axhspan(0.5, 2.0, color="#eaf4ec", zorder=0); r1.axhline(1, color=INK, lw=1)
for i, (runs, col, lab) in enumerate(sets):
    v = np.median([band(t, x) / br for t, x, _ in runs], 0)
    r1.bar(xs + (i - 1) * 0.27, v, 0.27, color=col, label=lab)
    for xx, vv in zip(xs + (i - 1) * 0.27, v): r1.text(xx, vv + 0.03, f"{vv:.2f}", ha="center", fontsize=7.5)
    print(lab, np.round(v, 2))
r1.set_xticks(xs, ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]); r1.set_ylabel(L("実機に対する比（1 ＝ 一致）", "ratio to real (1 = equal)"))
r1.set_ylim(0, 2.6); r1.legend(fontsize=8, loc="upper right"); r1.text(5.45, 1.93, L("0.5–2 倍", "0.5–2×"), fontsize=8, color="#1a7f37", ha="right")
result_title(r1, L("動いている間の 5–20 Hz の揺れ：実機に対する比", "5–20 Hz vibration while moving: ratio to real"))
save(fig, "card-ftv-kuka-ratio")

# ------------------------------------------------------------------ 4. spectrum and the gap left
fig, left, r1, r2 = card(L("周波数で見る：合ったところと、残っている差", "By frequency: what matches and the gap that is left"), right=2)
sketch_axes(left, xlim=(-0.5, 0.55), ylim=(-0.62, 0.55), title=L("揺れの成分（モデル）", "Components (model)"))
notes(left, L(["歯車の回転ムラ：モーターの回転についてくる線",
               "　（巡航で 40 Hz とその倍数。速さの 0.8 乗で大きくなる。",
               "　 重力の負荷の向きが変わると 1.4 倍）",
               "動いている間だけの固定の線：100 Hz、300 Hz",
               "腕のしなり：動いている間は速さによらない揺れ、",
               "　止まっている間だけの 9.3 Hz の山",
               "世界に向きが固定された揺れ（12 Hz 付近）",
               "広帯域の床"],
              ["Gear ripple: lines that follow the motor speed",
               "  (40 Hz and multiples in cruise; grows as speed^0.8;",
               "   ×1.4 after the gravity load reverses)",
               "Fixed lines only while moving: 100 Hz, 300 Hz",
               "Arm flex: speed-independent while moving,",
               "  a 9.3 Hz peak only at rest",
               "A vibration fixed in the world (around 12 Hz)",
               "Broadband floor"]), y=0.45, size=8.8, color=INK)
def spec(t, x, k, n=1024):
    fs = 1 / np.median(np.diff(t)); m = (t >= 0.5) & (t <= 5.5); f, p = welch(x[m, k], fs=fs, nperseg=n); return f, np.sqrt(p)
for ax, k, lim, nper, title in ((r1, 0, (0, 340), 1024, L("Fx：0–340 Hz", "Fx: 0–340 Hz")), (r2, 1, (2, 45), 1024, L("Fy：2–45 Hz の拡大", "Fy: zoom on 2–45 Hz"))):
    ps = [spec(t, x, k) for t, x, _ in hk]; f = ps[0][0]; P = np.array([p for _, p in ps])
    ax.fill_between(f, P.min(0), P.max(0), color=HK_C, alpha=0.25, lw=0)
    ax.semilogy(f, np.median(P, 0), color=HK_C, lw=1.2, label=L("箱庭 6 回（中央値、帯は最小〜最大）", "Hakoniwa 6 runs (median, band min–max)"))
    fr, pr = spec(tw, real, k); ax.semilogy(fr, pr, color=REAL_C, lw=1.1, label=L("実機", "real"))
    ax.set_xlim(*lim); ax.grid(alpha=0.3, which="both"); ax.set_xlabel(L("周波数 [Hz]（動いている 0.5–5.5 s）", "frequency [Hz] (moving, 0.5–5.5 s)"))
    ax.set_ylabel(L("大きさ [N/√Hz]", "amplitude [N/√Hz]")); result_title(ax, title)
r1.legend(fontsize=7.5, loc="upper right")
r2.axvspan(15, 30, color="#fbe3e4", zorder=0)
r2.text(22.5, 0.9, L("ここが合っていない：\n実機は 13 Hz の細い山と深い谷", "Not matched yet:\nnarrow 13 Hz peak and deep valley"), transform=r2.get_xaxis_transform(), ha="center", fontsize=8, color=C2)
save(fig, "card-ftv-kuka-spectrum")
