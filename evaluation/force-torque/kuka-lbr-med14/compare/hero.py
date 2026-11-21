"""Overview figure of the KUKA real-data page. python3 compare/hero.py ja|en (writes compare/out/cards/)."""
import sys, numpy as np, matplotlib
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import kukadata as K
JA = sys.argv[1] == "ja"
L = lambda a, b: a if JA else b
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from scipy.signal import welch
plt.rcParams["font.family"] = ["Hiragino Sans", "sans-serif"]
ax = ["fx","fy","fz","tx","ty","tz"]; lab = ["Fx [N]","Fy [N]","Fz [N]","Tx [N·m]","Ty [N·m]","Tz [N·m]"]
tw, real = K.real()
runs = [np.column_stack([t, x]) for t, x, _ in K.runs("robot-t14")]
fig, A = plt.subplots(6, 2, figsize=(14, 15), gridspec_kw={"width_ratios":[1.6,1]})
def psd(x, t, fs):
    m = (t>=0.5)&(t<=5.5); f,p = welch(x[m], fs=fs, nperseg=1024); return f, np.sqrt(p)
for k in range(6):
    a = A[k,0]
    a.plot(tw, real[:,k], color="#222", lw=0.6, label=L("実機（公開データ）", "real (public recording)"))
    r = runs[0]; a.plot(r[:,0], r[:,1+k], color="#d9480f", lw=0.6, alpha=0.75, label=L("箱庭（1 回目）", "Hakoniwa (run 1)"))
    a.set_xlim(0, 6.3); a.set_ylabel(lab[k]); a.grid(alpha=0.3)
    b = A[k,1]
    for i,r in enumerate(runs):
        f,p = psd(r[:,1+k], r[:,0], 1000); b.semilogy(f, p, color="#d9480f", lw=0.6, alpha=0.5, label=L("箱庭 6 回", "Hakoniwa, 6 runs") if i==0 else None)
    f,p = psd(real[:,k], tw, 697.84); b.semilogy(f, p, color="#222", lw=0.9, label=L("実機", "real"))
    b.set_xlim(0, 340); b.grid(alpha=0.3, which="both")
A[0,0].legend(loc="upper left", fontsize=9); A[0,1].legend(loc="upper right", fontsize=9)
A[5,0].set_xlabel(L("時間 [s]（0–1 s 加速、1–4.4 s 巡航、4.4–5.2 s 減速、その後 停止）", "time [s] (0–1 s accel, 1–4.4 s cruise, 4.4–5.2 s decel, then stopped)"))
A[5,1].set_xlabel(L("周波数 [Hz]（動いている 0.5–5.5 s）", "frequency [Hz] (moving, 0.5–5.5 s)"))
fig.suptitle(L("KUKA の手首の力覚センサー：実機の記録と、箱庭で同じ動きをさせた結果（生の値）", "KUKA wrist force-torque sensor: the real recording and Hakoniwa running the same motion (raw values)"), x=0.01, ha="left", fontsize=14)
fig.tight_layout(rect=[0,0,1,0.97])
(K.OUT / "cards").mkdir(parents=True, exist_ok=True)
fig.savefig(K.OUT / "cards" / f"ftv-kuka-overview-{sys.argv[1]}.png", dpi=110)
