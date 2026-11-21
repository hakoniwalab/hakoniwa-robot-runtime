"""Tool mass, centre of mass and sensor bias from the 24-orientation calibration (Zenodo 11096791, 0-calibration).
Model: F = bF + m g,  T = bT + c x (m g)  (g = gravity vector in the sensor frame, columns gx gy gz)."""
import sys, json, numpy as np
d = np.genfromtxt(sys.argv[1], delimiter=",", names=True)
G = np.stack([d["gx"], d["gy"], d["gz"]], 1); F = np.stack([d["fx"], d["fy"], d["fz"]], 1); T = np.stack([d["tx"], d["ty"], d["tz"]], 1)
n = len(d)
# forces: unknowns m, bF(3)
A = np.zeros((3 * n, 4)); y = F.reshape(-1)
for i in range(n):
    A[3*i:3*i+3, 0] = G[i]; A[3*i:3*i+3, 1:4] = np.eye(3)
xf, *_ = np.linalg.lstsq(A, y, rcond=None); m, bF = xf[0], xf[1:]
# torques: T = bT + c x (m g) = bT - (m g) x c = bT - skew(m g) c ; unknowns bT(3), c(3)
def skew(v): return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
B = np.zeros((3 * n, 6)); z = T.reshape(-1)
for i in range(n):
    B[3*i:3*i+3, 0:3] = np.eye(3); B[3*i:3*i+3, 3:6] = -skew(m * G[i])
xt, *_ = np.linalg.lstsq(B, z, rcond=None); bT, c = xt[:3], xt[3:]
rf = (A @ xf - y).reshape(-1, 3); rt = (B @ xt - z).reshape(-1, 3)
out = {"samples": int(n), "mass_kg": round(float(m), 4), "com_m_sensor_frame": [round(float(v), 4) for v in c],
       "force_bias_N": [round(float(v), 3) for v in bF], "torque_bias_Nm": [round(float(v), 4) for v in bT],
       "rms_residual_force_N": [round(float(v), 3) for v in np.sqrt((rf ** 2).mean(0))],
       "rms_residual_torque_Nm": [round(float(v), 4) for v in np.sqrt((rt ** 2).mean(0))]}
print(json.dumps(out, indent=1)); json.dump(out, open(sys.argv[2], "w"), indent=1)
