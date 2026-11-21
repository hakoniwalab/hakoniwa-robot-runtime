# Wrist force-torque sensor of a KUKA arm against a real recording

The settings, tools and data behind two comparisons of a simulated wrist force-torque sensor with a real one:

- **Hakoniwa against the real arm**: the published page [Force-torque vibration: comparison with a real recording](https://github.com/hakoniwalab/hakoniwa-sensor-adapter/blob/main/docs/evaluation/force-torque/real-data-kuka.md) (its cards come from `compare/cards_kuka.py` and `compare/hero.py`).
- **Hakoniwa and Gazebo against the real arm**: one true wrench from MuJoCo goes through three sensor set-ups — Hakoniwa (Sensor PRO with the vibration model), Gazebo (gz-sensors), and Gazebo with Hakoniwa's vibration model in front of it (`compare/cards_gazebo.py`).

The robot (the KUKA iiwa 14 model and its recipe) is in [hakoniwa-robot-arm](https://github.com/hakoniwalab/hakoniwa-robot-arm); this folder holds only the sensor set fitted to the real arm and the evaluation.

## The real recording

Zenodo record 11096791 ([doi:10.5281/zenodo.11096791](https://doi.org/10.5281/zenodo.11096791)): A. Skrede, "Accelerometer and Force/Torque Sensor Measurements for Parameter and State Estimation of an Unknown Robot End Effector", 2024, CC BY 4.0. A KUKA LBR Med 14 with an ATI Gamma force-torque sensor (about 700 Hz) and a 0.93 kg tool turns one joint by 180° in about 5 s. The data are not in this repository; `data/fetch.py` downloads them and checks every file's MD5.

## What is here

| Path | Contents |
|---|---|
| `sensors/` | The sensor set for the iiwa14 recipe: `iiwa14-sensors-hakoniwa.json` (the wrist sensor with Sensor PRO, its mount and the vibration model), `force_torque/wrist-ft-ati-gamma-fitted.json` (the ATI Gamma noise per axis), `vibration/kuka-lbr-med14-t14.json` (the vibration model used in both comparisons) and the earlier `t12`, `t12-novar` |
| `install_sensors.py` | Copies `sensors/` into a hakoniwa-robot-arm checkout, where the iiwa14 recipe reads it with `--sensors --sensor-implementation hakoniwa` |
| `fitting/fit_noise.py` | The force-torque noise from the real arm at rest: reproduces `sensors/force_torque/wrist-ft-ati-gamma-fitted.json` exactly |
| `fitting/tool_fit.py` | The tool mass, centre of mass and sensor bias from the 24-pose calibration: reproduces `compare/config/tool.json` exactly |
| `compare/` | The comparison and card scripts, the Gazebo configuration with the same noise (`config/gz-ati-gamma-fitted.sdf`) and the bias of the real sensor (`config/tool.json`) |
| `runs/` | The simulated runs the cards are drawn from (float32): `harness-t12.npz`, `harness-t14.npz` (20 seeds each, the sensor stage driven alone on the KUKA model), `robot-t14.npz` (6 runs of the iiwa 14 through the robot runtime), `truth.npz` (the true wrench, no noise) and `truth-vibration-t14.npz` (the true wrench plus the vibration model, seeds 1–20) |
| `data/fetch.py` | Downloads the real recording |

The vibration settings (`vibration/*.json`) were fitted with Hakoniwa Lab's own procedure; the fitting of the noise and of the tool is public here. The vibration model's specification is public: [force-torque-vibration.md](https://github.com/hakoniwalab/hakoniwa-sensor-adapter/blob/main/docs/spec/force-torque-vibration.md).

## Drawing the cards

The card helpers come from [hakoniwa-sensor-adapter](https://github.com/hakoniwalab/hakoniwa-sensor-adapter), checked out beside this repository (or set `HAKONIWA_SENSOR_ADAPTER_ROOT`). Python 3 with NumPy, SciPy and Matplotlib.

```bash
cd evaluation/force-torque/kuka-lbr-med14
python3 data/fetch.py
python3 compare/cards_kuka.py en
python3 compare/hero.py en
```

The cards go to `compare/out/cards/` (not committed); `ja` gives the Japanese ones.

For the Gazebo comparison, first run the Gazebo sensor on the same truth. This needs `ft-series` from hakoniwa-sensor-adapter's `tools/compare`, built with pixi as its README describes; it reads the truth from a file with `--motion file`:

```bash
python3 compare/run_gazebo.py <path to ft-series>
python3 compare/cards_gazebo.py en
```

`run_gazebo.py` writes 20 runs each to `compare/out/gazebo/` and `compare/out/gazebo-vibration/`.

## Using the sensor set in the iiwa14 recipe

```bash
python3 install_sensors.py <path to hakoniwa-robot-arm>
```

copies the files into `recipes/iiwa14/config/sensors/` of that checkout without committing them. The recipe then uses them when it runs with `--sensors --sensor-implementation hakoniwa`, built with Sensor PRO as hakoniwa-robot-arm's build procedure describes (`HAKO_ROBOT_ARM_SENSOR_IMPLEMENTATIONS`, `HAKO_ROBOT_ARM_SENSOR_LIBRARIES`, and `HAKO_ROBOT_ARM_SENSOR_HOST_MODELS` for the vibration model). Without the option the recipe keeps its default sensor, which reports the true wrench.

## Results in short

| | Hakoniwa | Gazebo | Gazebo + Hakoniwa vibration |
|---|---|---|---|
| Gravity part, RMS difference from real (Fx / Fz) | 0.42 / 0.54 N | the same | the same |
| Vibration while moving, 5–20 Hz, ratio to real (six axes, median of 20) | 0.77–1.30 | 0.03–0.14 | 0.77–1.29 |
| Vibration while moving, 20–150 Hz | 0.78–1.12 | 0.01–0.07 | 0.78–1.12 |
| Correlation of the Fz vibration with \|cos θ\| (real 0.80) | 0.86 | −0.05 | 0.86 |

The noise of all three is the same fitted per-axis sigma. Gazebo has no model of the vibration of a moving arm; the vibration model sits in front of the sensor, so it works with Gazebo's sensor too.
