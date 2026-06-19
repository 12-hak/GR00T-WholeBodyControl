# Quest 3 arm teleop — sim2sim setup (Windows + WSL)

This guide covers **Meta Quest 3** arm teleoperation into GR00T **GEAR-SONIC sim2sim** on **Windows 11 + WSL2 Ubuntu 22.04**. Legs use the kinematic planner; arms use **VR 3-point** tracking over ZMQ (same wire format as Pico teleop).

## Architecture

```
Quest 3 (WebXR)  --HTTPS/WSS-->  quest3_manager_server.py  --ZMQ 5556-->  deploy (zmq_manager)  -->  MuJoCo
```

| Component | Path |
|-----------|------|
| WebXR client | `gear_sonic/utils/teleop/quest/webxr_client.html` |
| Teleop bridge | `gear_sonic/scripts/quest3_manager_server.py` |
| Pose / calib | `gear_sonic/utils/teleop/quest/quest_pose.py`, `quest_joint_calib.py` |
| WSL launcher | `install_scripts/run_quest3_teleop_wsl.sh` |
| Sim + deploy | `install_scripts/run_mujoco_sim_wsl.sh` |
| Port forward (Windows) | `install_scripts/setup_quest3_ports.ps1` |

**Ports**

| Port | Purpose |
|------|---------|
| 8766 | HTTPS page + WSS pose stream (Quest browser) |
| 5556 | ZMQ pub — planner + command (teleop → deploy) |
| 5557 | ZMQ feedback — robot joint state (deploy → teleop) |

---

## Prerequisites

- Windows 11 with **WSL2** (`Ubuntu-22.04`)
- Repo on a WSL-accessible path, e.g. `/mnt/d/python/GR00T-WholeBodyControl`
- **Quest 3** on the **same Wi‑Fi** as the PC
- Completed base sim2sim install: `.venv_sim`, deploy build, HuggingFace ONNX models, planner TRT (see main [README](../README.md) / install scripts)
- Teleop venv (created automatically on first teleop run, or manually below)

---

## One-time setup

### 1. Teleop Python environment (WSL)

```bash
cd /mnt/d/python/GR00T-WholeBodyControl
bash install_scripts/install_quest3_teleop_wsl.sh
```

Venv default: `~/.venv_gear_sonic_teleop`

### 2. Windows port forward (Administrator, once per machine)

Quest reaches WSL via Windows LAN IP. Forward **8766** from Windows → WSL:

```powershell
cd D:\python\GR00T-WholeBodyControl
powershell -ExecutionPolicy Bypass -File install_scripts\setup_quest3_ports.ps1
```

Note your LAN IP from the script output (e.g. `192.168.1.235`).

### 3. Optional: set LAN IP explicitly

If auto-detection is wrong:

```bash
export QUEST_HOST_IP=192.168.1.235   # in WSL before teleop
```

---

## Startup sequence (every session)

Use **three WSL terminals**. Order matters.

### Terminal 1 — MuJoCo simulation

```bash
cd /mnt/d/python/GR00T-WholeBodyControl
bash install_scripts/run_mujoco_sim_wsl.sh sim
```

When the sim window is up, press **`9`** to enable the elastic band (keeps the robot upright while learning balance).

### Terminal 2 — Deploy stack (ZMQ input)

```bash
cd /mnt/d/python/GR00T-WholeBodyControl
bash install_scripts/run_mujoco_sim_wsl.sh deploy-zmq
```

This runs `deploy.sh sim --input-type zmq_manager` with the locomotion planner. **Do not** use keyboard deploy mode for Quest teleop.

Wait until deploy reports it is listening for ZMQ.

### Terminal 3 — Quest teleop bridge

```bash
cd /mnt/d/python/GR00T-WholeBodyControl
bash install_scripts/run_quest3_teleop_wsl.sh
```

The script prints the Quest URL, e.g.:

```
https://192.168.1.235:8766/webxr_client.html?host=192.168.1.235
```

**Focus this terminal** for keyboard commands (`c`, `j`, `s`, `v`, …).

### Quest headset

1. Open the URL in the **Quest Browser** (not on PC — close duplicate tabs on PC).
2. Accept the **self-signed certificate** (Advanced → Proceed).
3. Tap **Passthrough** (or VR room).
4. Confirm the HUD shows WebSocket connected and stick values when you move thumbsticks.

---

## Calibration and enable tracking

Recommended first-time flow:

| Step | Where | Action |
|------|--------|--------|
| 1 | Terminal 3 | **`j`** — joint calibration wizard (~14 poses, see below) |
| 2 | Terminal 3 | **`s`** — start policy (planner idle) |
| 3 | Terminal 3 | **`v`** — enable arm VR 3-point tracking |
| 4 | Quest | Move controllers; head look should tilt upper body, arms follow controllers |

**Quick calib only:** **`c`** — 3 s countdown, hold T-pose, then **`v`**.

### Keyboard (Terminal 3)

| Key | Action |
|-----|--------|
| **`j`** | Joint calibration wizard (best arm/head alignment) |
| **`c`** | Quick T-pose calibration (3 s countdown) |
| **`s`** | Start policy (planner mode) |
| **`v`** | Arm VR_3PT tracking |
| **`p`** | Planner idle (arms not tracked) |
| **`o`** | Emergency stop |

**Quest controllers**

| Input | Action |
|-------|--------|
| Left stick | Move / strafe (relative to facing) |
| Right stick X | Turn in place |
| Left stick click | Enable VR_3PT (when in planner idle) |
| A + X (both hands) | Emergency stop |

### Joint calibration wizard (`j`)

Follow prompts in Terminal 3. Each step: **3 s countdown** → hold pose → auto-capture.

1. T-pose + look straight ahead  
2–6. Left arm: out, forward, up, back, down  
7–11. Right arm: same five directions  
12–14. Head: forward, down, up  

When finished, press **`s`** then **`v`**.

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| Quest cannot open page | Re-run `setup_quest3_ports.ps1` as Admin; PC and Quest on same Wi‑Fi |
| Certificate error | Accept on Quest; URL must use **https://** and your PC LAN IP |
| No pose data | Tap **Passthrough** on page; wake both controllers; hard-refresh (close Browser app) |
| `ZMQ port 5556 in use` | Kill stale teleop: `fuser -k 5556/tcp` in WSL |
| Keys ignored | Click Terminal 3 (teleop), not deploy/sim window |
| Arms wrong after code changes | Re-run **`j`** or **`c`**, then **`s`** → **`v`** |
| Stale teleop instance | Only one `quest3_manager_server.py` at a time |

---

## Files changed for Quest (for Git)

If you publish your fork, these are the main Quest-specific paths:

```
gear_sonic/scripts/quest3_manager_server.py
gear_sonic/utils/teleop/quest/
install_scripts/run_quest3_teleop_wsl.sh
install_scripts/install_quest3_teleop_wsl.sh
install_scripts/setup_quest3_ports.ps1
install_scripts/get_quest_lan_ip.ps1
docs/quest3_teleop.md
gear_sonic/scripts/pico_manager_thread_server.py   # FeedbackReader quiet= flag
```

Do **not** commit virtualenvs (`.venv_sim/`, `.venv_teleop/`) or large model binaries — see `.gitignore`.

---

## Publishing to GitHub

### Option A — Fork of NVIDIA repo

1. Fork [GR00T-WholeBodyControl](https://github.com/NVIDIA/GR00T-WholeBodyControl) on GitHub.
2. Add your fork as remote and push your branch:

```bash
cd /mnt/d/python/GR00T-WholeBodyControl
git remote add myfork git@github.com:YOUR_USER/GR00T-WholeBodyControl.git
git checkout -b quest3-teleop
git add gear_sonic/utils/teleop/quest/ gear_sonic/scripts/quest3_manager_server.py \
        install_scripts/run_quest3_teleop_wsl.sh install_scripts/install_quest3_teleop_wsl.sh \
        install_scripts/setup_quest3_ports.ps1 install_scripts/get_quest_lan_ip.ps1 \
        docs/quest3_teleop.md
git commit -m "Add Quest 3 WebXR arm teleop for sim2sim"
git push -u myfork quest3-teleop
```

3. Open a Pull Request on GitHub, or keep the branch on your fork.

### Option B — New repository (your copy only)

1. Create an empty repo on GitHub (no README if you already have one locally).
2. Point your local repo at it:

```bash
git remote add origin git@github.com:YOUR_USER/YOUR_REPO.git
git branch -M main
git push -u origin main
```

Use **Git LFS** for large assets if you track model files (`git lfs install` — already used in this project for some assets).

### SSH vs HTTPS

- SSH: `git@github.com:USER/REPO.git` (requires [SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh))
- HTTPS: `https://github.com/USER/REPO.git` (use a [Personal Access Token](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token) instead of password)

### Before pushing

```bash
git status          # review changes
git diff            # review diffs
```

Exclude secrets, `.env`, and local venv directories. Respect the upstream **Apache 2.0** license if you fork NVIDIA’s repo.

---

## Related docs

- Main project: [README.md](../README.md)
- Official teleop tutorials: [GR00T-WholeBodyControl docs](https://nvlabs.github.io/GR00T-WholeBodyControl/)
