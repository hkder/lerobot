# SO-101 handoff

Everything needed to continue this SO-101 leader/follower setup on another machine: code, calibration, known problems and the plan for running NVIDIA GR00T N1.7 on it.

Branch: `fix/feetech-open-position-limits` on `hkder/lerobot` (based on huggingface/lerobot `73ab8903`).

## Where things stand

| Part | Status |
|---|---|
| Both arms assembled, servo IDs set, calibrated | Done. Calibration files are in `so101/calibration/`. |
| Follower servos F2 and F4 moving the wrong way | Fixed in code on this branch (see "The F2/F4 fix"). The servos are fine; don't replace them. |
| Teleop | Works well with stiffness P=32 and no `max_relative_target`. |
| Cameras | Wrist (index 0) and top (index 1), both 1280×720 at 30 fps. |
| Rest pose | Saved in `so101/tools/rest.json`. The follower parks there after teleop and the tools. |
| Next | Record demos, fine-tune GR00T N1.7 on Brev, run it on DGX Spark. |

## Hardware

**Power: never swap the adapters.**

| Adapter | Board | Servos |
|---|---|---|
| 12V 5A | Follower (the arm with the gripper claw) | 6× STS3215 C018, 12V |
| 6V 3A | Leader (the arm with the handle and trigger) | STS3215 low-voltage (6–7.4V); L2 is a different, stronger type |

12V on the leader board can damage its servos.

- Servo IDs are 1–6 from the base up: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper. They're labeled F1–F6 and L1–L6.
- Use USB-C cables that carry data. A charge-only cable makes the board invisible.
- Plug the second camera straight into the computer. One bus-powered hub couldn't power two cameras.
- On a Mac, turn off the iPhone's Continuity Camera (Settings → General → AirPlay & Continuity). It renumbers the cameras.

## Files in `so101/`

| File | What it is |
|---|---|
| `so101.sh` | One command for everything. Run it with no arguments for the list. |
| `config.env` | Serial ports and camera numbers for this machine. **Edit this on a new machine.** |
| `calibration/` | Leader and follower calibration, copied into place by `so101.sh install-calibration`. |
| `tools/rest.json` | Saved rest pose of the follower. |
| `tools/*.py` | Helpers used by `so101.sh`: camera viewer, rest pose, one-joint calibration and teleop, logging, vision demo. |

## New laptop setup (the arms plug into this one)

Tested on macOS. Linux should work the same way; ports look like `/dev/ttyACM0`.

1. Install [uv](https://docs.astral.sh/uv/) and git, then:
   ```bash
   git clone -b fix/feetech-open-position-limits https://github.com/hkder/lerobot.git ~/lerobot
   cd ~/lerobot
   uv sync --locked --python 3.12 --extra dataset --extra feetech --extra viz --extra async
   ```
   Use Python 3.12. Python 3.14 breaks every `lerobot-*` command with `TypeError: str | None is not callable`.
2. Copy the calibration into place:
   ```bash
   so101/so101.sh install-calibration
   ```
3. Plug in both boards (12V follower, 6V leader) and both cameras, then find the ports:
   ```bash
   so101/so101.sh ports
   ```
   Unplug one board and run it again to see which port belongs to which board. Put them in `so101/config.env`. Check the camera numbers with `so101/so101.sh cameras`.
4. Check:
   ```bash
   so101/so101.sh check    # expect follower ~12V, leader ~6V, both "calibrated: yes"
   so101/so101.sh teleop
   ```

**macOS only:** if torchcodec fails to load (`Could not load libtorchcodec`), Homebrew's FFmpeg is too new. Run `brew install ffmpeg@8` and add `export DYLD_FALLBACK_LIBRARY_PATH=/opt/homebrew/opt/ffmpeg@8/lib` to `~/.zshrc`.

## Daily commands

```bash
so101/so101.sh teleop-cam                    # teleop with live camera view (Rerun)
so101/so101.sh cam                           # just the cameras, with real Hz
so101/so101.sh record <hf-user>/<dataset> "<task sentence>" 20
so101/so101.sh rest                          # park the follower, then relax it
so101/so101.sh relax                         # follower limp right away (hold it first)
```

Match the two arms' poses roughly before starting teleop. The follower goes straight to the leader's pose.

## The F2/F4 fix (why this branch exists)

**Symptom:** follower shoulder_lift (F2) and wrist_flex (F4) drove away from every target and stalled against the hard stop with an overload error.

**Cause:** the position limits that calibration writes into the servos (`Min/Max_Position_Limit`). With those limits cleared to 0–4095, both servos work. Put back, they fail again. Their motors, sensors and settings match the working servos. Upstream report: [huggingface/lerobot#3585](https://github.com/huggingface/lerobot/issues/3585#issuecomment-5816932404).

**Change on this branch:**
- `src/lerobot/motors/feetech/feetech.py`: `write_calibration` sets the servo limits to 0–4095 and still writes the homing offset. `is_calibrated` compares only the homing offsets.
- `src/lerobot/motors/motors_bus.py`: targets in degrees mode (the SO-101 default) are now clamped to the calibrated range, so the joints still can't be sent past their range.

On `main` without this change, F2 and F4 break again.

## Teleop tuning (measured)

| Setting | Shoulder average gap | Shoulder delay |
|---|---|---|
| P=16 with `max_relative_target=5` | 19° | 187 ms |
| **P=32, no `max_relative_target`** | **1°** | **78 ms** |

`max_relative_target=5` keeps the gap to the target so small that the shoulder can't produce enough force against gravity, so it stalls. The scripts use P=32 without it.

## GR00T N1.7 plan

```
record demos (laptop)  →  fine-tune (Brev GPU)  →  serve the model (DGX Spark)  →  robot acts (laptop)
```

**1. Record** about 20 demos of one task, 20 s each. Put the object in different spots.
```bash
uv run hf auth login
so101/so101.sh record <hf-user>/so101-pick-cube "Pick up the red cube and put it in the cup" 20
```

**2. Fine-tune on Brev.** Use an A100 or H100 instance. Settings are from `docs/source/groot.mdx`; `new_embodiment` is the setting for robots GR00T wasn't trained on, like the SO-101.
```bash
git clone -b fix/feetech-open-position-limits https://github.com/hkder/lerobot.git && cd lerobot
uv sync --locked --python 3.12 --extra groot --extra training
uv run hf auth login
uv run lerobot-train \
  --dataset.repo_id=<hf-user>/so101-pick-cube --dataset.image_transforms.enable=true \
  --policy.type=groot --policy.base_model_path=nvidia/GR00T-N1.7-3B \
  --policy.embodiment_tag=new_embodiment \
  --policy.chunk_size=16 --policy.n_action_steps=16 \
  --policy.use_relative_actions=true --policy.relative_exclude_joints='["gripper"]' \
  --policy.use_bf16=true --policy.device=cuda \
  --policy.push_to_hub=true --policy.repo_id=<hf-user>/so101-pick-cube-groot \
  --batch_size=64 --steps=20000 --save_freq=5000 --use_policy_training_preset=true \
  --output_dir=outputs/train/so101-pick-cube-groot
```
If there are only about 20 demos, fewer steps (5k–10k) may be enough. Then also set `--policy.scheduler_decay_steps` to the same number.

**3. Serve on DGX Spark:**
```bash
uv sync --locked --python 3.12 --extra groot --extra async
uv run python -m lerobot.async_inference.policy_server --host=0.0.0.0 --port=8080
```
Spark is ARM-based. GR00T on ARM hasn't been tested here, and one GR00T dependency (decord) only installs on x86. **Try this install early, before training.**

**4. Run the robot from the laptop:**
```bash
so101/so101.sh remote <spark-host>:8080 groot <hf-user>/so101-pick-cube-groot "Pick up the red cube and put it in the cup"
```
If the connection to Spark is slow (ping over 100 ms), raise `--chunk_size_threshold` in `so101.sh` from 0.5 to 0.7.

**Recording and running must match:** same cameras, same camera positions, same resolution, same task sentence.

## Open items

- The GR00T install on DGX Spark (ARM) isn't tested yet.
- The robot needs a flat baseboard (plywood or MDF, about 40×60 cm, 18 mm thick) with a non-slip mat. It slides on the rounded desk edge, which also shifts the camera views between recordings.
- The upstream issue (#3585) is waiting on a maintainer reply. Only open a PR if they want this change.
