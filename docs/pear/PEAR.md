# PEAR: Training Reasoning Models in Simulation to Natively Use Real Robots

**MIT Han Lab (MIT, UC Berkeley, UCSD, NVIDIA, Inco AI) · Nov 13, 2026 · arXiv:2611.18562 · Apache-2.0**

- Paper: <https://arxiv.org/abs/2611.18562>
- Code: <https://github.com/mit-han-lab/pear>
- Blog: <https://mit-han-lab.github.io/pear-blog/>
- Model weights: linked from the repo README

## TL;DR

An open-weight 27B reasoning model (Qwen3.8-27B) is post-trained with **Robot-Use RL (RURL)** entirely in simulation — no demonstrations, no harness, no planner, no skill library — and learns to drive real robots natively. The model behaves like a computer-use agent: it observes the environment, reasons in a chain of thought, and emits the next end-effector pose directly.

## Why it matters

- **The CUA playbook, applied to robots.** Frontier computer-use agents (GPT-6 Astra, Claude Fable 5.2) got good through RL on simulated environments. PEAR mirrors that recipe for robot control, and shows the resulting skills **transfer**: to unseen robots, to the real world, and to novel instructions.
- **Open-weight models close the gap.** Until now, strong robot-use performance belonged to frontier proprietary models. A 27B open model trained in sim approaches frontier performance on LIBERO at maximum reasoning effort.
- **Reasoning models vs VLA policies.** Compared with π0.5 (a VLA fine-tuned on demonstrations), the RURL-trained model is far more robust to *task* changes: on LIBERO-Pro, when assigned a new task in a familiar scene, π0.5 succeeds in only 0.75% of episodes vs **57.4%** for the PEAR-trained model.

## Results

| Evaluation | Base model | After RURL | Reference |
|---|---|---|---|
| LIBERO (sim) | 1.6% | **74.4%** | Approaches GPT-6 Astra / GPT-6.1 Sol at max reasoning effort |
| Real dual-arm YAM (unseen robot, real world) | 0/30 | **60% of trials** | RL on LIBERO + RoboDojo |
| Real YAM, LIBERO-only checkpoint | — | 23% | Transfers despite never seeing the robot |
| LIBERO-Pro (new task, familiar scene) | — | **57.4%** | π0.5: 0.75% |
| Nine reasoning/coding/vision benchmarks | — | No measurable degradation | EvalScope |

## System

PEAR is a post-training engine for agentic robotics, built on **veRL** and **LMCache**, using **vLLM** and **Ray**:

- **Model server:** vLLM with decode megakernels — one RTX 5090 or GB300 (Blackwell), or H100/H200 (Hopper).
- **Robot station:** the agent loop that drives the robot — **YAM** (two arms, with grippers) or **SO-101** (one arm, with gripper).
- **Training configs:** H100 — rollouts served with TP4, policy trained FSDP8 × SP8 (32,512 tokens/GPU); GB300 — TP2 serving, FSDP4 × SP4 (69,632 tokens/GPU); each node forms one FSDP group.

## Benchmarks shipped

- **Robotic:** LIBERO, LIBERO-Plus, LIBERO-Pro, RoboDojo, RoboLab-120
- **General:** nine reasoning, coding, and vision benchmarks through EvalScope

## Getting started

Install PEAR and everything a LIBERO run needs on a GPU node (see the repo README). Deployment is two pieces: the vLLM model server and the robot station running the agent loop.

## Context in the field

PEAR lands the same week as several data points that frame where robot learning is heading:

- **CoRL 2026 (Austin, Nov 9–12)** — the field's two bets on stage: Fei-Fei Li's world models vs Russ Tedrake's factory robots. Best Paper went to *SuperMimic: Scaling Humanoid Whole-Body Control with One Differentiable World Model*.
- **NVIDIA GR00T 2 (CoRL)** — 71% success across 50 tasks from one demo each, 95% from five demos (company-reported); Isaac Lab 3.0 adds the Newton solver.
- **Runway Praxis-1** — 27B world action model, open weights (Apache-2.0), 46.2% on RoboLab-120.
- **FLUX 3 Action (Black Forest Labs)** — 7B open-weights world action model; see the [World Action Models](../../README.md#world-action-models) section.

The through-line: RL post-training in simulation — whether of reasoning models (PEAR), world action models (Praxis-1), or humanoid control (SuperMimic) — is becoming the default recipe for capable robot behavior, with demonstrations playing a shrinking role.

## Related reading

- [Agentic robotics, LLMs, and robot policy — a primer (Voxel51)](https://voxel51.com/blog/agentic-robotics-llm-robot-policy)
- [MIT Han Lab — PEAR project page](https://hanlab.mit.edu)
