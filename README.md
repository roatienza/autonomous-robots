# Autonomous Robots

Lecture Notes and Experiments on Autonomous Robots

| **Topic** | **Note** | **Code** |
| :--- | :---: | :--- |
| ROS2 | [PDF](https://drive.google.com/file/d/1LaWyp9g2lKIGXSB5wsFIJdPkhkGgJtfK/view?usp=sharing) | [ROS2](ros2) |
| Navigation | [PDF](https://drive.google.com/file/d/1UzeNSkWWHATLXmAEcyfabvVfhwmRMu5V/view?usp=sharing) | - |
| Localization | [PDF](https://drive.google.com/file/d/1nyhgjG2B07__5PhBgBXe9yiMRfqCk8Bm/view?usp=sharing) | - |
| Perception | [PDF](https://drive.google.com/file/d/1tZcBpqq-5jJ2vID9ryoN6sflCSZ9sJsk/view?usp=sharing) | - |
| Visual Geometry | [PDF](https://drive.google.com/file/d/1mW31Csyx3tbVoknNvRaBG17z1x5YXhbv/view?usp=sharing) | - | 
| Kinematics | [PDF](https://drive.google.com/file/d/1BGR3RCgn05K2cvbFuy3s8DrGYGJIbtZS/view?usp=sharing) | - |
| Trajectory Planning | [PDF](https://drive.google.com/file/d/1k-yoVY_Dtodx9sMfIQIB4mRo2bJb5U6_/view?usp=sharing) | - |
| Diffusion and Flow Matching Applied to Behavior Cloning | [PDF](https://drive.google.com/file/d/16nYQnJlKJMPRRhzOSQyBAMCq_Vb3OWeS/view?usp=sharing) | - |
| Vision Language Action | [PDF](https://drive.google.com/file/d/16DkjUin-H7kM-TYA_RxHv2txZeEP07EX/view?usp=sharing) | - |
| World Models | [PDF](https://drive.google.com/file/d/1mQJ5EZiMQ-u3dqR98dQXwDwgKQ5iyhWA/view?usp=sharing)| - |

## Weekly Digest

Tracking the week in robotics + AI — news, products, announcements, research breakthroughs, papers, and open-source projects. One file per ISO week under [`docs/weekly/`](docs/weekly/), newest first.

| Week | Digest | Highlights |
| :--- | :--- | :--- |
| 2026-W46 (Nov 9–15) | [docs/weekly/2026-W46.md](docs/weekly/2026-W46.md) | NVIDIA CEO transition · PEAR (MIT Han Lab) trains a 27B reasoning model in sim to drive real robots · Gemini Robotics 2.5 at 96% on live Mercedes lines · GXO signs for up to 1,000 Apollo humanoids · CoRL 2026: SuperMimic best paper, GR00T 2 |
| 2026-W45 (Nov 2–8) | [docs/weekly/2026-W45.md](docs/weekly/2026-W45.md) | Meituan-Robotics-0 and Open-HumanWAM foundation models · agentic manipulation policies · humanoid soccer, parkour, and minutes-fast locomotion (STRIDE) · dexterous-hands survey |

## Books

- [Introduction to Autonomous Robots](https://github.com/Introduction-to-Autonomous-Robots/Introduction-to-Autonomous-Robots)
- [Introduction to Autonomous Mobile Robots](https://www.ucg.ac.me/skladiste/blog_13268/objava_56689/fajlovi/Introduction%20to%20Autonomous%20Mobile%20Robots%20book.pdf)
- [Reinforcement Leanring: An Overview by Kevin Murphy](https://arxiv.org/abs/2412.05265)
- [SLAM Handbook](http://asrl.utias.utoronto.ca/~tdb/slam/)
- [Principles of Robot Autonomy](https://arxiv.org/abs/2608.03496) :star: — Stanford book (Gammelli, Lorenzetti, Luo, Zardini, Pavone), 531 pp., forthcoming from Cambridge University Press: a unified, deployment-aware introduction to modern autonomy stacks bridging classical robotics and physical AI, with hands-on Jupyter notebooks and implementation-driven exercises for every major topic

## Lectures

- [MIT 16.485 — Visual Navigation for Autonomous Vehicles (VNAV)](https://vnav.mit.edu/) — mathematical foundations of visual navigation (geometry to optimization), state-of-the-art algorithms, and software packages
- [Robot Learning by ETH](https://cvg.ethz.ch/lectures/Robot-Learning/)
- [LIBERO](https://libero-project.github.io/main.html)
- [University of Texas-Dallas](https://labs.utdallas.edu/irvl/courses/fall-2025-cs-6341-robotics/)
- [Flow Matching and Diffusion Models](docs/diffusion_flow_matching/LECTURE_NOTES.md) :star:

## Blogs
- [3D Perception by NVIDIA](https://developer.nvidia.com/blog/r2d2-building-ai-based-3d-robot-perception-and-mapping-with-nvidia-research/)
- [Robotics from First Principles](https://interlatent.com/blog/interlatent-modern-ai-robotics-first-principles)
- [Frontier System for Physical](https://www.a16z.news/p/frontier-systems-for-the-physical)

### Affordance

- [Learning Affordances at Inference-Time for Vision-Language-Action Models](https://arxiv.org/pdf/2510.19752)
- [RT-Affordance: Affordances are Versatile Intermediate Representations for Robot Manipulation](https://arxiv.org/abs/2411.02704)

### Agents in Robotics

- [RoboGen](https://github.com/Genesis-Embodied-AI/RoboGen)
- [OpenMind](https://openmind.org/)
- [Ark Framework](https://github.com/Robotics-Ark/ark_framework)
- [PEAR](https://github.com/mit-han-lab/pear) :star: — a Post-training Engine for Agentic Robotics (MIT Han Lab). Trains an open 27B reasoning model with Robot-Use RL entirely in simulation, without demonstrations or a harness; the model observes, reasons in chain-of-thought, and emits the next end-effector pose directly. LIBERO 1.6% → 74.4%; 60% success on a real dual-arm YAM (base: 0%); LIBERO-Pro 57.4% vs π0.5's 0.75%; general reasoning/coding/vision intact. Built on veRL + LMCache, vLLM + Ray; runs on RTX 5090/GB300 or H100/H200; drives YAM and SO-101 arms. Deep dive: [docs/pear/PEAR.md](docs/pear/PEAR.md)

### AutoResearch - Recursive Intelligence

- [Sakana AI Scientist](https://sakana.ai/ai-scientist-nature/)
- [AutoResearch](https://github.com/karpathy/autoresearch)
- [AutoResearchClaw](https://github.com/aiming-lab/AutoResearchClaw)
- [Sakana RSI Lab](https://sakana.ai/rsi-lab/)
- [Hierarchical Reasoning Model](https://github.com/sapientinc/HRM/)
- [First Steps Toward Automated AI Research](https://www.recursive.com/articles/first-steps-toward-automated-ai-research)

### Data Visualization

- [`rerun.io`](https://rerun.io/)
  
### Datasets

- [EgoDex](https://arxiv.org/pdf/2505.11709)
- [Princeton Digital Assets](https://princeton-vl.github.io/infinigen-sim/)
- [GraspGen](https://graspgen.github.io/)
- [ShareGPT-4o-Image](https://github.com/FreedomIntelligence/ShareGPT-4o-Image)
- [Wan Video Generation](https://github.com/Wan-Video/Wan2.2?tab=readme-ov-file)
- [RoboGen](https://arxiv.org/pdf/2311.01455)
- [Large Scale Grasp Dataset](https://research.nvidia.com/publication/2021-05_acronym-large-scale-grasp-dataset-based-simulation)
- [Rosario Agricultural Dataset](https://cifasis.github.io/rosariov2/)
- [Project Go-Big](https://www.figure.ai/news/project-go-big)
- [Behavior Challenge by Stanford](https://behavior.stanford.edu/index.html)
- [BridgeData V2: A Dataset for Robot Learning at Scale](https://rail-berkeley.github.io/bridgedata/)
- [Robocasa](https://robocasa.ai/) :star2:
- [Egocentric Dataset](https://huggingface.co/datasets/builddotai/Egocentric-10K)
- [Hesai Kosmo spatial camera (Calder Brief)](https://calder-brief.pages.dev/hesai-kosmo-spatial-camera-october-2026) - Robotic vision hardware for robot perception
- [ACRONYM: A Large-Scale Grasp Dataset Based on Simulation](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=9560844&casa_token=vgC8MR5HFdMAAAAA:75QBFeoIA_gzxBaQboj7SI5pIVsGL1uCRCeQOUo7QUypqqP8LUF8-IK5EIzTMqCuFqIx0BVBISgC&tag=1)
- [Spatial AI](https://huggingface.co/datasets/spatial-ai/sea-small)
- [In-N-On Scaling Egocentric manipulation with in-the-wild and on-task data](https://xiongyicai.github.io/In-N-On/)
- [Articraft-10K: Agentic System for Scalable Articulated 3D Asset Generation](docs/dataset/ARTICRAFT3D.md) :star:
- [HumanNet: Scaling Human-Centric Video Learning to One Million Hours](docs/dataset/HUMANNET.md) :star:
- [GR00T Dataset](https://huggingface.co/datasets/nvidia/PhysicalAI-Robotics-GR00T-X-Embodiment-Sim) :star:
- [Scalable Behavior Cloning with Open Data, Training, and Evaluation](https://abc.bot/)
- [LightwheelAI EgoDemo](https://huggingface.co/datasets/LightwheelAI/EgoDemo) :star: — 50-hour developer sample of EgoSuite-Open100K (100K hours, 15,000+ tasks/scenes), the largest fully-annotated open egocentric human dataset: hand + full-body pose, semantic annotations, in LeRobot v3 / MCAP / raw MP4 (gated access)

### Depth Models

- [Depth Anything](https://arxiv.org/pdf/2511.10647)

### Development Kit

- [SunFounder](https://www.sunfounder.com/collections/robotics/products/picar-x)
- [Isaac NVIDIA](https://nvidia-isaac-ros.github.io/getting_started/index.html)

### Diffusion in Robotics

- [Diffusion Policy: Visuomotor Policy Learning via Action Diffusion](https://arxiv.org/pdf/2303.04137)
- [Diffusion vs Autoregressive in Robotics](https://blog.ml.cmu.edu/2025/09/22/diffusion-beats-autoregressive-in-data-constrained-settings/)

### Development Kits for Robotics

- [Segway Nova Carter](https://robotics.segway.com/%20nova-carter/)
- [Muse Gadgets](https://gadgets.muse.ai/) — open source hardware for your Muse: program an off-the-shelf ESP32 board or set up a Raspberry Pi with their SDKs, then connect Muse to your displays, buttons, sensors, actuators, and whatever else you've got lying on your workbench; it's built by hackers, for hackers, just for fun — side effects of tinkering may include bricked boards, voided warranties, brownouts, or bankruptcies, so proceed at your own risk; featured builds include a Raspberry Pi 5 working with Home Assistant and other Linux apps, a Seeed reTerminal E1002 color e-ink display for morning briefings, reminders, and shopping lists, and a plug-in TV stick (coming soon); the ready-made [Muse Home Link](https://gadgets.muse.ai/home-link) — included with an active Muse subscription, US only, one per subscriber — connects Muse to compatible home devices over Wi-Fi and local HTTP APIs — it's a USB-C stick (35 × 42 × 10 mm, powers off USB-C or USB-A) built on an ESP32-C5 — 32-bit RISC-V at 240 MHz, 8 MB PSRAM, 8 MB flash, dual-band Wi-Fi 6 (2.4 and 5 GHz), status LED for pairing and device state — that runs official firmware based on the open ESP32 Device SDK and can't be reflashed; plug it into any USB-C or USB-A power source near your router and pair it over Bluetooth LE in the Muse app, choosing the Wi-Fi network it joins; community-built skills teach it new tricks — turn on a light, control your TV, send a document to a printer — and the [skills catalog](https://github.com/facebookincubator/muse-gadget-sdk/tree/main/skills) covers Philips Hue, Sonos, Apple TV, Google Nest speakers, and Samsung TVs, plus Roku, robot vacuums, and more ([order on muse.ai](https://muse.ai/?settings_tab=general&pane=home-link) — claiming holds your place in line, and Meta will notify you when it ships); skills can change or break, so don't rely on them for anything safety-critical like home security, emergencies, or medical needs; every gadget pairs with an [SDK token](https://gadgets.muse.ai/settings/sdk-tokens) (review the [Gadget SDK Terms](https://gadgets.muse.ai/sdk-terms) first — tokens are for personal, non-commercial use on your own Muse account, one token covers at most 50 devices, you can't embed one in anything you sell or list publicly, and they're not a supported product: they may change, stop working, or be withdrawn at any time; hand a build to someone else and you must tell them what it does with their data before they pair it); gadget builders share projects in [#projects on the community Discord](https://discord.gg/tkpSAd9MJ) ([main invite](https://discord.gg/3bhjCkZdd6), 325 members online as of 20 Nov 2026) — featured ones get linked to their repo, and posts want a photo or short video, a line or two about the build, and a GitHub repo with a README ready; devices shown on the site are made and sold by other companies — Meta doesn't endorse or warrant them, and names and logos belong to their owners.
- [Muse Gadget SDK](https://github.com/facebookincubator/muse-gadget-sdk) — open source ESP32 and Linux SDKs behind Muse Gadgets (Apache 2.0): screens, audio in/out, sensors, Home Assistant on a Raspberry Pi, and your own commands ([ESP32 Device SDK](https://github.com/facebookincubator/muse-gadget-sdk/tree/main/esp32), [Linux Device SDK](https://github.com/facebookincubator/muse-gadget-sdk/tree/main/linux)); the gadgets page features the Raspberry Pi 5, Waveshare ESP32-S3-Touch-AMOLED-1.75C, Seeed reTerminal E1002, M5Stack StickS3, AiPi Lite, ideaspark ESP32 1.9", Home Assistant Voice PE, and a plug-in TV stick (coming soon) — or build support for an entirely new board; buy links on the gadgets page: [Pi 5](https://www.walmart.com/ip/Raspberry-Pi-5-SC1160-8GB-RAM-Single-Board-Computer/5820957894), [Waveshare](https://www.waveshare.com/esp32-s3-touch-amoled-1.75c.htm), [reTerminal E1002](https://www.seeedstudio.com/reTerminal-E1002-p-6533.html), [StickS3](https://shop.m5stack.com/products/m5sticks3-esp32s3-mini-iot-dev-kit), [AiPi Lite](https://aipi.com/products/aipi-lite), [ideaspark](https://www.amazon.com/dp/B0D6QXC813), [HA Voice PE](https://www.home-assistant.io/voice-pe/) — each card's Source link drops straight into that board's config in the SDK tree ([Pi 5](https://github.com/facebookincubator/muse-gadget-sdk/tree/main/linux), [Waveshare](https://github.com/facebookincubator/muse-gadget-sdk/blob/main/esp32/devices/sdkconfig.muse-waveshare-s3-175c), [reTerminal E1002](https://github.com/facebookincubator/muse-gadget-sdk/blob/main/esp32/devices/sdkconfig.reterminal-e1002), [StickS3](https://github.com/facebookincubator/muse-gadget-sdk/blob/main/esp32/devices/sdkconfig.muse-m5stack-sticks3), [AiPi Lite](https://github.com/facebookincubator/muse-gadget-sdk/blob/main/esp32/devices/sdkconfig.muse-aipi), [ideaspark](https://github.com/facebookincubator/muse-gadget-sdk/blob/main/esp32/devices/sdkconfig.ideaspark), [HA Voice PE](https://github.com/facebookincubator/muse-gadget-sdk/blob/main/esp32/devices/sdkconfig.home-assistant-voice)) — while the ESP32 SDK builds for ~45 boards, quickest start the ESP32-C5 DevKitC-1, which works as-is with its built-in status light and BOOT button; on boards with the home-network tunnel, Muse can reach devices you already own and anything you build with a local HTTP API; the fastest way to build is [Muse Code](https://developer.meta.com/ai/lp/muse-code/) — install with `curl -fsSL https://dev.meta.ai/install.sh | sh`, run `muse --disable-sandbox` from the esp32 directory, and ask it to build and flash (`--disable-sandbox` lets it reach the board's USB serial port and download the ESP-IDF v6.0.3 toolchain; it reads the AGENTS.md, and any agent that reads AGENTS.md works too); a build without a token can get one over serial (`>sdk_token=mgst_…`, stored in NVS) — the [Muse Gadget Web Flasher](https://facebookincubator.github.io/muse-gadget-sdk/) does this after installing (Chrome or Edge on desktop, no ESP-IDF setup); pair gadgets in the Muse iOS/Android app under Settings > Devices with Developer mode on — they show up prefixed "MuseGadget"; the Muse Gadgets Test Android/iPhone apps stream your phone's camera or a clip to an ESP32 gadget's screen over Wi-Fi (H.264 or VP8 decoded on the board, 30 fps on the round 466 px boards — [protocols/VIDEO.md](https://github.com/facebookincubator/muse-gadget-sdk/blob/main/protocols/VIDEO.md) explains how, [protocols/DEVICE.md](https://github.com/facebookincubator/muse-gadget-sdk/blob/main/protocols/DEVICE.md) how a gadget describes itself to Muse); an on-device "Hey Muse" wake word is in the tree but off by default and not enabled on any board yet (ExecuTorch + ESP-NN, `hey_muse.pte`, `MUSE_WAKEWORD_ENABLE`); battery-input power profiling rides a PPK2 and a board-neutral diagnostic core that doesn't pair or need a token (`power_sweep.py`); each SDK directory ships a README plus an AGENTS.md for coding agents like Muse Code.

### Embodied AI

- [Embodied AI Paper List](https://github.com/HCPLab-SYSU/Embodied_AI_Paper_List)

### Evaluation

VLA evaluation is moving beyond LIBERO success-rate leaderboards toward a broader stack: unified sim-benchmark harnesses, real-robot production metrics, and independent real-world evaluation.

- [How to Evaluate General-Purpose Robot Policies for Real-World Deployment (NVIDIA)](https://developer.nvidia.com/blog/how-to-evaluate-general-purpose-robot-policies-for-real-world-deployment/?mkt_tok=MTU2LU9GTi03NDIAAAGjMUJWoLLaUr36zBYdUdVaQp7E9DYrGprhdCskquGRaZv-mE60_tkkemeRjSTQ5A-r4AimuJURPVVqALdK3IdPaHvFDfg4Lsx6NucolkhuBjkvMThlyuEI)
- [VLA Evaluation Harness (Allen AI)](https://github.com/allenai/vla-evaluation-harness) — one framework to evaluate any VLA model on any robot simulation benchmark: 18+ benchmarks (LIBERO, SimplerEnv, CALVIN, ManiSkill2, RoboCasa, RoboTwin, RLBench, ...) behind a single interface, with a [VLA leaderboard](https://allenai.github.io/vla-evaluation-harness/leaderboard/).
- [LeRobot Evaluation](https://huggingface.co/docs/lerobot/en/adding_benchmarks) — `lerobot-eval` gives one evaluation interface across multiple sim benchmarks, each wrapped as a Gymnasium environment behind a standard `gym.Env` interface.
- [PhAIL — Physical AI Leaderboard](https://phail.ai) — real-robot benchmark (Franka FR3) scored on production metrics like throughput and failures; distributional methodology (time-to-success CDF, Human-Relative Throughput, KS significance tests). [Paper](https://arxiv.org/abs/2605.29710)
- [Robocurve](https://robocurve.org) — independent, real-world robot evaluation: open-source [Inspect Robots](https://github.com/robocurve/inspect-robots) framework (any model × any embodiment × any benchmark, with full trace logs and Rerun visualization) plus the World Evals benchmark catalog.
- [RoboDojo](https://github.com/RoboDojo-Benchmark/RoboDojo) — brings sim + real-world evaluation together: 42 sim tasks and 18 real-world tasks across 3 embodiments, five capability dimensions, heterogeneous parallel simulation in Isaac Sim, and a reproducible RealEval system. [Paper](https://arxiv.org/abs/2607.04434)

### Foundation Models

- [Perpeption Foundation Model](https://ai.meta.com/blog/meta-fair-updates-perception-localization-reasoning/)
- [Vision Language Model Book](https://books.google.com.ph/books?hl=en&lr=&id=EnqCEQAAQBAJ&oi=fnd&pg=PA228&ots=gd6NsKrxmc&sig=xrZ7-mB00G-w_QyRlAXGEhfLNAU&redir_esc=y#v=onepage&q&f=false)
- [Streaming VLM - Han Lab MIT](https://github.com/mit-han-lab/streaming-vlm)

### GPU Programming

- [Modern GPU Programming For MLSys](https://mlc.ai/modern-gpu-programming-for-mlsys/)

### Grasping

- [FoundationGrasp and GraspGPT](https://github.com/mkt1412/GraspGPT_public)
- [Visual Dexterity: In-Hand Reorientation of Novel and Complex Object Shapes](https://taochenshh.github.io/projects/visual-dexterity)
- [AffordGrasp](https://arxiv.org/pdf/2503.00778)
- [Grasp Diffusion](https://github.com/robotgradient/grasp_diffusion)
- [Open-Vocabulary Affordance Detection in 3D Point Clouds](https://arxiv.org/pdf/2303.02401)
- [GraspGen](https://graspgen.github.io/)


### Humanoid 

- [OpenArm - Open Source Humanoid](https://github.com/enactic/OpenArm)
- [All Humanoids as of Oct 2025](https://spectrum.ieee.org/video-friday-robotic-hands-2674168909)
- [Awesome Humanoid Learning](https://github.com/jonyzhang2023/awesome-humanoid-learning)
- [WUJI Hand as used by Genesis AI](https://wuji.tech/en/) 
- [Isaac GROOT](https://github.com/NVIDIA/Isaac-GR00T)
- [Noetix Robotics](https://en.noetixrobotics.com/)
- [Humanoid Robots - Not Solution](https://www.forbes.com/sites/trondarneundheim/2025/12/01/humanoid-robots-wont-save-manufacturing-heres-what-will/)

### Inference Pipeline

- [DeepStream8](https://developer.nvidia.com/blog/build-a-real-time-visual-inspection-pipeline-with-nvidia-tao-6-and-nvidia-deepstream-8/)

### Memory for Robotics

- [RoboMME: A Large-Scale Standardized Benchmark for VLA Memory](docs/benchmarks/ROBOMME.md) :star:
- [Enter the Mind Palace: Reasoning and Planning for Long-term Active Embodied Question Answering](https://arxiv.org/pdf/2507.12846)
- [NVIDIA's ReMEmbR](https://nvidia-ai-iot.github.io/remembr)
- [FindingDory](https://arxiv.org/pdf/2506.15635)
- [VLAs Memory](https://www.pi.website/research/memory)
- [Observational Memory](https://mastra.ai/docs/memory/observational-memory)

### Motion Retargeting and Prediction

- [General Motion Retargeting](https://github.com/YanjieZe/GMR)
- [Imitating Humans](https://kimhanjung.github.io/UniSkill/)
- [MolmoMotion: Language-guided 3D motion forecasting](https://huggingface.co/blog/allenai/molmomotion)

### Neural Robots

- [Neural Robot Dynamics](https://arxiv.org/pdf/2508.15755)

### Pointing

- [Molmo2 AllenAI](https://playground.allenai.org/?model=molmo2-8b)



### Reinforcement Learning Framework

- [EasyR1](https://github.com/hiyouga/EasyR1)
- [R-Zero](https://github.com/Chengsong-Huang/R-Zero)
- [verl: Volcano Engine Reinforcement Learning for LLMs](https://github.com/volcengine/verl)
- [UWLab](https://github.com/UW-Lab/UWLab) and [Omni-Reset](https://weirdlabuw.github.io/omnireset/)
- [Reinforcement Learning Book by K Murphy](https://arxiv.org/pdf/2412.05265)
- [Robot-Use RL (RURL) — PEAR](https://github.com/mit-han-lab/pear) — RL post-training of a reasoning model for native robot use, entirely in simulation without demonstrations; see [docs/pear/PEAR.md](docs/pear/PEAR.md)
- [Agentic RL](https://cameronrwolfe.substack.com/p/agentic-rl)
- [NVIDIA Molt Agent RL Framework](https://github.com/NVIDIA-NeMo/labs-molt.git) ⭐

### Robots (Commercially Available)

- [YAM](https://i2rt.com/products/yam-6-dof-arm)
- [SO-101](https://github.com/huggingface/lerobot/tree/main/examples) — single-arm robot used for PEAR deployment runs

### Rodney Brooks Essays

- [Why Today’s Humanoids Won’t Learn Dexterity](https://rodneybrooks.com/why-todays-humanoids-wont-learn-dexterity/)

### Simulation

- [The State of Simulation for Physical AI: An Overview (NVIDIA)](https://huggingface.co/blog/nvidia/state-of-simulation-for-physical-ai)
- [Habitat](https://github.com/facebookresearch/habitat-lab/)
- [Newton](https://developer.nvidia.com/newton-physics)
- [Viral Humanoid](https://viral-humanoid.github.io/)
- [Molmo-Bot](https://allenai.github.io/MolmoBot/)
- [Real2Sim2Real-SimFoundry](https://research.nvidia.com/labs/gear/simfoundry/)
- [Genesis World (supports touch/tactile](https://github.com/Genesis-Embodied-AI/genesis-world)



### Touch

Detailed summaries and resources can be found in the [Touch Documentation](docs/touch/README.md)
- [Awesome Touch](docs/touch/Awesome_Touch.md) :star:
- [Spatially-anchored Tactile Awareness for Robust Dexterous Manipulation](https://arxiv.org/html/2510.14647v1)
- [Intrinsic sense of touch for intuitive physical human-robot interaction](https://www.science.org/doi/10.1126/scirobotics.adn4008)
- [LingBot-Touch](https://robottoday.com/article/robottoday-weekly-november-9-13-2026) (Ant Group Robbyant) — open-sourced tactile AI model for robot hands, trained on ~5,000 hours of recordings across three tactile sensor types and 26 real-world tasks on two hands


### Tracking

- [3D Object Tracking](https://light.princeton.edu/publication/inverse-rendering-tracking/)
- [Person Tracking (Sports)](https://colab.research.google.com/github/roboflow-ai/notebooks/blob/main/notebooks/basketball-ai-how-to-detect-track-and-identify-basketball-players.ipynb)
  

### Vision

- [Dinov3](https://ai.meta.com/blog/dinov3-self-supervised-vision-model)
- [Map Anything](https://map-anything.github.io/)

### Vision Language Action (VLA)

- [Gemini Robotics](https://arxiv.org/pdf/2510.03342)
- [OpenPi](https://github.com/Physical-Intelligence/openpi)
- [State of VLA 2026](https://mbreuss.github.io/blog_post_iclr_26_vla.html)
- [RynnBrain by Alibaba](https://arxiv.org/pdf/2602.14979v1)
- [Qwen-VLA](https://arxiv.org/abs/2605.30280)

### World Models

Detailed summaries and resources can be found in the [World Models Documentation](docs/world_models/README.md)
- [ESI-Bench: Embodied Spatial Intelligence Benchmark](docs/benchmarks/ESI_BENCH.md) :star:

### World Action Models

- [FLUX 3 Action by Black Forest Labs](https://bfl.ai/models/flux-3-action) — a 7B **World Action Model** for robot control that predicts actions jointly with video. Trained across images, video, and audio for broader generalization than action demonstrations alone, then adapted via joint video-action training and finetuning for a target embodiment and action space (e.g. the 50-dim EE50 end-effector space). Published with the report and weights to make the recipe transparent — from pretraining/midtraining to finetuning and inference optimizations for local deployment. Beats the strongest open VLA (π0.5, 28.0%) and the Cosmos 3 Nano WAM (36.8%) on RoboLab, and is far cheaper to run than a WAM (Cosmos 3 Nano FP8 needs ~4.7x the per-second processing of π0.5 BF16 on a B200). A hybrid pairing with the Astra frontier reasoning model reaches 90% episode success at $8.77 and 8 min per success — 29% cheaper and 40% faster than the best alternative, up to 53.64% more success per dollar.
- [Pretrained to Imagine, Fine-Tuned to Act: The Rise of World Action Models](https://developer.nvidia.com/blog/pretrained-to-imagine-fine-tuned-to-act-the-rise-of-world-action-models/)
- [World Action Models: A Survey](https://arxiv.org/pdf/2606.20781) — *Shen et al., 2026* (57 pages). A comprehensive survey clarifying the boundaries among world models, video generation models, action-grounded video world models, VLA policies, and World Action Models (WAMs). WAMs are **embodied predictive-action models** that forecast the future to inform action. The survey organizes existing work through two views: (1) **what each method generates** — rendered futures, latent futures, or video-generation-free action reasoning; and (2) **architectural decomposition** by predictive substrate, backbone, action coupling, and deployment regime. Key themes include interactability, causality, persistence, physical plausibility, and generalization. The emerging design pattern: WAMs are generating less of the future while preserving what control requires, trading representational richness against compute, memory, latency, and action-label cost. [Homepage](https://world-action-models.github.io/)
- [Praxis-1 (Runway)](https://robottoday.com/article/robottoday-weekly-november-9-13-2026) — 27-billion-parameter world action model, open weights under Apache 2.0; 46.2% on NVIDIA's RoboLab-120 leaderboard
