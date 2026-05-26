# RoboMME: Benchmarking and Understanding Memory for Robotic Generalist Policies

RoboMME is a large-scale, cognitively motivated robotic benchmark designed to evaluate and advance Vision-Language-Action (VLA) models in long-horizon, history-dependent manipulation scenarios.

## 📌 Overview
Memory is critical for robotic tasks that involve counting repeated actions or manipulating objects that become temporarily occluded. While VLA models have begun incorporating memory, evaluations have been narrow and non-standardized. RoboMME provides a standardized testbed to systematically understand and compare memory mechanisms.

### Key Statistics
- **Tasks**: 16 diverse long-horizon manipulation tasks.
- **Data**: 1,600 demonstrations, totaling 770k high-quality training timesteps.
- **Backbone**: Built upon the $\pi_{0.5}$ architecture.
- **Model Suite**: 14 memory-augmented VLA variants.

---

## 🧠 Memory Taxonomy
Drawing from cognitive theories of human memory, RoboMME categorizes memory into four dimensions, each with a corresponding task suite:

| Memory Type | Task Suite | Primary Demand | Example Task |
| :--- | :--- | :--- | :--- |
| **Temporal** | Counting | Accumulating and reasoning over past events. | Counting placed green cubes and stopping. |
| **Spatial** | Permanence | Tracking object locations under occlusion/change. | Resolving mask after cubes swap positions. |
| **Object** | Reference | Identifying objects under varied referential cues. | Recalling a briefly highlighted cube. |
| **Procedural** | Imitation | Reproducing previously demonstrated motion patterns. | Replicating a demonstrated trajectory. |

![Task Category](robomme_media/task_category.png)
![Task Memory Correspondence](robomme_media/task_memory_correspondence.jpg)

---

## 🛠️ Model Design & Architecture
The project explores various memory representations and integration strategies to determine the most effective way to augment VLA models.

### 1. Memory Representations
- **Symbolic Memory**: Uses language subgoals concatenated with task instructions (non-differentiable).
- **Perceptual Memory**: Represents history as visual features (e.g., multi-frame tokens, memory banks).
- **Recurrent Memory**: Compresses context into fixed-size latent states via recurrent models.

### 2. Integration Strategies
- **Memory-as-Context**: Appends memory embeddings to inputs for joint processing.
- **Memory-as-Modulator**: Conditions the action expert via adaptive LayerNorm (AdaLN).
- **Memory-as-Expert**: Adds a dedicated memory expert interacting via block-wise causal attention.

![Model Design](robomme_media/model_design.jpg)
![FLOPs Comparison](robomme_media/flops_compare.jpg)

---

## 📈 Results & Findings
The experimental results indicate that **no single memory design is universally superior**; effectiveness is highly task-dependent.

- **Symbolic Memory** excels at counting and short-horizon reasoning.
- **Perceptual Memory** is critical for time-sensitive and motion-centric behaviors.
- **Optimal Balance**: The combination of **Perceptual Memory** with the **Memory-as-Modulator** design achieved the best balance between performance and computational efficiency.

![MME-VLA Suite Results](robomme_media/MME-VLA-suite-results.png)

---

## 🎥 Demonstrations

### Introduction
![Intro](robomme_media/intro.gif)

### Task Suite Highlights
| Task | Demonstration |
| :--- | :--- |
| **Bin Fill** | ![Bin Fill](robomme_media/BinFill.gif) |
| **Pick X Times** | ![Pick X Times](robomme_media/PickXtimes.gif) |
| **Swing X Times** | ![Swing X Times](robomme_media/SwingXtimes.gif) |
| **Stop Cube** | ![Stop Cube](robomme_media/StopCube.gif) |
| **Video Unmask** | ![Video Unmask](robomme_media/VideoUnmask.gif) |
| **Button Unmask** | ![Button Unmask](robomme_media/ButtonUnmask.gif) |
| **Video Unmask Swap** | ![Video Unmask Swap](robomme_media/VideoUnmaskSwap.gif) |
| **Button Unmask Swap** | ![Button Unmask Swap](robomme_media/ButtonUnmaskSwap.gif) |
| **Pick Highlight** | ![Pick Highlight](robomme_media/PickHighlight.gif) |
| **Video Repick** | ![Video Repick](robomme_media/VideoRepick.gif) |
| **Video Place Button** | ![Video Place Button](robomme_media/VideoPlaceButton.gif) |
| **Video Place Order** | ![Video Place Order](robomme_media/VideoPlaceOrder.gif) |
| **Move Cube** | ![Move Cube](robomme_media/MoveCube.gif) |
| **Insert Peg** | ![Insert Peg](robomme_media/InsertPeg.gif) |
| **Draw Pattern** | ![Draw Pattern](robomme_media/DrawPattern.gif) |
| **Route Stick** | ![Route Stick](robomme_media/RouteStick.gif) |

### Real World Application
![Real World Tasks](robomme_media/real_world_tasks.jpg)
![Real World Results](robomme_media/real_world_result.png)

---
*For more details, visit the [official project page](https://robomme.github.io/) or read the [paper on arXiv](https://arxiv.org/html/2603.04639).*
