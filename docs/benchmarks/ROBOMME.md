# RoboMME: A Large-Scale Standardized Benchmark for VLA Memory

RoboMME is a large-scale standardized benchmark designed to evaluate and advance Vision-Language-Action (VLA) models in long-horizon, history-dependent robotic manipulation scenarios.

## Overview
Memory is critical for tasks that involve counting repeated actions or manipulating objects that become temporarily occluded. RoboMME addresses the lack of standardized evaluation for memory mechanisms in VLA models.

### Key Contributions
- **Standardized Benchmark**: 16 manipulation tasks across a taxonomy evaluating temporal, spatial, object, and procedural memory.
- **VLA Suite**: 14 memory-augmented VLA variants based on the $\pi 0.5$ backbone to explore memory representations and integration strategies.
- **Findings**: The effectiveness of memory representations is highly task-dependent, with different designs offering distinct advantages.

## Task Taxonomy and Memory
The benchmark evaluates several types of memory:
- **Temporal Memory**: Remembering events over time.
- **Spatial Memory**: Remembering locations of objects.
- **Object Memory**: Remembering properties of objects.
- **Procedural Memory**: Remembering sequences of actions.

![Task Category](robomme_media/task_category.png)
![Task Memory Correspondence](robomme_media/task_memory_correspondence.jpg)

## Model Design
RoboMME explores various memory-augmented VLA architectures.

![Model Design](robomme_media/model_design.jpg)
![FLOPs Comparison](robomme_media/flops_compare.jpg)

## Results
The suite of memory-augmented VLAs shows varying performance across tasks.

![MME-VLA Suite Results](robomme_media/MME-VLA-suite-results.png)

## Demonstrations

### Intro
![Intro](robomme_media/intro.gif)

### Tasks
- **Bin Fill**: ![Bin Fill](robomme_media/BinFill.gif)
- **Pick X Times**: ![Pick X Times](robomme_media/PickXtimes.gif)
- **Swing X Times**: ![Swing X Times](robomme_media/SwingXtimes.gif)
- **Stop Cube**: ![Stop Cube](robomme_media/StopCube.gif)
- **Video Unmask**: ![Video Unmask](robomme_media/VideoUnmask.gif)
- **Button Unmask**: ![Button Unmask](robomme_media/ButtonUnmask.gif)
- **Video Unmask Swap**: ![Video Unmask Swap](robomme_media/VideoUnmaskSwap.gif)
- **Button Unmask Swap**: ![Button Unmask Swap](robomme_media/ButtonUnmaskSwap.gif)
- **Pick Highlight**: ![Pick Highlight](robomme_media/PickHighlight.gif)
- **Video Repick**: ![Video Repick](robomme_media/VideoRepick.gif)
- **Video Place Button**: ![Video Place Button](robomme_media/VideoPlaceButton.gif)
- **Video Place Order**: ![Video Place Order](robomme_media/VideoPlaceOrder.gif)
- **Move Cube**: ![Move Cube](robomme_media/MoveCube.gif)
- **Insert Peg**: ![Insert Peg](robomme_media/InsertPeg.gif)
- **Draw Pattern**: ![Draw Pattern](robomme_media/DrawPattern.gif)
- **Route Stick**: ![Route Stick](robomme_media/RouteStick.gif)

## Real World Application
![Real World Tasks](robomme_media/real_world_tasks.jpg)
![Real World Results](robomme_media/real_world_result.png)
