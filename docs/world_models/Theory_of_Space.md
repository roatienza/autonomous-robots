# Theory of Space: Can Foundation Models Construct Spatial Beliefs through Active Exploration?

> **Authors**: Pingyue Zhang, Zihan Huang, Yue Wang, Jieyu Zhang, Letian Xue, Zihan Wang, Qineng Wang, Keshigeyan Chandrasegaran, Ruohan Zhang, Yejin Choi, Ranjay Krishna, Jiajun Wu, Li Fei-Fei, Manling Li
> **Venue**: ICLR 2026
> **Link**: [arXiv:2602.07055](https://arxiv.org/abs/2602.07055)

## Overview
This paper investigates whether multimodal foundation models can develop **Theory of Space** — an agent's ability to actively acquire information through self-directed exploration and to construct, revise, and exploit a spatial belief from sequential, partial observations. The work introduces a benchmark for curiosity-driven exploration to build accurate cognitive maps, along with a novel **spatial belief probing** technique that prompts models to reveal their internal spatial representations at each step.

## Core Contributions

### 1. Theory of Space Framework
- Defines spatial embodied intelligence as the capacity to **act to acquire information under partial observability**.
- Proposes a benchmark where agents perform curiosity-driven exploration to construct cognitive maps.

### 2. Spatial Belief Probing
- A key innovation that prompts foundation models to verbalize their internal spatial beliefs at every exploration step, enabling diagnosis of how spatial knowledge evolves (or degrades) over time.

### 3. Key Findings — Critical Bottlenecks

| Bottleneck | Description |
|---|---|
| **Active-Passive Gap** | Performance drops significantly when agents must autonomously gather information, compared to passive perception tasks where foundation models excel. |
| **Exploration Inefficiency** | Models explore unsystematically, performing far worse than simple program-based proxies in building accurate spatial maps. |
| **Global Belief Instability** | While initial perception is a bottleneck, the deeper issue is that global spatial beliefs become unstable over time, causing spatial knowledge to degrade during extended exploration. |
| **Belief Inertia** | Using a false belief paradigm, the authors show that agents fail to update obsolete priors with new evidence. This is present in text-based agents but **particularly severe in vision-based models**. |

## Key Takeaway
Current foundation models struggle to maintain **coherent, revisable spatial beliefs** during active exploration. While they excel at passive perception, the transition to active, self-directed information gathering exposes fundamental limitations in how these models represent, update, and reason about space over time.

## Relevance to Autonomous Robotics
This work directly addresses a core challenge in embodied AI: building agents that can navigate and understand environments through active exploration rather than relying on pre-existing complete information. The identified bottlenecks (belief inertia, instability, active-passive gap) are critical design considerations for autonomous robot perception and navigation systems.
