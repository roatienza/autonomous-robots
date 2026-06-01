---
# 🛠️ Stable-WorldModel
**Standardizing World Model Research with MPC**

**Link**: [GitHub](https://github.com/galilai-group/stable-worldmodel)

---

## 📌 Core Objective
- **Standardization**: Reducing research fragmentation by providing a unified pipeline for world model development.
- **MPC Integration**: Focusing on the intersection of generative world models and **Model Predictive Control (MPC)**.
- **Reproducibility**: Ensuring that data collection and evaluation are consistent across different research groups.

---

## ⚙️ Key Technical Features
- **Three-Stage Workflow**: 
    - **Collect**: Efficient data gathering from environments.
    - **Train**: Training the world model on collected transitions.
    - **Evaluate**: Testing the model's predictive accuracy and control performance.
- **High-Performance Data**: Uses **LanceDB** for high-throughput recording (up to 3.4x faster than HDF5).
- **Planning Solvers**: Includes 7 different MPC solvers to evaluate how well a world model can be used for planning.

---

## 🚀 Visual Demonstration
![MPC Planning](media/route_stick.gif)
*Example of a robot using a world model to plan a trajectory (MPC).*

---

## 🤖 Robotics Relevance
- **Closed-Loop Control**: Bridges the gap between "predicting the future" and "acting in the world."
- **Benchmark Consistency**: Provides a set of 30+ standardized environments to compare different world model architectures.
- **Efficient Iteration**: Faster data handling allows for quicker training-evaluation loops.
---
