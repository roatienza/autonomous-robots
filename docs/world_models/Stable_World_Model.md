# Stable-WorldModel: Reproducible World Model Research with MPC

> **Link**: [GitHub](https://github.com/galilai-group/stable-worldmodel)

## Overview
Stable-WorldModel is an open-source Python platform designed to standardize world model research using **Model Predictive Control (MPC)**. It addresses the fragmentation in research by providing a unified pipeline for data collection, training, and evaluation.

## Key Features
- **Unified Pipeline**: A three-stage "Collect $\rightarrow$ Train $\rightarrow$ Evaluate" workflow.
- **Standardized Environments**: Supports 30+ environments with controllable variation.
- **Efficient Data Handling**: Uses LanceDB for high-throughput data recording (up to 3.4x faster than HDF5).
- **MPC Integration**: Includes 7 different planning solvers to evaluate world-model-based control.
