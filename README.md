# Multi-agent-interaction-with-enhanced-safety-through-reachability-analysis

This repository contains the full implementation of the a control framework able to perform overtaking maneuvers in realistic scenarios enhancing safety through the Hamilton-Jacobi backward reachability analysis; along with video for each simulation.

## 📁 Folder Structure

- It contains the complete implementation of the multi-agent model predictive controller and simulation environment.
- `Trajectories_video_collection/` – Contains recorded simulations corresponding to the scenarios described in the paper. These videos were generated using 'main.m'.

## ▶️ Running the Code

To run the simulations and reproduce the results:

1. **Install MATLAB** (R2021a or later recommended)
2. **Install ACADOS** and compile it with MATLAB interface support  
   See (https://docs.acados.org) for installation instructions
3. To use this code, backward reachable tube (BRT) data, grid and gradients are required. They can be computed using either the (https://github.com/StanfordASL/hj_reachability) or the (https://github.com/HJReachability/helperOC) toolboxes.

## 📄 License and Citation

If you use this code or refer to the results, please cite our work.

