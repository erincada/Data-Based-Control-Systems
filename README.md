# ME 418/518 – Data-Based Control

This repository contains my coursework for **ME 418/518 – Data-Based Control**.  
It includes selected homework assignments, the midterm exam, and the course project, focusing on data-driven system identification, model predictive control (MPC), and learning-based control extensions.

---

## Homework Assignments

### Homework 2 – Subspace Identification (N4SID)
This assignment focuses on manual implementation of the N4SID algorithm using synthetic input–output data.  
The system order is determined via singular value decomposition, and state trajectories are reconstructed using projected future outputs.

**Included figure:**  
- Estimated state trajectories obtained from the manual N4SID procedure.

<p align="center">
  <img src="images/hw2.png" width="650">
</p>

---

### Homework 3 – MPC Prediction Formulation
This homework studies the prediction structure used in Model Predictive Control.  
The lifted prediction model is constructed using the derived \(F\), \(H\), \(S\), and \(\Phi\) matrices, and the effect of a step change in the control increments is analyzed.

**Included figure:**  
- Predicted output trajectory under a step \(\Delta U\).

<p align="center">
  <img src="images/hw3.png" width="650">
</p>

---

### Homework 4 – Model Predictive Control Design
In this assignment, unconstrained and constrained MPC controllers are designed and evaluated.  
Different prediction and control horizon choices are compared to illustrate the trade-off between tracking performance and control effort.

**Included figure:**  
- Unconstrained MPC step reference tracking for a selected \((N_p, N_c)\) pair.

<p align="center">
  <img src="images/hw4.png" width="650">
</p>

---

## Midterm Exam

The midterm exam covers data generation, signal cleaning, FIR and ARX model identification, and MPC design under constraints and measurement noise.  
All simulations were implemented in MATLAB, and the report includes detailed explanations and validation results.

**Included figure:**  
- MPC tracking performance under measurement noise for a step reference.

<p align="center">
  <img src="images/mt.png" width="650">
</p>

---

### Homework 5 – On-Policy vs Off-Policy Reinforcement Learning (Cliff Walking)

This assignment compares **Sarsa (on-policy)** and **Q-learning (off-policy)** algorithms using the classic Cliff Walking environment.  
Both methods were implemented from scratch and evaluated over multiple independent runs to obtain averaged learning curves.

The results highlight a key conceptual difference:  
Sarsa accounts for the risk introduced by exploration and therefore learns a safer path away from the cliff, while Q-learning converges to the optimal (shortest) path but suffers from poor online performance due to occasional exploratory actions that lead to catastrophic penalties.

**Included figure:**  
- Average episode return comparison of Sarsa and Q-learning over 500 episodes.

<p align="center">
  <img src="images/hw5.png" width="650">
</p>

---

## Course Project

### Residual-Aware RL Tuning of MPC on Data-Driven Models

The course project proposes a **Residual-Aware Reinforcement Learning–MPC framework**, where an RL agent tunes MPC parameters based on real-time model residuals.  
Data-driven identification methods (ARX, FIR, N4SID) are used to construct predictive models, and the one-step prediction residual is treated as a measure of model reliability.

The main objective is to enable the controller to adapt its aggressiveness depending on how much it can trust its internal model, improving robustness under modeling errors and uncertainty.

Project materials in this repository include:
- Project proposal,
- Progress report,
- Supporting references and planned evaluation metrics.

---

## Notes

- All MATLAB code was written independently for this course.
- Large language models and code-generation tools were **not used** during exams or restricted assessments.
- Figures included here are representative summaries; full derivations and results are provided in the submitted reports.

---

**Author:** Erinç Ada Ceylan  
**Program:** Mechanical Engineering, Bilkent University  
**Course:** ME 418/518 – Data-Based Control
