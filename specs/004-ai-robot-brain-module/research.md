# Research & Decisions: Module 3: The AI-Robot Brain (NVIDIA Isaac™)

This document outlines key decisions made during the planning phase for "Module 3: The AI-Robot Brain (NVIDIA Isaac™)".

## 1. Handling Synthetic Data Storage for Demos

-   **Decision**: Demos that generate synthetic data will output to local files or directories within their respective example folders. This data will be used ephemerally for demonstration purposes or for subsequent processing within the demo's scope.
-   **Rationale**: This module focuses on the concepts of photorealistic simulation, synthetic data generation, VSLAM, and navigation. Persistent database storage for generated data is outside the scope of these introductory examples and would introduce unnecessary complexity, detracting from the core learning objectives.
-   **Alternatives considered**: Implementing persistent database storage was considered but deemed overly complex for introductory material. Printing data directly to console or streaming it in real-time within the simulation environment without explicit saving were also considered, but local file saving offers a good balance for demonstration and potential offline analysis.
