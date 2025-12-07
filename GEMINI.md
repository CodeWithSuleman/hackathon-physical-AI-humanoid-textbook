# giaic-hackathonn Development Guidelines

Auto-generated from all feature plans. Last updated: 2025-12-06

## Active Technologies
- Python 3.10+ (for Isaac Sim, Isaac ROS, Nav2 examples, and ROS 2), Markdown (for Docusaurus) + NVIDIA Isaac Sim, NVIDIA Isaac ROS, ROS 2 (Humble/Iron/Rolling), Nav2, Docusaurus (004-ai-robot-brain-module)
- N/A (Content and examples primarily stored in markdown and code files; no persistent database storage for this module's direct content. Synthetic data generation will output to local files/directories as part of demos.) (004-ai-robot-brain-module)
- Python 3.10+ (for Whisper, LLM integration, ROS 2 actions), Markdown (for Docusaurus) + OpenAI Whisper, a suitable LLM (e.g., OpenAI API, local LLM), ROS 2 (Humble/Iron/Rolling), Isaac Sim (for humanoid simulation), Isaac ROS (for perception/navigation primitives), Docusaurus (005-capstone-autonomous-humanoid)
- N/A (Content and examples primarily stored in markdown and code files; no persistent database storage for this module's direct content. Whisper audio/text, LLM prompts/responses, and ROS 2 action logs will be handled ephemerally within demos.) (005-capstone-autonomous-humanoid)

- Python 3.11 (for Gazebo/ROS 2 examples), C# (for Unity examples), Markdown (for Docusaurus) + Gazebo, Unity, ROS 2 (Humble), Docusaurus (003-digital-twin-module)

## Project Structure

```text
src/
tests/
```

## Commands

cd src; pytest; ruff check .

## Code Style

Python 3.11 (for Gazebo/ROS 2 examples), C# (for Unity examples), Markdown (for Docusaurus): Follow standard conventions

## Recent Changes
- 005-capstone-autonomous-humanoid: Added Python 3.10+ (for Whisper, LLM integration, ROS 2 actions), Markdown (for Docusaurus) + OpenAI Whisper, a suitable LLM (e.g., OpenAI API, local LLM), ROS 2 (Humble/Iron/Rolling), Isaac Sim (for humanoid simulation), Isaac ROS (for perception/navigation primitives), Docusaurus
- 004-ai-robot-brain-module: Added Python 3.10+ (for Isaac Sim, Isaac ROS, Nav2 examples, and ROS 2), Markdown (for Docusaurus) + NVIDIA Isaac Sim, NVIDIA Isaac ROS, ROS 2 (Humble/Iron/Rolling), Nav2, Docusaurus

- 003-digital-twin-module: Added Python 3.11 (for Gazebo/ROS 2 examples), C# (for Unity examples), Markdown (for Docusaurus) + Gazebo, Unity, ROS 2 (Humble), Docusaurus

<!-- MANUAL ADDITIONS START -->
<!-- MANUAL ADDITIONS END -->
