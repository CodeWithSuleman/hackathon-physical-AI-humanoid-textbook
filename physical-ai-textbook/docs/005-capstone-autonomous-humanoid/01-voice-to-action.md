# Voice-to-Action: Whisper Basics and Command Extraction

Welcome to Chapter 1 of the Capstone Project! In this module, we'll explore the Vision-Language-Action (VLA) pipeline, enabling autonomous humanoid robots to respond to natural language commands. Our journey begins with the crucial first step: converting spoken instructions into actionable text commands using OpenAI Whisper.

## The VLA Pipeline: Voice → LLM Plan → ROS 2 Actions

The Vision-Language-Action (VLA) pipeline represents a powerful paradigm for human-robot interaction and autonomous robot control. It bridges the gap between high-level natural language commands and low-level robot actions through cognitive planning. The pipeline consists of three main stages:

1.  **Voice Input**: The human operator provides a command verbally.
2.  **LLM Plan (Cognitive Planning)**: A Large Language Model (LLM) interprets the natural language command, breaks it down into sub-tasks, and translates it into a sequence of executable robot actions.
3.  **ROS 2 Actions (Autonomous Execution)**: The robot executes the planned sequence of actions using its existing ROS 2 capabilities (e.g., navigation, manipulation, perception).

This chapter focuses on the initial "Voice Input" stage, utilizing OpenAI Whisper to convert speech into text.

## OpenAI Whisper: Speech-to-Text for Robotics

OpenAI Whisper is a general-purpose speech-to-text model. It is trained on a large dataset of diverse audio and is also a multi-task model that can perform multilingual speech recognition, speech translation, and language identification. Its robustness to accents, background noise, and technical jargon makes it an excellent candidate for robotics applications where clear voice commands are essential.

### How Whisper Works (Basics)

Whisper takes an audio input (e.g., a WAV file or live audio stream) and outputs a transcription of the spoken words. It works by:

1.  **Audio Preprocessing**: The input audio is converted into a log-Mel spectrogram, a representation suitable for neural networks.
2.  **Encoder-Decoder Architecture**: Whisper uses a transformer-based sequence-to-sequence model. The encoder processes the audio features, and the decoder predicts the sequence of text tokens.
3.  **Tokenization**: The output text is generated as a sequence of tokens, which are then converted back into human-readable words.

### Command Extraction from Whisper Output

Once Whisper transcribes speech into text, the next step is to extract meaningful commands that the robot can understand. This often involves:

*   **Keyword Spotting**: Identifying specific keywords or phrases that trigger robot behaviors.
*   **Intent Recognition**: Understanding the user's overall goal (e.g., "navigate," "pick up," "find").
*   **Entity Extraction**: Identifying parameters for the commands (e.g., "kitchen" for navigation, "red block" for picking up).

This extraction can be rule-based (using regular expressions), or more sophisticated methods involving smaller language models or NLP techniques can be employed. The cleaner and more consistent the Whisper output, the easier this extraction process becomes.

## Running the Whisper Voice-to-Command Demo

You can find a simple Whisper voice-to-command demo script in `examples/capstone_whisper_demos/voice_to_command_demo.py`. Refer to the `README.md` file within that directory for detailed setup and execution instructions.

To run this demo:

1.  **Ensure Prerequisites**: Make sure you have a working microphone and your `OPENAI_API_KEY` environment variable is set.
2.  **Install Python Libraries**: The script requires `openai`, `pydub`, and `sounddevice`. Install them using `pip install -r requirements.txt` from the project root.
3.  **Navigate to Example**:
    ```bash
    cd examples/capstone_whisper_demos/
    ```
4.  **Run the Script**:
    ```bash
    python3 voice_to_command_demo.py
    ```
    The script will prompt you to speak a command, record your voice, transcribe it using Whisper, and then attempt a conceptual command extraction.

## Conclusion

OpenAI Whisper serves as a powerful front-end for our VLA pipeline, accurately converting spoken commands into text. This capability unlocks intuitive voice control for humanoid robots, paving the way for more natural human-robot interaction. With reliable text transcription, we can then proceed to the cognitive planning stage, where Large Language Models will translate these commands into actionable robot behaviors.

## Further Reading

*   [OpenAI Whisper GitHub](https://github.com/openai/whisper)
*   [OpenAI Whisper Paper](https://arxiv.org/abs/2212.04356)
*   [ROS 2 Actions Documentation](https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)
