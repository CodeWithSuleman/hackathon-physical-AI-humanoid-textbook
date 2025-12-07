# examples/capstone_whisper_demos/voice_to_command_demo.py

import os
import openai
import sounddevice as sd
import numpy as np
import scipy.io.wavfile as wavfile
import io
import time

# Load OpenAI API Key from environment variable
openai.api_key = os.getenv("OPENAI_API_KEY")

# --- Configuration ---
SAMPLE_RATE = 16000  # Whisper requires 16 kHz
CHANNELS = 1         # Mono audio
DTYPE = 'int16'      # Data type for audio recording
RECORD_SECONDS = 5   # Duration of recording
WAV_FILE_PATH = "recorded_command.wav"

def record_audio(duration, samplerate, channels, dtype, filename):
    """Records audio from the microphone and saves it to a WAV file."""
    print(f"Recording for {duration} seconds. Please speak your command...")
    audio_data = sd.rec(int(duration * samplerate), samplerate=samplerate, channels=channels, dtype=dtype)
    sd.wait() # Wait until recording is finished
    print("Recording finished. Processing...")
    
    # Normalize to float32 before saving for Whisper compatibility
    normalized_audio = audio_data / np.iinfo(dtype).max
    wavfile.write(filename, samplerate, normalized_audio)
    print(f"Audio saved to {filename}")

def transcribe_audio_whisper(audio_file_path):
    """Transcribes audio using OpenAI Whisper API."""
    if not openai.api_key:
        raise ValueError("OPENAI_API_KEY environment variable not set.")
    
    try:
        with open(audio_file_path, "rb") as audio_file:
            transcript = openai.Audio.transcribe("whisper-1", audio_file)
        return transcript["text"]
    except openai.error.AuthenticationError:
        print("Error: Invalid OpenAI API key. Please check your OPENAI_API_KEY environment variable.")
        return None
    except Exception as e:
        print(f"An error occurred during transcription: {e}")
        return None

def extract_command(transcription):
    """
    Conceptual function to extract a robot command from transcription.
    This would be much more sophisticated in a real application.
    """
    if not transcription:
        return "No command extracted."

    # Simple keyword spotting for demonstration
    transcription_lower = transcription.lower()
    
    if "move forward" in transcription_lower or "go forward" in transcription_lower:
        return "ACTION: move_forward"
    elif "stop" in transcription_lower:
        return "ACTION: stop_robot"
    elif "turn left" in transcription_lower:
        return "ACTION: turn_left"
    elif "turn right" in transcription_lower:
        return "ACTION: turn_right"
    elif "pick up" in transcription_lower or "grab" in transcription_lower:
        return "ACTION: pick_up_object"
    elif "find" in transcription_lower and "object" in transcription_lower:
        return "ACTION: detect_object"
    else:
        return f"ACTION: unknown_command ('{transcription_lower}')"

if __name__ == "__main__":
    print("--- OpenAI Whisper Voice-to-Command Demo ---")
    
    try:
        # 1. Record audio
        record_audio(RECORD_SECONDS, SAMPLE_RATE, CHANNELS, DTYPE, WAV_FILE_PATH)
        
        # 2. Transcribe audio
        print(f"Transcribing {WAV_FILE_PATH}...")
        transcription_text = transcribe_audio_whisper(WAV_FILE_PATH)
        
        if transcription_text:
            print(f"Transcription: \"{transcription_text}\"")
            
            # 3. Extract command
            robot_command = extract_command(transcription_text)
            print(f"Extracted Robot Command: {robot_command}")
        else:
            print("Transcription failed or returned empty.")

    except ValueError as ve:
        print(f"Configuration Error: {ve}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        # Clean up recorded file
        if os.path.exists(WAV_FILE_PATH):
            os.remove(WAV_FILE_PATH)
            print(f"Cleaned up {WAV_FILE_PATH}")
