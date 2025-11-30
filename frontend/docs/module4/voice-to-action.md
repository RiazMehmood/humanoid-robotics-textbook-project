---
sidebar_position: 2
---

# Voice-to-Action with OpenAI Whisper

## Overview

This section covers using **OpenAI Whisper** to convert spoken voice commands into text, which can then be processed by LLMs to generate robot actions.

## OpenAI Whisper

Whisper is an automatic speech recognition (ASR) system that:

- Supports multiple languages
- Handles various accents and noise
- Provides high accuracy
- Can run locally or via API

## Installation

```bash
pip install openai-whisper
```

## Basic Usage

### Python Implementation

```python
import whisper

# Load model
model = whisper.load_model("base")

# Transcribe audio
result = model.transcribe("audio.wav")
print(result["text"])
```

### Real-time Processing

```python
import whisper
import pyaudio
import wave

model = whisper.load_model("base")

# Record audio
chunk = 1024
format = pyaudio.paInt16
channels = 1
rate = 16000

p = pyaudio.PyAudio()
stream = p.open(format=format, channels=channels, rate=rate,
                frames_per_buffer=chunk, input=True)

frames = []
for i in range(0, int(rate / chunk * 3)):  # 3 seconds
    data = stream.read(chunk)
    frames.append(data)

stream.stop_stream()
stream.close()
p.terminate()

# Save and transcribe
wf = wave.open("temp.wav", 'wb')
wf.setnchannels(channels)
wf.setsampwidth(p.get_sample_size(format))
wf.setframerate(rate)
wf.writeframes(b''.join(frames))
wf.close()

result = model.transcribe("temp.wav")
command = result["text"]
```

## ROS 2 Integration

### Voice Command Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')
        self.publisher = self.create_publisher(String, '/voice_command', 10)
        self.model = whisper.load_model("base")
        self.timer = self.create_timer(5.0, self.process_voice)
    
    def process_voice(self):
        # Record and transcribe
        audio_file = self.record_audio()
        result = self.model.transcribe(audio_file)
        
        # Publish command
        msg = String()
        msg.data = result["text"]
        self.publisher.publish(msg)
        self.get_logger().info(f'Voice command: {msg.data}')
```

## Error Handling

```python
try:
    result = model.transcribe(audio_file)
    command = result["text"].strip()
    if not command:
        raise ValueError("Empty transcription")
except Exception as e:
    self.get_logger().error(f'Transcription error: {e}')
    return None
```

## Next Steps

Learn about [Cognitive Planning](/module4/cognitive-planning) to translate commands into actions.

