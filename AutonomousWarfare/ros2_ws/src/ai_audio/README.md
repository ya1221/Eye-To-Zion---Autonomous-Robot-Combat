# ai_audio

## Purpose
Detect physical impacts to the robot's chassis using machine learning on audio signals.

## Logic
Listens to the ICS-43434 I2S microphone stream, runs an inference model (CNN/Audio processing) to classify the sound, and determines if it constitutes an 'impact' or a hit.

## Data Flow
- **Input:** Raw audio data from I2S microphone.
- **Output:** Publishes `std_msgs/String` to `/audio/impact_alert` when an impact is detected.
