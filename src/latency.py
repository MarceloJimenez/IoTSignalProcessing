import paho.mqtt.client as mqtt
import time
import json

# MQTT Configuration
BROKER = "192.168.28.32"  # Replace with your MQTT broker's IP
PORT = 1883
TOPIC = "iot-homework"

# Callback when a message is received
def on_message(client, userdata, msg):
    try:
        # Parse the received message
        payload = json.loads(msg.payload.decode())
        sent_timestamp = payload.get("timestamp", 0)  # Timestamp from ESP32 in microseconds
        received_timestamp = int(time.time() * 1e6)  # Current time in microseconds

        # Calculate latency in microseconds
        latency_microseconds = received_timestamp - sent_timestamp

        # Convert latency to seconds
        latency_seconds = latency_microseconds / 1e6
        print(f"Data: {payload}, Latency: {latency_seconds:.6f} seconds")
    except Exception as e:
        print(f"Error processing message: {e}")

# Set up MQTT client
client = mqtt.Client()
client.on_message = on_message

# Connect to the broker and subscribe to the topic
client.connect(BROKER, PORT, 60)
client.subscribe(TOPIC)

# Start the MQTT loop
print("Listening for messages...")
client.loop_forever()