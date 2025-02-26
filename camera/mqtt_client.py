import paho.mqtt.client as mqtt

class MQTTClient:
    def __init__(self, broker: str, port: int):
        self.broker = broker
        self.port = port
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.connect(self.broker, self.port)
        print(f'Connected to MQTT broker at {self.broker}:{self.port}')

    def publish(self, topic: str, message: str):
        try:
            self.client.publish(topic, payload=message)
            print(f'Message published to topic {topic}')
        except Exception as e:
            print(f'Failed to publish message: {e}')

    def disconnect(self):
        try:
            self.client.disconnect()
            print('Disconnected from MQTT broker')
        except Exception as e:
            print(f'Failed to disconnect from MQTT broker: {e}')

# Example usage
if __name__ == '__main__':
    mqtt_client = MQTTClient(broker='test.mosquitto.org', port=1883)
    # Publish a message
    mqtt_client.publish(topic='test/topic', message='Hello, MQTT!')

    mqtt_client.disconnect()