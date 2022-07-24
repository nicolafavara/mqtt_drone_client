import commands
import paho.mqtt.client as mqtt
import sys
import json
import time

BROKER_ADDRESS = "127.0.0.1"
SUB_TOPIC = "mqtt/commands"
MQTT_PORT = 1883

def on_connect(client, userdata, flags, rc):
    
    if rc == 0:
        print("Connection successful.\nWaiting for messages...")
        client.subscribe(SUB_TOPIC)
    else:
        print("Connection failed with returned code: ", rc)


def on_message(client, userdata, msg):

    t = time.localtime()
    current_time = time.strftime("%H:%M:%S", t)

    command = json.loads(msg.payload)

    velocity = command["velocity"]
    yaw_rate = command["yaw_rate"]

    if command["axes"] is None:
        print(
            current_time + " - new message -> " 
            "btn_pressed: " +  command["btn_pressed"] + ", " +
            "velocity: " + str(velocity) + ", " +
            "yaw_rate: " + str(yaw_rate)
        )
        btn_pressed = command["btn_pressed"]

        commands.eval_command(btn_pressed, velocity, yaw_rate)
    else:
        axes = command["axes"]
        print(
            current_time + " - new message -> " 
            "velocity: " + str(velocity) + ", " +
            "yaw_rate: " + str(yaw_rate) + ", " +
            "throttle: " + str(axes["throttle"]) + ", " +
            "yaw: " + str(axes["yaw"]) + ", " +
            "pitch: " + str(axes["pitch"]) + ", " +
            "roll: " + str(axes["roll"])
        )
        throttle = float("%.2f" % float(axes["throttle"]))
        yaw = float("%.2f" % float(axes["yaw"]))
        pitch = float("%.2f" % float(axes["pitch"]))
        roll = float("%.2f" % float(axes["roll"]))
        commands.sim_rc(velocity, yaw_rate, throttle, yaw, pitch, roll)
  

def start_client(address):

    client = mqtt.Client() 
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        client.connect(address, MQTT_PORT)
    except ConnectionRefusedError:
        print("connection refused.");
        sys.exit()

    client.loop_forever()


start_client(BROKER_ADDRESS)