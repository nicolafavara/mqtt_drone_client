from tkinter import Y
from pymavlink import mavutil
import math
import copter_modes

import time
import glob
import serial

DIRECTION_COMMANDS = ["up", "down", "south", "east", "west", "north"]
YAW_COMMANDS = ["yaw_ccw", "yaw_cw"]
CHANGE_MODE_COMMANDS = ["land", "guided"]
OTHER_COMMANDS = ["stop", "arm", "takeoff"]
VALID_COMMANDS = DIRECTION_COMMANDS + YAW_COMMANDS + CHANGE_MODE_COMMANDS + OTHER_COMMANDS

DEFAULT_TAKEOFF_ALT = 5
MAX_VELOCITY = 15
MAX_DEGREES = 25
connection = None
is_connected = False


# Region methods to send pymavlink commands to drone

def connect():
    """
    function to connect to the drone and wait for a heartbeat
    """
    # Start a connection listening to a UDP port on the
    # print("connecting...")
    # the_connection = mavutil.mavlink_connection('udpin:localhost:14551')
    # wait_heartbeat(the_connection)
    # return the_connection

    # Create the connection
    # Need to provide the serial port and baudrate

    def find_ports():
        ports = glob.glob('/dev/ttyACM[0-9]*')
        res = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                res.append(port)
            except:
                pass
        return res

    lista_porte = find_ports()
    print(lista_porte)

    global is_connected
    global connection

    if len(lista_porte) == 0:
        try:
            print('TEST CONNESSIONE SERIALE')

            connection = mavutil.mavlink_connection('/dev/tty1', baud=57600)
            heartbeat = connection.wait_heartbeat(blocking=True, timeout=5)  # attendo il segnale di heartbeat
            if heartbeat:
                print('\nHEARTBEAT OK\n')
                is_connected = True
            else:
                print('\nHEARTBEAT ERROR\n')
        except:
            pass
    else:
        i = 0
        port_ok = False
        while i < len(lista_porte) and not port_ok:
            try:
                connection = mavutil.mavlink_connection(lista_porte[i], baud=57600)
                heartbeat = connection.wait_heartbeat(blocking=True, timeout=3)  # attendo il segnale di heartbeat
                if heartbeat:
                    print('\nHEARTBEAT OK\n')
                    is_connected = True
                    port_ok = True
                else:
                    print('\nHEARTBEAT ERROR\n')
            except:
                pass
            i += 1
        time.sleep(0.5)


def wait_heartbeat(the_connection):
    print("waiting for heartbeat...")
    # wait heartbeat
    r = the_connection.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
    if r is not None:
        print("heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))
        return the_connection
    else:
        print("Connection time is out")


def arm(the_connection, motors_enable):
    """
    function that arm (motors_enable=1) or disarm motors (motors_enable=0)
    """
    if motors_enable == True:
        print("arming...")
    else:
        print("disarming...")

    # confermation field (4th field of COMMAND_LONG)
    confirmation = 0
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component, 
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        confirmation,
        motors_enable,
        0, 0, 0, 0, 0, 0
    )
    
    # wait until arming confirmed 
    #print("Waiting for the vehicle to arm")
    #the_connection.motors_armed_wait()
    #print('Armed!')


def change_mode(the_connection, m):
    """
        function to change mode
    """
    print("setting mode...")

    mode = None

    if m == "land":
        mode = copter_modes.MODE_LAND
    elif m == "guided":
        mode = copter_modes.MODE_GUIDED
    else:
        return

    confirmation = 0  # confermation field (4th field of COMMAND_LONG)
    the_connection.mav.command_long_send(
        the_connection.target_system, 
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        confirmation, 
        1,
        mode,
        0, 0, 0, 0, 0
    )

    wait_ack(the_connection)


def takeoff(the_connection):
    """
    function to perform take-off
    """
    print("sending take-off command")
    # confermation field (4th field of COMMAND_LONG)
    confirmation = 0
    the_connection.mav.command_long_send(
        the_connection.target_system, 
        the_connection.target_component, 
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        confirmation,
        0,
        0,
        0,
        0,
        0, #latitude (if we pass latitude and longitude equal to 0 it will actually use the current one of the vehicle)
        0, #longitude
        DEFAULT_TAKEOFF_ALT, #altitude
    )
    wait_ack(the_connection)


def change_vel(the_connection, direction, velocity):

    type_mask = int(0b000111000000)
    x = 0
    y = 0
    z = 0

    if direction.lower() == 'north':
        x = velocity

    elif direction.lower() == 'east':
        y = velocity
        
    elif direction.lower() == 'down':
        z = velocity

    elif direction.lower() == 'south':
        x = -velocity

    elif direction.lower() == 'west':
        y = -velocity

    elif direction.lower() == 'up':
        z = -velocity

    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        0, 
        the_connection.target_system, 
        the_connection.target_component, 
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        type_mask, 
        0, #x_pos
        0, #y_pos
        0, #z_pos
        x, #x_velocity
        y, #y_velocity
        z, #Z_velocity
        0, #x_acc
        0, #y_acc
        0, #z_acc
        0, #yaw
        0  #yaw_rate (if 0 stop vehicle yaw from changing)
    ))


def yaw(the_connection, degree):

    rad = math.radians(degree)

    type_mask = int(0b010111000111)
    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        0, 
        the_connection.target_system, 
        the_connection.target_component, 
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        type_mask, 
        0, #x_pos
        0, #y_pos
        0, #z_pos
        0, #x_velocity
        0, #y_velocity
        0, #Z_velocity
        0, #x_acc
        0, #y_acc
        0, #z_acc
        0, #yaw
        rad  #yaw_rate (if 0 stop vehicle yaw from changing)
    ))


def stop(the_connection):

    type_mask = int(0b000111000111)

    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        0, 
        the_connection.target_system, 
        the_connection.target_component, 
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        type_mask, 
        0, #x_pos
        0, #y_pos
        0, #z_pos
        0, #x_velocity
        0, #y_velocity
        0, #Z_velocity
        0, #x_acc
        0, #y_acc
        0, #z_acc
        0, #yaw
        0  #yaw_rate (if 0 stop vehicle yaw from changing)
    )) 



def wait_ack(the_connection):
    msg = ""
    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
    if msg:
        print(msg)
    else:
        print("ACK message not received.")


# end region
# Region for command methods via buttons


def eval_command(btn_pressed, velocity, degrees):

    if btn_pressed not in VALID_COMMANDS:
        print("command not found.")
        return 

    if velocity > MAX_VELOCITY:
        velocity = MAX_VELOCITY

    if degrees > MAX_DEGREES:
        degrees = MAX_DEGREES

    # try connecting to the drone
    global connection

    if connection is None:
        connection = connect()

    if connection is None:
        print("Drone connection attempt failed.")
        return

    if btn_pressed in DIRECTION_COMMANDS:
        change_vel(connection, btn_pressed, velocity)

    elif btn_pressed in YAW_COMMANDS:
        if btn_pressed == "yaw_ccw":
            degrees = -degrees

        yaw(connection, degrees)

    elif btn_pressed in CHANGE_MODE_COMMANDS:
        change_mode(connection, btn_pressed)

    elif btn_pressed == "arm":
        arm(connection, True)

    elif btn_pressed == "takeoff":
        takeoff(connection)

    elif btn_pressed == "stop":
        stop(connection)
        connection = None

#end region
# Region for command methods via radiocontroller

def change_vel_from_rc(the_connection, velocity, degrees, throttle, yaw, pitch, roll):

    type_mask = int(0b010111000000)
    x_vel = pitch * velocity
    y_vel = roll * velocity
    z_vel = throttle * (-velocity)
    rad = math.radians(yaw * degrees)
    print("x_vel: %.2f y_vel: %.2f z_vel: %.2f rad: %.2f" % (x_vel, y_vel, z_vel, rad))

    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        0, 
        the_connection.target_system, 
        the_connection.target_component, 
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        type_mask, 
        0, #x_pos
        0, #y_pos
        0, #z_pos
        x_vel, #x_velocity
        y_vel, #y_velocity
        z_vel, #Z_velocity
        0, #x_acc
        0, #y_acc
        0, #z_acc
        0, #yaw
        rad  #yaw_rate (if 0 stop vehicle yaw from changing)
    ))


def sim_rc(velocity, degrees, throttle, yaw, pitch, roll):

    global connection

    if connection is None:
        connection = connect()

    if connection is None:
        print("Drone connection attempt failed.")
        return

    change_vel_from_rc(connection, velocity, degrees, throttle, yaw, pitch, roll)
