import odrive
from odrive.enums import *
import math
import time
import tkinter



##RESTART THIS CODE WHEN THE ROBOT IS READY TO RUN!!!!!##

def key_handler(event):
    print(event.char, event.keysym, event.keycode)

depth_time = 9
depth_speed = 7

depth_speed_return = ((depth_speed*4))
depth_time_return = (depth_time/2)
excav_speed = -5




def start_excav(event): 
    print("Starting Excavation...")
    odrve.axis0.controller.input_vel = excav_speed
def stop_excav(event):
    print("Stopping Excavation...") 
    odrve.axis0.controller.input_vel = 0

def depth_up(event):
    print("Starting at", odrvd.axis0.pos_estimate)
    print("Raising Excavator")
    odrvd.axis0.controller.config.vel_limit = depth_speed_return
    odrvd.axis0.controller.input_pos = (starting_depth)
    time.sleep(1)
    odrve.axis0.controller.input_vel = 0
    print ("Done")
def depth_down(event):
    print("Starting at", odrvd.axis0.pos_estimate)
    print("Lowering Excavator")
    odrvd.axis0.controller.config.vel_limit = depth_speed
    odrvd.axis0.controller.input_pos = (starting_depth + 70)
    print ("Done")

def depth_up_inc(event):
    odrvd.axis0.controller.config.vel_limit = depth_speed_return
    odrvd.axis0.controller.input_pos = (odrvd.axis0.pos_estimate - 2.5)
    print("Raising Excavator")
    print ("Done")
def depth_down_inc(event):
    odrvd.axis0.controller.config.vel_limit = depth_speed_return
    odrvd.axis0.controller.input_pos = (odrvd.axis0.pos_estimate + 2.5)
    print("Lowering Excavator")
    print ("Done")

def openC(event):
    odrvc.axis0.controller.input_pos = (float(starting_const) - 0.4)
    print("Bucket Opened",float(starting_const) - 0.4 )
def closeC(event):
    odrvc.axis0.controller.input_pos = starting_const
    print("Bucket Closed")

def e_stop(event): 
    odrve.axis0.controller.input_vel = 0
    odrvd.axis0.controller.input_vel = 0
    odrvc.axis0.controller.input_pos = starting_const
    print("Emergency Stop")


print("Finding ODrives...")

#CHANGE FOR NEW ODRIVES GET SERIAL NUM

odrv_excav = odrive.find_any(serial_number ="3977345E3331")
odrv_depth = odrive.find_any(serial_number ="3965347A3331")
odrv_const = odrive.find_any(serial_number ="366D33653432")



odrv_const.clear_errors()
odrv_depth.clear_errors()
odrv_excav.clear_errors()

print("ODrives Connected")
print("Construction Voltage: ", str(odrv_const.vbus_voltage))
print("Depth Voltage: ", str(odrv_depth.vbus_voltage))
print("Excavation Voltage: ", str(odrv_excav.vbus_voltage))


odrvd = odrv_depth
odrvd.config.dc_bus_overvoltage_trip_level = 51.25
odrvd.config.dc_bus_undervoltage_trip_level = 10.5
odrvd.config.dc_max_positive_current = math.inf
odrvd.config.dc_max_negative_current = -math.inf
odrvd.config.brake_resistor0.enable = False
odrvd.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
odrvd.axis0.config.motor.pole_pairs = 7
odrvd.axis0.config.motor.torque_constant = 0.05513333333333333
odrvd.axis0.config.motor.current_soft_max = 70
odrvd.axis0.config.motor.current_hard_max = 90
odrvd.axis0.config.motor.calibration_current = 10
odrvd.axis0.config.motor.resistance_calib_max_voltage = 2
odrvd.axis0.config.calibration_lockin.current = 10
odrvd.axis0.motor.motor_thermistor.config.enabled = False
odrvd.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
odrvd.axis0.controller.config.input_mode = InputMode.VEL_RAMP
odrvd.axis0.controller.config.vel_limit = 100
odrvd.axis0.controller.config.vel_limit_tolerance = 1.2
odrvd.axis0.config.torque_soft_min = -math.inf
odrvd.axis0.config.torque_soft_max = math.inf
odrvd.axis0.controller.config.vel_ramp_rate = 50
odrvd.can.config.protocol = Protocol.NONE
odrvd.axis0.config.enable_watchdog = False
odrvd.axis0.config.load_encoder = EncoderId.RS485_ENCODER0
odrvd.axis0.config.commutation_encoder = EncoderId.RS485_ENCODER0
odrvd.rs485_encoder_group0.config.mode = Rs485EncoderMode.AMT21_EVENT_DRIVEN
odrvd.config.enable_uart_a = False
odrve = odrv_excav
odrve.config.dc_bus_overvoltage_trip_level = 51.25
odrve.config.dc_bus_undervoltage_trip_level = 10.5
odrve.config.dc_max_positive_current = math.inf
odrve.config.dc_max_negative_current = -math.inf
odrve.config.brake_resistor0.enable = False
odrve.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
odrve.axis0.config.motor.pole_pairs = 20
odrve.axis0.config.motor.torque_constant = 0.0827
odrve.axis0.config.motor.current_soft_max = 50
odrve.axis0.config.motor.current_hard_max = 70
odrve.axis0.config.motor.calibration_current = 10
odrve.axis0.config.motor.resistance_calib_max_voltage = 2
odrve.axis0.config.calibration_lockin.current = 10
odrve.axis0.motor.motor_thermistor.config.enabled = False
odrve.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
odrve.axis0.controller.config.input_mode = InputMode.VEL_RAMP
odrve.axis0.controller.config.vel_limit = 10
odrve.axis0.controller.config.vel_limit_tolerance = 1.1
odrve.axis0.config.torque_soft_min = -math.inf
odrve.axis0.config.torque_soft_max = math.inf
odrve.axis0.trap_traj.config.accel_limit = 5
odrve.axis0.controller.config.vel_ramp_rate = 5
odrve.can.config.protocol = Protocol.NONE
odrve.axis0.config.enable_watchdog = False
odrve.axis0.config.load_encoder = EncoderId.ONBOARD_ENCODER0
odrve.axis0.config.commutation_encoder = EncoderId.ONBOARD_ENCODER0
odrve.config.enable_uart_a = False
odrvc = odrv_const
odrvc.config.dc_bus_overvoltage_trip_level = 56
odrvc.config.dc_bus_undervoltage_trip_level = 10.5
odrvc.config.dc_max_positive_current = math.inf
odrvc.config.dc_max_negative_current = -math.inf
odrvc.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
odrvc.axis0.config.motor.pole_pairs = 7
odrvc.axis0.config.motor.torque_constant = 0.05513333333333333
odrvc.axis0.config.motor.current_soft_max = 70
odrvc.axis0.config.motor.current_hard_max = 90
odrvc.axis0.config.motor.calibration_current = 10
odrvc.axis0.config.motor.resistance_calib_max_voltage = 2
odrvc.axis0.config.calibration_lockin.current = 10
odrvc.axis0.motor.motor_thermistor.config.enabled = False
odrvc.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
odrvc.axis0.controller.config.input_mode = InputMode.VEL_RAMP
odrvc.axis0.controller.config.vel_limit = 2
odrvc.axis0.controller.config.vel_limit_tolerance = 500.5
odrvc.axis0.config.torque_soft_min = -math.inf
odrvc.axis0.config.torque_soft_max = math.inf
odrvc.axis0.trap_traj.config.accel_limit = 5
odrvc.axis0.controller.config.vel_ramp_rate = 5
odrvc.can.config.protocol = Protocol.SIMPLE
odrvc.can.config.baud_rate = 250000
odrvc.axis0.config.can.node_id = 0
odrvc.axis0.config.can.heartbeat_msg_rate_ms = 100
odrvc.axis0.config.can.encoder_msg_rate_ms = 10
odrvc.axis0.config.can.iq_msg_rate_ms = 10
odrvc.axis0.config.can.torques_msg_rate_ms = 10
odrvc.axis0.config.can.error_msg_rate_ms = 10
odrvc.axis0.config.can.temperature_msg_rate_ms = 10
odrvc.axis0.config.can.bus_voltage_msg_rate_ms = 10
odrvc.axis0.config.enable_watchdog = False
odrvc.axis0.config.load_encoder = EncoderId.RS485_ENCODER0
odrvc.axis0.config.commutation_encoder = EncoderId.RS485_ENCODER0
odrvc.rs485_encoder_group0.config.mode = Rs485EncoderMode.AMT21_EVENT_DRIVEN
odrvc.config.enable_uart_a = False

global starting_const, starting_depth
starting_const = odrvc.axis0.pos_estimate # Sync
starting_depth = odrvd.axis0.pos_estimate # Sync
print("Construction started at:", starting_const, "\n Depth started at:", starting_depth)

##### START OF TKINTER #####
root = tkinter.Tk()
root.bind("<Key>", key_handler)

root.bind("q", start_excav) #start excavation
root.bind("w", stop_excav) #start excavation
root.bind("z", openC) #open construction
root.bind("x", closeC) #close construction
root.bind("<Up>", depth_up) # depth up
root.bind("<Down>", depth_down) # depth down
root.bind("<n>", depth_up_inc) # depth up
root.bind("<m>", depth_down_inc) # depth down
root.bind("p", e_stop) # Estop

odrvd.axis0.controller.config.input_mode = InputMode.POS_FILTER
odrve.axis0.controller.config.input_mode = InputMode.VEL_RAMP
odrvc.axis0.controller.config.input_mode = InputMode.POS_FILTER

odrvd.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL
odrve.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
odrvc.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL

odrvd.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
odrve.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
odrvc.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL

print ("Control Mode Set")
print("Ready to Drive")
root.mainloop()

