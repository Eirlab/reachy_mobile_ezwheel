import os

os.system("rosnode kill teleop_twist_joy")
os.system("rosnode kill hector_mapping")
os.system("rosnode kill amcl")
os.system("rosnode kill tf2_web_republisher")
os.system("rosnode kill xbox_joystick")
os.system("rosnode kill rosbridge_websocket")
os.system("rosnode kill pose_from_tf")
