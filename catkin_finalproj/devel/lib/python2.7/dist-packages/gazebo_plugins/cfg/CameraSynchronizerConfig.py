## *********************************************************
##
## File autogenerated for the gazebo_plugins package
## by the dynamic_reconfigure package.
## Please do not edit.
##
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 245, 'name': 'Default', 'parent': 0, 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [], 'parameters': [{'srcline': 290, 'description': 'Projector pulse frequency in Hz.', 'max': 120.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'projector_rate', 'edit_method': '', 'default': 60.0, 'level': 31, 'min': 40.0, 'type': 'double'}, {'srcline': 290, 'description': 'Length of the projector pulses in s. At high currents the hardware may limit the pulse length.', 'max': 0.002, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'projector_pulse_length', 'edit_method': '', 'default': 0.002, 'level': 31, 'min': 0.001, 'type': 'double'}, {'srcline': 290, 'description': 'How far off-center the intermediate projector pulses are. Zero is on-center, one is touching the following pulse.', 'max': 1.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'projector_pulse_shift', 'edit_method': '', 'default': 0.0, 'level': 31, 'min': 0.0, 'type': 'double'}, {'srcline': 290, 'description': 'Indicates whether the projector should be off, on when in use or on all the time.', 'max': 3, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'projector_mode', 'edit_method': "{'enum_description': 'The projectors operating mode.', 'enum': [{'srcline': 56, 'description': 'The projector is always off.', 'srcfile': '/home/ur3/catkin_finalproj/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': 'ProjectorOff'}, {'srcline': 57, 'description': 'The projector is on if one of the cameras is using it.', 'srcfile': '/home/ur3/catkin_finalproj/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'cconsttype': 'const int', 'value': 2, 'ctype': 'int', 'type': 'int', 'name': 'ProjectorAuto'}, {'srcline': 58, 'description': 'The projector is always on.', 'srcfile': '/home/ur3/catkin_finalproj/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'cconsttype': 'const int', 'value': 3, 'ctype': 'int', 'type': 'int', 'name': 'ProjectorOn'}]}", 'default': 2, 'level': 31, 'min': 1, 'type': 'int'}, {'srcline': 290, 'description': 'Indicates if the projector should turn off when the prosilica camera is exposing.', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'prosilica_projector_inhibit', 'edit_method': '', 'default': False, 'level': 16, 'min': False, 'type': 'bool'}, {'srcline': 290, 'description': 'Indicates the frame rate for both stereo cameras in Hz. (Gets rounded to suitable divisors of projector_rate.)', 'max': 60.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'stereo_rate', 'edit_method': '', 'default': 30.0, 'level': 3, 'min': 1.0, 'type': 'double'}, {'srcline': 290, 'description': 'Indicates the triggering mode of the wide stereo camera.', 'max': 4, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'wide_stereo_trig_mode', 'edit_method': "{'enum_description': 'The triggering mode for the wide camera.', 'enum': [{'srcline': 62, 'description': 'The cameras frequency can be set independently of the projector frequency. There is no deterministic phase relation between projector firing and camera triggering.', 'srcfile': '/home/ur3/catkin_finalproj/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'cconsttype': 'const int', 'value': 2, 'ctype': 'int', 'type': 'int', 'name': 'IgnoreProjector'}, {'srcline': 63, 'description': 'The camera always exposes while the projector is on.', 'srcfile': '/home/ur3/catkin_finalproj/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'cconsttype': 'const int', 'value': 3, 'ctype': 'int', 'type': 'int', 'name': 'WithProjector'}, {'srcline': 64, 'description': 'The camera always exposes while the projector is off.', 'srcfile': '/home/ur3/catkin_finalproj/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'cconsttype': 'const int', 'value': 4, 'ctype': 'int', 'type': 'int', 'name': 'WithoutProjector'}]}", 'default': 4, 'level': 3, 'min': 2, 'type': 'int'}, {'srcline': 290, 'description': 'Indicates the triggering mode of the narrow stereo camera.', 'max': 5, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'narrow_stereo_trig_mode', 'edit_method': "{'enum_description': 'The triggering mode for the narrow camera.', 'enum': [{'srcline': 62, 'description': 'The cameras frequency can be set independently of the projector frequency. There is no deterministic phase relation between projector firing and camera triggering.', 'srcfile': '/home/ur3/catkin_finalproj/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'cconsttype': 'const int', 'value': 2, 'ctype': 'int', 'type': 'int', 'name': 'IgnoreProjector'}, {'srcline': 63, 'description': 'The camera always exposes while the projector is on.', 'srcfile': '/home/ur3/catkin_finalproj/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'cconsttype': 'const int', 'value': 3, 'ctype': 'int', 'type': 'int', 'name': 'WithProjector'}, {'srcline': 64, 'description': 'The camera always exposes while the projector is off.', 'srcfile': '/home/ur3/catkin_finalproj/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'cconsttype': 'const int', 'value': 4, 'ctype': 'int', 'type': 'int', 'name': 'WithoutProjector'}, {'srcline': 65, 'description': 'The camera alternates between frames with and without the projector.', 'srcfile': '/home/ur3/catkin_finalproj/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'cconsttype': 'const int', 'value': 5, 'ctype': 'int', 'type': 'int', 'name': 'AlternateProjector'}]}", 'default': 4, 'level': 3, 'min': 2, 'type': 'int'}, {'srcline': 290, 'description': 'Indicates the frame rate for the right forearm camera in Hz. (Gets rounded to suitable divisors of projector_rate.)', 'max': 60.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'forearm_r_rate', 'edit_method': '', 'default': 30.0, 'level': 4, 'min': 1.0, 'type': 'double'}, {'srcline': 290, 'description': 'Indicates the triggering mode of the right forearm camera.', 'max': 4, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'forearm_r_trig_mode', 'edit_method': "{'enum_description': 'The triggering mode for a forearm camera.', 'enum': [{'srcline': 61, 'description': 'The camera does not use the trigger input.', 'srcfile': '/home/ur3/catkin_finalproj/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': 'InternalTrigger'}, {'srcline': 62, 'description': 'The cameras frequency can be set independently of the projector frequency. There is no deterministic phase relation between projector firing and camera triggering.', 'srcfile': '/home/ur3/catkin_finalproj/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'cconsttype': 'const int', 'value': 2, 'ctype': 'int', 'type': 'int', 'name': 'IgnoreProjector'}, {'srcline': 63, 'description': 'The camera always exposes while the projector is on.', 'srcfile': '/home/ur3/catkin_finalproj/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'cconsttype': 'const int', 'value': 3, 'ctype': 'int', 'type': 'int', 'name': 'WithProjector'}, {'srcline': 64, 'description': 'The camera always exposes while the projector is off.', 'srcfile': '/home/ur3/catkin_finalproj/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'cconsttype': 'const int', 'value': 4, 'ctype': 'int', 'type': 'int', 'name': 'WithoutProjector'}]}", 'default': 1, 'level': 4, 'min': 1, 'type': 'int'}, {'srcline': 290, 'description': 'Indicates the frame rate for the left forearm camera in Hz. (Gets rounded to suitable divisors of projector_rate.)', 'max': 60.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'forearm_l_rate', 'edit_method': '', 'default': 30.0, 'level': 8, 'min': 1.0, 'type': 'double'}, {'srcline': 290, 'description': 'Indicates the triggering mode of the left forearm camera.', 'max': 4, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'forearm_l_trig_mode', 'edit_method': "{'enum_description': 'The triggering mode for a forearm camera.', 'enum': [{'srcline': 61, 'description': 'The camera does not use the trigger input.', 'srcfile': '/home/ur3/catkin_finalproj/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': 'InternalTrigger'}, {'srcline': 62, 'description': 'The cameras frequency can be set independently of the projector frequency. There is no deterministic phase relation between projector firing and camera triggering.', 'srcfile': '/home/ur3/catkin_finalproj/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'cconsttype': 'const int', 'value': 2, 'ctype': 'int', 'type': 'int', 'name': 'IgnoreProjector'}, {'srcline': 63, 'description': 'The camera always exposes while the projector is on.', 'srcfile': '/home/ur3/catkin_finalproj/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'cconsttype': 'const int', 'value': 3, 'ctype': 'int', 'type': 'int', 'name': 'WithProjector'}, {'srcline': 64, 'description': 'The camera always exposes while the projector is off.', 'srcfile': '/home/ur3/catkin_finalproj/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'cconsttype': 'const int', 'value': 4, 'ctype': 'int', 'type': 'int', 'name': 'WithoutProjector'}]}", 'default': 1, 'level': 8, 'min': 1, 'type': 'int'}, {'srcline': 290, 'description': 'Adds a time shift in seconds to the projector timing. Useful for debugging but not in normal use.', 'max': 0.1, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'projector_tweak', 'edit_method': '', 'default': 0.0, 'level': 31, 'min': -0.1, 'type': 'double'}, {'srcline': 290, 'description': 'Does a hard reset of all the cameras using a long pulse on the trigger line. This parameter resets itself to false after 3 to 4 seconds.', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'camera_reset', 'edit_method': '', 'default': False, 'level': 31, 'min': False, 'type': 'bool'}], 'type': '', 'id': 0}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

CameraSynchronizer_ProjectorOff = 1
CameraSynchronizer_ProjectorAuto = 2
CameraSynchronizer_ProjectorOn = 3
CameraSynchronizer_InternalTrigger = 1
CameraSynchronizer_IgnoreProjector = 2
CameraSynchronizer_WithProjector = 3
CameraSynchronizer_WithoutProjector = 4
CameraSynchronizer_AlternateProjector = 5
