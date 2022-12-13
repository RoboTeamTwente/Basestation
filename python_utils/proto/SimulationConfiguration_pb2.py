# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: SimulationConfiguration.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import proto.Vector2f_pb2 as Vector2f__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='SimulationConfiguration.proto',
  package='proto',
  syntax='proto3',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\x1dSimulationConfiguration.proto\x12\x05proto\x1a\x0eVector2f.proto\"\xbd\x01\n\x16SimulationBallLocation\x12\t\n\x01x\x18\x01 \x01(\x02\x12\t\n\x01y\x18\x02 \x01(\x02\x12\t\n\x01z\x18\x03 \x01(\x02\x12\x12\n\nx_velocity\x18\x04 \x01(\x02\x12\x12\n\ny_velocity\x18\x05 \x01(\x02\x12\x12\n\nz_velocity\x18\x06 \x01(\x02\x12\x1b\n\x13velocity_in_rolling\x18\x07 \x01(\x08\x12\x17\n\x0fteleport_safely\x18\x08 \x01(\x08\x12\x10\n\x08\x62y_force\x18\t \x01(\x08\"\xd6\x01\n\x17SimulationRobotLocation\x12\n\n\x02id\x18\x01 \x01(\r\x12\x16\n\x0eis_team_yellow\x18\x02 \x01(\x08\x12\t\n\x01x\x18\x03 \x01(\x02\x12\t\n\x01y\x18\x04 \x01(\x02\x12\x12\n\nx_velocity\x18\x05 \x01(\x02\x12\x12\n\ny_velocity\x18\x06 \x01(\x02\x12\x18\n\x10\x61ngular_velocity\x18\x07 \x01(\x02\x12\x13\n\x0borientation\x18\x08 \x01(\x02\x12\x18\n\x10present_on_field\x18\t \x01(\x08\x12\x10\n\x08\x62y_force\x18\n \x01(\x08\"\xee\x03\n\x19SimulationRobotProperties\x12\n\n\x02id\x18\x01 \x01(\r\x12\x16\n\x0eis_team_yellow\x18\x02 \x01(\x08\x12\x0e\n\x06radius\x18\x03 \x01(\x02\x12\x0e\n\x06height\x18\x04 \x01(\x02\x12\x0c\n\x04mass\x18\x05 \x01(\x02\x12\x16\n\x0emax_kick_speed\x18\x06 \x01(\x02\x12\x16\n\x0emax_chip_speed\x18\x07 \x01(\x02\x12#\n\x1b\x63\x65nter_to_dribbler_distance\x18\x08 \x01(\x02\x12\x18\n\x10max_acceleration\x18\t \x01(\x02\x12 \n\x18max_angular_acceleration\x18\n \x01(\x02\x12\x18\n\x10max_deceleration\x18\x0b \x01(\x02\x12 \n\x18max_angular_deceleration\x18\x0c \x01(\x02\x12\x14\n\x0cmax_velocity\x18\r \x01(\x02\x12\x1c\n\x14max_angular_velocity\x18\x0e \x01(\x02\x12\x1f\n\x17\x66ront_right_wheel_angle\x18\x0f \x01(\x02\x12\x1e\n\x16\x62\x61\x63k_right_wheel_angle\x18\x10 \x01(\x02\x12\x1d\n\x15\x62\x61\x63k_left_wheel_angle\x18\x11 \x01(\x02\x12\x1e\n\x16\x66ront_left_wheel_angle\x18\x12 \x01(\x02\"\xf3\x01\n\x17SimulationConfiguration\x12\x34\n\rball_location\x18\x01 \x01(\x0b\x32\x1d.proto.SimulationBallLocation\x12\x37\n\x0frobot_locations\x18\x02 \x03(\x0b\x32\x1e.proto.SimulationRobotLocation\x12:\n\x10robot_properties\x18\x03 \x03(\x0b\x32 .proto.SimulationRobotProperties\x12\x18\n\x10simulation_speed\x18\x04 \x01(\x02\x12\x13\n\x0bvision_port\x18\x05 \x01(\rb\x06proto3'
  ,
  dependencies=[Vector2f__pb2.DESCRIPTOR,])




_SIMULATIONBALLLOCATION = _descriptor.Descriptor(
  name='SimulationBallLocation',
  full_name='proto.SimulationBallLocation',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='x', full_name='proto.SimulationBallLocation.x', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='y', full_name='proto.SimulationBallLocation.y', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='z', full_name='proto.SimulationBallLocation.z', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='x_velocity', full_name='proto.SimulationBallLocation.x_velocity', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='y_velocity', full_name='proto.SimulationBallLocation.y_velocity', index=4,
      number=5, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='z_velocity', full_name='proto.SimulationBallLocation.z_velocity', index=5,
      number=6, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='velocity_in_rolling', full_name='proto.SimulationBallLocation.velocity_in_rolling', index=6,
      number=7, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='teleport_safely', full_name='proto.SimulationBallLocation.teleport_safely', index=7,
      number=8, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='by_force', full_name='proto.SimulationBallLocation.by_force', index=8,
      number=9, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=57,
  serialized_end=246,
)


_SIMULATIONROBOTLOCATION = _descriptor.Descriptor(
  name='SimulationRobotLocation',
  full_name='proto.SimulationRobotLocation',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='proto.SimulationRobotLocation.id', index=0,
      number=1, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='is_team_yellow', full_name='proto.SimulationRobotLocation.is_team_yellow', index=1,
      number=2, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='x', full_name='proto.SimulationRobotLocation.x', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='y', full_name='proto.SimulationRobotLocation.y', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='x_velocity', full_name='proto.SimulationRobotLocation.x_velocity', index=4,
      number=5, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='y_velocity', full_name='proto.SimulationRobotLocation.y_velocity', index=5,
      number=6, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='angular_velocity', full_name='proto.SimulationRobotLocation.angular_velocity', index=6,
      number=7, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='orientation', full_name='proto.SimulationRobotLocation.orientation', index=7,
      number=8, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='present_on_field', full_name='proto.SimulationRobotLocation.present_on_field', index=8,
      number=9, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='by_force', full_name='proto.SimulationRobotLocation.by_force', index=9,
      number=10, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=249,
  serialized_end=463,
)


_SIMULATIONROBOTPROPERTIES = _descriptor.Descriptor(
  name='SimulationRobotProperties',
  full_name='proto.SimulationRobotProperties',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='proto.SimulationRobotProperties.id', index=0,
      number=1, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='is_team_yellow', full_name='proto.SimulationRobotProperties.is_team_yellow', index=1,
      number=2, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='radius', full_name='proto.SimulationRobotProperties.radius', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='height', full_name='proto.SimulationRobotProperties.height', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='mass', full_name='proto.SimulationRobotProperties.mass', index=4,
      number=5, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='max_kick_speed', full_name='proto.SimulationRobotProperties.max_kick_speed', index=5,
      number=6, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='max_chip_speed', full_name='proto.SimulationRobotProperties.max_chip_speed', index=6,
      number=7, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='center_to_dribbler_distance', full_name='proto.SimulationRobotProperties.center_to_dribbler_distance', index=7,
      number=8, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='max_acceleration', full_name='proto.SimulationRobotProperties.max_acceleration', index=8,
      number=9, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='max_angular_acceleration', full_name='proto.SimulationRobotProperties.max_angular_acceleration', index=9,
      number=10, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='max_deceleration', full_name='proto.SimulationRobotProperties.max_deceleration', index=10,
      number=11, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='max_angular_deceleration', full_name='proto.SimulationRobotProperties.max_angular_deceleration', index=11,
      number=12, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='max_velocity', full_name='proto.SimulationRobotProperties.max_velocity', index=12,
      number=13, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='max_angular_velocity', full_name='proto.SimulationRobotProperties.max_angular_velocity', index=13,
      number=14, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='front_right_wheel_angle', full_name='proto.SimulationRobotProperties.front_right_wheel_angle', index=14,
      number=15, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='back_right_wheel_angle', full_name='proto.SimulationRobotProperties.back_right_wheel_angle', index=15,
      number=16, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='back_left_wheel_angle', full_name='proto.SimulationRobotProperties.back_left_wheel_angle', index=16,
      number=17, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='front_left_wheel_angle', full_name='proto.SimulationRobotProperties.front_left_wheel_angle', index=17,
      number=18, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=466,
  serialized_end=960,
)


_SIMULATIONCONFIGURATION = _descriptor.Descriptor(
  name='SimulationConfiguration',
  full_name='proto.SimulationConfiguration',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='ball_location', full_name='proto.SimulationConfiguration.ball_location', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='robot_locations', full_name='proto.SimulationConfiguration.robot_locations', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='robot_properties', full_name='proto.SimulationConfiguration.robot_properties', index=2,
      number=3, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='simulation_speed', full_name='proto.SimulationConfiguration.simulation_speed', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='vision_port', full_name='proto.SimulationConfiguration.vision_port', index=4,
      number=5, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=963,
  serialized_end=1206,
)

_SIMULATIONCONFIGURATION.fields_by_name['ball_location'].message_type = _SIMULATIONBALLLOCATION
_SIMULATIONCONFIGURATION.fields_by_name['robot_locations'].message_type = _SIMULATIONROBOTLOCATION
_SIMULATIONCONFIGURATION.fields_by_name['robot_properties'].message_type = _SIMULATIONROBOTPROPERTIES
DESCRIPTOR.message_types_by_name['SimulationBallLocation'] = _SIMULATIONBALLLOCATION
DESCRIPTOR.message_types_by_name['SimulationRobotLocation'] = _SIMULATIONROBOTLOCATION
DESCRIPTOR.message_types_by_name['SimulationRobotProperties'] = _SIMULATIONROBOTPROPERTIES
DESCRIPTOR.message_types_by_name['SimulationConfiguration'] = _SIMULATIONCONFIGURATION
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

SimulationBallLocation = _reflection.GeneratedProtocolMessageType('SimulationBallLocation', (_message.Message,), {
  'DESCRIPTOR' : _SIMULATIONBALLLOCATION,
  '__module__' : 'SimulationConfiguration_pb2'
  # @@protoc_insertion_point(class_scope:proto.SimulationBallLocation)
  })
_sym_db.RegisterMessage(SimulationBallLocation)

SimulationRobotLocation = _reflection.GeneratedProtocolMessageType('SimulationRobotLocation', (_message.Message,), {
  'DESCRIPTOR' : _SIMULATIONROBOTLOCATION,
  '__module__' : 'SimulationConfiguration_pb2'
  # @@protoc_insertion_point(class_scope:proto.SimulationRobotLocation)
  })
_sym_db.RegisterMessage(SimulationRobotLocation)

SimulationRobotProperties = _reflection.GeneratedProtocolMessageType('SimulationRobotProperties', (_message.Message,), {
  'DESCRIPTOR' : _SIMULATIONROBOTPROPERTIES,
  '__module__' : 'SimulationConfiguration_pb2'
  # @@protoc_insertion_point(class_scope:proto.SimulationRobotProperties)
  })
_sym_db.RegisterMessage(SimulationRobotProperties)

SimulationConfiguration = _reflection.GeneratedProtocolMessageType('SimulationConfiguration', (_message.Message,), {
  'DESCRIPTOR' : _SIMULATIONCONFIGURATION,
  '__module__' : 'SimulationConfiguration_pb2'
  # @@protoc_insertion_point(class_scope:proto.SimulationConfiguration)
  })
_sym_db.RegisterMessage(SimulationConfiguration)


# @@protoc_insertion_point(module_scope)