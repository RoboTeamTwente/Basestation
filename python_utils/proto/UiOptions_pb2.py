# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: UiOptions.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='UiOptions.proto',
  package='proto',
  syntax='proto3',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\x0fUiOptions.proto\x12\x05proto\"`\n\x06Slider\x12\x0c\n\x04text\x18\x01 \x01(\t\x12\x0b\n\x03min\x18\x02 \x01(\x02\x12\x0b\n\x03max\x18\x03 \x01(\x02\x12\x10\n\x08interval\x18\x04 \x01(\x02\x12\x0f\n\x07\x64\x65\x66\x61ult\x18\x05 \x01(\x02\x12\x0b\n\x03\x64pi\x18\x06 \x01(\x05\"\x18\n\x08\x43heckbox\x12\x0c\n\x04text\x18\x01 \x01(\t\")\n\x08\x44ropdown\x12\x0c\n\x04text\x18\x01 \x01(\t\x12\x0f\n\x07options\x18\x03 \x03(\t\"\x1e\n\x0bRadioButton\x12\x0f\n\x07options\x18\x02 \x03(\t\"\x19\n\tTextField\x12\x0c\n\x04text\x18\x01 \x01(\t\"\xb9\x02\n\x13UiOptionDeclaration\x12\x0c\n\x04path\x18\x01 \x01(\t\x12\x12\n\nis_mutable\x18\x02 \x01(\x08\x12\x13\n\x0b\x64\x65scription\x18\x03 \x01(\t\x12\x1f\n\x06slider\x18\x04 \x01(\x0b\x32\r.proto.SliderH\x00\x12#\n\x08\x63heckbox\x18\x05 \x01(\x0b\x32\x0f.proto.CheckboxH\x00\x12#\n\x08\x64ropdown\x18\x06 \x01(\x0b\x32\x0f.proto.DropdownH\x00\x12)\n\x0bradiobutton\x18\x07 \x01(\x0b\x32\x12.proto.RadioButtonH\x00\x12%\n\ttextfield\x18\x08 \x01(\x0b\x32\x10.proto.TextFieldH\x00\x12\x1f\n\x07\x64\x65\x66\x61ult\x18\t \x01(\x0b\x32\x0e.proto.UiValueB\r\n\x0bui_elements\"C\n\x14UiOptionDeclarations\x12+\n\x07options\x18\x01 \x03(\x0b\x32\x1a.proto.UiOptionDeclaration\"n\n\x07UiValue\x12\x15\n\x0b\x66loat_value\x18\x01 \x01(\x02H\x00\x12\x14\n\nbool_value\x18\x02 \x01(\x08H\x00\x12\x17\n\rinteger_value\x18\x03 \x01(\x03H\x00\x12\x14\n\ntext_value\x18\x04 \x01(\tH\x00\x42\x07\n\x05value\"}\n\x08UiValues\x12\x30\n\tui_values\x18\x01 \x03(\x0b\x32\x1d.proto.UiValues.UiValuesEntry\x1a?\n\rUiValuesEntry\x12\x0b\n\x03key\x18\x01 \x01(\t\x12\x1d\n\x05value\x18\x02 \x01(\x0b\x32\x0e.proto.UiValue:\x02\x38\x01\x62\x06proto3'
)




_SLIDER = _descriptor.Descriptor(
  name='Slider',
  full_name='proto.Slider',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='text', full_name='proto.Slider.text', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='min', full_name='proto.Slider.min', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='max', full_name='proto.Slider.max', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='interval', full_name='proto.Slider.interval', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='default', full_name='proto.Slider.default', index=4,
      number=5, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='dpi', full_name='proto.Slider.dpi', index=5,
      number=6, type=5, cpp_type=1, label=1,
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
  serialized_start=26,
  serialized_end=122,
)


_CHECKBOX = _descriptor.Descriptor(
  name='Checkbox',
  full_name='proto.Checkbox',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='text', full_name='proto.Checkbox.text', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
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
  serialized_start=124,
  serialized_end=148,
)


_DROPDOWN = _descriptor.Descriptor(
  name='Dropdown',
  full_name='proto.Dropdown',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='text', full_name='proto.Dropdown.text', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='options', full_name='proto.Dropdown.options', index=1,
      number=3, type=9, cpp_type=9, label=3,
      has_default_value=False, default_value=[],
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
  serialized_start=150,
  serialized_end=191,
)


_RADIOBUTTON = _descriptor.Descriptor(
  name='RadioButton',
  full_name='proto.RadioButton',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='options', full_name='proto.RadioButton.options', index=0,
      number=2, type=9, cpp_type=9, label=3,
      has_default_value=False, default_value=[],
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
  serialized_start=193,
  serialized_end=223,
)


_TEXTFIELD = _descriptor.Descriptor(
  name='TextField',
  full_name='proto.TextField',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='text', full_name='proto.TextField.text', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
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
  serialized_start=225,
  serialized_end=250,
)


_UIOPTIONDECLARATION = _descriptor.Descriptor(
  name='UiOptionDeclaration',
  full_name='proto.UiOptionDeclaration',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='path', full_name='proto.UiOptionDeclaration.path', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='is_mutable', full_name='proto.UiOptionDeclaration.is_mutable', index=1,
      number=2, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='description', full_name='proto.UiOptionDeclaration.description', index=2,
      number=3, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='slider', full_name='proto.UiOptionDeclaration.slider', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='checkbox', full_name='proto.UiOptionDeclaration.checkbox', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='dropdown', full_name='proto.UiOptionDeclaration.dropdown', index=5,
      number=6, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='radiobutton', full_name='proto.UiOptionDeclaration.radiobutton', index=6,
      number=7, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='textfield', full_name='proto.UiOptionDeclaration.textfield', index=7,
      number=8, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='default', full_name='proto.UiOptionDeclaration.default', index=8,
      number=9, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
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
    _descriptor.OneofDescriptor(
      name='ui_elements', full_name='proto.UiOptionDeclaration.ui_elements',
      index=0, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
  ],
  serialized_start=253,
  serialized_end=566,
)


_UIOPTIONDECLARATIONS = _descriptor.Descriptor(
  name='UiOptionDeclarations',
  full_name='proto.UiOptionDeclarations',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='options', full_name='proto.UiOptionDeclarations.options', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
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
  serialized_start=568,
  serialized_end=635,
)


_UIVALUE = _descriptor.Descriptor(
  name='UiValue',
  full_name='proto.UiValue',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='float_value', full_name='proto.UiValue.float_value', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='bool_value', full_name='proto.UiValue.bool_value', index=1,
      number=2, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='integer_value', full_name='proto.UiValue.integer_value', index=2,
      number=3, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='text_value', full_name='proto.UiValue.text_value', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
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
    _descriptor.OneofDescriptor(
      name='value', full_name='proto.UiValue.value',
      index=0, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
  ],
  serialized_start=637,
  serialized_end=747,
)


_UIVALUES_UIVALUESENTRY = _descriptor.Descriptor(
  name='UiValuesEntry',
  full_name='proto.UiValues.UiValuesEntry',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='key', full_name='proto.UiValues.UiValuesEntry.key', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='value', full_name='proto.UiValues.UiValuesEntry.value', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=b'8\001',
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=811,
  serialized_end=874,
)

_UIVALUES = _descriptor.Descriptor(
  name='UiValues',
  full_name='proto.UiValues',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='ui_values', full_name='proto.UiValues.ui_values', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[_UIVALUES_UIVALUESENTRY, ],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=749,
  serialized_end=874,
)

_UIOPTIONDECLARATION.fields_by_name['slider'].message_type = _SLIDER
_UIOPTIONDECLARATION.fields_by_name['checkbox'].message_type = _CHECKBOX
_UIOPTIONDECLARATION.fields_by_name['dropdown'].message_type = _DROPDOWN
_UIOPTIONDECLARATION.fields_by_name['radiobutton'].message_type = _RADIOBUTTON
_UIOPTIONDECLARATION.fields_by_name['textfield'].message_type = _TEXTFIELD
_UIOPTIONDECLARATION.fields_by_name['default'].message_type = _UIVALUE
_UIOPTIONDECLARATION.oneofs_by_name['ui_elements'].fields.append(
  _UIOPTIONDECLARATION.fields_by_name['slider'])
_UIOPTIONDECLARATION.fields_by_name['slider'].containing_oneof = _UIOPTIONDECLARATION.oneofs_by_name['ui_elements']
_UIOPTIONDECLARATION.oneofs_by_name['ui_elements'].fields.append(
  _UIOPTIONDECLARATION.fields_by_name['checkbox'])
_UIOPTIONDECLARATION.fields_by_name['checkbox'].containing_oneof = _UIOPTIONDECLARATION.oneofs_by_name['ui_elements']
_UIOPTIONDECLARATION.oneofs_by_name['ui_elements'].fields.append(
  _UIOPTIONDECLARATION.fields_by_name['dropdown'])
_UIOPTIONDECLARATION.fields_by_name['dropdown'].containing_oneof = _UIOPTIONDECLARATION.oneofs_by_name['ui_elements']
_UIOPTIONDECLARATION.oneofs_by_name['ui_elements'].fields.append(
  _UIOPTIONDECLARATION.fields_by_name['radiobutton'])
_UIOPTIONDECLARATION.fields_by_name['radiobutton'].containing_oneof = _UIOPTIONDECLARATION.oneofs_by_name['ui_elements']
_UIOPTIONDECLARATION.oneofs_by_name['ui_elements'].fields.append(
  _UIOPTIONDECLARATION.fields_by_name['textfield'])
_UIOPTIONDECLARATION.fields_by_name['textfield'].containing_oneof = _UIOPTIONDECLARATION.oneofs_by_name['ui_elements']
_UIOPTIONDECLARATIONS.fields_by_name['options'].message_type = _UIOPTIONDECLARATION
_UIVALUE.oneofs_by_name['value'].fields.append(
  _UIVALUE.fields_by_name['float_value'])
_UIVALUE.fields_by_name['float_value'].containing_oneof = _UIVALUE.oneofs_by_name['value']
_UIVALUE.oneofs_by_name['value'].fields.append(
  _UIVALUE.fields_by_name['bool_value'])
_UIVALUE.fields_by_name['bool_value'].containing_oneof = _UIVALUE.oneofs_by_name['value']
_UIVALUE.oneofs_by_name['value'].fields.append(
  _UIVALUE.fields_by_name['integer_value'])
_UIVALUE.fields_by_name['integer_value'].containing_oneof = _UIVALUE.oneofs_by_name['value']
_UIVALUE.oneofs_by_name['value'].fields.append(
  _UIVALUE.fields_by_name['text_value'])
_UIVALUE.fields_by_name['text_value'].containing_oneof = _UIVALUE.oneofs_by_name['value']
_UIVALUES_UIVALUESENTRY.fields_by_name['value'].message_type = _UIVALUE
_UIVALUES_UIVALUESENTRY.containing_type = _UIVALUES
_UIVALUES.fields_by_name['ui_values'].message_type = _UIVALUES_UIVALUESENTRY
DESCRIPTOR.message_types_by_name['Slider'] = _SLIDER
DESCRIPTOR.message_types_by_name['Checkbox'] = _CHECKBOX
DESCRIPTOR.message_types_by_name['Dropdown'] = _DROPDOWN
DESCRIPTOR.message_types_by_name['RadioButton'] = _RADIOBUTTON
DESCRIPTOR.message_types_by_name['TextField'] = _TEXTFIELD
DESCRIPTOR.message_types_by_name['UiOptionDeclaration'] = _UIOPTIONDECLARATION
DESCRIPTOR.message_types_by_name['UiOptionDeclarations'] = _UIOPTIONDECLARATIONS
DESCRIPTOR.message_types_by_name['UiValue'] = _UIVALUE
DESCRIPTOR.message_types_by_name['UiValues'] = _UIVALUES
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Slider = _reflection.GeneratedProtocolMessageType('Slider', (_message.Message,), {
  'DESCRIPTOR' : _SLIDER,
  '__module__' : 'UiOptions_pb2'
  # @@protoc_insertion_point(class_scope:proto.Slider)
  })
_sym_db.RegisterMessage(Slider)

Checkbox = _reflection.GeneratedProtocolMessageType('Checkbox', (_message.Message,), {
  'DESCRIPTOR' : _CHECKBOX,
  '__module__' : 'UiOptions_pb2'
  # @@protoc_insertion_point(class_scope:proto.Checkbox)
  })
_sym_db.RegisterMessage(Checkbox)

Dropdown = _reflection.GeneratedProtocolMessageType('Dropdown', (_message.Message,), {
  'DESCRIPTOR' : _DROPDOWN,
  '__module__' : 'UiOptions_pb2'
  # @@protoc_insertion_point(class_scope:proto.Dropdown)
  })
_sym_db.RegisterMessage(Dropdown)

RadioButton = _reflection.GeneratedProtocolMessageType('RadioButton', (_message.Message,), {
  'DESCRIPTOR' : _RADIOBUTTON,
  '__module__' : 'UiOptions_pb2'
  # @@protoc_insertion_point(class_scope:proto.RadioButton)
  })
_sym_db.RegisterMessage(RadioButton)

TextField = _reflection.GeneratedProtocolMessageType('TextField', (_message.Message,), {
  'DESCRIPTOR' : _TEXTFIELD,
  '__module__' : 'UiOptions_pb2'
  # @@protoc_insertion_point(class_scope:proto.TextField)
  })
_sym_db.RegisterMessage(TextField)

UiOptionDeclaration = _reflection.GeneratedProtocolMessageType('UiOptionDeclaration', (_message.Message,), {
  'DESCRIPTOR' : _UIOPTIONDECLARATION,
  '__module__' : 'UiOptions_pb2'
  # @@protoc_insertion_point(class_scope:proto.UiOptionDeclaration)
  })
_sym_db.RegisterMessage(UiOptionDeclaration)

UiOptionDeclarations = _reflection.GeneratedProtocolMessageType('UiOptionDeclarations', (_message.Message,), {
  'DESCRIPTOR' : _UIOPTIONDECLARATIONS,
  '__module__' : 'UiOptions_pb2'
  # @@protoc_insertion_point(class_scope:proto.UiOptionDeclarations)
  })
_sym_db.RegisterMessage(UiOptionDeclarations)

UiValue = _reflection.GeneratedProtocolMessageType('UiValue', (_message.Message,), {
  'DESCRIPTOR' : _UIVALUE,
  '__module__' : 'UiOptions_pb2'
  # @@protoc_insertion_point(class_scope:proto.UiValue)
  })
_sym_db.RegisterMessage(UiValue)

UiValues = _reflection.GeneratedProtocolMessageType('UiValues', (_message.Message,), {

  'UiValuesEntry' : _reflection.GeneratedProtocolMessageType('UiValuesEntry', (_message.Message,), {
    'DESCRIPTOR' : _UIVALUES_UIVALUESENTRY,
    '__module__' : 'UiOptions_pb2'
    # @@protoc_insertion_point(class_scope:proto.UiValues.UiValuesEntry)
    })
  ,
  'DESCRIPTOR' : _UIVALUES,
  '__module__' : 'UiOptions_pb2'
  # @@protoc_insertion_point(class_scope:proto.UiValues)
  })
_sym_db.RegisterMessage(UiValues)
_sym_db.RegisterMessage(UiValues.UiValuesEntry)


_UIVALUES_UIVALUESENTRY._options = None
# @@protoc_insertion_point(module_scope)
