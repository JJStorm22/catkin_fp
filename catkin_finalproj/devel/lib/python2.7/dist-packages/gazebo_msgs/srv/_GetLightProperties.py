# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from gazebo_msgs/GetLightPropertiesRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class GetLightPropertiesRequest(genpy.Message):
  _md5sum = "4fb676dfb4741fc866365702a859441c"
  _type = "gazebo_msgs/GetLightPropertiesRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """string light_name
"""
  __slots__ = ['light_name']
  _slot_types = ['string']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       light_name

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(GetLightPropertiesRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.light_name is None:
        self.light_name = ''
    else:
      self.light_name = ''

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self.light_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.light_name = str[start:end].decode('utf-8')
      else:
        self.light_name = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self.light_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.light_name = str[start:end].decode('utf-8')
      else:
        self.light_name = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from gazebo_msgs/GetLightPropertiesResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class GetLightPropertiesResponse(genpy.Message):
  _md5sum = "9a19ddd5aab4c13b7643d1722c709f1f"
  _type = "gazebo_msgs/GetLightPropertiesResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """std_msgs/ColorRGBA diffuse
float64 attenuation_constant
float64 attenuation_linear
float64 attenuation_quadratic
bool success
string status_message


================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
"""
  __slots__ = ['diffuse','attenuation_constant','attenuation_linear','attenuation_quadratic','success','status_message']
  _slot_types = ['std_msgs/ColorRGBA','float64','float64','float64','bool','string']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       diffuse,attenuation_constant,attenuation_linear,attenuation_quadratic,success,status_message

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(GetLightPropertiesResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.diffuse is None:
        self.diffuse = std_msgs.msg.ColorRGBA()
      if self.attenuation_constant is None:
        self.attenuation_constant = 0.
      if self.attenuation_linear is None:
        self.attenuation_linear = 0.
      if self.attenuation_quadratic is None:
        self.attenuation_quadratic = 0.
      if self.success is None:
        self.success = False
      if self.status_message is None:
        self.status_message = ''
    else:
      self.diffuse = std_msgs.msg.ColorRGBA()
      self.attenuation_constant = 0.
      self.attenuation_linear = 0.
      self.attenuation_quadratic = 0.
      self.success = False
      self.status_message = ''

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_4f3dB().pack(_x.diffuse.r, _x.diffuse.g, _x.diffuse.b, _x.diffuse.a, _x.attenuation_constant, _x.attenuation_linear, _x.attenuation_quadratic, _x.success))
      _x = self.status_message
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.diffuse is None:
        self.diffuse = std_msgs.msg.ColorRGBA()
      end = 0
      _x = self
      start = end
      end += 41
      (_x.diffuse.r, _x.diffuse.g, _x.diffuse.b, _x.diffuse.a, _x.attenuation_constant, _x.attenuation_linear, _x.attenuation_quadratic, _x.success,) = _get_struct_4f3dB().unpack(str[start:end])
      self.success = bool(self.success)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.status_message = str[start:end].decode('utf-8')
      else:
        self.status_message = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_4f3dB().pack(_x.diffuse.r, _x.diffuse.g, _x.diffuse.b, _x.diffuse.a, _x.attenuation_constant, _x.attenuation_linear, _x.attenuation_quadratic, _x.success))
      _x = self.status_message
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.diffuse is None:
        self.diffuse = std_msgs.msg.ColorRGBA()
      end = 0
      _x = self
      start = end
      end += 41
      (_x.diffuse.r, _x.diffuse.g, _x.diffuse.b, _x.diffuse.a, _x.attenuation_constant, _x.attenuation_linear, _x.attenuation_quadratic, _x.success,) = _get_struct_4f3dB().unpack(str[start:end])
      self.success = bool(self.success)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.status_message = str[start:end].decode('utf-8')
      else:
        self.status_message = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_4f3dB = None
def _get_struct_4f3dB():
    global _struct_4f3dB
    if _struct_4f3dB is None:
        _struct_4f3dB = struct.Struct("<4f3dB")
    return _struct_4f3dB
class GetLightProperties(object):
  _type          = 'gazebo_msgs/GetLightProperties'
  _md5sum = 'df2cef87e13e2e6990e81e8aaa454c19'
  _request_class  = GetLightPropertiesRequest
  _response_class = GetLightPropertiesResponse