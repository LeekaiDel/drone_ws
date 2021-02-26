# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from drone_msgs/WindowPointDir.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import drone_msgs.msg
import geometry_msgs.msg

class WindowPointDir(genpy.Message):
  _md5sum = "6e775f1bde836c88e4039d51e180cd67"
  _type = "drone_msgs/WindowPointDir"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """ bool found_window
 drone_msgs/DronePose point

================================================================================
MSG: drone_msgs/DronePose
geometry_msgs/Point point
float32 course

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
"""
  __slots__ = ['found_window','point']
  _slot_types = ['bool','drone_msgs/DronePose']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       found_window,point

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(WindowPointDir, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.found_window is None:
        self.found_window = False
      if self.point is None:
        self.point = drone_msgs.msg.DronePose()
    else:
      self.found_window = False
      self.point = drone_msgs.msg.DronePose()

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
      buff.write(_get_struct_B3df().pack(_x.found_window, _x.point.point.x, _x.point.point.y, _x.point.point.z, _x.point.course))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.point is None:
        self.point = drone_msgs.msg.DronePose()
      end = 0
      _x = self
      start = end
      end += 29
      (_x.found_window, _x.point.point.x, _x.point.point.y, _x.point.point.z, _x.point.course,) = _get_struct_B3df().unpack(str[start:end])
      self.found_window = bool(self.found_window)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_B3df().pack(_x.found_window, _x.point.point.x, _x.point.point.y, _x.point.point.z, _x.point.course))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.point is None:
        self.point = drone_msgs.msg.DronePose()
      end = 0
      _x = self
      start = end
      end += 29
      (_x.found_window, _x.point.point.x, _x.point.point.y, _x.point.point.z, _x.point.course,) = _get_struct_B3df().unpack(str[start:end])
      self.found_window = bool(self.found_window)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_B3df = None
def _get_struct_B3df():
    global _struct_B3df
    if _struct_B3df is None:
        _struct_B3df = struct.Struct("<B3df")
    return _struct_B3df
