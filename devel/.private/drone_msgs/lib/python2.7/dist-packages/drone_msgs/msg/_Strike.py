# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from drone_msgs/Strike.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class Strike(genpy.Message):
  _md5sum = "bc5465527f2e02efe558071ee95658cf"
  _type = "drone_msgs/Strike"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """int8 id_drone   # the id of the drone that shoots (0,1..n)
int8 team_num   # the number of team of the drone that shoots (0,1..n)
float32 shot  # the force of the shot"""
  __slots__ = ['id_drone','team_num','shot']
  _slot_types = ['int8','int8','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       id_drone,team_num,shot

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Strike, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.id_drone is None:
        self.id_drone = 0
      if self.team_num is None:
        self.team_num = 0
      if self.shot is None:
        self.shot = 0.
    else:
      self.id_drone = 0
      self.team_num = 0
      self.shot = 0.

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
      buff.write(_get_struct_2bf().pack(_x.id_drone, _x.team_num, _x.shot))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 6
      (_x.id_drone, _x.team_num, _x.shot,) = _get_struct_2bf().unpack(str[start:end])
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
      buff.write(_get_struct_2bf().pack(_x.id_drone, _x.team_num, _x.shot))
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
      end = 0
      _x = self
      start = end
      end += 6
      (_x.id_drone, _x.team_num, _x.shot,) = _get_struct_2bf().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2bf = None
def _get_struct_2bf():
    global _struct_2bf
    if _struct_2bf is None:
        _struct_2bf = struct.Struct("<2bf")
    return _struct_2bf
