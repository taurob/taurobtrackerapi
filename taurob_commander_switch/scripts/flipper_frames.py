import struct
from PyCRC.CRC16 import CRC16

def unpack_command(data):
    t = struct.unpack('!BBBBhH', data)

    if t[0] != 1:
        raise ValueError('wrong frame ID')

    if t[1] != 1:
        raise ValueError('wrong protocol version')

    res = {
        'seqno': t[2],
        'active': bool(t[3] & 0x01),
        'angle': t[4] / 1000.
    }

    return res

def pack_status(data):
    packed = struct.pack('!BBBBIHhhH',
        2, # frame ID
        1, # protcol version
        data['seqno'],
        0, # flags
        0, 0, # timestamps
        int(data['angle'] * 1000),
        0, # speed
        0, # current
    )

    crc = CRC16(modbus_flag=True).calculate(packed)
    packed += struct.pack('!H', crc)

    return packed
