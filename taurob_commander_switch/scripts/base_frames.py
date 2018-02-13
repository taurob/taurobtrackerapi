import re

YV_DIV_FACTOR = 2700.
XV_DIV_FACTOR = -450.

def unpack_command(data):
    m_seqno = re.search('SQ:(?P<seqno>\d+)', data)
    if m_seqno is None:
        raise ValueError('seqno not found')

    m_xv = re.search('XV:(?P<xv>-?\d+)', data)
    if m_xv is None:
        raise ValueError('xv not found')

    m_yv = re.search('YV:(?P<yv>-?\d+)', data)
    if m_yv is None:
        raise ValueError('yv not found')

    xv = int(m_xv.group('xv'))
    yv = int(m_yv.group('yv'))

    res = {
        'seqno': int(m_seqno.group('seqno')),
        'linear': yv / YV_DIV_FACTOR,
        'angular': xv / XV_DIV_FACTOR
    }

    return res

def pack_status(data):
    pass
