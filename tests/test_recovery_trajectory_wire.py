import math
import struct

import pytest

from aic_transfuser_lite.runtime.recovery_trajectory_wire import decode_trajectory_summary


def wire(*, endian='<', frame='map', speeds=None, sec=135, nanosec=179_996_978):
    speeds = [5/3.6]*30 if speeds is None else speeds
    name=frame.encode()+b'\0'
    body=bytearray(struct.pack(endian+'iII',sec,nanosec,len(name))+name)
    body+=bytes((-len(body))%4)
    body+=struct.pack(endian+'I',len(speeds))
    body+=bytes((-len(body))%8)
    for i,speed in enumerate(speeds):
        body+=struct.pack(endian+'iI7d6f',i,0,89600+i,43100.,0.,0.,0.,0.,1.,speed,0.,0.,0.,0.,0.)
    return (b'\x00\x01\0\0' if endian=='<' else b'\x00\0\0\0')+body


@pytest.mark.parametrize('endian',['<','>'])
@pytest.mark.parametrize('frame',['map','odom','base_link'])
def test_full_pinned_layout_both_byte_orders_and_header_alignment(endian,frame):
    r=decode_trajectory_summary(wire(endian=endian,frame=frame))
    assert (r.header.stamp.sec,r.header.stamp.nanosec)==(135,179_996_978)
    assert r.header.frame_id==frame and r.point_count==30 and r.target_speed_valid


@pytest.mark.parametrize('bad_speed',[float('nan'),float('inf'),-1.,0.,1.38892])
@pytest.mark.parametrize('index',[0,14,29])
def test_speed_check_covers_every_point(bad_speed,index):
    values=[5/3.6]*30;values[index]=bad_speed
    assert not decode_trajectory_summary(wire(speeds=values)).target_speed_valid


@pytest.mark.parametrize('delta',[-1e-5,-9e-6,0.,9e-6,1e-5])
def test_float32_speed_predicate_matches_original_math_check(delta):
    value=struct.unpack('<f',struct.pack('<f',5/3.6+delta))[0]
    assert decode_trajectory_summary(wire(speeds=[value]*20)).target_speed_valid == math.isclose(value,5/3.6,abs_tol=1e-5)


def test_large_message_keeps_only_compact_metadata():
    r=decode_trajectory_summary(wire(speeds=[5/3.6]*2866))
    assert r.point_count==2866 and r.target_speed_valid
    assert not hasattr(r,'points')


@pytest.mark.parametrize('kind',['short','encoding','negative_time','nanoseconds','frame_size','nul','count','truncated','trailing'])
def test_bad_wire_data_fails_explicitly(kind):
    b=bytearray(wire())
    if kind=='short':b=b[:10]
    elif kind=='encoding':b[1]=9
    elif kind=='negative_time':struct.pack_into('<i',b,4,-1)
    elif kind=='nanoseconds':struct.pack_into('<I',b,8,1_000_000_000)
    elif kind=='frame_size':struct.pack_into('<I',b,12,257)
    elif kind=='nul':b[19]=1
    elif kind=='count':struct.pack_into('<I',b,20,10001)
    elif kind=='truncated':b=b[:-1]
    elif kind=='trailing':b+=b'\0'
    with pytest.raises(ValueError,match='TRAJECTORY_CDR_'):
        decode_trajectory_summary(bytes(b))
