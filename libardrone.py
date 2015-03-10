# Python AR.Drone 2.0
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
"""
Python library for the AR.Drone 2.0
Works on windows and Linux
Soon to be ported to Python 3
"""

# Based on code by Bastian Venthur, jbpassot, adetaylor
# https://github.com/venthur
# https://github.com/jbpassot
# https://github.com/adetaylor

import logging
logger = logging.getLogger()
import socket
import struct
import threading
import time
import numpy

import arnetwork

# For video decoding
FFMPEG = r'C:\Program Files (x86)\ffmpeg-20150304-git-7da7d26-win64-static\bin\ffmpeg.exe'

ARDRONE_COMMAND_PORT = 5556
ARDRONE_NAVDATA_PORT = 5554
ARDRONE_VIDEO_PORT = 5555
ARDRONE_CONTROL_PORT = 5559

# Do these have a special meaning ?
SESSION_ID = '943dac23'
USER_ID = '36355d78'
APP_ID = '21d958e4'

# 0: "Not defined"
# 131072:  "Landed"
# 393216:  "Taking-off-Floor"
# 393217:  "Taking-off-Air"
# 262144:  "Hovering"
# 524288:  "Landing"
# 458752:  "Stabilizing"
# 196608:  "Moving"
# 262153 and 196613 and 262155 and 196614 and 458753:  "Undefined"
ctrl_state_dict={0:0, 131072:1, 393216:2, 393217:3, 262144:4, 524288:5, 458752:6, 196608:7, 262153:8, 196613:9, 262155:10, 196614:11, 458753: 12}

def check_str(v):
    assert isinstance(v,str)
    return v
def check_int(low=None,high=None):
    def f(v):
        assert isinstance(v,int)
        assert low is None or low<=v
        assert high is None or v<=high
        return str(v)
    return f
def check_bool(v):
    assert isinstance(v,bool)
    return 'TRUE' if v else 'FALSE'

config_options = {
    'custom:session_id': check_str,
    'custom:profile_id': check_str,
    'custom:application_id': check_str,
    'video:bitrate_control_mode': check_int(low=0,high=1),
    'video:bitrate': check_int(low=1),
    'video:max_bitrate': check_int(low=1),
    'video:video_channel': check_int(low=0,high=1),
    'video:codec_fps': check_int(low=1),
    'video:video_codec': check_int(),
    'general:navdata_demo': check_bool,
    'control:altitutde_max': check_int(low=10,high=100000),
    }
# Possible value for video codec:
# NULL_CODEC    = 0,
# UVLC_CODEC    = 0x20,       // codec_type value is used for START_CODE
# P264_CODEC    = 0x40,
# MP4_360P_CODEC = 0x80,
# H264_360P_CODEC = 0x81,
# MP4_360P_H264_720P_CODEC = 0x82,
# H264_720P_CODEC = 0x83,
# MP4_360P_SLRS_CODEC = 0x84,
# H264_360P_SLRS_CODEC = 0x85,
# H264_720P_SLRS_CODEC = 0x86,
# H264_AUTO_RESIZE_CODEC = 0x87,    // resolution is automatically adjusted according to bitrate
# MP4_360P_H264_360P_CODEC = 0x88,

#==================================================================================================
class ARDrone(object):
    """ARDrone Class.

    Instantiate this class to control your drone and receive decoded video and navdata.
    """
#==================================================================================================

    def __init__(self, ssid=None, hd=False):

        self.seq_nr = 1
        self.timer_t = 0.2
        self.com_watchdog_timer = threading.Timer(self.timer_t, self.commwdg)
        self.lock = threading.Lock()
        self.speed = 0.2
        self.hd = hd
        self.image_shape = (720, 1280, 3) if hd else (360, 640, 3)
        self.config_ids_string = [SESSION_ID, USER_ID, APP_ID]
        self.config({
            'custom:session_id':SESSION_ID,
            'custom:profile_id':USER_ID,
            'custom:application_id':APP_ID,
            'video:bitrate_control_mode':1,
            'video:video_channel':1,
            'video:bitrate':500,
            'video:max_bitrate':500,
            'video:codec_fps':30,
            'video:video_codec':0x83 if hd else 0x81,
            'general:navdata_demo':True,
            'control:altitude_max':20000,
            })
        time.sleep(1.)
        self.image = numpy.zeros(self.image_shape,dtype='uint8')
        self.navdata = dict()
        self.navdata[0] = dict(
          ctrl_state=0,
          battery=0,
          theta=0,
          phi=0,
          psi=0, 
          altitude=0,
          vx=0.,
          vy=0.,
          vz=0.,
          num_frames=0)
        self.ssid = ssid
        self.network = arnetwork.network(self)

    def takeoff(self):
        """Make the drone takeoff."""
        self.at(at_ftrim)
        self.at(at_ref, True)

    def land(self):
        """Make the drone land."""
        self.at(at_ref, False)

    def hover(self):
        """Make the drone hover."""
        self.at(at_pcmd, False, 0, 0, 0, 0)

    def move_left(self):
        """Make the drone move left."""
        self.at(at_pcmd, True, -self.speed, 0, 0, 0)

    def move_right(self):
        """Make the drone move right."""
        self.at(at_pcmd, True, self.speed, 0, 0, 0)

    def move_up(self):
        """Make the drone rise upwards."""
        self.at(at_pcmd, True, 0, 0, self.speed, 0)

    def move_down(self):
        """Make the drone decent downwards."""
        self.at(at_pcmd, True, 0, 0, -self.speed, 0)

    def move_forward(self):
        """Make the drone move forward."""
        self.at(at_pcmd, True, 0, -self.speed, 0, 0)

    def move_backward(self):
        """Make the drone move backwards."""
        self.at(at_pcmd, True, 0, self.speed, 0, 0)

    def turn_left(self):
        """Make the drone rotate left."""
        self.at(at_pcmd, True, 0, 0, 0, -self.speed)

    def turn_right(self):
        """Make the drone rotate right."""
        self.at(at_pcmd, True, 0, 0, 0, self.speed)

    def reset(self):
        """Toggle the drone's emergency state."""
        self.at(at_ftrim)
        time.sleep(0.1)
        self.at(at_ref, False, True)
        time.sleep(0.1)
        self.at(at_ref, False, False)

    def trim(self):
        """Flat trim the drone."""
        self.at(at_ftrim)

    def set_speed(self, speed):
        """Set the drone's speed.

        Valid values are floats from [0..1]
        """
        self.speed = speed

    def set_camera_view(self, downward):
        """
        Set which video camera is used. If 'downward' is true,
        downward camera will be viewed - otherwise frontwards.
        """
        self.config({'video:video_channel':0 if downward else 1})

    def event_boom(self):
        """Boom event"""
        self.at(at_led, 13,2,4)
        self.at(at_anim, 3, 1000)
        
    def event_turnarround(self):
        """Make the drone turnarround."""
        self.at(at_led, 13,2,4)
        self.at(at_anim, 6, 5000)
        
    def event_yawshake(self):
        """Make the drone execute yawshake YEAH !"""
        self.at(at_led, 13,2,4)
        self.at(at_anim, 8, 2000)
        
    def event_yawdance(self):
        """Make the drone execute yawdance YEAH !"""
        self.at(at_led, 13,2,4)
        self.at(at_anim, 9, 5000)
        
    def event_thetamixed(self):
        """Make the drone execute thetamixed !"""
        self.at(at_led, 13,2,4)
        self.at(at_anim, 14, 5000)

    def at(self, cmd, *args, **kwargs):
        """Wrapper for the low level at commands.

        This method takes care that the sequence number is increased after each
        at command and the watchdog timer is started to make sure the drone
        receives a command at least every second.
        """
        self.lock.acquire()
        self.com_watchdog_timer.cancel()
        cmd(self.seq_nr, *args, **kwargs)
        self.seq_nr += 1
        self.com_watchdog_timer = threading.Timer(self.timer_t, self.commwdg)
        self.com_watchdog_timer.start()
        self.lock.release()

    def config(self,cfg):
        self.at(at_config_ids,self.config_ids_string)
        for k,v in cfg.items():
            v = config_options[k](v)
            self.at(at_config,k,v)

    def commwdg(self):
        """Communication watchdog signal.

        This needs to be send regulary to keep the communication w/ the drone
        alive.
        """
        self.at(at_comwdg)

    def halt(self):
        """Shutdown the drone.

        This method does not land or halt the actual drone, but the
        communication with the drone. You should call it at the end of your
        application to close all sockets, pipes, processes and threads related
        with this object.
        """
        self.lock.acquire()
        self.com_watchdog_timer.cancel()
        self.network.halt()
        self.lock.release()

    def set_image(self,image):
        self.image = image

    def set_navdata(self, navdata):
        self.navdata = navdata

#==================================================================================================
# Low level AT Commands
#==================================================================================================

def at_ref(seq, takeoff, emergency=False):
    """
    Basic behaviour of the drone: take-off/landing, emergency stop/reset)

    Parameters:
    seq -- sequence number
    takeoff -- True: Takeoff / False: Land
    emergency -- True: Turn off the engines
    """
    p = 0b10001010101000000000000000000
    if takeoff:
        p += 0b1000000000
    if emergency:
        p += 0b0100000000
    at("REF", seq, [p])

def at_pcmd(seq, progressive, lr, fb, vv, va):
    """
    Makes the drone move (translate/rotate).

    Parameters:
    seq -- sequence number
    progressive -- True: enable progressive commands, False: disable (i.e.
        enable hovering mode)
    lr -- left-right tilt: float [-1..1] negative: left, positive: right
    rb -- front-back tilt: float [-1..1] negative: forwards, positive:
        backwards
    vv -- vertical speed: float [-1..1] negative: go down, positive: rise
    va -- angular speed: float [-1..1] negative: spin left, positive: spin
        right

    The above float values are a percentage of the maximum speed.
    """
    p = 1 if progressive else 0
    at("PCMD", seq, [p, float(lr), float(fb), float(vv), float(va)])

def at_ftrim(seq):
    """
    Tell the drone it's lying horizontally.

    Parameters:
    seq -- sequence number
    """
    at("FTRIM", seq, [])

def at_zap(seq, stream):
    """
    Selects which video stream to send on the video UDP port.

    Parameters:
    seq -- sequence number
    stream -- Integer: video stream to broadcast
    """
    # FIXME: improve parameters to select the modes directly
    at("ZAP", seq, [stream])

def at_config(seq, option, value):
    """Set configuration parameters of the drone."""
    at("CONFIG", seq, [str(option), str(value)])

def at_config_ids(seq, value):
    """Set configuration parameters of the drone."""
    at("CONFIG_IDS", seq, value)

def at_ctrl(seq, num):
    """Ask the parrot to drop its configuration file"""
    at("CTRL", seq, [num, 0])

def at_comwdg(seq):
    """
    Reset communication watchdog.
    """
    # FIXME: no sequence number
    at("COMWDG", seq, [])

def at_aflight(seq, flag):
    """
    Makes the drone fly autonomously.

    Parameters:
    seq -- sequence number
    flag -- Integer: 1: start flight, 0: stop flight
    """
    at("AFLIGHT", seq, [flag])

def at_pwm(seq, m1, m2, m3, m4):
    """
    Sends control values directly to the engines, overriding control loops.

    Parameters:
    seq -- sequence number
    m1 -- front left command
    m2 -- fright right command
    m3 -- back right command
    m4 -- back left command
    """
    # FIXME: what type do mx have?
    raise NotImplementedError()

def at_led(seq, anim, f, d):
    """
    Control the drones LED.

    Parameters:
    seq -- sequence number
    anim -- Integer: animation to play
    f -- ?: frequence in HZ of the animation
    d -- Integer: total duration in seconds of the animation
    """
    at("LED", seq, [anim, float(f), d]) 

def at_anim(seq, anim, d):
    """
    Makes the drone execute a predefined movement (animation).

    Parameters:
    seq -- sequcence number
    anim -- Integer: animation to play
    d -- Integer: total duration in sections of the animation
    """
    at("ANIM", seq, [anim, d])

def at(command, seq, params):
    """
    Parameters:
    command -- the command
    seq -- the sequence number
    params -- a list of elements which can be either int, float or string
    """
    param_str = ''
    for p in params:
        if type(p) == int:
            param_str += ",%d" % p
        elif type(p) == float:
            param_str += ",%d" % f2i(p)
        elif type(p) == str:
            param_str += ',"' + p + '"'
    msg = "AT*%s=%i%s\r" % (command, seq, param_str)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(msg, ("192.168.1.1", ARDRONE_COMMAND_PORT))

def f2i(f):
    """Interpret IEEE-754 floating-point value as signed integer.

    Arguments:
    f -- floating point value
    """
    return struct.unpack('i', struct.pack('f', f))[0]
