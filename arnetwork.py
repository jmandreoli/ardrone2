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

import logging
logger = logging.getLogger(__name__)

import re
import threading
import select
import socket
import subprocess
import struct
import numpy

import libardrone
import paveparser

#==================================================================================================
class network (object):
#==================================================================================================
    """
An instance of this class collects the sensor data from the drone and updates the :attr:`image` and :attr:`navdata` attributes of *drone*\.
    """

    def __init__(self,drone):
        wifi_connect(done.ssid)
        self.running = True
        self.threads = []
        self.threads.extend(ctrlvideo(drone,self))
        self.threads.extend(ctrlnavdata(drone,self))
        for t in self.threads:
            t.daemon = True # just in case it cannot be joined on exit
            t.start()
    def halt(self):
        self.running = False
        for t in self.threads: t.join(1.)
        wifi_disconnect()

def wifi_connect(ssid):
    logger.info('Scanning wifi networks for SSID=%s...',ssid)
    n = 0
    pat = re.compile('SSID \d+ : (.+?)',re.MULTILINE)
    while True:
        x = subprocess.check_output(('netsh','wlan','show','networks'))
        L = pat.findall(x.decode('utf-8'))
        if ssid in L:
            logger.info('SSID %s found. Attempting connection...')
            break
        n += 1
        assert n<=10, 'Too many wifi scan attempts'
        time.sleep(1)
    x = subprocess.check_output(('netsh','wlan','connect','ssid="{}"'.format(ssid)))
    logger.info('Wifi connection successful:\n%s',x.decode('utf-8'))

def wifi_disconnect():
    x = subprocess.check_output(('netsh','wlan','disconnect'))
    logger.info('Wifi disconnection successful:\n%s',x.decode('utf-8'))

#==================================================================================================
def ctrlvideo(drone,main):
#==================================================================================================
    ffmpeg = libardrone.FFMPEG
    if ffmpeg is None: ffmpeg = 'ffmpeg'
    cmd = (ffmpeg,
      '-i','-',
      '-f','image2pipe',
      '-pix_fmt', 'rgb24',
      '-codec:v','rawvideo',
      '-')
    sub = subprocess.Popen(cmd,stdin=subprocess.PIPE,stdout=subprocess.PIPE,bufsize=0)
    tparse = threading.Thread(target=video_parse,args=(sub.stdin,main))
    tproc = threading.Thread(target=video_process,args=(sub.stdout,sub,drone,main))
    return tparse,tproc

def video_parse(pipe,main):
    sock = socket.create_connection(('192.168.1.1',libardrone.ARDRONE_VIDEO_PORT))
    parser = paveparser.PaVEParser(pipe)
    try:
        logger.info('[video_parse] Starting loop')
        while main.running:
            data = sock.recv(65565)
            parser.write(data)
    finally:
        logger.info('[video_parse] Stopping loop')
        pipe.close()

def video_process(pipe,sub,drone,main):
    imgshape = drone.image.shape
    imgsize = imgshape[0]*imgshape[1]*imgshape[2]
    try:
        logger.info('[video_process] Starting loop')
        while main.running:
            try: x = numpy.fromstring(pipe.read(imgsize),count=imgsize,dtype='uint8')
            except IOError: break
            x.shape = imgshape
            drone.set_image(x)
            pipe.flush()
    finally:
        logger.info('[video_process] Stopping loop')
        sub.terminate()

#==================================================================================================
def ctrlnavdata(drone,main):
#==================================================================================================
    t = threading.Thread(target=navdata_process,args=(drone,main))
    return (t,)

def navdata_process(drone,main):
    def _connect():
        nav_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        nav_socket.setblocking(0)
        nav_socket.bind(('', libardrone.ARDRONE_NAVDATA_PORT))
        nav_socket.sendto("\x01\x00\x00\x00", ('192.168.1.1', libardrone.ARDRONE_NAVDATA_PORT))
        control_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        control_socket.connect(('192.168.1.1', libardrone.ARDRONE_CONTROL_PORT))
        control_socket.setblocking(0)
        logger.info('[control] Connection established')
        return nav_socket, control_socket

    def _disconnect(nav_socket, control_socket):
        logger.info('[control] Disconnecting from AR Drone')
        nav_socket.close()
        control_socket.close()

    nav_socket, control_socket = _connect()

    connection_lost = 0
    reconnection_needed = False
    logger.info('[navdata_process] Starting loop')
    while main.running:
        if reconnection_needed:
            _disconnect(nav_socket, control_socket)
            nav_socket, control_socket = _connect()
            reconnection_needed = False
        inputready, outputready, exceptready = select.select([nav_socket, control_socket], [], [], 1.)
        if len(inputready) == 0:
            connection_lost += 1
            reconnection_needed = True
            continue
        for i in inputready:
            if i == nav_socket:
                while True:
                    try: data = nav_socket.recv(500)
                    except IOError: break
                navdata, has_information = navdata_decode(data)
                if has_information: drone.set_navdata(navdata)
            elif i == control_socket:
                while True:
                    try:
                        data = control_socket.recv(65535)
                        if len(data) == 0:
                            logger.warning('[control] Received an empty packet on control socket')
                            reconnection_needed = True
                            break
                        else:
                            logger.warning('[control] %s', data)
                    except IOError:
                        break
    logger.info('[navdata_process] Stopping loop')
    _disconnect(nav_socket, control_socket)

def navdata_decode(packet):
    """Decode a navdata packet."""
    offset = 0
    _ = struct.unpack_from("IIII", packet, offset)
    drone_state = dict()
    drone_state['fly_mask'] = _[1] & 1 # FLY MASK : (0) ardrone is landed, (1) ardrone is flying
    drone_state['video_mask'] = _[1] >> 1 & 1 # VIDEO MASK : (0) video disable, (1) video enable
    drone_state['vision_mask'] = _[1] >> 2 & 1 # VISION MASK : (0) vision disable, (1) vision enable */
    drone_state['control_mask'] = _[1] >> 3 & 1 # CONTROL ALGO (0) euler angles control, (1) angular speed control */
    drone_state['altitude_mask'] = _[1] >> 4 & 1 # ALTITUDE CONTROL ALGO : (0) altitude control inactive (1) altitude control active */
    drone_state['user_feedback_start'] = _[1] >> 5 & 1 # USER feedback : Start button state */
    drone_state['command_mask'] = _[1] >> 6 & 1 # Control command ACK : (0) None, (1) one received */
    drone_state['fw_file_mask'] = _[1] >> 7 & 1 # Firmware file is good (1) */
    drone_state['fw_ver_mask'] = _[1] >> 8 & 1 # Firmware update is newer (1) */
    drone_state['fw_upd_mask'] = _[1] >> 9 & 1 # Firmware update is ongoing (1) */
    drone_state['navdata_demo_mask'] = _[1] >> 10 & 1 # Navdata demo : (0) All navdata, (1) only navdata demo */
    drone_state['navdata_bootstrap'] = _[1] >> 11 & 1 # Navdata bootstrap : (0) options sent in all or demo mode, (1) no navdata options sent */
    drone_state['motors_mask'] = _[1] >> 12 & 1 # Motor status : (0) Ok, (1) Motors problem */
    drone_state['com_lost_mask'] = _[1] >> 13 & 1 # Communication lost : (1) com problem, (0) Com is ok */
    drone_state['vbat_low'] = _[1] >> 15 & 1 # VBat low : (1) too low, (0) Ok */
    drone_state['user_el'] = _[1] >> 16 & 1 # User Emergency Landing : (1) User EL is ON, (0) User EL is OFF*/
    drone_state['timer_elapsed'] = _[1] >> 17 & 1 # Timer elapsed : (1) elapsed, (0) not elapsed */
    drone_state['angles_out_of_range'] = _[1] >> 19 & 1 # Angles : (0) Ok, (1) out of range */
    drone_state['ultrasound_mask'] = _[1] >> 21 & 1 # Ultrasonic sensor : (0) Ok, (1) deaf */
    drone_state['cutout_mask'] = _[1] >> 22 & 1 # Cutout system detection : (0) Not detected, (1) detected */
    drone_state['pic_version_mask'] = _[1] >> 23 & 1 # PIC Version number OK : (0) a bad version number, (1) version number is OK */
    drone_state['atcodec_thread_on'] = _[1] >> 24 & 1 # ATCodec thread ON : (0) thread OFF (1) thread ON */
    drone_state['navdata_thread_on'] = _[1] >> 25 & 1 # Navdata thread ON : (0) thread OFF (1) thread ON */
    drone_state['video_thread_on'] = _[1] >> 26 & 1 # Video thread ON : (0) thread OFF (1) thread ON */
    drone_state['acq_thread_on'] = _[1] >> 27 & 1 # Acquisition thread ON : (0) thread OFF (1) thread ON */
    drone_state['ctrl_watchdog_mask'] = _[1] >> 28 & 1 # CTRL watchdog : (1) delay in control execution (> 5ms), (0) control is well scheduled */
    drone_state['adc_watchdog_mask'] = _[1] >> 29 & 1 # ADC Watchdog : (1) delay in uart2 dsr (> 5ms), (0) uart2 is good */
    drone_state['com_watchdog_mask'] = _[1] >> 30 & 1 # Communication Watchdog : (1) com problem, (0) Com is ok */
    drone_state['emergency_mask'] = _[1] >> 31 & 1 # Emergency landing : (0) no emergency, (1) emergency */
    data = dict()
    data['drone_state'] = drone_state
    data['header'] = _[0]
    data['seq_nr'] = _[2]
    data['vision_flag'] = _[3]
    offset += struct.calcsize("IIII")
    has_flying_information = False
    while True:
        try:
            id_nr, size = struct.unpack_from("HH", packet, offset)
            offset += struct.calcsize("HH")
        except struct.error:
            break
        values = []
        for i in range(size - struct.calcsize("HH")):
            values.append(struct.unpack_from("c", packet, offset)[0])
            offset += struct.calcsize("c")
        # navdata_tag_t in navdata-common.h
        if id_nr == 0:
            has_flying_information = True
            values = struct.unpack_from("IIfffifffI", "".join(values))
            values = dict(zip(['ctrl_state', 'battery', 'theta', 'phi', 'psi', 'altitude', 'vx', 'vy', 'vz', 'num_frames'], values))
            # convert the millidegrees into degrees and round to int, as they
            # are not so precise anyways
            for i in 'theta', 'phi', 'psi':
                values[i] = int(values[i] / 1000)
        data[id_nr] = values
    return data, has_flying_information
