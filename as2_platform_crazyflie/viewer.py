#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""AI-deck demo viewer."""

# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2021 Bitcraze AB
#
#  AI-deck demo
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License along with
#  this program; if not, write to the Free Software Foundation, Inc., 51
#  Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.


#####################################################################
# Originally taken from the Bitcraze AIDeck GAP8 examples:
#  https://github.com/bitcraze/aideck-gap8-examples
#
# Modified by Miguel Granero & Miguel Fernandez. 2022
#   - ROS Package created
#   - Modification to functionality to adapt it to a better ros usage
#   - Image publisher
#   - Color correction
####################################################################

import socket
import struct
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
# import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def balance_color(img):
    """Balance the color of an image."""
    mean = [0.0, 0.0, 0.0]
    factor = [1.0, 1.0, 1.0]
    img_out = img.copy()
    for c in range(3):
        mean[c] = np.mean(img[:, :, c].flatten())
        factor[c] = 255.0/mean[c]
        img_out[:, :, c] = np.clip(img_out[:, :, c]*factor[c], 0, 255)

    print(f'Factor: {factor}')
    return img_out, factor


class AIdeckPublisher(Node):
    """Class for the publisher."""

    def __init__(self):
        """Construct the publisher."""
        super().__init__('aideck_stream_publisher')

        # Args for setting IP/port of AI-deck. Default settings are for when
        # AI-deck is in AP mode.
        self.declare_parameter('ip', '192.168.4.1')
        deck_ip = self.get_parameter('ip').value
        self.declare_parameter('port', 5000)
        deck_port = self.get_parameter('port').value
        self.declare_parameter('save_flag', False)
        self.declare_parameter('show_flag', False)

        self.factors = [1.8648577393897736, 1.2606252586922309, 1.4528872589128194]

        self.publisher_ = self.create_publisher(Image, 'aideck/image', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        print(f'Connecting to socket on {deck_ip}:{deck_port}...')
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((deck_ip, deck_port))
        print('Socket connected')

        self.br = CvBridge()

        self.start = time.time()
        self.count = 0

    def __del___(self):
        """Destructor."""
        self.close()

    def close(self):
        """Close the socket."""
        print('Closing socket')
        self.client_socket.close()

    def colorCorrectBayer(self, img_, factors=[1, 1, 1]):
        """
        Color correction for the RGB Camera.

        It has a sensor with a Bayer pattern, which has more green
        cells than blue and red, so if the image is not treated,
        it will have a green-ish look.
        """
        # TODO(fixme): Apply an actual color correction without luminosity loss. -> histogram level
        # This is just an approximation
        img = img_.copy()
        for i in range(3):
            img[:, :, i] = np.clip(img[:, :, i]*factors[i], 0, 255)
        return img

    def rx_bytes(self, size, client_socket):
        """Read N bytes from the socket."""
        data = bytearray()
        while len(data) < size:
            data.extend(client_socket.recv(size-len(data)))
        return data

    def getImage(self, client_socket):
        """Receive an image from the socket."""
        # imgdata = None
        # data_buffer = bytearray()
        # First get the info
        packet_info_raw = self.rx_bytes(4, client_socket)
        # print(packetInfoRaw)
        length, _ = struct.unpack('<HBB', packet_info_raw)
        # print("Length is {}".format(length))
        # print("Route is 0x{:02X}->0x{:02X}".format(routing & 0xF, routing >> 4))
        # print("Function is 0x{:02X}".format(function))

        img_header = self.rx_bytes(length - 2, client_socket)
        # print(imgHeader)
        # print("Length of data is {}".format(len(imgHeader)))
        magic, _width, _height, _depth, _format, size = struct.unpack('<BHHBBI', img_header)

        imgs = None

        if magic == 0xBC:
            # print("Magic is good")
            # print("Resolution is {}x{} with depth of {} byte(s)".format(width, height, depth))
            # print("Image format is {}".format(format))
            # print("Image size is {} bytes".format(size))

            # Now we start rx the image, this will be split up in packages of some size
            img_stream = bytearray()

            while len(img_stream) < size:
                packet_info_raw = self.rx_bytes(4, client_socket)
                length, _dst, _src = struct.unpack('<HBB', packet_info_raw)
                # print("Chunk size is {} ({:02X}->{:02X})".format(length, src, dst))
                chunk = self.rx_bytes(length - 2, client_socket)
                img_stream.extend(chunk)
            self.count = self.count + 1
            # mean_time_per_image = (time.time()-self.start) / self.count

            # TODO(fixme): Change to debug and rclpy
            # print("{}".format(meanTimePerImage))
            # print("{}".format(1/meanTimePerImage))

            if _format == 0:
                bayer_img = np.frombuffer(img_stream, dtype=np.uint8)
                bayer_img.shape = (244, 324)
                color_img = cv2.cvtColor(bayer_img, cv2.COLOR_BayerBG2BGR)

                k = cv2.waitKey(1)
                if k == ord('b'):
                    _, self.factors = balance_color(color_img)
                if self.get_parameter('save_flag').value:
                    cv2.imwrite(f'stream_out/raw/img_{self.count:06d}.png', bayer_img)
                    cv2.imwrite(f'stream_out/debayer/img_{self.count:06d}.png', color_img)
                if self.get_parameter('show_flag').value:
                    cv2.imshow('Raw', bayer_img)
                    cv2.imshow('Color', self.colorCorrectBayer(color_img, self.factors))
                    cv2.waitKey(1)
                imgs = [bayer_img, color_img]
            else:
                if self.get_parameter('save_flag').value:
                    with open('img.jpeg', 'wb') as f:
                        f.write(img_stream)
                nparr = np.frombuffer(img_stream, np.uint8)
                decoded = cv2.imdecode(nparr, cv2.IMREAD_UNCHANGED)
                if self.get_parameter('show_flag').value:
                    cv2.imshow('JPEG', decoded)
                    cv2.waitKey(1)
                imgs = [decoded]

        return _format, imgs

    def timer_callback(self):
        """Call the getImage function and publishes the image."""
        msg = Image()
        msg.header.frame_id = 'aideck'
        msg.header.stamp = self.get_clock().now().to_msg()
        _format, imgs = self.getImage(self.client_socket)

        if imgs is not None and _format == 0:
            # self.get_logger().info('Publishing: "%s"' % self.i)
            img = imgs[-1]
            msg = self.br.cv2_to_imgmsg(self.colorCorrectBayer(img, self.factors), encoding='bgr8')
            self.publisher_.publish(msg)
            self.i += 1
