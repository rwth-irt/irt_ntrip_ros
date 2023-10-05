import dataclasses
import sys
import time
import math
from enum import Enum
from dataclasses import dataclass
import json

import rclpy
from rclpy.node import Node
from novatel_oem7_msgs.msg import BESTPOS
from novatel_oem7_msgs.srv import Oem7AbasciiCmd
from ntrip_ros.srv import UpdateReferenceStation
from ntrip_ros.msg import NtripCaster as NtripCasterMsg

import geopy.distance
from paho.mqtt import client as mqtt_client
from geopy.geocoders import Nominatim

import socket

@dataclass
class Position:
    latitude: float
    longitude: float


@dataclass
class NtripData:
    uri: str
    port: int
    username: str
    password: str
    mountpoint: str



@dataclass
class NtripCaster:
    ntrip: NtripData
    mqtt: NtripData
    novatel: NtripData
    position: Position
    state_name: str


@dataclass
class MqttConfig:
    broker: str
    port: int
    topic: str
    id: str
    username: str
    password: str


@dataclass
class RequestTimes:
    ntripconfig: float
    saveconfig: float


class NOVATEL(Enum):
    NTRIPCONFIG = 0
    LOG = 1
    SAVECONFIG = 2


NtripCasterDb = {
    # GERMANY
    
    
    # NETHERLANDS
    # http://monitor.use-snip.com/?hostUrl=gnss1.tudelft.nl&port=2101
    # and http://gnss1.tudelft.nl/dpga/index.html)
    'AMEL00': NtripCaster(NtripData('gnss1.tudelft.nl', 2101, '', '', 'AMEL00NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'AMEL00NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'AMEL00NLD0'),
                          Position(53.26235, 5.76489),
                          'None'),
    #'APEL00': NtripCaster(NtripData('gnss1.tudelft.nl', 2101, '', '', 'APEL00NLD0'),
   #                       NtripData('gnss1.tudelft.nl', 2101, '', '', 'APEL00NLD0'),
    #                      NtripData('gnss1.tudelft.nl', 2101, '', '', 'APEL00NLD0'),
    #                      Position(52.0256, 5.96229),
    #                      'None'),
    'BORS00': NtripCaster(NtripData('gnss1.tudelft.nl', 2101, '', '', 'BORS00NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'BORS00NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'BORS00NLD0'),
                          Position(51.7, 3.06),
                          'None'),
    #'CBW100': NtripCaster(NtripData('gnss1.tudelft.nl', 2101, '', '', 'CBW100NLD0'),
    #                      NtripData('gnss1.tudelft.nl', 2101, '', '', 'CBW100NLD0'),
    #                      NtripData('gnss1.tudelft.nl', 2101, '', '', 'CBW100NLD0'),
    #                      Position(51.78282, 4.92618),
    #                      'None'),
    'DELF00': NtripCaster(NtripData('gnss1.tudelft.nl', 2101, '', '', 'DELF00NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'DELF00NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'DELF00NLD0'),
                          Position(51.79943, 4.38758),
                          'None'),
    'DLF100': NtripCaster(NtripData('gnss1.tudelft.nl', 2101, '', '', 'DLF100NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'DLF100NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'DLF100NLD0'),
                          Position(51.79932, 4.38746),
                          'None'),
    'DLF500': NtripCaster(NtripData('gnss1.tudelft.nl', 2101, '', '', 'DLF500NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'DLF500NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'DLF500NLD0'),
                          Position(51.99, 4.39),
                          'None'),
    'DLF700': NtripCaster(NtripData('gnss1.tudelft.nl', 2101, '', '', 'DLF700NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'DLF700NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'DLF700NLD0'),
                          Position(51.99, 4.39),
                          'None'),
    'DLF800': NtripCaster(NtripData('gnss1.tudelft.nl', 2101, '', '', 'DLF800NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'DLF800NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'DLF800NLD0'),
                          Position(51.99, 4.39),
                          'None'),
    'DLF900': NtripCaster(NtripData('gnss1.tudelft.nl', 2101, '', '', 'DLF900NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'DLF900NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'DLF900NLD0'),
                          Position(51.99, 4.39),
                          'None'),
    'DLFV00': NtripCaster(NtripData('gnss1.tudelft.nl', 2101, '', '', 'DLFV00NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'DLFV00NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'DLFV00NLD0'),
                          Position(51.99, 4.39),
                          'None'),
    'DLFV02': NtripCaster(NtripData('gnss1.tudelft.nl', 2101, '', '', 'DLFV02NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'DLFV02NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'DLFV02NLD0'),
                          Position(51.99, 4.39),
                          'None'),
    'DLFV03': NtripCaster(NtripData('gnss1.tudelft.nl', 2101, '', '', 'DLFV03NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'DLFV03NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'DLFV03NLD0'),
                          Position(51.99, 4.39),
                          'None'),
    'EIJS00': NtripCaster(NtripData('gnss1.tudelft.nl', 2101, '', '', 'EIJS00NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'EIJS00NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'EIJS00NLD0'),
                          Position(50.56975, 5.68361),
                          'None'),
    'EPL100': NtripCaster(NtripData('gnss1.tudelft.nl', 2101, '', '', 'EPL100NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'EPL100NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'EPL100NLD0'),
                          Position(51.88, 3.27),
                          'None'),
    'IJMU00': NtripCaster(NtripData('gnss1.tudelft.nl', 2101, '', '', 'IJMU00NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'IJMU00NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'IJMU00NLD0'),
                          Position(52.27607, 4.55606),
                          'None'),
    #'KOS100': NtripCaster(NtripData('gnss1.tudelft.nl', 2101, '', '', 'KOS100NLD0'),
    #                      NtripData('gnss1.tudelft.nl', 2101, '', '', 'KOS100NLD0'),
    #                      NtripData('gnss1.tudelft.nl', 2101, '', '', 'KOS100NLD0'),
    #                      Position(51.98697, 5.81823),
    #                      'None'),
    'SCHI00': NtripCaster(NtripData('gnss1.tudelft.nl', 2101, '', '', 'SCHI00NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'SCHI00NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'SCHI00NLD0'),
                          Position(53.29327, 6.16227),
                          'None'),
    'TERS00': NtripCaster(NtripData('gnss1.tudelft.nl', 2101, '', '', 'TERS00NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'TERS00NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'TERS00NLD0'),
                          Position(53.17846, 5.21939),
                          'None'),
    'TXE200': NtripCaster(NtripData('gnss1.tudelft.nl', 2101, '', '', 'TXE200NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'TXE200NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'TXE200NLD0'),
                          Position(52.85619, 4.85117),
                          'None'),
    'VLIE00': NtripCaster(NtripData('gnss1.tudelft.nl', 2101, '', '', 'VLIE00NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'VLIE00NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'VLIE00NLD0'),
                          Position(53.11182, 5.09186),
                          'None'),
    'VLIS00': NtripCaster(NtripData('gnss1.tudelft.nl', 2101, '', '', 'VLIS00NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'VLIS00NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'VLIS00NLD0'),
                          Position(51.25535, 3.59732),
                          'None'),
    'WSRA00': NtripCaster(NtripData('gnss1.tudelft.nl', 2101, '', '', 'WSRA00NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'WSRA00NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'WSRA00NLD0'),
                          Position(52.7295, 6.6045),
                          'None'),
    'WSRT00': NtripCaster(NtripData('gnss1.tudelft.nl', 2101, '', '', 'WSRT00NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'WSRT00NLD0'),
                          NtripData('gnss1.tudelft.nl', 2101, '', '', 'WSRT00NLD0'),
                          Position(52.7295, 6.6045),
                          'None'),
    #'ZEGV00': NtripCaster(NtripData('gnss1.tudelft.nl', 2101, '', '', 'ZEGV00NLD0'),
    #                      NtripData('gnss1.tudelft.nl', 2101, '', '', 'ZEGV00NLD0'),
    #                      NtripData('gnss1.tudelft.nl', 2101, '', '', 'ZEGV00NLD0'),
    #                      Position(51.95135, 4.83919),
    #                      'None')
}


# Parameters:
# General
#   max_pos_std
#   wait_time
#   polling_frequency
#   update_ntrip
#   update_novatel
#   update_mqtt
# MQTT
#   broker
#   port
#   topic
#   id
#   username
#   password

class ReferenceStationTracker(Node):

    def __init__(self):
        super().__init__('ref_station_tracker_node')
        self.get_logger().info("Node Parameters:")
        self.declare_parameter('General.update_ntrip', False)
        self.get_logger().info(f"[General] update_ntrip: {self.get_parameter('General.update_ntrip').value}")
        self.declare_parameter('General.update_novatel', False)
        self.get_logger().info(f"[General] update_novatel: {self.get_parameter('General.update_novatel').value}")
        self.declare_parameter('General.update_mqtt', False)
        self.get_logger().info(f"[General] update_mqtt: {self.get_parameter('General.update_mqtt').value}")
        self.declare_parameter('General.max_pos_std', 0.0)
        self.get_logger().info(f"[General] max_pos_std: {self.get_parameter('General.max_pos_std').value}")
        self.declare_parameter('General.wait_time', 0.0)
        self.get_logger().info(f"[General] wait_time: {self.get_parameter('General.wait_time').value}")
        self.declare_parameter('General.polling_frequency', 0.0)
        self.get_logger().info(f"[General] polling_frequency: {self.get_parameter('General.polling_frequency').value}")
        self.declare_parameter('MQTT.broker', '')
        self.get_logger().info(f"[MQTT] broker: {self.get_parameter('MQTT.broker').value}")
        self.declare_parameter('MQTT.port', 0)
        self.get_logger().info(f"[MQTT] port: {self.get_parameter('MQTT.port').value}")
        self.declare_parameter('MQTT.topic', '')
        self.get_logger().info(f"[MQTT] topic: {self.get_parameter('MQTT.topic').value}")
        self.declare_parameter('MQTT.id', '')
        self.get_logger().info(f"[MQTT] id: {self.get_parameter('MQTT.id').value}")
        self.declare_parameter('MQTT.username', '')
        self.get_logger().info(f"[MQTT] username: {self.get_parameter('MQTT.username').value}")
        self.declare_parameter('MQTT.password', '')
        self.get_logger().info(f"[MQTT] password: {self.get_parameter('MQTT.password').value}")

        self.geolocator = Nominatim(user_agent='ref_station_tracker_node')

        self.last_request_times = RequestTimes(sys.float_info.min, sys.float_info.min)
        self.wait_time = self.get_parameter('General.wait_time').value
        self.max_pos_std = self.get_parameter('General.max_pos_std').value

        self.polling_frequency = self.get_parameter('General.polling_frequency').value
        self.update_ntrip = self.get_parameter('General.update_ntrip').value
        self.update_novatel = self.get_parameter('General.update_novatel').value
        self.update_mqtt = self.get_parameter('General.update_mqtt').value
        self.client_futures = []
        self.ntrip_publisher = self.create_publisher(
            NtripCasterMsg,
            '/irt/ntrip_caster',
            1
        )
        self.gps_fix_subscription = self.create_subscription(
            BESTPOS,
            '/novatel/oem7/bestpos',
            self.pos_cb,
            1
        )

        self.ntrip_client = self.create_client(
            UpdateReferenceStation,
            '/irt/update_reference_station'
        )
        if self.update_ntrip:
            while not self.ntrip_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service update_reference_station not available, waiting again...')

        self.novatel_client = self.create_client(
            Oem7AbasciiCmd,
            '/novatel/oem7/Oem7Cmd'
        )
        if self.update_novatel:
            while not self.novatel_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service Oem7Cmd not available, waiting again...')

        self.mqtt_config = MqttConfig(
            self.get_parameter('MQTT.broker').value,
            self.get_parameter('MQTT.port').value,
            self.get_parameter('MQTT.topic').value,
            self.get_parameter('MQTT.id').value,
            self.get_parameter('MQTT.username').value,
            self.get_parameter('MQTT.password').value
        )
        if self.update_mqtt:
            self.mqtt_client = self.mqtt_connect()
        else:
            self.mqtt_client = None

        self.ntrip_settings = None
        self.novatel_settings = None
        self.mqtt_settings = None
        self.ntrip_position = None

    def mqtt_connect(self):
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                print("Connected to MQTT Broker!")
            else:
                print("Failed to connect, return code %d\n", rc)

        # Set Connecting Client ID
        client = mqtt_client.Client(self.mqtt_config.id)
        client.username_pw_set(self.mqtt_config.username, self.mqtt_config.password)
        client.on_connect = on_connect
        client.connect(self.mqtt_config.broker, self.mqtt_config.port)
        return client

    def send_requests(self):
        if self.update_novatel:
            self.novatel_request()
        if self.update_ntrip:
            self.ntrip_request()
        if self.update_mqtt:
            self.mqtt_request()

    def ntrip_request(self):
        self.get_logger().info('Sending UpdateReferenceStation Request to NTRIP_ROS')
        request = UpdateReferenceStation.Request()
        request.uri = self.ntrip_settings.uri
        request.port = self.ntrip_settings.port
        request.username = self.ntrip_settings.username
        request.password = self.ntrip_settings.password
        request.mountpoint = self.ntrip_settings.mountpoint
        self.client_futures.append(self.ntrip_client.call_async(request))

    def novatel_request(self):

        # Type the following console commands to connect Novatel receiver as client to ntrip caster:
        #
        # NTRIPCONFIG NCOM1 CLIENT V1 <NTRIP_CAST_IP>:<PORT> <MOUNTPOINT> <USERNAME> <PASSWORD>
        # NTRIPCONFIG NCOM1 CLIENT V1 195.227.70.119:2101 VRS_3_4G_NW nw-710989 railir
        #
        # sleep(2s)
        #
        # INTERFACEMODE NCOM1 RTCMV3 NOVATEL OFF
        # LOG NCOM1 GPGGA ONTIME 5
        # SAVECONFIG
        #
        # Before this, you can check the availabe mountpoints with:
        #    NTRIPSOURCETABLE <NTRIP_CAST_IP>:<PORT>
        # LOG SOURCETABLE
        #
        # Maybe you have to enable receivers ethernet functionality:
        # ETHCONFIG ETHA AUTO AUTO AUTO AUTO
        # Optional: define static IP: IPCONFIG ETHA STATIC <STATIC_IP> <SUBNET_MASK> <GATEWAY_IP>
        # SAVEETHERNETDATA ETHA
        self.get_logger().info('Sending NTRIPCONFIG Command to NOVATEL')
        request = Oem7AbasciiCmd.Request()
        request.cmd = f'NTRIPCONFIG NCOM1 CLIENT V1 {self.novatel_settings.uri}:{self.novatel_settings.port} {self.novatel_settings.mountpoint} {self.novatel_settings.username} {self.novatel_settings.password}'
        self.client_futures.append(self.novatel_client.call_async(request))
        self.last_request_times.ntripconfig = time.time()

    def mqtt_request(self):
        self.get_logger().info(f'Sending MQTT msg to {self.mqtt_config.topic}')
        settings_dict = self.mqtt_settings.__dict__
        ip = settings_dict['uri']
        try:
            ip = socket.gethostbyname(settings_dict['uri'])
            self.get_logger().info('Resolved IP %s from host %s' % (ip, settings_dict['uri']))
        except Exception as e:
            self.get_logger().warn('Cannot resolve ip from host name %s' % settings_dict['uri'])
            return
        settings_dict['uri'] = ip
        msg = json.dumps(settings_dict)
        status = -1
        while status != 0:
            result = self.mqtt_client.publish(self.mqtt_config.topic, msg)
            time.sleep(1)
            # result: [0, 1]
            status = result[0]
        self.last_request_times.mqtt = time.time()

    def pos_cb(self, msg):
        try:
            current_location = self.geolocator.reverse(f'{msg.lat}, {msg.lon}')
        except:
            self.get_logger().warn("Cannot connect to geolocator server!")
            return
        full_requests_sent = False
        self.get_logger().info(f"Current location: {current_location.address}")
        current_pos_std = math.sqrt(msg.lat_stdev ** 2 + msg.lon_stdev ** 2)
        self.get_logger().info(f"Current 2d-SD: {current_pos_std}")

        # Find better caster, if current caster results in worse positioning than user-defined threshold
        if 'Deutschland' in current_location.address or current_pos_std > self.max_pos_std or self.ntrip_settings is None or (time.time() - self.last_request_times.saveconfig >= self.wait_time):
            key_spec = ""
            if 'Deutschland' in current_location.address:
                key_spec = "SAPOS"

            location_raw = current_location.raw
            current_caster = self.get_caster(msg.lat, msg.lon, key_spec, location_raw)

            self.get_logger().info(f"Got new caster: {current_caster}")
            if(not current_caster):
                return
            current_ntrip_settings = current_caster.ntrip
            current_novatel_settings = current_caster.novatel
            current_mqtt_settings = current_caster.mqtt
            current_position = current_caster.position
            if current_ntrip_settings != self.ntrip_settings:
                self.ntrip_settings = current_ntrip_settings
                self.novatel_settings = current_novatel_settings
                self.mqtt_settings = current_mqtt_settings
                self.ntrip_position = current_position
                self.send_requests()
                if not full_requests_sent:
                    full_requests_sent = True

        if not full_requests_sent:
            if self.update_mqtt:
                self.mqtt_request()
            if self.update_ntrip:
                self.ntrip_request()

        caster_msg = NtripCasterMsg()
        caster_msg.header.stamp = self.get_clock().now().to_msg()
        caster_msg.lat = self.ntrip_position.latitude
        caster_msg.lon = self.ntrip_position.longitude
        caster_msg.uri = self.ntrip_settings.uri
        caster_msg.port = self.ntrip_settings.port
        caster_msg.mountpoint = self.ntrip_settings.mountpoint
        self.ntrip_publisher.publish(caster_msg)

    def get_caster(self, latitude, longitude, key_spec = "", location_raw=None):
        # get the closest ntrip caster to current latitude and longitude
        smallest_distance = sys.float_info.max
        closest_caster = None

        try:

            for key, value in NtripCasterDb.items():
                if key_spec and key_spec not in key:
                    continue
                #print(location_raw['address']['state'], value.state_name)
                if location_raw and value.state_name == location_raw['address']['state']:
                  #print("#########################################machted key: ", key)
                  return NtripCasterDb[key]


                rover_caster_distance = geopy.distance.distance((latitude, longitude),
                                                                (value.position.latitude, value.position.longitude))
                if rover_caster_distance < smallest_distance:
                    smallest_distance = rover_caster_distance
                    closest_caster = key
            return NtripCasterDb[closest_caster]
        except:
            self.get_logger().warn('Sending UpdateReferenceStation Request to NTRIP_ROS')
            return None

    def spin(self):
        current_novatel_cmd = NOVATEL.NTRIPCONFIG

        while rclpy.ok():
            rclpy.spin_once(self)
            incomplete_futures = []

            for future in self.client_futures:
                if future.done():
                    response = future.result()

                    self.get_logger().info('*************RESPOSE: %s' % str(response))

                    # NTRIP
                    if hasattr(response, 'success'):
                        if response.success:
                            self.get_logger().info('Ntrip Caster updated')
                        else:
                            self.get_logger().info('Ntrip Caster update failed')

                    # NOVATEL
                    if hasattr(response, 'rsp'):
                        self.get_logger().info(f'Novatel response {response.rsp}')
                        # Response from NTRIPCONFIG cmd -> Send LOG cmd
                        if current_novatel_cmd == NOVATEL.NTRIPCONFIG:
                            # Wait for wait_time seconds after issuing the NTRIPCONFIG cmd
                            if time.time() - self.last_request_times.ntripconfig < 1.0:
                                # We still need this future to trigger the next request after 1s has elapsed
                                incomplete_futures.append(future)
                                continue
                            self.get_logger().info('Sending LOG Command to NOVATEL')
                            request = Oem7AbasciiCmd.Request()
                            request.cmd = 'LOG NCOM1 GPGGA ONTIME 5'
                            self.client_futures.append(self.novatel_client.call_async(request))
                            current_novatel_cmd = NOVATEL.LOG
                            continue
                        # Response from LOG cmd -> Send SAVECONFIG cmd
                        if current_novatel_cmd == NOVATEL.LOG:
                            self.get_logger().info('Sending SAVECONFIG Command to NOVATEL')    
                            request = Oem7AbasciiCmd.Request()
                            request.cmd = 'SAVECONFIG'
                            self.client_futures.append(self.novatel_client.call_async(request))
                            self.last_request_times.saveconfig = time.time()
                            current_novatel_cmd = NOVATEL.SAVECONFIG
                            continue
                        # Response from SAVECONFIG cmd
                        if current_novatel_cmd == NOVATEL.SAVECONFIG:
                            self.get_logger().info(f"Going to sleep for {self.wait_time} seconds before trying to update the ntrip caster again...")
                            current_novatel_cmd = NOVATEL.NTRIPCONFIG
                            continue
                else:
                    incomplete_futures.append(future)

            self.client_futures = incomplete_futures
            # Manually enforce the polling frequency
            time.sleep(1.0 / self.polling_frequency)
