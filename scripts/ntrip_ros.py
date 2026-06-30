#!/usr/bin/env python3

import os
import sys
import json
from typing import List

import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String

from ntrip_ros_base import NTRIPRosBase, _RTCM_MSGS_NAME
from ntrip_client.ntrip_client import NTRIPClient
from ntrip_client.nmea_parser import NMEA_DEFAULT_MAX_LENGTH, NMEA_DEFAULT_MIN_LENGTH

class NTRIPRos(NTRIPRosBase):
  def __init__(self):
    # Init the node and declare params
    super().__init__('ntrip_client')
    self.declare_parameters(
      namespace='',
      parameters=[
        ('host', '127.0.0.1'),
        ('port', 2101),
        ('mountpoint', 'mount'),
        ('ntrip_version', 'None'),
        ('user_agent', NTRIPClient.DEFAULT_USER_AGENT),
        ('authenticate', False),
        ('username', ''),
        ('password', ''),
        ('ssl', False),
        ('cert', 'None'),
        ('key', 'None'),
        ('ca_cert', 'None'),
        ('ntrip_server_hz', 1), # set send_nmea() to 1hz
        ('send_nmea', True),
        ('reconnect_attempt_wait_max_seconds', NTRIPClient.DEFAULT_RECONNECT_ATTEMPT_WAIT_MAX_SECONDS),
        ('rtcm_timeout_seconds', NTRIPClient.DEFAULT_RTCM_TIMEOUT_SECONDS),
        ('recovery_period_s', 5.0),
      ]
    )

    # Initialize all internal variables in constructor
    # Will be loaded in 'load_parameters' function.
    self.host = None
    self.port = None
    self.mountpoint = None
    self.ntrip_version = None
    self.user_agent = None
    self.authenticate = None
    self.username = None
    self.password = None
    self.ssl = None
    self.cert = None
    self.key = None
    self.ca_cert = None
    self.rtcm_timeout_seconds = None

    self.load_parameters()
    self.add_on_set_parameters_callback(self.on_set_parameters_callback)

    # Set the rate at which RTCM requests and NMEA messages are sent
    self.rtcm_request_rate = 1.0 / self.get_parameter('ntrip_server_hz').value

    # Whether to forward NMEA from the 'nmea' topic up to the caster. Only needed for
    # virtual/relayed (VRS) mountpoints; disable for plain base stations to avoid the
    # idle subscriber cost and uploading position to the caster. If the caster is rtk2go,
    # exit with an error if send_nmea is true, since rtk2go will ban the IP and possibly
    # this client if it receives persistent NMEA during error conditions.
    self._send_nmea = self.get_parameter('send_nmea').value
    if self._send_nmea and ((self.host == "rtk2go.com") or (self.host == "3.143.243.81")):
      self.get_logger().error('rtk2go blocks clients that send NMEA excessively, but send_nmea is true and host is rtk2go; exiting to avoid IP ban. Set send_nmea to false to fix this.')
      sys.exit(1)

    # Initialize variables to store the most recent NMEA message
    self._latest_nmea = None

    # Setup a server frequency confirmation publisher
    self._rate_confirm_pub = self.create_publisher(String, 'ntrip_server_hz', 10)

    # Initialize the client
    self._client = self.init_ntrip_client()
    self.run()

    # Periodically re-initialize the client if its configuration changed via a parameter update.
    # Deliberately does NOT retry on disconnect here -- that's the NTRIPClient's own backoff
    # logic (request_reconnect/try_reconnect), so we don't fight its rate limiting and risk
    # an rtk2go ban from reconnecting too aggressively.
    self._ntrip_config_updated = False
    self.recovery_timer = self.create_timer(
      self.get_parameter('recovery_period_s').value,
      self.recovery_callback)

  def load_parameters(self):
    """Load ROS parameters."""
    # Read some mandatory config
    self.host = self.get_parameter('host').value
    self.port = self.get_parameter('port').value
    self.mountpoint = self.get_parameter('mountpoint').value

    # Optionally get the ntrip version from the launch file
    self.ntrip_version = self.get_parameter('ntrip_version').value
    if self.ntrip_version == 'None':
      self.ntrip_version = None

    # User-Agent presented to the caster. rtk2go blocks the stock signature, so this
    # is configurable; an empty/'None' value falls back to the client default.
    self.user_agent = self.get_parameter('user_agent').value
    if not self.user_agent or self.user_agent == 'None':
      self.user_agent = NTRIPClient.DEFAULT_USER_AGENT

    # If we were asked to authenticate, read the username and password
    self.username = None
    self.password = None
    self.authenticate = self.get_parameter('authenticate').value
    if self.authenticate:
      self.username = self.get_parameter('username').value
      self.password = self.get_parameter('password').value
      if not self.username or not self.password:
        raise ValueError(f'Invalid username/password: {self.username}/{self.password}')

    self.ssl = self.get_parameter('ssl').value
    self.cert = self.get_parameter('cert').value
    if self.cert == 'None':
      self.cert = None
    self.key = self.get_parameter('key').value
    if self.key == 'None':
      self.key = None
    self.ca_cert = self.get_parameter('ca_cert').value
    if self.ca_cert == 'None':
      self.ca_cert = None

    self.rtcm_timeout_seconds = self.get_parameter('rtcm_timeout_seconds').value

  def on_set_parameters_callback(self, parameters: List[Parameter]) -> SetParametersResult:
    """Callback on parameter update
    This allows to validate every parameter update and automatically
    reload the necessary component when an update is triggered
    """
    def is_string(name: str, param: Parameter) -> bool:
      """Helper function to reduce verbosity"""
      return param.name == name and param.type_ == Parameter.Type.STRING

    def is_bool(name: str, param: Parameter) -> bool:
      """Helper function to reduce verbosity"""
      return param.name == name and param.type_ == Parameter.Type.BOOL

    def is_integer(name: str, param: Parameter) -> bool:
      """Helper function to reduce verbosity"""
      return param.name == name and param.type_ == Parameter.Type.INTEGER

    self.get_logger().warn(f'{parameters}')

    for parameter in parameters:
      if is_string('host', parameter):
        self.host = parameter.value
      elif is_integer('port', parameter):
        self.port = parameter.value
      elif is_string('mountpoint', parameter):
        self.mountpoint = parameter.value
      elif is_string('username', parameter):
        self.username = parameter.value
      elif is_string('password', parameter):
        self.password = parameter.value
      elif is_bool('ssl', parameter):
        self.ssl = parameter.value
      elif is_string('cert', parameter):
        self.cert = parameter.value
        self.cert = self.cert if self.cert != 'None' else None
      elif is_string('key', parameter):
        self.key = parameter.value
        self.key = self.key if self.key != 'None' else None
      elif is_string('ca_cert', parameter):
        self.ca_cert = parameter.value
        self.ca_cert = self.ca_cert if self.ca_cert != 'None' else None
      else:
          # parameters unrelated to the NTRIP configuration
          # such as a timer period
          pass

    self._ntrip_config_updated = True
    return SetParametersResult(successful=True)

  def init_ntrip_client(self):
    """Initialize a NTRIP client using class internal variable."""
    client = NTRIPClient(
      host=self.host,
      port=self.port,
      mountpoint=self.mountpoint,
      ntrip_version=self.ntrip_version,
      username=self.username,
      password=self.password,
      user_agent=self.user_agent,
      logerr=self.get_logger().error,
      logwarn=self.get_logger().warning,
      loginfo=self.get_logger().info,
      logdebug=self.get_logger().debug
      )
    client.ssl = self.ssl
    client.cert = self.cert
    client.key = self.key
    client.ca_cert = self.ca_cert

    client.nmea_parser.nmea_max_length = self._nmea_max_length
    client.nmea_parser.nmea_min_length = self._nmea_min_length
    client.reconnect_attempt_max = self._reconnect_attempt_max
    client.reconnect_attempt_wait_seconds = self._reconnect_attempt_wait_seconds
    client.reconnect_attempt_wait_max_seconds = self.get_parameter('reconnect_attempt_wait_max_seconds').value
    client.rtcm_timeout_seconds = self.rtcm_timeout_seconds

    return client

  def recovery_callback(self):
    """Re-initialize the NTRIP client if its configuration was updated through a parameter update."""
    if self._ntrip_config_updated:
      self.stop()
      # Re-initialize ntrip client with updated configuration
      self._client = self.init_ntrip_client()
      self.run()
      self._ntrip_config_updated = False

  # override run() in the base class with a version that retries reconnect and that only
  # subscribes to nmea and fix if needed
  def run(self):
    # Attempt initial connection; if it fails, enter backoff retry instead of exiting
    if not self._client.connect():
      self.get_logger().warning('Initial connection to NTRIP server failed, will retry with backoff')
      self._client.request_reconnect(reason='Initial connection failed')

    # Setup the subscriber for NMEA and fix data, unless NMEA forwarding is disabled
    self._nmea_sub = None
    self._fix_sub = None
    if self._send_nmea:
      self._nmea_sub = self.create_subscription(Sentence, 'nmea', self.subscribe_nmea, 10)
      self._fix_sub = self.create_subscription(NavSatFix, 'fix', self.subscribe_fix, 10)
    else:
      self.get_logger().info('send_nmea is false; not subscribing to NMEA or fix or forwarding nmea to the caster')

    # Start the timer that will send both RTCM and NMEA data at the configured rate
    self._rtcm_timer = self.create_timer(self.rtcm_request_rate, self.send_rtcm_and_nmea)

    return True

  # override subscribe_nmea() with version that works with reconnects
  def subscribe_nmea(self, nmea):
    # Cache the latest NMEA sentence
    self._latest_nmea = nmea.sentence

  def send_rtcm_and_nmea(self):
    # Request and publish RTCM data (also drives reconnect attempts)
    for raw_rtcm in self._client.recv_rtcm():
      self._rtcm_pub.publish(self._create_rtcm_message(raw_rtcm))

    # Send cached NMEA data if enabled and connected (skip during reconnect to avoid log spam)
    if self._send_nmea and self._latest_nmea is not None and not self._client.reconnecting:
      self._client.send_nmea(self._latest_nmea)

    # Publish a confirmation message to indicate the send_rtcm_and_nmea call
    confirmation_msg = String()
    confirmation_msg.data = "RTCM and NMEA sent at rate: {} Hz".format(1.0 / self.rtcm_request_rate)
    self._rate_confirm_pub.publish(confirmation_msg)

if __name__ == '__main__':
  # Start the node
  rclpy.init()
  node = NTRIPRos()
  try:
    # Spin until we are shut down
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  except BaseException as e:
    raise e
  finally:
    node.stop()

    # Shutdown the node and stop rclpy
    rclpy.shutdown()
