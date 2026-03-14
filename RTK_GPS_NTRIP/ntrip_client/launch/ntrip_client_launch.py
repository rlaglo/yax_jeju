from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
      return LaunchDescription([
          
          # NTRIP 서버 정보
          DeclareLaunchArgument('host',                  default_value='RTS1.ngii.go.kr'),
          DeclareLaunchArgument('port',                  default_value='2101'),
          DeclareLaunchArgument('mountpoint',            default_value='RTK-RTCM32'),

          # 인증 관련
          DeclareLaunchArgument('authenticate',          default_value='true'),
          DeclareLaunchArgument('username',              default_value='ryan0826'),
          DeclareLaunchArgument('password',              default_value='ngii'),
          DeclareLaunchArgument('ntrip_version',         default_value='Ntrip/2.0'),


          DeclareLaunchArgument('ssl',                   default_value='false'),
          DeclareLaunchArgument('cert',                  default_value='None'),
          DeclareLaunchArgument('key',                   default_value='None'),
          DeclareLaunchArgument('ca_cert',               default_value='None'),
          DeclareLaunchArgument('debug',                 default_value='false'),
        
          # 메시지 패키지
          DeclareLaunchArgument('rtcm_message_package',  default_value='rtcm_msgs'),

          DeclareLaunchArgument('nmea_gga_sentence',     default_value='$GPGGA,140328.430,3733.704,N,12656.201,E,1,12,1.0,0.0,M,0.0,M,,*67'),

          # Pass an environment variable to the node
          SetEnvironmentVariable(name='NTRIP_CLIENT_DEBUG', value=LaunchConfiguration('debug')),

          # ******************************************************************
          # NTRIP Client Node
          # ******************************************************************
          Node(
                name='ntrip_client_node',
                namespace='ntrip_client',
                package='ntrip_client',
                executable='ntrip_ros.py',
                parameters=[
                  {
                    # Required parameters used to connect to the NTRIP server
                    'host': LaunchConfiguration('host'),
                    'port': LaunchConfiguration('port'),
                    'mountpoint': LaunchConfiguration('mountpoint'),

                    # Optional parameter that will set the NTRIP version in the initial HTTP request to the NTRIP caster.
                    'ntrip_version': LaunchConfiguration('ntrip_version'),

                    # If this is set to true, we will read the username and password and attempt to authenticate. If not, we will attempt to connect unauthenticated
                    'authenticate': LaunchConfiguration('authenticate'),

                    # If authenticate is set the true, we will use these to authenticate with the server
                    'username': LaunchConfiguration('username'),
                    'password': LaunchConfiguration('password'),

                    # Whether to connect with SSL. cert, key, and ca_cert options will only take effect if this is true
                    'ssl': LaunchConfiguration('ssl'),

                    # If the NTRIP caster uses cert based authentication, you can specify the cert and keys to use with these options
                    'cert': LaunchConfiguration('cert'),
                    'key':  LaunchConfiguration('key'),

                    # If the NTRIP caster uses self signed certs, or you need to use a different CA chain, specify the path to the file here
                    'ca_cert': LaunchConfiguration('ca_cert'),

                    # This frame ID will be added to the RTCM messages published by this node
                    'rtcm_frame_id': 'odom',

                    # Optional parameters that will allow for longer or shorter NMEA messages.
                    'nmea_max_length': 100,
                    'nmea_min_length': 3,

                    # Use this parameter to change the type of RTCM message published by the node
                    'rtcm_message_package': LaunchConfiguration('rtcm_message_package'),

                    # Reconnect parameters
                    'reconnect_attempt_max': 10,
                    'reconnect_attempt_wait_seconds': 5,

                    # RTCM timeout
                    'rtcm_timeout_seconds': 4,
                    
                    # Static NMEA GGA sentence
                    'nmea_gga_sentence': LaunchConfiguration('nmea_gga_sentence')
                  }
                ],
          )
      ])