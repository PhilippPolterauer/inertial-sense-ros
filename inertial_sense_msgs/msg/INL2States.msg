Header header                       # GPS time of week (since Sunday morning) in seconds
geometry_msgs/Quaternion quatecef   # Quaternion body rotation with respect to ECEF
geometry_msgs/Vector3 velecef       # (m/s) Velocity in ECEF frame
geometry_msgs/Vector3 posecef       # (m) Position in ECEF frame
geometry_msgs/Vector3 gyrobias      # (rad/s) Gyro bias
geometry_msgs/Vector3 accelbias     # (m/s^2) Accelerometer bias
float32 barobias                    # (m) Barometer bias
float32 magdec                      # (rad) Magnetic declination
float32 magInc                      # (rad) Magnetic inclination
