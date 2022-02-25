This repository uses Mahony's IMU sensor fusion algorithm which is a DCM filter.

The explanation of the algorithm:
https://ahrs.readthedocs.io/en/latest/filters/mahony.html#mahony2008

The C code of Mahony's DCM filter algorithm comes from:
https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/

The C code of mapping quaternion to Euler angles comes from:
http://www.realtimerendering.com/resources/GraphicsGems/gemsiv/euler_angle/

18, Feb:
It's not stable, and the visualization is not good as well. We can only get rough orientation in Euler by running this code.
