# mars_camera_module
Contains all the code that runs on the MARS camera module.

## Building

Download the Hailort .deb TAPPAS installer zip file from the HAILO developer zone
and place it in the root directory of the repository. The following versions are
recommended for compatibility with the Raspberry Pi:

- HailoRT: 4.20.0
- TAPPAS: 3.31.0

Then run:

```bash
ROS_UID=${UID} docker-compose build
```
