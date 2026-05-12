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
ROS_UID=${UID} ROS_SSH_PUBLIC_KEY=$(cat ${HOME}/.ssh/id_rsa.pub) docker compose --profile dev build
```

## Developing with Zed

Use Zed's remote deployment functionality with Docker to develop this repo.
First, start the development containers:

```bash
ROS_UID=${UID} ROS_SSH_PUBLIC_KEY=$(cat ${HOME}/.ssh/id_rsa.pub) docker compose --profile dev up -d
```

Now, set up Zed for remote deployment. An easy way to do that is by adding
the following section to `~/.config/zed/settings.json`:

```json
"ssh_connections": [
    {
      "host": "localhost",
      "username": "ros",
      "port": 1825,
      "args": [],
      "projects": [
        {
          "paths": [
            "/home/ros/ros_libcamera"
          ]
        }
      ]
    }
  ],
```

After this, you should be able to open the remote path. To test that everything
is working, run the `catkin_make` task from within Zed. If you get an error
about the workspace setup file not existing, you will have to manually
initialize the workspace first.

## Testing on the Camera'

Some tools are also provided for testing the camera while connected to your
development machine. First, ensure that the IP of your machine is set statically
to 192.168.1.7.

Then, you can start `rviz` and `rqt`:

```bash
ROS_UID=${UID} ROS_SSH_PUBLIC_KEY=$(cat ${HOME}/.ssh/id_rsa.pub) docker compose --profile visualize up -d
```
