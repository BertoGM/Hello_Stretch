# simple_web_teleop

A single HTML page with two buttons that drive the Stretch arm to its top or
bottom lift position via rosbridge.

## Prerequisites on the sim machine

Install every dependency from Hello Robot's
[Stretch Simulation — Web Interface Control](https://docs.hello-robot.com/0.3/ros2/stretch_simulation/#web-interface-control)
guide.

## Running on the sim machine

Start the sim and the web interface in two terminals:

```bash
# Terminal 1: simulation with cameras
MUJOCO_GL=egl ros2 launch stretch_simulation stretch_mujoco_driver.launch.py \
    use_mujoco_viewer:=false mode:=position use_cameras:=true

# Terminal 2: web interface (starts rosbridge at wss://localhost:9090)
ros2 launch stretch_simulation stretch_simulation_web_interface.launch.py
```

Then in a browser on the sim machine:

1. First-time only: visit <https://localhost:9090> and click through the
   self-signed cert warning.
2. Open `index.html`.

## Running from a different machine

Open an SSH tunnel from your laptop to the sim machine — no code changes
needed, because `localhost` in the HTML resolves to your laptop's forwarded
port.

```bash
ssh -L 9090:localhost:9090 <user>@<sim-host>
```

Leave that terminal open, accept the cert at <https://localhost:9090> once,
then open `index.html` locally.
