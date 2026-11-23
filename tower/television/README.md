# TeleVision: ZED video to the Apple Vision Pro

Stereo video from a ZED Mini to a WebXR headset (Apple Vision Pro), based
on [Open-TeleVision](https://github.com/OpenTeleVision/TeleVision). The
headset's head pose comes back as the ROS topic `/television/head_pose`
(`geometry_msgs/PoseStamped`, X forward, Y left, Z up).

Runs in Docker on any Linux machine with an NVIDIA GPU. Commands from this
folder; `make help` lists them.

## Once per machine

1. NVIDIA driver >= 570 (`nvidia-smi`) and the
   [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html):

       # Ubuntu: add NVIDIA's repository first (link above). Arch: from the repos.
       sudo apt install nvidia-container-toolkit    # or: sudo pacman -S nvidia-container-toolkit
       sudo nvidia-ctk runtime configure --runtime=docker
       sudo systemctl restart docker

2. Build and start:

       make setup
       make udev        # sudo: lets the container open the ZED

3. HTTPS certificate (the headset only opens WebXR pages over HTTPS). One of:

   - **Tailscale** (machine and headset in the same tailnet, any network):

         make cert-tailscale

     Needs HTTPS certificates enabled in the tailnet's admin console. Lasts
     90 days: rerun it when the headset says the certificate expired.

   - **Own CA** (same local network):

         make cert

     Copy `certs/ca/rootCA.pem` to the headset (AirDrop, mail) and install it:
     Settings > General > VPN & Device Management (install the profile), then
     Settings > General > About > Certificate Trust Settings (turn it on).
     Rerun `make cert` if the machine's address changes.

   Never commit or share `certs/`: it holds the private keys.

4. On the headset: Settings > Apps > Safari > Advanced > Feature Flags >
   WebXR on.

5. Open TCP port 8012 in the firewall, if there is one
   (`sudo ufw allow 8012`).

## Each session

1. Plug the ZED in (USB 3), then `make run`.
2. `make stream`. On the headset, open in Safari
   `https://<machine>:8012` (e.g. `https://tower.tail219772.ts.net:8012`),
   then "Enter VR". Bookmark it.
   `<machine>`: its Tailscale name (`make cert-tailscale` prints the
   address), or its IP with `make cert`.
3. `make stop` at the end.

With Tailscale, `tailscale ping <headset>` should say `via <ip>:<port>`:
`via DERP` means a relay, with more delay.

Without the ZED or the headset: `make stream-fake` (test pattern: `L` in the
left eye, `R` in the right one, the orange bar in front of the grid). Then
`make test-stream` checks it like a headset would, or open
`https://localhost:8012` in a browser on this machine.
`make head-echo` prints the head pose while a headset is connected.

## Camera that follows the head (in testing)

The ZED's mount has 2 servos (ID 1 up/down, ID 2 left/right) on their own
U2D2. With `make stream` running and the headset in VR:

1. Move the camera to neutral by hand (`make neck-read` shows the angles;
   neutral in `ros2_ws/television_teleop/config/neck.yaml`). Ctrl+C.
2. `make neck`: dry run, prints where it would move the servos. Nothing moves.
   With the headset on, look forward and keep still until it says
   "Reference set".
3. `make neck LIVE=1`: the servos follow the head, at most 3° from
   neutral (`RANGE=...`), 4°/s. Ctrl+C turns the torque off.

It stops by itself (torque off) when the head pose stops for 2 s or a
servo can't reach its goal. `make test-neck` checks all this with fake
servos.

Settings (resolution, rates, JPEG quality):
`ros2_ws/television_teleop/config/stream.yaml`, then restart `make stream`.
