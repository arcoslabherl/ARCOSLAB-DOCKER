"""Find the GELLO's serial port, so 'make gello' needs no GELLO_PORT.

'make run' mounts the host's /dev at /hostdev (read-only), so a GELLO
plugged in after the container started shows up there. Prints the port
on stdout; messages go to stderr. Exit code 1 if no GELLO is found.

Usage: gello_find_port [auto | <port>]
  auto    look in /hostdev/serial/by-id/ and keep the adapter whose
          first servo answers a Dynamixel ping
  <port>  use this one; a host path (/dev/...) is translated to /hostdev
"""

import glob
import os
import sys

HOST_DEV = "/hostdev"


def container_path(port, host_dev=HOST_DEV):
    """The container path of a host port (/dev/... -> /hostdev/...).

    Unchanged if the container has no /hostdev (started by an older
    'make run', with --device) or the path is not under /dev.
    """
    if port.startswith("/dev/") and os.path.isdir(host_dev):
        return host_dev + port[len("/dev"):]
    return port


def candidates(host_dev=HOST_DEV):
    """USB-serial adapters by stable name (by-id), ttyUSB* only: the
    container may only open those (cgroup rule in the Makefile)."""
    by_id = sorted(glob.glob(os.path.join(host_dev, "serial", "by-id", "*")))
    return [path for path in by_id
            if os.path.basename(os.path.realpath(path)).startswith("ttyUSB")]


def choose(ports, ping):
    """Pick the GELLO among the adapters.

    ping(port) returns None if a servo answers, or the reason why not.
    Returns (port, None) or (None, error message).
    """
    if not ports:
        return None, ("No GELLO found: no USB serial adapter is connected.\n"
                      "Plug in the GELLO's USB cable.")

    failures = {port: ping(port) for port in ports}
    answering = [port for port, reason in failures.items() if reason is None]

    if len(answering) == 1:
        return answering[0], None
    if answering:
        listed = "\n".join("    " + port for port in answering)
        return None, (f"Several GELLOs answer:\n{listed}\n"
                      "Choose one with GELLO_PORT=<port>.")
    listed = "\n".join(f"    {port}: {reason}"
                       for port, reason in failures.items())
    return None, (f"No GELLO answers on the USB serial adapters:\n{listed}\n"
                  "Is the GELLO powered? (Its servos need their own supply.)")


def dynamixel_ping(baudrate, servo_id):
    """A ping(port) for choose(), with Dynamixel protocol 2.0."""
    from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

    def ping(port):
        handler = PortHandler(port)
        try:
            if not handler.openPort() or not handler.setBaudRate(baudrate):
                return "cannot open it"
        except Exception as error:  # pyserial: permission denied, busy...
            return f"cannot open it ({error})"
        try:
            packet = PacketHandler(2.0)
            _, result, _ = packet.ping(handler, servo_id)
            if result != COMM_SUCCESS:
                return (f"servo {servo_id} does not answer "
                        f"({packet.getTxRxResult(result)})")
            return None
        finally:
            handler.closePort()

    return ping


def driver_settings():
    """Baud rate and first servo id from the package's YAML."""
    # Imported here so the unit tests run without ROS sourced.
    from ament_index_python.packages import get_package_share_directory
    import yaml

    path = os.path.join(get_package_share_directory("gello_teleop"),
                        "config", "gello_panda.yaml")
    with open(path) as config_file:
        params = yaml.safe_load(config_file)["gello_driver"]["ros__parameters"]
    return params["baudrate"], params["joint_ids"][0]


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "auto"

    if port != "auto":
        path = container_path(port)
        if not os.path.exists(path):
            print(f"ERROR: GELLO_PORT {port} not found. "
                  "Is the GELLO plugged in?", file=sys.stderr)
            return 1
        print(path)
        return 0

    if not os.path.isdir(HOST_DEV):
        print("ERROR: this container was started by an older 'make run', "
              "which cannot find the GELLO. Restart it:\n"
              "    make stop && make run", file=sys.stderr)
        return 1
    port, error = choose(candidates(), dynamixel_ping(*driver_settings()))
    if error:
        print("ERROR: " + error, file=sys.stderr)
        return 1
    print(f"GELLO found at {port}", file=sys.stderr)
    print(port)
    return 0


if __name__ == "__main__":
    sys.exit(main())
