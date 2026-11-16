"""find_port: which adapter 'make gello' picks as the GELLO."""

import os

from gello_teleop import find_port

GELLO = "/hostdev/serial/by-id/usb-FTDI_GELLO-if00-port0"
OTHER = "/hostdev/serial/by-id/usb-FTDI_OTHER-if00-port0"


def answers(*ports):
    """A fake ping: only these ports answer."""
    return lambda port: None if port in ports else "servo 1 does not answer"


def test_no_adapter():
    port, error = find_port.choose([], answers())
    assert port is None and "no USB serial adapter" in error


def test_one_adapter_that_answers():
    assert find_port.choose([GELLO], answers(GELLO)) == (GELLO, None)


def test_one_adapter_without_power():
    port, error = find_port.choose([GELLO], answers())
    assert port is None and "powered" in error and GELLO in error


def test_picks_the_one_that_answers():
    assert find_port.choose([OTHER, GELLO], answers(GELLO)) == (GELLO, None)


def test_several_gellos_need_gello_port():
    port, error = find_port.choose([GELLO, OTHER], answers(GELLO, OTHER))
    assert port is None and "GELLO_PORT" in error


def test_host_path_goes_to_hostdev(tmp_path):
    host_dev = str(tmp_path)
    assert (find_port.container_path("/dev/ttyUSB0", host_dev)
            == os.path.join(host_dev, "ttyUSB0"))


def test_path_unchanged_without_hostdev(tmp_path):
    # A container started by an older 'make run' (--device): no /hostdev
    missing = str(tmp_path / "missing")
    assert find_port.container_path("/dev/ttyUSB0", missing) == "/dev/ttyUSB0"


def test_container_path_unchanged(tmp_path):
    path = os.path.join(str(tmp_path), "ttyUSB0")
    assert find_port.container_path(path, str(tmp_path)) == path


def test_candidates_only_usb_serial(tmp_path):
    by_id = tmp_path / "serial" / "by-id"
    by_id.mkdir(parents=True)
    for name in ("ttyUSB0", "ttyACM0"):
        (tmp_path / name).touch()
    os.symlink("../../ttyUSB0", by_id / "usb-FTDI_GELLO-if00-port0")
    os.symlink("../../ttyACM0", by_id / "usb-Arduino-if00")
    assert find_port.candidates(str(tmp_path)) == [
        str(by_id / "usb-FTDI_GELLO-if00-port0")]


def test_no_serial_devices(tmp_path):
    # /dev/serial only exists while a serial adapter is plugged in
    assert find_port.candidates(str(tmp_path)) == []
