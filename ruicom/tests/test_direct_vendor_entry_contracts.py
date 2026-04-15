from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _read(rel: str) -> str:
    return (ROOT / rel).read_text(encoding='utf-8')


def test_nav_c_requires_real_driver_launches() -> None:
    text = _read('vendor_workspace/newznzc_ws/src/nav_demo/launch/nav_c.launch')
    assert 'driver_launch" value="$(arg lidar_driver_launch)"' in text
    assert 'driver_launch" value="$(arg imu_driver_launch)"' in text
    assert '<arg name="require_driver_launch" value="true" />' in text


def test_gmapping_requires_real_driver_launches() -> None:
    text = _read('vendor_workspace/newznzc_ws/src/car_bringup/launch/gmapping.launch')
    assert 'driver_launch" value="$(arg lidar_driver_launch)"' in text
    assert 'driver_launch" value="$(arg imu_driver_launch)"' in text
    assert text.count('<arg name="require_driver_launch" value="true" />') >= 2


def test_vendor_wrappers_default_to_fail_fast() -> None:
    wrappers = [
        'vendor_workspace/newznzc_ws/src/OrbbecSDK_ROS/dabai_dcw2.launch',
        'vendor_workspace/newznzc_ws/src/lslidar_driver/launch/lslidar_serial.launch',
        'vendor_workspace/newznzc_ws/src/wit_ros_imu/launch/wit_imu.launch',
    ]
    for rel in wrappers:
        assert '<arg name="require_driver_launch" default="true" />' in _read(rel)
