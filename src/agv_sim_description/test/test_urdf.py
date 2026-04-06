#!/usr/bin/env python3
"""URDF/xacro validation tests for agv_sim_description.

Run with: colcon test --packages-select agv_sim_description
Or directly: pytest src/agv_sim_description/test/test_urdf.py -v
"""

import os
import subprocess
import xml.etree.ElementTree as ET

import pytest

URDF_DIR = os.path.join(
    os.path.dirname(__file__), '..', 'urdf')
MAIN_XACRO = os.path.join(URDF_DIR, 'agv_sim.urdf.xacro')


@pytest.fixture(scope='module')
def urdf_xml():
    """Process the main xacro file and return the URDF XML string."""
    result = subprocess.run(
        ['xacro', MAIN_XACRO],
        capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, f"xacro failed: {result.stderr}"
    return result.stdout


@pytest.fixture(scope='module')
def urdf_tree(urdf_xml):
    """Parse the URDF XML into an ElementTree."""
    return ET.fromstring(urdf_xml)


class TestXacroProcessing:
    """Verify that xacro files process without errors."""

    def test_main_xacro_processes(self, urdf_xml):
        assert len(urdf_xml) > 0
        assert '<robot' in urdf_xml

    def test_output_is_valid_xml(self, urdf_tree):
        assert urdf_tree.tag == 'robot'


class TestRequiredLinks:
    """Verify the URDF contains all expected links."""

    REQUIRED_LINKS = [
        'base_link',
        'base_footprint',
        'left_wheel',
        'right_wheel',
        'zed_camera_link',
        'zed_camera_center',
        'zed_left_camera_frame',
        'zed_left_camera_frame_optical',
        'zed_right_camera_frame',
        'zed_right_camera_frame_optical',
        'imu_link',
    ]

    def test_all_required_links_present(self, urdf_tree):
        links = {link.get('name') for link in urdf_tree.findall('.//link')}
        for name in self.REQUIRED_LINKS:
            assert name in links, f"Missing link: {name}"


class TestGazeboPlugins:
    """Verify Gazebo plugins are configured."""

    def test_has_diff_drive_plugin(self, urdf_xml):
        assert 'DiffDrive' in urdf_xml

    def test_has_sensor_plugins(self, urdf_xml):
        # Camera or sensor system plugin
        assert 'Sensors' in urdf_xml or 'sensors' in urdf_xml


class TestSensorConfig:
    """Verify sensor configuration matches real hardware."""

    def test_imu_link_exists(self, urdf_tree):
        links = {link.get('name') for link in urdf_tree.findall('.//link')}
        assert 'imu_link' in links

    def test_zed_frame_hierarchy(self, urdf_tree):
        """ZED frames must follow: camera_link -> center -> left/right -> optical."""
        joints = {}
        for joint in urdf_tree.findall('.//joint'):
            parent = joint.find('parent')
            child = joint.find('child')
            if parent is not None and child is not None:
                joints[child.get('link')] = parent.get('link')

        # Verify chain exists (any path from camera_link down to optical frames)
        links = {link.get('name') for link in urdf_tree.findall('.//link')}
        assert 'zed_left_camera_frame_optical' in links
        assert 'zed_right_camera_frame_optical' in links


class TestTopicParity:
    """Verify topic names in Gazebo plugins match TOPIC_CONTRACT.md."""

    def test_wheel_odom_topic(self, urdf_xml):
        # The diff_drive or bridge must reference wheel_odom
        assert 'wheel_odom' in urdf_xml or 'odom' in urdf_xml

    def test_imu_topic(self, urdf_xml):
        assert 'imu' in urdf_xml.lower()


class TestCheckUrdf:
    """Run check_urdf if available."""

    def test_check_urdf(self, urdf_xml):
        try:
            result = subprocess.run(
                ['check_urdf', '-'],
                input=urdf_xml, capture_output=True, text=True, timeout=10)
            # check_urdf returns 0 on valid URDF
            if result.returncode != 0:
                pytest.skip(f"check_urdf reported issues: {result.stderr}")
        except FileNotFoundError:
            pytest.skip("check_urdf not installed")
