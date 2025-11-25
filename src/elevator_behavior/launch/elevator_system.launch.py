#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # -------- Launch Arguments --------
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock (Gazebo 등)'
    )

    start_floor_arg = DeclareLaunchArgument(
        'start_floor_idx',
        default_value='1',   # 0=B1, 1=1F, 2=2F, ...
        description='엘리베이터 시작 층 인덱스 (B1, 1, ..., 5=)'
    )

    dest_floor_arg = DeclareLaunchArgument(
        'dest_floor_idx',
        default_value='3',
        description='엘리베이터 목적 층 인덱스 (B1, 1, ..., 5)'
    )

    start_floor_idx = LaunchConfiguration('start_floor_idx')
    dest_floor_idx = LaunchConfiguration('dest_floor_idx')

    # -------- Nodes --------

    # 1) 문 열림 감지 (LiDAR ROI)
    door_detector_node = Node(
        package='elevator',
        executable='elevator_door_roi_detector.py',  # ros2 run과 동일하게
        name='elevator_door_roi_detector',
        output='screen',
        parameters=[{
            'scan_topic': '/scan',
            'roi_center_deg': 180.0,                # 전방 기준
            'roi_width_deg': 30.0,
            'closed_ref_dist': 1.2,
            'open_delta': 0.7,
            'min_valid_ratio': 0.25,
            'median_window': 3,
            'hysteresis': 0.1,
            'door_flag_topic': '/elevator/door_open_flag'
        }]
    )

    # 2) 층 검출 (IMU 기반)
    floor_detector_node = Node(
        package='elevator',
        executable='elevator_floor_detector.py',
        name='elevator_floor_detector',
        output='screen',
        parameters=[{
            'imu_topic': '/ebimu/imu',
            'start_floor_idx': start_floor_idx,     # Launch Arg로 설정
            'dest_floor_idx': dest_floor_idx,       # Launch Arg로 설정
            'out_flag_topic': '/elevator/out_flag',
            'auto_start': True
        }]
    )

    # 3) FSM (door + floor flag → /destination 명령)
    fsm_node = Node(
        package='elevator',
        executable='elevator_fsm.py',
        name='elevator_fsm',
        output='screen',
        parameters=[{
            'destination_topic': '/destination',            # navigator와 연결
            'door_flag_topic': '/elevator/door_open_flag', # 1) 노드 출력
            'out_flag_topic': '/elevator/out_flag',        # 2) 노드 출력
            'cmd_topic': '/elevator_fsm_cmd',
            'state_topic': '/elevator_fsm_state',
            'door_open_min_count': 3
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        start_floor_arg,
        dest_floor_arg,
        door_detector_node,
        floor_detector_node,
        fsm_node,
    ])
