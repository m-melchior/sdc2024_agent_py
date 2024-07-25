#!/usr/bin/env python

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    micro_xrce_agent = ExecuteProcess(
        cmd=[[
            'MicroXRCEAgent udp4 --port 8888 -v '
        ]],
        shell=True
    )

    sdc2024_agent_node = Node(
        package='sdc2024_agent_py',
        executable='sdc2024_agent.py',
        output='screen',
        shell=True,
    )

    return LaunchDescription([
        micro_xrce_agent,
        sdc2024_agent_node
    ])
