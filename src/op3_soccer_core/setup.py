from setuptools import setup, find_packages

package_name = 'op3_soccer_core'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/soccer_stack.launch.py',
            'launch/soccer_stack_single.launch.py',
            'launch/soccer_stack_full.launch.py',
            'launch/soccer_4v4_gui_full_yolo.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/rc_hl_kidsize.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mdzulfikri',
    maintainer_email='motionbascorro@gmail.com',
    description='Strategy architecture for RoboCup KidSize humanoid OP3.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # Layer 1 — GC
            'gc_state_adapter = op3_soccer_core.gc_state_adapter_node:main',
            # Layer 2 — Compliance
            'op3_compliance = op3_soccer_core.compliance_node:main',
            # Layer 3 — Safety
            'op3_safety_monitor = op3_soccer_core.safety_monitor_node:main',
            # Layer 4 — Tactical
            'op3_tactical = op3_soccer_core.tactical_node:main',
            # Layer 5 — Team Coordination
            'op3_team_comm = op3_soccer_core.team_comm_node:main',
            'op3_team_coordinator = op3_soccer_core.team_coordinator_node:main',
            # Layer 6 — Planning
            'op3_planner = op3_soccer_core.planner_node:main',
            # Layer 7 — Perception / Localization
            'op3_ball_estimator = op3_soccer_core.ball_estimator_node:main',
            'op3_perception = op3_soccer_core.perception_node:main',
            'op3_localization = op3_soccer_core.localization_node:main',
            # Layer 8 — Motion
            'op3_motion = op3_soccer_core.motion_node:main',
            # Simulation adapter (replaces op3_manager in Webots)
            'op3_webots_walking_bridge = op3_soccer_core.webots_walking_bridge_node:main',
            # Legacy (kept for reference during transition)
            'op3_soccer_brain = op3_soccer_core.soccer_brain_node:main',
        ],
    },
)
