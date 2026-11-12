from setuptools import setup

package_name = 'humanoid_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/display.launch.py',
            'launch/physics_demo.launch.py',
            'launch/quad_physics_walk.launch.py',
        ]),
        ('share/' + package_name + '/rviz', ['rviz/humanoid_sim.rviz']),
        ('share/' + package_name + '/mujoco', [
            'mujoco/ball_collision.xml',
            'mujoco/quad_walk.xml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='computer_graphics',
    maintainer_email='noreply@example.com',
    description=(
        'Republishes the computer_graphics OpenGL demo walking/trot animation '
        'as ROS2 joint_states + TF for RViz2 visualization.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'humanoid_state_publisher = humanoid_sim.humanoid_state_publisher:main',
            'quadruped_state_publisher = humanoid_sim.quadruped_state_publisher:main',
            'mujoco_ball_sim = humanoid_sim.mujoco_ball_sim:main',
            'quad_mujoco_walk = humanoid_sim.quad_mujoco_walk:main',
        ],
    },
)
