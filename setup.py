from glob import glob
from setuptools import find_packages, setup

setup(
    name='amr_research_lab',
    version='0.1.0',
    packages=find_packages(exclude=('tests',)),
    python_requires='>=3.10',
    install_requires=['setuptools'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/amr_research_lab']),
        ('share/amr_research_lab', ['package.xml']),
        ('share/amr_research_lab/launch', glob('launch/*.launch.py')),
        ('share/amr_research_lab/config', glob('config/*.rviz')),
    ],
    maintainer='ROS with Robots contributors',
    maintainer_email='266934573+Hareespandy-N@users.noreply.github.com',
    description='Original simulation-first AMR planning, tracking, and docking experiments',
    long_description='Educational AMR experiments with original code and diagrams; see README.md.',
    license='MIT',
    url='https://github.com/Hareespandy-N/ROS-with-robots',
    entry_points={'console_scripts': [
        'amr-lab = amr_research_lab.__main__:main',
        'sim_node = amr_research_lab.ros_node:main',
    ]},
)
