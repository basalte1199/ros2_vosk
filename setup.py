from setuptools import setup

package_name = 'ros2_vosk'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adrian',
    maintainer_email='agrigo2001@yahoo.com.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tts_engine=ros2_vosk.tts_engine:main',
            'vosk_node=ros2_vosk.vosk_node:main',
            'vosk_ros_model_downloader=ros2_vosk.vosk_ros_model_downloader:main',
            'vosk_service=ros2_vosk.vosk_service:main',
        ],
    },
)
