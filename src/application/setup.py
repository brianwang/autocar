from setuptools import setup

package_name = 'autocar_application'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Autocar Dev',
    maintainer_email='dev@autocar.local',
    description='Autocar application layer',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_machine = autocar_application.state_machine:main',
            'voice_pipeline = autocar_application.voice_pipeline:main',
            'web_dashboard = autocar_application.web_dashboard:main',
        ],
    },
)
