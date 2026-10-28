from setuptools import setup


package_name = 'wa1_robot_state'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wanglei',
    maintainer_email='wanglei@example.com',
    description='WA1 upper-limb state and safety-lock command-line tools',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'wa1_state = wa1_robot_state.cli:main',
        ],
    },
)
