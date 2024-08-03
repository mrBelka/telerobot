from setuptools import find_packages, setup

package_name = 'audio_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='base1',
    maintainer_email='mishklgpmi@mail.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server1 = audio_driver.server1:main',
            'client1 = audio_driver.client1:main',
            'server2 = audio_driver.server2:main',            
            'client2 = audio_driver.client2:main'
        ],
    },
)
