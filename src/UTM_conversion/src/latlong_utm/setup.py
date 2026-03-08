#lattoutm: Converts lat-lon coordinates to utm (Publishes to topic1, Subscribes to topic2)
#utmtolat: Converts utm coordinates to lat-lon (Publishes to topic2, Subscribes to topic1)
#latlonpub: Publishes a few lat-lon coordinates for conversion into utm (Publishes to topic2)
#utmpub: Publishes a few utm coordinates for conversion into lat-lon (Publishes to topic1)


from setuptools import setup

package_name = 'latlong_utm'

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
    maintainer='sanket',
    maintainer_email='sanket151001@gmail.com',
    description='Publisher and Subscriber for interconversion of data between latlong and utm',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'lattoutm = latlong_utm.latlong_to_utm:main',
                'utmtolat = latlong_utm.utm_to_latlong:main', ############
                'latlonpub = latlong_utm.latlon:main',        ############
                'utmpub = latlong_utm.utmpubfunc:main',        ############
        ],
    },
)
