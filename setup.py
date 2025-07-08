from setuptools import find_packages, setup

package_name = 'dr_writer'

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
    maintainer='we',
    maintainer_email='llaayy.kr@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'drawing = dr_writer.drawing:main',
            'multi_stroke_drawing = dr_writer.multi_stroke_drawing:main',
            'multi_stroke_drawing_img = dr_writer.multi_stroke_drawing_img:main',
            'drawing_and_erase = dr_writer.drawing_and_erase:main',

            'test_movel = dr_writer.test:main',
            'test_topic = dr_writer.test2:main',
            'test_spline = dr_writer.test3:main',
            'test_white_board = dr_writer.test4:main',
            'test_amovesx = dr_writer.test5:main',
            'multi_stroke_board = dr_writer.multi_stroke_board:main',
            'multi_stroke_board_with_erase = dr_writer.multi_stroke_board_with_erase:main',
            
            'edge_extractor = dr_writer.edge_extractor:main',

            'monitoring = dr_writer.monitoring:main',
        ],
    },
)
