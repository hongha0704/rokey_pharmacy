from setuptools import find_packages, setup
import glob
import os

package_name = 'rokey_project'

weight_files = glob.glob(os.path.join('weights', '*.pt'))
classifier_files = glob.glob(os.path.join('weights', '*.pth'))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/weights', weight_files),
        ('share/' + package_name + '/weights', classifier_files),
        ('share/' + package_name + '/models', ['models/hello_rokey_8332_32.tflite']),
        ('share/' + package_name + '/models', ['models/rokey-stop_en_linux_v3_0_0.ppn']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hongha',
    maintainer_email='hongha@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "QR_detector=rokey_project.QR_detector:main",
            "docx_to_qr=rokey_project.docx_to_qr:main",
            "picture_capture=rokey_project.picture_capture:main",
            "object_detection=rokey_project.object_detection:main",
            "pose_estimation=rokey_project.pose_estimation:main",
            "segmentation=rokey_project.segmentation:main",
            "robot_grip_pill=rokey_project.robot_grip_pill:main",
            "robot_control=rokey_project.robot_control:main",
            "robot_grip_test=rokey_project.robot_grip_test:main",
            "main_vision_realsense=rokey_project.main_vision_realsense:main",
            "main_vision_realsense_250602=rokey_project.main_vision_realsense_250602:main",
            "main_vision_webcam=rokey_project.main_vision_webcam:main",
            "main_robot_control=rokey_project.main_robot_control:main",
            "main_robot_control_250602=rokey_project.main_robot_control_250602:main",
            "main_publish_test=rokey_project.main_publish_test:main",
            "speaker=rokey_project.ai_speaker:main",
            "ultra=rokey_project.dual_ultra:main",
        ],
    },
)
