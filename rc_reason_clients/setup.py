## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['rc_reason_clients'],
    package_dir={'': 'src'},
    scripts=['scripts/rc_hand_eye_calibration_client',
             'scripts/rc_april_tag_detect_client',
             'scripts/rc_qr_code_detect_client']
)

setup(**setup_args)
