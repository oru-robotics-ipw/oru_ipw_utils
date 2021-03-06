## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['oru_ipw_acceleration', 'oru_ipw_cmd_inverter', 'oru_ipw_system_analysis'],
    package_dir={'': 'src'},
    install_requires=['attrs>=19.1.0', 'numpy>=1.11,<1.12', 'ansicolors'],
)

setup(**setup_args)
