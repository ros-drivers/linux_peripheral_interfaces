## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
<<<<<<< HEAD:pr2_computer_monitor/setup.py
    packages=['pr2_computer_monitor'],
=======
    packages=['computer_hw'],
>>>>>>> d60763b ([capability] Add computer_hw package that is previously called _pr2_computer_monitor):computer_hw/setup.py
    package_dir={'': 'src'})

setup(**setup_args)
