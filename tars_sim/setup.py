from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
	version='0.0.0',
	scripts=['scripts/tars_sim']
)

setup(**setup_args)