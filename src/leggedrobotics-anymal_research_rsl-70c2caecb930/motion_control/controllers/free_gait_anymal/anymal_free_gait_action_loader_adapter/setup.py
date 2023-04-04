from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    # packages=['anymal_free_gait_action_loader_adapter'],
    # package_dir={'': ''}
)

setup(**d)
