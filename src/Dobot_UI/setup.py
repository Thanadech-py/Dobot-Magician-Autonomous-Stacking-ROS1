from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['Dobot_UI', 'Dobot_UI.widgets', 'Dobot_UI.models'],
    package_dir={'': '.'}
)

setup(**d)
