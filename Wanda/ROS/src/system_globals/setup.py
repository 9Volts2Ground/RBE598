from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['system_globals'],
    package_dir={'': 'src'},
    install_requires=['numpy']
)

setup(**d)