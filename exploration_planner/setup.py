from setuptools import setup

package_name = 'exploration_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={'': 'src'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TianHao',
    maintainer_email='17687198578@163.com',
    description='Global exploration planner',
    license='MIT',
)
