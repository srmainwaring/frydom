from setuptools import setup, find_packages

setup(

    name='frydom_tools',

    version='2.0.0',

    packages=find_packages(),

    author='P.-Y. Wuillaume',

    author_email='pierre-yves.wuillaume@dice-engineering.com',

    description='Python module for reading Nemoh output files and writing HDB5 file for FRyDoM',

    long_description=open('README.rst').read(),

    include_package_data=True,

    install_requires=['numpy', 'matplotlib', 'h5py', 'argcomplete'],

    entry_points={
        'console_scripts': ['hdb5tool=frydom.HDB5tool.HDB5tool:main',
        'generate_video=frydom.generate_video:main',
        'generate_gif=frydom.generate_gif:main',
        'hdb5merge=frydom.HDB5merge.HDB5merge:main']
    },

    classifiers=[
        'Programming Language :: Python',
        'Programming Language :: Python :: 3.7',
        'Development Status :: Production',
        'Natural Language :: English',
        'Operating System :: OS Independent',
        'Topic :: Scientific/Engineering',
    ],

)
