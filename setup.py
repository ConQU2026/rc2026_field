from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'rc2026_field'

def package_files(data_files, directory_list):
    '''
    Collects all files in the specified directories and prepares them for installation.
    Args:
        data_files (list): List to append the collected files to.
        directory_list (list): List of directories to search for files.
    Returns:
        list: Updated data_files list with collected files.
    '''
    paths_dict = {}
    for directory in directory_list:
        for (path, directories, filenames) in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)
                if install_path in paths_dict:
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]
    
    for key in paths_dict:
        data_files.append((key, paths_dict[key]))
    return data_files


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files= package_files([
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
    ], ['resource', 'launch', 'urdf', 'config', 'rviz', 'meshes']),
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rc3',
    maintainer_email='edmounds@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            # 'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'target_controller = rc2026_field.target_controller:main',
        ],
    },
)
