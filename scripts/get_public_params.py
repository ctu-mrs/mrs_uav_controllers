#!/usr/bin/python3

import rospkg
import os

class ParamsGetter:

    def __init__(self):

        package_name = "mrs_uav_controllers"

        rospack = rospkg.RosPack()

        package_path = rospack.get_path(package_name)

        file_paths = []

        for path, subdirs, files in os.walk(package_path + "/config/public/"):
            for name in files:
                file_paths.append(os.path.join(path, name))

        for file_path in file_paths:

            if os.path.getsize(file_path) == 0:
                continue

            with open(file_path, 'r') as file:
                print(file.read())

if __name__ == '__main__':
    params_getter = ParamsGetter()
