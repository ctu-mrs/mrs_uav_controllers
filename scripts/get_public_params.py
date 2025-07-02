#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory
import os

class ParamsGetter:

    def __init__(self):

        package_name = "mrs_uav_controllers"

        package_path = get_package_share_directory(package_name)

        file_paths = []

        for path, subdirs, files in os.walk(package_path + "/config/public/"):
            for name in files:
                if name.endswith(".yaml") or name.endswith(".yml"):
                  file_paths.append(os.path.join(path, name))

        for file_path in file_paths:

            if os.path.getsize(file_path) == 0:
                continue

            with open(file_path, 'r') as file:
                try:
                    contents = file.read()
                    print("######################################")
                    print("# The following section was taken from:")
                    print("# {}".format(file_path))
                    print("")
                    print(contents)
                except:
                    print("There was a problem while opening the file '{}'".format(file_path))
                    break

if __name__ == '__main__':
    params_getter = ParamsGetter()
