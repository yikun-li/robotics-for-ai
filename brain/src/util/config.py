#Importing some python utilities
import os
import sys

def set_aldebaran_path():
    """Set aldebaran software path"""
    path = `os.environ.get("AL_DIR")`
    home = `os.environ.get("HOME")`

    #Checking AL_DIR
    if path == "None":
        # runs on the nao: set path to location of software on Nao
        alPath = "/opt/naoqi/lib"
        alPath = alPath.replace("~", home)
        alPath = alPath.replace("'", "")
        sys.path.append(alPath)
    else:
        # runs on pc: path is specified in AL_DIR
        alPath = path + "/lib"
        alPath = alPath.replace("~", home)
        alPath = alPath.replace("'", "")
        sys.path.append(alPath)


