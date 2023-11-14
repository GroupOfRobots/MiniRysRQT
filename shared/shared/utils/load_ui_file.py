import os

from python_qt_binding import loadUi
from ament_index_python import get_resource

def loadUiFile(widget, packageName, fileName):
    _, packagePath = get_resource('packages', packageName)
    uiFile = os.path.join(packagePath, 'share', packageName, 'resource', fileName)
    loadUi(uiFile, widget)