import json
import os

from ament_index_python import get_resource

def initializeRobotsOptions(comboBox):
    _, shared_package_path = get_resource('packages', 'shared')
    dataFilePath = os.path.join(shared_package_path, 'share', 'shared', 'data', 'robots')

    for index, fileName in enumerate(os.listdir(dataFilePath)):
        filePath = dataFilePath + '/' + fileName
        dataFile = open(filePath)
        data = json.load(dataFile)
        dataFile.close()
        robotName = data['robotName']
        id = data['id']

        itemData = {
            "fileName": fileName,
            "filePath": filePath,
            "id": id,
        }

        comboBox.addItem(robotName, itemData)