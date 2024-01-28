def getSSHData(data):
    sshData = data.get('ssh', {})
    host = sshData.get('host', '')
    username = sshData.get('username')
    password = sshData.get('password')
    port = _parsePort(sshData)
    return host, port, username, password


def getSSHHost(data):
    sshData = data.get('ssh', {})
    return sshData.get('host', '')


def getSSHUsername(data):
    sshData = data.get('ssh', {})
    return sshData.get('username')


def getSSHPassword(data):
    sshData = data.get('ssh', {})
    return sshData.get('password')


def getSSHPort(data):
    sshData = data.get('ssh', {})
    return _parsePort(sshData)


def _parsePort(sshData):
    try:
        return int(sshData.get('port'))
    except Exception as exception:
        return 22
