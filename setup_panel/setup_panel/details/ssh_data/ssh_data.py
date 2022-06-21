

from python_qt_binding.QtWidgets import QWidget

class SshData(QWidget):
    def __init__(self, widget):
        super(SshData, self).__init__()
        self.widget = widget
        data = self.widget.data
        
        self.setSshData(data['ssh'])

    def setSshData(self, sshData):
        self.widget.sshPortInputUI.setText(sshData['port'])
        self.widget.sshUsernameInputUI.setText(sshData['username'])
        self.widget.sshPasswordInputUI.setText(sshData['password'])