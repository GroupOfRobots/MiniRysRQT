from shared.utils.ssh_data import getSSHPassword
def killSSHProcess(pid, data, ssh):
    command = f'sudo kill -9 {pid}'
    try:
        password = getSSHPassword(data)
        stdin, stdout, stderr = ssh.exec_command(command)
        stdin.write(password + '\n')
        stdin.flush()
        return None, command
    except Exception as exception:
        print("Exception killProcess")
        print(exception)
        return exception, command