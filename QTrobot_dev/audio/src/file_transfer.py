import paramiko


class FileTransfer:
    def __init__(self, remote_host, remote_username, remote_password, remote_path):
        self.remote_host = remote_host
        self.remote_username = remote_username
        self.remote_password = remote_password
        self.remote_path = remote_path
        self.ssh = paramiko.SSHClient()
        self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.ssh.connect(self.remote_host, username=self.remote_username, password=self.remote_password)
        self.sftp = self.ssh.open_sftp()

    def __del__(self):
        if self.sftp:
            self.sftp.close()
        if self.ssh:
            self.ssh.close()
        
    def transfer_file(self, local_path):
        self.sftp.put(local_path, self.remote_path)
        return self.remote_path
