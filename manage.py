import yaml
import os
import shutil
import tarfile

yml = yaml.load(open("docker-compose.yml"))
ssh_redir = "ssh_redir/"
homedir = "/app/home/"
sshuser = os.listdir(ssh_redir)

def getusers():
    print("allhome and piper will not use")
    users = yml['services'].keys()
    target_users = []
    for user in users:
        if user not in ['allhome', 'piper']:
            target_users.append(user)
    return target_users


def setIP(users):
    for user in users:
        useryml = yml['services'][user]
        ip = useryml['networks']['mynet']['ipv4_address']

        sshfolder = ssh_redir + user + '/'
        sshpip =  sshfolder + "sshpiper_upstream"
        if user not in sshuser:
            os.mkdir(sshfolder)
            os.chown(sshfolder, 1000, 1000)
            open(sshpip, "w").write("ubuntu@" + ip)
            os.chown(sshpip, 1000, 1000)
            os.chmod(sshpip, 0o600)
        else:
            sshuser.remove(user)
            open(sshpip, "w").write("ubuntu@" + ip)

    for user in sshuser:
        sshfolder = ssh_redir + user + '/'
        shutil.rmtree(sshfolder)

def setHOME(users):
    for user in users:
        useryml = yml['services'][user]
        volume = ""
        for v in useryml['volumes']:
            if v.endswith(":/home/ubuntu"):
                volume = v.split(':')[0]
        if not volume:
            raise IndexError

        # volume = "/app/home/" + user + '/'
        os.makedirs(volume, exist_ok=True)
        os.chown(volume, 1000, 1000)

        if os.path.isfile(volume + ".bashrc"):
            shutil.copy2(volume + ".bashrc", volume + ".bashrc.old")
            os.chown(volume + ".bashrc.old", 1000, 1000)

        tarpath = '/app/default_home/guest101/all.tar'
        tar_ref = tarfile.open(tarpath)
        tar_ref.extractall(volume)
        tar_ref.close()

users = getusers()
setIP(users)
setHOME(users)
# setHOME(["guest102"])
