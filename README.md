# Lab304-server

## versions
name: linnil1/serverbox
tag:
* Base1.0: cuda9.1 + cudnn7 (20180715)
* Learn1.0: Base1.0 + TF1.9.0 + torch0.4
* Show1.0: Learn1.0 + User
* Show1.1: Learn1.0 + User (Update)
* Show2.0: Learn1.0 + User + VNC + gnome + pcmanfm

* Base3.0: cuda9.0  + cudnn7 (20180731)
* Show3.0: Base3.0 + User + VNC + gnome + pcmanfm
* Learn3.0: Show3.0 + TF1.9.0 + torch0.4 + keras2.2.2
* Learn3.1: Show3.0 + Caffe2(2018/9/8) 
* Show3.1: Base3.0 + ssh

## Build

If you want to build `Base1.0.df`

`docker build -f Base1.0.df . -t linnil1/serverbox:base1.0`

and so on.

## Some building Details

### allhome
In dockercompose file,

this is use for make a safer place for API connected from outside.

### Show2.0
This is base on https://www.linode.com/docs/applications/remote-desktop/install-vnc-on-ubuntu-16-04/

You can git clone https://github.com/novnc/noVNC to test
`./utils/launch.sh --vnc your-container-ip:5900`

Use pcmanfm
https://wiki.lxde.org/en/PCManFM

### Modify password with sha512 hash code
`$ perl -p -i -e 's/(linnil1:).*?(:.+)/\1xxx\2/g' /etc/shadow`

if you use in python, take care for special character '/$.'

``` python
pwd = pw.replace(r'/', r'\/').replace('$',r'\$')                                                                                                                   
container.exec_run(r'perl -p -i -e "s/(ubuntu:).*?(:.+)/\1' + pwd + r'\2/g" /etc/shadow')
````

### Show3.0
tenserflow 1.9 build at numpy 1.14
so you need to downgrade numpy.

### docker compose
When you resume all container, please use `docker-compose up --no-recreate -d`.

## Run with docker-compose

1. modify user in dockercompose
```
  guest102: # Change user name
    devices:
    - /dev/nvidia0 # first GPU
    - /dev/nvidia1 # second GPU
    - /dev/nvidiactl
    - /dev/nvidia-uvm
    - /dev/nvidia-uvm-tools
    image: linnil1/serverbox:show2.0
    networks:
      mynet:
        ipv4_address: 172.18.0.1 # you can use static ip or not
    volumes:
    - ./default_home/guest102:/home/ubuntu # Chnage name
    - nvidia_driver_375.66:/usr/local/nvidia:ro # GPU driver version
```

2. copy defalut home to user's home
remember to create `.vnc/passwd` which is vnc password
use `vnc4passwd` to generate
```
cp ./default_home/guest101 -r /home/guest102
chown -R 1000:1000 /home/guest102
mkdir -p ssh_redir/guest102
echo "user@ip" > ssh_redir/guest102/sshpiper_upstream
```

2.1 
or just call `./manage.sh` to build up all things

3. Add user data into database in LabServer
```
your.domain.name/adminpage
add user and add container
```
or see README in LabServer

4. Ask newuser to change password

5. Ask newuser to login NextCloud

6. Add data sharing of nextcloud
* goto `Settings > External storages` to add permission
* goto `Users` to add newuser to new group

7. If you somehow stop containers, 
please use `docker-compose up --no-recreate -d` to continue

## attension
You need to set password of ubuntu by any methods.
like `docker exec -it 2018summer_guest101_1 passwd ubuntu`

