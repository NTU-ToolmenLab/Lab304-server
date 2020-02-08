# Lab304-server

This repo is only to store dockerfile and its environment.

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
* ml2018fall: For 2018ML course
* Base3.2: python2 + cuda9.0  + cudnn7 (20180921)
* Show3.2: Base3.2 + User + VNC + gnome + pcmanfm
* Learn3.2: Show3.2 + Caffe2(2018/9/21)

* Base3.3: cuda10.0 + cudnn7 + user(20190213)
* Show3.3: Base3.3 + VNC + gnome + pcmanfm + firefox + pycharm(20190214)
* Learn3.3: Show3.3 + python3 + TF1.13-rc1 + Keras2.2.4 + Torch(20190214)
* Learn3.4: Show3.3 + python2 + caffe2 + detectron(20190214)
* Learn3.5: Show3.3 + python3 + TF1.13.1 + Keras2.2.4 + Torch(20190214) + fastai_1.0.45 + sublime

* Base3.6: cuda10.0 + cudnn7 + user(20190426)
* Show3.6: Base3.6 + VNC + gnome-terminal + pcmanfm + firefox + pycharm + sublime + jupyter(20190426)
* Learn3.6: Show3.6 + python3 + TF1.13.1 + Keras2.2.4 + Torch(20190426) + fastai1.0.51

* Base3.7: cuda10.2 + cudnn7 + user(20190918)
* Show3.7: Base3.7 + VNC + gnome-terminal + pcmanfm + firefox + pycharm + sublime + jupyter
* Learn3.7: Show3.7 + python3 + TF2.0.0rc1 + Keras2.2.5 + Torch1.2.0 + fastai1.0.57
* Learn3.8: Show3.7 + python3 + TF2.1.0 + Keras2.3.1 + Torch1.4.0 + fastai1.0.60

## Build

If you want to build `Base1.0.df`

`docker build -f Base1.0.df -t linnil1/serverbox:base1.0 .`

and so on.

## Some building Details

### Show2.0
This is base on https://www.linode.com/docs/applications/remote-desktop/install-vnc-on-ubuntu-16-04/

You can git clone https://github.com/novnc/noVNC to test
`./utils/launch.sh --vnc your-container-ip:5900`

Use pcmanfm
https://wiki.lxde.org/en/PCManFM


### Show3.0
tenserflow 1.9 build at numpy 1.14
so you need to downgrade numpy.


## Modify password with sha512 hash code
`docker exec -it 2018summer_guest101_1 passwd ubuntu`

or

`$ perl -p -i -e 's/(linnil1:).*?(:.+)/\1xxx\2/g' /etc/shadow`

if you use in python, take care for special character '/$.'

``` python
pwd = pw.replace(r'/', r'\/').replace('$',r'\$')                                                                                                                   
container.exec_run(r'perl -p -i -e "s/(ubuntu:).*?(:.+)/\1' + pwd + r'\2/g" /etc/shadow')
```

## HOME environment
I put it at `default_home`.

You can package it:
`./pack.sh`
