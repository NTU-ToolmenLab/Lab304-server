# Lab304-server

## versions
name: linnil1/serverbox
tag:
* Base1.0: cuda9.1 cudnn7 20180715
* Learn1.0: Base1.0 TF1.9.0  torch0.4
* Show1.0: Learn1.0 + User
* Show2.0: Learn1.0 + User + VNC + gnome + pcmanfm

## Build

If you want to build `Base1.0.df`

```
cp Base1.0.df Dockerfile
docker build . -t linnil1/serverbox:base1.0
```

and so on.

## Some building Details

### Show2.0
This is base on https://www.linode.com/docs/applications/remote-desktop/install-vnc-on-ubuntu-16-04/

You can git clone https://github.com/novnc/noVNC to test
`./utils/launch.sh --vnc your-container-ip:5900`

Use pcmanfm
https://wiki.lxde.org/en/PCManFM
