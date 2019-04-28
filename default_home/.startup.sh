#setting up vncserver
# if [[ -e /tmp/.X0-lock && -e /tmp/.X11-unix/X0 ]]; then
# 	rm -f /tmp/.X0-lock /tmp/.X11-unix/X0
# fi

rm -rf /tmp/.X0-lock /tmp/.X11-unix/X0
# fi
# if [[ -e /home/ubuntu/.vnc/*.pid && -e /home/ubuntu/.vnc/*.log ]]; then
# 	rm -f /home/ubuntu/.vnc/*.pid /home/ubuntu/.vnc/*.log
# fi

# run 
mkdir -p /home/tmp
chown 1000:1000 /home/tmp
mv /home/ubuntu/.bashrc.default /etc/bash.bashrc
service ssh start
ldconfig

hn=$(cat /etc/hostname)
if [ -x "$(command -v jupyterhub)" ]; then
  mkdir -p .jupyter/$hn
  chown ubuntu:ubuntu .jupyter/$hn
  su ubuntu sh -c "cd .jupyter/$hn && jupyterhub --url http://0.0.0.0:8000/jupyter/$hn --Spawner.notebook_dir=/home &"
fi

su ubuntu sh -c 'vncserver :0 -geometry 1280x720 && tail -f /home/ubuntu/.vnc/*.log'
