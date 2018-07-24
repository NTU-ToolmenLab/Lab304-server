docker run -it --rm -v /home:/home -v $(pwd):/app python:3.7-slim /bin/bash -c "
    pip3 install pyyaml;
    cd /app
    python3 manage.py"

