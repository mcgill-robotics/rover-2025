# What Is Gst-Python


## Install Dependencies

```sh
sudo apt-get install gstreamer-1.0
sudo apt-get install gstreamer1.0-dev

sudo apt-get install python3.6 python3.6-dev python-dev python3-dev
sudo apt-get install python3-pip python-dev 
sudo apt-get install python3.6-venv

sudo apt-get install git autoconf automake libtool

sudo apt-get install python3-gi python-gst-1.0 
sudo apt-get install libgirepository1.0-dev
sudo apt-get install libcairo2-dev gir1.2-gstreamer-1.0
```


## Install Gst-Python

```sh
git clone https://github.com/GStreamer/gst-python.git
cd gst-python

GSTREAMER_VERSION=$(gst-launch-1.0 --version | grep version | tr -s ' ' '\n' | tail -1)
git checkout $GSTREAMER_VERSION

PYTHON=/usr/bin/python3
LIBPYTHON=$($PYTHON -c 'from distutils import sysconfig; print(sysconfig.get_config_var("LDLIBRARY"))')
LIBPYTHONPATH=$(dirname $(ldconfig -p | grep -w $LIBPYTHON | head -1 | tr ' ' '\n' | grep /))
PREFIX=$(dirname $(dirname $(which python))) # in jetson nano, `PREFIX=~/.local` to use local site-packages,

./autogen.sh --disable-gtk-doc --noconfigure
./configure --with-libpython-dir=$LIBPYTHONPATH --prefix $PREFIX

make
make install
```

## Check Gst-Python API

```sh
python -c "import gi; gi.require_version('Gst', '1.0'); \
gi.require_version('GstApp', '1.0'); \
gi.require_version('GstVideo', '1.0'); \
gi.require_version('GstBase', '1.0')"
```

## Check Gst-Python Plugin

```sh
python3 gstreamer_empty_plugin_test_case.py
```

## Install gstreamer-python package(Optional)

```sh
git clone git@github.com:jackersson/gstreamer-python.git
cd gstreamer-python
### to skip ./build-gst-python.sh
pip install . -v --install-option "build_py" --install-option "--skip-gst-python"
```


[How to install Gstreamer Python Bindings](http://lifestyletransfer.com/how-to-install-gstreamer-python-bindings/)