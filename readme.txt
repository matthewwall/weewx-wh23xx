weewx-wh23xx

This is a weewx driver for Fine Offset WH23xx hardware.  These stations are
commonly available as Tycon TP2700 weather stations.

Installation

0) install weewx, select 'Simulator' driver (see the weewx user guide)

dpkg -i weewx_x.y.z-r.deb

1) download the driver

wget -O weewx-wh23xx.zip https://github.com/matthewwall/weewx-wh23xx/archive/master.zip

2) install the driver

wee_extension --install weewx-wh23xx.zip

3) configure the driver

wee_config --reconfigure

4) start weewx

sudo /etc/init.d/weewx start
