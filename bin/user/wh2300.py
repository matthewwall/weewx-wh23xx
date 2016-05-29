#!/usr/bin/env python
# Copyright 2016 Matthew Wall, all rights reserved
"""
Collect data from Fine Offset WH2300 stations.

Based on the protocol specified in "TP2700 EEPROM data structure" V1.0 with
serial number FOS-ENG-022-A for model WH2300, and "TP2700 PC Protocol".

The station has 3552 records.  Each record is 18 bytes.  The timestamp for each
record is stored separately from the record.

0x0000 to 0x0258 : system, max/min, alarms, etc

0x0259 to 0x02c7 : 110 bytes : page flag structure
each byte is 0x01 to 0x20 or 0xff, indicating how many records in the page

0x02c8 to 0x063f : 110 8-byte segments : table structure (timestamps)
each 8-byte segment is a timestamp
  year
  month
  day
  hour
  minute
  second
  interval (lsb)
  interval (msb)

0x0640 to 0xffff : records
each record is 18 bytes
"""

from __future__ import with_statement
import syslog
import time
import usb

import weewx.drivers
from weewx.wxformulas import calculate_rain

DRIVER_NAME = 'WH2300'
DRIVER_VERSION = '0.1'

def loader(config_dict, _):
    return WH2300Driver(**config_dict[DRIVER_NAME])

def confeditor_loader():
    return WH2300ConfigurationEditor()


def logmsg(level, msg):
    syslog.syslog(level, 'wh2300: %s' % msg)

def logdbg(msg):
    logmsg(syslog.LOG_DEBUG, msg)

def loginf(msg):
    logmsg(syslog.LOG_INFO, msg)

def logerr(msg):
    logmsg(syslog.LOG_ERR, msg)


class WH2300ConfigurationEditor(weewx.drivers.AbstractConfEditor):
    @property
    def default_stanza(self):
        return """
[WH2300]
    # This section is for Fine Offset WH2300 stations

    # The model name such as Tycon, or TP2700
    model = Tycon TP2700

    # The driver to use
    driver = user.wh2300
"""


class WH2300Driver(weewx.drivers.AbstractDevice):
    def __init__(self, **stn_dict):
        loginf('driver version is %s' % DRIVER_VERSION)
        self._model = stn_dict.get('model', 'Tycon TP2700')
        self._poll_interval = int(stn_dict.get('poll_interval', 15))
        loginf('poll interval is %s' % poll_interval)
        max_tries = int(stn_dict.get('max_tries', 10))
        retry_wait = int(stn_dict.get('retry_wait', 10))
        self.last_rain = None
        self._station = WH2300Station(max_tries, retry_wait)
        self._station.open()

    def closePort(self):
        self._station.close()

    def hardware_name(self):
        return self._model

    def genLoopPackets(self):
        while True:
            raw = self._station.get_current()
            if raw:
                logdbg("raw data: %s" % raw)
                parsed = WH2300Station.parse_raw(raw)
                logdbg("parsed data: %s" % parsed)
                packet = self._data_to_packet(parsed)
                yield packet
            time.sleep(self._poll_interval)

    def _data_to_packet(self, data):
        pkt = {'dateTime': int(time.time() + 0.5), 'usUnits': weewx.US}
        pkt['windDir'] = data.get('wind_dir')
        pkg['windSpeed'] = data.get('wind_speed')
        pkg['windGust'] = data.get('gust_speed')
        pkg['inHumidity'] = data.get('humidity_in')
        pkg['outHumidity'] = data.get('humidity_out')
        pkg['inTemp'] = data.get('temperature_in')
        pkg['outTemp'] = data.get('temperature_out')
        pkg['pressure'] = data.get('pressure')
        pkg['light'] = data.get('light')
        pkg['UV'] = data.get('uv')
        pkt['rain'] = calculate_rain(data['rain_total'], self.last_rain)
        pkt['rxCheckPercent'] = (1 - pkt['no_sensors']) * 100
        self.last_rain = data['rain_total']
        return pkt


class WH2300Station(object):
    def __init__(self, vendor_id=0x10c4, product_id=0x8468, interface=0,
                 max_tries=10, retry_wait=5):
        self.vendor_id = vendor_id
        self.product_id = product_id
        self.iface = interface
        self.max_tries = max_tries
        self.retry_wait = retry_wait
        self.devh = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, _, value, traceback):
        self.close()

    def open(self):
        dev = self._find_dev(self.vendor_id, self.product_id)
        if not dev:
            logcrt("Cannot find USB device with VendorID=0x%04x ProductID=0x%04x" % (self.vendor_id, self.product_id))
            raise weewx.WeeWxIOError('Unable to find station on USB')

        self.devh = dev.open()
        if not self.devh:
            raise weewx.WeeWxIOError('Open USB device failed')

        self.devh.reset()

        # be sure kernel does not claim the interface on linux systems
        try:
            self.devh.detachKernelDriver(self.iface)
        except (AttributeError, usb.USBError):
            pass

        # attempt to claim the interface
        try:
            self.devh.claimInterface(self.iface)
            self.devh.setAltInterface(self.iface)
        except usb.USBError, e:
            self.close()
            logcrt("Unable to claim USB interface %s: %s" % (self.iface, e))
            raise weewx.WeeWxIOError(e)

    def close(self):
        if self.devh:
            try:
                self.devh.releaseInterface()
            except (ValueError, usb.USBError), e:
                logerr("release interface failed: %s" % e)
            self.devh = None

    @staticmethod
    def _find_dev(vendor_id, product_id):
        """Find the vendor and product ID on the USB."""
        for bus in usb.busses():
            for dev in bus.devices:
                if dev.idVendor == vendor_id and dev.idProduct == product_id:
                    loginf('Found device on USB bus=%s device=%s' %
                           (bus.dirname, dev.filename))
                    return dev
        return None

    def _raw_read(self, addr):
        reqbuf = [0x05, 0xAF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        reqbuf[4] = addr / 0x10000
        reqbuf[3] = (addr - (reqbuf[4] * 0x10000)) / 0x100
        reqbuf[2] = addr - (reqbuf[4] * 0x10000) - (reqbuf[3] * 0x100)
        reqbuf[5] = (reqbuf[1] ^ reqbuf[2] ^ reqbuf[3] ^ reqbuf[4])
        ret = self.devh.controlMsg(requestType=0x21,
                                   request=usb.REQ_SET_CONFIGURATION,
                                   value=0x0200,
                                   index=0x0000,
                                   buffer=reqbuf,
                                   timeout=self.TIMEOUT)
        if ret != 8:
            raise BadRead('Unexpected response to data request: %s != 8' % ret)

        time.sleep(0.1)  # te923tool is 0.3
        start_ts = time.time()
        rbuf = []
        while time.time() - start_ts < 3:
            try:
                buf = self.devh.interruptRead(
                    self.ENDPOINT_IN, self.READ_LENGTH, self.TIMEOUT)
                if buf:
                    nbytes = buf[0]
                    if nbytes > 7 or nbytes > len(buf) - 1:
                        raise BadRead("Bogus length during read: %d" % nbytes)
                    rbuf.extend(buf[1:1 + nbytes])
                if len(rbuf) >= 34:
                    break
            except usb.USBError, e:
                errmsg = repr(e)
                if not ('No data available' in errmsg or 'No error' in errmsg):
                    raise weewx.WeeWxIOError(e)
            time.sleep(0.009) # te923tool is 0.15
        else:
            logdbg("timeout while reading: ignoring bytes: %s" % _fmt(rbuf))
            raise BadRead("Timeout after %d bytes" % len(rbuf))

        return rbuf

    def get_data(self, addr, length):
        logdbg("get %s bytes from address 0x%06x" % (length, addr))
        buf = self.raw_read(addr)
        logdbg("station said: %s" % ' '.join(["%0.2X" % ord(c) for c in buf]))
        return buf

    def get_data_with_retry(self, addr, length=32):
        for ntries in range(0, self.max_tries):
            try:
                return self.get_data(addr, length)
            except (usb.USBError, weewx.WeeWxIOError), e:
                loginf("Failed attempt %d of %d to get readings: %s" %
                       (ntries + 1, self.max_tries, e))
                time.sleep(self.retry_wait)
        else:
            msg = "Max retries (%d) exceeded for address 0x%06x" % (
                self.max_tries, addr)
            logerr(msg)
            raise weewx.RetriesExceeded(msg)

    def get_current(self):
        # equivalent of the READ_RECORD operation
        # FIXME: calculate address for next read
        return self.get_data_with_retry(addr)

    def set_time(self):
        # FIXME: implement set_time
        raise NotImplementedError("set_time is not implemented")

    def clear_max_min(self):
        # FIXME: implement clear_max_min
        raise NotImplementedError("clear_max_min is not implemented")

    def clear_history(self):
        # FIXME: implement clear history
        raise NotImplementedError("clear_history is not implemented")

    def get_station_info(self):
        buf = self.get_data_with_retry(0x0000, 0x0201)
        data = dict()
        data['eeprom'] = "0x%02x 0x%02x" % (buf[0], buf[1])
        data['model'] = "0x%02x 0x%02x" % (buf[2], buf[3])
        data['version'] = "0x%02x" % buf[4]
        data['id'] = "0x%02x 0x%02x 0x%02x 0x%02x" % (buf[5], buf[6], buf[7], buf[8])
        data['rain_season'] = buf[0x18] # month 1-12
        data['lcd_contrast'] = buf[0x1b] # 0x17-0x1f
        data['timezone'] = buf[0x1c] # -12-12
        data['interval'] = buf[0x1a] * 256 + buf[0x19] # seconds 8-14400 (240m)
        data['latitude'] = buf[0x1e] * 256 + buf[0x1d]
        data['longitude'] = buf[0x20] * 256 + buf[0x1f]
        data['weather'] = buf[0x21]
        data['storm'] = buf[0x22]
        return data

    @staticmethod
    def parse_raw(raw):
        # each record is 18 bytes
        # FIXME: some of the values for invalid data do not make sense:
        #  light: 0xfff specified, using 0xffffff
        #  uv: 0xff specified, using 0xffff
        #  wind_dir: 0x1f specified, using 0x1ff
        data = dict()
        if not raw:
            logdbg("empty raw data")
            return data
        if len(raw) != 18:
            logdbg("wrong number of bytes in raw data: %s" % len(raw))
            return data
        x = ((raw[0] & 0x01) << 8) + raw[1]
        data['wind_dir'] = None if x == 0x1ff else x # compass degree
        x = ((raw[0] & 0x02) / 0x02 << 8) + raw[2]
        data['wind_speed'] = None if x == 0x1ff else x / 10.0 # m/s
        x = ((raw[0] & 0x04) / 0x04 << 8) + raw[3]
        data['gust_speed'] = None if x == 0x1ff else x / 10.0 # m/s
        data['rain_total'] = (((raw[0] & 0x08) / 0x08 << 16) + (raw[5] << 8) + raw[4]) * 0.1 # 0.0-9999.9 mm
        data['rain_overflow'] = (raw[0] & 0x10) / 0x10 # bit 4
        data['no_sensors'] = (raw[0] & 0x80) / 0x80 # bit 7
        data['humidity_in'] = None if raw[6] == 0xff else raw[6]
        data['humidity_out'] = None if raw[7] == 0xff else raw[7]
        x = ((raw[9] & 0x0f) << 8) + raw[8]
        data['temperature_in'] = None if x == 0xfff else x / 10.0 - 40.0 # C
        x = ((raw[9] & 0xf0) << 8) + raw[10]
        data['temperature_out'] = None if x == 0xfff else x / 10.0 - 40.0 # C
        x = (raw[11] << 8) + raw[12]
        data['pressure'] = None if x == 0xffff else x / 10.0 # hpa
        x = (raw[15] << 16) + (raw[14] << 8) + raw[13]
        data['light'] = None if x == 0xffffff else x / 10.0 # 0.0-300000.0 lux
        x = (raw[17] << 8) + raw[16]
        data['uv'] = None if x == 0xffff else x # 0-20000 uW/m^2
        return data


# define a main entry point for basic testing of the station.  invoke this as
# follows from the weewx root dir:
#
# PYTHONPATH=bin python bin/user/wh2300.py

if __name__ == '__main__':
    import optparse

    usage = """%prog [options] [--debug] [--help]"""

    syslog.openlog('wh2300', syslog.LOG_PID | syslog.LOG_CONS)
    syslog.setlogmask(syslog.LOG_UPTO(syslog.LOG_INFO))
    parser = optparse.OptionParser(usage=usage)
    parser.add_option('--version', dest='version', action='store_true',
                      help='display driver version')
    parser.add_option('--debug', dest='debug', action='store_true',
                      help='display diagnostic information while running')
    (options, args) = parser.parse_args()

    if options.version:
        print "driver version %s" % DRIVER_VERSION
        exit(1)

    if options.debug:
        syslog.setlogmask(syslog.LOG_UPTO(syslog.LOG_DEBUG))

    with WH2300Station() as s:
        print s.get_station_info()
        while True:
            raw = s.get_current()
            print "raw:", raw
            print "parsed:", WH2300Station.parse_raw(raw)
            time.sleep(5)
