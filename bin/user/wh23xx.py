#!/usr/bin/env python
# Copyright 2016 Matthew Wall, all rights reserved
#
# Thanks to Lloyd Kinsella

# FIXME: this should be called wh23xx

"""
Collect data from Fine Offset WH23xx stations, including:

  WH2300 (with RCC)
  WH2301 (no RCC)
  WH4000
  Tycon TP2700

Based on the protocol specified in "TP2700 EEPROM data structure" V1.0 with
serial number FOS-ENG-022-A for model WH2300, and "TP2700 PC Protocol".

The console works with the all-in-one instrument cluster, or the separate
instruments.

The station includes a light sensor.  The light sensor output is in lux, but
the station has a multiplicative lux-to-radiation conversion using a constant
in the station, which is factory set to 126.7.  The station reports light in
lux, radiation in micro-W per square meter (labelled as UV), and UV index on
a scale of 1-15 (labelled as UVI).

The data logger in the console retains data through a power cycle.

Current data include the following:

  in temperature
  out temperature
  dewpoint
  windchill
  heatindex
  in humidity
  out humidity
  abs pressure
  rel pressure
  wind direction
  wind speed
  gust speed
  rain event
  rain rate
  rain hour
  rain day
  rain week
  rain month
  rain year
  rain total
  light
  uv (radiation)
  uv index

Historical records include the following:

  wind direction
  wind speed
  gust speed
  rain total
  in humidity
  out humidity
  in temperature
  out temperature
  pressure
  light
  UV (radiation)

The station has 3552 records.  Each record is 18 bytes.  The timestamp for each
record is stored separately from the record.

Memory Map

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

Decoding current weather data

Temperature is value + 40C
Temperature, pressure, wind speed, rainfall, light are value / 10.0
Data are stored as hi byte first then lo byte
For 1 byte word, 0xff indicates invalid
For 2 byte word, 0xffff indicates invalid
For 4 byte word, 0xffffffff indicates invalid

Commands

TIME_SYNC         0x01    [0x02 0x09]
READ_EEPROM       0x02    [0x02 0x05]
WRITE_EEPROM      0x03
READ_RECORD       0x04    [0x02 0x02]
READ_MAX          0x05    [0x02 0x02]
READ_MIN          0x06    [0x02 0x02]
READ_MAX_DAY      0x07    [0x02 0x02]
READ_MIN_DAY      0x08    [0x02 0x02]
CLEAR_MAX_MIN_DAY 0x09
PARAM_CHANGED     0x0a
CLEAR_HISTORY     0x0b
READ_PARAM        0x0c

CMD_RESULT        0xf0

The checksum in each message is simply the lo byte of the sum of the bytes in
each message.

Time Sync (9 bytes)

  TIME_SYNC    1
  year         1 0x00-0x99 -> 2000 -> 2099
  month        1
  day          1
  hour         1
  minute       1
  second       1
  1/125 second 1 0x00-0x07
  checksum     1

Read EEPROM (5 bytes)

  READ_EEPROM  1
  address      2 lo hi
  size         1 1-56
  checksum     1

  READ_EEPROM  1
  size         1 1-56
  data         x
  checksum     1

Write EEPROM (x bytes)

  WRITE_EEPROM 1
  address      2 lo hi
  size         1 1-12
  data         x
  checksum     1

Read Record (2 bytes)

  READ_RECORD  1 read the current value
  checksum     1

  READ_RECORD  1
  size         1
  data         x
  checksum     1

Param Changed (4 bytes)

  PARAM_CHANGED 1
  parameter     2
  checksum      1

  parameter values:
    0x0001 alarm tag or value changed
    0x0002 latitude/longitude/timezone changed
    0x0004 parameters have changed
    0x0008 max/min value has changed
    0x0010 history has been emptied

Read Parameter (2 bytes)

  READ_PARAM 1
  checksum   1

  READ_PARAM 1
  size       1
  data       1
  checksum   1

  data values:
    bit 0: 01: UART 10: ASK 00: FSK
    bit 1: 01: UART function 10: ASK function 00: FSK function
    bit 2: 1: RCC 0: no RCC

Command Result

  CMD_RESULT 1
  CMD        1
  result     2
  checksum   1

  return values:
    1 RT_SUCCESS
    2 RT_INVALID_USER_PASS
    3 RT_INVALID_ID
    4 RT_INVALID_CRC
    5 RT_BUSY
    6 RT_TOO_SIZE
    7 RT_ERROR
    8 RT_UNKNOWN_CMD
    9 RT_INVALID_PARAM
"""

from __future__ import with_statement
import syslog
import time
import usb

import weewx.drivers
from weeutil.weeutil import timestamp_to_string
from weewx.wxformulas import calculate_rain

DRIVER_NAME = 'WH23xx'
DRIVER_VERSION = '0.6'

def loader(config_dict, _):
    return WH23xxDriver(**config_dict[DRIVER_NAME])

def confeditor_loader():
    return WH23xxConfigurationEditor()


def logmsg(level, msg):
    syslog.syslog(level, 'wh23xx: %s' % msg)

def logdbg(msg):
    logmsg(syslog.LOG_DEBUG, msg)

def loginf(msg):
    logmsg(syslog.LOG_INFO, msg)

def logerr(msg):
    logmsg(syslog.LOG_ERR, msg)


#' '.join(["%0.2X" % ord(c) for c in buf]))
def _fmt(buf):
    if buf:
        return "%s (len=%s)" % (' '.join(["%02x" % x for x in buf]), len(buf))
    return ''

def _calc_checksum(a):
    s = 0
    for x in a:
        s += x
    return s & 0xff

def _get_bit(x, bit):
    return 1 if ((x & (1 << bit)) == (1 << bit)) else 0

def _decode_bytes(buf, idx, nbytes, func):
    # if all bytes are 0xff, the value is not valid...
    for j in range(nbytes):
        if buf[idx + j] != 0xff:
            break
    else:
        return None
    # ...otherwise, calculate a value from the bytes, MSB first
    x = 0
    for j in range(nbytes):
        x += buf[idx + j] << ((nbytes - j - 1) * 8)
    return func(x)

def _signed(x):
    v = x & 0xf
    if x & 0xf0 == 0xf0:
        v *= -1
    return v


class WH23xxConfigurationEditor(weewx.drivers.AbstractConfEditor):
    @property
    def default_stanza(self):
        return """
[WH23xx]
    # This section is for Fine Offset WH23xx stations

    # The model name such as Tycon, or TP2700
    model = Tycon TP2700

    # The driver to use
    driver = user.wh23xx
"""


class WH23xxDriver(weewx.drivers.AbstractDevice):
    def __init__(self, **stn_dict):
        loginf('driver version is %s' % DRIVER_VERSION)
        self._model = stn_dict.get('model', 'Tycon TP2700')
        self._poll_interval = int(stn_dict.get('poll_interval', 15))
        loginf('poll interval is %s' % self._poll_interval)
        self.max_tries = int(stn_dict.get('max_tries', 5))
        self.retry_wait = int(stn_dict.get('retry_wait', 10))
        self.last_rain = None
        self._station = WH23xxStation()
        self._station.open()

    def closePort(self):
        self._station.close()

    def hardware_name(self):
        return self._model

    def genLoopPackets(self):
        while True:
            raw = self._get_current()
            logdbg("raw data: %s" % raw)
            if raw:
                decoded = WH23xxStation.decode_weather_data(raw)
                logdbg("decoded data: %s" % decoded)
                packet = self._data_to_packet(decoded)
                logdbg("packet: %s" % packet)
                yield packet
            time.sleep(self._poll_interval)

    def _get_current(self):
        ntries = 0
        while ntries < self.max_tries:
            ntries += 1
            try:
                return self._station.get_current()
            except usb.USBError, e:
                errmsg = repr(e)
                if 'No data available' in errmsg or 'No error' in errmsg:
                    ntries -= 1
                else:
                    logerr("read: failed attempt %d of %d: %s" %
                           (ntries, self.max_tries, e))
                time.sleep(self.retry_wait)
        msg = "read failed: max retries (%d) exceeded" % self.max_tries
        logerr(msg)
        raise weewx.RetriesExceeded(msg)

    def _data_to_packet(self, data):
        # convert from the dictionary-of-dictionaries to a simple dictionary
        # of observation values.
        pkt = {'dateTime': int(time.time() + 0.5), 'usUnits': weewx.METRICWX}
        pkt['windDir'] = data.get('wind_dir', {}).get('value')
        pkt['windSpeed'] = data.get('wind_speed', {}).get('value')
        pkt['windGust'] = data.get('gust_speed', {}).get('value')
        pkt['inHumidity'] = data.get('in_humidity', {}).get('value')
        pkt['outHumidity'] = data.get('out_humidity', {}).get('value')
        pkt['inTemp'] = data.get('in_temp', {}).get('value')
        pkt['outTemp'] = data.get('out_temp', {}).get('value')
        pkt['pressure'] = data.get('abs_baro', {}).get('value')
        pkt['light'] = data.get('light', {}).get('value')
        pkt['radiation'] = data.get('uv', {}).get('value')
        pkt['UV'] = data.get('uvi', {}).get('value')
        rain_total = data.get('rain_totals', {}).get('value')
        pkt['rain'] = calculate_rain(rain_total, self.last_rain)
        self.last_rain = rain_total
        # FIXME: get measure of connectivity to sensors
        return pkt


class WH23xxStation(object):
    # usb values obtained from 'sudo lsusb -v'
    USB_ENDPOINT_IN = 0x82
    USB_ENDPOINT_OUT = 0x02
    USB_PACKET_SIZE = 0x40 # 64 bytes

    # from the vendor documentation
    TIME_SYNC = 0x01
    READ_EEPROM = 0x02
    WRITE_EEPROM = 0x03
    READ_RECORD = 0x04
    READ_MAX = 0x05
    READ_MIN = 0x06
    READ_MAX_DAY = 0x07
    READ_MIN_DAY = 0x08
    CLEAR_MAX_MIN_DAY = 0x09
    PARAM_CHANGED = 0x0a
    CLEAR_HISTORY = 0x0b
    READ_PARAM = 0x0c

    CMD_RESULT = 0xf0

    PARAM_ITEM_ALARM = 0x0001
    PARAM_ITEM_TIMEZONE = 0x0002
    PARAM_ITEM_PARAM = 0x0004
    PARAM_ITEM_MAX_MIN = 0x0008
    PARAM_ITEM_HISTORY = 0x0010

    RT_SUCCESS = 0x0000
    RT_INVALID_USER_PASS = 0x0001
    RT_INVALID_ID = 0x0002
    RT_INVALID_CRC = 0x0004
    RT_BUSY = 0x0008
    RT_TOO_SIZE = 0x0010
    RT_ERROR = 0x0020
    RT_UNKNOWN_CMD = 0x0040
    RT_INVALID_PARAM = 0x0080

    INVALID_DATA_8 = 0xff
    INVALID_DATA_16 = 0xffff
    INVALID_DATA_32 = 0xffffffff

    ITEM_INTEMP = 0x01 # C
    ITEM_OUTTEMP = 0x02 # C
    ITEM_DEWPOINT = 0x03 # C
    ITEM_WINDCHILL = 0x04 # C
    ITEM_HEATINDEX = 0x05 # C
    ITEM_INHUMI = 0x06 # %
    ITEM_OUTHUMI = 0x07 # %
    ITEM_ABSBARO = 0x08 # mbar
    ITEM_RELBARO = 0x09 # mbar
    ITEM_WINDDIRECTION = 0x0a # degree
    ITEM_WINDSPEED = 0x0b # m/s
    ITEM_GUSTSPEED = 0x0c # m/s
    ITEM_RAINEVENT = 0x0d # mm
    ITEM_RAINRATE = 0x0e # mm/h
    ITEM_RAINHOUR = 0x0f # mm
    ITEM_RAINDAY = 0x10 # mm
    ITEM_RAINWEEK = 0x11 # mm
    ITEM_RAINMONTH = 0x12 # mm
    ITEM_RAINYEAR = 0x13 # mm
    ITEM_RAINTOTALS = 0x14 # mm
    ITEM_LIGHT = 0x15 # lux
    ITEM_UV = 0x16 # uW/m^2
    ITEM_UVI = 0x17 # 0-15 index

    ITEM_TIME = 0x40
    ITEM_DATE = 0x80

    def __init__(self):
        self.vendor_id = 0x10c4
        self.product_id = 0x8468
        self.iface = 0
        self.timeout = 1000
        self.devh = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, _, value, traceback):
        self.close()

    def open(self):
        dev = self._find_dev(self.vendor_id, self.product_id)
        if not dev:
            logerr("Cannot find USB device with VendorID=0x%04x ProductID=0x%04x" % (self.vendor_id, self.product_id))
            raise weewx.WeeWxIOError('Unable to find station on USB')

        self.devh = dev.open()
        if not self.devh:
            raise weewx.WeeWxIOError('Open USB device failed')

        # be sure kernel does not claim the interface on linux systems
        try:
            self.devh.detachKernelDriver(self.iface)
        except (AttributeError, usb.USBError):
            pass

        # attempt to unwedge the device
        self._reset()

        # attempt to claim the interface
        try:
            self.devh.claimInterface(self.iface)
            self.devh.setAltInterface(self.iface)
        except usb.USBError, e:
            self.close()
            logerr("Unable to claim USB interface %s: %s" % (self.iface, e))
            raise weewx.WeeWxIOError(e)

    def close(self):
        if self.devh:
            try:
                self.devh.releaseInterface()
            except (ValueError, usb.USBError), e:
                logerr("release interface failed: %s" % e)
            self.devh = None

    def _reset(self):
        # use a usb reset to restore communication with the station.
        # specific cases include when you do an interrupt write with bogus
        # data.  use a reset to bring the station back to responsiveness.
        # unfortunately it is not immediate.  sometimes it takes one reset.
        # sometimes it takes multiple resets.
        for x in range(5):
            try:
                self.devh.reset()
                break
            except usb.USBError:
                time.sleep(2)

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

    def _write(self, label, buf):
        logdbg("%s: cmdbuf: %s" % (label, _fmt(buf)))
        cnt = self.devh.interruptWrite(self.USB_ENDPOINT_OUT, buf, self.timeout)
        if cnt != len(buf):
            raise weewx.WeeWxIOError('%s: bad write length=%s for command %s' %
                                     (label, cnt, _fmt(buf)))

    def _time_sync(self, ts):
        logdbg("time sync to %s (%s)" % (ts, timestamp_to_string(ts)))
        t = time.localtime(ts)
        cmd = [WH23xxStation.TIME_SYNC,
               t.tm_year - 2000, t.tm_mon, t.tm_mday,
               t.tm_hour, t.tm_min, t.tm_sec, 0]
        chksum = _calc_checksum(cmd)
        buf = [0x02, 0x09]
        buf.extend(cmd)
        buf.append(chksum)
        self._write("time_sync", buf)

    def _read_eeprom(self, addr, size):
        # initiate a read by sending the READ_EEPROM command.
        addr_lo = addr & 0xff
        addr_hi = (addr / 256) & 0xff
        cmd = [WH23xxStation.READ_EEPROM, addr_lo, addr_hi, size]
        chksum = _calc_checksum(cmd)
        buf = [0x02, 0x05]
        buf.extend(cmd)
        buf.append(chksum)
        self._write("read_eeprom", buf)

        # now do the actual read.
        buf = self.devh.interruptRead(
            self.USB_ENDPOINT_IN,
            self.USB_PACKET_SIZE,
            self.timeout)
        if not buf:
            raise weewx.WeeWxIOError('read_eeprom failed: empty read')
        logdbg("read_eeprom: buf: %s" % _fmt(buf))
        if buf[0] != 0x01 or buf[2] != WH23xxStation.READ_EEPROM:
            raise weewx.WeeWxIOError('read_eeprom: bad reply: '
                                     'got %02x %02x %02x %02x, '
                                     'exp 01 .. %02x ..' %
                                     (buf[0], buf[1], buf[2], buf[3],
                                      WH23xxStation.READ_EEPROM))
        logdbg("size: %s" % buf[3])
        return buf[4:]

    def _read_record(self):
        # initiate a read by sending the READ_RECORD command.
        buf = [0x02, 0x02,
               WH23xxStation.READ_RECORD,
               WH23xxStation.READ_RECORD]
        self._write("read_record", buf)

        # now do the actual read.  the station should respond with a single
        # READ_RECORD response spread over (probably) multiple USB packets.
        # each USB packet starts with two bytes, 0x01 followed by the usb
        # packet size.  we check these, but ignore them.  the response
        # contains the READ_RECORD reply, the size of the reply data, the
        # reply data, and a checksum.
        tmp = []
        record_size = 0
        buf = self.devh.interruptRead(
            self.USB_ENDPOINT_IN,
            self.USB_PACKET_SIZE,
            self.timeout)
        if not buf:
            return None
        logdbg("read_record: buf: %s" % _fmt(buf))
        if buf[0] != 0x01:
            raise weewx.WeeWxIOError('read_record: bad first byte: '
                                     '0x%02x != 0x01' % buf[0])
        if buf[2] != WH23xxStation.READ_RECORD:
            raise weewx.WeeWxIOError('read_record: missing READ_RECORD: '
                                     '0x%02x != 0x%02x' %
                                     (buf[2], WH23xxStation.READ_RECORD))
        record_size = buf[3]
        logdbg("record_size: %s" % record_size)
        tmp.extend(buf[4:]) # skip 0x01, payload_size, 0x04, record_size
        while len(tmp) < record_size:
            # FIXME: prevent infinite loop
            buf = self.devh.interruptRead(
                self.USB_ENDPOINT_IN,
                self.USB_PACKET_SIZE,
                self.timeout)
            logdbg("read_record: buf: %s" % _fmt(buf))
            tmp.extend(buf[2:]) # skip 0x01 and payload_size
        rbuf = tmp[0:record_size] # prune off any dangling bytes

        # verify the checksum for the record
        tmp = [WH23xxStation.READ_RECORD, record_size]
        tmp.extend(rbuf)
        chksum = _calc_checksum(tmp)
        logdbg("read_record: rbuf: %s chksum=0x%02x" %
               (_fmt(rbuf), chksum))
        return rbuf

    def _clear_max_min(self):
        logdbg("clear max/min")
        buf = [0x02, 0x02,
               WH23xxStation.CLEAR_MAX_MIN_DAY,
               WH23xxStation.CLEAR_MAX_MIN_DAY]
        self._write("clear_max_min", buf)
        logdbg("max/min cleared")

    def _clear_history(self):
        logdbg("clear history")
        buf = [0x02, 0x02,
               WH23xxStation.CLEAR_HISTORY,
               WH23xxStation.CLEAR_HISTORY]
        self._write("clear_history", buf)
        logdbg("history cleared")

    def get_current(self):
        return self._read_record()

    def sync_time(self):
        self._time_sync(time.time())

    def clear_max_min(self):
        self._clear_max_min()

    def clear_history(self):
        self._clear_history()

    def get_station_info(self):
        # decode the memory starting at address 0x0, which contains the station
        # status and configuration info.  return the data as a dictionary.
        buf = self._read_eeprom(0x0000, 56)
        return self.decode_station_info(buf)

    # this map associates the item identifier with [label, num_bytes, function]
    # required for decoding weather data from raw bytes.
    ITEM_MAPPING = {
        ITEM_INTEMP: ['in_temp', 2, lambda x : x / 10.0 - 40.0],
        ITEM_OUTTEMP: ['out_temp', 2, lambda x : x / 10.0 - 40.0],
        ITEM_DEWPOINT: ['dewpoint', 2, lambda x : x / 10.0 - 40.0],
        ITEM_WINDCHILL: ['windchill', 2, lambda x : x / 10.0 - 40.0],
        ITEM_HEATINDEX: ['heatindex', 2, lambda x : x / 10.0 - 40.0],
        ITEM_INHUMI: ['in_humidity', 1, lambda x : x],
        ITEM_OUTHUMI: ['out_humidity', 1, lambda x : x],
        ITEM_ABSBARO: ['abs_baro', 2, lambda x : x / 10.0],
        ITEM_RELBARO: ['rel_baro', 2, lambda x : x / 10.0],
        ITEM_WINDDIRECTION: ['wind_dir', 2, lambda x : x],
        ITEM_WINDSPEED: ['wind_speed', 2, lambda x : x / 10.0],
        ITEM_GUSTSPEED: ['gust_speed', 2, lambda x : x / 10.0],
        ITEM_RAINEVENT: ['rain_event', 4, lambda x : x / 10.0],
        ITEM_RAINRATE: ['rain_rate', 4, lambda x : x / 10.0],
        ITEM_RAINHOUR: ['rain_hour', 4, lambda x : x / 10.0],
        ITEM_RAINDAY: ['rain_day', 4, lambda x : x / 10.0],
        ITEM_RAINWEEK: ['rain_week', 4, lambda x : x / 10.0],
        ITEM_RAINMONTH: ['rain_month', 4, lambda x : x / 10.0],
        ITEM_RAINYEAR: ['rain_year', 4, lambda x : x / 10.0],
        ITEM_RAINTOTALS: ['rain_totals', 4, lambda x : x / 10.0],
        ITEM_LIGHT: ['light', 4, lambda x : x / 10.0],
        ITEM_UV: ['uv', 2, lambda x : x / 1000.0],
        ITEM_UVI: ['uvi', 1, lambda x : x],
        }

    @staticmethod
    def decode_weather_data(raw):
        # decode a sequence of bytes into current weather data.  the sequence
        # can be variable length.  an identifier byte is followed by one to
        # four data bytes.  identifier bytes have a value of ITEM_* bitwise
        # or with date and/or time if there is an associated time.
        #
        # so we simply walk the array, decoding as we go.  put the result into
        # a dictionary that contains a dictionary for each observation.
        #
        # if there is a failure, log it and bail out.
        data = dict()
        i = 0
        while i < len(raw):
            item = item_raw = raw[i]
            i += 1

            has_date = (item & WH23xxStation.ITEM_DATE) != 0
            has_time = (item & WH23xxStation.ITEM_TIME) != 0

            if has_date:
                item = item & ~WH23xxStation.ITEM_DATE
            if has_time:
                item = item & ~WH23xxStation.ITEM_TIME

            label = None
            obs = dict()
            mapping = WH23xxStation.ITEM_MAPPING.get(item)
            if mapping:
                if i + mapping[1] - 1 >= len(raw):
                    raise weewx.WeeWxIOError(
                        "not enough bytes for %s: idx=%s numbytes=%s bytes=%s"
                        % (mapping[0], i, mapping[1], raw))
                # bytes are decoded MSB first, then function is applied
                label = mapping[0]
                obs['value'] = _decode_bytes(raw, i, mapping[1], mapping[2])
                i += mapping[1]
            else:
                logerr("no mapping for item id 0x%02x (0x%02x)"
                       " at index %s of %s" % (item, item_raw, i-1, _fmt(raw)))
                raise weewx.WeeWxIOError("no mapping for id 0x%02x" % item)

            if has_date:
                # year.month.day
                obs['date'] = "%04d.%02d.%02d" % (
                    2000 + raw[i], raw[i+1], raw[i+2])
                i += 3

            if has_time:
                # hour:minute
                obs['time'] = "%02d:%02d" % (raw[i], raw[i+1])
                i += 2

            # workaround firmware bug for invalid light value
            if (item == WH23xxStation.ITEM_LIGHT and
                obs['value'] == 0xffffff / 10.0):
                obs['value'] = None

            logdbg("%s: %s (0x%02x 0x%02x)" % (label, obs, item, item_raw))
            data[label] = obs
        return data

    @staticmethod
    def decode_history_record(raw):
        # each record is 18 bytes
        #
        # NOTE: the docs specify values for invalid that do not make sense:
        #  light: 0xfff specified, using 0xffffff
        #  uv: 0xff specified, using 0xffff
        #  wind_dir: 0x1f specified, using 0x1ff
        data = dict()
        if not raw:
            logdbg("empty raw data")
            return data
        if len(raw) != 18:
            logdbg("wrong number of bytes in raw data: %s != 18" % len(raw))
            return data
        x = ((raw[0] & 0x01) << 8) + raw[1]
        data['wind_dir'] = None if x == 0x1ff else x # compass degree
        x = (((raw[0] & 0x02) / 0x02) << 8) + raw[2]
        data['wind_speed'] = None if x == 0x1ff else x / 10.0 # m/s
        x = (((raw[0] & 0x04) / 0x04) << 8) + raw[3]
        data['gust_speed'] = None if x == 0x1ff else x / 10.0 # m/s
        data['rain_total'] = ((((raw[0] & 0x08) / 0x08) << 16) + (raw[5] << 8) + raw[4]) * 0.1 # 0.0-9999.9 mm
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
        data['uv'] = None if x == 0xffff else x / 1000.0 # 0-20000 W/m^2
        return data

    @staticmethod
    def decode_station_info(raw):
        data = dict()
        data['eeprom'] = "0x%02x%02x" % (raw[0], raw[1]) # 0x55aa
        data['model'] = "0x%02x%02x" % (raw[2], raw[3]) # 0x0023
        data['version'] = "0x%02x" % raw[4] # 0x10
        data['id'] = "0x%02x%02x%02x%02x" % (raw[5], raw[6], raw[7], raw[8])
        for i in range(0, 8):
            data['factory_unit_flag_1_bit%s' % i] = _get_bit(raw[0x09], i)
        for i in range(0, 8):
            data['factory_unit_flag_2_bit%s' % i] = _get_bit(raw[0x0a], i)
        for i in range(0, 8):
            data['option_1_bit%s' % i] = _get_bit(raw[0x0b], i)
        for i in range(0, 8):
            data['option_2_bit%s' % i] = _get_bit(raw[0x0c], i)
        data['mode'] = 'ASK' if (raw[0x0c] & 0xf0) == 0xf0 else 'UART'
        data['lux_to_rad_factor'] = (raw[0x0e] * 256 + raw[0x0d]) / 10.0
        for i in range(0, 8):
            data['unit_setting_flag_1_bit%s' % i] = _get_bit(raw[0x10], i)
        for i in range(0, 8):
            data['unit_setting_flag_2_bit%s' % i] = _get_bit(raw[0x11], i)
        for i in range(0, 8):
            data['display_setting_flag_1_bit%s' % i] = _get_bit(raw[0x12], i)
        for i in range(0, 8):
            data['display_setting_flag_2_bit%s' % i] = _get_bit(raw[0x13], i)
        for i in range(0, 8):
            data['display_setting_flag_3_bit%s' % i] = _get_bit(raw[0x14], i)
        for i in range(0, 8):
            data['alarm_enable_flag_1_bit%s' % i] = _get_bit(raw[0x15], i)
        for i in range(0, 8):
            data['alarm_enable_flag_2_bit%s' % i] = _get_bit(raw[0x16], i)
        for i in range(0, 8):
            data['alarm_enable_flag_3_bit%s' % i] = _get_bit(raw[0x17], i)
        data['rain_season'] = raw[0x18] # month 1..12
        data['interval'] = raw[0x1a] * 256 + raw[0x19] # seconds 8..14400 (240m)
        data['lcd_contrast'] = "%s (0x%02x)" % (raw[0x1b]-0x16, raw[0x1b]) # 0x17..0x1f
        data['timezone'] = _signed(raw[0x1c]) # -12..12
        data['latitude'] = raw[0x1e] * 256 + raw[0x1d]
        data['longitude'] = raw[0x20] * 256 + raw[0x1f]
        data['weather'] = raw[0x21]
        data['storm'] = raw[0x22]
        data['offset_temperature_in'] = (raw[0x24] * 256 + raw[0x23]) / 10.0
        data['offset_humidity_in'] = raw[0x25]
        data['offset_temperature_out'] = (raw[0x27] * 256 + raw[0x26]) / 10.0
        data['offset_humidity_out'] = raw[0x28]
        data['offset_pressure_abs'] = (raw[0x2a] * 256 + raw[0x29]) / 10.0
        data['offset_pressure_rel'] = (raw[0x2c] * 256 + raw[0x2b]) / 10.0
        data['offset_wind_dir'] = raw[0x2e] * 256 + raw[0x2d]
        data['coefficient_wind'] = raw[0x2f] / 100.0 # 0.1..2.5
        data['coefficient_rain'] = raw[0x30] / 100.0 # 0.1..2.5
        data['coefficient_light'] = (raw[0x32] * 256 + raw[0x31]) / 100.0 # 0.1..10.0
        data['coefficient_uv'] = (raw[0x34] * 256 + raw[0x33]) / 100.0 # 0.1..10.0
        return data


# define a main entry point for basic testing of the station.  invoke this as
# follows from the weewx root dir:
#
# PYTHONPATH=bin python bin/user/wh23xx.py

if __name__ == '__main__':

    INFO_DATA = [
        "55 aa 00 23 10 bc 7a 28 28 52 a2 01 02 f3 04 ff 53 a2 01 4a b2 00 00 00 01 2c 01 1b fb 00 00 00 00 03 04 00 00 00 00 00 00 00 00 c3 ff 00 00 64 64 64 00 64 00 ff ff ff 6b",
        ]
    CURRENT_DATA = [
        "01 02 8f 02 02 13 03 02 11 04 02 13 05 02 13 06 32 07 63 08 27 f0 09 27 b2 0a 00 5a 0b 00 2b 0c 00 3b 0e 00 00 00 00 10 00 00 00 75 11 00 00 00 a2 12 00 00 00 75 13 00 00 04 c5 14 00 00 04 c5 15 00 ff ff ff 16 ff ff 17 ff",
        "01 02 90 02 02 13 03 02 11 04 02 13 05 02 13 06 32 07 63 08 27 f0 09 27 b2 0a 00 5a 0b 00 17 0c 00 21 0e 00 00 00 00 10 00 00 00 75 11 00 00 00 a2 12 00 00 00 75 13 00 00 04 c5 14 00 00 04 c5 15 00 ff ff ff 16 ff ff 17 ff",
        ]
    HISTORY_DATA = [
        "",
        "",
        ]
    CORE_PARAMETERS = ['eeprom', 'id', 'interval', 'latitude', 'longitude',
                       'mode', 'model', 'timezone', 'version']

    def print_info(x, display_keys=None):
        keys = x.keys() if not display_keys else list(set(x.keys()) & set(display_keys))
        keys.sort()
        for k in keys:
            print "%s: %s" % (k, x[k])

    import optparse

    usage = """%prog [options] [--debug] [--help]"""

    syslog.openlog('wh23xx', syslog.LOG_PID | syslog.LOG_CONS)
    syslog.setlogmask(syslog.LOG_UPTO(syslog.LOG_INFO))
    parser = optparse.OptionParser(usage=usage)
    parser.add_option('--version', dest='version', action='store_true',
                      help='display driver version')
    parser.add_option('--debug', dest='debug', action='store_true',
                      help='display diagnostic information while running')
    parser.add_option('--action', dest='action', default='current',
                      help='actions include: info, info-all, current, test-decode-info, test-decode-current, test-decode-history, dump')
    (options, args) = parser.parse_args()

    if options.version:
        print "driver version %s" % DRIVER_VERSION
        exit(1)

    if options.debug:
        syslog.setlogmask(syslog.LOG_UPTO(syslog.LOG_DEBUG))

    if options.action == 'info':
        with WH23xxStation() as s:
            print_info(s.get_station_info(), CORE_PARAMETERS)
    elif options.action == 'info-all':
        with WH23xxStation() as s:
            print_info(s.get_station_info())
    elif options.action == 'current':
        with WH23xxStation() as s:
            while True:
                raw = s.get_current()
                if options.debug:
                    print _fmt(raw)
                print WH23xxStation.decode_weather_data(raw)
                time.sleep(5)
    elif options.action == 'sync-time':
        with WH23xxStation() as s:
            s.sync_time()
    elif options.action == 'clear-history':
        with WH23xxStation() as s:
            s.clear_history()
    elif options.action == 'test-decode-info':
        for row in INFO_DATA:
            raw = [int(x, 16) for x in row.split()]
            print _fmt(raw)
            print WH23xxStation.decode_station_info(raw)
    elif options.action == 'test-decode-current':
        for row in CURRENT_DATA:
            raw = [int(x, 16) for x in row.split()]
            print _fmt(raw)
            print WH23xxStation.decode_weather_data(raw)
    elif options.action == 'test-decode-history':
        for row in HISTORY_DATA:
            raw = [int(x, 16) for x in row.split()]
            print _fmt(raw)
            print WH23xxStation.decode_history_record(raw)
    elif options.action == 'eeprom-time':
        with WH23xxStation() as s:
            raw = s._read_eeprom(0x02c8, 8)
            print _fmt(raw[0:8])
            print "%04d.%02d.%02d %02d:%02d %ss" % (
                2000 + raw[0], raw[1], raw[2], raw[3], raw[4],
                raw[5] + raw[6] * 256)
    elif options.action == 'dump':
        with WH23xxStation() as s:
            size = 0x20
            for i in range(0x0000, 0xffff, size):
                for n in range(0, 3):
                    try:
                        raw = s._read_eeprom(i, 0x20)
                        print "%04x" % i, _fmt(raw[:size])
                        break
                    except Exception, e:
                        print "failed read %d of 3 for 0x%04x: %s" % (n+1, i, e)
                        print "waiting 3 seconds before retry"
                        time.sleep(3)
                else:
                    raise Exception("retries failed")
