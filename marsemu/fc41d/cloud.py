"""Offline stand-in for the Hamedata/Marstek HTTP cloud.

Only what the firmwares need to make progress is answered: the time
handshake (``getDateInfo*.php``) and generic "accepted" answers for the
reporting endpoints.  Everything is overridable with ``Network.add_http_rule``.
"""
from __future__ import annotations

import datetime as _dt
from typing import Callable, List, Tuple


#: the four trailing fields of the real cloud's clock reply (as Marsrelay,
#: github.com/tomquist/marsrelay, mirrors them).  Without them a B2500 rejects
#: the date, keeps asking every 15 s, and each request blocks its module for
#: ~10 s, long enough to lose its CT meter.
TIME_SUFFIX = "04_0_0_0"

#: what the real cloud tells every B2500 about its output limits (``getDeviceFire``)
DEVICE_FIRE = b'{"code":1,"show":0,"msg":"ok","data":{"id":255,"s":360,"c":30,"i":0,"p":800}}'
#: ``getRealtimeSoc`` without a charge level: the battery keeps its own estimate
#: (the real cloud would tell an unknown device 0 %)
REALTIME_SOC = b'{"code":1,"show":0,"msg":"ok","data":{"time_no":0}}'


def server_time_body(now: _dt.datetime) -> bytes:
    """``getDateInfoeu.php`` answers with the server's local time and four fixed fields."""
    return (now.strftime("_%Y_%m_%d_%H_%M_%S_") + TIME_SUFFIX).encode()


def hamedata_rules(now: Callable[[], _dt.datetime]) -> List[Tuple[str, int, object]]:
    ok_json = b'{"code":1,"msg":"success","data":[]}'
    return [
        (r"/getDateInfo[a-z]*\.php", 200, lambda url, body: server_time_body(now())),
        (r"/app/Solar/puterrinfo\.php", 200, lambda url, body: b"_1" if body else b"_2"),
        (r"/app/Solar/put", 200, b"1"),
        (r"/ems/api/v1/getDeviceFire", 200, DEVICE_FIRE),
        (r"/ems/api/v1/getRealtimeSoc", 200, REALTIME_SOC),
        (r"/prod/api/v1/setB2500Report", 200, b'{"code":1,"msg":"ok"}'),
        (r"/prod/api/v1/setSmallReporting", 200, b'{"code":1,"msg":"ok","data":{"de":1}}'),
        # CT002 (122+) / CT003 every 5 min; a CT002 without a 200 repeats it back to back
        (r"/prod/api/v1/setCtReporting", 200, b'{"code":1,"msg":"success"}'),
        (r"/(prod|ems)/api/v[12]/set[A-Za-z]+Report", 200, ok_json),
        (r"/ems/api/v1/get", 200, ok_json),
        (r"/external-services/api/v1/", 200, ok_json),
        (r"/getWeatherInfo\.php", 200, b"{}"),
        (r"marstekcloud\.com/data-upload/", 200, ok_json),
    ]
