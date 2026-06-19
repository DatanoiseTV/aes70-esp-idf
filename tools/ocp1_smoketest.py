#!/usr/bin/env python3
"""
ocp1_smoketest.py - exercise a running AES70 device over OCP.1 (AES70-3).

A minimal OCP.1 controller that connects to a device, reads its identity,
enumerates the object tree, round-trips a gain value, and verifies that a
PropertyChanged notification is delivered for a subscribed property. Intended
as a quick on-the-wire check of a device built with this component (e.g. the
examples/p4_nano_dsp demo).

Usage:
    python3 ocp1_smoketest.py <host> [port]

Exit code 0 if all checks pass, 1 otherwise.

SPDX-License-Identifier: MIT
"""
import socket
import struct
import sys

SYNC = 0x3B
CMD_RRQ, NTF, RSP = 1, 2, 3

# Standard ClassIDs used to recognise objects during enumeration.
CID_GAIN = (1, 1, 1, 5)


class Ocp1:
    def __init__(self, host, port):
        self.s = socket.create_connection((host, port), timeout=5)
        self.s.settimeout(5)
        self.h = 1

    def _rxall(self, n):
        b = b""
        while len(b) < n:
            c = self.s.recv(n - len(b))
            if not c:
                raise IOError("connection closed by device")
            b += c
        return b

    def _read_pdu(self):
        sync, ver, pdusize, mtype, mcount = struct.unpack(">BHIBH", self._rxall(10))
        if sync != SYNC:
            raise IOError(f"bad sync byte {sync:#x}")
        return mtype, self._rxall(pdusize + 3 - 10)

    def call(self, target, level, index, params=b"", pcount=0, notifications=None):
        """Send a command (response required) and return (status, paramCount, payload).
        Any notifications received while waiting for the response are appended to
        the `notifications` list, if given."""
        handle = self.h
        self.h += 1
        msg = struct.pack(">IIHHB", handle, target, level, index, pcount) + params
        msg = struct.pack(">I", len(msg) + 4) + msg          # commandSize (inclusive)
        pdu = struct.pack(">BHIBH", SYNC, 1, 7 + len(msg), CMD_RRQ, 1) + msg
        self.s.sendall(pdu)
        while True:
            mtype, data = self._read_pdu()
            if mtype == RSP:
                rsize, = struct.unpack(">I", data[0:4])
                return data[8], data[9], data[10:rsize]   # status, paramCount, payload
            if mtype == NTF and notifications is not None:
                notifications.append(data)

    def close(self):
        self.s.close()


def rd_str(b, o):
    n, = struct.unpack(">H", b[o:o + 2])
    o += 2
    return b[o:o + n].decode(errors="replace"), o + n


def rd_classid(b, o):
    n, = struct.unpack(">H", b[o:o + 2])
    o += 2
    fields = struct.unpack(">" + "H" * n, b[o:o + 2 * n])
    o += 2 * n
    ver, = struct.unpack(">H", b[o:o + 2])
    return fields, ver, o + 2


PASS = True


def check(name, cond, detail=""):
    global PASS
    print(("PASS" if cond else "FAIL") + f"  {name}" + (f"  ({detail})" if detail else ""))
    PASS = PASS and cond


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    host = sys.argv[1]
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 65000

    c = Ocp1(host, port)

    # OcaDeviceManager (ONo 1): identity.
    st, _, pl = c.call(1, 3, 4)                      # GetDeviceName
    name, _ = rd_str(pl, 0)
    check("GetDeviceName", st == 0, f'name="{name}"')

    st, _, pl = c.call(1, 3, 6)                      # GetModelDescription
    mfr, o = rd_str(pl, 0); model, o = rd_str(pl, o); ver, _ = rd_str(pl, o)
    check("GetModelDescription", st == 0, f'{mfr} / {model} / {ver}')

    # OcaRoot.GetClassIdentification on the root block (ONo 100).
    st, _, pl = c.call(100, 1, 1)
    fields, cver, _ = rd_classid(pl, 0)
    check("Root block ClassID", st == 0 and fields == (1, 1, 3), f"{fields} v{cver}")

    # OcaBlock.GetActionObjectsRecursive (100, 3.6): enumerate, find a gain.
    st, _, pl = c.call(100, 3, 6)
    cnt, = struct.unpack(">H", pl[0:2]); o = 2
    gain_ono = None
    for _ in range(cnt):
        ono, = struct.unpack(">I", pl[o:o + 4]); o += 4
        cid, _v, o = rd_classid(pl, o)
        _container, = struct.unpack(">I", pl[o:o + 4]); o += 4
        if cid == CID_GAIN and gain_ono is None:
            gain_ono = ono
    check("GetActionObjectsRecursive", st == 0 and cnt > 0, f"{cnt} objects, gain ONo={gain_ono}")

    if gain_ono is not None:
        # GetGain / SetGain round-trip.
        st, pc, pl = c.call(gain_ono, 4, 1)
        g, mn, mx = struct.unpack(">fff", pl[0:12])
        check("GetGain", st == 0 and pc == 3, f"{g} dB [{mn}..{mx}]")

        target_db = max(mn, min(mx, -6.0))
        st, _, _ = c.call(gain_ono, 4, 2, struct.pack(">f", target_db), pcount=1)
        check("SetGain", st == 0, f"set {target_db} dB")
        st, _, pl = c.call(gain_ono, 4, 1)
        g, _, _ = struct.unpack(">fff", pl[0:12])
        check("GetGain after Set", abs(g - target_db) < 1e-4, f"{g} dB")

        # Subscribe to the gain and confirm a PropertyChanged notification.
        sub = (struct.pack(">IHH", gain_ono, 1, 1) +    # Event: emitter + PropertyChanged(1,1)
               struct.pack(">IHH", 0, 0, 0) +           # Subscriber (null OcaMethod)
               struct.pack(">H", 0) +                   # Context (empty blob)
               struct.pack(">B", 1) +                   # DeliveryMode
               struct.pack(">H", 0))                    # Destination (empty)
        st, _, _ = c.call(4, 3, 1, sub, pcount=5)
        check("AddSubscription", st == 0)

        ntfs = []
        c.call(gain_ono, 4, 2, struct.pack(">f", max(mn, min(mx, -3.0))), pcount=1, notifications=ntfs)
        ok = False
        if ntfs:
            d = ntfs[0]; o = 4 + 4 + 4 + 1            # skip size, targetONo, methodID, paramCount
            clen, = struct.unpack(">H", d[o:o + 2]); o += 2 + clen
            em, = struct.unpack(">I", d[o:o + 4]); o += 4
            evl, evi = struct.unpack(">HH", d[o:o + 4]); o += 4
            pl_, pi_ = struct.unpack(">HH", d[o:o + 4]); o += 4
            val, = struct.unpack(">f", d[o:o + 4])
            ok = em == gain_ono and (evl, evi) == (1, 1) and (pl_, pi_) == (4, 1)
        check("PropertyChanged notification", ok, f"{len(ntfs)} received")

        c.call(gain_ono, 4, 2, struct.pack(">f", g), pcount=1)   # restore

    # Error paths.
    st, _, _ = c.call(999999, 1, 1)
    check("Unknown ONo -> BadONo(5)", st == 5, f"status={st}")

    c.close()
    print("\nRESULT:", "ALL PASS" if PASS else "FAILURES PRESENT")
    return 0 if PASS else 1


if __name__ == "__main__":
    sys.exit(main())
