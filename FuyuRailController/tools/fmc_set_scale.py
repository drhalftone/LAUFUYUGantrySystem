# Set FMC4030 per-axis subdivision (div) and soft limits, preserving every other parameter.
# Backs up the controller's current parameter block before writing, then reads it back.
#
#   python fmc_set_scale.py                 # read + print only
#   python fmc_set_scale.py --write         # write DIV / SOFT_LIMIT below
#   python fmc_set_scale.py --restore FILE  # write a saved backup .bin back to the controller
#
# Layout of machine_device_para (MSVC default alignment, 92 bytes), verified against the device:
#   0 id, 4 baud232, 8 baud485, 12 char ip[15] (+1 pad), 28 port,
#   32 div[3], 44 lead[3], 56 softLimitMax[3], 68 softLimitMin[3], 80 homeTime[3]
import ctypes, struct, sys, time, datetime, pathlib

DLL = str(pathlib.Path(__file__).resolve().parent.parent / "sdk" / "FMC4030-Dll.dll")
IP, PORT, ID = b"192.168.0.30", 8088, 0
SIZE = 92

DIV = 1600          # driver microsteps/rev (SW5-8 = OFF OFF ON ON)
SOFT_LIMIT = 650    # mm, applied to both softLimitMax and softLimitMin
AXES = (0, 1, 2)

def decode(raw):
    d = {}
    d["id"], d["baud232"], d["baud485"] = struct.unpack_from("<3I", raw, 0)
    d["ip"] = raw[12:27].split(b"\0")[0].decode(errors="replace")
    d["port"], = struct.unpack_from("<i", raw, 28)
    for name, off in (("div", 32), ("lead", 44), ("softMax", 56), ("softMin", 68), ("homeTime", 80)):
        d[name] = list(struct.unpack_from("<3i", raw, off))
    return d

def show(label, raw):
    d = decode(raw)
    print(f"{label}: id={d['id']} ip={d['ip']} port={d['port']}")
    for a in range(3):
        ppm = d["div"][a] / d["lead"][a] if d["lead"][a] else float("nan")
        print(f"  axis {a}: div={d['div'][a]} lead={d['lead'][a]} -> {ppm:.1f} pulses/mm, "
              f"soft +{d['softMax'][a]}/-{d['softMin'][a]}, homeTime={d['homeTime'][a]} ms")
    return d

def read_para(lib):
    for _ in range(5):
        buf = (ctypes.c_ubyte * 256)()
        if lib.FMC4030_Get_Device_Para(ID, buf) == 0:
            return bytes(buf)[:SIZE]
        time.sleep(1.0)   # the controller needs ~1 s after open before it answers
    raise RuntimeError("FMC4030_Get_Device_Para failed")

def write_para(lib, raw):
    buf = (ctypes.c_ubyte * 256).from_buffer_copy(raw + bytes(256 - len(raw)))
    rc = lib.FMC4030_Set_Device_Para(ID, buf)
    # -6 = no acknowledgement: the controller commits the parameters to flash but often
    # doesn't reply in time.  Let the read-back decide whether the write took.
    if rc == -6:
        print("Set_Device_Para: no acknowledgement (-6); verifying by read-back")
    elif rc != 0:
        raise RuntimeError(f"FMC4030_Set_Device_Para returned {rc}")

def main():
    lib = ctypes.WinDLL(DLL)
    if lib.FMC4030_Open_Device(ID, ctypes.c_char_p(IP), PORT) != 0:
        sys.exit("open failed -- is FuyuRailController still running?")
    try:
        time.sleep(1.0)
        raw = read_para(lib)
        cur = show("current", raw)
        if cur["port"] != PORT or cur["ip"] != IP.decode():
            sys.exit("decoded ip/port don't match -- layout assumption wrong, refusing to write")

        if "--restore" in sys.argv:
            new = pathlib.Path(sys.argv[sys.argv.index("--restore") + 1]).read_bytes()[:SIZE]
        elif "--write" in sys.argv:
            new = bytearray(raw)
            for a in AXES:
                struct.pack_into("<i", new, 32 + 4 * a, DIV)
                struct.pack_into("<i", new, 56 + 4 * a, SOFT_LIMIT)
                struct.pack_into("<i", new, 68 + 4 * a, SOFT_LIMIT)
            new = bytes(new)
        else:
            return

        stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        backup = pathlib.Path(__file__).resolve().parent / f"fmc4030_params_backup_{stamp}.bin"
        backup.write_bytes(raw)
        print(f"backup saved: {backup}")

        write_para(lib, new)
        time.sleep(1.0)
        after = show("read back", read_para(lib))
        want = decode(new)
        ok = all(after[k] == want[k] for k in ("div", "lead", "softMax", "softMin", "port", "ip"))
        print("VERIFIED" if ok else "MISMATCH -- read-back differs from what was written")
    finally:
        lib.FMC4030_Close_Device(ID)

if __name__ == "__main__":
    main()
