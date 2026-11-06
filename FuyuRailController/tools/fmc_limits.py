# Read the FMC4030 limit-switch inputs (read-only; never moves anything).
#
#   python fmc_limits.py            # one snapshot
#   python fmc_limits.py --watch    # live: prints a line every time a switch changes (Ctrl+C to quit)
#
# FuyuRailController must be closed: the controller accepts one TCP client.
#
# Layout of machine_status (FUYU FMC4030-Dll.h, MSVC default alignment):
#   0 float realPos[3], 12 float realSpeed[3], 24 inputStatus, 28 outputStatus,
#   32 limitNStatus, 36 limitPStatus, 40 machineRunStatus, 44 axisStatus[3], 56 homeStatus, 60 file[600]
# limitN/limitP are bitmasks: bit 0 = axis 1 (LN1/LP1), bit 1 = axis 2, bit 2 = axis 3.
# The layout is sanity-checked by comparing realPos against FMC4030_Get_Axis_Current_Pos.
import ctypes, struct, sys, time, pathlib

DLL = str(pathlib.Path(__file__).resolve().parent.parent / "sdk" / "FMC4030-Dll.dll")
IP, PORT, ID = b"192.168.0.30", 8088, 0

# Wiring per FUYU_FMC4030_Assembly_Wiring_Manual.html section 10 (axis 2 unused).
SWITCHES = (("LN1", "X - (idler end)", "N", 0), ("LP1", "X + (motor end)", "P", 0),
            ("LN3", "Y - (idler end)", "N", 2), ("LP3", "Y + (motor end)", "P", 2),
            ("LN2", "axis 2 (unused)",   "N", 1), ("LP2", "axis 2 (unused)", "P", 1))

def status(lib):
    buf = (ctypes.c_ubyte * 1024)()
    rc = lib.FMC4030_Get_Machine_Status(ID, buf)
    if rc != 0:
        raise RuntimeError(f"FMC4030_Get_Machine_Status returned {rc}")
    raw = bytes(buf)
    pos = struct.unpack_from("<3f", raw, 0)
    inp, out, limN, limP, run = struct.unpack_from("<5I", raw, 24)
    home, = struct.unpack_from("<I", raw, 56)
    return pos, inp, limN, limP, run, home

def describe(limN, limP):
    parts = []
    for name, label, side, bit in SWITCHES:
        on = ((limN if side == "N" else limP) >> bit) & 1
        parts.append(f"{name}={'TRIPPED' if on else 'clear  '}")
    return "  ".join(parts)

def main():
    lib = ctypes.WinDLL(DLL)
    if lib.FMC4030_Open_Device(ID, ctypes.c_char_p(IP), PORT) != 0:
        sys.exit("open failed -- is FuyuRailController still running?")
    try:
        time.sleep(1.0)   # controller needs ~1 s after open before it answers
        for _ in range(5):
            try:
                pos, inp, limN, limP, run, home = status(lib)
                break
            except RuntimeError:
                time.sleep(1.0)
        else:
            sys.exit("FMC4030_Get_Machine_Status kept failing")

        # Layout check: positions from the status block must match the per-axis query.
        p = ctypes.c_float()
        direct = []
        for a in range(3):
            lib.FMC4030_Get_Axis_Current_Pos(ID, a, ctypes.byref(p))
            direct.append(p.value)
        ok = all(abs(x - y) < 0.01 for x, y in zip(pos, direct))
        print(f"positions (status) {['%.3f' % x for x in pos]}  (direct) {['%.3f' % x for x in direct]}"
              f"  -> layout {'OK' if ok else 'MISMATCH -- limit bits below may be wrong'}")
        print(f"raw: limitN=0x{limN:X} limitP=0x{limP:X} input=0x{inp:X} run={run} home=0x{home:X}")
        print()
        for name, label, side, bit in SWITCHES:
            on = ((limN if side == "N" else limP) >> bit) & 1
            print(f"  {name:4} {label:20} {'TRIPPED' if on else 'clear'}")

        if "--watch" in sys.argv:
            print("\nwatching -- block each sensor in turn (Ctrl+C to quit)")
            last = (limN, limP)
            while True:
                time.sleep(0.1)
                _, _, limN, limP, _, _ = status(lib)
                if (limN, limP) != last:
                    print(time.strftime("%H:%M:%S"), describe(limN, limP), flush=True)
                    last = (limN, limP)
    except KeyboardInterrupt:
        pass
    finally:
        lib.FMC4030_Close_Device(ID)

if __name__ == "__main__":
    main()
