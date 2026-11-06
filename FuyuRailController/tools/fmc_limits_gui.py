# Live limit-switch monitor for the FMC4030 (read-only; never moves anything).
#
#   python fmc_limits_gui.py
#
# Each tile turns red while its switch is tripped, so you can block the sensors one at a
# time and see which input each one lands on.  All six inputs are shown, including the
# unused axis-2 pair, so a switch plugged into the wrong terminal still shows up.
# A switch changing state flashes its tile and is added to the log at the bottom.
#
# FuyuRailController must be closed: the controller accepts one TCP client.
import ctypes, sys, time, tkinter as tk
from fmc_limits import DLL, IP, PORT, ID, status

TILES = (  # (row, col, input, side, bit, label)
    (0, 0, "LN1", "N", 0, "X gantry −\n(idler end)"),
    (0, 1, "LP1", "P", 0, "X gantry +\n(motor end)"),
    (1, 0, "LN3", "N", 2, "Y −\n(idler end)"),
    (1, 1, "LP3", "P", 2, "Y +\n(motor end)"),
    (2, 0, "LN2", "N", 1, "axis 2 −\n(unused)"),
    (2, 1, "LP2", "P", 1, "axis 2 +\n(unused)"),
)
CLEAR, TRIPPED, FLASH, BG = "#2e7d32", "#c62828", "#ffb300", "#1e1e1e"

class App:
    def __init__(self, root, lib):
        self.root, self.lib, self.last = root, lib, None
        root.title("FMC4030 Limit Switches")
        root.configure(bg=BG)
        self.tiles = {}
        grid = tk.Frame(root, bg=BG)
        grid.pack(padx=12, pady=12)
        for r, c, name, side, bit, label in TILES:
            f = tk.Label(grid, width=16, height=5, font=("Segoe UI", 14, "bold"),
                         fg="white", bg=CLEAR, text=f"{name}\n{label}")
            if name in ("LN2", "LP2"):
                f.configure(font=("Segoe UI", 11))
            f.grid(row=r, column=c, padx=6, pady=6)
            self.tiles[name] = (f, side, bit)
        self.pos = tk.Label(root, bg=BG, fg="#ccc", font=("Consolas", 11))
        self.pos.pack()
        self.log = tk.Listbox(root, height=8, width=48, bg="#111", fg="#ddd", font=("Consolas", 10))
        self.log.pack(padx=12, pady=(6, 12))
        self.poll()

    def poll(self):
        try:
            pos, inp, limN, limP, run, home = status(self.lib)
        except RuntimeError as e:
            self.pos.configure(text=f"read failed: {e}", fg="#f66")
            self.root.after(500, self.poll)
            return
        self.pos.configure(fg="#ccc", text=f"X {pos[0]:8.3f} mm    Y {pos[2]:8.3f} mm    raw N=0x{limN:X} P=0x{limP:X}")
        for name, (w, side, bit) in self.tiles.items():
            on = ((limN if side == "N" else limP) >> bit) & 1
            prev = None if self.last is None else ((self.last[0] if side == "N" else self.last[1]) >> bit) & 1
            w.configure(bg=TRIPPED if on else CLEAR)
            if prev is not None and on != prev:
                self.log.insert(0, f"{time.strftime('%H:%M:%S')}  {name}  {'TRIPPED' if on else 'clear'}")
                if on:
                    w.configure(bg=FLASH)
                    self.root.after(150, lambda w=w: w.configure(bg=TRIPPED))
        self.last = (limN, limP)
        self.root.after(100, self.poll)

def main():
    lib = ctypes.WinDLL(DLL)
    if lib.FMC4030_Open_Device(ID, ctypes.c_char_p(IP), PORT) != 0:
        sys.exit("open failed -- is FuyuRailController still running?")
    time.sleep(1.0)  # controller needs ~1 s after open before it answers
    root = tk.Tk()
    try:
        App(root, lib)
        root.mainloop()
    finally:
        lib.FMC4030_Close_Device(ID)

if __name__ == "__main__":
    main()
