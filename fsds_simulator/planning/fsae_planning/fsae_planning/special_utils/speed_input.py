"""
Manual target-speed input for skidpad characterisation.

The skidpad mode normally ramps the target speed up automatically until the car
slides off the lane.  This utility lets an operator instead type a target speed
(m/s) on the fly: a tiny always-on-top text box (tkinter) runs in a background
thread and the planner polls :meth:`SpeedInput.latest` each cycle, driving the
car at the entered speed instead of the ramp.

When no GUI display is available (headless run, no tkinter) it transparently
falls back to reading speeds from stdin in the terminal.  Either way the input
runs on its own daemon thread, so the ROS executor is never blocked.

Usage::

    speed = SpeedInput(minimum=0.0, maximum=25.0, logger=node.get_logger().info)
    speed.start()
    ...
    target = speed.latest()      # float m/s, or None until the operator enters one
    ...
    speed.stop()
"""
import threading


class SpeedInput:
    """Background target-speed entry (GUI text box, or terminal fallback)."""

    def __init__(self, initial=None, minimum=0.0, maximum=30.0, logger=None):
        self._value = initial              # latest entered speed, or None
        self._min = float(minimum)
        self._max = float(maximum)
        self._log = logger or (lambda msg: print(msg))
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

    # -- public API ---------------------------------------------------------

    def start(self) -> 'SpeedInput':
        """Spawn the input thread (GUI if possible, else terminal). Returns self."""
        self._thread = threading.Thread(target=self._run, name='speed-input',
                                        daemon=True)
        self._thread.start()
        return self

    def latest(self):
        """Latest entered target speed (m/s), or None if nothing entered yet."""
        with self._lock:
            return self._value

    def stop(self) -> None:
        """Ask the input thread to close (GUI window / terminal loop)."""
        self._stop.set()

    # -- internals ----------------------------------------------------------

    def _apply(self, raw) -> bool:
        """Parse, clamp and store a raw entry; log the outcome. True on success."""
        try:
            value = float(str(raw).strip())
        except (TypeError, ValueError):
            self._log(f'Speed input: "{raw}" is not a number — ignored.')
            return False
        clamped = max(self._min, min(self._max, value))
        with self._lock:
            self._value = clamped
        note = '' if clamped == value else f' (clamped to [{self._min:g}, {self._max:g}])'
        self._log(f'Speed input: target set to {clamped:.2f} m/s{note}.')
        return True

    def _run(self) -> None:
        try:
            self._run_gui()
        except Exception as exc:                       # no display / no tkinter
            self._log(f'Speed input: GUI unavailable ({exc!r}); reading from terminal.')
            self._run_terminal()

    def _run_gui(self) -> None:
        import tkinter as tk

        root = tk.Tk()
        root.title('Skidpad speed')
        root.attributes('-topmost', True)
        root.resizable(False, False)

        entry = tk.Entry(root, width=10, font=('TkDefaultFont', 14), justify='center')
        entry.grid(row=0, column=0, columnspan=2, padx=8, pady=(8, 4))
        entry.focus_set()

        current = tk.StringVar(value='target: ramp')
        tk.Label(root, textvariable=current).grid(row=1, column=0, columnspan=2,
                                                  padx=8, pady=(0, 4))

        def submit(*_):
            if self._apply(entry.get()):
                current.set(f'target: {self.latest():.2f} m/s')
            entry.delete(0, tk.END)

        tk.Button(root, text='Set (m/s)', command=submit).grid(
            row=2, column=0, columnspan=2, padx=8, pady=(0, 8))
        entry.bind('<Return>', submit)

        def poll_stop():
            if self._stop.is_set():
                root.destroy()
                return
            root.after(200, poll_stop)

        root.protocol('WM_DELETE_WINDOW', self._stop.set)
        root.after(200, poll_stop)
        self._log('Speed input: type a target speed (m/s) and press Enter.')
        root.mainloop()

    def _run_terminal(self) -> None:
        while not self._stop.is_set():
            try:
                raw = input('Target speed (m/s) > ')
            except (EOFError, KeyboardInterrupt):
                return                                 # no interactive stdin
            except Exception:
                return
            if raw.strip():
                self._apply(raw)
