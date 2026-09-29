"""
gui/launcher/theme.py — a flat, muted palette on top of ttk's stdlib "clam"
theme (the only built-in theme that's actually restylable -- "default"/
"alt"/"classic" are the dated Motif-style look this is deliberately moving
away from). No new dependency: clam ships with tkinter itself.
"""

from __future__ import annotations

import tkinter as tk
from tkinter import font as tkfont
from tkinter import ttk

# ---------------------------------------------------------------------------
# Theme: a flat, muted palette on top of ttk's stdlib "clam" theme (the only
# built-in theme that's actually restylable -- "default"/"alt"/"classic" are
# the dated Motif-style look this is deliberately moving away from). No new
# dependency: clam ships with tkinter itself.
# ---------------------------------------------------------------------------

class Palette:
    bg = "#1e1f22"          # app background
    surface = "#26272b"     # panels/cards
    surface_alt = "#2d2f34"  # inputs, list rows
    border = "#3a3c42"
    text = "#e7e8ea"
    text_muted = "#9a9da5"
    accent = "#5b8def"      # primary action
    accent_hover = "#6f9bf2"
    accent_text = "#0d1117"
    danger = "#e5626b"
    success = "#4caf6f"     # save-confirmation flash
    font_family = "Segoe UI"
    font_family_fallback = "Helvetica"


def _resolve_font_family() -> str:
    available = set(tkfont.families())
    for candidate in (Palette.font_family, Palette.font_family_fallback,
                      "DejaVu Sans", "Arial"):
        if candidate in available:
            return candidate
    return "TkDefaultFont"


def apply_theme(root: tk.Tk) -> None:
    """Configures ttk's 'clam' theme with a flat, dark, minimal palette and
    consistent spacing. Called once, on the root window, before any widgets
    are built -- ttk styles are process-global, not per-widget."""
    family = _resolve_font_family()
    base_font = (family, 10)
    heading_font = (family, 11, "bold")
    small_font = (family, 9)

    root.option_add("*Font", base_font)
    root.configure(bg=Palette.bg)

    style = ttk.Style(root)
    style.theme_use("clam")

    style.configure(".", background=Palette.bg, foreground=Palette.text,
                     font=base_font, borderwidth=0)
    style.configure("TFrame", background=Palette.bg)
    style.configure("Card.TFrame", background=Palette.surface)
    style.configure("TLabel", background=Palette.bg, foreground=Palette.text)
    style.configure("Card.TLabel", background=Palette.surface, foreground=Palette.text)
    style.configure("Muted.TLabel", background=Palette.bg, foreground=Palette.text_muted,
                     font=small_font)
    style.configure("CardMuted.TLabel", background=Palette.surface, foreground=Palette.text_muted,
                     font=small_font)
    style.configure("Heading.TLabel", background=Palette.bg, foreground=Palette.text,
                     font=heading_font)
    style.configure("SectionHeading.TLabel", background=Palette.bg,
                     foreground=Palette.text_muted, font=heading_font)
    # Save-confirmation flash / unsaved-changes warning on a status label --
    # a distinct STYLE, not a direct .config(foreground=...) call, since
    # that is what ttk widgets require for a runtime color change to
    # actually repaint under the 'clam' theme this app uses throughout.
    style.configure("Success.TLabel", background=Palette.bg, foreground=Palette.success,
                     font=small_font)
    style.configure("Warning.TLabel", background=Palette.bg, foreground=Palette.danger,
                     font=small_font)

    style.configure("TNotebook", background=Palette.bg, borderwidth=0, tabmargins=(0, 6, 0, 0))
    style.configure("TNotebook.Tab", background=Palette.surface, foreground=Palette.text_muted,
                     padding=(16, 8), borderwidth=0, font=base_font)
    style.map("TNotebook.Tab",
              background=[("selected", Palette.bg)],
              foreground=[("selected", Palette.text)])

    style.configure("TButton", background=Palette.surface_alt, foreground=Palette.text,
                     padding=(12, 7), borderwidth=0, focusthickness=0, relief="flat",
                     font=base_font)
    style.map("TButton",
              background=[("active", Palette.border), ("pressed", Palette.border)])

    style.configure("Accent.TButton", background=Palette.accent, foreground=Palette.accent_text,
                     padding=(14, 8), borderwidth=0, relief="flat", font=(family, 10, "bold"))
    style.map("Accent.TButton",
              background=[("active", Palette.accent_hover), ("pressed", Palette.accent_hover)])

    style.configure("TEntry", fieldbackground=Palette.surface_alt, foreground=Palette.text,
                     insertcolor=Palette.text, bordercolor=Palette.border,
                     lightcolor=Palette.surface_alt, darkcolor=Palette.surface_alt,
                     borderwidth=1, padding=6)
    style.map("TEntry", bordercolor=[("focus", Palette.accent)])

    style.configure("TCombobox", fieldbackground=Palette.surface_alt, foreground=Palette.text,
                     background=Palette.surface_alt, bordercolor=Palette.border,
                     arrowcolor=Palette.text_muted, borderwidth=1, padding=6)
    style.map("TCombobox",
              fieldbackground=[("readonly", Palette.surface_alt)],
              foreground=[("readonly", Palette.text)])
    root.option_add("*TCombobox*Listbox.background", Palette.surface_alt)
    root.option_add("*TCombobox*Listbox.foreground", Palette.text)
    root.option_add("*TCombobox*Listbox.selectBackground", Palette.accent)

    style.configure("TCheckbutton", background=Palette.bg, foreground=Palette.text,
                     font=base_font, focuscolor=Palette.bg)
    style.map("TCheckbutton", background=[("active", Palette.bg)])
    style.configure("Card.TCheckbutton", background=Palette.surface, foreground=Palette.text,
                     font=base_font, focuscolor=Palette.surface)
    style.map("Card.TCheckbutton", background=[("active", Palette.surface)])

    style.configure("TRadiobutton", background=Palette.bg, foreground=Palette.text,
                     font=base_font, focuscolor=Palette.bg)
    style.map("TRadiobutton", background=[("active", Palette.bg)])

    style.configure("Vertical.TScrollbar", background=Palette.surface_alt,
                     troughcolor=Palette.bg, bordercolor=Palette.bg,
                     arrowcolor=Palette.text_muted, borderwidth=0)
    style.map("Vertical.TScrollbar", background=[("active", Palette.border)])

    style.configure("TSeparator", background=Palette.border)


def _field_label(parent: tk.Widget, row: int, label: str, desc: str = "",
                  label_style: str = "TLabel", desc_style: str = "Muted.TLabel",
                  columnspan: int = 1, wraplength: int = 420) -> None:
    """Places LABEL in column 0 of `row`, and, if given, a small muted DESC
    line spanning the same columns in the row immediately below it -- so
    every field across the app carries a short description of what it does
    and what unit it's in, not just a bare name. The control itself
    (entry/combobox/checkbutton) is added separately by the caller at
    (row, column=1+); DESC's row is left otherwise empty so it never
    collides with the control."""
    ttk.Label(parent, text=label, style=label_style).grid(
        row=row, column=0, sticky="nw", pady=(8, 0 if desc else 8))
    if desc:
        ttk.Label(parent, text=desc, style=desc_style, wraplength=wraplength).grid(
            row=row + 1, column=0, columnspan=columnspan, sticky="nw", pady=(0, 8))


def _make_scrollable(parent: tk.Widget, body_padding=(24, 24, 24, 24)) -> ttk.Frame:
    """Wraps `parent` in a vertically-scrollable canvas and returns the
    inner content frame to build a tab's widgets into. Every tab uses this
    (not just Settings, which had it originally) since a tab's content can
    exceed the window's fixed height depending on which optional rows are
    showing (e.g. the Launch tab's "Record new track" fields, or a export/
    stop button row) -- without it, the fixed-size window just clips the
    bottom of the tab with no way to reach it, which is exactly what
    happened before this existed. Mouse-wheel scrolling is bound only while
    the pointer is over this canvas, so it doesn't hijack the notebook's
    own scroll-if-any or a sibling tab's canvas."""
    canvas = tk.Canvas(parent, borderwidth=0, highlightthickness=0, background=Palette.bg)
    scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
    body = ttk.Frame(canvas, padding=body_padding)
    body_window = canvas.create_window((0, 0), window=body, anchor="nw")
    body.bind("<Configure>", lambda _e: canvas.configure(scrollregion=canvas.bbox("all")))
    canvas.bind("<Configure>", lambda e: canvas.itemconfigure(body_window, width=e.width))
    canvas.configure(yscrollcommand=scrollbar.set)
    canvas.pack(side="left", fill="both", expand=True)
    scrollbar.pack(side="right", fill="y")

    def _on_wheel(event):
        # Linux (X11) delivers Button-4/5 with no `.delta`; Windows/macOS
        # deliver <MouseWheel> with a signed `.delta` (multiples of 120 on
        # Windows). Handle both rather than assuming one platform.
        if getattr(event, "num", None) == 4:
            canvas.yview_scroll(-1, "units")
        elif getattr(event, "num", None) == 5:
            canvas.yview_scroll(1, "units")
        else:
            canvas.yview_scroll(-1 if event.delta > 0 else 1, "units")

    def _bind_wheel(_e=None):
        canvas.bind_all("<MouseWheel>", _on_wheel)
        canvas.bind_all("<Button-4>", _on_wheel)
        canvas.bind_all("<Button-5>", _on_wheel)

    def _unbind_wheel(_e=None):
        canvas.unbind_all("<MouseWheel>")
        canvas.unbind_all("<Button-4>")
        canvas.unbind_all("<Button-5>")

    canvas.bind("<Enter>", _bind_wheel)
    canvas.bind("<Leave>", _unbind_wheel)
    return body

