"""Matplotlib figures for the design docs (light surface, validated palette:
blue / orange / aqua in fixed order; thin marks; hairline grid; no dual axes)."""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
AXIS = "#c3c2b7"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a"]
GOOD = "#0ca30c"
CRITICAL = "#d03b3b"
PHASE_NAMES = ["Pad", "Boost", "Coast", "Descent", "Landing burn", "Touchdown", "Chute"]


def style() -> None:
    plt.rcParams.update({
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE,
        "savefig.facecolor": SURFACE, "axes.edgecolor": AXIS,
        "axes.labelcolor": INK2, "axes.titlecolor": INK, "xtick.color": MUTED,
        "ytick.color": MUTED, "text.color": INK, "axes.grid": True,
        "grid.color": GRID, "grid.linewidth": 0.8, "grid.linestyle": "-",
        "axes.spines.top": False, "axes.spines.right": False,
        "lines.linewidth": 1.6, "lines.solid_capstyle": "round",
        "lines.solid_joinstyle": "round", "font.size": 9.5,
        "axes.titlesize": 10.5, "axes.titleweight": "bold",
        "axes.titlelocation": "left", "legend.frameon": False,
        "font.family": "DejaVu Sans", "axes.axisbelow": True,
    })


def _phase_bands(ax, t, phase, labels: bool = False) -> None:
    edges = np.flatnonzero(np.diff(phase)) + 1
    starts = np.concatenate([[0], edges])
    ends = np.concatenate([edges, [len(t) - 1]])
    for k, (a, b) in enumerate(zip(starts, ends)):
        if k % 2 == 1:
            ax.axvspan(t[a], t[b], color="#f0efec", lw=0, zorder=0)
        if labels and t[b] - t[a] > 1.3:
            ax.text(0.5 * (t[a] + t[b]), 1.02, PHASE_NAMES[int(phase[a])],
                    transform=ax.get_xaxis_transform(), ha="center", va="bottom",
                    fontsize=8, color=INK2)


def mission(res, path: str, title: str) -> None:
    """Five stacked panels sharing time: altitude, vertical speed, throttle,
    gimbal, tilt."""
    style()
    t, L = res.t, res.log
    fig, axs = plt.subplots(5, 1, figsize=(9, 10), sharex=True,
                            gridspec_kw={"height_ratios": [2.2, 1.6, 1, 1, 1]})
    for ax in axs:
        _phase_bands(ax, t, L["phase"], labels=(ax is axs[0]))
    axs[0].plot(t, L["z_feet"], color=SERIES[0])
    axs[0].set_ylabel("Altitude of feet (m)")
    i = int(np.argmax(L["z_feet"]))
    axs[0].plot(t[i], L["z_feet"][i], "o", ms=6, color=SERIES[0], mec=SURFACE, mew=1.5)
    axs[0].annotate(f"apogee {L['z_feet'][i]:.0f} m ({L['z_feet'][i] * 3.281:.0f} ft)",
                    (t[i], L["z_feet"][i]), xytext=(8, -4), textcoords="offset points",
                    color=INK2, fontsize=8.5, va="top")
    axs[0].set_title(title, pad=18)
    axs[1].plot(t, L["vz"], color=SERIES[0])
    axs[1].axhline(0, color=AXIS, lw=0.8)
    axs[1].set_ylabel("Vertical speed (m/s)")
    axs[2].plot(t, L["throttle"], color=SERIES[1])
    axs[2].set_ylim(0, 1.05)
    axs[2].set_ylabel("Landing throttle k")
    axs[3].plot(t, L["gimbal_deg"], color=SERIES[0])
    axs[3].set_ylabel("Gimbal (deg)")
    axs[4].plot(t, L["theta_deg"], color=SERIES[0])
    axs[4].set_ylabel("Tilt (deg)")
    axs[4].set_xlabel("Time since liftoff (s)")
    fig.align_ylabels(axs)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def landing_zoom(res, path: str, title: str) -> None:
    """The last ~4 s: altitude and vertical speed, throttle, thrust."""
    style()
    t, L = res.t, res.log
    fire = [e[0] for e in res.events if e[1].startswith("FIRE")]
    t0 = fire[0] - 0.8 if fire else t[-1] - 4
    m = (t >= t0) & (t <= t0 + 4.5)
    fig, axs = plt.subplots(4, 1, figsize=(9, 8), sharex=True)
    tt = t[m] - (fire[0] if fire else 0)
    axs[0].plot(tt, L["z_feet"][m], color=SERIES[0])
    axs[0].set_ylabel("Feet altitude (m)")
    axs[0].set_ylim(bottom=-0.5)
    axs[1].plot(tt, L["vz"][m], color=SERIES[0])
    axs[1].axhline(0, color=AXIS, lw=0.8)
    axs[1].set_ylabel("Vertical speed (m/s)")
    axs[2].plot(tt, L["throttle"][m], color=SERIES[1])
    axs[2].set_ylim(0, 1.05)
    axs[2].set_ylabel("Throttle k (paddles)")
    axs[3].plot(tt, L["thrust"][m], color=SERIES[1])
    axs[3].set_ylabel("Delivered thrust (N)")
    axs[3].plot(tt, L["mass"][m] * 9.80665, color=MUTED, lw=1.0)
    axs[3].text(tt[-1], L["mass"][m][-1] * 9.80665, " weight", color=INK2, fontsize=8,
                va="center")
    axs[3].set_xlabel("Time since ignition command (s)")
    for e_t, e_name in res.events:
        if fire and t0 <= e_t <= t0 + 4.5 and e_name.split()[0] in (
                "FIRE", "IGNITION", "LEGS", "TOUCHDOWN"):
            for ax in axs:
                ax.axvline(e_t - fire[0], color=AXIS, lw=0.8)
            axs[0].text(e_t - fire[0], 1.02, e_name.split()[0].lower(), rotation=0,
                        transform=axs[0].get_xaxis_transform(), ha="center",
                        va="bottom", fontsize=8, color=INK2)
    axs[0].set_title(title, pad=18)
    fig.align_ylabels(axs)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def mc_histograms(rows, path: str, title: str, vz_lim: float, vx_lim: float) -> None:
    style()
    vz = np.abs([r["vz"] for r in rows])
    vx = np.abs([r["vx"] for r in rows])
    fig, axs = plt.subplots(1, 2, figsize=(9, 3.6))
    for ax, data, lim, lab in [(axs[0], vz, vz_lim, "Vertical touchdown speed (m/s)"),
                               (axs[1], vx, vx_lim, "Lateral touchdown speed (m/s)")]:
        data = data[np.isfinite(data)]
        bins = np.linspace(0, max(lim * 2, np.percentile(data, 99.5)), 28)
        ax.hist(data, bins=bins, color=SERIES[0], edgecolor=SURFACE, linewidth=1.0)
        ax.axvline(lim, color=CRITICAL, lw=1.2)
        ax.text(lim, 0.97, f"  limit {lim:g} m/s", transform=ax.get_xaxis_transform(),
                color=INK2, fontsize=8.5, va="top")
        within = 100 * np.mean(data <= lim)
        ax.set_title(f"{lab}: {within:.0f}% within limit", fontsize=9.5)
        ax.set_ylabel("Simulated flights")
    fig.suptitle(title, x=0.01, ha="left", fontweight="bold", fontsize=10.5)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def success_vs_bins(labels, pct, counts, path: str, title: str, xlabel: str) -> None:
    style()
    fig, ax = plt.subplots(figsize=(7, 3.4))
    x = np.arange(len(labels))
    ax.bar(x, pct, width=0.45, color=SERIES[0])
    for xi, p, n in zip(x, pct, counts):
        ax.text(xi, p + 1.5, f"{p:.0f}%  (n={n})", ha="center", fontsize=8.5, color=INK2)
    ax.set_xticks(x, labels)
    ax.set_ylim(0, 110)
    ax.set_ylabel("Clean landings (%)")
    ax.set_xlabel(xlabel)
    ax.set_title(title)
    ax.grid(axis="x", visible=False)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def delay_sensitivity(delays, series: dict, path: str, title: str, limit: float) -> None:
    """series: {label: touchdown |vz| list} - one line per throttle authority."""
    style()
    fig, ax = plt.subplots(figsize=(7.5, 3.8))
    for (lab, y), col in zip(series.items(), SERIES):
        ax.plot(delays, y, color=col, marker="o", ms=4.5, mec=SURFACE, mew=1.2, label=lab)
        ax.annotate(lab, (delays[-1], y[-1]), xytext=(6, 0), textcoords="offset points",
                    va="center", fontsize=8.5, color=INK2)
    ax.axhline(limit, color=CRITICAL, lw=1.2)
    ax.text(delays[-1] + 0.11, limit, f"leg limit {limit:g} m/s", va="bottom", ha="right",
            fontsize=8.5, color=INK2)
    ax.set_xlabel("Actual ignition delay (s)  - flight computer assumes 0.30 s")
    ax.set_ylabel("Worst impact speed (m/s)\n(touchdown or hop fall-back)")
    ax.set_title(title)
    ax.legend(loc="upper left", fontsize=8.5)
    ax.set_xlim(delays[0] - 0.02, delays[-1] + 0.12)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
