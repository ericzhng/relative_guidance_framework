"""Plots and animation of approach simulations."""

from lvguidance.visualization.animation import animate_engagement
from lvguidance.visualization.comparison import plot_comparison, plot_metric_bars
from lvguidance.visualization.engagement import plot_engagement
from lvguidance.visualization.terminal import plot_terminal_phase

__all__ = [
    "animate_engagement",
    "plot_comparison",
    "plot_engagement",
    "plot_metric_bars",
    "plot_terminal_phase",
]
