"""Visualization suite for guidance systems."""

from lvguidance.core.visualization.comparison import plot_comparison, plot_metric_bars
from lvguidance.core.visualization.engagement import plot_engagement
from lvguidance.core.visualization.style import apply_style, color_for, save_figure

__all__ = [
    "apply_style",
    "color_for",
    "plot_comparison",
    "plot_engagement",
    "plot_metric_bars",
    "save_figure",
]
