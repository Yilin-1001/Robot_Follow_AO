from __future__ import annotations

from collections import OrderedDict

from robot_follow.ao.core.geometry import AxisAlignedBox


_DEFAULT_ZONE_BOUNDS = OrderedDict(
    (
        ("front", (0.2, 1.0, -0.35, 0.35, -0.15, 1.5)),
        ("left", (-0.05, 0.7, 0.35, 0.75, -0.15, 1.5)),
        ("right", (-0.05, 0.7, -0.75, -0.35, -0.15, 1.5)),
        ("rear", (-0.6, -0.1, -0.35, 0.35, -0.15, 1.5)),
        ("rotation", (-0.65, 0.65, -0.75, 0.75, -0.15, 1.5)),
    )
)


def default_zone_bounds():
    return OrderedDict(
        (name, AxisAlignedBox(*values))
        for name, values in _DEFAULT_ZONE_BOUNDS.items()
    )


def zone_bounds_from_config(zones_config):
    bounds = getattr(zones_config, "bounds", None)
    if not bounds:
        return default_zone_bounds()
    ordered = OrderedDict()
    for name in _DEFAULT_ZONE_BOUNDS:
        cfg = bounds[name]
        ordered[name] = AxisAlignedBox(
            float(cfg.x_min),
            float(cfg.x_max),
            float(cfg.y_min),
            float(cfg.y_max),
            float(cfg.z_min),
            float(cfg.z_max),
        )
    return ordered
