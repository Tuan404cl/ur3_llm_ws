"""World model built from camera detections.

Pure Python (no ROS) so it can be unit-tested and reused by the planner,
the validator and the executor.

SceneState JSON (published by perception_node on /scene_state):
{
  "stamp": 1234.5,
  "objects": {"red_cube": {"x": .., "y": .., "z": .., "yaw": .., "zone": "zone_b" | null}},
  "zones":   {"zone_b": {"x": .., "y": .., "detected": true, "occupant": "blue_cube" | null}}
}
"""
import copy
import json
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import yaml

KNOWN_OBJECTS = ['red_cube', 'yellow_cube', 'blue_cube', 'green_cube', 'purple_cube']
KNOWN_ZONES = ['zone_a', 'zone_b', 'zone_c']


def load_scene_config(path: str) -> dict:
    with open(path, 'r') as f:
        return yaml.safe_load(f)


@dataclass
class ObjectState:
    name: str
    x: float
    y: float
    z: float = 0.82
    yaw: float = 0.0
    zone: Optional[str] = None


@dataclass
class ZoneState:
    name: str
    x: float
    y: float
    detected: bool = True
    occupant: Optional[str] = None


@dataclass
class SceneState:
    objects: Dict[str, ObjectState] = field(default_factory=dict)
    zones: Dict[str, ZoneState] = field(default_factory=dict)
    stamp: float = 0.0

    # ------------------------------------------------------------------ io
    @classmethod
    def from_dict(cls, d: dict) -> 'SceneState':
        objs = {n: ObjectState(name=n, **v) for n, v in d.get('objects', {}).items()}
        zones = {n: ZoneState(name=n, **v) for n, v in d.get('zones', {}).items()}
        return cls(objects=objs, zones=zones, stamp=float(d.get('stamp', 0.0)))

    @classmethod
    def from_json(cls, s: str) -> 'SceneState':
        return cls.from_dict(json.loads(s))

    def to_dict(self) -> dict:
        return {
            'stamp': self.stamp,
            'objects': {n: {'x': round(o.x, 4), 'y': round(o.y, 4), 'z': round(o.z, 4),
                            'yaw': round(o.yaw, 3), 'zone': o.zone}
                        for n, o in self.objects.items()},
            'zones': {n: {'x': round(z.x, 4), 'y': round(z.y, 4), 'detected': z.detected,
                          'occupant': z.occupant}
                      for n, z in self.zones.items()},
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict())

    def copy(self) -> 'SceneState':
        return copy.deepcopy(self)

    # ------------------------------------------------------------ queries
    def find_object(self, name: str) -> Optional[ObjectState]:
        return self.objects.get(name)

    def check_zone(self, zone: str) -> Optional[str]:
        """Return the occupant of `zone` (object name) or None if empty."""
        z = self.zones.get(zone)
        return z.occupant if z else None

    def free_zones(self) -> List[str]:
        return [n for n, z in self.zones.items() if z.occupant is None]

    def recompute_occupancy(self, radius: float) -> None:
        """Assign each object to the zone whose centre is within `radius`."""
        for z in self.zones.values():
            z.occupant = None
        for o in self.objects.values():
            o.zone = None
            best, best_d = None, radius
            for z in self.zones.values():
                d = math.hypot(o.x - z.x, o.y - z.y)
                if d < best_d:
                    best, best_d = z, d
            if best is not None:
                o.zone = best.name
                best.occupant = o.name

    def summary_text(self) -> str:
        """Compact human/LLM readable description."""
        lines = ['Objects detected by the camera:']
        for n in sorted(self.objects):
            o = self.objects[n]
            where = o.zone if o.zone else f'table ({o.x:.2f}, {o.y:.2f})'
            lines.append(f'  - {n}: {where}')
        missing = [n for n in KNOWN_OBJECTS if n not in self.objects]
        if missing:
            lines.append(f'  (not visible: {", ".join(missing)})')
        lines.append('Zones:')
        for n in sorted(self.zones):
            z = self.zones[n]
            lines.append(f'  - {n}: ' + (f'OCCUPIED by {z.occupant}' if z.occupant else 'EMPTY'))
        return '\n'.join(lines)


# ---------------------------------------------------------------- geometry
def in_workspace(x: float, y: float, cfg: dict) -> Tuple[bool, str]:
    """Check that a top-down grasp/place at (x, y) is inside the robot workspace."""
    bx, by = cfg['robot_base'][0], cfg['robot_base'][1]
    ws, tb = cfg['workspace'], cfg['table']
    r = math.hypot(x - bx, y - by)
    if not (tb['x_min'] <= x <= tb['x_max'] and tb['y_min'] <= y <= tb['y_max']):
        return False, f'({x:.2f},{y:.2f}) is off the table'
    if r < ws['r_min'] or r > ws['r_max']:
        return False, f'({x:.2f},{y:.2f}) radius {r:.2f} m outside reach [{ws["r_min"]}, {ws["r_max"]}]'
    if x < ws['x_min'] or not (ws['y_min'] <= y <= ws['y_max']):
        return False, f'({x:.2f},{y:.2f}) outside workspace box'
    return True, ''


def find_free_position(scene: SceneState, cfg: dict, moving_object: Optional[str] = None,
                       reserved: Optional[List[Tuple[float, float]]] = None,
                       prefer_near: Optional[Tuple[float, float]] = None
                       ) -> Optional[Tuple[float, float]]:
    """Find an empty spot on the table (not inside any zone) for a temporary place.

    Candidates on a grid inside the reachable workspace must keep
    `clearance_objects` from every other block (camera-detected), and
    `clearance_zones` from every zone centre (so we never 'park' a block in
    a zone).  Among valid candidates we prefer spots close to `prefer_near`
    (short motion) and at a comfortable reach radius.
    """
    fs = cfg['free_space']
    tb = cfg['table']
    reserved = list(reserved or [])
    obstacles = [(o.x, o.y) for n, o in scene.objects.items() if n != moving_object] + reserved
    zones = [(z.x, z.y) for z in scene.zones.values()]
    bx, by = cfg['robot_base'][0], cfg['robot_base'][1]
    r_mid = 0.5 * (cfg['workspace']['r_min'] + cfg['workspace']['r_max'])

    best, best_cost = None, float('inf')
    step = fs['grid_step']
    nx = int((tb['x_max'] - tb['x_min']) / step) + 1
    ny = int((tb['y_max'] - tb['y_min']) / step) + 1
    for i in range(nx):
        x = tb['x_min'] + fs['margin_table'] + i * step
        if x > tb['x_max'] - fs['margin_table']:
            break
        for j in range(ny):
            y = tb['y_min'] + fs['margin_table'] + j * step
            if y > tb['y_max'] - fs['margin_table']:
                break
            ok, _ = in_workspace(x, y, cfg)
            if not ok:
                continue
            if any(math.hypot(x - ox, y - oy) < fs['clearance_objects'] for ox, oy in obstacles):
                continue
            if any(math.hypot(x - zx, y - zy) < fs['clearance_zones'] for zx, zy in zones):
                continue
            cost = 2.0 * abs(math.hypot(x - bx, y - by) - r_mid)
            if prefer_near is not None:
                cost += math.hypot(x - prefer_near[0], y - prefer_near[1])
            if cost < best_cost:
                best, best_cost = (round(x, 3), round(y, 3)), cost
    return best

