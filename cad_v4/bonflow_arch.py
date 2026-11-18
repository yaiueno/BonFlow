#!/usr/bin/env python3
"""BonFlow-Open-v4.0 Canonical Object-Oriented Architecture & Parametric 1U CAD Engine.

License: MIT License (see ../../LICENSE).
Safety & Operational Scope Disclaimer:
  Provided "AS IS" strictly for educational and preliminary cold-run verification
  using aqueous model fluids at ambient pressure and 5V DC low-voltage control.
  All custom fabrication, pressurization, and chemical handling remain solely at
  the user's own risk and responsibility.

Software-Hardware Isomorphism:
  Every 1U cartridge (46x46mm on a 25.0mm pitch optical grid) subclasses `FluidicModule`,
  implementing both logical fluidic routing (`ports`, `internal_paths`, `capabilities`,
  `requirements`) and physical geometry (`channel_segments_mm`, `chamber_cavities_mm`).
  Calling `export_1u_cad()` compiles a 4-layer watertight B-Rep cartridge (`base`,
  `gasket`, `lid`, `retainer`) and verifies 0.0000 mm^3 interference across all layers
  and the 4 corner M3 clamping bolts.
"""
from __future__ import annotations

import json
import math
import re
from abc import ABC, abstractmethod
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any


# BonFlow-Open-v4.0 Mechanical Envelope Constants (mm)
PITCH_MM = 25.0
HALF_PITCH_MM = 12.5
UNIT_SIZE_MM = 46.0
Z_STANDOFF_H = 10.0
BASE_H = 10.0
GASKET_H = 0.8
LID_H = 5.0
RETAIN_H = 1.7
CH_DEPTH_MM = 2.2
CH_WIDTH_MM = 2.2
Z_GASKET = Z_STANDOFF_H + BASE_H           # 20.0
Z_LID = Z_GASKET + GASKET_H                # 20.8
Z_RETAIN = Z_LID + LID_H                   # 25.8
Z_TOP_BOLT = Z_RETAIN + RETAIN_H           # 27.5
M3_CORNERS = ((-HALF_PITCH_MM, -HALF_PITCH_MM), (-HALF_PITCH_MM, HALF_PITCH_MM),
              (HALF_PITCH_MM, -HALF_PITCH_MM), (HALF_PITCH_MM, HALF_PITCH_MM))


class PortKind(str, Enum):
    FLUIDIC = "fluidic"
    I2C = "i2c"
    POWER = "power"


class PortRole(str, Enum):
    SOURCE = "source"
    INLET = "inlet"
    OUTLET = "outlet"
    SINK = "sink"
    BUS = "bus"


@dataclass(frozen=True)
class Port:
    """Standard BonFlow module interface port with local 1U XY coordinates (mm)."""

    name: str
    kind: PortKind = PortKind.FLUIDIC
    role: PortRole = PortRole.INLET
    spec: str = "OD1.6mm_FEP_Vertical_PressFit"
    local_xy_mm: tuple[float, float] = (0.0, 0.0)


@dataclass(frozen=True)
class MechanicalFootprint:
    """25mm optical-grid mounting geometry for 1U (46x46mm) modules."""

    units_u: int = 1
    pitch_mm: float = PITCH_MM
    hole_diameter_mm: float = 6.5
    standoff_mm: float = Z_STANDOFF_H
    slot_xy_mm: tuple[float, float] = (0.0, 0.0)


@dataclass
class RailAuditResult:
    """Evaluation result of a BonFlowRail under a given module state map."""

    open_paths: list[list[str]] = field(default_factory=list)
    reachable_sinks: set[str] = field(default_factory=set)
    blocked_branches: list[str] = field(default_factory=list)
    dead_heads: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)

    @property
    def is_ok(self) -> bool:
        return not self.errors and not self.dead_heads


def _pairwise_brep_overlap_mm3(solids: dict[str, Any]) -> list[tuple[str, str, float]]:
    """Compute exact B-Rep intersection volumes > 0.01 mm^3 across part layers."""
    names = list(solids.keys())
    bboxes = {k: v.bounding_box() for k, v in solids.items()}

    def disjoint(b1: Any, b2: Any, tol: float = 0.05) -> bool:
        return (
            b1.max.X < b2.min.X - tol or b2.max.X < b1.min.X - tol
            or b1.max.Y < b2.min.Y - tol or b2.max.Y < b1.min.Y - tol
            or b1.max.Z < b2.min.Z - tol or b2.max.Z < b1.min.Z - tol
        )

    overlaps: list[tuple[str, str, float]] = []
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = names[i], names[j]
            if disjoint(bboxes[a], bboxes[b]):
                continue
            sa, sb = solids[a], solids[b]
            inter = sa.intersect(sb)
            if inter is None:
                continue
            vol = sum(s.volume for s in inter.solids()) if hasattr(inter, "solids") else float(getattr(inter, "volume", 0.0))
            if vol > 0.01:
                overlaps.append((a, b, vol))
    return overlaps


class FluidicModule(ABC):
    """Abstract base class defining both logical routing & 1U CAD generation contract."""

    def __init__(
        self,
        module_id: str,
        footprint: MechanicalFootprint | None = None,
        wetted_materials: tuple[str, ...] = (),
        internal_volume_uL: float = 0.0,
    ) -> None:
        self.module_id = module_id
        self.footprint = footprint or MechanicalFootprint()
        self.wetted_materials = tuple(wetted_materials)
        self.internal_volume_uL = float(internal_volume_uL)

    @abstractmethod
    def ports(self) -> dict[str, Port]:
        """Return mapping of port_name -> Port."""

    @abstractmethod
    def internal_paths(self, state: dict[str, Any]) -> list[tuple[str, str]]:
        """Return active internal directed paths (from_port, to_port) for state."""

    def capabilities(self, state: dict[str, Any]) -> set[str]:
        """Capabilities provided by this module (e.g., 'backpressure')."""
        return set()

    def requirements(self, state: dict[str, Any]) -> set[str]:
        """Downstream capabilities required on active paths (e.g., 'backpressure')."""
        return set()

    def module_state(self, state: dict[str, Any]) -> Any:
        return state.get(self.module_id)

    def channel_segments_mm(self) -> list[tuple[tuple[float, float], tuple[float, float], float]]:
        """Optional open top-surface CNC channel segments ((x0,y0), (x1,y1), width_mm)."""
        return []

    def chamber_cavities_mm(self) -> list[tuple[float, float, float, float]]:
        """Optional top-open circular chambers (cx_mm, cy_mm, radius_mm, depth_mm)."""
        return []

    @staticmethod
    def _point_to_segment_dist(
        px: float, py: float, x0: float, y0: float, x1: float, y1: float
    ) -> float:
        """Exact Euclidean distance from point (px, py) to segment (x0, y0)-(x1, y1)."""
        dx, dy = x1 - x0, y1 - y0
        len_sq = dx * dx + dy * dy
        if len_sq <= 1e-12:
            return math.hypot(px - x0, py - y0)
        t = max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / len_sq))
        return math.hypot(px - (x0 + t * dx), py - (y0 + t * dy))

    def validate_keepout(self) -> list[str]:
        """Enforce 1U 46x46mm envelope and M3 corner bolt keep-out clearances."""
        violations: list[str] = []
        max_xy = (UNIT_SIZE_MM / 2.0) * self.footprint.units_u - 4.0
        rim_x = max_xy + 0.5
        rim_y = (UNIT_SIZE_MM / 2.0) - 3.5

        for pname, port in self.ports().items():
            if port.kind != PortKind.FLUIDIC:
                continue
            px, py = port.local_xy_mm
            if abs(px) > max_xy or abs(py) > (UNIT_SIZE_MM / 2.0 - 4.0):
                violations.append(
                    f"{self.module_id}.{pname}: local_xy_mm={port.local_xy_mm} exceeds 1U margin (+/-{max_xy:.1f}, +/-19.0 mm)"
                )
            for bx, by in M3_CORNERS:
                dist = math.hypot(px - bx, py - by)
                if dist < 4.8:
                    violations.append(
                        f"{self.module_id}.{pname}: {dist:.2f}mm from M3 bolt ({bx},{by}) < 4.8mm sleeve+bolt keep-out"
                    )

        segs = self.channel_segments_mm()
        if segs and (CH_DEPTH_MM <= 0.0 or CH_DEPTH_MM > BASE_H - 2.0):
            violations.append(
                f"{self.module_id}: CH_DEPTH_MM={CH_DEPTH_MM:.2f}mm out of safe range (0, {BASE_H - 2.0:.1f}] mm"
            )
        for idx, ((x0, y0), (x1, y1), chw) in enumerate(segs):
            if chw <= 0.0:
                violations.append(f"{self.module_id}.seg[{idx}]: non-positive channel width {chw}")
                continue
            half_w = chw / 2.0
            for sx, sy in ((x0, y0), (x1, y1)):
                if (
                    abs(sx) > max_xy
                    or abs(sy) > (UNIT_SIZE_MM / 2.0 - 4.0)
                    or abs(sx) + half_w > rim_x
                    or abs(sy) + half_w > rim_y
                ):
                    violations.append(
                        f"{self.module_id}.seg[{idx}]: endpoint ({sx:.1f},{sy:.1f}) w={chw:.1f}mm breaches 1U sealing rim (+/-{max_xy:.1f}, +/-{rim_y:.1f} mm)"
                    )
                    break
            for bx, by in M3_CORNERS:
                edge_dist = self._point_to_segment_dist(bx, by, x0, y0, x1, y1) - half_w
                if edge_dist < 3.2:
                    violations.append(
                        f"{self.module_id}.seg[{idx}]: {edge_dist:.2f}mm channel-edge to M3 bolt ({bx},{by}) < 3.2mm keep-out"
                    )

        for idx, (cx, cy, cr, cd) in enumerate(self.chamber_cavities_mm()):
            if cr <= 0.0:
                violations.append(f"{self.module_id}.chamber[{idx}]: non-positive radius {cr}")
                continue
            if cd <= 0.0 or cd > BASE_H - 2.0:
                violations.append(
                    f"{self.module_id}.chamber[{idx}]: depth={cd:.2f}mm out of safe floor range (0, {BASE_H - 2.0:.1f}] mm"
                )
            if abs(cx) + cr > rim_x or abs(cy) + cr > rim_y:
                violations.append(
                    f"{self.module_id}.chamber[{idx}]: ({cx:.1f},{cy:.1f}) r={cr:.1f}mm breaches 1U outer sealing rim (+/-{rim_x:.1f}, +/-{rim_y:.1f} mm)"
                )
            for bx, by in M3_CORNERS:
                edge_dist = math.hypot(cx - bx, cy - by) - cr
                if edge_dist < 3.2:
                    violations.append(
                        f"{self.module_id}.chamber[{idx}]: {edge_dist:.2f}mm chamber-wall to M3 bolt ({bx},{by}) < 3.2mm keep-out"
                    )

        return violations

    def build_1u_cad(self) -> dict[str, Any]:
        """Generate 4-layer watertight B-Rep CAD (base, gasket, lid, retainer) centered at local XY."""
        violations = self.validate_keepout()
        if violations:
            raise ValueError("Keep-out violations:\n  " + "\n  ".join(violations))

        from build123d import Align, Box, BuildPart, Compound, Cylinder, Location, Locations, Mode, chamfer

        fluidic_ports = [p for p in self.ports().values() if p.kind == PortKind.FLUIDIC]
        segs = self.channel_segments_mm()
        chambers = self.chamber_cavities_mm()
        w_1u = UNIT_SIZE_MM * self.footprint.units_u

        def sub_m3_holes(z0: float, h: float, r: float) -> None:
            for bx, by in M3_CORNERS:
                with Locations((bx, by, z0)):
                    Cylinder(radius=r, height=h, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)

        def _emit_fluid_cavities(z_pad: float, mode: Any) -> None:
            for (x0, y0), (x1, y1), chw in segs:
                dx, dy = x1 - x0, y1 - y0
                seg_len = math.hypot(dx, dy)
                if seg_len > 1e-6:
                    ang = math.degrees(math.atan2(dy, dx))
                    with Locations(Location(((x0 + x1) / 2.0, (y0 + y1) / 2.0, Z_GASKET - CH_DEPTH_MM), (0, 0, ang))):
                        Box(seg_len, chw, CH_DEPTH_MM + z_pad, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=mode)
                for px, py in ((x0, y0), (x1, y1)):
                    with Locations((px, py, Z_GASKET - CH_DEPTH_MM)):
                        Cylinder(radius=chw / 2.0, height=CH_DEPTH_MM + z_pad, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=mode)
            for cx, cy, cr, cd in chambers:
                with Locations((cx, cy, Z_GASKET - cd)):
                    Cylinder(radius=cr, height=cd + z_pad, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=mode)

        # 1. CNC Base Core (Z = 10.0 .. 20.0 mm)
        with BuildPart() as base_bp:
            with Locations((0.0, 0.0, Z_STANDOFF_H)):
                Box(w_1u, UNIT_SIZE_MM, BASE_H, align=(Align.CENTER, Align.CENTER, Align.MIN))
            chamfer(base_bp.edges(), length=0.35)
            _emit_fluid_cavities(0.2, Mode.SUBTRACT)
            sub_m3_holes(Z_STANDOFF_H - 0.1, BASE_H + 0.2, r=1.65)

        # 2. Cutout Silicone Gasket (Z = 20.0 .. 20.8 mm)
        with BuildPart() as gasket_bp:
            with Locations((0.0, 0.0, Z_GASKET)):
                Box(w_1u - 2.0, UNIT_SIZE_MM - 2.0, GASKET_H, align=(Align.CENTER, Align.CENTER, Align.MIN))
            for port in fluidic_ports:
                px, py = port.local_xy_mm
                with Locations((px, py, Z_GASKET - 0.1)):
                    Cylinder(radius=2.0, height=GASKET_H + 0.2, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)
            sub_m3_holes(Z_GASKET - 0.1, GASKET_H + 0.2, r=1.80)

        # 3. 5mm PMMA Lid with Vertical Press-Fit Bores & Top Sleeve Counterbores (Z = 20.8 .. 25.8 mm)
        with BuildPart() as lid_bp:
            with Locations((0.0, 0.0, Z_LID)):
                Box(w_1u, UNIT_SIZE_MM, LID_H, align=(Align.CENTER, Align.CENTER, Align.MIN))
            chamfer(lid_bp.edges(), length=0.30)
            for port in fluidic_ports:
                px, py = port.local_xy_mm
                with Locations((px, py, Z_LID - 0.1)):
                    Cylinder(radius=1.50, height=LID_H + 0.2, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)
                with Locations((px, py, Z_LID + 3.2)):
                    Cylinder(radius=3.20, height=LID_H - 3.0, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)
            sub_m3_holes(Z_LID - 0.1, LID_H + 0.2, r=1.65)

        # 4. Acrylic Tube Retainer Plate (Z = 25.8 .. 27.5 mm)
        with BuildPart() as retainer_bp:
            with Locations((0.0, 0.0, Z_RETAIN)):
                Box(w_1u, 33.0, RETAIN_H, align=(Align.CENTER, Align.CENTER, Align.MIN))
            chamfer(retainer_bp.edges(), length=0.20)
            for port in fluidic_ports:
                px, py = port.local_xy_mm
                with Locations((px, py, Z_RETAIN - 0.1)):
                    Cylinder(radius=1.00, height=RETAIN_H + 0.2, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)
            sub_m3_holes(Z_RETAIN - 0.1, RETAIN_H + 0.2, r=1.65)

        # Virtual M3 corner bolts for assembly interference verification
        bolt_solids = []
        for bx, by in M3_CORNERS:
            with BuildPart() as b:
                with Locations((bx, by, 3.2)):
                    Cylinder(radius=1.42, height=Z_TOP_BOLT - 3.2, align=(Align.CENTER, Align.CENTER, Align.MIN))
                with Locations((bx, by, Z_TOP_BOLT)):
                    Cylinder(radius=2.65, height=2.5, align=(Align.CENTER, Align.CENTER, Align.MIN))
            bolt_solids.append(b.part)

        layers = {
            "base": base_bp.part,
            "gasket": gasket_bp.part,
            "lid": lid_bp.part,
            "retainer": retainer_bp.part,
        }
        audit_parts = {**layers, "m3_bolts": Compound(children=bolt_solids)}
        overlaps = _pairwise_brep_overlap_mm3(audit_parts)
        if overlaps:
            raise RuntimeError(f"B-Rep interference detected in {self.module_id}: {overlaps}")
        if segs or chambers:
            with BuildPart() as cavity_bp:
                _emit_fluid_cavities(0.0, Mode.ADD)
            keepout_solids = []
            for bx, by in M3_CORNERS:
                with BuildPart() as ko_bp:
                    with Locations((bx, by, Z_STANDOFF_H)):
                        Cylinder(radius=3.0, height=BASE_H, align=(Align.CENTER, Align.CENTER, Align.MIN))
                keepout_solids.append(ko_bp.part)
            cavity_overlaps = _pairwise_brep_overlap_mm3(
                {"fluid_cavity": cavity_bp.part, "m3_keepout": Compound(children=keepout_solids)}
            )
            if cavity_overlaps:
                raise RuntimeError(f"Fluid cavity breaches M3 keep-out sleeve in {self.module_id}: {cavity_overlaps}")
        return layers

    def export_1u_cad(self, out_dir: str | Path) -> dict[str, Path]:
        """Compile, interference-audit, and export 1U STEP assembly + STL layers."""
        from build123d import Compound, export_step, export_stl

        out_path = Path(out_dir)
        out_path.mkdir(parents=True, exist_ok=True)
        layers = self.build_1u_cad()

        exported: dict[str, Path] = {}
        for lname, part in layers.items():
            stl_file = out_path / f"{self.module_id}_{lname}.stl"
            export_stl(part, str(stl_file), tolerance=0.06, angular_tolerance=0.14)
            exported[f"stl_{lname}"] = stl_file

        step_file = out_path / f"{self.module_id}_assembly.step"
        export_step(Compound(children=list(layers.values())), str(step_file))
        exported["step_assembly"] = step_file
        return exported


class ControllerModule(FluidicModule):
    """[ELEC] XIAO ESP32-S3 control bay (isolated I2C/power bus only)."""

    def ports(self) -> dict[str, Port]:
        return {
            "I2C_BUS": Port("I2C_BUS", PortKind.I2C, PortRole.BUS, "4pin_3.3V_I2C"),
        }

    def internal_paths(self, state: dict[str, Any]) -> list[tuple[str, str]]:
        return []

    def capabilities(self, state: dict[str, Any]) -> set[str]:
        return {"controller", "i2c_master"}


class ValveModule(FluidicModule):
    """[VALVE] Stopcock valve module (2-way or 3-way routing)."""

    def __init__(
        self,
        module_id: str,
        footprint: MechanicalFootprint | None = None,
        wetted_materials: tuple[str, ...] = ("PMMA", "Silicone", "PP_Stopcock"),
        internal_volume_uL: float = 145.0,
        is_source_entry: bool = False,
        three_way: bool = False,
    ) -> None:
        super().__init__(module_id, footprint, wetted_materials, internal_volume_uL)
        self.is_source_entry = is_source_entry
        self.three_way = three_way

    def ports(self) -> dict[str, Port]:
        in_role = PortRole.SOURCE if self.is_source_entry else PortRole.INLET
        ps = {
            "IN": Port("IN", PortKind.FLUIDIC, in_role, local_xy_mm=(-15.0, 0.0)),
            "OUT": Port("OUT", PortKind.FLUIDIC, PortRole.OUTLET, local_xy_mm=(15.0, 0.0)),
        }
        if self.three_way:
            ps["WASTE"] = Port("WASTE", PortKind.FLUIDIC, PortRole.SINK, local_xy_mm=(0.0, -15.0))
        return ps

    def internal_paths(self, state: dict[str, Any]) -> list[tuple[str, str]]:
        mode = self.module_state(state) or "open"
        if mode in ("open", "pass"):
            return [("IN", "OUT")]
        if mode == "waste" and self.three_way:
            return [("IN", "WASTE")]
        return []


class JunctionModule(FluidicModule):
    """[MIX] Multi-inlet CNC chip merging into a common outlet."""

    def __init__(
        self,
        module_id: str,
        footprint: MechanicalFootprint | None = None,
        wetted_materials: tuple[str, ...] = ("PMMA", "Silicone"),
        internal_volume_uL: float = 164.6,
        inlet_names: tuple[str, ...] = ("W", "N", "S"),
        outlet_name: str = "E",
        is_source_entry: bool = False,
    ) -> None:
        super().__init__(module_id, footprint, wetted_materials, internal_volume_uL)
        self.inlet_names = inlet_names
        self.outlet_name = outlet_name
        self.is_source_entry = is_source_entry

    def ports(self) -> dict[str, Port]:
        coords = {"W": (-15.0, 0.0), "N": (0.0, 15.0), "S": (0.0, -15.0), "E": (15.0, 0.0)}
        in_role = PortRole.SOURCE if self.is_source_entry else PortRole.INLET
        ps = {
            n: Port(n, PortKind.FLUIDIC, in_role, local_xy_mm=coords.get(n, (0.0, 0.0)))
            for n in self.inlet_names
        }
        ps[self.outlet_name] = Port(
            self.outlet_name, PortKind.FLUIDIC, PortRole.OUTLET, local_xy_mm=coords.get(self.outlet_name, (15.0, 0.0))
        )
        return ps

    def internal_paths(self, state: dict[str, Any]) -> list[tuple[str, str]]:
        return [(n, self.outlet_name) for n in self.inlet_names]

    def channel_segments_mm(self) -> list[tuple[tuple[float, float], tuple[float, float], float]]:
        return [
            ((-15.0, 0.0), (15.0, 0.0), CH_WIDTH_MM),
            ((0.0, -15.0), (0.0, 15.0), CH_WIDTH_MM),
        ]


class ThermalModule(FluidicModule):
    """[THERMO] Heated reaction/waterbath FEP coil requiring downstream BPR at >=60C."""

    def __init__(
        self,
        module_id: str,
        footprint: MechanicalFootprint | None = None,
        wetted_materials: tuple[str, ...] = ("FEP",),
        internal_volume_uL: float = 240.0,
        target_temp_C: float = 63.0,
    ) -> None:
        super().__init__(module_id, footprint, wetted_materials, internal_volume_uL)
        self.target_temp_C = float(target_temp_C)

    def ports(self) -> dict[str, Port]:
        return {
            "IN": Port("IN", PortKind.FLUIDIC, PortRole.INLET, local_xy_mm=(-8.0, 0.0)),
            "OUT": Port("OUT", PortKind.FLUIDIC, PortRole.OUTLET, local_xy_mm=(8.0, 0.0)),
        }

    def internal_paths(self, state: dict[str, Any]) -> list[tuple[str, str]]:
        return [("IN", "OUT")]

    def requirements(self, state: dict[str, Any]) -> set[str]:
        mode = self.module_state(state)
        temp = mode if isinstance(mode, (int, float)) else self.target_temp_C
        if mode != "off" and temp >= 60.0:
            return {"backpressure"}
        return set()


class SensorModule(FluidicModule):
    """[SENSE] Non-wetted optical I2C sensing cell through transparent FEP."""

    def ports(self) -> dict[str, Port]:
        return {
            "IN": Port("IN", PortKind.FLUIDIC, PortRole.INLET, local_xy_mm=(-14.0, 0.0)),
            "OUT": Port("OUT", PortKind.FLUIDIC, PortRole.OUTLET, local_xy_mm=(14.0, 0.0)),
            "I2C_BUS": Port("I2C_BUS", PortKind.I2C, PortRole.BUS, "4pin_3.3V_I2C"),
        }

    def internal_paths(self, state: dict[str, Any]) -> list[tuple[str, str]]:
        return [("IN", "OUT")]

    def capabilities(self, state: dict[str, Any]) -> set[str]:
        return {"optical_sensing"}


class BackpressureDripModule(FluidicModule):
    """[DRIP] Outlet M3 pinch BPR + 1.5mL microtube collection sink."""

    def __init__(
        self,
        module_id: str,
        footprint: MechanicalFootprint | None = None,
        wetted_materials: tuple[str, ...] = ("FEP", "Silicone", "PP_Microtube"),
        internal_volume_uL: float = 20.0,
        bpr_adjustable: bool = True,
    ) -> None:
        super().__init__(module_id, footprint, wetted_materials, internal_volume_uL)
        self.bpr_adjustable = bpr_adjustable

    def ports(self) -> dict[str, Port]:
        return {
            "IN": Port("IN", PortKind.FLUIDIC, PortRole.INLET, local_xy_mm=(0.0, 12.0)),
            "COLLECT": Port("COLLECT", PortKind.FLUIDIC, PortRole.SINK, "1.5mL_Microtube", (0.0, 0.0)),
        }

    def internal_paths(self, state: dict[str, Any]) -> list[tuple[str, str]]:
        if self.module_state(state) == "closed":
            return []
        return [("IN", "COLLECT")]

    def capabilities(self, state: dict[str, Any]) -> set[str]:
        if self.bpr_adjustable and self.module_state(state) not in ("open_loose", "bypass"):
            return {"backpressure"}
        return set()


class SwitchableTrapModule(FluidicModule):
    """Generic solid-phase settling / capture trap (wash vs elute phase switch).

    Demonstrates that third-party labs can subclass `FluidicModule` once to obtain
    both graph verification (`wash -> WASTE`, `elute -> ELUTE`) and a verified 4-layer
    1U 3D CAD cartridge (`export_1u_cad`).
    """

    def ports(self) -> dict[str, Port]:
        return {
            "IN": Port("IN", PortKind.FLUIDIC, PortRole.INLET, local_xy_mm=(-14.0, 0.0)),
            "ELUTE": Port("ELUTE", PortKind.FLUIDIC, PortRole.OUTLET, local_xy_mm=(14.0, 7.5)),
            "WASTE": Port("WASTE", PortKind.FLUIDIC, PortRole.SINK, "Waste_Drain", (14.0, -7.5)),
        }

    def internal_paths(self, state: dict[str, Any]) -> list[tuple[str, str]]:
        phase = self.module_state(state) or "wash"
        if phase == "wash":
            return [("IN", "WASTE")]
        if phase == "elute":
            return [("IN", "ELUTE")]
        return []

    def channel_segments_mm(self) -> list[tuple[tuple[float, float], tuple[float, float], float]]:
        return [
            ((-14.0, 0.0), (0.0, 0.0), CH_WIDTH_MM),
            ((0.0, 0.0), (14.0, 7.5), CH_WIDTH_MM),
            ((0.0, 0.0), (14.0, -7.5), CH_WIDTH_MM),
        ]

    def chamber_cavities_mm(self) -> list[tuple[float, float, float, float]]:
        return [(0.0, 0.0, 5.5, 3.5)]


class BonFlowRail:
    """Architecture container & generic reachability / constraint verifier."""

    _EDGE_RE = re.compile(r"^\s*([A-Za-z0-9_]+)\.([A-Za-z0-9_]+)\s*->\s*([A-Za-z0-9_]+)\.([A-Za-z0-9_]+)\s*$")

    def __init__(self, standard: str = "BonFlow-Open-v4.0", pitch_mm: float = PITCH_MM) -> None:
        self.standard = standard
        self.pitch_mm = pitch_mm
        self.modules: dict[str, FluidicModule] = {}
        self.edges: list[tuple[tuple[str, str], tuple[str, str]]] = []

    def add_module(self, module: FluidicModule) -> None:
        if module.module_id in self.modules:
            raise ValueError(f"Duplicate module_id: {module.module_id}")
        self.modules[module.module_id] = module

    def connect(self, edge_expr: str) -> None:
        """Parse 'src_mod.PORT -> dst_mod.PORT' and register external tube edge."""
        m = self._EDGE_RE.match(edge_expr)
        if not m:
            raise ValueError(f"Invalid topology edge syntax: {edge_expr!r}")
        src_mod, src_port, dst_mod, dst_port = m.groups()
        if src_mod not in self.modules or dst_mod not in self.modules:
            raise KeyError(f"Unknown module in edge: {edge_expr!r}")
        sp = self.modules[src_mod].ports().get(src_port)
        dp = self.modules[dst_mod].ports().get(dst_port)
        if sp is None or dp is None:
            raise KeyError(f"Unknown port in edge: {edge_expr!r}")
        if sp.kind != dp.kind:
            raise TypeError(f"Port kind mismatch: {src_mod}.{src_port}({sp.kind}) -> {dst_mod}.{dst_port}({dp.kind})")
        if src_mod == dst_mod:
            raise ValueError(f"Self-loop external edge not allowed: {edge_expr!r}")
        edge = ((src_mod, src_port), (dst_mod, dst_port))
        if edge in self.edges:
            raise ValueError(f"Duplicate edge already registered: {edge_expr!r}")
        src_node, dst_node = edge
        if any(s == src_node for s, _ in self.edges):
            raise ValueError(f"Double-wired outgoing port (1 tube per port): {src_mod}.{src_port}")
        if any(d == dst_node for _, d in self.edges):
            raise ValueError(f"Double-wired incoming port (1 tube per port): {dst_mod}.{dst_port}")
        self.edges.append(edge)

    @classmethod
    def from_json_file(cls, path: str | Path) -> "BonFlowRail":
        """Instantiate a rail polymorphically from fluidic_graph.json."""
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        pitch = float((data.get("mechanical_grid") or {}).get("pitch_mm", PITCH_MM))
        rail = cls(standard=str(data.get("standard", "BonFlow-Open-v4.0")), pitch_mm=pitch)

        mod_entries = data.get("modules", [])
        topo_strs = data.get("default_topology", [])
        incoming_mods = {
            cls._EDGE_RE.match(e).group(3)
            for e in topo_strs
            if cls._EDGE_RE.match(e)
        }

        for entry in mod_entries:
            mid = entry["id"]
            slot = tuple(float(v) for v in entry.get("slot", [0.0, 0.0]))
            fp = MechanicalFootprint(units_u=1, pitch_mm=pitch, slot_xy_mm=(slot[0], slot[1]))
            mats = tuple(entry.get("wetted_materials", ()))
            vol = float(entry.get("internal_volume_uL", 0.0))

            if "controller" in entry:
                mod: FluidicModule = ControllerModule(mid, fp, mats, vol)
            elif "valve" in mid:
                mod = ValveModule(mid, fp, mats or ("PMMA", "Silicone", "PP_Stopcock"), vol, is_source_entry=(mid not in incoming_mods))
            elif "mix" in mid:
                mod = JunctionModule(mid, fp, mats or ("PMMA", "Silicone"), vol, is_source_entry=(mid not in incoming_mods))
            elif "thermo" in mid or "temp_range_C" in entry or "target_temp_C" in entry:
                t_max = max(entry["temp_range_C"]) if "temp_range_C" in entry else float(entry.get("target_temp_C", 63.0))
                mod = ThermalModule(mid, fp, mats or ("FEP",), vol, target_temp_C=t_max)
            elif "sensor" in entry or "sense" in mid:
                mod = SensorModule(mid, fp, mats or ("FEP",), vol)
            elif entry.get("bpr_adjustable") or "drip" in mid:
                mod = BackpressureDripModule(mid, fp, mats or ("FEP", "Silicone", "PP_Microtube"), vol, bool(entry.get("bpr_adjustable", True)))
            else:
                raise ValueError(f"Cannot infer FluidicModule subclass for JSON entry: {mid}")
            rail.add_module(mod)

        for edge_str in topo_strs:
            rail.connect(edge_str)
        return rail

    def _audit_mechanics(self, res: RailAuditResult) -> None:
        placed = list(self.modules.values())
        for mod in placed:
            sx, sy = mod.footprint.slot_xy_mm
            if abs(sx % self.pitch_mm) > 1e-6 or abs(sy % self.pitch_mm) > 1e-6:
                res.warnings.append(f"{mod.module_id}: slot {mod.footprint.slot_xy_mm} off {self.pitch_mm}mm grid")
            for v in mod.validate_keepout():
                res.errors.append(v)
        for i, m1 in enumerate(placed):
            x1, y1 = m1.footprint.slot_xy_mm
            for m2 in placed[i + 1 :]:
                x2, y2 = m2.footprint.slot_xy_mm
                min_dx = 23.0 * (m1.footprint.units_u + m2.footprint.units_u)
                if abs(y1 - y2) < 46.0 and abs(x1 - x2) < min_dx - 1e-6:
                    res.errors.append(f"Mechanical overlap between {m1.module_id} and {m2.module_id}")

    def evaluate(
        self,
        state: dict[str, Any] | None = None,
        active_sources: list[str] | None = None,
    ) -> RailAuditResult:
        """Evaluate open/blocked reachability, dead-head pressure traps, and module constraints."""
        st = state or {}
        res = RailAuditResult()
        self._audit_mechanics(res)

        adj: dict[tuple[str, str], list[tuple[str, str]]] = {}
        for src, dst in self.edges:
            adj.setdefault(src, []).append(dst)
        for mid, mod in self.modules.items():
            for p_in, p_out in mod.internal_paths(st):
                adj.setdefault((mid, p_in), []).append((mid, p_out))

        sources: list[tuple[str, str]] = []
        if active_sources:
            for s in active_sources:
                m, p = s.split(".", 1)
                sources.append((m, p))
        else:
            for mid, mod in self.modules.items():
                for pname, port in mod.ports().items():
                    if port.kind == PortKind.FLUIDIC and port.role == PortRole.SOURCE:
                        sources.append((mid, pname))

        if not sources:
            res.errors.append("No active fluidic SOURCE port defined on rail.")
            return res

        for src_node in sources:
            q: deque[list[tuple[str, str]]] = deque([[src_node]])
            found_sink_for_source = False

            while q:
                path = q.popleft()
                cur = path[-1]
                cur_mod = self.modules[cur[0]]
                cur_port = cur_mod.ports()[cur[1]]

                if cur_port.role == PortRole.SINK:
                    found_sink_for_source = True
                    res.reachable_sinks.add(f"{cur[0]}.{cur[1]}")
                    res.open_paths.append([f"{m}.{p}" for m, p in path])
                    mods_on_path = list(dict.fromkeys(m for m, _ in path))
                    for idx, mid in enumerate(mods_on_path):
                        reqs = self.modules[mid].requirements(st)
                        if not reqs:
                            continue
                        downstream_caps: set[str] = set()
                        for d_mid in mods_on_path[idx:]:
                            downstream_caps |= self.modules[d_mid].capabilities(st)
                        missing = reqs - downstream_caps
                        if missing:
                            err = (
                                f"{mid} requires downstream capability {sorted(missing)} on path {mods_on_path}"
                            )
                            if err not in res.errors:
                                res.errors.append(err)
                    continue

                nxt_list = adj.get(cur, [])
                if not nxt_list:
                    loc = f"{cur[0]}.{cur[1]}"
                    if cur_port.role == PortRole.OUTLET:
                        leak_err = f"UNPLUGGED_OUTLET_LEAK: open outlet port {loc} has no downstream tube or sink"
                        if leak_err not in res.errors:
                            res.errors.append(leak_err)
                    elif loc not in res.blocked_branches:
                        res.blocked_branches.append(loc)
                    continue

                for nxt in nxt_list:
                    if nxt in path:
                        warn_msg = (
                            f"Recirculation cycle detected along {[f'{m}.{p}' for m, p in path]} -> {nxt[0]}.{nxt[1]}"
                        )
                        if warn_msg not in res.warnings:
                            res.warnings.append(warn_msg)
                        continue
                    q.append(path + [nxt])

            if not found_sink_for_source:
                res.dead_heads.append(f"{src_node[0]}.{src_node[1]} cannot reach any SINK (dead-head pressure block)")

        return res


def _run_self_tests() -> None:
    here = Path(__file__).resolve().parent
    json_path = here / "fluidic_graph.json"
    rail = BonFlowRail.from_json_file(json_path)

    # Test 1A: Default 6U rail open -> reaches drip_1u.COLLECT cleanly with backpressure satisfied
    res_open = rail.evaluate({"valve_1u": "open"})
    assert res_open.is_ok, f"Expected OK, got {res_open}"
    assert "drip_1u.COLLECT" in res_open.reachable_sinks
    print("[Test 1A - 6U Open] OK -> sinks:", sorted(res_open.reachable_sinks), "| path:", " -> ".join(res_open.open_paths[0]))

    # Test 1B: Valve closed -> dead-head detected cleanly
    res_closed = rail.evaluate({"valve_1u": "closed"})
    assert not res_closed.is_ok and res_closed.dead_heads, f"Expected dead-head, got {res_closed}"
    print("[Test 1B - 6U Closed] OK -> dead_heads:", res_closed.dead_heads)

    # Test 2: Insert SwitchableTrapModule between mix_1u and thermo_1u + verify topology & CAD export
    trap = SwitchableTrapModule(
        "custom_trap_1u",
        MechanicalFootprint(units_u=1, pitch_mm=PITCH_MM, slot_xy_mm=(0.0, 50.0)),
    )
    rail.add_module(trap)
    rail.edges = [e for e in rail.edges if e != (("mix_1u", "E"), ("thermo_1u", "IN"))]
    rail.connect("mix_1u.E -> custom_trap_1u.IN")
    rail.connect("custom_trap_1u.ELUTE -> thermo_1u.IN")

    res_wash = rail.evaluate({"valve_1u": "open", "custom_trap_1u": "wash"})
    assert res_wash.is_ok and res_wash.reachable_sinks == {"custom_trap_1u.WASTE"}
    print("[Test 2A - Custom Subclass Wash] OK -> sinks:", sorted(res_wash.reachable_sinks))

    res_elute = rail.evaluate({"valve_1u": "open", "custom_trap_1u": "elute"})
    assert res_elute.is_ok and res_elute.reachable_sinks == {"drip_1u.COLLECT"}
    print("[Test 2B - Custom Subclass Elute] OK -> sinks:", sorted(res_elute.reachable_sinks))

    # Test 3: Parametric 4-layer B-Rep 1U CAD generation & 0.0000 mm^3 collision audit from OOP subclass
    cad_out = here / "custom_1u_out"
    exported = trap.export_1u_cad(cad_out)
    for k, fpath in exported.items():
        assert fpath.exists() and fpath.stat().st_size > 0, f"Missing/empty export: {fpath}"
    print(f"[Test 3 - 1U Parametric CAD & 0.0mm^3 Collision Audit] OK -> exported {len(exported)} files in {cad_out}")


def _expect_raises(exc_type: type[BaseException], fn: Any, msg_sub: str = "") -> BaseException:
    try:
        fn()
    except exc_type as exc:
        if msg_sub and msg_sub not in str(exc):
            raise AssertionError(f"Raised {exc_type.__name__}({exc!s}), expected substring {msg_sub!r}") from exc
        return exc
    raise AssertionError(f"Expected {exc_type.__name__} to be raised")


def _run_adversarial_tests() -> None:
    """12 adversarial edge-case tests across topology, hydraulics, and 1U CNC geometry."""
    here = Path(__file__).resolve().parent

    class _TeeSplitter(FluidicModule):
        def ports(self) -> dict[str, Port]:
            return {
                "IN": Port("IN", PortKind.FLUIDIC, PortRole.INLET, local_xy_mm=(-14.0, 0.0)),
                "OUT_A": Port("OUT_A", PortKind.FLUIDIC, PortRole.OUTLET, local_xy_mm=(0.0, 14.0)),
                "OUT_B": Port("OUT_B", PortKind.FLUIDIC, PortRole.OUTLET, local_xy_mm=(14.0, 0.0)),
            }

        def internal_paths(self, state: dict[str, Any]) -> list[tuple[str, str]]:
            return [("IN", "OUT_A"), ("IN", "OUT_B")]

    class _CustomGeomModule(FluidicModule):
        def __init__(
            self,
            module_id: str,
            custom_ports: dict[str, Port],
            segs: list[tuple[tuple[float, float], tuple[float, float], float]] | None = None,
            chambers: list[tuple[float, float, float, float]] | None = None,
            bypass_keepout: bool = False,
        ) -> None:
            super().__init__(module_id, MechanicalFootprint())
            self._ports = custom_ports
            self._segs = segs or []
            self._chambers = chambers or []
            self._bypass_keepout = bypass_keepout

        def ports(self) -> dict[str, Port]:
            return self._ports

        def internal_paths(self, state: dict[str, Any]) -> list[tuple[str, str]]:
            return []

        def channel_segments_mm(self) -> list[tuple[tuple[float, float], tuple[float, float], float]]:
            return self._segs

        def chamber_cavities_mm(self) -> list[tuple[float, float, float, float]]:
            return self._chambers

        def validate_keepout(self) -> list[str]:
            if self._bypass_keepout:
                return []
            return super().validate_keepout()

    # [Adv 1] 9U PCR manifest: root JunctionModule (3 sources), valve lockouts, loose BPR vs 95/72/58C
    pcr = BonFlowRail.from_json_file(here / "fluidic_graph_pcr.json")
    pcr_open = pcr.evaluate()
    assert pcr_open.is_ok and pcr_open.reachable_sinks == {"drip_1u.COLLECT"}, f"Adv 1 open failed: {pcr_open}"
    assert len(pcr_open.open_paths) == 3, f"Expected 3 source paths from root mix_1u, got {len(pcr_open.open_paths)}"
    pcr_vin_closed = pcr.evaluate({"valve_in_1u": "closed"})
    assert not pcr_vin_closed.is_ok and len(pcr_vin_closed.dead_heads) == 3
    assert "valve_in_1u.IN" in pcr_vin_closed.blocked_branches
    pcr_vout_closed = pcr.evaluate({"valve_out_1u": "closed"})
    assert not pcr_vout_closed.is_ok and len(pcr_vout_closed.dead_heads) == 3
    assert "valve_out_1u.IN" in pcr_vout_closed.blocked_branches
    pcr_loose = pcr.evaluate({"drip_1u": "open_loose"})
    assert not pcr_loose.is_ok
    assert (
        any(e.startswith("thermo_95_1u ") for e in pcr_loose.errors)
        and any(e.startswith("thermo_72_1u ") for e in pcr_loose.errors)
        and not any(e.startswith("thermo_58_1u ") for e in pcr_loose.errors)
    ), f"Expected 95C & 72C backpressure errors (not 58C), got: {pcr_loose.errors}"
    print("[Adv 1 - 9U PCR Root Junction + Dual Valve Lockouts + 95/72/58C BPR] OK")

    # [Adv 2] Unplugged open OUTLET port leak vs safely non-routed outlet
    r_leak = BonFlowRail()
    r_leak.add_module(ValveModule("v1", MechanicalFootprint(slot_xy_mm=(0.0, 0.0)), is_source_entry=True))
    r_leak.add_module(ThermalModule("h1", MechanicalFootprint(slot_xy_mm=(50.0, 0.0)), target_temp_C=55.0))
    r_leak.connect("v1.OUT -> h1.IN")
    res_leak = r_leak.evaluate({"v1": "open"})
    assert not res_leak.is_ok
    assert any("UNPLUGGED_OUTLET_LEAK" in e and "h1.OUT" in e for e in res_leak.errors), f"Expected leak error: {res_leak}"
    assert res_leak.dead_heads, "Expected dead-head alongside unplugged outlet"

    r_trap = BonFlowRail()
    r_trap.add_module(ValveModule("v1", MechanicalFootprint(slot_xy_mm=(0.0, 0.0)), is_source_entry=True))
    r_trap.add_module(SwitchableTrapModule("trap", MechanicalFootprint(slot_xy_mm=(50.0, 0.0))))
    r_trap.connect("v1.OUT -> trap.IN")
    assert r_trap.evaluate({"v1": "open", "trap": "wash"}).is_ok
    res_trap_elute_leak = r_trap.evaluate({"v1": "open", "trap": "elute"})
    assert any("UNPLUGGED_OUTLET_LEAK" in e and "trap.ELUTE" in e for e in res_trap_elute_leak.errors)
    print("[Adv 2 - Unplugged OUTLET Spill Detection] OK")

    # [Adv 3] Heated flow (>=60C) routed directly to unpressurized SINK (missing BPR)
    r_bpr = BonFlowRail()
    r_bpr.add_module(ValveModule("v1", MechanicalFootprint(slot_xy_mm=(0.0, 0.0)), is_source_entry=True))
    r_bpr.add_module(ThermalModule("h65", MechanicalFootprint(slot_xy_mm=(50.0, 0.0)), target_temp_C=65.0))
    r_bpr.add_module(SwitchableTrapModule("trap", MechanicalFootprint(slot_xy_mm=(100.0, 0.0))))
    r_bpr.connect("v1.OUT -> h65.IN")
    r_bpr.connect("h65.OUT -> trap.IN")
    res_bpr = r_bpr.evaluate({"v1": "open", "trap": "wash"})
    assert not res_bpr.is_ok and not res_bpr.dead_heads
    assert any("h65" in e and "backpressure" in e for e in res_bpr.errors), f"Expected backpressure error: {res_bpr}"
    lamp_loose = BonFlowRail.from_json_file(here / "fluidic_graph.json").evaluate({"valve_1u": "open", "drip_1u": "open_loose"})
    assert not lamp_loose.is_ok and any("thermo_1u" in e and "backpressure" in e for e in lamp_loose.errors)
    print("[Adv 3 - Heated Flow to Unpressurized Sink / Loose BPR] OK")

    # [Adv 4] Recirculation cycle detection (4a trapped cycle, 4b cycle + valid sink exit)
    r_cyc = BonFlowRail()
    r_cyc.add_module(JunctionModule("mix", MechanicalFootprint(slot_xy_mm=(0.0, 0.0)), is_source_entry=True))
    r_cyc.add_module(ThermalModule("th", MechanicalFootprint(slot_xy_mm=(50.0, 0.0)), target_temp_C=55.0))
    r_cyc.connect("mix.E -> th.IN")
    r_cyc.connect("th.OUT -> mix.N")
    res_cyc = r_cyc.evaluate()
    assert any("Recirculation cycle" in w for w in res_cyc.warnings), f"Expected cycle warning: {res_cyc}"
    assert not res_cyc.is_ok and len(res_cyc.dead_heads) == 3

    r_cyc_exit = BonFlowRail()
    r_cyc_exit.add_module(ValveModule("v", MechanicalFootprint(slot_xy_mm=(0.0, 0.0)), is_source_entry=True))
    r_cyc_exit.add_module(JunctionModule("mix", MechanicalFootprint(slot_xy_mm=(50.0, 0.0))))
    r_cyc_exit.add_module(_TeeSplitter("split", MechanicalFootprint(slot_xy_mm=(100.0, 0.0))))
    r_cyc_exit.add_module(BackpressureDripModule("drip", MechanicalFootprint(slot_xy_mm=(150.0, 0.0))))
    r_cyc_exit.connect("v.OUT -> mix.W")
    r_cyc_exit.connect("mix.E -> split.IN")
    r_cyc_exit.connect("split.OUT_A -> mix.N")
    r_cyc_exit.connect("split.OUT_B -> drip.IN")
    res_cyc_exit = r_cyc_exit.evaluate()
    assert res_cyc_exit.is_ok and "drip.COLLECT" in res_cyc_exit.reachable_sinks, f"Expected valid exit: {res_cyc_exit}"
    assert any("Recirculation cycle" in w for w in res_cyc_exit.warnings)
    print("[Adv 4 - Cycle Loop Detection (Trapped Dead-Head & Cycle+Exit)] OK")

    # [Adv 5] Diamond split-and-merge where only ONE branch carries a 90C heater bypassing BPR
    r_dia = BonFlowRail()
    r_dia.add_module(ValveModule("v", MechanicalFootprint(slot_xy_mm=(0.0, 0.0)), is_source_entry=True))
    r_dia.add_module(_TeeSplitter("split", MechanicalFootprint(slot_xy_mm=(50.0, 0.0))))
    r_dia.add_module(ThermalModule("h90", MechanicalFootprint(slot_xy_mm=(100.0, 50.0)), target_temp_C=90.0))
    r_dia.add_module(JunctionModule("mix", MechanicalFootprint(slot_xy_mm=(150.0, 0.0))))
    r_dia.add_module(SwitchableTrapModule("sink_trap", MechanicalFootprint(slot_xy_mm=(200.0, 0.0))))
    r_dia.connect("v.OUT -> split.IN")
    r_dia.connect("split.OUT_A -> mix.W")
    r_dia.connect("split.OUT_B -> h90.IN")
    r_dia.connect("h90.OUT -> mix.N")
    r_dia.connect("mix.E -> sink_trap.IN")
    res_dia = r_dia.evaluate({"v": "open", "sink_trap": "wash"})
    assert not res_dia.is_ok and len(res_dia.open_paths) == 2, f"Expected 2 open diamond paths, got {res_dia}"
    assert any("h90" in e and "backpressure" in e for e in res_dia.errors), (
        f"Heated diamond branch missed post-merge requirement check: {res_dia}"
    )
    print("[Adv 5 - Diamond Split-Merge Partial Heater Verification] OK")

    # [Adv 6] Port double-wiring & self-loop rejection in connect()
    r_wire = BonFlowRail.from_json_file(here / "fluidic_graph.json")
    _expect_raises(ValueError, lambda: r_wire.connect("mix_1u.E -> mix_1u.W"), "Self-loop")
    _expect_raises(ValueError, lambda: r_wire.connect("valve_1u.OUT -> mix_1u.W"), "Duplicate edge")
    _expect_raises(ValueError, lambda: r_wire.connect("valve_1u.OUT -> mix_1u.N"), "Double-wired outgoing")
    r_wire.add_module(ValveModule("v2", MechanicalFootprint(slot_xy_mm=(0.0, -50.0))))
    _expect_raises(ValueError, lambda: r_wire.connect("v2.OUT -> mix_1u.W"), "Double-wired incoming")
    print("[Adv 6 - Self-Loop, Duplicate & Double-Wired Port Rejection] OK")

    # [Adv 7] Cross-domain I2C-to-Fluidic & invalid port/module/syntax errors
    r_x = BonFlowRail.from_json_file(here / "fluidic_graph.json")
    _expect_raises(TypeError, lambda: r_x.connect("elec_1u.I2C_BUS -> valve_1u.IN"), "Port kind mismatch")
    _expect_raises(KeyError, lambda: r_x.connect("ghost.OUT -> valve_1u.IN"), "Unknown module")
    _expect_raises(KeyError, lambda: r_x.connect("valve_1u.MISSING -> mix_1u.W"), "Unknown port")
    _expect_raises(ValueError, lambda: r_x.connect("invalid edge"), "Invalid topology")
    print("[Adv 7 - Cross-Domain I2C/Fluidic & Syntax Error Rejection] OK")

    # [Adv 8] Physical 46x46mm 1U plate mechanical overlap collision
    r_ov = BonFlowRail()
    r_ov.add_module(ValveModule("v1", MechanicalFootprint(slot_xy_mm=(0.0, 0.0)), is_source_entry=True))
    r_ov.add_module(BackpressureDripModule("d1", MechanicalFootprint(slot_xy_mm=(25.0, 25.0))))
    r_ov.connect("v1.OUT -> d1.IN")
    res_ov = r_ov.evaluate()
    assert not res_ov.is_ok and any("Mechanical overlap" in e for e in res_ov.errors), f"Expected overlap error: {res_ov}"
    print("[Adv 8 - 46x46mm Overlapping 1U Plate Collision] OK")

    # [Adv 9] Off-25mm optical grid slot warning
    r_off = BonFlowRail()
    r_off.add_module(ValveModule("v1", MechanicalFootprint(slot_xy_mm=(12.5, 0.0)), is_source_entry=True))
    r_off.add_module(BackpressureDripModule("d1", MechanicalFootprint(slot_xy_mm=(75.0, 0.0))))
    r_off.connect("v1.OUT -> d1.IN")
    res_off = r_off.evaluate()
    assert res_off.is_ok and any("off 25.0mm grid" in w for w in res_off.warnings), f"Expected off-grid warning: {res_off}"
    print("[Adv 9 - Off-Grid 25mm Pitch Warning] OK")

    # [Adv 10] Fluidic port inside M3 corner bolt keep-out & outer 1U margin breach
    bad_port_bolt = _CustomGeomModule(
        "bad_bolt_port",
        {"IN": Port("IN", PortKind.FLUIDIC, PortRole.INLET, local_xy_mm=(12.5, 12.5))},
    )
    assert any("sleeve+bolt keep-out" in v for v in bad_port_bolt.validate_keepout())
    _expect_raises(ValueError, bad_port_bolt.build_1u_cad, "Keep-out violations")
    bad_port_rim = _CustomGeomModule(
        "bad_rim_port",
        {"IN": Port("IN", PortKind.FLUIDIC, PortRole.INLET, local_xy_mm=(20.0, 0.0))},
    )
    assert any("exceeds 1U margin" in v for v in bad_port_rim.validate_keepout())
    print("[Adv 10 - Port-to-M3 Bolt & 1U Margin Violation] OK")

    # [Adv 11] Safe endpoints (-15,6)->(-6,15), diagonal channel cuts across M3 corner (-12.5,12.5)
    diag_ports = {
        "IN": Port("IN", PortKind.FLUIDIC, PortRole.INLET, local_xy_mm=(-15.0, 6.0)),
        "OUT": Port("OUT", PortKind.FLUIDIC, PortRole.OUTLET, local_xy_mm=(-6.0, 15.0)),
    }
    diag_seg = [((-15.0, 6.0), (-6.0, 15.0), CH_WIDTH_MM)]
    bad_diag = _CustomGeomModule("bad_diag", diag_ports, segs=diag_seg)
    assert any("channel-edge to M3 bolt" in v for v in bad_diag.validate_keepout()), (
        f"Expected segment keep-out error, got {bad_diag.validate_keepout()}"
    )
    _expect_raises(ValueError, bad_diag.build_1u_cad, "Keep-out violations")
    bad_diag_brep = _CustomGeomModule("bad_diag_brep", diag_ports, segs=diag_seg, bypass_keepout=True)
    _expect_raises(RuntimeError, bad_diag_brep.build_1u_cad, "Fluid cavity breaches M3 keep-out sleeve")
    print("[Adv 11 - Diagonal Channel Segment Keep-Out + B-Rep Sleeve Check] OK")

    # [Adv 12] Through-floor chamber depth (9.5mm > 8.0mm), chamber-to-bolt collision & rim breach
    bad_deep = _CustomGeomModule(
        "bad_deep",
        {"IN": Port("IN", PortKind.FLUIDIC, PortRole.INLET, local_xy_mm=(-14.0, 0.0))},
        chambers=[(0.0, 0.0, 4.0, 9.5)],
    )
    assert any("out of safe floor range" in v for v in bad_deep.validate_keepout())

    bad_ch_bolt = _CustomGeomModule(
        "bad_ch_bolt",
        {"IN": Port("IN", PortKind.FLUIDIC, PortRole.INLET, local_xy_mm=(-14.0, 0.0))},
        chambers=[(8.5, 8.5, 3.5, 3.5)],
    )
    assert any("chamber-wall to M3 bolt" in v for v in bad_ch_bolt.validate_keepout())
    bad_ch_brep = _CustomGeomModule(
        "bad_ch_brep",
        {"IN": Port("IN", PortKind.FLUIDIC, PortRole.INLET, local_xy_mm=(-14.0, 0.0))},
        chambers=[(8.5, 8.5, 3.5, 3.5)],
        bypass_keepout=True,
    )
    _expect_raises(RuntimeError, bad_ch_brep.build_1u_cad, "Fluid cavity breaches M3 keep-out sleeve")

    bad_ch_rim = _CustomGeomModule(
        "bad_ch_rim",
        {"IN": Port("IN", PortKind.FLUIDIC, PortRole.INLET, local_xy_mm=(-14.0, 0.0))},
        chambers=[(17.0, 0.0, 3.5, 3.5)],
    )
    assert any("breaches 1U outer sealing rim" in v for v in bad_ch_rim.validate_keepout())
    print("[Adv 12 - Through-Floor Chamber (9.5mm), Bolt Collision & Sealing-Rim Breach] OK")


if __name__ == "__main__":
    _run_self_tests()
    _run_adversarial_tests()
