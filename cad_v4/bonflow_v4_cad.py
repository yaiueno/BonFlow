"""Parametric 3D CAD assembly for BonFlow Standard v4.0 (25mm Grid Arbitrary-Order 1U Patchable System):
- Universal A5 Optical Matrix Baseplate (165 x 125 x 5 mm, 6x4 array of Ø6.5mm holes on 25.0 mm pitch)
- Universal 1U Module Footprint (46 x 46 mm) elevated on 10mm M3 Hex Standoffs ('Nut-Feet') + M3 through-bolts
- Universal Top Vertical Press-Fit Port (Ø3.0mm lid hole + Ø4.0mm relief gasket + OD 3.2mm silicone sleeve + retainer)
- 6 Independent 1U Slots on a 3x2 Grid, connected in ANY order via overhead OD 1.6mm FEP jumper tubes:
    [Row +25] (-50,+25) [VALVE]  ->  (0,+25) [MIX]   ->  (+50,+25) [THERMO]
    [Row -25] (-50,-25) [ELEC]   <-  (0,-25) [DRIP]  <-  (+50,-25) [SENSE]
- Exports STLs, STEP assembly, SDL 'fluidic_graph.json' manifest, and runs zero-interference checks.
"""
import json
import math
from pathlib import Path

from build123d import (
    Align,
    Box,
    BuildLine,
    BuildPart,
    BuildSketch,
    Circle,
    Cylinder,
    Helix,
    Locations,
    Mode,
    Plane,
    Polygon,
    RadiusArc,
    Spline,
    chamfer,
    export_step,
    export_stl,
    sweep,
)

OUT_DIR = Path("/root/tw_rag/work/bizcon/cad_v4/parts")
OUT_DIR.mkdir(parents=True, exist_ok=True)

PITCH = 25.0
HALF_P = PITCH / 2.0  # 12.5 mm from module center to corner M3 standoff
UNIT_W = 46.0
Z_STANDOFF_H = 10.0   # Standoff top surface where every 1U module sits
LOCATOR_D = 3.2       # Locator boss insertion depth into baseplate

# 6 module centers on the linear 1-row rail (50.0mm spacing = 2x25.0mm pitch, Y=0)
POS_ELEC = (-125.0, 0.0)
POS_VALVE = (-75.0, 0.0)
POS_MIX = (-25.0, 0.0)
POS_THERMO = (25.0, 0.0)
POS_SENSE = (75.0, 0.0)
POS_DRIP = (125.0, 0.0)

ALL_SLOTS = [POS_ELEC, POS_VALVE, POS_MIX, POS_THERMO, POS_SENSE, POS_DRIP]

CH_W = 2.2
CH_D = 2.2
BASE_H = 10.0         # Z = 10..20
GASKET_H = 0.8        # Z = 20..20.8
LID_H = 5.0           # Z = 20.8..25.8
RETAIN_H = 1.7        # Z = 25.8..27.5
Z_GASKET = Z_STANDOFF_H + BASE_H               # 20.0
Z_LID = Z_GASKET + GASKET_H                    # 20.8
Z_RETAIN = Z_LID + LID_H                       # 25.8
Z_TOP_BOLT = Z_RETAIN + RETAIN_H               # 27.5


def hex_prism(flat_to_flat: float, height: float):
    """Return a hexagonal prism centered in XY, aligned MIN in Z."""
    r_circum = (flat_to_flat / 2.0) / math.cos(math.pi / 6.0)
    with BuildPart() as p:
        with BuildSketch() as sk:
            Polygon(*[(r_circum * math.cos(i * math.pi / 3.0), r_circum * math.sin(i * math.pi / 3.0)) for i in range(6)])
        from build123d import extrude
        extrude(sk.sketch, amount=height)
    return p.part


def make_base_plate(length=316.0, cols=12, start_x=-137.5, anchor_x=150.0):
    """Linear 1-Row Optical Rail Baseplate with cols x 2 Ø6.5mm holes on 25.0mm pitch."""
    with BuildPart() as p:
        with Locations((0, 0, -5.0)):
            Box(length, 68.0, 5.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
        chamfer(p.edges(), length=0.4)
        # Universal col x 2 matrix of Ø6.5mm round locator holes on 25.0mm optical grid
        for i in range(cols):
            ix = start_x + i * PITCH
            for iy in (-12.5, 12.5):
                with Locations((ix, iy, -5.2)):
                    Cylinder(radius=3.25, height=5.4, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)
        # Outer breadboard M4 anchor slots at outer corners
        for sx in (-anchor_x, anchor_x):
            for sy in (-26.0, 26.0):
                with Locations((sx, sy, -5.2)):
                    Cylinder(radius=2.2, height=5.4, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)
    return p.part


def make_standoff_feet(slots=ALL_SLOTS):
    """M3 10mm Brass Hex Standoffs (4 per 1U slot) with Ø6.2mm locator bosses."""
    hex_geom = hex_prism(5.5, Z_STANDOFF_H)
    with BuildPart() as p:
        for cx, cy in slots:
            for sx in (-HALF_P, HALF_P):
                for sy in (-HALF_P, HALF_P):
                    px, py = cx + sx, cy + sy
                    with Locations((px, py, 0.0)):
                        Cylinder(radius=3.10, height=LOCATOR_D, align=(Align.CENTER, Align.CENTER, Align.MAX))
                        p.part += hex_geom.translate((px, py, 0.0))
                    # Top M3 internal bore (clearance r=1.55 so bolts sit with zero geometric interference)
                    with Locations((px, py, 2.0)):
                        Cylinder(radius=1.55, height=Z_STANDOFF_H - 1.8, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)
    return p.part


def _add_bolt_at(px, py, z_bot, z_head_base, head_h=2.5):
    with BuildPart() as b:
        with Locations((px, py, z_bot)):
            Cylinder(radius=1.42, height=z_head_base - z_bot, align=(Align.CENTER, Align.CENTER, Align.MIN))
        with Locations((px, py, z_head_base)):
            Cylinder(radius=2.65, height=head_h, align=(Align.CENTER, Align.CENTER, Align.MIN))
        with Locations((px, py, z_head_base + 0.9)):
            Cylinder(radius=1.35, height=head_h, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)
    return b.part


def make_m3_bolts(bolt_tops=None):
    """All M3 clamping bolts across the 1U slots."""
    if bolt_tops is None:
        bolt_tops = {
            POS_VALVE: Z_TOP_BOLT,    # 27.5
            POS_MIX: Z_TOP_BOLT,      # 27.5
            POS_THERMO: Z_STANDOFF_H + 4.0,  # 14.0
            POS_SENSE: Z_STANDOFF_H + 4.0,   # 14.0
            POS_DRIP: Z_STANDOFF_H + 4.0,    # 14.0
            POS_ELEC: Z_STANDOFF_H + 8.5,    # 18.5
        }
    bolts = []
    for slot, z_hb in bolt_tops.items():
        cx, cy = slot
        for sx in (-HALF_P, HALF_P):
            for sy in (-HALF_P, HALF_P):
                bolts.append(_add_bolt_at(cx + sx, cy + sy, 3.2, z_hb))
    from build123d import Compound
    return Compound(children=bolts)


def _subtract_4_corner_bolts(cx, cy, z0, h, r=1.65):
    for sx in (-HALF_P, HALF_P):
        for sy in (-HALF_P, HALF_P):
            with Locations((cx + sx, cy + sy, z0)):
                Cylinder(radius=r, height=h, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)


# ---------------------------------------------------------------------------
# Slot 1: [VALVE] (-50, +25)
# ---------------------------------------------------------------------------
VALVE_PORTS = [(POS_VALVE[0] - 15.0, POS_VALVE[1]), (POS_VALVE[0] + 15.0, POS_VALVE[1])]


def make_valve_module(pos=POS_VALVE):
    cx, cy = pos
    ports = [(cx - 15.0, cy), (cx + 15.0, cy)]
    # Base block (Z=10..20)
    with BuildPart() as base:
        with Locations((cx, cy, Z_STANDOFF_H)):
            Box(UNIT_W, UNIT_W, BASE_H, align=(Align.CENTER, Align.CENTER, Align.MIN))
        chamfer(base.edges(), length=0.35)
        with Locations((cx, cy, Z_GASKET - CH_D / 2.0)):
            Box(34.0, CH_W, CH_D + 0.2, mode=Mode.SUBTRACT)
        with Locations((cx, cy, Z_GASKET)):
            Cylinder(radius=4.6, height=6.0, align=(Align.CENTER, Align.CENTER, Align.MAX), mode=Mode.SUBTRACT)
        for px, py in ports:
            with Locations((px, py, Z_GASKET - CH_D)):
                Cylinder(radius=0.60, height=0.5, align=(Align.CENTER, Align.CENTER, Align.MAX), mode=Mode.SUBTRACT)
        _subtract_4_corner_bolts(cx, cy, Z_STANDOFF_H - 0.1, BASE_H + 0.2)

    # Cutout Silicone Gasket (Z=20..20.8) with Ø4.0mm port relief holes
    with BuildPart() as gasket:
        with Locations((cx, cy, Z_GASKET)):
            Box(UNIT_W - 2.0, UNIT_W - 2.0, GASKET_H, align=(Align.CENTER, Align.CENTER, Align.MIN))
            # Relief cutouts in +Y and -Y non-channel zones so clamping force concentrates on rim & channel
            for sy in (-13.0, 13.0):
                with Locations((cx, cy + sy, Z_GASKET - 0.1)):
                    Box(14.0, 10.0, GASKET_H + 0.2, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)
        with Locations((cx, cy, Z_GASKET - 0.1)):
            Cylinder(radius=4.8, height=GASKET_H + 0.2, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)
        for px, py in ports:
            with Locations((px, py, Z_GASKET - 0.1)):
                Cylinder(radius=2.0, height=GASKET_H + 0.2, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)
        _subtract_4_corner_bolts(cx, cy, Z_GASKET - 0.1, GASKET_H + 0.2, r=1.8)

    # 5mm PMMA Lid (Z=20.8..25.8) with Ø3.0mm reamed vertical ports
    with BuildPart() as lid:
        with Locations((cx, cy, Z_LID)):
            Box(UNIT_W, UNIT_W, LID_H, align=(Align.CENTER, Align.CENTER, Align.MIN))
        chamfer(lid.edges(), length=0.3)
        with Locations((cx, cy, Z_LID - 0.1)):
            Cylinder(radius=2.6, height=LID_H + 0.2, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)
        for px, py in ports:
            with Locations((px, py, Z_LID - 0.1)):
                Cylinder(radius=1.50, height=LID_H + 0.2, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)
        _subtract_4_corner_bolts(cx, cy, Z_LID - 0.1, LID_H + 0.2)

    # Top Acrylic Retainer Plate (Z=25.8..27.5) with Ø1.8mm tube pass-throughs
    with BuildPart() as retainer:
        with Locations((cx, cy, Z_RETAIN)):
            Box(UNIT_W, 33.0, RETAIN_H, align=(Align.CENTER, Align.CENTER, Align.MIN))
        chamfer(retainer.edges(), length=0.2)
        with Locations((cx, cy, Z_RETAIN - 0.1)):
            Cylinder(radius=5.0, height=RETAIN_H + 0.2, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)
        for px, py in ports:
            with Locations((px, py, Z_RETAIN - 0.1)):
                Cylinder(radius=0.90, height=RETAIN_H + 0.2, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)
        _subtract_4_corner_bolts(cx, cy, Z_RETAIN - 0.1, RETAIN_H + 0.2)

    # Disposable Stopcock + Handle
    with BuildPart() as stopcock:
        with Locations((cx, cy, Z_GASKET - 5.5)):
            Cylinder(radius=4.3, height=5.8, align=(Align.CENTER, Align.CENTER, Align.MIN))
        with Locations((cx, cy, Z_GASKET + 0.3)):
            Cylinder(radius=2.2, height=7.8, align=(Align.CENTER, Align.CENTER, Align.MIN))
    with BuildPart() as handle:
        z_h = Z_GASKET + 8.3
        with Locations((cx, cy, z_h)):
            Cylinder(radius=4.2, height=2.2, align=(Align.CENTER, Align.CENTER, Align.MIN))
        with Locations((cx - 3.5, cy, z_h + 2.2)):
            Box(12.5, 4.0, 4.5, align=(Align.CENTER, Align.CENTER, Align.MIN))
        chamfer(handle.edges(), length=0.25)
        with Locations((cx, cy, z_h - 0.1)):
            Cylinder(radius=2.35, height=1.6, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)

    return base.part, gasket.part, lid.part, retainer.part, stopcock.part, handle.part


# ---------------------------------------------------------------------------
# Slot 2: [MIX] (0, +25)
# ---------------------------------------------------------------------------
MIX_PORTS = [
    (POS_MIX[0] - 15.0, POS_MIX[1]),
    (POS_MIX[0] + 15.0, POS_MIX[1]),
    (POS_MIX[0], POS_MIX[1] + 15.0),
    (POS_MIX[0], POS_MIX[1] - 15.0),
]


def make_mix_module(pos=POS_MIX):
    cx, cy = pos
    ports = [
        (cx - 15.0, cy),
        (cx + 15.0, cy),
        (cx, cy + 15.0),
        (cx, cy - 15.0),
    ]
    with BuildPart() as base:
        with Locations((cx, cy, Z_STANDOFF_H)):
            Box(UNIT_W, UNIT_W, BASE_H, align=(Align.CENTER, Align.CENTER, Align.MIN))
        chamfer(base.edges(), length=0.35)
        with Locations((cx, cy, Z_GASKET - CH_D / 2.0)):
            Box(34.0, CH_W, CH_D + 0.2, mode=Mode.SUBTRACT)
            Box(CH_W, 34.0, CH_D + 0.2, mode=Mode.SUBTRACT)
        for px, py in ports:
            with Locations((px, py, Z_GASKET - CH_D)):
                Cylinder(radius=0.60, height=0.5, align=(Align.CENTER, Align.CENTER, Align.MAX), mode=Mode.SUBTRACT)
        _subtract_4_corner_bolts(cx, cy, Z_STANDOFF_H - 0.1, BASE_H + 0.2)

    with BuildPart() as gasket:
        with Locations((cx, cy, Z_GASKET)):
            Box(UNIT_W - 2.0, UNIT_W - 2.0, GASKET_H, align=(Align.CENTER, Align.CENTER, Align.MIN))
        # Cutout 4 non-wetted quadrants inside the frame so gasket pressure focuses around the cross channel
        for sx in (-1, 1):
            for sy in (-1, 1):
                with Locations((cx + sx * 6.5, cy + sy * 6.5, Z_GASKET - 0.1)):
                    Box(7.2, 7.2, GASKET_H + 0.2, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)
        for px, py in ports:
            with Locations((px, py, Z_GASKET - 0.1)):
                Cylinder(radius=2.0, height=GASKET_H + 0.2, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)
        _subtract_4_corner_bolts(cx, cy, Z_GASKET - 0.1, GASKET_H + 0.2, r=1.8)

    with BuildPart() as lid:
        with Locations((cx, cy, Z_LID)):
            Box(UNIT_W, UNIT_W, LID_H, align=(Align.CENTER, Align.CENTER, Align.MIN))
        chamfer(lid.edges(), length=0.3)
        for px, py in ports:
            with Locations((px, py, Z_LID - 0.1)):
                Cylinder(radius=1.50, height=LID_H + 0.2, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)
        _subtract_4_corner_bolts(cx, cy, Z_LID - 0.1, LID_H + 0.2)

    with BuildPart() as retainer:
        with Locations((cx, cy, Z_RETAIN)):
            Box(UNIT_W, UNIT_W, RETAIN_H, align=(Align.CENTER, Align.CENTER, Align.MIN))
        # Large central observation diamond/window + 4 corner relief cutouts leaving cross-arms for 4 ports
        for sx in (-1, 1):
            for sy in (-1, 1):
                with Locations((cx + sx * 6.0, cy + sy * 6.0, Z_RETAIN - 0.1)):
                    Box(7.5, 7.5, RETAIN_H + 0.2, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)
        chamfer(retainer.edges(), length=0.15)
        for px, py in ports:
            with Locations((px, py, Z_RETAIN - 0.1)):
                Cylinder(radius=0.90, height=RETAIN_H + 0.2, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)
        _subtract_4_corner_bolts(cx, cy, Z_RETAIN - 0.1, RETAIN_H + 0.2)

    return base.part, gasket.part, lid.part, retainer.part


# ---------------------------------------------------------------------------
# Slot 3: [THERMO] (+25, 0) — Water-Bath Cup + Laser Cross-Bobbin + FEP Coil
# ---------------------------------------------------------------------------
THERMO_PORTS = [(POS_THERMO[0] - 9.8, POS_THERMO[1]), (POS_THERMO[0] + 9.8, POS_THERMO[1])]


def make_thermo_module(pos=POS_THERMO):
    cx, cy = pos
    ports = [(cx - 9.8, cy), (cx + 9.8, cy)]
    # Base flange plate (Z=10..14) with pocket clearing 4 corner M3 bolts (r_bolt_inner = 15.03mm)
    with BuildPart() as tbase:
        with Locations((cx, cy, Z_STANDOFF_H)):
            Box(UNIT_W, UNIT_W, 4.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
        chamfer(tbase.edges(), length=0.3)
        with Locations((cx, cy, Z_STANDOFF_H + 2.4)):
            Cylinder(radius=14.6, height=2.0, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)
        _subtract_4_corner_bolts(cx, cy, Z_STANDOFF_H - 0.1, 4.2)

    # Translucent PP Water-Bath Cup (Z=12.5..38.0, OD=28.8mm < 30.05mm inner bolt circle)
    with BuildPart() as cup:
        with Locations((cx, cy, Z_STANDOFF_H + 2.5)):
            Cylinder(radius=14.4, height=25.5, align=(Align.CENTER, Align.CENTER, Align.MIN))
        with Locations((cx, cy, Z_STANDOFF_H + 4.2)):
            Cylinder(radius=12.8, height=24.2, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)

    # Laser-cut Cross-Bobbin inside cup (Z=14.6..34.8)
    with BuildPart() as bobbin:
        with Locations((cx, cy, Z_STANDOFF_H + 4.6)):
            Box(17.6, 2.2, 20.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
            Box(2.2, 17.6, 20.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
            # Bottom spacer ring keeping bobbin off floor
            Cylinder(radius=11.2, height=1.5, align=(Align.CENTER, Align.CENTER, Align.MIN))
        with Locations((cx, cy, Z_STANDOFF_H + 4.4)):
            Cylinder(radius=4.5, height=21.0, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)

    # Central Immersion Heater + Thermistor Probe (shaft Z=14.8..41.2 through lid hole, collar Z=41.25..44.0)
    with BuildPart() as heater:
        with Locations((cx, cy, Z_STANDOFF_H + 4.8)):
            Cylinder(radius=3.8, height=26.4, align=(Align.CENTER, Align.CENTER, Align.MIN))
        with Locations((cx, cy, 41.25)):
            Cylinder(radius=5.2, height=2.75, align=(Align.CENTER, Align.CENTER, Align.MIN))

    # Submerged Helical FEP Coil inside bath around bobbin (coil radius=9.8, Z=17.5..33.7)
    with BuildPart() as coil:
        with BuildLine() as hl:
            h_path = Helix(pitch=3.6, height=16.2, radius=9.8, center=(cx, cy, 17.5))
        with BuildSketch(Plane(origin=h_path.position_at(0), z_dir=h_path.tangent_at(0))):
            Circle(0.80)
        sweep(path=hl.line)

    # Insulated Lid (Z=38.0..41.2, radius=14.8 < 15.03mm bolt clearance)
    with BuildPart() as tlid:
        with Locations((cx, cy, 38.0)):
            Cylinder(radius=14.8, height=3.2, align=(Align.CENTER, Align.CENTER, Align.MIN))
        chamfer(tlid.edges(), length=0.25)
        with Locations((cx, cy, 37.9)):
            Cylinder(radius=4.1, height=3.5, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)
        for px, py in ports:
            with Locations((px, py, 37.9)):
                Cylinder(radius=1.50, height=3.5, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)

    return tbase.part, cup.part, bobbin.part, heater.part, coil.part, tlid.part


# ---------------------------------------------------------------------------
# Slot 4: [SENSE] (+75, 0) — Matte-Black Optical Clip + TCS34725 + Aperture Slit
# ---------------------------------------------------------------------------
SENSE_PORTS = [(POS_SENSE[0] - 15.0, POS_SENSE[1]), (POS_SENSE[0] + 15.0, POS_SENSE[1])]


def make_sense_module(pos=POS_SENSE):
    cx, cy = pos
    ports = [(cx - 15.0, cy), (cx + 15.0, cy)]
    # Dark light-tight enclosure (Y=±9.2 < ±9.85mm M3 bolt inner edge) + corner bolt flange (Z=10..14)
    with BuildPart() as box:
        with Locations((cx, cy, Z_STANDOFF_H)):
            Box(UNIT_W, UNIT_W, 4.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
            Box(38.0, 18.4, 14.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
        chamfer(box.edges(), length=0.25)
        # Light-tight chamber interior + optical U-slot for FEP tube & 0.6mm slit aperture
        with Locations((cx, cy, Z_STANDOFF_H + 3.0)):
            Box(33.0, 14.0, 11.5, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)
        _subtract_4_corner_bolts(cx, cy, Z_STANDOFF_H - 0.1, 4.2)

    # TCS34725 PCB + White LED facing the FEP sensing line
    with BuildPart() as pcb:
        with Locations((cx, cy - 3.5, Z_STANDOFF_H + 3.5)):
            Box(20.0, 2.0, 10.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
        with Locations((cx, cy + 3.5, Z_STANDOFF_H + 3.5)):
            Box(10.0, 2.0, 8.0, align=(Align.CENTER, Align.CENTER, Align.MIN))

    # Top matte-black cover (Z=24..27) with Ø3.0mm press-fit ports
    with BuildPart() as lid:
        with Locations((cx, cy, Z_STANDOFF_H + 14.0)):
            Box(38.0, 18.4, 3.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
        chamfer(lid.edges(), length=0.2)
        for px, py in ports:
            with Locations((px, py, Z_STANDOFF_H + 13.9)):
                Cylinder(radius=1.50, height=3.2, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)

    return box.part, pcb.part, lid.part


# ---------------------------------------------------------------------------
# Slot 5: [DRIP] (+125, 0) — M3 Backpressure Pinch + 1.5 mL Microtube Cradle
# ---------------------------------------------------------------------------
DRIP_PORT = (POS_DRIP[0], POS_DRIP[1])


def make_drip_module(pos=POS_DRIP):
    cx, cy = pos
    # Open-front collection tower frame on 4 standoffs (Z=10..38, pillar X=±9.2 < ±9.85mm bolt edge)
    with BuildPart() as frame:
        # Baseplate flange (Z=10..14) with central microtube cup pocket
        with Locations((cx, cy, Z_STANDOFF_H)):
            Box(UNIT_W, UNIT_W, 4.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
        with Locations((cx, cy, Z_STANDOFF_H + 1.5)):
            Cylinder(radius=5.8, height=3.0, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)
        # Rear bridge & upper BPR Pinch Head Block narrowed in X to 18.4mm so rear M3 bolts clear cleanly
        with Locations((cx, cy + 16.0, Z_STANDOFF_H + 4.0)):
            Box(18.4, 8.0, 24.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
        with Locations((cx, cy + 5.0, 30.0)):
            Box(18.4, 24.0, 8.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
        chamfer(frame.edges(), length=0.25)
        # Vertical pinch bore at (cx, cy, Z=29.8..38.2)
        with Locations((cx, cy, 29.8)):
            Cylinder(radius=1.60, height=8.4, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)
        # Horizontal M3 BPR thumbscrew bore from +X into pinch sleeve
        with Locations(Plane.YZ.offset(cx + 9.5)):
            with Locations((cy, 34.0)):
                Cylinder(radius=1.55, height=8.5, align=(Align.CENTER, Align.CENTER, Align.MAX), mode=Mode.SUBTRACT)
        _subtract_4_corner_bolts(cx, cy, Z_STANDOFF_H - 0.1, 4.2)

    # M3 Backpressure Thumbscrew (entering from +X)
    with BuildPart() as bpr:
        with Locations(Plane.YZ.offset(cx + 15.0)):
            with Locations((cy, 34.0)):
                Cylinder(radius=1.42, height=13.5, align=(Align.CENTER, Align.CENTER, Align.MAX))
                Cylinder(radius=4.5, height=4.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
        chamfer(bpr.edges(), length=0.2)

    # 1.5mL Polypropylene Microtube seated in base cradle (Z=11.8..27.0, tapered bottom + collar)
    with BuildPart() as tube:
        with Locations((cx, cy, Z_STANDOFF_H + 2.0)):
            Cylinder(radius=4.2, height=6.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
        with Locations((cx, cy, Z_STANDOFF_H + 8.0)):
            Cylinder(radius=5.2, height=9.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
        with Locations((cx, cy, Z_STANDOFF_H + 17.0)):
            Cylinder(radius=6.2, height=1.2, align=(Align.CENTER, Align.CENTER, Align.MIN))
        with Locations((cx, cy, Z_STANDOFF_H + 4.0)):
            Cylinder(radius=4.4, height=15.0, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)

    return frame.part, bpr.part, tube.part


# ---------------------------------------------------------------------------
# Slot 6: [ELEC_BAY] (-50, -25) — Isolated XIAO ESP32-S3 + I2C Bus
# ---------------------------------------------------------------------------
def make_elec_module(pos=POS_ELEC):
    cx, cy = pos
    with BuildPart() as box:
        with Locations((cx, cy, Z_STANDOFF_H)):
            Box(UNIT_W, UNIT_W, 8.5, align=(Align.CENTER, Align.CENTER, Align.MIN))
        chamfer(box.edges(), length=0.3)
        # Central pocket (38x38) with 4 corner M3 mounting bosses left solid for clean standoff bolting
        with Locations((cx, cy, Z_STANDOFF_H + 2.0)):
            Box(38.0, 38.0, 6.8, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)
        for sx in (-HALF_P, HALF_P):
            for sy in (-HALF_P, HALF_P):
                with Locations((cx + sx, cy + sy, Z_STANDOFF_H + 2.0)):
                    Cylinder(radius=3.2, height=6.5, align=(Align.CENTER, Align.CENTER, Align.MIN))
        # Front -Y USB-C U-slot
        with Locations((cx, cy - 21.0, Z_STANDOFF_H + 6.2)):
            Box(10.5, 5.0, 4.8, mode=Mode.SUBTRACT)
        # Front -Y I2C 4-pin header slot
        with Locations((cx + 14.5, cy - 21.5, Z_STANDOFF_H + 4.4)):
            Box(12.6, 4.0, 4.8, mode=Mode.SUBTRACT)
        _subtract_4_corner_bolts(cx, cy, Z_STANDOFF_H - 0.1, 9.0)

    with BuildPart() as pcb:
        with Locations((cx, cy, Z_STANDOFF_H + 2.2)):
            Box(36.0, 36.0, 1.6, align=(Align.CENTER, Align.CENTER, Align.MIN))
        # Corner clearance cutouts around M3 bosses (radius 3.4mm)
        _subtract_4_corner_bolts(cx, cy, Z_STANDOFF_H + 2.0, 2.0, r=3.5)
        with Locations((cx + 13.5, cy + 4.0, Z_STANDOFF_H + 3.8)):
            Box(6.0, 9.0, 1.8, align=(Align.CENTER, Align.CENTER, Align.MIN))

    with BuildPart() as xiao:
        # Centered in X inside (±8.75) clearly separated from PCB Z=13.80 & aux IC
        with Locations((cx, cy - 5.5, Z_STANDOFF_H + 3.85)):
            Box(17.5, 21.0, 1.4, align=(Align.CENTER, Align.CENTER, Align.MIN))
        with Locations((cx, cy - 3.5, Z_STANDOFF_H + 5.25)):
            Box(11.5, 11.0, 1.8, align=(Align.CENTER, Align.CENTER, Align.MIN))
        with Locations((cx, cy - 16.0, Z_STANDOFF_H + 4.25)):
            Box(9.0, 7.2, 2.6, align=(Align.CENTER, Align.CENTER, Align.MIN))

    with BuildPart() as i2c_h:
        with Locations((cx + 14.5, cy - 22.8, Z_STANDOFF_H + 4.4)):
            Box(12.4, 5.0, 4.6)
            with Locations((0, -1.5, 0)):
                Box(10.6, 3.2, 3.2, mode=Mode.SUBTRACT)
        for i in range(4):
            with Locations((cx + 14.5 + (i - 1.5) * 2.54, cy - 23.6, Z_STANDOFF_H + 4.4)):
                Box(0.70, 5.2, 0.70, mode=Mode.SUBTRACT)

    with BuildPart() as i2c_p:
        for i in range(4):
            with Locations((cx + 14.5 + (i - 1.5) * 2.54, cy - 23.6, Z_STANDOFF_H + 4.4)):
                Box(0.64, 4.0, 0.64)

    return box.part, pcb.part, xiao.part, i2c_h.part, i2c_p.part


# ---------------------------------------------------------------------------
# Universal Silicone Sleeves & Arbitrary-Order Overhead FEP Jumper Tubes
# ---------------------------------------------------------------------------
def make_sleeves(port_specs=None):
    """Nominal OD 2.94mm silicone press-fit sleeves flush inside every Ø3.0mm lid port hole."""
    if port_specs is None:
        port_specs = (
            [(x, y, Z_LID, LID_H - 0.05) for x, y in VALVE_PORTS]
            + [(x, y, Z_LID, LID_H - 0.05) for x, y in MIX_PORTS]
            + [(x, y, 37.8, 3.4) for x, y in THERMO_PORTS]
            + [(x, y, 23.5, 3.4) for x, y in SENSE_PORTS]
            + [(DRIP_PORT[0], DRIP_PORT[1], 29.8, 8.2)]
        )
    with BuildPart() as p:
        for px, py, z0, h in port_specs:
            with Locations((px, py, z0)):
                Cylinder(radius=1.47, height=h, align=(Align.CENTER, Align.CENTER, Align.MIN))
                Cylinder(radius=0.86, height=h + 0.4, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)
    return p.part


def _arch_tube(p0, p1, apex_z, radius=0.80, lead0=11.0, lead1=11.0):
    from build123d import Line
    x0, y0, z0 = p0
    x1, y1, z1 = p1
    p0_top = (x0, y0, z0 + lead0)
    p1_top = (x1, y1, z1 + lead1)
    mx, my = (x0 + x1) * 0.5, (y0 + y1) * 0.5
    arch_pts = [
        p0_top,
        (x0 * 0.72 + x1 * 0.28, y0 * 0.72 + y1 * 0.28, apex_z - 1.5),
        (mx, my, apex_z),
        (x0 * 0.28 + x1 * 0.72, y0 * 0.28 + y1 * 0.72, apex_z - 1.5),
        p1_top,
    ]
    with BuildPart() as p:
        with BuildLine() as ln:
            Line((x0, y0, z0), p0_top)
            Spline(arch_pts, tangents=((0, 0, 1), (0, 0, -1)))
            Line(p1_top, (x1, y1, z1))
        with BuildSketch(Plane(origin=(x0, y0, z0), z_dir=(0, 0, 1))):
            Circle(radius)
        sweep(path=ln.wires()[0])
    return p.part


def make_jumper_tubes():
    """Linear left-to-right patching: VALVE -> MIX -> THERMO -> SENSE -> DRIP."""
    from build123d import Compound
    tubes = [
        # External Inlet Stub -> VALVE IN (clean rear feed clearing ELEC_BAY)
        _arch_tube((VALVE_PORTS[0][0], VALVE_PORTS[0][1] + 24.0, 18.0), (VALVE_PORTS[0][0], VALVE_PORTS[0][1], 19.0), apex_z=40.0, lead0=8.0, lead1=12.0),
        # VALVE OUT -> MIX W
        _arch_tube((VALVE_PORTS[1][0], VALVE_PORTS[1][1], 19.0), (MIX_PORTS[0][0], MIX_PORTS[0][1], 19.0), apex_z=42.0, lead0=12.0, lead1=12.0),
        # Second Reagent Feed -> MIX N
        _arch_tube((MIX_PORTS[2][0], MIX_PORTS[2][1] + 15.0, 18.0), (MIX_PORTS[2][0], MIX_PORTS[2][1], 19.0), apex_z=40.0, lead0=8.0, lead1=12.0),
        # MIX E -> THERMO IN
        _arch_tube((MIX_PORTS[1][0], MIX_PORTS[1][1], 19.0), (THERMO_PORTS[0][0], THERMO_PORTS[0][1], 36.0), apex_z=56.0, lead0=12.0, lead1=11.0),
        # THERMO OUT -> SENSE IN (left-to-right straight rail)
        _arch_tube((THERMO_PORTS[1][0], THERMO_PORTS[1][1], 36.0), (SENSE_PORTS[0][0], SENSE_PORTS[0][1], 21.0), apex_z=56.0, lead0=11.0, lead1=11.0),
        # SENSE OUT -> DRIP IN
        _arch_tube((SENSE_PORTS[1][0], SENSE_PORTS[1][1], 21.0), (DRIP_PORT[0], DRIP_PORT[1], 27.8), apex_z=52.0, lead0=11.0, lead1=14.0),
    ]
    return Compound(children=tubes)


def _wire_run(pts, radius):
    """Build a smooth insulated cable run with straight tangent control."""
    with BuildPart() as p:
        with BuildLine() as ln:
            sp = Spline(pts)
        with BuildSketch(Plane(origin=sp.position_at(0), z_dir=sp.tangent_at(0))):
            Circle(radius)
        sweep(path=ln.line)
    return p.part


def make_wiring_harness():
    """3D Sensor I2C cable (ELEC -> SENSE) & Silicone Heater/Thermistor cable (ELEC -> THERMO) + mating plugs."""
    # 1. Mating connector housings (black nylon plugs + heater cap boot)
    with BuildPart() as plugs:
        # [ELEC] front 4-pin I2C mating plug (clear of i2c_pins ending at Y=-25.6)
        with Locations((POS_ELEC[0] + 14.5, POS_ELEC[1] - 27.6, Z_STANDOFF_H + 4.4)):
            Box(11.0, 3.6, 4.2)
        # [ELEC] rear 2-pin heater power plug (clear of elec_bus_base Y=+23.0)
        with Locations((POS_ELEC[0] + 12.0, POS_ELEC[1] + 25.4, Z_STANDOFF_H + 4.8)):
            Box(8.4, 3.6, 4.2)
        # [THERMO] top cartridge heater insulated boot (seated above heater collar Z=44.0)
        with Locations((POS_THERMO[0], POS_THERMO[1], 44.05)):
            Cylinder(radius=3.2, height=3.2, align=(Align.CENTER, Align.CENTER, Align.MIN))
        # [SENSE] front I2C plug mated flush against optical housing front (-Y wall at Y=-9.2, above Z=14 flange)
        with Locations((POS_SENSE[0], POS_SENSE[1] - 11.4, 17.5)):
            Box(9.2, 3.8, 3.8)

    # 2. Sensor I2C 4-conductor cable (stays clear at Y=-29.5, Z=7.0 and rises above Z=14 flange before crossing Y=-23.0)
    s_pts = [
        (POS_ELEC[0] + 14.5, POS_ELEC[1] - 29.3, 14.4),
        (POS_ELEC[0] + 17.0, POS_ELEC[1] - 29.8, 10.5),
        (POS_ELEC[0] + 24.0, -29.5, 7.0),
        (-80.0, -29.5, 7.0),
        (-25.0, -29.5, 7.0),
        (25.0, -29.5, 7.0),
        (61.0, -29.5, 7.4),
        (71.0, -27.5, 13.8),
        (74.5, -24.5, 17.2),
        (POS_SENSE[0], -18.0, 17.5),
        (POS_SENSE[0], POS_SENSE[1] - 13.2, 17.5),
    ]
    sensor_cable = _wire_run(s_pts, radius=1.15) - plugs.part

    # 3. Heater + Thermistor high-temp silicone cable (rear rail Y=+29.5, Z=7.0 -> THERMO top boot)
    h_pts = [
        (POS_ELEC[0] + 12.0, POS_ELEC[1] + 27.1, 14.8),
        (POS_ELEC[0] + 15.0, POS_ELEC[1] + 29.3, 10.5),
        (POS_ELEC[0] + 25.0, 29.5, 7.0),
        (-75.0, 29.5, 7.0),
        (-25.0, 29.5, 7.0),
        (13.0, 29.5, 7.4),
        (23.0, 28.0, 24.0),
        (24.5, 25.8, 45.0),
        (POS_THERMO[0], 14.0, 50.0),
        (POS_THERMO[0], POS_THERMO[1], 47.0),
    ]
    heater_cable = _wire_run(h_pts, radius=1.20) - plugs.part

    return plugs.part, sensor_cable, heater_cable


def export_sdl_manifest():
    manifest = {
        "standard": "BonFlow-Open-v4.0",
        "layout": "linear_1x6_rail",
        "mechanical_grid": {
            "pitch_mm": PITCH,
            "baseplate_hole_diameter_mm": 6.5,
            "standoff_type": "M3 Hex Standoff 10.0mm",
            "port_interface": "Vertical Press-Fit Ø3.0mm Reamed Lid + OD 3.2/ID 1.5mm Silicone Sleeve + Retainer + OD 1.6mm FEP",
        },
        "modules": [
            {"id": "elec_1u", "slot": list(POS_ELEC), "controller": "Seeed_XIAO_ESP32S3", "bus": "I2C_3.3V_USBSerial_JSON"},
            {"id": "valve_1u", "slot": list(POS_VALVE), "wetted_materials": ["PMMA", "Silicone", "PP_Stopcock"], "internal_volume_uL": 145.0},
            {"id": "mix_1u", "slot": list(POS_MIX), "wetted_materials": ["PMMA", "Silicone"], "internal_volume_uL": 164.6},
            {"id": "thermo_1u", "slot": list(POS_THERMO), "wetted_materials": ["FEP"], "internal_volume_uL": 240.0, "temp_range_C": [25, 95]},
            {"id": "sense_1u", "slot": list(POS_SENSE), "wetted_materials": ["FEP"], "internal_volume_uL": 18.0, "sensor": "TCS34725_I2C"},
            {"id": "drip_1u", "slot": list(POS_DRIP), "wetted_materials": ["FEP", "Silicone", "PP_Microtube"], "bpr_adjustable": True},
        ],
        "default_topology": ["valve_1u.OUT -> mix_1u.W", "mix_1u.E -> thermo_1u.IN", "thermo_1u.OUT -> sense_1u.IN", "sense_1u.OUT -> drip_1u.IN"],
    }
    out = OUT_DIR.parent / "fluidic_graph.json"
    out.write_text(json.dumps(manifest, indent=2))
    print(f"Exported SDL manifest: {out}")


def check_interferences(parts):
    """Exhaustively audit all N*(N-1)/2 pairs with exact AABB + solid distance pruning."""
    names = list(parts.keys())
    solids_map = {
        k: (list(parts[k].solids()) if hasattr(parts[k], "solids") and parts[k].solids() else [parts[k]])
        for k in names
    }
    bboxes = {k: parts[k].bounding_box() for k in names}

    def disjoint(b1, b2, tol=1e-4):
        return (
            b1.max.X < b2.min.X - tol or b2.max.X < b1.min.X - tol
            or b1.max.Y < b2.min.Y - tol or b2.max.Y < b1.min.Y - tol
            or b1.max.Z < b2.min.Z - tol or b2.max.Z < b1.min.Z - tol
        )

    def vol_inter(a, b):
        if disjoint(bboxes[a], bboxes[b]):
            return 0.0
        tot = 0.0
        for sa in solids_map[a]:
            ba = sa.bounding_box()
            for sb in solids_map[b]:
                bb = sb.bounding_box()
                if disjoint(ba, bb):
                    continue
                b_inside = (
                    (ba.min.X >= bb.min.X and ba.max.X <= bb.max.X and ba.min.Y >= bb.min.Y and ba.max.Y <= bb.max.Y and ba.min.Z >= bb.min.Z and ba.max.Z <= bb.max.Z)
                    or (bb.min.X >= ba.min.X and bb.max.X <= ba.max.X and bb.min.Y >= ba.min.Y and bb.max.Y <= ba.max.Y and bb.min.Z >= ba.min.Z and bb.max.Z <= ba.max.Z)
                )
                if not b_inside and sa.distance(sb) > 0.01:
                    continue
                inter = sa.intersect(sb)
                if inter is None:
                    continue
                if hasattr(inter, "solids"):
                    tot += sum(s.volume for s in inter.solids())
                else:
                    tot += getattr(inter, "volume", 0.0)
        return tot

    bad = 0
    checked = 0
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = names[i], names[j]
            checked += 1
            v = vol_inter(a, b)
            if v > 0.01:
                bad += 1
                print(f"COLLISION: {a} vs {b}: {v:.4f} mm^3", flush=True)
    print(f"Audited {checked} pairs across {len(names)} parts: {bad} collisions > 0.01 mm^3", flush=True)
    return bad


def main():
    v_base, v_gask, v_lid, v_ret, v_sc, v_hnd = make_valve_module()
    m_base, m_gask, m_lid, m_ret = make_mix_module()
    t_base, t_cup, t_bob, t_htr, t_coil, t_lid = make_thermo_module()
    s_box, s_pcb, s_lid = make_sense_module()
    d_frm, d_bpr, d_tube = make_drip_module()
    e_box, e_pcb, e_xiao, e_i2ch, e_i2cp = make_elec_module()
    w_plugs, w_sensor, w_heater = make_wiring_harness()

    parts = {
        "base_plate": make_base_plate(),
        "standoff_feet": make_standoff_feet(),
        "m3_bolts": make_m3_bolts(),
        "sleeves": make_sleeves(),
        "jumper_tubes": make_jumper_tubes(),
        "wire_plugs": w_plugs,
        "sensor_wires": w_sensor,
        "heater_wires": w_heater,
        "valve_base": v_base,
        "valve_gasket": v_gask,
        "valve_lid": v_lid,
        "valve_retainer": v_ret,
        "disposable_stopcock": v_sc,
        "valve_handle": v_hnd,
        "mix_base": m_base,
        "mix_gasket": m_gask,
        "mix_lid": m_lid,
        "mix_retainer": m_ret,
        "thermo_base": t_base,
        "thermo_cup": t_cup,
        "thermo_bobbin": t_bob,
        "thermo_heater": t_htr,
        "thermo_coil": t_coil,
        "thermo_lid": t_lid,
        "sense_housing": s_box,
        "sense_pcb": s_pcb,
        "sense_lid": s_lid,
        "drip_frame": d_frm,
        "bpr_screw": d_bpr,
        "microtube": d_tube,
        "elec_bus_base": e_box,
        "carrier_pcb": e_pcb,
        "xiao_s3": e_xiao,
        "i2c_housing": e_i2ch,
        "i2c_pins": e_i2cp,
    }

    for name, part in parts.items():
        stl_path = OUT_DIR / f"{name}.stl"
        export_stl(part, str(stl_path), tolerance=0.06, angular_tolerance=0.14)
        print(f"Exported {stl_path.name}")

    step_path = OUT_DIR.parent / "bonflow_v4_assembly.step"
    from build123d import Compound
    export_step(Compound(children=[v_base, m_base, t_base, d_frm, e_box]), str(step_path))
    print(f"Exported STEP: {step_path}")

    export_sdl_manifest()
    bad = check_interferences(parts)
    if bad:
        raise SystemExit(f"COLLISION FAILURE: {bad} pairs > 0.01 mm^3")


if __name__ == "__main__":
    main()
