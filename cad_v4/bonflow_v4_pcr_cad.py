"""Parametric 3D CAD assembly for BonFlow Standard v4.0-PCR (9-Unit Continuous-Flow PCR Rail):
- Linear Optical Rail Baseplate (466 x 68 x 5 mm, 18x2 array of Ø6.5mm holes on 25.0 mm pitch)
- 9 Universal 1U Slots on Y = 0.0 (50.0 mm center spacing), sealing the 95°C denaturation zone with dual flanking stopcock valves:
    (-200, 0) [ELEC]       -> Controller (XIAO ESP32-S3 + 3-zone Heater & I2C Bus)
    (-150, 0) [MIX]        -> Cross-Junction Mixer Chip
    (-100, 0) [VALVE-IN]   -> Upstream Flanking Valve (blocks thermal backflow into MIX)
    ( -50, 0) [THERMO-95]  -> 95°C Denaturation Water-Bath + FEP Coil
    (   0, 0) [VALVE-OUT]  -> Downstream Flanking Valve (isolates 95°C expansion / degassing)
    ( +50, 0) [THERMO-58]  -> 58°C Annealing Water-Bath + FEP Coil
    (+100, 0) [THERMO-72]  -> 72°C Extension Water-Bath + FEP Coil
    (+150, 0) [SENSE]      -> TCS34725 Real-Time Fluorescence / Colorimetric Detector
    (+200, 0) [DRIP]       -> M3 Backpressure Pinch + 1.5 mL Microtube Collector
- Exports STLs to parts_pcr/, STEP assembly, SDL 'fluidic_graph_pcr.json', and verifies 0.0000 mm^3 collision.
"""
import json
from pathlib import Path

from build123d import (
    Align,
    Box,
    BuildPart,
    Compound,
    Cylinder,
    Locations,
    export_step,
    export_stl,
)

import bonflow_v4_cad as base_v4

OUT_DIR = Path("/root/tw_rag/work/bizcon/cad_v4/parts_pcr")
OUT_DIR.mkdir(parents=True, exist_ok=True)

POS_ELEC = (-200.0, 0.0)
POS_MIX = (-150.0, 0.0)
POS_VALVE_IN = (-100.0, 0.0)
POS_THERMO_95 = (-50.0, 0.0)
POS_VALVE_OUT = (0.0, 0.0)
POS_THERMO_58 = (50.0, 0.0)
POS_THERMO_72 = (100.0, 0.0)
POS_SENSE = (150.0, 0.0)
POS_DRIP = (200.0, 0.0)

THERMO_ZONES = [
    ("thermo_95", POS_THERMO_95, 95, 26.2, -192.0),
    ("thermo_58", POS_THERMO_58, 58, 28.8, -188.0),
    ("thermo_72", POS_THERMO_72, 72, 31.4, -184.0),
]

ALL_SLOTS_PCR = [
    POS_ELEC,
    POS_MIX,
    POS_VALVE_IN,
    POS_THERMO_95,
    POS_VALVE_OUT,
    POS_THERMO_58,
    POS_THERMO_72,
    POS_SENSE,
    POS_DRIP,
]

MIX_PORTS = [
    (POS_MIX[0] - 15.0, 0.0),
    (POS_MIX[0] + 15.0, 0.0),
    (POS_MIX[0], 15.0),
    (POS_MIX[0], -15.0),
]
VALVE_IN_PORTS = [(POS_VALVE_IN[0] - 15.0, 0.0), (POS_VALVE_IN[0] + 15.0, 0.0)]
T95_PORTS = [(POS_THERMO_95[0] - 13.5, 0.0), (POS_THERMO_95[0] + 13.5, 0.0)]
VALVE_OUT_PORTS = [(POS_VALVE_OUT[0] - 15.0, 0.0), (POS_VALVE_OUT[0] + 15.0, 0.0)]
T58_PORTS = [(POS_THERMO_58[0] - 13.5, 0.0), (POS_THERMO_58[0] + 13.5, 0.0)]
T72_PORTS = [(POS_THERMO_72[0] - 13.5, 0.0), (POS_THERMO_72[0] + 13.5, 0.0)]
SENSE_PORTS = [(POS_SENSE[0] - 15.0, 0.0), (POS_SENSE[0] + 15.0, 0.0)]
DRIP_PORT = POS_DRIP


def make_pcr_sleeves():
    port_specs = (
        [(x, y, base_v4.Z_LID - 0.5, 6.0) for x, y in MIX_PORTS]
        + [(x, y, base_v4.Z_LID - 0.5, 6.0) for x, y in VALVE_IN_PORTS]
        + [(x, y, 37.5, 4.5) for x, y in T95_PORTS]
        + [(x, y, base_v4.Z_LID - 0.5, 6.0) for x, y in VALVE_OUT_PORTS]
        + [(x, y, 37.5, 4.5) for x, y in T58_PORTS]
        + [(x, y, 37.5, 4.5) for x, y in T72_PORTS]
        + [(x, y, 23.5, 4.5) for x, y in SENSE_PORTS]
        + [(DRIP_PORT[0], DRIP_PORT[1], 29.5, 8.8)]
    )
    return base_v4.make_sleeves(port_specs)


def make_pcr_jumper_tubes():
    tubes = [
        # External Reagent 1 -> MIX W (rear feed clear of ELEC)
        base_v4._arch_tube((MIX_PORTS[0][0], 24.0, 18.0), (MIX_PORTS[0][0], 0.0, 19.0), apex_z=40.0, lead0=8.0, lead1=12.0),
        # External Reagent 2 -> MIX N
        base_v4._arch_tube((MIX_PORTS[2][0], 30.0, 18.0), (MIX_PORTS[2][0], 15.0, 19.0), apex_z=40.0, lead0=8.0, lead1=12.0),
        # MIX E -> VALVE-IN IN
        base_v4._arch_tube((MIX_PORTS[1][0], 0.0, 19.0), (VALVE_IN_PORTS[0][0], 0.0, 19.0), apex_z=42.0, lead0=12.0, lead1=12.0),
        # VALVE-IN OUT -> THERMO-95 IN
        base_v4._arch_tube((VALVE_IN_PORTS[1][0], 0.0, 19.0), (T95_PORTS[0][0], 0.0, 36.0), apex_z=56.0, lead0=12.0, lead1=11.0),
        # THERMO-95 OUT -> VALVE-OUT IN
        base_v4._arch_tube((T95_PORTS[1][0], 0.0, 36.0), (VALVE_OUT_PORTS[0][0], 0.0, 19.0), apex_z=56.0, lead0=11.0, lead1=12.0),
        # VALVE-OUT OUT -> THERMO-58 IN
        base_v4._arch_tube((VALVE_OUT_PORTS[1][0], 0.0, 19.0), (T58_PORTS[0][0], 0.0, 36.0), apex_z=56.0, lead0=12.0, lead1=11.0),
        # THERMO-58 OUT -> THERMO-72 IN
        base_v4._arch_tube((T58_PORTS[1][0], 0.0, 36.0), (T72_PORTS[0][0], 0.0, 36.0), apex_z=54.0, lead0=11.0, lead1=11.0),
        # THERMO-72 OUT -> SENSE IN
        base_v4._arch_tube((T72_PORTS[1][0], 0.0, 36.0), (SENSE_PORTS[0][0], 0.0, 21.0), apex_z=56.0, lead0=11.0, lead1=11.0),
        # SENSE OUT -> DRIP IN
        base_v4._arch_tube((SENSE_PORTS[1][0], 0.0, 21.0), (DRIP_PORT[0], 0.0, 27.8), apex_z=52.0, lead0=11.0, lead1=14.0),
    ]
    return Compound(children=tubes)


def make_pcr_wiring_harness():
    # 1. Connector housings & 3 top heater boots
    with BuildPart() as plugs:
        # [ELEC] front 4-pin I2C mating plug
        with Locations((POS_ELEC[0] + 14.5, -27.8, base_v4.Z_STANDOFF_H + 4.4)):
            Box(11.0, 4.0, 4.2)
        # [ELEC] rear 3-zone heater manifold plug
        with Locations((POS_ELEC[0] + 12.0, 25.4, base_v4.Z_STANDOFF_H + 4.8)):
            Box(14.0, 4.4, 4.2)
        # Top cartridge heater insulated boots for 95C / 58C / 72C baths
        for _, pos, _, _, _ in THERMO_ZONES:
            with Locations((pos[0], pos[1], 42.6)):
                Cylinder(radius=3.2, height=3.4, align=(Align.CENTER, Align.CENTER, Align.MIN))
        # [SENSE] front I2C plug mated to optical housing (-Y=11.0)
        with Locations((POS_SENSE[0], -13.2, 16.5)):
            Box(9.2, 4.2, 3.8)

    # 2. Sensor I2C 4-conductor cable (front rail Y=-28.5, Z=6.2 -> [SENSE])
    s_pts = [
        (POS_ELEC[0] + 14.5, -29.6, 14.4),
        (POS_ELEC[0] + 19.0, -30.0, 11.2),
        (POS_ELEC[0] + 28.0, -28.5, 6.4),
        (-130.0, -28.5, 6.4),
        (-60.0, -28.5, 6.4),
        (10.0, -28.5, 6.4),
        (80.0, -28.5, 6.4),
        (POS_SENSE[0] - 14.0, -28.5, 6.4),
        (POS_SENSE[0] - 2.5, -22.5, 11.8),
        (POS_SENSE[0], -15.1, 16.5),
    ]
    sensor_cable = base_v4._wire_run(s_pts, radius=1.15)

    # 3. 3-Zone Silicone Heater + Thermistor cables (rear separated lanes Y=+26.2, +28.8, +31.4)
    heater_runs = []
    for _, (tx, ty), _, lane_y, start_x in THERMO_ZONES:
        x_flat_start = start_x + 22.0
        x_flat_end = tx - 18.0
        pts = [
            (start_x, 27.4, 14.8),
            (start_x + 9.0, lane_y, 9.6),
            (x_flat_start, lane_y, 6.4),
        ]
        span = x_flat_end - x_flat_start
        n_seg = max(2, int(span // 40))
        for k in range(1, n_seg):
            pts.append((x_flat_start + span * (k / n_seg), lane_y, 6.4))
        pts.extend([
            (x_flat_end, lane_y, 6.4),
            (tx - 2.5, lane_y, 20.0),
            (tx, 25.5, 42.5),
            (tx, 14.0, 48.0),
            (tx, ty, 45.8),
        ])
        heater_runs.append(base_v4._wire_run(pts, radius=1.05))
    heater_cable = Compound(children=heater_runs)

    return plugs.part, sensor_cable, heater_cable


def export_pcr_sdl_manifest():
    manifest = {
        "standard": "BonFlow-Open-v4.0-PCR",
        "layout": "linear_1x9_rail",
        "mechanical_grid": {
            "pitch_mm": base_v4.PITCH,
            "baseplate_mm": [466.0, 68.0, 5.0],
            "hole_array": "18x2 Ø6.5mm",
            "standoff_type": "M3 Hex Standoff 10.0mm",
            "port_interface": "Vertical Press-Fit Ø3.0mm Reamed Lid + OD 3.2/ID 1.5mm Silicone Sleeve + Retainer + OD 1.6mm FEP",
        },
        "modules": [
            {"id": "elec_1u", "slot": list(POS_ELEC), "controller": "Seeed_XIAO_ESP32S3", "zones": 3},
            {"id": "mix_1u", "slot": list(POS_MIX), "wetted_materials": ["PMMA", "Silicone"]},
            {"id": "valve_in_1u", "slot": list(POS_VALVE_IN), "role": "Upstream_Backflow_Lock_95C"},
            {"id": "thermo_95_1u", "slot": list(POS_THERMO_95), "target_temp_C": 95, "role": "Denaturation"},
            {"id": "valve_out_1u", "slot": list(POS_VALVE_OUT), "role": "Downstream_Pressure_Lock_95C"},
            {"id": "thermo_58_1u", "slot": list(POS_THERMO_58), "target_temp_C": 58, "role": "Annealing"},
            {"id": "thermo_72_1u", "slot": list(POS_THERMO_72), "target_temp_C": 72, "role": "Extension"},
            {"id": "sense_1u", "slot": list(POS_SENSE), "sensor": "TCS34725_I2C"},
            {"id": "drip_1u", "slot": list(POS_DRIP), "bpr_adjustable": True, "collector": "1.5mL_Microtube"},
        ],
        "default_topology": [
            "mix_1u.E -> valve_in_1u.IN",
            "valve_in_1u.OUT -> thermo_95_1u.IN",
            "thermo_95_1u.OUT -> valve_out_1u.IN",
            "valve_out_1u.OUT -> thermo_58_1u.IN",
            "thermo_58_1u.OUT -> thermo_72_1u.IN",
            "thermo_72_1u.OUT -> sense_1u.IN",
            "sense_1u.OUT -> drip_1u.IN",
        ],
    }
    out = OUT_DIR.parent / "fluidic_graph_pcr.json"
    out.write_text(json.dumps(manifest, indent=2))
    print(f"Exported PCR SDL manifest: {out}")


def check_pcr_interferences(parts):
    def _solids(shp):
        return list(shp.solids()) if hasattr(shp, "solids") and shp.solids() else [shp]

    def _bbox_disjoint(b1, b2, tol=1e-4):
        return (
            b1.max.X < b2.min.X - tol or b2.max.X < b1.min.X - tol
            or b1.max.Y < b2.min.Y - tol or b2.max.Y < b1.min.Y - tol
            or b1.max.Z < b2.min.Z - tol or b2.max.Z < b1.min.Z - tol
        )

    def vol_inter(a, b):
        tot = 0.0
        for sa in _solids(parts[a]):
            ba = sa.bounding_box()
            for sb in _solids(parts[b]):
                bb = sb.bounding_box()
                if _bbox_disjoint(ba, bb):
                    continue
                # Fast exact minimum surface distance early-out when not enclosed
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

    pairs = [
        ("base_plate", "standoff_feet"),
        ("standoff_feet", "m3_bolts"),
        ("valve_in_base", "m3_bolts"),
        ("valve_in_lid", "m3_bolts"),
        ("valve_in_retainer", "m3_bolts"),
        ("valve_in_lid", "sleeves"),
        ("valve_out_base", "m3_bolts"),
        ("valve_out_lid", "m3_bolts"),
        ("valve_out_retainer", "m3_bolts"),
        ("valve_out_lid", "sleeves"),
        ("mix_base", "m3_bolts"),
        ("mix_lid", "m3_bolts"),
        ("mix_retainer", "m3_bolts"),
        ("mix_lid", "sleeves"),
        ("sense_housing", "sense_pcb"),
        ("sense_lid", "sleeves"),
        ("drip_frame", "bpr_screw"),
        ("drip_frame", "microtube"),
        ("elec_bus_base", "xiao_s3"),
        ("elec_bus_base", "m3_bolts"),
        ("carrier_pcb", "m3_bolts"),
        ("xiao_s3", "m3_bolts"),
        ("sleeves", "jumper_tubes"),
        ("base_plate", "sensor_wires"),
        ("base_plate", "heater_wires"),
        ("standoff_feet", "sensor_wires"),
        ("standoff_feet", "heater_wires"),
        ("elec_bus_base", "wire_plugs"),
        ("sense_housing", "wire_plugs"),
    ]
    for prefix, _, _, _, _ in THERMO_ZONES:
        pairs.extend([
            (f"{prefix}_cup", f"{prefix}_bobbin"),
            (f"{prefix}_bobbin", f"{prefix}_heater"),
            (f"{prefix}_lid", "sleeves"),
            (f"{prefix}_lid", "heater_wires"),
            (f"{prefix}_heater", "jumper_tubes"),
            ("wire_plugs", "jumper_tubes"),
        ])

    bad = 0
    for a, b in pairs:
        v = vol_inter(a, b)
        flag = " <-- COLLISION!" if v > 0.05 else ""
        if v > 0.05:
            bad += 1
        print(f"{a} vs {b}: {v:.4f} mm^3{flag}", flush=True)
    return bad


def main():
    for old_stl in OUT_DIR.glob("*.stl"):
        old_stl.unlink()

    e_box, e_pcb, e_xiao, e_i2ch, e_i2cp = base_v4.make_elec_module(POS_ELEC)
    m_base, m_gask, m_lid, m_ret = base_v4.make_mix_module(POS_MIX)
    vin_base, vin_gask, vin_lid, vin_ret, vin_sc, vin_hnd = base_v4.make_valve_module(POS_VALVE_IN)
    vout_base, vout_gask, vout_lid, vout_ret, vout_sc, vout_hnd = base_v4.make_valve_module(POS_VALVE_OUT)
    s_box, s_pcb, s_lid = base_v4.make_sense_module(POS_SENSE)
    d_frm, d_bpr, d_tube = base_v4.make_drip_module(POS_DRIP)
    w_plugs, w_sensor, w_heater = make_pcr_wiring_harness()

    bolt_tops = {
        POS_ELEC: base_v4.Z_STANDOFF_H + 8.5,
        POS_MIX: base_v4.Z_TOP_BOLT,
        POS_VALVE_IN: base_v4.Z_TOP_BOLT,
        POS_THERMO_95: base_v4.Z_STANDOFF_H + 4.0,
        POS_VALVE_OUT: base_v4.Z_TOP_BOLT,
        POS_THERMO_58: base_v4.Z_STANDOFF_H + 4.0,
        POS_THERMO_72: base_v4.Z_STANDOFF_H + 4.0,
        POS_SENSE: base_v4.Z_STANDOFF_H + 4.0,
        POS_DRIP: base_v4.Z_STANDOFF_H + 4.0,
    }

    parts = {
        "base_plate": base_v4.make_base_plate(length=466.0, cols=18, start_x=-212.5, anchor_x=225.0),
        "standoff_feet": base_v4.make_standoff_feet(ALL_SLOTS_PCR),
        "m3_bolts": base_v4.make_m3_bolts(bolt_tops),
        "sleeves": make_pcr_sleeves(),
        "jumper_tubes": make_pcr_jumper_tubes(),
        "wire_plugs": w_plugs,
        "sensor_wires": w_sensor,
        "heater_wires": w_heater,
        "elec_bus_base": e_box,
        "carrier_pcb": e_pcb,
        "xiao_s3": e_xiao,
        "i2c_housing": e_i2ch,
        "i2c_pins": e_i2cp,
        "mix_base": m_base,
        "mix_gasket": m_gask,
        "mix_lid": m_lid,
        "mix_retainer": m_ret,
        "valve_in_base": vin_base,
        "valve_in_gasket": vin_gask,
        "valve_in_lid": vin_lid,
        "valve_in_retainer": vin_ret,
        "valve_in_stopcock": vin_sc,
        "valve_in_handle": vin_hnd,
        "valve_out_base": vout_base,
        "valve_out_gasket": vout_gask,
        "valve_out_lid": vout_lid,
        "valve_out_retainer": vout_ret,
        "valve_out_stopcock": vout_sc,
        "valve_out_handle": vout_hnd,
        "sense_housing": s_box,
        "sense_pcb": s_pcb,
        "sense_lid": s_lid,
        "drip_frame": d_frm,
        "bpr_screw": d_bpr,
        "microtube": d_tube,
    }

    step_children = [e_box, m_base, vin_base, vout_base, d_frm]
    for prefix, pos, _, _, _ in THERMO_ZONES:
        tb, tc, tbob, thtr, tcoil, tlid = base_v4.make_thermo_module(pos)
        parts[f"{prefix}_base"] = tb
        parts[f"{prefix}_cup"] = tc
        parts[f"{prefix}_bobbin"] = tbob
        parts[f"{prefix}_heater"] = thtr
        parts[f"{prefix}_coil"] = tcoil
        parts[f"{prefix}_lid"] = tlid
        step_children.append(tb)

    for name, part in parts.items():
        stl_path = OUT_DIR / f"{name}.stl"
        export_stl(part, str(stl_path), tolerance=0.05, angular_tolerance=0.12)
        print(f"Exported {stl_path.name}", flush=True)

    step_path = OUT_DIR.parent / "bonflow_v4_pcr_assembly.step"
    export_step(Compound(children=step_children), str(step_path))
    print(f"Exported STEP: {step_path}", flush=True)

    export_pcr_sdl_manifest()
    bad = check_pcr_interferences(parts)
    print(f"Total parts exported: {len(parts)}, collision pairs (>0.05 mm^3): {bad}", flush=True)
    if bad:
        raise SystemExit(f"COLLISION FAILURE: {bad} pairs")


if __name__ == "__main__":
    main()
