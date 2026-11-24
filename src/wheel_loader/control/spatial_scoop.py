"""Geometry-gated scoop goals in world metres/radians; cylinder actuation is separate."""

import numpy as np


class SpatialScoop:
    def __init__(self, cfg):
        self.cfg = cfg
        self.phase = "settle"
        self.entered = 0.0
        self.done = False
        self.failed = False
        self.reason = None
        self.events = []
        self.reverse_x = None
        self.curl_entry_mass = None
        self.stall_since = None

    def transition(self, phase, t, observation):
        self.events.append(
            {
                "from": self.phase,
                "to": phase,
                "time_s": t,
                "penetration_m": observation["penetration"],
                "bucket_mass_kg": observation["mass"],
                "clearance_m": observation["clearance"],
            }
        )
        self.phase = phase
        self.entered = t

    def goal(self, t, o):
        c = self.cfg
        age = t - self.entered
        h = c["attack_height_m"]
        pitch = c["attack_pitch_rad"]
        speed = 0.0
        safe = (
            o["clearance"] >= c["minimum_clearance_m"]
            and o["clearance"] + min(o["clearance_rate"], 0) * c["braking_horizon_s"]
            >= c["minimum_clearance_m"]
        )
        aligned = (
            abs(o["lip_z"] - h) < c["height_tolerance_m"]
            and abs(o["pitch"] - pitch) < c["pitch_tolerance_rad"]
        )
        if self.phase == "settle":
            h = c["transport_height_m"]
            pitch = c["attack_pitch_rad"]
            if t >= c["settle_s"] and safe:
                self.transition("lower" if c["unloaded"] else "approach", t, o)
        elif self.phase == "lower":
            if aligned and safe and age > 1:
                self.transition("curl", t, o)
        elif self.phase == "approach":
            if aligned and safe:
                speed = c["approach_speed_m_s"]
                if o["distance_to_entry"] <= c["standoff_m"]:
                    self.transition("penetrate", t, o)
        elif self.phase == "penetrate":
            speed = c["penetration_speed_m_s"] if aligned and safe else 0.0
            stalled = (
                o["active_material_contact"]
                and o["penetration"] >= c["minimum_stall_entry_m"]
                and abs(o["forward_speed_m_s"]) < c["stall_speed_m_s"]
            )
            self.stall_since = (
                (t if self.stall_since is None else self.stall_since)
                if stalled
                else None
            )
            stall_trigger = (
                self.stall_since is not None
                and t - self.stall_since >= c["stall_duration_s"]
            )
            if (
                o["active_material_contact"]
                and (o["penetration"] >= c["curl_entry_depth_m"] or stall_trigger)
                and safe
            ):
                self.curl_entry_mass = o["mass"]
                self.transition("curl", t, o)
            elif o["penetration"] > c["penetration_depth_m"]:
                self.failed = True
                self.reason = "penetration_depth_without_material_contact"
        elif self.phase == "curl":
            pitch = c["curl_pitch_rad"]
            h = c["curl_height_m"]
            speed = (
                c["curl_advance_speed_m_s"]
                if not c["unloaded"]
                and o["penetration"] < c["penetration_depth_m"]
                and safe
                else 0.0
            )
            if (
                abs(o["pitch"] - pitch) < c["pitch_tolerance_rad"]
                and safe
                and (c["unloaded"] or o["mass"] >= c["minimum_payload_kg"])
            ):
                self.transition("lift", t, o)
        elif self.phase == "lift":
            h = max(c["carry_height_m"], o["pile_top"] + c["breakout_clearance_m"])
            pitch = c["carry_pitch_rad"]
            if abs(o["lip_z"] - h) < c["height_tolerance_m"] and safe:
                self.reverse_x = o["rear_x"]
                self.transition("retain" if c["unloaded"] else "reverse", t, o)
        elif self.phase in {"reverse", "retain", "done"}:
            h = max(c["carry_height_m"], o["pile_top"] + c["breakout_clearance_m"])
            pitch = c["carry_pitch_rad"]
            if self.phase == "reverse":
                speed = -c["reverse_speed_m_s"] if safe else 0.0
                if self.reverse_x - o["rear_x"] >= c["reverse_distance_m"]:
                    self.transition("retain", t, o)
            elif self.phase == "retain" and age >= c["retain_s"]:
                if c["unloaded"] or o["mass"] >= c["minimum_payload_kg"]:
                    self.done = True
                    self.transition("done", t, o)
                else:
                    self.failed = True
                    self.reason = "no_retained_payload"
        if age > c["phase_timeout_s"] and self.phase != "done":
            self.failed = True
            self.reason = "phase_timeout_" + self.phase
        if o["clearance"] < -c["ground_overlap_tolerance_m"]:
            self.failed = True
            self.reason = "bucket_ground_clearance_violation"
        if not safe or self.failed or self.done:
            speed = 0.0
        if (
            self.phase in {"penetrate", "curl"}
            and o["reaction_force_N"] >= c["breakout_threshold_N"]
        ):
            speed = 0.0
        return np.array([h, pitch, speed])
