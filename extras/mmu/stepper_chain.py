# -*- coding: utf-8 -*-
# Happy Hare MMU Software
#
# Stepper chain abstraction.
#
# Represents the filament path as an ordered list of N stages (driven steppers)
# with N-1 buffers between adjacent stages. Generalises the legacy two-stepper
# gear+extruder topology to arbitrary N >= 2 (booster, multi-booster, etc.).
#
# Design doc: doc/design/stepper-chain.md
#
# This module is pure Python with no Klipper dependencies so it can be unit
# tested standalone. It is the in-memory model of the chain; integration with
# Klipper kinematics, trapq cutover, sensor wiring, and sync feedback lives in
# other modules that consume this one (introduced in later phases).
#
# Copyright (C) 2026  Happy Hare project
#
# (\_/)
# ( *,*)
# (")_(") Happy Hare Ready
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#

from collections import namedtuple


# ---------- Stage roles & grip kinds ----------

# A stage's role identifies what kind of Klipper object it controls. The runtime
# consumer of the chain uses this to dispatch to the correct rebind / trapq
# logic; the chain itself only stores the tag.
ROLE_MMU_RAIL         = "mmu_rail"          # Stepper on the MMU kinematics
ROLE_KLIPPER_EXTRUDER = "klipper_extruder"  # The toolhead extruder

ROLES = (ROLE_MMU_RAIL, ROLE_KLIPPER_EXTRUDER)

# A stage's grip describes whether it can release the filament at runtime.
#   - GRIP_RELEASABLE: stage has a servo/idler that can be disengaged. While
#     disengaged the stage drops out of the sync chain (its neighbors couple
#     directly through the now-passive node).
#   - GRIP_ALWAYS:     stage always grips the filament; cannot disengage.
GRIP_RELEASABLE = "releasable"
GRIP_ALWAYS     = "always"

GRIPS = (GRIP_RELEASABLE, GRIP_ALWAYS)


# ---------- Chain-master direction ----------

# The chain-master end is the end currently issuing moves. Mastership then
# cascades inward: for each adjacent pair the stage closer to this end is the
# local master; the other is the local slave.
#
#   MASTER_TOOLHEAD: print / tip-form / purge (extruder is master)
#   MASTER_SPOOL:    MMU loader/unloader / homing / calibration (gear is master)
MASTER_TOOLHEAD = "toolhead"
MASTER_SPOOL    = "spool"

MASTER_ENDS = (MASTER_TOOLHEAD, MASTER_SPOOL)


# ---------- Stage / Buffer / PsfConfig ----------

class PsfConfig(object):
    """Sync feedback sensor configuration for one buffer.

    A PSF can be a tension switch, a compression switch, an analog (proportional)
    sensor, or any combination. At least one must be configured to be active.
    """

    def __init__(self, tension_sensor=None, compression_sensor=None,
                 analog_sensor=None, neutral_point=0.5):
        self.tension_sensor     = tension_sensor
        self.compression_sensor = compression_sensor
        self.analog_sensor      = analog_sensor
        self.neutral_point      = float(neutral_point)

    def is_configured(self):
        return bool(self.tension_sensor or self.compression_sensor or self.analog_sensor)

    def __repr__(self):
        bits = []
        if self.tension_sensor:     bits.append("tension=%r" % self.tension_sensor)
        if self.compression_sensor: bits.append("compression=%r" % self.compression_sensor)
        if self.analog_sensor:      bits.append("analog=%r" % self.analog_sensor)
        return "PsfConfig(%s)" % ", ".join(bits) if bits else "PsfConfig(unconfigured)"


class Stage(object):
    """One driven stepper in the filament path."""

    def __init__(self, name, stepper_ref, role, grip):
        if role not in ROLES:
            raise ValueError("Stage %r: invalid role %r (must be one of %r)" % (name, role, ROLES))
        if grip not in GRIPS:
            raise ValueError("Stage %r: invalid grip %r (must be one of %r)" % (name, grip, GRIPS))
        if not name:
            raise ValueError("Stage requires a non-empty name")
        if not stepper_ref:
            raise ValueError("Stage %r: stepper_ref is required" % name)
        self.name        = name
        self.stepper_ref = stepper_ref
        self.role        = role
        self.grip        = grip

    def __repr__(self):
        return "Stage(name=%r, stepper_ref=%r, role=%r, grip=%r)" % (
            self.name, self.stepper_ref, self.role, self.grip)


class Buffer(object):
    """Filament path segment between two adjacent stages.

    Holds the PSF (if any) governing this buffer's sync feedback, and optional
    presence sensors at the downstream stage's entry (``pre_sensor``) and exit
    (``post_sensor``).
    """

    def __init__(self, upstream_stage, downstream_stage,
                 psf=None, pre_sensor=None, post_sensor=None):
        if downstream_stage != upstream_stage + 1:
            raise ValueError("Buffer connects %d-%d; stages must be adjacent (downstream = upstream+1)"
                             % (upstream_stage, downstream_stage))
        self.upstream_stage   = upstream_stage
        self.downstream_stage = downstream_stage
        self.psf              = psf
        self.pre_sensor       = pre_sensor
        self.post_sensor      = post_sensor

    def __repr__(self):
        return "Buffer(%d-%d, psf=%r, pre=%r, post=%r)" % (
            self.upstream_stage, self.downstream_stage,
            self.psf, self.pre_sensor, self.post_sensor)


# A run of disengaged stages flanked by engaged stages on both sides.
# Both indices are inclusive.
DisengagedSpan = namedtuple("DisengagedSpan", ["start", "end", "left_engaged", "right_engaged"])


# ---------- Chain ----------

class Chain(object):
    """Ordered chain of stages with buffers between them.

    Invariants:
      - len(stages) >= 2
      - len(buffers) == len(stages) - 1
      - buffers[i] connects stages[i] and stages[i+1]
      - master_end is one of MASTER_ENDS
      - engagement[i] respects stages[i].grip (GRIP_ALWAYS stages are always engaged)

    Runtime state:
      - engagement: which stages currently grip the filament
      - master_end: which end of the chain is currently driving moves
    """

    def __init__(self, stages, buffers, master_end=MASTER_TOOLHEAD, engagement=None):
        n = len(stages)
        if n < 2:
            raise ValueError("Chain requires at least 2 stages (got %d)" % n)
        if len(buffers) != n - 1:
            raise ValueError("Chain has %d stages but %d buffers (expected %d)"
                             % (n, len(buffers), n - 1))
        for i, b in enumerate(buffers):
            if b.upstream_stage != i or b.downstream_stage != i + 1:
                raise ValueError(
                    "Buffer %d must connect stages %d-%d, got %d-%d"
                    % (i, i, i + 1, b.upstream_stage, b.downstream_stage))
        if master_end not in MASTER_ENDS:
            raise ValueError("Invalid master_end %r (must be one of %r)" % (master_end, MASTER_ENDS))

        self.stages  = list(stages)
        self.buffers = list(buffers)
        self._master_end = master_end

        # Default engagement: all stages engaged. GRIP_ALWAYS stages must be
        # engaged at all times; releasable stages default to engaged but can be
        # toggled off later.
        if engagement is None:
            self._engaged = [True] * n
        else:
            if len(engagement) != n:
                raise ValueError("engagement length %d does not match stage count %d"
                                 % (len(engagement), n))
            self._engaged = [bool(e) for e in engagement]
        for i, st in enumerate(self.stages):
            if st.grip == GRIP_ALWAYS and not self._engaged[i]:
                raise ValueError("Stage %r has grip=always; cannot be disengaged" % st.name)

    # ---- shape ----

    @property
    def n(self):
        return len(self.stages)

    def __len__(self):
        return len(self.stages)

    def stage_index(self, name):
        for i, st in enumerate(self.stages):
            if st.name == name:
                return i
        raise KeyError("No stage named %r" % name)

    # ---- master end ----

    @property
    def master_end(self):
        return self._master_end

    def set_master_end(self, master_end):
        if master_end not in MASTER_ENDS:
            raise ValueError("Invalid master_end %r" % master_end)
        self._master_end = master_end

    # ---- engagement ----

    def is_engaged(self, stage_idx):
        return self._engaged[stage_idx]

    def set_engaged(self, stage_idx, engaged):
        if not engaged and self.stages[stage_idx].grip == GRIP_ALWAYS:
            raise ValueError("Stage %r has grip=always; cannot be disengaged"
                             % self.stages[stage_idx].name)
        self._engaged[stage_idx] = bool(engaged)

    def engagement(self):
        # Return a copy so callers can't mutate internal state directly.
        return list(self._engaged)

    def engaged_stages(self):
        return [i for i, e in enumerate(self._engaged) if e]

    # ---- mastership cascade ----

    def chain_master_stage(self):
        """Index of the chain-master end's outermost engaged stage.

        With MASTER_TOOLHEAD this is the highest-index engaged stage; with
        MASTER_SPOOL the lowest-index engaged stage.
        """
        engaged = self.engaged_stages()
        if not engaged:
            raise RuntimeError("Chain has no engaged stages")
        return engaged[-1] if self._master_end == MASTER_TOOLHEAD else engaged[0]

    def local_master(self, buffer_idx):
        """Local master for the buffer's adjacent pair.

        The local master is the stage of the pair closer to the chain-master
        end. This rule is direction-symmetric: with MASTER_TOOLHEAD the
        downstream stage is master; with MASTER_SPOOL the upstream stage is
        master.
        """
        b = self.buffers[buffer_idx]
        return b.downstream_stage if self._master_end == MASTER_TOOLHEAD else b.upstream_stage

    def local_slave(self, buffer_idx):
        """Local slave for the buffer's adjacent pair (the non-master stage)."""
        b = self.buffers[buffer_idx]
        return b.upstream_stage if self._master_end == MASTER_TOOLHEAD else b.downstream_stage

    # ---- disengaged spans ----

    def disengaged_spans(self):
        """All maximal runs of disengaged stages flanked by engaged stages on
        both sides.

        A span represents a "transparent bridge": the steppers on either side
        couple directly across the disengaged interior, and the interior PSFs
        (if any) combine to feed back into the slave's correction.

        Disengaged stages at the chain ends are not spans (no flanking engaged
        stage on the outer side) and are returned only via ``engaged_stages``.
        """
        spans = []
        i = 0
        while i < self.n:
            if self._engaged[i]:
                i += 1
                continue
            start = i
            while i < self.n and not self._engaged[i]:
                i += 1
            end = i - 1
            if start > 0 and end < self.n - 1 \
                    and self._engaged[start - 1] and self._engaged[end + 1]:
                spans.append(DisengagedSpan(
                    start=start, end=end,
                    left_engaged=start - 1, right_engaged=end + 1))
        return spans

    def buffer_indices_between(self, left_stage, right_stage):
        """Buffer indices spanned by the two given stage indices (inclusive).

        With ``left_stage=A``, ``right_stage=B`` (A < B), returns the indices
        of buffers (A, A+1), (A+1, A+2), ..., (B-1, B). When stages A and B
        are adjacent (B == A+1) this is a single buffer index [A].

        Used by sync feedback: if the stages between A and B are disengaged,
        all PSFs in these buffers combine to govern the local slave of the
        (A, B) pair.
        """
        if left_stage >= right_stage:
            raise ValueError("left_stage (%d) must be < right_stage (%d)"
                             % (left_stage, right_stage))
        return list(range(left_stage, right_stage))


# ---------- Legacy N=2 chain factory ----------

def chain_from_legacy_n2(
        gear_stepper_ref="stepper_mmu_gear",
        extruder_stepper_ref="extruder",
        psf=None,
        extruder_pre_sensor=None,
        extruder_post_sensor=None):
    """Build the N=2 chain corresponding to today's legacy gear+extruder topology.

    This is the chain shape that existing Happy Hare configs implicitly declare:
    one gear stage (releasable; servo may release filament grip) followed by
    one extruder stage (always grips). Buffer 0 holds the optional PSF and the
    extruder's pre/post sensors.

    Phase 1 uses this factory both for unit tests and as the reference shape
    that ``chain_from_config`` will produce when no booster or other extension
    is configured. Phase 2 and later will wire production code through chain
    accessors; the resulting behavior at N=2 must remain bit-for-bit identical
    to today.
    """
    stages = [
        Stage(name="gear",     stepper_ref=gear_stepper_ref,
              role=ROLE_MMU_RAIL,         grip=GRIP_RELEASABLE),
        Stage(name="extruder", stepper_ref=extruder_stepper_ref,
              role=ROLE_KLIPPER_EXTRUDER, grip=GRIP_ALWAYS),
    ]
    buffers = [
        Buffer(upstream_stage=0, downstream_stage=1,
               psf=psf, pre_sensor=extruder_pre_sensor, post_sensor=extruder_post_sensor),
    ]
    return Chain(stages=stages, buffers=buffers)
