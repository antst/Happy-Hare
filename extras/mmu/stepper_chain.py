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


# ---------- SyncGroup: point-in-time sync state descriptor ----------

# Legacy 4-state sync enum values from MmuToolHead (in extras/mmu_machine.py).
# Kept here as constants so the bijection helpers below can be unit-tested
# without importing the Klipper-dependent mmu_machine module.
LEGACY_NONE                    = None
LEGACY_EXTRUDER_SYNCED_TO_GEAR = 1   # "gear+extruder": gear is master, extruder follows on gear rail
LEGACY_EXTRUDER_ONLY_ON_GEAR   = 2   # "extruder":       extruder follows on gear rail, gear disabled
LEGACY_GEAR_SYNCED_TO_EXTRUDER = 3   # "extruder+gear":  extruder is master, gear follows on extruder
LEGACY_GEAR_ONLY               = 4   # "gear":           independent; same logical state as None but with a fence


class SyncGroup(object):
    """Point-in-time description of the chain's mechanical sync state.

    Captures three pieces:
      - ``engagement``: per-stage boolean, which stages currently grip the
        filament (and therefore participate in the sync chain).
      - ``master_end``: which end of the chain is the chain-master at this
        moment (cascade then determines local masters per buffer).
      - ``sync_active``: whether the engaged stages are mechanically coupled
        (sync mode on) or operating independently (sync mode off).

    Transitions between two ``SyncGroup`` instances drive the Klipper trapq
    cutover work that ``_resync_no_lock`` does today. A future phase will
    refactor that function to consume ``SyncGroup`` directly; for now this
    class is a documented descriptor with a bijection to/from the legacy
    4-state enum so call sites can be migrated incrementally.

    At N=2, the bijection to legacy modes is:
        LEGACY_NONE / LEGACY_GEAR_ONLY:
            SyncGroup([T,T], any master, sync_active=False)
            (None and GEAR_ONLY are equivalent in chain semantics; the legacy
             distinction is a Klipper plumbing fence, not a chain state.)
        LEGACY_GEAR_SYNCED_TO_EXTRUDER:
            SyncGroup([T,T], MASTER_TOOLHEAD, sync_active=True)
        LEGACY_EXTRUDER_SYNCED_TO_GEAR:
            SyncGroup([T,T], MASTER_SPOOL, sync_active=True)
        LEGACY_EXTRUDER_ONLY_ON_GEAR:
            SyncGroup([F,T], MASTER_SPOOL, sync_active=True)
    """

    def __init__(self, engagement, master_end=MASTER_TOOLHEAD, sync_active=True):
        if master_end not in MASTER_ENDS:
            raise ValueError("Invalid master_end %r" % master_end)
        if len(engagement) < 2:
            raise ValueError("SyncGroup requires at least 2 stages of engagement")
        self.engagement  = [bool(e) for e in engagement]
        self.master_end  = master_end
        self.sync_active = bool(sync_active)

    @property
    def n(self):
        return len(self.engagement)

    def engaged_stages(self):
        return [i for i, e in enumerate(self.engagement) if e]

    def __eq__(self, other):
        if not isinstance(other, SyncGroup):
            return NotImplemented
        return (self.engagement == other.engagement
                and self.master_end == other.master_end
                and self.sync_active == other.sync_active)

    def __ne__(self, other):
        result = self.__eq__(other)
        if result is NotImplemented:
            return result
        return not result

    def __hash__(self):
        return hash((tuple(self.engagement), self.master_end, self.sync_active))

    def __repr__(self):
        return "SyncGroup(engagement=%r, master_end=%r, sync_active=%r)" % (
            self.engagement, self.master_end, self.sync_active)


def sync_group_from_legacy(legacy_mode):
    """Map a legacy 4-state sync enum value to a SyncGroup (N=2 only).

    LEGACY_NONE and LEGACY_GEAR_ONLY both map to the same SyncGroup with
    ``sync_active=False`` because they are equivalent in chain-mechanical
    semantics (the difference is a Klipper plumbing fence). The
    ``master_end`` is set to MASTER_TOOLHEAD for the unsynced case purely as
    a default; it has no operational effect when ``sync_active=False``.
    """
    if legacy_mode in (LEGACY_NONE, LEGACY_GEAR_ONLY):
        return SyncGroup(engagement=[True, True],
                         master_end=MASTER_TOOLHEAD, sync_active=False)
    if legacy_mode == LEGACY_GEAR_SYNCED_TO_EXTRUDER:
        return SyncGroup(engagement=[True, True],
                         master_end=MASTER_TOOLHEAD, sync_active=True)
    if legacy_mode == LEGACY_EXTRUDER_SYNCED_TO_GEAR:
        return SyncGroup(engagement=[True, True],
                         master_end=MASTER_SPOOL, sync_active=True)
    if legacy_mode == LEGACY_EXTRUDER_ONLY_ON_GEAR:
        return SyncGroup(engagement=[False, True],
                         master_end=MASTER_SPOOL, sync_active=True)
    raise ValueError("Unknown legacy sync mode: %r" % legacy_mode)


def sync_group_to_legacy(group, prefer_gear_only_for_unsynced=False):
    """Map a SyncGroup back to a legacy 4-state value (N=2 only).

    The mapping is bijective except for the unsynced case: ``sync_active=False``
    maps to either LEGACY_NONE (default) or LEGACY_GEAR_ONLY (when
    ``prefer_gear_only_for_unsynced=True``). Use LEGACY_GEAR_ONLY when the
    caller wants the protective-wait fence behavior of the legacy state.

    Raises ValueError if the SyncGroup describes a state that has no legacy
    equivalent (e.g., N != 2, or a 3-stage configuration).
    """
    if group.n != 2:
        raise ValueError("Legacy enum only describes N=2 sync states (got N=%d)" % group.n)
    if not group.sync_active:
        # Engagement must be all True for the unsynced state to match legacy
        # (legacy code never represents "unsynced with one stepper disengaged"
        # as a sync_mode; that's handled via _reconfigure_rail_no_lock).
        if group.engagement != [True, True]:
            raise ValueError("Unsynced SyncGroup with disengaged stages has no legacy enum equivalent")
        return LEGACY_GEAR_ONLY if prefer_gear_only_for_unsynced else LEGACY_NONE
    # sync_active = True
    if group.engagement == [True, True]:
        if group.master_end == MASTER_TOOLHEAD:
            return LEGACY_GEAR_SYNCED_TO_EXTRUDER
        else:  # MASTER_SPOOL
            return LEGACY_EXTRUDER_SYNCED_TO_GEAR
    if group.engagement == [False, True] and group.master_end == MASTER_SPOOL:
        return LEGACY_EXTRUDER_ONLY_ON_GEAR
    raise ValueError(
        "SyncGroup %r has no legacy enum equivalent (likely N>=3 or invalid N=2 mix)" % (group,))


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
