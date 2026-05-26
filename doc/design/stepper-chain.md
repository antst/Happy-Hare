# Stepper Chain Architecture

Status: **draft / proposal**
Branch: `feature/stepper-chain`

## 1. Motivation

Happy Hare today assumes exactly two driven steppers along the filament path: the **gear** (on the MMU) and the **extruder** (on the toolhead). The sync mechanism, PSF (Pressure/Sync Feedback) manager, load/unload state machine, bowden calibration, and a long list of smaller utilities all encode that 2-stepper assumption directly.

A growing class of setups wants a **third stepper** ("booster") in between — for long bowdens, remote MMU placements, IDEX with a separate post-selector drive, or simply for higher feed pressure. A natural follow-on is "what if there are several boosters along the way." Rather than special-casing booster=3, this proposal generalises the model to an **N-stepper filament chain**, of which today's 2-stepper layout is the N=2 instance.

The user-facing config stays simple (declare what's there, in order). The internal model becomes a uniform array of stages and buffers that all algorithms iterate over. **Backward compatibility is achieved by construction**: existing configs implicitly declare N=2, and every code path must collapse cleanly to today's behavior at N=2 — bit-for-bit.

This document is the agreed design before any code lands. The implementation plan is at the end (§9).

## 2. Concepts

### 2.1 Chain

A **chain** is an ordered list of `N ≥ 2` **stages** along the filament path, indexed from the spool side (0) to the toolhead side (N-1):

```
stage 0 (gear)  →  buffer 0  →  stage 1 (booster)  →  buffer 1  →  stage 2 (extruder)
```

For N adjacent stages there are `N-1` **buffers**.

### 2.2 Stage

Each stage represents one driven stepper. It has:

| Field | Meaning |
| --- | --- |
| `name` | Logical identifier (`gear`, `booster`, `extruder`, …) |
| `stepper_ref` | Klipper stepper or extruder object name |
| `role` | `mmu_rail` (lives on the MMU kinematics) or `klipper_extruder` (the real Klipper extruder) |
| `grip` | `releasable` (can disengage from the filament at runtime) or `always` (permanently grips) |
| `engage_cb` / `release_cb` | Optional callbacks for grip control (servo, idler, etc.) |

Today: `[Stage(gear, releasable), Stage(extruder, always)]`.
With booster: `[Stage(gear, releasable), Stage(booster, configurable), Stage(extruder, always)]`.
The booster's grip is configurable per-installation — some boosters always grip, others have a servo.

### 2.3 Buffer

Each buffer sits between two adjacent stages. It has:

| Field | Meaning |
| --- | --- |
| `upstream_stage` / `downstream_stage` | Indices into the chain |
| `psf` | Optional sync-feedback sensor (tension switch, compression switch, analog, or any combination) |
| `pre_sensor` | Optional presence sensor at the downstream stage's *entry* (e.g. `pre_booster`, `extruder` switch) |
| `post_sensor` | Optional presence sensor at the downstream stage's *exit* (e.g. `post_booster`, `toolhead` switch) |
| `length` | Calibrated filament distance for this segment (per gate) |

Sensors are **per buffer**, not per stage. A sensor "before stepper X" is logically the `pre_sensor` of buffer (X-1, X); a sensor "after stepper X" is the `post_sensor` of the same buffer. Adjacent buffers can share a physical sensor in the schema if it logically marks both "exit of one buffer" and "entry of the next."

### 2.4 Location-agnostic

The chain abstraction makes no assumption about which MCU a stepper lives on. A stage's `stepper_ref` resolves to whatever pin namespace the user configured — `mmu:`, `mcu:`, `toolhead:`, or a dedicated `booster_mcu:`. This naturally supports: booster on the MMU board, booster on the printer mainboard, booster on the toolhead, or a booster with its own MCU sitting halfway down a 15 m bowden run.

## 3. Sync Model

### 3.1 Chain-master and mastership cascade

At any given moment there is one **chain-master end**: either `toolhead` (the toolhead extruder is driving) or `spool` (the MMU gear is driving). The chain-master end flips with the operation:

- Printing, tip-forming, purging → chain-master = `toolhead`
- MMU loader/unloader moves, calibration, homing → chain-master = `spool`

From the chain-master end, **mastership cascades inward**: for any adjacent pair of stages, the one *closer to the chain-master end* is the **local master**, and the other is the **local slave**. With N=3 and chain-master = `toolhead`:

- Pair (booster, extruder): local master = extruder, local slave = booster
- Pair (gear, booster): local master = booster, local slave = gear
- Booster is the "middle manager" — slave of one pair, master of the other.

With chain-master = `spool` the roles flip end-to-end; the booster is still the middle manager but now slave to the gear and master to the extruder.

### 3.2 PSF correction rule

A PSF in buffer `i` (between stages `i` and `i+1`) corrects the **rotation distance of the local slave of that pair**. This single rule generalises:

- Print, N=2: PSF0 between gear and extruder → corrects gear RD (extruder is local master). Matches today's behavior.
- Print, N=3: PSF0 corrects gear RD; PSF1 corrects booster RD.
- MMU op, N=3: PSF0 corrects booster RD; PSF1 corrects extruder RD.

Same PSF hardware, no extra state — the governed stepper is derived from the chain-master direction. No special case is needed for the booster.

### 3.3 Sync-group transitions

A **sync group** is the set of currently engaged stages plus the chain-master end. The runtime represents this as a small descriptor (array of `engaged?` booleans plus a master end). All sync-mode changes — entering sync for a print, dropping out for a wipe-tower handoff, releasing gear for a special operation — become **transitions between two sync-group descriptors**.

The runtime hot path is a single function:

```
resync(old_group, new_group):
    detach steppers that are in old but not new
    attach steppers that are in new but not old
    set master end as configured
```

At N=2 this collapses to the current `_resync_no_lock` logic. At N≥3 it handles arbitrary engagement patterns uniformly.

### 3.4 Disengaged stages and span combining

A `releasable` stage may disengage from the filament at runtime. While disengaged, it **drops out of the sync chain**: the steppers on either side couple directly through the now-passive node.

A **disengaged span** is a maximal run of engaged-disengaged-engaged stages (with one or more disengaged stages in the middle). The filament passes through the disengaged stages with no active drive, so the buffers on either side of the disengaged span effectively merge into **one logical buffer** spanning the gap.

For sync feedback, the PSFs *inside* the span are combined and their result drives the slave stepper of the now-direct pair:

- **Switch PSFs**: OR-combine. Tension on any interior switch → bridge is tense; compression on any → bridge is compressed. Conflicting signals (tension on one PSF + compression on another inside the same span) are an anomaly and should be logged.
- **Analog PSFs**: average (or weighted average by segment length / trust). Sum-of-signed-readings is acceptable as long as scaling is consistent.
- **Mixed**: treat switch as bounded analog (0 / 0.5 / 1) and average.

When all stages are engaged, every span has length 1, every buffer has its own PSF, and the combining logic is a no-op — recovering the per-buffer behavior of §3.2.

### 3.5 Active-master semantics for moves

The current `motor=` string protocol in `trace_filament_move` (e.g. `"gear" | "gear+extruder" | "synced" | "extruder"`) is replaced by a chain-aware descriptor: which stage is the active driver and which sync group is in effect. The legacy strings remain accepted at N=2 for backward compatibility.

## 4. Sensor Model

Existing sensors (`pre_gate_X`, `mmu_gate`, `mmu_gear_X`, `extruder`, `toolhead`) keep their positions and meanings. New sensors `pre_booster` and `post_booster` slot in around the booster stage as the pre/post sensors of buffers 0 and 1 respectively.

The `MmuSensorManager`'s ordered list, endstop bindings, and `_get_sensors` position table become **derived from the chain definition** rather than hardcoded. Each buffer contributes its pre/post sensors (if defined) to the ordered list in chain order.

The ASCII path summary in `mmu.py` (currently around lines 2024-2042) is regenerated from the chain.

## 5. Calibration Model

### 5.1 Per-segment bowden lengths

Today's `bowden_lengths[gate]` (one length per gate) becomes `bowden_lengths[gate][buffer_idx]` — one length per buffer per gate. With N=2, `buffer_idx` is always 0 and the data model is shape-compatible with legacy saves.

Calibration runs **per buffer, independently**, reusing the existing algorithms (manual / sensor / collision) once per buffer:

- Buffer 0 (gear → booster entry): gear drives alone (booster grip released or pass-through) until `pre_booster` trips. That distance is buffer 0's length.
- Buffer 1 (booster → extruder): with gear released, booster drives until the `extruder` switch trips. That distance is buffer 1's length.

For N=2, only one buffer exists and the procedure is unchanged.

### 5.2 PSF calibration

Each PSF is calibrated independently (neutral point, sensitivity). This matches §3.2: each PSF governs one slave; tuning them independently is the natural decomposition.

### 5.3 What does *not* change

- Per-stepper rotation distance: still per stepper, no new concept.
- Encoder ratio: encoder remains on the gear, measuring buffer 0 only.
- `toolhead_entry_to_extruder` scalar: unchanged.

## 6. Load / Unload Sequence

The load and unload functions become **data-driven from the chain definition**. The general shape of a load:

```
for buffer in chain.buffers:
    drive the upstream stage of `buffer` (with downstream stages released or
        passively passing filament) until the next sensor trips, or move the
        calibrated buffer length if no sensor is present
    hand off: release upstream grip if releasable, engage downstream grip if
        not already engaged, optionally enter sync group with downstream as
        the new active driver
final extruder load: existing _load_extruder logic, but invoked with the
    chain stage = klipper_extruder rather than a hardcoded reference
```

Unload mirrors this in reverse.

The 12-value `FILAMENT_POS_*` enum is extended with per-buffer crossing states (`FILAMENT_POS_HOMED_PRE_BUFFER_i`, `FILAMENT_POS_IN_BUFFER_i`). Today's `START_BOWDEN` / `END_BOWDEN` / `HOMED_ENTRY` map onto the N=2 chain's buffer 0 states; persisted recovery vars are migrated trivially.

`recover_filament_pos` derives valid recovery transitions from the chain's sensor topology rather than from a hardcoded table.

## 7. Backward Compatibility

This is the load-bearing constraint of the entire refactor.

**Implicit chain construction.** An existing config with no booster section auto-derives the chain `[Stage(gear, releasable), Stage(extruder, always)]` with one buffer. No config migration is required; nothing in the user's `mmu_hardware.cfg` needs to change.

**N=2 reproduces today's behavior bit-for-bit.** Every new code path must collapse cleanly to the legacy behavior at N=2. This is the discipline that prevents drift:

- The "for stage in chain" loops execute exactly the legacy 2-stage code path when N=2.
- The sync-group transition logic produces exactly the legacy enum transitions when only two stages can be engaged.
- The PSF manager runs exactly one controller when there is exactly one PSF.
- The load/unload sequence reduces to today's `_load_bowden / _home_to_extruder / _load_extruder` invocation sequence.
- The legacy `motor=` strings are still accepted in `trace_filament_move`.

**No new branches for legacy.** The implementation must not be peppered with `if N == 2: ... else: ...` — the generic code *is* the legacy code at N=2. Every branch on chain length is a smell.

**Persisted vars.** Save formats are extended in a forward-compatible way (one extra dimension where needed); legacy saves load into the N=2 slot.

**Test invariants.** The existing test suite (`test/`) must pass with no edits in Phase 1 (the no-op refactor). Any test edit signals a behavior change that needs to be examined.

## 8. Data Model (sketch)

```python
# extras/mmu/stepper_chain.py

from dataclasses import dataclass, field
from typing import Callable, List, Literal, Optional

Grip = Literal["releasable", "always"]
Role = Literal["mmu_rail", "klipper_extruder"]
MasterEnd = Literal["spool", "toolhead"]

@dataclass
class Stage:
    name: str
    stepper_ref: str
    role: Role
    grip: Grip
    engage_cb: Optional[Callable[[], None]] = None
    release_cb: Optional[Callable[[], None]] = None

@dataclass
class PsfConfig:
    tension_sensor: Optional[str] = None
    compression_sensor: Optional[str] = None
    analog_sensor: Optional[str] = None
    neutral_point: float = 0.5
    # ... tuning params

@dataclass
class Buffer:
    upstream_stage: int
    downstream_stage: int
    psf: Optional[PsfConfig] = None
    pre_sensor: Optional[str] = None
    post_sensor: Optional[str] = None
    length_per_gate: List[float] = field(default_factory=list)

@dataclass
class Chain:
    stages: List[Stage]
    buffers: List[Buffer]   # len == len(stages) - 1
    # runtime state
    engaged: List[bool] = field(default_factory=list)
    master_end: MasterEnd = "toolhead"

    def local_master(self, buffer_idx: int) -> int: ...
    def local_slave(self, buffer_idx: int) -> int: ...
    def engaged_stages(self) -> List[int]: ...
    def disengaged_spans(self) -> List[range]: ...
    def effective_buffer(self, stage_a: int, stage_b: int) -> 'EffectiveBuffer': ...

@dataclass
class SyncGroup:
    engaged: List[bool]
    master_end: MasterEnd
```

The runtime owns one `Chain` instance, derived from config at startup. Transitions mutate the `engaged` list and the `master_end` and call into `resync(...)`.

## 9. Phased Implementation Plan

Each phase lands as a separate reviewable PR. Phases 1-3 are the foundational refactor; 4-7 build on it; 8 sweeps up the long tail.

### Phase 1 — No-op chain refactor (next)

**Goal.** Introduce the chain abstraction and route the existing two-stepper code paths through it, with **zero observable behavior change**. This is the safest possible foundation: the refactor lands first as an "identity" PR that future phases extend.

**Changes:**
- New file `extras/mmu/stepper_chain.py` containing `Stage`, `Buffer`, `Chain`, `SyncGroup`, and `chain_from_config(config) -> Chain` that derives the N=2 chain from existing `mmu_hardware.cfg`.
- `extras/mmu_machine.py`: replace direct `rails[0]/rails[1]` indexing and `GEAR_STEPPER_CONFIG` usage with chain accessors. `MmuKinematics` and `_resync_no_lock` are made to walk the chain (length 2 today). No new rails; no schema change.
- `extras/mmu/mmu.py`: introduce a chain accessor (`self.chain`) and route the existing call sites that ask "is gear synced to extruder" through chain queries. `wrap_sync_gear_to_extruder` and friends stay as wrappers that delegate to the chain transition logic. The 4-state enum is preserved but becomes a derived view of the chain's `SyncGroup`.
- `extras/mmu/mmu_sync_feedback_manager.py`: the single PSF controller is wrapped in a "buffer 0" accessor — same behavior, prepared for §3.4 generalisation in Phase 3.
- No changes to config schema, persisted vars, sensor manager, calibration, or load/unload state machine.

**Acceptance:**
- All existing tests pass without edits.
- `git diff` of behavior-relevant code is reviewable in one sitting.
- Hardware smoke test on a real N=2 setup (single MMU, no booster) shows no regression.

### Phase 2 — Generic sync state machine

Replace the 4-state enum with the `SyncGroup` descriptor. `_resync_no_lock` becomes a transition function `transition(old: SyncGroup, new: SyncGroup)`. The `motor=` string protocol is extended (legacy strings still accepted at N=2) so that `trace_filament_move` can express "active driver = stage i, sync group = G".

### Phase 3 — Generic PSF manager

Replace the single PSF controller with a list, one per buffer. Route `mmu:sync_feedback` events per buffer id. Implement disengaged-span combining (§3.4). At N=2 with one PSF, behavior is unchanged.

### Phase 4 — Sensor additions

Add `pre_booster_switch_pin` and `post_booster_switch_pin` to `[mmu_sensors]`. Add `SENSOR_PRE_BOOSTER` / `SENSOR_POST_BOOSTER` constants. Wire into `MmuSensorManager` and the chain's buffer sensors. (Schema is additive; existing configs are unaffected.)

### Phase 5 — Data-driven load/unload sequence

Refactor `_load_gate / _load_bowden / _home_to_extruder / _load_extruder` and their unload mirrors to iterate over the chain's buffers as described in §6. Extend `FILAMENT_POS_*` and update `recover_filament_pos`. At N=2 the iteration produces the same sequence of moves and state transitions as today.

### Phase 6 — Per-segment calibration

Replace `bowden_lengths[gate]` with `bowden_lengths[gate][buffer_idx]`. Update `MmuCalibrationManager` to calibrate each buffer independently; autotune likewise. New save vars; legacy saves migrate trivially.

### Phase 7 — Booster stepper integration

The first phase that *adds new hardware support* rather than refactoring. Add `[stepper_mmu_booster]` config schema (optional, location-agnostic). Add `BOOSTER_STEPPER_CONFIG` constant. Extend `chain_from_config(...)` to include the booster stage when configured. Provide grip control (servo, idler, or "always engaged" no-op).

Hardware testing on a real 3-stepper setup.

### Phase 8 — Cross-cutting cleanup

Generalise remaining two-stepper assumptions: `motors_onoff` enum entries, `_adjust_gear_current` / `wrap_gear_current` parallels for the booster, `dump_rails` and kinematics axis labels, `MmuExtruderStepper` to accept a chain rather than a single rail, etc. Each is small; there are many.

## 10. Open Questions / Future Extensions

- **Tree topologies (v4 roadmap).** The chain is linear. Multi-MMU setups feeding one extruder, or IDEX with separate feed paths, want a *tree*. The chain's `Stage` is intentionally a leaf-like abstraction so that a future `CompositeNode` (subtree of stages with its own internal chain) can replace it without rewriting the rest. Out of scope for this proposal.
- **Per-lane multi-stepper.** Limited to one stepper per gate (the existing gear). Chain extension applies only to the post-selector common path. Generalising to per-lane chains is plausible but currently no use case justifies the complexity.
- **Encoder reinterpretation.** The encoder lives on the gear and measures only buffer 0. When the gear is released (during print, in some operations), encoder readings need to be ignored rather than interpreted as clog. Handled in Phase 5 / Phase 8.
- **Disengaged span PSF combining strategy.** Default proposed in §3.4 is OR-for-switches, average-for-analog. Tunable per buffer via config in case real installations prefer different behavior.
- **Calibration UX for N≥3.** Each buffer is calibrated independently. UX may benefit from a "calibrate all" wizard that runs them in order with appropriate engage/release transitions between steps.
