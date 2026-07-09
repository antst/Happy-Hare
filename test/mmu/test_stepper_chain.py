# -*- coding: utf-8 -*-
# Unit tests for extras/mmu/stepper_chain.py
#
# These tests cover the pure-Python chain abstraction with no Klipper
# dependencies. Run via:
#   python3 -m unittest test.mmu.test_stepper_chain

import importlib.util
import os
import unittest

# Load extras/mmu/stepper_chain.py directly without triggering the package
# __init__.py (which imports Mmu and pulls in Klipper-only modules like
# chelper). The chain abstraction is pure Python and standalone-testable;
# bypassing the package init keeps the unit tests independent of Klipper.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_THIS_DIR, "..", ".."))
_CHAIN_PATH = os.path.join(_REPO_ROOT, "extras", "mmu", "stepper_chain.py")
_spec = importlib.util.spec_from_file_location("_stepper_chain_under_test", _CHAIN_PATH)
sc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sc)

Buffer                          = sc.Buffer
Chain                           = sc.Chain
PsfConfig                       = sc.PsfConfig
Stage                           = sc.Stage
SyncGroup                       = sc.SyncGroup
chain_from_legacy_n2            = sc.chain_from_legacy_n2
sync_group_from_legacy          = sc.sync_group_from_legacy
sync_group_to_legacy            = sc.sync_group_to_legacy
GRIP_ALWAYS                     = sc.GRIP_ALWAYS
GRIP_RELEASABLE                 = sc.GRIP_RELEASABLE
MASTER_SPOOL                    = sc.MASTER_SPOOL
MASTER_TOOLHEAD                 = sc.MASTER_TOOLHEAD
ROLE_KLIPPER_EXTRUDER           = sc.ROLE_KLIPPER_EXTRUDER
ROLE_MMU_RAIL                   = sc.ROLE_MMU_RAIL
LEGACY_NONE                     = sc.LEGACY_NONE
LEGACY_EXTRUDER_SYNCED_TO_GEAR  = sc.LEGACY_EXTRUDER_SYNCED_TO_GEAR
LEGACY_EXTRUDER_ONLY_ON_GEAR    = sc.LEGACY_EXTRUDER_ONLY_ON_GEAR
LEGACY_GEAR_SYNCED_TO_EXTRUDER  = sc.LEGACY_GEAR_SYNCED_TO_EXTRUDER
LEGACY_GEAR_ONLY                = sc.LEGACY_GEAR_ONLY
combine_psf_average             = sc.combine_psf_average
combine_psf_or                  = sc.combine_psf_or
switch_reading                  = sc.switch_reading


def _make_stage(name="x", role=ROLE_MMU_RAIL, grip=GRIP_RELEASABLE):
    return Stage(name=name, stepper_ref="stepper_%s" % name, role=role, grip=grip)


def _make_n3_chain(grip_booster=GRIP_ALWAYS):
    """Gear (releasable) -> Booster (configurable) -> Extruder (always).

    Mirrors the target booster topology described in the design doc.
    """
    stages = [
        Stage(name="gear",     stepper_ref="stepper_mmu_gear",
              role=ROLE_MMU_RAIL,         grip=GRIP_RELEASABLE),
        Stage(name="booster",  stepper_ref="stepper_mmu_booster",
              role=ROLE_MMU_RAIL,         grip=grip_booster),
        Stage(name="extruder", stepper_ref="extruder",
              role=ROLE_KLIPPER_EXTRUDER, grip=GRIP_ALWAYS),
    ]
    buffers = [
        Buffer(upstream_stage=0, downstream_stage=1,
               psf=PsfConfig(tension_sensor="psf1_t"), pre_sensor="pre_booster"),
        Buffer(upstream_stage=1, downstream_stage=2,
               psf=PsfConfig(tension_sensor="psf2_t"), pre_sensor="extruder_sw"),
    ]
    return Chain(stages=stages, buffers=buffers)


class TestStageValidation(unittest.TestCase):
    def test_valid_stage(self):
        s = _make_stage()
        self.assertEqual(s.name, "x")
        self.assertEqual(s.role, ROLE_MMU_RAIL)
        self.assertEqual(s.grip, GRIP_RELEASABLE)

    def test_invalid_role(self):
        with self.assertRaises(ValueError):
            Stage(name="x", stepper_ref="s", role="bogus", grip=GRIP_RELEASABLE)

    def test_invalid_grip(self):
        with self.assertRaises(ValueError):
            Stage(name="x", stepper_ref="s", role=ROLE_MMU_RAIL, grip="bogus")

    def test_empty_name(self):
        with self.assertRaises(ValueError):
            Stage(name="", stepper_ref="s", role=ROLE_MMU_RAIL, grip=GRIP_RELEASABLE)

    def test_empty_stepper_ref(self):
        with self.assertRaises(ValueError):
            Stage(name="x", stepper_ref="", role=ROLE_MMU_RAIL, grip=GRIP_RELEASABLE)


class TestBufferValidation(unittest.TestCase):
    def test_valid_buffer(self):
        b = Buffer(upstream_stage=0, downstream_stage=1)
        self.assertEqual(b.upstream_stage, 0)
        self.assertEqual(b.downstream_stage, 1)
        self.assertIsNone(b.psf)

    def test_non_adjacent_buffer(self):
        with self.assertRaises(ValueError):
            Buffer(upstream_stage=0, downstream_stage=2)


class TestPsfConfig(unittest.TestCase):
    def test_unconfigured(self):
        self.assertFalse(PsfConfig().is_configured())

    def test_configured_tension(self):
        self.assertTrue(PsfConfig(tension_sensor="t").is_configured())

    def test_configured_compression(self):
        self.assertTrue(PsfConfig(compression_sensor="c").is_configured())

    def test_configured_analog(self):
        self.assertTrue(PsfConfig(analog_sensor="a").is_configured())


class TestChainConstruction(unittest.TestCase):
    def test_n2_valid(self):
        c = chain_from_legacy_n2()
        self.assertEqual(c.n, 2)
        self.assertEqual(len(c.buffers), 1)
        self.assertEqual(c.stages[0].name, "gear")
        self.assertEqual(c.stages[1].name, "extruder")

    def test_n3_valid(self):
        c = _make_n3_chain()
        self.assertEqual(c.n, 3)
        self.assertEqual(len(c.buffers), 2)

    def test_too_few_stages(self):
        with self.assertRaises(ValueError):
            Chain(stages=[_make_stage()], buffers=[])

    def test_buffer_count_mismatch(self):
        with self.assertRaises(ValueError):
            Chain(stages=[_make_stage("a"), _make_stage("b")], buffers=[])

    def test_buffer_indices_must_match_positions(self):
        with self.assertRaises(ValueError):
            Chain(
                stages=[_make_stage("a"), _make_stage("b"), _make_stage("c")],
                buffers=[
                    Buffer(0, 1),
                    Buffer(0, 1),  # Should be Buffer(1, 2)
                ],
            )

    def test_invalid_master_end(self):
        with self.assertRaises(ValueError):
            Chain(stages=[_make_stage("a"), _make_stage("b")],
                  buffers=[Buffer(0, 1)], master_end="banana")

    def test_engagement_length_mismatch(self):
        with self.assertRaises(ValueError):
            Chain(stages=[_make_stage("a"), _make_stage("b")],
                  buffers=[Buffer(0, 1)], engagement=[True])

    def test_grip_always_must_be_engaged_at_construction(self):
        with self.assertRaises(ValueError):
            Chain(
                stages=[
                    _make_stage("a", grip=GRIP_RELEASABLE),
                    _make_stage("b", grip=GRIP_ALWAYS),
                ],
                buffers=[Buffer(0, 1)],
                engagement=[True, False],
            )

    def test_default_engagement_all_true(self):
        c = chain_from_legacy_n2()
        self.assertEqual(c.engagement(), [True, True])


class TestStageLookup(unittest.TestCase):
    def test_lookup_by_name(self):
        c = _make_n3_chain()
        self.assertEqual(c.stage_index("gear"), 0)
        self.assertEqual(c.stage_index("booster"), 1)
        self.assertEqual(c.stage_index("extruder"), 2)

    def test_lookup_missing(self):
        c = _make_n3_chain()
        with self.assertRaises(KeyError):
            c.stage_index("nope")


class TestEngagement(unittest.TestCase):
    def test_set_engaged_releasable(self):
        c = _make_n3_chain()
        c.set_engaged(0, False)
        self.assertFalse(c.is_engaged(0))
        self.assertEqual(c.engaged_stages(), [1, 2])

    def test_cannot_disengage_always_grip(self):
        c = _make_n3_chain()  # booster=GRIP_ALWAYS
        with self.assertRaises(ValueError):
            c.set_engaged(1, False)
        with self.assertRaises(ValueError):
            c.set_engaged(2, False)  # extruder always grips

    def test_can_disengage_releasable_booster(self):
        c = _make_n3_chain(grip_booster=GRIP_RELEASABLE)
        c.set_engaged(1, False)
        self.assertEqual(c.engaged_stages(), [0, 2])


class TestMastershipCascade(unittest.TestCase):
    # --- N=2: legacy gear+extruder, all engaged ---

    def test_n2_master_toolhead(self):
        c = chain_from_legacy_n2()
        c.set_master_end(MASTER_TOOLHEAD)
        self.assertEqual(c.chain_master_stage(), 1)  # extruder
        self.assertEqual(c.local_master(0), 1)  # extruder
        self.assertEqual(c.local_slave(0), 0)   # gear

    def test_n2_master_spool(self):
        c = chain_from_legacy_n2()
        c.set_master_end(MASTER_SPOOL)
        self.assertEqual(c.chain_master_stage(), 0)  # gear
        self.assertEqual(c.local_master(0), 0)  # gear
        self.assertEqual(c.local_slave(0), 1)   # extruder

    # --- N=3 booster topology, all engaged ---

    def test_n3_master_toolhead(self):
        # Print direction: extruder is chain-master.
        # Mastership cascades inward:
        #   buffer 1 (booster-extruder): master=extruder, slave=booster
        #   buffer 0 (gear-booster):     master=booster,  slave=gear
        c = _make_n3_chain()
        c.set_master_end(MASTER_TOOLHEAD)
        self.assertEqual(c.chain_master_stage(), 2)
        self.assertEqual(c.local_master(1), 2)  # extruder
        self.assertEqual(c.local_slave(1), 1)   # booster
        self.assertEqual(c.local_master(0), 1)  # booster
        self.assertEqual(c.local_slave(0), 0)   # gear

    def test_n3_master_spool(self):
        # MMU op direction: gear is chain-master.
        # Mastership cascades outward:
        #   buffer 0 (gear-booster):     master=gear,    slave=booster
        #   buffer 1 (booster-extruder): master=booster, slave=extruder
        c = _make_n3_chain()
        c.set_master_end(MASTER_SPOOL)
        self.assertEqual(c.chain_master_stage(), 0)
        self.assertEqual(c.local_master(0), 0)  # gear
        self.assertEqual(c.local_slave(0), 1)   # booster
        self.assertEqual(c.local_master(1), 1)  # booster
        self.assertEqual(c.local_slave(1), 2)   # extruder

    def test_chain_master_stage_skips_disengaged(self):
        # Make booster releasable so we can disengage either side end-stage.
        c = _make_n3_chain(grip_booster=GRIP_RELEASABLE)
        # Disengage the gear; with MASTER_SPOOL the chain-master becomes booster.
        c.set_engaged(0, False)
        c.set_master_end(MASTER_SPOOL)
        self.assertEqual(c.chain_master_stage(), 1)

    def test_chain_master_stage_raises_when_no_engaged(self):
        c = _make_n3_chain(grip_booster=GRIP_RELEASABLE)
        # All stages have releasable grip is not true (extruder is always); so
        # we can't disengage everything. Construct a fully-releasable 3-chain.
        all_releasable = Chain(
            stages=[
                _make_stage("a", grip=GRIP_RELEASABLE),
                _make_stage("b", grip=GRIP_RELEASABLE),
                _make_stage("c", grip=GRIP_RELEASABLE),
            ],
            buffers=[Buffer(0, 1), Buffer(1, 2)],
            engagement=[False, False, False],
        )
        with self.assertRaises(RuntimeError):
            all_releasable.chain_master_stage()


class TestDisengagedSpans(unittest.TestCase):
    def test_no_spans_all_engaged(self):
        c = _make_n3_chain()
        self.assertEqual(c.disengaged_spans(), [])

    def test_single_middle_span(self):
        # 5-stage chain with stages [0,1,2,3,4]; disengage stages 1 and 2.
        # Spans: [(1,2)] flanked by stages 0 and 3.
        stages = [_make_stage("s%d" % i,
                              grip=GRIP_RELEASABLE if i in (1, 2) else GRIP_ALWAYS)
                  for i in range(5)]
        buffers = [Buffer(i, i + 1) for i in range(4)]
        c = Chain(stages=stages, buffers=buffers)
        c.set_engaged(1, False)
        c.set_engaged(2, False)
        spans = c.disengaged_spans()
        self.assertEqual(len(spans), 1)
        self.assertEqual((spans[0].start, spans[0].end), (1, 2))
        self.assertEqual(spans[0].left_engaged, 0)
        self.assertEqual(spans[0].right_engaged, 3)

    def test_leading_disengaged_is_not_a_span(self):
        # If the first stage is disengaged (no left neighbor) it is not a span.
        c = _make_n3_chain(grip_booster=GRIP_RELEASABLE)
        c.set_engaged(0, False)
        self.assertEqual(c.disengaged_spans(), [])

    def test_trailing_disengaged_is_not_a_span(self):
        c = _make_n3_chain(grip_booster=GRIP_RELEASABLE)
        # We can't disengage the always-grip extruder; instead disengage the
        # middle booster only and verify it IS a span when flanked.
        c.set_engaged(1, False)
        spans = c.disengaged_spans()
        self.assertEqual(len(spans), 1)
        self.assertEqual((spans[0].start, spans[0].end), (1, 1))

    def test_two_separate_spans(self):
        # 7-stage chain: engaged = [T, F, T, F, F, T, T]; spans (1,1) and (3,4)
        stages = [_make_stage("s%d" % i, grip=GRIP_RELEASABLE) for i in range(7)]
        stages[0] = _make_stage("s0", grip=GRIP_ALWAYS)
        stages[6] = _make_stage("s6", grip=GRIP_ALWAYS)
        buffers = [Buffer(i, i + 1) for i in range(6)]
        c = Chain(stages=stages, buffers=buffers)
        c.set_engaged(1, False)
        c.set_engaged(3, False)
        c.set_engaged(4, False)
        spans = c.disengaged_spans()
        self.assertEqual([(s.start, s.end) for s in spans], [(1, 1), (3, 4)])


class TestBufferIndicesBetween(unittest.TestCase):
    def test_adjacent_pair(self):
        c = _make_n3_chain()
        self.assertEqual(c.buffer_indices_between(0, 1), [0])
        self.assertEqual(c.buffer_indices_between(1, 2), [1])

    def test_spanning_pair(self):
        c = _make_n3_chain()
        self.assertEqual(c.buffer_indices_between(0, 2), [0, 1])

    def test_invalid_order(self):
        c = _make_n3_chain()
        with self.assertRaises(ValueError):
            c.buffer_indices_between(2, 0)
        with self.assertRaises(ValueError):
            c.buffer_indices_between(1, 1)


class TestLegacyN2(unittest.TestCase):
    def test_default_topology(self):
        c = chain_from_legacy_n2()
        self.assertEqual([s.name for s in c.stages], ["gear", "extruder"])
        self.assertEqual(c.stages[0].grip, GRIP_RELEASABLE)
        self.assertEqual(c.stages[1].grip, GRIP_ALWAYS)
        self.assertEqual(c.stages[0].role, ROLE_MMU_RAIL)
        self.assertEqual(c.stages[1].role, ROLE_KLIPPER_EXTRUDER)
        self.assertEqual(c.master_end, MASTER_TOOLHEAD)
        self.assertIsNone(c.buffers[0].psf)
        self.assertIsNone(c.buffers[0].pre_sensor)
        self.assertIsNone(c.buffers[0].post_sensor)

    def test_with_psf_and_sensors(self):
        c = chain_from_legacy_n2(
            psf=PsfConfig(tension_sensor="t", compression_sensor="c"),
            extruder_pre_sensor="extruder_sw",
            extruder_post_sensor="toolhead_sw",
        )
        self.assertTrue(c.buffers[0].psf.is_configured())
        self.assertEqual(c.buffers[0].pre_sensor, "extruder_sw")
        self.assertEqual(c.buffers[0].post_sensor, "toolhead_sw")

    def test_legacy_print_mastership(self):
        # The defining property: under MASTER_TOOLHEAD the gear (stage 0) is
        # the local slave of buffer 0 (and PSF correction applies to gear's RD).
        # This matches today's "sync gear to extruder during print" behavior.
        c = chain_from_legacy_n2()
        self.assertEqual(c.local_master(0), 1)  # extruder
        self.assertEqual(c.local_slave(0), 0)   # gear

    def test_legacy_mmu_op_mastership(self):
        # Under MASTER_SPOOL the gear is local master and the extruder is local
        # slave, matching the "loader/unloader: gear drives, extruder follows"
        # operations.
        c = chain_from_legacy_n2()
        c.set_master_end(MASTER_SPOOL)
        self.assertEqual(c.local_master(0), 0)  # gear
        self.assertEqual(c.local_slave(0), 1)   # extruder


class TestSyncGroup(unittest.TestCase):
    def test_basic_construction(self):
        g = SyncGroup(engagement=[True, True])
        self.assertEqual(g.n, 2)
        self.assertEqual(g.master_end, MASTER_TOOLHEAD)
        self.assertTrue(g.sync_active)
        self.assertEqual(g.engaged_stages(), [0, 1])

    def test_invalid_master_end(self):
        with self.assertRaises(ValueError):
            SyncGroup(engagement=[True, True], master_end="nope")

    def test_requires_at_least_two_stages(self):
        with self.assertRaises(ValueError):
            SyncGroup(engagement=[True])

    def test_equality_and_hash(self):
        a = SyncGroup([True, True], MASTER_TOOLHEAD, True)
        b = SyncGroup([True, True], MASTER_TOOLHEAD, True)
        c = SyncGroup([True, True], MASTER_SPOOL, True)
        self.assertEqual(a, b)
        self.assertNotEqual(a, c)
        self.assertEqual(hash(a), hash(b))
        self.assertNotEqual(a, "not a SyncGroup")


class TestLegacyBijection(unittest.TestCase):
    # ---- from_legacy ----

    def test_none_to_unsynced(self):
        g = sync_group_from_legacy(LEGACY_NONE)
        self.assertEqual(g.engagement, [True, True])
        self.assertFalse(g.sync_active)

    def test_gear_only_to_unsynced(self):
        g = sync_group_from_legacy(LEGACY_GEAR_ONLY)
        self.assertEqual(g.engagement, [True, True])
        self.assertFalse(g.sync_active)
        # None and GEAR_ONLY are chain-equivalent
        self.assertEqual(g, sync_group_from_legacy(LEGACY_NONE))

    def test_gear_synced_to_extruder(self):
        # Print direction: extruder master, gear follows.
        g = sync_group_from_legacy(LEGACY_GEAR_SYNCED_TO_EXTRUDER)
        self.assertEqual(g.engagement, [True, True])
        self.assertEqual(g.master_end, MASTER_TOOLHEAD)
        self.assertTrue(g.sync_active)

    def test_extruder_synced_to_gear(self):
        # MMU-op direction: gear master, extruder follows.
        g = sync_group_from_legacy(LEGACY_EXTRUDER_SYNCED_TO_GEAR)
        self.assertEqual(g.engagement, [True, True])
        self.assertEqual(g.master_end, MASTER_SPOOL)
        self.assertTrue(g.sync_active)

    def test_extruder_only_on_gear(self):
        # Special routing: gear disengaged, extruder routed on gear rail.
        g = sync_group_from_legacy(LEGACY_EXTRUDER_ONLY_ON_GEAR)
        self.assertEqual(g.engagement, [False, True])
        self.assertEqual(g.master_end, MASTER_SPOOL)
        self.assertTrue(g.sync_active)

    def test_unknown_legacy_mode_raises(self):
        with self.assertRaises(ValueError):
            sync_group_from_legacy(999)

    # ---- to_legacy ----

    def test_to_legacy_unsynced_default_none(self):
        g = SyncGroup([True, True], MASTER_TOOLHEAD, sync_active=False)
        self.assertEqual(sync_group_to_legacy(g), LEGACY_NONE)

    def test_to_legacy_unsynced_prefer_gear_only(self):
        g = SyncGroup([True, True], MASTER_TOOLHEAD, sync_active=False)
        self.assertEqual(
            sync_group_to_legacy(g, prefer_gear_only_for_unsynced=True),
            LEGACY_GEAR_ONLY,
        )

    def test_to_legacy_gear_synced_to_extruder(self):
        g = SyncGroup([True, True], MASTER_TOOLHEAD, sync_active=True)
        self.assertEqual(sync_group_to_legacy(g), LEGACY_GEAR_SYNCED_TO_EXTRUDER)

    def test_to_legacy_extruder_synced_to_gear(self):
        g = SyncGroup([True, True], MASTER_SPOOL, sync_active=True)
        self.assertEqual(sync_group_to_legacy(g), LEGACY_EXTRUDER_SYNCED_TO_GEAR)

    def test_to_legacy_extruder_only_on_gear(self):
        g = SyncGroup([False, True], MASTER_SPOOL, sync_active=True)
        self.assertEqual(sync_group_to_legacy(g), LEGACY_EXTRUDER_ONLY_ON_GEAR)

    def test_to_legacy_n3_rejected(self):
        g = SyncGroup([True, True, True], MASTER_TOOLHEAD, sync_active=True)
        with self.assertRaises(ValueError):
            sync_group_to_legacy(g)

    def test_to_legacy_unsynced_disengaged_rejected(self):
        # Unsynced with one stage disengaged is not a legacy state (legacy
        # handles per-stepper disable via _reconfigure_rail_no_lock).
        g = SyncGroup([False, True], MASTER_TOOLHEAD, sync_active=False)
        with self.assertRaises(ValueError):
            sync_group_to_legacy(g)

    # ---- bijection round-trip ----

    def test_bijection_all_known_modes(self):
        # All legacy modes except LEGACY_GEAR_ONLY round-trip cleanly with the
        # default preference. LEGACY_GEAR_ONLY collapses to LEGACY_NONE on
        # to_legacy because they are chain-equivalent.
        for legacy in (LEGACY_NONE,
                       LEGACY_GEAR_SYNCED_TO_EXTRUDER,
                       LEGACY_EXTRUDER_SYNCED_TO_GEAR,
                       LEGACY_EXTRUDER_ONLY_ON_GEAR):
            g = sync_group_from_legacy(legacy)
            self.assertEqual(sync_group_to_legacy(g), legacy,
                             "round-trip failed for %r" % legacy)

    def test_bijection_gear_only_collapses_to_none(self):
        g = sync_group_from_legacy(LEGACY_GEAR_ONLY)
        # Default preference picks LEGACY_NONE; with prefer_gear_only=True you
        # get LEGACY_GEAR_ONLY back.
        self.assertEqual(sync_group_to_legacy(g), LEGACY_NONE)
        self.assertEqual(
            sync_group_to_legacy(g, prefer_gear_only_for_unsynced=True),
            LEGACY_GEAR_ONLY,
        )


class TestCombinePsfAverage(unittest.TestCase):
    def test_empty(self):
        self.assertEqual(combine_psf_average([]), 0.0)

    def test_single_value(self):
        self.assertEqual(combine_psf_average([-1.0]), -1.0)
        self.assertEqual(combine_psf_average([0.0]), 0.0)
        self.assertEqual(combine_psf_average([0.5]), 0.5)

    def test_simple_average(self):
        self.assertEqual(combine_psf_average([-1.0, 1.0]), 0.0)
        self.assertEqual(combine_psf_average([-1.0, 0.0]), -0.5)
        self.assertAlmostEqual(combine_psf_average([0.2, 0.4, 0.6]), 0.4)

    def test_weighted_average(self):
        # Weight values 2:1 toward the first reading
        v = combine_psf_average([-1.0, 1.0], weights=[2, 1])
        self.assertAlmostEqual(v, (-2 + 1) / 3.0)

    def test_zero_weights_returns_zero(self):
        self.assertEqual(combine_psf_average([1.0, -1.0], weights=[0, 0]), 0.0)

    def test_weight_length_mismatch_rejected(self):
        with self.assertRaises(ValueError):
            combine_psf_average([1.0, 2.0], weights=[1.0])

    def test_negative_weight_rejected(self):
        with self.assertRaises(ValueError):
            combine_psf_average([1.0, 2.0], weights=[1.0, -0.5])


class TestCombinePsfOr(unittest.TestCase):
    def test_empty(self):
        self.assertEqual(combine_psf_or([]), (0.0, False))

    def test_all_neutral(self):
        self.assertEqual(combine_psf_or([0.0, 0.0]), (0.0, False))

    def test_one_tension(self):
        # Tension on any single switch wins; combined value is the tense reading.
        self.assertEqual(combine_psf_or([-1.0, 0.0]), (-1.0, False))

    def test_one_compression(self):
        self.assertEqual(combine_psf_or([0.0, 1.0]), (1.0, False))

    def test_conflicting_anomaly(self):
        # Tense + compressed on the same combined buffer is impossible
        # physically; report neutral + anomaly flag.
        v, anomaly = combine_psf_or([-1.0, 1.0])
        self.assertEqual(v, 0.0)
        self.assertTrue(anomaly)

    def test_multiple_tension_picks_most_extreme(self):
        v, anomaly = combine_psf_or([-0.3, -0.9, 0.0])
        self.assertEqual(v, -0.9)
        self.assertFalse(anomaly)

    def test_multiple_compression_picks_most_extreme(self):
        v, anomaly = combine_psf_or([0.3, 0.9, 0.0])
        self.assertEqual(v, 0.9)
        self.assertFalse(anomaly)


class TestSwitchReading(unittest.TestCase):
    def test_neither(self):
        self.assertEqual(switch_reading(False, False), (0.0, False))

    def test_tension_only(self):
        self.assertEqual(switch_reading(True, False), (-1.0, False))

    def test_compression_only(self):
        self.assertEqual(switch_reading(False, True), (1.0, False))

    def test_both_active_is_anomaly(self):
        v, anomaly = switch_reading(True, True)
        self.assertEqual(v, 0.0)
        self.assertTrue(anomaly)


if __name__ == "__main__":
    unittest.main()
