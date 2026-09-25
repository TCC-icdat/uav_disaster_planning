from __future__ import annotations

import hashlib
from math import hypot
from pathlib import Path

import pytest

from uav_planning.canonical.eo import EOEventGenerator, EOSweepPlanner
from uav_planning.canonical.loader import load_canonical_scenario
from uav_planning.canonical.sar import (
    CanonicalRouteEvaluator,
    CanonicalTaskAdapter,
    build_sar_service_geometries,
)
from uav_planning.models import Pose2D
from uav_planning.routing.evaluator import DubinsTravelTimeProvider


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs" / "canonical" / "post_earthquake_v1.yaml"


@pytest.fixture(scope="module")
def canonical_data():
    scenario = load_canonical_scenario(CONFIG)
    sweep = EOSweepPlanner(scenario).plan()
    releases = EOEventGenerator(scenario, sweep).releases()
    services = build_sar_service_geometries(scenario)
    return scenario, sweep, releases, services


def test_eo_footprint_dimensions(canonical_data) -> None:
    _, sweep, _, _ = canonical_data
    assert sweep.footprint.cross_track_width_m == pytest.approx(692.820323, abs=1e-6)
    assert sweep.footprint.along_track_length_m == pytest.approx(497.056275, abs=1e-6)
    assert sweep.lane_spacing_m == pytest.approx(554.256258, abs=1e-6)
    assert sweep.lane_count == 11


def test_all_hidden_aois_are_detected(canonical_data) -> None:
    scenario, _, releases, _ = canonical_data
    hidden_aois = {aoi.aoi_id for aoi in scenario.aois if not aoi.initially_known}
    detected = {
        event.aoi_id
        for event in releases
        if not event.initially_known and event.path_sample_index is not None
    }
    assert detected == hidden_aois
    assert len(detected) == 9


def test_release_time_comes_from_eo_coverage(canonical_data) -> None:
    scenario, sweep, releases, _ = canonical_data
    aoi_by_id = {aoi.aoi_id: aoi for aoi in scenario.aois}
    for event in releases:
        if event.initially_known:
            assert event.release_time == 0.0
            continue
        index = event.path_sample_index
        assert index is not None
        row = sweep.samples[index]
        pose = Pose2D(float(row[0]), float(row[1]), float(row[2]))
        assert sweep.footprint.fully_contains(pose, aoi_by_id[event.aoi_id].polygon)
        assert event.release_time == pytest.approx(
            sweep.cumulative_distance_m[index] / scenario.eo.speed_mps
        )
        if index > 0:
            prior = sweep.samples[index - 1]
            prior_pose = Pose2D(float(prior[0]), float(prior[1]), float(prior[2]))
            assert not sweep.footprint.fully_contains(
                prior_pose, aoi_by_id[event.aoi_id].polygon
            )


def test_all_aoi_sizes_fit_single_sar_swath(canonical_data) -> None:
    scenario, _, _, services = canonical_data
    for aoi in scenario.aois:
        assert aoi.width_m <= services[aoi.task_id].swath_width_m
        assert services[aoi.task_id].aoi_fits


def test_sar_service_segment_covers_aoi(canonical_data) -> None:
    scenario, _, _, services = canonical_data
    for aoi in scenario.aois:
        service = services[aoi.task_id]
        assert hypot(
            service.exit_pose.x - service.entry_pose.x,
            service.exit_pose.y - service.entry_pose.y,
        ) == pytest.approx(service.scan_length_m)
        assert service.scan_length_m == max(aoi.length_m + 300.0, 600.0)
        assert service.service_time_s == pytest.approx(
            service.scan_length_m / scenario.sar.speed_mps
        )
        assert service.aoi_fits


def test_canonical_config_is_deterministic() -> None:
    first = load_canonical_scenario(CONFIG)
    second = load_canonical_scenario(CONFIG)
    assert first == second
    assert tuple(aoi.aoi_id for aoi in first.aois) == tuple(
        f"A{index:02d}" for index in range(1, 13)
    )
    assert first.aois[0].center == (1000.0, 4500.0)
    assert first.aois[-1].center == (2500.0, 4000.0)


def test_canonical_route_evaluator_uses_scan_exit_pose(canonical_data) -> None:
    scenario, _, releases, services = canonical_data
    tasks = CanonicalTaskAdapter(scenario, services, releases).tasks()
    evaluator = CanonicalRouteEvaluator(DubinsTravelTimeProvider(), services)
    for task in tasks:
        assert evaluator.task_completion_pose(task) == services[task.task_id].exit_pose


def test_legacy_formal_experiments_unchanged() -> None:
    expected = {
        "E1": "90ce8f9469d0c47d1b536b54b169c9f9b98d20874b89891780748a950aa776e0",
        "E2": "75cd5d083f763064bdbcf9aa59875cf22121dbcb8be7dba3aafe11e15e0c8da8",
        "E3": "3ffddb04e403558ab7131c000eff6526ea033b9e88cfbe051b0cc440d7bbf59c",
        "E4": "4140284713cba758ab7003f95710f8dcf0a6f76a6410ce4b443e4d40e9adc58d",
    }
    for experiment, digest in expected.items():
        raw = ROOT / "results" / "formal" / experiment / "raw.csv"
        assert hashlib.sha256(raw.read_bytes()).hexdigest() == digest
