"""Tests for autocar state machine."""
import pytest

STATES = ["BOOTUP", "IDLE", "AUTONOMY", "PAUSED", "ESTOP"]
TRANSITIONS = {
    "BOOTUP": ["IDLE"],
    "IDLE": ["AUTONOMY"],
    "AUTONOMY": ["PAUSED", "ESTOP"],
    "PAUSED": ["AUTONOMY", "ESTOP", "IDLE"],
    "ESTOP": ["IDLE"],
}


def test_all_states_defined():
    assert len(STATES) == 5


def test_estop_blocks_all():
    assert "AUTONOMY" not in TRANSITIONS["ESTOP"]
    assert "PAUSED" not in TRANSITIONS["ESTOP"]


def test_autonomy_cannot_go_to_idle_directly():
    assert "IDLE" not in TRANSITIONS["AUTONOMY"]


def test_paused_can_resume():
    assert "AUTONOMY" in TRANSITIONS["PAUSED"]


def test_bootup_only_to_idle():
    assert TRANSITIONS["BOOTUP"] == ["IDLE"]


def test_all_states_have_transitions():
    for state in STATES:
        assert len(TRANSITIONS[state]) > 0
