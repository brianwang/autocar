"""Tests for voice pipeline intent parsing."""
import pytest


INTENT_RULES = [
    ("多远", "QUERY"),
    ("继续", "RESUME"),
    ("去", "NAVIGATE"),
    ("到", "NAVIGATE"),
    ("停", "PAUSE"),
    ("走", "OVERRIDE_PLAN"),
    ("绕", "REPLAN"),
]


def parse_intent(text: str) -> str:
    for keyword, intent in INTENT_RULES:
        if keyword in text:
            return intent
    return "UNKNOWN"


def test_navigate_intent():
    assert parse_intent("去3号楼") == "NAVIGATE"
    assert parse_intent("到菜鸟驿站") == "NAVIGATE"


def test_pause_intent():
    assert parse_intent("停一下") == "PAUSE"


def test_replan_intent():
    assert parse_intent("前面有人绕一下") == "REPLAN"


def test_query_intent():
    assert parse_intent("还有多远") == "QUERY"


def test_unknown_intent():
    assert parse_intent("今天天气怎么样") == "UNKNOWN"


def test_resume_intent():
    assert parse_intent("继续走") == "RESUME"
