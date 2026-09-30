"""Pure parsing and request-lineage rules for the Explorer--SCAN protocol."""

from typing import Dict, Optional, Tuple


REFERENCE_SCOPED_STATUSES = {
    "PATH_ACCEPTED", "PATH_TRAJECTORY_READY", "REACHED", "WAIT_TARGET",
    "BLOCKED", "LOCAL_REPAIR_EXHAUSTED", "REFERENCE_PATH_REJECTED",
    "INVALID_REFERENCE_PATH",
}


def parse_planning_status(value: str) -> Tuple[str, Optional[int]]:
    """Decode SCAN status while retaining compatibility with plain statuses."""
    fields = value.split()
    if not fields:
        return "", None
    request_generation = None
    for field in fields[1:]:
        if not field.startswith("request_id="):
            continue
        try:
            parsed = int(field.partition("=")[2])
        except ValueError:
            continue
        if parsed > 0:
            request_generation = parsed
    return fields[0], request_generation


def planning_status_matches_request(
        status: str, request_generation: Optional[int],
        pending_generation: Optional[int],
        active_generation: Optional[int]) -> bool:
    """Return true only when a path-scoped status belongs to this request."""
    if status not in REFERENCE_SCOPED_STATUSES:
        return True
    expected = (pending_generation if status in (
        "PATH_ACCEPTED", "PATH_TRAJECTORY_READY",
        "REFERENCE_PATH_REJECTED", "INVALID_REFERENCE_PATH")
        else active_generation)
    return expected is not None and request_generation == expected


def completion_status_is_new(status: str, request_generation: Optional[int],
                             last_completed: Optional[int]) -> bool:
    """Deduplicate the REACHED event and its repeated WAIT_TARGET state."""
    return (status in ("REACHED", "WAIT_TARGET")
            and request_generation is not None
            and request_generation != last_completed)


def parse_local_execution_event(value: str) -> Optional[Dict[str, object]]:
    """Decode a terminal local-execution event with exact lineage fields."""
    fields = value.split()
    if not fields:
        return None
    parsed: Dict[str, object] = {"event": fields[0]}
    for field in fields[1:]:
        if "=" not in field:
            continue
        key, raw = field.split("=", 1)
        parsed[key] = raw
    try:
        parsed["request_id"] = int(parsed["request_id"])
        parsed["trajectory_id"] = int(parsed["trajectory_id"])
        parsed["terminal_error"] = float(
            parsed.get("terminal_error", "nan"))
    except (KeyError, TypeError, ValueError):
        return None
    return parsed
