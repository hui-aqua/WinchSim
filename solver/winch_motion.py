"""Shared prescribed winch motion profiles used by quasi-static analyses."""

import math


def prescribed_motion(winch, time_s, duration_s=None):
    """Return vertical displacement and signed speed for the configured profile.

    ``schedule`` rows are piecewise-constant commands keyed by their start
    time. ``in`` retrieves the rope and moves the load upward; ``out`` pays
    out rope and moves it downward. Legacy single-speed/reversal inputs remain
    supported.
    """
    mode = winch.get("mode", "prescribed_speed")
    if mode != "prescribed_speed":
        raise ValueError(f"Unsupported winch mode: {mode!r}")

    schedule = winch.get("schedule")
    if "schedule" in winch:
        if duration_s is None:
            raise ValueError("solver.duration_s is required for a winch motion schedule")
        duration = float(duration_s)
        if not math.isfinite(duration) or duration < 0:
            raise ValueError("solver.duration_s must be finite and non-negative")
        rows = []
        for index, row in enumerate(schedule):
            start = float(row["time_s"])
            speed = float(row["speed_m_per_s"])
            direction = row["direction"]
            if not math.isfinite(start) or not math.isfinite(speed) or speed < 0:
                raise ValueError(f"winch.schedule row {index + 1} has invalid time or speed")
            if direction not in ("in", "out"):
                raise ValueError(f"winch.schedule row {index + 1} direction must be 'in' or 'out'")
            if start < 0 or (start >= duration and duration > 0):
                raise ValueError(f"winch.schedule row {index + 1} time_s must be within the simulation duration")
            if index == 0 and start != 0:
                raise ValueError("winch.schedule must start at time_s=0")
            if rows and start <= rows[-1][0]:
                raise ValueError("winch.schedule time_s values must be strictly increasing")
            rows.append((start, speed, 1.0 if direction == "in" else -1.0))
        if not rows:
            raise ValueError("winch.schedule must contain at least one row")

        elapsed = min(max(float(time_s), 0.0), duration)
        displacement = 0.0
        for index, (start, speed, direction) in enumerate(rows):
            end = rows[index + 1][0] if index + 1 < len(rows) else duration
            displacement += direction * speed * max(0.0, min(elapsed, end) - start)
            if elapsed < end or index == len(rows) - 1:
                return displacement, direction * speed

    speed = abs(float(winch["payout_or_retrieval_speed_m_per_s"]))
    direction_name = winch.get("direction", "upward")
    if direction_name not in ("upward", "downward"):
        raise ValueError("winch.direction must be 'upward' or 'downward'")
    direction = -1.0 if direction_name == "downward" else 1.0
    reversal_time = winch.get("reversal_time_s")
    if reversal_time is None:
        return direction * speed * time_s, direction * speed
    reversal_time = float(reversal_time)
    if reversal_time <= 0:
        raise ValueError("winch.reversal_time_s must be positive")
    elapsed_up = min(max(time_s, 0.0), reversal_time)
    elapsed_down = max(0.0, time_s - reversal_time)
    displacement = direction * speed * (elapsed_up - elapsed_down)
    signed_speed = direction * speed if time_s < reversal_time else -direction * speed
    return displacement, signed_speed
