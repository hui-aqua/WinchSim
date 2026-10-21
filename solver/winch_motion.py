"""Shared prescribed winch motion profiles used by quasi-static analyses."""


def prescribed_motion(winch, time_s):
    """Return vertical displacement and signed speed for the configured profile.

    A reversal is applied only when ``reversal_time_s`` is present. This keeps
    the profile definition in the model input rather than embedding lift-cycle
    timing in a particular analysis implementation.
    """
    mode = winch.get("mode", "prescribed_speed")
    if mode != "prescribed_speed":
        raise ValueError(f"Unsupported winch mode: {mode!r}")
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
