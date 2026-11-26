"""Formatting shared by the ZPNet operator displays."""


def altitude_text(altitude_m, *, signed=False) -> str:
    """Render GNSS or barometric altitude as integer meters (feet)."""
    if altitude_m is None:
        return "---"
    meters = float(altitude_m)
    spec = "+,d" if signed else ",d"
    return f"{round(meters):{spec}} ({round(meters / 0.3048):{spec}})"
