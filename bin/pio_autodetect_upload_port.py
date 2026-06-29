Import("env")

# Auto-select upload port if user did not explicitly pass one.
# This helps when ESP32-S3 re-enumerates to a different COM after reboot/bootloader entry.


def _normalize(value):
    return str(value).upper() if value is not None else ""


def _has_user_upload_port():
    # Respect already resolved upload port from PlatformIO (e.g. --upload-port COMx).
    current = str(env.get("UPLOAD_PORT", "")).strip()
    if current:
        return True

    # Honor CLI override: pio run ... --upload-port COMx
    options = env.GetProjectOption("upload_port", "")
    try:
        cli = env.GetOption("upload_port")
    except Exception:
        cli = ""
    return bool(cli) or bool(options)


def _pick_port():
    try:
        from serial.tools import list_ports
    except Exception:
        return None

    ports = list(list_ports.comports())
    if not ports:
        return None

    pioenv = _normalize(env.get("PIOENV", ""))

    # For RP targets, only consider Raspberry Pi VID devices.
    # Never fall back to unrelated USB-UART bridges (e.g. CH340 on COM3).
    if pioenv.startswith("RP2040") or pioenv.startswith("RP2350"):
        preferred = [
            (0x2E8A, 0x000A),
            (0x2E8A, None),
        ]
        for vid, pid in preferred:
            for p in ports:
                p_vid = getattr(p, "vid", None)
                p_pid = getattr(p, "pid", None)
                if p_vid != vid:
                    continue
                if pid is not None and p_pid != pid:
                    continue
                return p.device
        return None

    # Prefer native ESP USB first, then CH340 bridge as fallback.
    preferred = [
        (0x303A, 0x1001),
        (0x303A, None),
        (0x1A86, 0x5722),
    ]

    for vid, pid in preferred:
        for p in ports:
            p_vid = getattr(p, "vid", None)
            p_pid = getattr(p, "pid", None)
            if p_vid != vid:
                continue
            if pid is not None and p_pid != pid:
                continue
            return p.device

    # Last resort: any serial USB device
    for p in ports:
        hwid = _normalize(getattr(p, "hwid", ""))
        if "USB" in hwid or "VID:PID" in hwid:
            return p.device

    return None


if not _has_user_upload_port():
    selected = _pick_port()
    if selected:
        print("[upload-port] Auto-selected {}".format(selected))
        env.Replace(UPLOAD_PORT=selected)
