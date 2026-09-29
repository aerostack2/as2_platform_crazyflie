"""
Patch crazyflie_cpp sources to add sendArmingRequest().

Runs with the fetched crazyflie_cpp-src directory as cwd.
Idempotent: each file checks for its own injected symbol before inserting.
"""

import pathlib

CRTP_H  = pathlib.Path("include/crazyflie_cpp/crtp.h")
CF_H    = pathlib.Path("include/crazyflie_cpp/Crazyflie.h")
CF_CPP  = pathlib.Path("src/Crazyflie.cpp")

# ── crtp.h ────────────────────────────────────────────────────────────────────

CRTP_GUARD  = "crtpPlatformArmingRequest"
CRTP_MARKER = "// Port 13 (Platform)\n"
CRTP_INSERT = """\
// Port 13 (Platform) — channel 0 commands
class crtpPlatformArmingRequest
  : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  explicit crtpPlatformArmingRequest(bool arm)
    : Packet(13, 0, 2)
  {
    setPayloadAt<uint8_t>(0, 1);              // PLATFORM_REQUEST_ARMING command id
    setPayloadAt<uint8_t>(1, arm ? 1u : 0u); // 1 = arm, 0 = disarm
  }
};

"""

# ── Crazyflie.h ───────────────────────────────────────────────────────────────

CFH_GUARD  = "void sendArmingRequest(bool arm);"
CFH_MARKER = "  void sendStop();"
CFH_INSERT = "  void sendArmingRequest(bool arm);\n"

# ── Crazyflie.cpp ─────────────────────────────────────────────────────────────

CFCPP_GUARD  = "Crazyflie::sendArmingRequest"
CFCPP_MARKER = "void Crazyflie::sendStop()"
CFCPP_INSERT = """\
void Crazyflie::sendArmingRequest(bool arm)
{
  crtpPlatformArmingRequest req(arm);
  m_connection.send(req);
}

"""


def patch(path: pathlib.Path, guard: str, marker: str, insert: str):
    text = path.read_text()
    if guard in text:
        print(f"  [skip] {path} already patched")
        return
    if marker not in text:
        raise RuntimeError(f"Marker not found in {path}: {marker!r}")
    text = text.replace(marker, insert + marker, 1)
    path.write_text(text)
    print(f"  [patched] {path}")


print("Applying crazyflie_cpp arming patch…")
patch(CRTP_H,  CRTP_GUARD,  CRTP_MARKER, CRTP_INSERT)
patch(CF_H,    CFH_GUARD,   CFH_MARKER,  CFH_INSERT)
patch(CF_CPP,  CFCPP_GUARD, CFCPP_MARKER, CFCPP_INSERT)
print("Done.")
