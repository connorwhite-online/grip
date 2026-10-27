"""Every dimension that drives the arm lives here. Units are millimetres.

Vendor dimensions marked VERIFY are nominal catalogue values; check them against the
part you actually buy before printing.
"""
from dataclasses import dataclass, field


@dataclass(frozen=True)
class Bearing:
    name: str
    id: float
    od: float
    w: float


@dataclass(frozen=True)
class Stepper:
    name: str
    frame: float          # square flange size
    length: float         # body length
    bolt_spacing: float   # square bolt pattern
    bolt_d: float         # clearance hole for mounting screws
    pilot_d: float        # centring boss diameter
    pilot_h: float
    shaft_d: float
    shaft_len: float


NEMA17_60 = Stepper("NEMA17 60mm 0.65Nm", 42.3, 60.0, 31.0, 3.4, 22.0, 2.0, 5.0, 24.0)
NEMA17_34 = Stepper("NEMA17 34mm pancake 0.28Nm", 42.3, 34.0, 31.0, 3.4, 22.0, 2.0, 5.0, 20.0)
NEMA23_56 = Stepper("NEMA23 56mm 1.2Nm", 56.4, 56.0, 47.14, 5.2, 38.1, 1.6, 8.0, 21.0)

B6801 = Bearing("6801-2RS", 12, 21, 5)
B6802 = Bearing("6802-2RS", 15, 24, 5)
B6805 = Bearing("6805-2RS", 25, 37, 7)
B6808 = Bearing("6808-2RS", 40, 52, 7)
B6812 = Bearing("6812-2RS", 60, 78, 10)


@dataclass(frozen=True)
class Cyclo:
    """One single-stage cycloidal reducer. Ratio = n_pins - 1."""
    name: str
    motor: Stepper
    n_pins: int            # ring pins (dowels) in the housing
    pin_circle_r: float    # R
    pin_r: float           # ring pin radius (dowel / 2)
    ecc: float             # eccentricity e
    disc_t: float          # each of the two discs
    n_out: int             # output pins
    out_circle_r: float
    out_pin_d: float
    out_bearing: Bearing
    ecc_bearing: Bearing
    wall: float = 5.0
    clearance: float = 0.15   # SLS fit clearance on sliding surfaces

    def __post_init__(self):
        # lobe tips swing out to R - rp + 2e; keep them inside the pin circle so the
        # housing bore can stay at R and the ring pins stay more than half captured
        assert 2 * self.ecc < self.pin_r, f"{self.name}: eccentricity too large for pin size"

    @property
    def ratio(self) -> int:
        return self.n_pins - 1

    @property
    def housing_r(self) -> float:
        return self.pin_circle_r + self.pin_r + self.wall

    @property
    def flange_t(self) -> float:
        return self.out_bearing.w

    @property
    def stack(self) -> float:
        """Housing height from motor face to output face."""
        return self.out_bearing.w + 1.0 + 2 * self.disc_t + 0.5 + 1.0 + 5.0


# Joint drives (version B)
J2_DRIVE = Cyclo("J2 shoulder 30:1", NEMA23_56, 31, 44.0, 2.0, 0.9, 7.0, 8, 26.0, 5.0, B6812, B6802)
J13_DRIVE = Cyclo("J1/J3 25:1", NEMA17_60, 26, 36.0, 2.0, 0.9, 6.0, 6, 21.0, 4.0, B6808, B6801)
WRIST_DRIVE = Cyclo("J4-J6 wrist 15:1", NEMA17_34, 16, 24.0, 1.5, 0.7, 5.0, 6, 14.0, 3.0, B6805, B6801, wall=4.0)


@dataclass(frozen=True)
class ArmGeometry:
    """Kinematic dimensions in the upright home pose (all joint axes intersect x = 0)."""
    z_j1_out: float = 120.0     # J1 output face above the table
    z_j2: float = 200.0         # shoulder axis height
    upper_arm: float = 210.0    # J2 -> J3
    forearm: float = 185.0      # J3 -> J5
    wrist: float = 105.0        # J5 -> tool flange
    y_j2_out: float = -30.0     # J2 output face (faces -Y)
    y_upper: float = -48.0      # upper arm mid-plane
    y_forearm: float = -25.0    # forearm / wrist centre line
    base_r: float = 85.0
    shell_wall: float = 2.0     # SLS PA12 wall; peak bending stress stays under ~6 MPa

    @property
    def z_j3(self):
        return self.z_j2 + self.upper_arm

    @property
    def z_j5(self):
        return self.z_j3 + self.forearm

    @property
    def z_flange(self):
        return self.z_j5 + self.wrist

    @property
    def reach(self):
        return self.upper_arm + self.forearm + self.wrist


GEOM = ArmGeometry()
