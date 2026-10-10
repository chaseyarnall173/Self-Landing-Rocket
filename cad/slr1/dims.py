"""SLR-1 master dimensions, in millimetres.

Stations ``s`` are measured from the nose tip, as everywhere else in the
project (docs, simulator, firmware). The CAD frame is the frame SolidWorks
will show:

* origin on the rocket axis at the aft end of the airframe tube;
* +Z points toward the nose, so ``z = L - s``;
* +X points at fin 1 (marked "N" for the pad heading), +Y at fin 2.

Fins sit at 0/90/180/270 deg, legs and petal cut lines at 45/135/225/315 deg.
The ascent motor is offset toward +X, the landing motor toward -X.
"""
import math

IN = 25.4
L = 60 * IN                       # 1524.0 overall airframe length


def z(s):
    """Station (mm from nose tip) -> CAD z (mm above the aft end)."""
    return L - s


# --- airframe tube: LOC Precision 3.9 in kraft -----------------------------
TUBE_OD = 4.0 * IN                # 101.6
TUBE_ID = 3.9 * IN                # 99.06
R_OUT = TUBE_OD / 2
R_IN = TUBE_ID / 2
CPL_OD = TUBE_ID                  # LOC couplers slip-fit inside the tube
CPL_WALL = 1.8
R_CPL_IN = CPL_OD / 2 - CPL_WALL  # 47.73
KERF = 1.0                        # saw cut between petals / door and tube

# --- nose: printed PETG tangent ogive --------------------------------------
NOSE_LEN = 12.75 * IN             # 323.85 exposed
NOSE_SHOULDER = 2.0 * IN          # 50.8; the airbrake sleeve needs the rest of the nose ring
NOSE_WALL = 0.8
NOSE_SHOULDER_WALL = 1.2
NOSE_TIP_R = 3.0                  # blunted tip radius
NOSE_SCREW_S = NOSE_LEN + NOSE_SHOULDER / 2   # 3 x M3 radial screws (nose is removable)

# --- tube pieces (stations) -------------------------------------------------
PETAL_S = (395.0, 545.0)          # airbrake band (150 mm), cut into 4 petals
NOSE_RING_S = (NOSE_LEN, PETAL_S[0])
BAND_S = (768.0, 793.0)           # avionics-bay switch band
FWD_TUBE_S = (PETAL_S[1], BAND_S[0])
AFT_TUBE_S = (BAND_S[1], L)
TUBE_TOTAL = (NOSE_RING_S[1] - NOSE_RING_S[0]) + (PETAL_S[1] - PETAL_S[0]) \
    + (FWD_TUBE_S[1] - FWD_TUBE_S[0]) + (BAND_S[1] - BAND_S[0]) \
    + (AFT_TUBE_S[1] - AFT_TUBE_S[0])          # 1200 mm = 47.25 in

# --- top airbrakes ----------------------------------------------------------
SLEEVE_S = (376.0, 565.0)         # coupler sleeve: 19 mm into the nose ring, 20 mm aft; windowed
PETAL_AZ = (0.0, 90.0, 180.0, 270.0)         # petal centres (cuts on leg lines)
PETAL_OPEN_DEG = 90.0
PETAL_HINGE_R = R_OUT + 2.2       # hinge-pin axis, just proud of the skin
PETAL_HINGE_W = 28.0
LATCH_S = 540.0                   # latch ring / petal tabs, near the aft edge

# --- backup chute bay -------------------------------------------------------
RECOV_BH_S = (565.0, 568.175)     # 1/8 in ply bulkhead with U-bolt
DOOR_S = (585.0, 690.0)
DOOR_AZ = 90.0
DOOR_HALF_DEG = 39.5              # ~70 mm wide

# --- avionics bay ------------------------------------------------------------
AVBAY_S = (700.0, 860.0)          # coupler
PLY = 3.175                       # 1/8 in birch ply
SLED_S = (712.0, 848.0)
AV_ROD_Y = 40.0                   # 2 x M3 threaded rods at (0, +-40)
KEY_AZ = 180.0                    # arm-key switch through the band
SWITCH_AZ = 160.0                 # main power switch
LEGPIN_AZ = 67.5                  # leg-release pin through the band (between legs)
STATIC_PORT_AZ = (90.0, 210.0, 330.0)

# --- aft end -----------------------------------------------------------------
AFT_CPL_LEN = 6 * IN              # 152.4 internal carbon liner (2 wraps of 200 gsm)
AFT_LINER_T = 0.5
R_AFT_IN = R_IN - AFT_LINER_T     # 49.03 bore of the motor bay

FIN_SCALE = 0.9                   # 90 % of the original 4.7/2.4/3.0/2.3 in fin (stability re-balanced)
FIN_ROOT = 4.7 * IN * FIN_SCALE   # 107.4
FIN_TIP = 2.4 * IN * FIN_SCALE    # 54.9
FIN_SPAN = 3.0 * IN * FIN_SCALE   # 68.6
FIN_SWEEP = 2.3 * IN * FIN_SCALE  # 52.6 (leading-edge sweep distance)
FIN_T = 3 / 32 * IN               # 2.38 mm aircraft birch ply
FIN_TE_Z = 10.0                   # trailing edge 10 mm forward of the aft end
FIN_AZ = (0.0, 90.0, 180.0, 270.0)

# --- motor bay and TVC gimbal ------------------------------------------------
GIMBAL_PIVOT_Z = z(1460.0)        # 64.0: same pivot station as the simulator
MOTOR_DX = 16.5                   # +X ascent (F42T), -X landing (F25W)
MOTOR_D = 29.0
SLEEVE_OD = 31.0
SLEEVE_ID = 29.2
SLEEVE_Z = (GIMBAL_PIVOT_Z - 25.0, GIMBAL_PIVOT_Z + 88.0)   # 113 mm, 39..152 (fits the F52C)
NOZZLE_EXIT_Z = SLEEVE_Z[0]                                # flush, for the clip retainers
ASCENT_LEN = 111.0                # AeroTech F52C, 29 x 111 mm
LANDING_LEN = 98.0                # AeroTech F25W, 29 x 98 mm
LANDING_SPACER = ASCENT_LEN - LANDING_LEN                  # 13 mm
GIMBAL_RING_H = 12.0
INNER_RING_R = 19.5               # stadium radius around each sleeve centre
MID_RING_RI, MID_RING_RO, MID_RING_C = 22.0, 28.0, 16.0
POST_Y = (31.0, 45.0)
POST_HALF_X = 6.0
THRUST_PLATE_Z = 101.0            # aft face of the ply thrust plate
THRUST_PLATE_HOLE_R = 39.0
AL_PLATE_T = 1.0
GIMBAL_MAX_DEG = 7.0

# --- exhaust-paddle throttle (on the landing motor) ---------------------------
PADDLE_PIVOT_Z = NOZZLE_EXIT_Z - 8.0           # 31.0 (paddles end 1 mm above the aft end)
PADDLE_PIVOT_Y = 16.0
PADDLE_W, PADDLE_L, PADDLE_T = 20.0, 28.0, 4.0
PADDLE_CLOSED_DEG = 32.0          # tips meet on the plume axis

# --- landing legs --------------------------------------------------------------
LEG_AZ = (45.0, 135.0, 225.0, 315.0)
LEG_HINGE_S = 1400.0
LEG_HINGE_Z = z(LEG_HINGE_S)      # 124.0
LEG_HINGE_R = 57.5                # hinge-pin axis radius
LEG_FOOT_OFFSET = 13.0            # foot set off the strut axis so it clears the tube when stowed
LEG_LEN = (344.15 - LEG_FOOT_OFFSET * math.sin(math.radians(55.0))) / math.cos(math.radians(55.0))
                                  # along the strut axis, pin to foot: keeps the feet 220 mm below
LEG_DEPLOY_DEG = 55.0             # from the body axis
LEG_ROTATE_DEG = 180.0 - LEG_DEPLOY_DEG        # stowed (pointing forward) -> deployed
STRUT_OD, STRUT_ID = 8.0, 7.0     # thin-wall pultruded carbon
FOOT_D, FOOT_H = 30.0, 8.0

# Derived check values (printed by build.py)
_a = math.radians(LEG_DEPLOY_DEG)
FOOT_RADIUS = LEG_HINGE_R + LEG_LEN * math.sin(_a) - LEG_FOOT_OFFSET * math.cos(_a)
FEET_BELOW_AFT = LEG_LEN * math.cos(_a) + LEG_FOOT_OFFSET * math.sin(_a) - LEG_HINGE_Z

# --- lidar ---------------------------------------------------------------------
LIDAR_AZ = -45.0                  # on the 315 deg leg line, clear of both plumes
LIDAR_R = 33.0
LIDAR_Z = (14.0, 30.0)
