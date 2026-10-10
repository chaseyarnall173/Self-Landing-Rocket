# 2. Regulations, certification and safety

> Not legal advice. Rules change; check the primary sources linked here
> before you fly. Several items below could only be checked against search
> summaries of the official pages (marked *verify*).

## 2.1 Bottom line

| Question | Answer for SLR-1 as designed |
|---|---|
| FAA class | **Class 1 - Model Rocket**, if liftoff mass stays <= 1500 g and total propellant <= 125 g ([14 CFR 101.22](https://www.ecfr.gov/current/title-14/chapter-I/subchapter-F/part-101/subpart-C/section-101.22)) |
| FAA notification / waiver | **Not required** for Class 1. Only the general rules of §101.23 apply: no hazard to people, property or aircraft. |
| Certification needed? | **No.** The F42T and F25W are model-rocket motors: each under 62.5 g of propellant and under 80 N·s. |
| Age | Both motors are within the CPSC consumer exemption (≤ 62.5 g propellant, < 80 N·s). Some vendors still sell composite motors 18+ only; *verify with your vendor*. |
| Where | At a **club launch** (strongly recommended) with RSO approval. Total impulse 127 N·s falls in the NAR "G" row: **minimum site dimension 1000 ft**. |
| Biggest approval risk | The recovery requirement. Carry the **automatic backup parachute** and brief the RSO before you arrive. |

If the build ends up **over 1500 g** it becomes a **high-power** rocket. That
means:
- Level 1 certification (NAR or Tripoli)
- the NAR High Power Safety Code
- launching under a club FAA waiver
- liftoff weight <= 1/3 of average thrust

The F42T gives ~40 N average, so a high-power build would need a bigger
ascent motor.

## 2.2 FAA - 14 CFR Part 101, Subpart C

**§101.22(a) Class 1 - Model Rocket.** An amateur rocket that:

1. "Uses no more than 125 grams (4.4 ounces) of propellant;"
2. "Uses a slow-burning propellant;"
3. "Is made of paper, wood, or breakable plastic;"
4. "Contains no substantial metal parts;" and
5. "Weighs no more than 1,500 grams (53 ounces), including the propellant."

([eCFR](https://www.ecfr.gov/current/title-14/chapter-I/subchapter-F/part-101/subpart-C/section-101.22))

Two points matter for this design:

- **Total propellant counts both motors.** The budget is 27.0 + 38.8 = 65.8 g.
- **"No substantial metal parts."** The gimbal pivots, the paddle arms and the
  servos are small internal parts. Keep the airframe itself paper, wood and
  plastic, and discuss the metal gimbal parts with your RSO. This is a
  judgement call the rule does not define.

**§101.25 / §101.27** apply only to Class 2 and above: cloud cover, visibility,
5 nmi from airports, notifying ATC 24-72 h ahead
([§101.25](https://www.ecfr.gov/current/title-14/chapter-I/subchapter-F/part-101/subpart-C/section-101.25),
[§101.27](https://federal-regs.com/title/14/part-101/101.27/)). They don't apply
while you stay Class 1, but follow the NAR code's equivalents anyway.

**Boston-area note.** Much of eastern Massachusetts sits under Logan's Class B
airspace. Fly at an established club field, not a local park.

## 2.3 NAR Model Rocket Safety Code (the rules your club will enforce)

From the [NAR Model Rocket Safety Code](https://www.nar.org/ModelRocketSafetyCode):

- **Size:** <= 1500 g at liftoff, <= 125 g propellant, <= 320 N·s total impulse.
- **Motors:** certified, commercially made motors only, never modified, used
  per the manufacturer's instructions. That covers removing or keeping the
  ejection charge (see [docs/04](04-propulsion.md#46-ejection-charges-and-delays)).
- **Ignition:** "an electrical launch system and electrical motor igniters ...
  a safety interlock in series with the launch switch, and ... a launch switch
  that returns to the 'off' position when released."
- **Misfires:** remove the interlock or disconnect the battery and **wait 60 s**
  before approaching.
- **Launch safety:** spectators at least **30 ft** away for E motors and above.
  Expect the RSO to push you to a "heads-up" or far pad for an experimental
  flight.
- **Recovery:** "I will use a recovery system such as a streamer or parachute
  ... so that it returns safely and undamaged and can be flown again." The
  propulsive landing is the recovery method. It uses no parachute in a normal
  flight, so the **automatic backup main chute is what satisfies this rule**.
  It deploys only if the landing motor fails to light or the flight leaves
  its safe envelope. Expect the RSO to treat the first flights as
  experimental.
- **Launcher:** rod, tower or rail within 30° of vertical. SLR-1 lifts off from
  a **flat pad with a short guide**. It is actively stabilised from t = 0 and
  is moving ~11 m/s after 1.5 m, below the usual 15 m/s rail-exit guideline. **Agree on
  this with the RSO in advance.** The fallback is a 1010 rail with the gimbal
  locked for the first 1.5 m.

**Launch-site table** (minimum site dimension):

| Total impulse | Class | Site |
|---|---|---|
| 40-80 N·s | F | 1000 ft |
| **80-160 N·s** | **G (SLR-1: 127 N·s)** | **1000 ft** |
| 160-320 N·s | two Gs (rev A design: 198 N·s) | 1500 ft |

([table summary](https://www.rocketryforum.com/threads/n-a-r-rules-question-launch-site-dimensions.84769/))

Since the 2017 edition, NFPA 1122 also allows site size to be computed from a
simulation ([NFPA 1122 2026](https://webstore.ansi.org/standards/nfpa/nfpa11222026)).

### Onboard ignition of the landing motor

No rule found in the NAR model code or NFPA 1122 bans a flight computer from
igniting a motor in flight (*verify against the 2026 NFPA text*). The closest
written rules are **Tripoli's airstart rules**, merged in the Unified Safety
Code v2 (effective January 2026,
[tripoli.org/safetycode](https://www.tripoli.org/safetycode)):

- onboard ignition devices are armed only with the rocket in launch position
  and before the launch igniter is connected;
- the airstart design must resist accidental triggering; and
- the RSO approves the airstart before launch.

SLR-1 meets that standard. See [docs/06](06-gnc.md#67-safety-interlocks):

- physical arm key in series with the pyro battery
- arming only when vertical on the pad with continuity on both igniters
- in-flight envelope checks before the FIRE command (tilt < 20°, altitude >
  5 m, descent between 8 and 35 m/s)
- a single igniter pulse per boot

## 2.4 Certification, purchasing, storage

| Topic | Rule |
|---|---|
| Motors for SLR-1 | No certification needed (model-rocket motors). |
| Level 1 (if you go high-power) | Join NAR or Tripoli, fly an H or I motor at a sanctioned launch with a certified witness. NAR juniors aged 14-17 have a Junior HPR program. ([NAR HPR cert](https://www.nar.org/HPRCertification)) |
| CPSC | 16 CFR 1500.85(a)(8) exempts devices with <= 62.5 g propellant **and < 80 N·s**, so G motors are outside the consumer exemption and are sold 18+ ([eCFR via Cornell](https://www.law.cornell.edu/cfr/text/16/1500.85)) |
| ATF | Motors with <= 62.5 g propellant are exempt (27 CFR 555.141(a)(10)). APCP has not been federally regulated as an explosive since the 2009 *Tripoli v. ATF* decision ([2010 notice](https://regulations.justia.com/regulations/fedreg/2010/01/08/2010-45.html)). |
| E-matches | Most are a grey area under the ATF LEUP rules. **Avoid the question:** use non-regulated motor igniters (e.g. AeroTech FirstFire / Estes-style starters, or ATF-approved non-regulated initiators). Use **servo/spring** mechanisms, not black-powder charges, for the airbrakes and chute. |
| Shipping | Composite motors ship ground-only as hazmat 1.4C; expect a ~$30-50 hazmat fee (*verify*). |

## 2.5 Massachusetts (Northeastern / Boston)

- **Statute.** MGL c.148 §9A has the Board of Fire Prevention regulate model
  rocketry under the NFPA standards. It defines a model rocket as one that
  "includes a device for returning it to the ground in a condition to fly
  again" ([MGL c.148 §9A](https://malegislature.gov/Laws/GeneralLaws/PartI/TitleXX/Chapter148/Section9A)).
  That is another reason to keep the backup parachute.
- **Regulation.** 527 CMR 1.00 Chapter 65 adopts NFPA 1122 / 1127
  ([527 CMR 1.00](https://www.mass.gov/doc/massachusetts-527-cmr-100-2021-edition-effective-may-12-2023/download)).
  - A fire-department permit is required to store **more than 50 lb** of model
    motors at a residence; you will have well under that.
  - No state launch permit or "certificate of competency" was found for model
    rockets (*verify with your local fire department*).
- **Where to fly:** CMASS (NAR Section #464,
  [nar.org listing](https://www.nar.org/local_club/central-massachusetts-spacemodeling-society-cmass-464/)).
  - **Woodsom Farm, Amesbury** is large, high-power capable, with FAA waivers to
    4000 ft. Easily covers the 1000 ft requirement.
  - **Acton** (School Street fields) allows up to G motors. Check that the
    field gives 1000 ft and that the club will host an experimental
    propulsive-landing flight.
  - Non-members fly for about $5 a day.
- **Campus:** check Northeastern Environmental Health & Safety rules before
  storing motors or doing static fires on or near campus.
- **Prior art nearby:** the UConn AIAA Propulsive Landing team
  ([aiaa.engr.rso.uconn.edu/pl](https://aiaa.engr.rso.uconn.edu/pl)).
- **Export control:** TVC guidance software and hardware can raise ITAR/EAR
  questions. If you publish the design or work with foreign nationals, ask
  Northeastern's export-control office.

## 2.6 Safety rules specific to this project

1. **Static-fire before you fly.** Every landing-motor lot gets static-fired on
   the load-cell stand with the *flight* igniter, to measure ignition delay and
   paddle authority. See [docs/09](09-test-plan.md).
2. **Never connect an igniter indoors or on the bench.** Use bench mode with
   dummy loads (resistors), never real igniters.
3. **Arm only at the pad, vertical, with the area clear.** Disarm (pull the
   key) before lowering or carrying the rocket.
4. **Late ignition under the chute ("hang fire")** is possible. The software
   then holds the nozzle pointing down. Spectators stay upwind and outside the
   descent cone.
5. **An unfired landing motor after an abort is a live motor.**
   - Approach only after 60 s, remove the arm key, then the igniters.
   - Treat it as a misfire.
6. **LiPo safety:**
   - charge on a balance charger in a fire bag, attended;
   - store at storage voltage;
   - retire any swollen pack;
   - check voltage at preflight (the software won't arm below 7.4 V).
7. **Fire safety:**
   - clear dry grass around the pad;
   - have water and an ABC extinguisher at the pad and at the static-fire stand;
   - the paddles and gimbal will be hot (APCP exhaust is ~2000 °C+). Wait
     before handling.
8. **Wind limit for landing attempts: 3 m/s (~6.7 mph) at the surface.** In
   the Monte Carlo, success is 97-100% below 3 m/s and 78% at 3-4 m/s.
