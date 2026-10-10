"""Print the itemised mass budget, CG, inertia and static margins.

    python sim/scripts/mass_budget.py
"""
import _path  # noqa: F401
from rocketsim.designs import SLR1_ITEMS, slr1

v = slr1()
print(f"{'Item':78s} {'g':>6s} {'station m':>9s}")
for it in SLR1_ITEMS:
    print(f"{it.name:78s} {it.mass_kg * 1000:6.0f} {it.station_m:9.2f}")
print(f"{'DRY TOTAL':78s} {v.dry_mass_kg * 1000:6.0f} {v.dry_cg_m:9.3f}")
for label, m in [("ascent motor", v.ascent_motor), ("landing motor", v.landing_motor)]:
    print(f"{label + ': ' + m.summary():78s}")
m0, cg0, i0 = v.mass_props(None, None)
mb, cgb, ib = v.mass_props(v.ascent_motor.burn_time, None)
ml, cgl, il = v.mass_props(v.ascent_motor.burn_time, v.landing_motor.burn_time)
print()
print(f"Liftoff mass      {m0 * 1000:7.0f} g   (model-rocket limit 1500 g -> margin {1500 - m0 * 1000:.0f} g)")
print(f"Total propellant  {v.propellant_total_g():7.1f} g   (limit 125 g)")
print(f"Total impulse     {v.ascent_motor.total_impulse + v.landing_motor.total_impulse:7.1f} N*s (limit 320 N*s)")
print(f"Liftoff T/W       {v.ascent_motor.peak_thrust / (m0 * 9.80665):.1f} peak, "
      f"{v.ascent_motor.average_thrust / (m0 * 9.80665):.1f} average")
print(f"CG at liftoff     {cg0:.3f} m from nose; pitch inertia {i0:.3f} kg m^2")
print(f"Landing mass      {ml * 1000:.0f}-{v.mass_props(v.ascent_motor.burn_time, 0)[0] * 1000:.0f} g")
print(f"Gimbal lever (pivot to CG) at landing: {v.gimbal_station_m - cgb:.3f} m")
print()
print("Static margins (linear components, calibers; + = stable):")
print(f"  nose-first, liftoff          {v.static_margin_cal(cg0, 0.0, True):+.2f}")
print(f"  nose-first, coast            {v.static_margin_cal(cgb, 0.0, True):+.2f}")
print(f"  tail-first, drogue out       {v.static_margin_cal(cgb, 1.0, False):+.2f}")
print(f"  tail-first, no drogue        {v.static_margin_cal(cgb, 0.0, False):+.2f}")
ca = v.ca_tail_first + sum(c.ca for c in v.components)
vt = (2 * mb * 9.80665 / (1.2 * ca * v.ref_area)) ** 0.5
print(f"Drogue descent terminal speed ~ {vt:.1f} m/s")
