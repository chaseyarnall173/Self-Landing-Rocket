# SLR-1 CAD (rev C)

Parametric 3D model of the whole rocket in [CadQuery](https://github.com/CadQuery/cadquery)
(Python on the OpenCascade kernel). See [docs/13](../docs/13-cad.md) for the full write-up.

```bash
pip install cadquery          # tested with 2.8
python cad/build.py           # writes cad/out/ (~3 min)
```

| Path | Contents |
|---|---|
| `slr1/dims.py` | Master dimensions (mm); stations from the nose tip, CAD z = 1524 − station |
| `slr1/airframe.py` | Nose, tube pieces, couplers, fins, airbrake petals + mechanism, chute door |
| `slr1/bays.py` | Backup-chute bay, avionics bay, wiring runs |
| `slr1/propulsion.py` | Motors, sleeves, gimbal, thrust plate, servos, exhaust paddles, lidar |
| `slr1/legs.py` | Landing legs (one modelled, four placed), release loop |
| `slr1/catalog.py` | Part / instance records, materials and densities, mass-budget lines |
| `slr1/massprops.py` | Mass, CG and inertia roll-up |
| `slr1/model.py` | Collects every part; flight (stowed) and landing (deployed) configurations |
| `build.py` | Exports, BOM, mass report, interference check, gimbal sweep |
| `viewer/index.html` | Interactive three.js viewer (loads `out/viewer/*`) |
| `tools/render_views.mjs` | Renders the still images in `docs/img/cad_*.png` |
| `out/` | Generated: `SLR1_flight.step`, `SLR1_landing.step`, `parts/`, `stl/`, `dxf/`, `viewer/`, `bom.csv`, reports |

**SolidWorks:** File → Open → STEP, pick `out/SLR1_flight.step`. It imports as
an assembly with one sub-assembly per bay and parts named by part number.

To view the 3D viewer locally, serve the folder: `cd cad/out/viewer && python -m http.server`,
then open http://localhost:8000.
