# 12. References

Research for this design was done in October 2026 from web sources. Facts
marked *verify* in the other docs came from search summaries of these pages.
Re-check rules and prices before relying on them.

## Prior art (hobby propulsive landing)

- BPS.space (Joe Barnard), first successful Scout F landing, 2022: [Hackaday](https://hackaday.com/2022/08/05/bps-space-succesfully-lands-a-model-rocket/); ["more slam than hover"](https://twitter.com/bps_space/status/1553103061135945728); [Space Explored](https://spaceexplored.com/2022/08/01/bps-model-rocket-landing-video/)
- BPS ceramic exhaust "pincers", up to 80% thrust cut: [The New Stack](https://thenewstack.io/one-mans-dream-to-build-a-working-spacex-style-model-rocket/); machined aluminium mount: [scifi.radio](https://scifi.radio/2022/08/07/meet-the-diy-rocketeer-who-can-copy-spacex/)
- BPS Scout E (2020), tipped over from horizontal velocity: [Hackaday](https://hackaday.com/2020/11/27/so-close-to-landing-a-model-rocket-on-its-tail/)
- BPS community wiki, IMU selection and gimbal design: [imu-selection](https://github.com/bps-space/bps-space.github.io/blob/master/electronics/imu-selection.md), [gimbal-design](https://github.com/bps-space/bps-space.github.io/blob/master/mechanical/gimbal-design.md); BPS TVC mount: [bps.space](https://bps.space/products/thrust-vector-control)
- Aryan Kapoor (JRD Propulsion), solid-motor landing on the 5th attempt, 2024: [Hackaday](https://hackaday.com/2024/07/10/model-rocket-nails-vertical-landing-after-three-year-effort/), [PopSci](https://www.popsci.com/science/model-rocket-landing/)
- Sparrow TVC hopper (gas shocks, ~900 g): [Hack Club](https://hardware.hackclub.com/projects/waaaaaaaaah/sparrow-tvc-hopper)
- EDF TVC self-landing rocket: [UrosKukovic](https://github.com/UrosKukovic/TVC_Self_Landing_Rocket); Starkit: [Hack Club](https://hardware.hackclub.com/projects/mikuel210/starkit)
- Parachute-then-propulsive pattern (RP3, Monte Carlo 70%): [EthanSheehan](https://github.com/EthanSheehan/EthanSheehan-RP3-Public)
- Open-source TVC code:
  - [tomkuttler/TVC-Rocket-Flight-Code](https://github.com/tomkuttler/TVC-Rocket-Flight-Code)
  - [xpbowler/tvc-rocket](https://github.com/xpbowler/tvc-rocket)
  - [Zentrik/TVC](https://github.com/Zentrik/TVC)
  - [AeroVECTOR](https://github.com/GuidodiPasquo/AeroVECTOR)
  - [MathWorks TVC Simulink](https://github.com/mathworks/Modeling-TVCd-Rocket-in-Simulink)
- UConn AIAA Propulsive Landing team: [aiaa.engr.rso.uconn.edu/pl](https://aiaa.engr.rso.uconn.edu/pl)
- 3D-printed 2-axis TVC gimbal study: [arXiv:2509.00061](https://arxiv.org/abs/2509.00061)
- Landing-leg kit (rubber band + nichrome release): [K-9 on Printables](https://www.printables.com/model/164898-k-9-model-rocket-landing-leg-system-v6)

## Motors and propulsion

- ThrustCurve.org motor database and RASP files: [thrustcurve.org](https://www.thrustcurve.org) - [F42T](https://www.thrustcurve.org/motors/AeroTech/F42T), [F25W](https://www.thrustcurve.org/motors/AeroTech/F25W), [F52C](https://www.thrustcurve.org/motors/AeroTech/F52C), [G25W](https://www.thrustcurve.org/motors/AeroTech/G25W), [G40W](https://www.thrustcurve.org/motors/AeroTech/G40W), [F15](https://www.thrustcurve.org/motors/Estes/F15)
- AeroTech G40W: [aerotech-rocketry.com](https://aerotech-rocketry.com/products/product_e54bba8f-a733-e5ba-b4f6-3db6a2d05b19); G80T: [aerotech-rocketry.com](https://aerotech-rocketry.com/products/product_12ac810d-0472-e322-6e54-c5d35b4eb0e5)
- Estes F15: [NAR certification sheet](https://www.nar.org/wp-content/uploads/2014/08/F15.pdf), [Estes](https://estesrockets.com/products/f15-4-engines)
- NFPA 1125 motor tolerances (forum summary): [Rocketry Forum](https://www.rocketryforum.com/threads/bp-motors-whats-the-variation-of-burn-time-per-nfpa-1125.176287/latest)
- Igniters: [MJG Firewire](https://www.erockets.biz/mjg-technology-36-starters-initiators-dipped-2pk-mjg-03), [J-Tek data sheet](https://electricmatch.com/upload/JTEK_TECHNICAL_INFORMATION_SHEET_SPECIFICATIONS.PDF), [Estes starter](https://www.erockets.biz/estes-model-rocket-star-tech-starters-igniters-for-flying-model-rockets-est-2303/); pyro current: [Altus Metrum](https://altusmetrum.org/FAQs/pyro_current.html)
- EDF units: [Freewing 80 mm](https://motionrc.com/products/freewing-80mm-12-blade-edf-6s-power-system-w-3658-2150kv-inrunner-motor-e72314), [JP Hobby 90 mm](https://motionrc.com/collections/edf-power-systems/products/jp-hobby-90mm-12-blade-edf-6s-power-system-w-1750kv-jph6007-102)
- Throttled hybrids: [Utah State 67:1 throttling](https://digitalcommons.usu.edu/mae_facpub/34); tethered hybrid hover: [Acta Astronautica](https://www.sciencedirect.com/science/article/abs/pii/S009457652100463X)
- JetCat P20-SX: [jetcat.de](https://www.jetcat.de/en/productdetails/produkte/jetcat/produkte/RC%20ENGINES/Engines/p20_sx)

## Avionics

- [Teensy 4.1](https://www.pjrc.com/store/teensy41.html) · [BMI088](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/) · [BMP390](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/pressure-sensors-bmp390.html) · [TFmini-S](https://en.benewake.com/TFminiS/) · [TF02-Pro data sheet](https://en.benewake.com/uploadfiles/2024/04/20240426135509321.pdf) · [VL53L1X (indoor only)](https://www.pololu.com/product/3415) · [NEO-M9N](https://www.sparkfun.com/sparkfun-gps-breakout-neo-m9n-u-fl-qwiic.html) · [RFM95W](https://hoperf.com/modules/lora/RFM95W.html) · [KST X08H](https://servodatabase.com/servo/kst/x08h) · [MG90S](https://servodatabase.com/servo/turnigy/mg90s)
- MPU6050 end of life: [Rapid](https://www.rapidonline.com/news/mpu6050-alternatives-equivalents)
- Pyro-channel design discussion: [Rocketry Forum](https://www.rocketryforum.com/threads/pyro-channel-for-custom-flight-computer-inquiry.166899/)
- Load-cell rates (HX711 too slow): [Arduino blog](https://blog.arduino.cc/2018/12/17/an-arduino-based-rocket-motor-thrust-test-stand/)
- PETG vs PLA thermal: [MakeItFrom](https://www.makeitfrom.com/compare/Glycol-Modified-Polyethylene-Terephthalate-PETG-PET-G/Polylactic-Acid-PLA-Polylactide)

## Airframe and aerodynamics

- LOC Precision [tubes](https://locprecision.com/products/cardboard-airframes), [nose cones](https://locprecision.com/product/nose-cones/), [rings](https://locprecision.com/products/300rings)
- Tube mass comparison: [Fab Academy](https://fabacademy.org/2025/labs/techworks/students/omar-albalbaki/Body%20Tubes%201c70e8ca7ada81a6b85ddbe39d7d69ba.html)
- Barrowman method: [Nakka Appendix B](https://www.nakka-rocketry.net/RD_Appendix_B.html), [Rocketmime](https://www.rocketmime.com/rockets/Barrowman.html), [OpenRocket technical documentation](https://openrocket.sourceforge.net/techdoc.pdf)
- Axial drag of flat-faced cylinders: [Aerospaceweb](https://aerospaceweb.org/question/aerodynamics/q0231.shtml)
- OpenRocket simulation extensions: [docs](https://openrocket.readthedocs.io/en/latest/user_guide/simulation_extensions.html); RocketPy: [PyPI](https://pypi.org/project/rocketpy/)

## Guidance, navigation and control (scholarly)

DOIs are listed for lookup through your library.

- B. Açıkmeşe, S. R. Ploen, "Convex Programming Approach to Powered Descent Guidance for Mars Landing," *J. Guidance, Control, and Dynamics* 30(5), 2007. [doi:10.2514/1.27553](https://doi.org/10.2514/1.27553)
- L. Blackmore, B. Açıkmeşe, D. P. Scharf, "Minimum-Landing-Error Powered-Descent Guidance for Mars Landing Using Convex Optimization," *JGCD* 33(4), 2010. [doi:10.2514/1.47202](https://doi.org/10.2514/1.47202)
- D. P. Scharf et al., "Implementation and Experimental Demonstration of Onboard Powered-Descent Guidance," *JGCD* 40(2), 2017. [doi:10.2514/1.G000399](https://doi.org/10.2514/1.G000399)
- D. Dueri et al., "Customized Real-Time Interior-Point Methods for Onboard Powered-Descent Guidance," *JGCD*, 2017. [doi:10.2514/1.G001480](https://doi.org/10.2514/1.G001480)
- L. Blackmore, "Autonomous Precision Landing of Space Rockets," *The Bridge / NAE Frontiers of Engineering*, 2016. [doi:10.17226/23659](https://doi.org/10.17226/23659)
- D. Malyuta et al., "Convex Optimization for Trajectory Generation," review, 2021. [arXiv:2108.02335](https://arxiv.org/abs/2108.02335)
- S. O. H. Madgwick et al., "Estimation of IMU and MARG orientation using a gradient descent algorithm," IEEE ICORR, 2011. [doi:10.1109/ICORR.2011.5975346](https://doi.org/10.1109/ICORR.2011.5975346)
- R. Mahony, T. Hamel, J.-M. Pflimlin, "Nonlinear Complementary Filters on the Special Orthogonal Group," *IEEE Trans. Automatic Control* 53(5), 2008. [doi:10.1109/TAC.2008.923738](https://doi.org/10.1109/TAC.2008.923738)
- G. Ceotto et al., "RocketPy: Six Degree-of-Freedom Rocket Trajectory Simulator," *J. Aerospace Engineering* 34(6), 2021. [doi:10.1061/(ASCE)AS.1943-5525.0001331](https://doi.org/10.1061/(ASCE)AS.1943-5525.0001331)
- Thrust-vector controller comparison for a finless rocket: [ResearchGate](https://www.researchgate.net/publication/369325159_Thrust_Vector_Controller_Comparison_for_a_Finless_Rocket)
- Launcher TVC with gain scheduling: [arXiv:2303.16983](https://arxiv.org/abs/2303.16983)

## Regulations and safety

- FAA 14 CFR Part 101: [§101.22](https://www.ecfr.gov/current/title-14/chapter-I/subchapter-F/part-101/subpart-C/section-101.22), [§101.25](https://www.ecfr.gov/current/title-14/chapter-I/subchapter-F/part-101/subpart-C/section-101.25), [§101.27](https://federal-regs.com/title/14/part-101/101.27/); 2008 final rule: [Federal Register](https://www.federalregister.gov/documents/2008/12/04/E8-28703/requirements-for-amateur-rocket-activities)
- NAR [Model Rocket Safety Code](https://www.nar.org/ModelRocketSafetyCode), [High Power Safety Code](https://nar.org/HPRSafetyCode), [HPR certification](https://www.nar.org/HPRCertification)
- Tripoli Unified Safety Code: [tripoli.org/safetycode](https://www.tripoli.org/safetycode)
- NFPA 1122 (2026): [ANSI webstore](https://webstore.ansi.org/standards/nfpa/nfpa11222026)
- CPSC 16 CFR 1500.85: [Cornell LII](https://www.law.cornell.edu/cfr/text/16/1500.85); ATF APCP notice: [Federal Register 2010](https://regulations.justia.com/regulations/fedreg/2010/01/08/2010-45.html); ATF exemption rule: [govinfo](https://www.govinfo.gov/content/pkg/FR-2006-08-11/html/06-6862.htm)
- Massachusetts: [MGL c.148 §9A](https://malegislature.gov/Laws/GeneralLaws/PartI/TitleXX/Chapter148/Section9A), [527 CMR 1.00](https://www.mass.gov/doc/massachusetts-527-cmr-100-2021-edition-effective-may-12-2023/download)
- CMASS (NAR #464): [nar.org](https://www.nar.org/local_club/central-massachusetts-spacemodeling-society-cmass-464/)

## CAD and visualisation

- CadQuery (parametric CAD in Python): [github.com/CadQuery/cadquery](https://github.com/CadQuery/cadquery), [documentation](https://cadquery.readthedocs.io)
- Open CASCADE Technology (the geometry kernel under CadQuery): [dev.opencascade.org](https://dev.opencascade.org)
- STEP AP214 / glTF 2.0 formats: [ISO 10303-214](https://www.iso.org/standard/43669.html), [Khronos glTF](https://www.khronos.org/gltf/)
- three.js (3D viewer): [threejs.org](https://threejs.org)
- LOC Precision 3.9 in airframe (3.9 in ID x 4.0 in OD, 0.05 in wall) and couplers: [cardboard airframes](https://locprecision.com/products/cardboard-airframes), [tube coupler](https://locprecision.com/products/tube-coupler)
