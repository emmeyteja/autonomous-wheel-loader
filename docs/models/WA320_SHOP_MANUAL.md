# WA320-7 shop-manual reference

- Source: Komatsu **SEN06202-03**, serial numbers **80001 and up**. PDF pages below are one-based; printed section numbers are the primary locator. The source hash and extracted reference values are in [the measurement ledger](../../configs/measurements/wa320_7.yaml).
- These are manufacturer-published specifications, not measurements of our selected machine. The PDF and its artwork stay outside the public repository.

## Cylinder dimensions

Printed 01-11 / 01-16; PDF 81 / 86. Dimensions in millimetres:

- Steering: bore 70, rod 40, stroke 453; pin distance 818–1271.
- Lift: bore 120, rod 75, stroke 729; pin distance 1327–2056.
- Bucket: bore 150, rod 110, stroke 558; pin distance 1354–1912.
- Each extended-minus-retracted pin distance equals the stated stroke. These supply areas and length envelopes; they do not identify the mounting coordinates or full PZ linkage.

## Configuration differences

- Non-EU specification, 01-7 / 01-8 (PDF 77–78): 15,220 kg, 2.8 m³ bucket, 2740 mm bucket width with bolt-on cutting edge (BOC).
- EU specification, 01-12 / 01-13 (PDF 82–83): 15,445 kg, 2.9 m³ bucket, 2750 mm width with BOC.
- Our selected paper/brochure model remains 15,175 kg, 3.0 m³ and 2685 mm. Do not mix bucket profiles, dump dimensions or regional specifications. Confirm applicability before promoting cylinder or mass data.
- Component handling weights, 01-17 through 01-20 (PDF 87–90), include boom 1313 kg, bellcrank 300 kg and tire/rim 341 kg each. Bare-frame weights cannot replace lumped frame bodies directly. Centres of mass and inertia tensors are still unidentified.

## Hydraulic and sensing evidence

- 01-10 (PDF 80): work-equipment valve setting 30.4 MPa, steering setting 20.6 MPa, pump displacement 90 cm³/rev. These are not constant operating pressures or actual flow.
- 20-13 (PDF 427): work-equipment relief acceptance 30.4 ±1.57 MPa for a new machine at high idle, oil 45–55 °C. The stated monitor error is approximately ±0.5 MPa; it is not a raw sensor calibration or simulation-error bound.
- The same page specifies unloaded boom raising 5.6 ±0.5 s under its test conditions. Keep this separate from the brochure's 6.3 s figure and reproduce the specified F/G posture before comparing.
- 10-315 (PDF 407): lift-bottom pressure sensor has a plotted 0–50 MPa to 0.5–4.5 V transfer curve. Dynamic response and full uncertainty are not established.
- 10-284 (PDF 376): documented inputs include lift bottom/head pressures, boom potentiometer, speed and pilot-pressure signals. Pilot pressure is not cylinder chamber pressure; switch signals are not continuous angle measurements.

## Integration limits

- Reference values are stored separately; the active vehicle and sensor availability are unchanged.
- Published dimensions reduce missing evidence for a hydraulic model. We still need matching cylinder mounts/linkage, configuration confirmation, chamber/valve dynamics, friction, power sharing and independent force/pressure traces.
- Continue using explicit unavailable pressure/stroke/payload channels until their physical models and validation exist. No real-machine interfaces are added.
