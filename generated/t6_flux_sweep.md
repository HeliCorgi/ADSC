# T6 — small-debris flux / hypervelocity-impact sweep

Evidence for the T6 open trade: direct removal of 1-10 cm fragments is **out of scope**, and this table shows why (it is documented physics, not neglect). Regenerate with `flux_sweep`. All debris-population figures are PLACEHOLDER (MASTER-8 / ESA spatial-density class; cite at fill time). No timestamp is embedded (R6).

## Hypervelocity impact energy (10 km/s)

| quantity | value |
|---|---:|
| specific kinetic energy | 50.0 MJ/kg |
| TNT specific-energy ratio | 12.0x |
| 1 cm Al sphere mass | 1.41 g |
| 1 cm Al sphere kinetic energy | 70.7 kJ |
| 1 cm Al sphere TNT equivalent | 16.9 g |

## Collector exposure (>= 1 cm objects)

PLACEHOLDER spatial densities: average 1.2e-06 /km^3, peak 1.0e-05 /km^3. Reference collector 100 m^2.

| density | hits/yr on 100 m^2 | mean interval | area for 1%/yr removal |
|---|---:|---:|---:|
| average (1.2e-06 /km^3) | 0.0379 | 26.4 yr | 31.7 km^2 |
| peak (1.0e-05 /km^3) | 0.3156 | 3.2 yr | 3.8 km^2 |

**Reading.** A cm-class impactor carries grenade-class energy (~17 g TNT); no material catches it intact -- shields work by shattering and shedding secondary ejecta. Removing 1%/yr of the >= 1 cm population needs a **km^2-scale** collector (31.7 km^2 at average density, 3.8 km^2 in a peak band) that is itself the largest collision cross-section in the band. Hence T6: fragment removal is excluded; laser photon-pressure/ablation nudging is the research lane (reference only, no implementation).
