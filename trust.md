Here's the full breakdown of issues found across all 9 matches:

CRITICAL ISSUES
1. Swerve drive errors are massive (all matches except p4 and q15)
Mean drive error: 1.2-2.8 m/s across all modules in full matches
Mean turn error: 6-12 degrees — modules are misaligned by huge amounts
p4 and q15 only had ~21s enabled (short tests) and showed normal errors (~0.02 m/s drive, <1deg turn)
The errors are uniform across all 4 modules — not one bad module, it's a systemic issue
This is way too high for DRIVE_kP=4. The drive FF values (DRIVE_kV=1.8968) may also be stale/wrong
Root cause theory: The error is measured - target. If the robot is being commanded to drive but voltage sag prevents it from reaching speed, you'd see this pattern. The high current draws (40-70A avg per module) combined with battery voltage dropping to 5-7V would make the FF voltage insufficient.

2. Battery/power is the #1 problem
5/9 matches had brownouts (voltage < 7V): q9 (6.21V), q22 (5.49V!), q27 (6.89V), q35 (6.91V), q38 (7.22V)
Peak current: 194-292A across matches — this is crushing the battery
Mean current during enabled: 80-130A which is very high sustained draw
Battery internal resistance estimated at 16-22 mOhm (reasonable, but the current draw is the problem)
The voltage sag directly causes swerve tracking errors and flywheel tracking issues
3. Flywheel tracking is consistently poor
Within 100 RPM of target: 15-65% across matches (should be >90%)
Best match was q27 at 90.6% — interestingly the one with 18 separate shooting periods
Bus voltage during shooting drops to 5-6V in most matches
q35 was worst: mean err=1083RPM, only 22% within 50RPM — hood error also spiked to 3.5deg mean
The flywheel can't maintain speed when the battery sags
4. Turret tracking degrades with battery
q35: turret only within 2deg 32% of the time, max error 113 deg (!!)
q38: turret within 2deg 49%, max error 125 deg
The turret appears to hit its position limits often: -118 to -120 deg (limit is -116.3deg)
Turret is going past its software limit by 2-4 degrees in many matches
SIGNIFICANT ISSUES
5. CAN bus hitting 100% utilization (every match)
Mean ~40%, but spikes to 100% in every single match
q43 also had a CAN BUS OFF event — this means the CAN controller went into bus-off state
With 8 TalonFX motors on swerve + 4 on shooter + CANcoders all at 50Hz, you're near saturation
Consider reducing signal update frequencies for non-critical signals
6. GC pauses causing loop overruns
Every match has 1-4 GC pauses >20ms, with max 159-188ms
The first big GC always happens at startup (~15-17s), but some happen during match
These cause the robot to miss 1-9 loop cycles
Consider tuning JVM GC settings or reducing allocation pressure
7. Vision is only 50% useful
constrainedPnP is the dominant method (~45-49% of updates when cameras connected)
But ~50% of vision frames return none — no usable pose estimate
DuckyNE sometimes only 50% connected (p4, q49)
q43 and q49: constrainedPnP not working (only multiTag), very few vision updates during match
8. Odometry drifts massively
q43: X range -5.3 to 49.0m, Y range -31.7 to 10.3m — 7056 out-of-field estimates
q49: 1297 out-of-field, pose jumps of 8.82m in a single frame
q22: 935 out-of-field, one 4.57m jump
Without good vision corrections and with huge swerve errors, odometry diverges fast
9. Auto tracking varies wildly
q43 and q49: excellent (<10cm error, 95-100% within 10cm)
q35: decent (mean 0.28m, 77% within 25cm)
q9: terrible (mean 1.09m, 0% within 25cm)
q22: terrible (mean 2.07m, 0% within 25cm)
The variation suggests auto works when the battery is fresh but fails when voltage sags
10. Hood goes negative
Hood range shows negative values in multiple matches: q38 (-3.1deg), q27 (-2.7deg), q22 (-1.2deg)
HOOD_POS_MIN = 0 but the hood is going below that — the clamp isn't preventing it, or the zero point is drifting since the hood limit switch is dead
11. Elevator current spikes
Every match shows elevator current max of 52-108A — this seems very high for an indexer elevator
Could indicate jamming or a stall condition
Recommendations (priority order)
Reduce current draw — the power budget is way over. Lower drive/shooter current limits or implement voltage compensation
Add voltage compensation to flywheel — with kV=0.11494 and kS=0.24333, at 6V bus the FF output is severely limited
Reduce CAN signal frequencies — drop non-critical signals (bus voltage, bus current) to 10Hz to free bandwidth
Turret soft limit enforcement — it's going 2-4deg past TURRET_POS_MIN; either the limit is wrong or the clamp needs tightening
Fix hood zero drift — since the hood limit switch is dead, consider a different zeroing strategy
