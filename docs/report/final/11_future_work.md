# 11. Future Work

**The optical see-through display.** The first version shows the AR view on the laptop's screen
as video see-through. The return path to a true windscreen display is designed in, not hoped for:
the renderer already hands every display an overlay scene (what to draw, and where on the road or
in the image) instead of a finished picture, so a `ProjectorSink` can draw only the overlays
through a combiner and ignore the video, with nothing upstream changed. What it needs is the
optics (a combiner or an automotive head-up projector), an eye-position estimate so road-space
items line up from the driver's viewpoint, and a brightness range that survives daylight.

**Thermal sensing.** A thermal camera would find pedestrians and animals at night and in glare,
where the visible camera is weakest and Kenyan roads are most dangerous to pedestrians. It was
discussed and deliberately deferred: it needs its own calibration to the LiDAR and its own
detection network, and the safety design gains nothing until the existing sensors are proven.

**A learned reckless-driving classifier.** The rules (tailgating, erratic speed, swerving) were
chosen because each can be explained and tested, and they have a documented limit: within a 3 s
track history a slow weave is indistinguishable from a quick lane change. A longer history is the
first fix; a learned temporal classifier, trained on recorded tracks, could follow, for warnings
only.

**Map-aware motion prediction.** The tracker predicts from physics (constant velocity, constant
turn rate, a stopping model). Using the road's geometry from the map would improve forecasts at
junctions and bends. Braking would still use measured range and closing speed only.

**Braking in bends.** The braking rule needs a target in both the car's turning and straight
paths. In a tight S-bend this makes the brake late (on the recorded drive, at a true time to
collision of 0.46–1.05 s rather than 1.8 s). The road's geometry from the map could define the
path instead; road tests decide whether it is needed.

**Hardware revision 2.** Findings from the circuit simulations, to fold into the next boards: a
resistor across the brake-light optocoupler's LED for a definite switching threshold; an inrush
limit on the pod's USB supply beyond the smaller capacitors already fitted; and the module
footprints confirmed against the parts as bought. (Two other findings were adopted on Rev 1.1
before ordering, as value changes: R63 4.7 kΩ, and C1/C2 1 µF / 4.7 µF.)

**Beyond the prototype.** Cloud services, over-the-air updates and fleet features were out of
scope. A system that brakes a car would also need a formal functional-safety process
{{cite:iso26262}} before it could leave a supervised test.
