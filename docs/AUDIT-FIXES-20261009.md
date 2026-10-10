# PMIV audit fixes

- Require matching PMAero 1.0 wind revision 3 and consume its vector-only buffer API.
- Retain detailed wind metadata for HUD/diagnostic consumers.
- Propagate contact normals into support, tire axes, slip feedback, impact effective mass,
  terrain response and gravity compensation.
- Use authored wheel radius for rounded voxel-edge support points. Other rigid supports
  and fluids retain their top-face/vertical support semantics.
- Keep pack grip, drive, steering, braking and rotor assistance values.
- Stop body-friction iterations after convergence; evaluate symmetric tire matrix pairs once.
- Add private terrain-probe and gear-solver timings.
- Correct attribution: upstream True Impact 0.5.7-delta, local derivative 0.5.8-delta.
- Preserve logger session reset, bounded chunks and recorder compression.

The rounded wheel probe is an approximation, not a full cylindrical tire model.
New wind/contact behavior and mixin application need live traces. This candidate
was compiled and packaged without running regression suites or Minecraft.

The rounded edge proxy is anchored to IV\'s existing lowest tire support point, which
already includes axle, tilt and width. Flat top-face support height is preserved.
