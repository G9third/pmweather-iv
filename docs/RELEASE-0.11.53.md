# PMWeather-IV 0.11.53

## Road suspension

Managed non-aircraft road vehicles now use a bounded series tire and suspension
response for intact wheels and treads that do not already animate vertically.
The accepted tire normal impulse drives one travel value per authored wheel or
track assembly; the same travel moves its support station and native part pose.
Fake tread samples retain their distributed contact and load allocation while
sharing their master's travel. Existing drivetrain, steering, skid-steer and
friction limits stay in the shared contact solve. Aircraft, liquid supports,
flat tires and authored vertical suspension keep their existing paths.

The fallback estimates static sag from support radius and distributes vehicle
mass across installed support samples. It is a generic approximation, not a
content-pack suspension specification; steering and support geometry are read
from IV's live parts. Authored vertical support animations take precedence.
Fallback travel is disabled when the body is near rollover and does not add
terrain probes outside the existing exact/predictive contact path.

## Mixed native and fallback support correction

Review of a mixed-support 0.11.52 trace found that fallback spring stiffness
and travel mass were divided by only the fallback-support count. If two of four
active support samples use native motion, the two remaining fallback samples
could each receive half the vehicle mass while native tire contacts use a
quarter each. The 0.11.53 solver now allocates both models from the same total
installed-support count, restoring consistent mass shares across the complete
support set.

The same trace exposed an animation eligibility check that evaluated a wheel's
steering/spin axis in its already animated orientation. That could misclassify
a planar authored rotation as vertical suspension motion. Eligibility now uses
the animation owner's parent-part frame or the vehicle model frame, while the
per-owner-tick observed vertical-station change guard continues to defer to
real authored suspension travel.

## Terrain material resistance

An immediately removed solid voxel now also consumes energy to displace its
collision-shape volume, estimated from the existing material density profile.
The energy remaining after fracture is reduced by an inelastic body/material
mass ratio before it becomes residual contact speed. The existing normal
contact solver remains the sole momentum response. Ejected material momentum is
not retained by the vehicle or fragment entities, and this is not a full
deformation or fragment simulation.

## HUD and limits

The corner badge is reduced to a two-line state and trim readout while enabled.
Right Alt remains the single toggle. Existing auto-trim calibration and linked
seat reconnect repairs from 0.11.51 and 0.11.49 remain in place.

The tire response remains unilateral and uses the shared normal impulse/friction
solve. Predictive crossings retain signed compliant compression, while explicit
True Impact residuals remain rigid. Pose recovery checks both suspension/body
travel and the actual tire or tread hardware envelope. Existing liquid support,
aircraft gear, flat tires and native authored suspension behavior are preserved.

Sable continues to use its existing uniform solidified-body volume estimate
for body pressure geometry and its existing center/inertia inputs. This release
does not change vehicle mass, CG, inertia, wind sampling or pressure coefficients;
an authored pack CG is not inferred or claimed. Wheel travel is a separate
support model and does not rebuild the rigid-body collision shape each substep.

The 0.11.52 trace was used to diagnose the mixed-support allocation; it does not
validate the 0.11.53 change in a live session. No tests or in-game validation
were run for this release. A later spawn and road-vehicle observation is still
needed to assess live handling across packs.
