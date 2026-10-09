package com.g9third.pmweatheriv.physics;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import com.g9third.pmweatheriv.physics.ModelSurfaceMap.Bounds;
import com.g9third.pmweatheriv.physics.ModelSurfaceMap.LiftingPatch;
import com.g9third.pmweatheriv.physics.ModelSurfaceMap.PreparedModel;
import com.g9third.pmweatheriv.physics.ModelSurfaceMap.SurfaceKind;

/**
 * Exercises production CG/AC preparation, force levers and bounded trim.
 * The section law below is a small-angle analytical fixture, not PMAero or a game simulation.
 */
public final class WingMomentRegression {
    private static int checks;
    private record Fixture(PreparedModel model, AirframeGeometry.Geometry geometry,
                           AirframeGeometry.SurfaceAreaPlan areas, Vec3d center) {}

    private static void require(boolean value, String message) {
        ++checks;
        if (!value) throw new AssertionError(message);
    }

    private static void near(double actual, double expected, double tolerance, String message) {
        require(Double.isFinite(actual) && Math.abs(actual - expected) <= tolerance,
            message + " actual=" + actual + " expected=" + expected);
    }

    public static void main(String[] args) {
        checkPhysicalLevers();
        checkNeutralCoefficient();
        checkAuthoredFlapTravel();
        checkArticulatedSurfaceFlow();
        checkVisibleControlCoupling();
        checkPartialControlSectionCoupling();
        checkMeasuredTailIncidence();
        near(LiftingSurfaceAdapter.projectedWingSpan(10, 7, 7), 10, 1e-12,
            "identity authored pose retains prepared span with sparse section centroids");
        near(LiftingSurfaceAdapter.projectedWingSpan(10, 7, 3.5), 5, 1e-12,
            "sweep projected span follows live/baseline sample ratio");
        near(LiftingSurfaceAdapter.projectedWingSpan(10, Double.NaN, 3), 10, 1e-12,
            "missing span evidence preserves prepared geometry");
        for (String configuration : List.of("conventional", "canard", "tailless")) {
            Fixture fixture = fixture(configuration, 0);
            checkConfiguration(configuration, fixture);
            checkDatumInvariance(configuration, fixture, fixture(configuration, 23.5));
        }
        checkSweptForceLever();
        System.out.println("WingMomentRegression: PASS (" + checks
            + " assertions; analytical trim/damping fixtures, no PMAero or game simulation)");
    }

    private static AirframeLoads.Accumulator accumulator(Vec3d center) {
        AircraftState state = new AircraftState();
        state.plan = new AirframePlan(null, null, null, null, new Vec3d(1, 1, 1),
            center, false, List.of(), List.of());
        return new AirframeLoads.Accumulator(state, false, false);
    }

    private static void checkPhysicalLevers() {
        Vec3d force = new Vec3d(0, 100, 0);
        Vec3d ac = new Vec3d(2, .4, 1);
        AirframeLoads.Accumulator aftCenter = accumulator(new Vec3d(0, 0, 0));
        AirframeLoads.Accumulator forwardCenter = accumulator(new Vec3d(0, 0, 2));
        near(aftCenter.momentArm(ac).cross(force).x(), -100, 1e-12,
            "upward force ahead of CG retains signed pitch moment");
        near(forwardCenter.momentArm(ac).cross(force).x(), 100, 1e-12,
            "moving CG ahead of the same AC reverses pitch moment");
        Vec3d pairMoment = forwardCenter.momentArm(ac).cross(force)
            .add(forwardCenter.momentArm(new Vec3d(-2, .4, 1)).cross(force));
        near(pairMoment.x(), 200, 1e-12, "paired wing moments do not cancel the common AC-CG lever");
        near(pairMoment.z(), 0, 1e-12, "paired symmetric lift cancels roll moment");
    }

    private static void checkNeutralCoefficient() {
        near(LiftingSurfaceAdapter.wingZeroLiftCoefficient(0), 0, 0,
            "un-authored neutral wing camber is zero");
        near(LiftingSurfaceAdapter.neutralCanardLiftCoefficient(), 0, 0,
            "un-authored canard camber is zero at geometric neutral");
        require(LiftingSurfaceAdapter.wingZeroLiftCoefficient(.5) > 0,
            "authored flap state retains lift increment");
        near(LiftingSurfaceAdapter.boundedAuthoredControlRadians(0, 0, 45, 5), 0, 0,
            "neutral controls add no pitch command");
        near(LiftingSurfaceAdapter.boundedAuthoredControlRadians(0, 15, 45, 5),
            Math.toRadians(5), 1e-12, "trim stays bounded");
    }

    private static void checkAuthoredFlapTravel() {
        var notches = List.of(0f, 15f, 30f);
        near(LiftingSurfaceAdapter.flapDeploymentFraction(0, notches), 0, 0, "neutral flap adds no camber");
        near(LiftingSurfaceAdapter.flapDeploymentFraction(15, notches), .5, 0,
            "physical flap angle uses authored travel rather than IV's 350 lift-law reference");
        near(LiftingSurfaceAdapter.flapDeploymentFraction(30, notches), 1, 0, "maximum authored notch is full deployment");
        near(LiftingSurfaceAdapter.flapDeploymentFraction(60, notches), 1, 0, "overtravel remains bounded");
        near(LiftingSurfaceAdapter.flapDeploymentFraction(-15, List.of(-30f, 0f, 30f)), -.5, 0,
            "authored reflex preserves the lift-increment sign");
        near(LiftingSurfaceAdapter.wingZeroLiftCoefficient(
            LiftingSurfaceAdapter.flapDeploymentFraction(-15, List.of(-30f, 0f))), -.275, 1e-12,
            "negative flap travel does not acquire universal positive camber");
        near(LiftingSurfaceAdapter.flapDeploymentFraction(10, List.of(0f, 10f, 20f, 30f)), 1.0 / 3, 1e-12,
            "different notch spacings share the same authored full-travel invariant");
        near(LiftingSurfaceAdapter.flapDeploymentFraction(15, List.of()), 15.0 / 350, 1e-12,
            "missing authored travel retains the explicit legacy approximation");
        near(LiftingSurfaceAdapter.flapDeploymentFraction(15, List.of(Float.NaN, 0f, 30f)), .5, 0,
            "invalid notch cannot contaminate finite authored travel");
        near(LiftingSurfaceAdapter.flapDeploymentFraction(15, java.util.Arrays.asList(null, 0f, 30f)), .5, 0,
            "null notch cannot contaminate finite authored travel");
        near(LiftingSurfaceAdapter.flapDeploymentFraction(Double.NaN, notches), 0, 0, "invalid actual angle stays neutral");
    }

    private static void checkArticulatedSurfaceFlow() {
        Vec3d parentMotion = new Vec3d(0.2, -0.1, 0.0);
        Vec3d flapRelativeMotion = new Vec3d(0.0, 2.0, -0.5);
        Vec3d stationaryPanel = LiftingSurfaceAdapter.mixedSurfaceKinematicVelocity(
            parentMotion, flapRelativeMotion, 0.0, 4.0
        );
        near(stationaryPanel.x(), parentMotion.x(), 1E-12,
            "a nonmoving control panel adds no relative-flow speed");
        near(stationaryPanel.y(), parentMotion.y(), 1E-12,
            "parent pose motion remains fully represented when control area is zero");

        Vec3d quarterFlap = LiftingSurfaceAdapter.mixedSurfaceKinematicVelocity(
            parentMotion, flapRelativeMotion, 1.0, 4.0
        );
        near(quarterFlap.x(), 0.2, 1E-12, "control motion preserves common parent translation");
        near(quarterFlap.y(), 0.4, 1E-12,
            "control-only airflow is integrated by its moving planform fraction");
        near(quarterFlap.z(), -0.125, 1E-12,
            "spanwise and chordwise articulation components are both retained");

        Vec3d fullFlap = LiftingSurfaceAdapter.mixedSurfaceKinematicVelocity(
            parentMotion, flapRelativeMotion, 3.0, 3.0
        );
        near(fullFlap.y(), 1.9, 1E-12,
            "an all-moving tail receives the complete relative actuation flow");
        Vec3d bounded = LiftingSurfaceAdapter.mixedSurfaceKinematicVelocity(
            parentMotion, flapRelativeMotion, 9.0, 3.0
        );
        near(bounded.y(), fullFlap.y(), 1E-12,
            "moving surface area cannot exceed the physical total-area share");
        Vec3d invalid = LiftingSurfaceAdapter.mixedSurfaceKinematicVelocity(
            parentMotion, new Vec3d(Double.NaN, 0.0, 0.0), 1.0, Double.NaN
        );
        near(invalid.x(), parentMotion.x(), 1E-12,
            "invalid geometry or motion cannot inject an articulated-flow impulse");
    }

    private static void checkVisibleControlCoupling() {
        List<LiftingPatch> patches = new ArrayList<>();
        patches.add(sectionPatch("wing_left", SurfaceKind.WING, -2.5, 0.0, 5.0));
        patches.add(sectionPatch("wing_right", SurfaceKind.WING, 2.5, 0.0, 5.0));
        patches.add(sectionPatch("aileron_left", SurfaceKind.AILERON, -2.5, 0.0, 5.0));
        patches.add(sectionPatch("aileron_right", SurfaceKind.AILERON, 2.5, 0.0, 5.0));
        patches.add(sectionPatch("tail_left", SurfaceKind.HORIZONTAL_TAIL, -1.0, 0.0, 0.0));
        patches.add(sectionPatch("tail_right", SurfaceKind.HORIZONTAL_TAIL, 1.0, 0.0, 0.0));
        patches.add(sectionPatch("elevator_left", SurfaceKind.ELEVATOR, -1.0, 0.0, 0.0));
        patches.add(sectionPatch("elevator_right", SurfaceKind.ELEVATOR, 1.0, 0.0, 0.0));
        Bounds bounds = new Bounds(new Vec3d(-5, -1, -5), new Vec3d(5, 1, 5), true);
        PreparedModel model = new PreparedModel("visibility-fixture", true, "fixture", bounds, bounds,
            List.of(), patches, 0, 0, 0, 0);
        AirframeGeometry.Geometry geometry = new AirframeGeometry.Geometry(
            2, 1, 10, 10, 10, 2, 0, 4, 4, 1, 4, 2, 0.5, 0, 10
        );
        AirframeGeometry.SurfaceAreaPlan areas = AirframeGeometry.SurfaceAreaPlan.from(model, geometry);
        LiftingPatch wingLeft = patches.get(0);
        LiftingPatch wingRight = patches.get(1);
        LiftingPatch tailLeft = patches.get(4);
        LiftingPatch tailRight = patches.get(5);
        var coupling = new LiftingSurfaceAdapter.AileronWingCoupling(true, 0, 5, .25, .5, 5);
        StructuralDamageMask.Snapshot intact = StructuralDamageMask.Snapshot.EMPTY;

        var visible = poseSnapshot(
            pose("aileron_left", AuthoredLiftingSurfacePoses.Status.AUTHORED, false, true, 0.1, 0),
            pose("aileron_right", AuthoredLiftingSurfacePoses.Status.AUTHORED, false, true, 0.1, 0),
            pose("elevator_left", AuthoredLiftingSurfacePoses.Status.AUTHORED, true, false, 0.1, 0),
            pose("elevator_right", AuthoredLiftingSurfacePoses.Status.AUTHORED, true, false, 0.1, 0)
        );
        double leftWingArea = LiftingSurfaceAdapter.wingControlAreaForWingPatch(
            model, wingLeft, coupling, areas, SurfaceKind.AILERON, visible, intact);
        double rightWingArea = LiftingSurfaceAdapter.wingControlAreaForWingPatch(
            model, wingRight, coupling, areas, SurfaceKind.AILERON, visible, intact);
        near(leftWingArea, 1.0, 1e-12, "visible left aileron contributes its parent area");
        near(rightWingArea, 1.0, 1e-12, "visible right aileron contributes its parent area");
        double fullStripControl = LiftingSurfaceAdapter.wingControlStripControlRadians(
            model, areas, visible, intact, wingLeft, coupling, SurfaceKind.AILERON,
            new LiftingSurfaceAdapter.ControlDeflection(0, .2, 0, 0, 0, 0));
        require(fullStripControl > 0, "visible aileron retains parent deflection authority");

        var hiddenLeft = poseSnapshot(
            pose("aileron_left", AuthoredLiftingSurfacePoses.Status.HIDDEN, false, true, 0.1, 0),
            pose("aileron_right", AuthoredLiftingSurfacePoses.Status.AUTHORED, false, true, 0.1, 0),
            pose("elevator_left", AuthoredLiftingSurfacePoses.Status.HIDDEN, true, false, 0.1, 0),
            pose("elevator_right", AuthoredLiftingSurfacePoses.Status.AUTHORED, true, false, 0.1, 0)
        );
        near(LiftingSurfaceAdapter.wingControlAreaForWingPatch(
            model, wingLeft, coupling, areas, SurfaceKind.AILERON, hiddenLeft, intact),
            0, 1e-12, "hidden left aileron contributes no parent force area");
        near(LiftingSurfaceAdapter.wingControlAreaForWingPatch(
            model, wingRight, coupling, areas, SurfaceKind.AILERON, hiddenLeft, intact),
            rightWingArea, 1e-12, "hidden left area is not reassigned to the right wing");
        near(LiftingSurfaceAdapter.wingControlStripControlRadians(
            model, areas, hiddenLeft, intact, wingLeft, coupling, SurfaceKind.AILERON,
            new LiftingSurfaceAdapter.ControlDeflection(0, .2, 0, 0, 0, 0)),
            0, 1e-12, "hidden left aileron loses scalar parent authority");
        near(LiftingSurfaceAdapter.wingControlStripControlRadians(
            model, areas, hiddenLeft, intact, wingRight, coupling, SurfaceKind.AILERON,
            new LiftingSurfaceAdapter.ControlDeflection(0, .2, 0, 0, 0, 0)),
            fullStripControl, 1e-12, "visible right aileron retains scalar authority");
        near(LiftingSurfaceAdapter.coupledParentControlRadians(
            model, areas, hiddenLeft, intact, tailLeft, null, SurfaceKind.ELEVATOR,
            new LiftingSurfaceAdapter.ControlDeflection(1, 0, 0, 0, 0, 0), .4),
            0, 1e-12, "hidden elevator leaves no scalar deflection on its parent");
        near(LiftingSurfaceAdapter.coupledParentControlRadians(
            model, areas, hiddenLeft, intact, tailRight, null, SurfaceKind.ELEVATOR,
            new LiftingSurfaceAdapter.ControlDeflection(1, 0, 0, 0, 0, 0), .4),
            .4, 1e-12, "visible elevator retains parent deflection authority");
        near(LiftingSurfaceAdapter.movingControlAreaForParentPatch(
            model, areas, tailLeft, SurfaceKind.HORIZONTAL_TAIL, SurfaceKind.ELEVATOR,
            hiddenLeft, intact),
            0, 1e-12, "hidden elevator contributes no parent force area");
        double rightTailArea = LiftingSurfaceAdapter.movingControlAreaForParentPatch(
            model, areas, tailRight, SurfaceKind.HORIZONTAL_TAIL, SurfaceKind.ELEVATOR,
            hiddenLeft, intact);
        near(rightTailArea, .5, 1e-12, "hidden elevator area is not redistributed onto the right tail");

        var frozenLeft = poseSnapshot(
            pose("aileron_left", AuthoredLiftingSurfacePoses.Status.FROZEN_NO_SOURCE, false, false, 0, 0),
            pose("aileron_right", AuthoredLiftingSurfacePoses.Status.AUTHORED, false, true, 0.1, 0),
            pose("elevator_left", AuthoredLiftingSurfacePoses.Status.FROZEN_NO_SOURCE, false, false, 0, 0),
            pose("elevator_right", AuthoredLiftingSurfacePoses.Status.AUTHORED, true, false, 0.1, 0)
        );
        near(LiftingSurfaceAdapter.wingControlAreaForWingPatch(
            model, wingLeft, coupling, areas, SurfaceKind.AILERON, frozenLeft, intact),
            leftWingArea, 1e-12, "visible frozen aileron retains physical parent area");
        near(LiftingSurfaceAdapter.wingControlStripControlRadians(
            model, areas, frozenLeft, intact, wingLeft, coupling, SurfaceKind.AILERON,
            new LiftingSurfaceAdapter.ControlDeflection(0, .2, 0, 0, 0, 0)),
            fullStripControl, 1e-12, "visible frozen aileron retains parent deflection authority");
        near(LiftingSurfaceAdapter.movingControlAreaForParentPatch(
            model, areas, tailLeft, SurfaceKind.HORIZONTAL_TAIL, SurfaceKind.ELEVATOR,
            frozenLeft, intact),
            .5, 1e-12, "visible frozen elevator retains physical parent area");
        Vec3d frozenRate = LiftingSurfaceAdapter.relativeWingControlVelocityForStrip(
            model, areas, frozenLeft, wingLeft,
            pose("wing_left", AuthoredLiftingSurfacePoses.Status.FROZEN_NO_SOURCE, false, false, 0, 0),
            coupling, SurfaceKind.AILERON, intact);
        near(frozenRate.length(), 0, 1e-12, "visible frozen aileron invents no articulation rate");
        Vec3d visibleRightRate = LiftingSurfaceAdapter.relativeWingControlVelocityForStrip(
            model, areas, visible, wingRight,
            pose("wing_right", AuthoredLiftingSurfacePoses.Status.FROZEN_NO_SOURCE, false, false, 0, 0),
            coupling, SurfaceKind.AILERON, intact);
        near(visibleRightRate.y(), 2.0, 1e-12, "visible authored rate uses the matching area subset");

        near(LiftingSurfaceAdapter.wingControlAreaForWingPatch(
            model, wingLeft, coupling, areas, SurfaceKind.AILERON, visible, intact),
            leftWingArea, 1e-12, "regained aileron restores its original parent share");
        near(LiftingSurfaceAdapter.coupledParentControlRadians(
            model, areas, visible, intact, tailLeft, null, SurfaceKind.ELEVATOR,
            new LiftingSurfaceAdapter.ControlDeflection(1, 0, 0, 0, 0, 0), .4),
            .4, 1e-12, "regained elevator restores its original parent authority");

        // A retained detached part has HIDDEN status and remains excluded.
        var detachedLeft = poseSnapshot(
            pose("aileron_left", AuthoredLiftingSurfacePoses.Status.HIDDEN, false, true, 0.1, 0),
            pose("aileron_right", AuthoredLiftingSurfacePoses.Status.AUTHORED, false, true, 0.1, 0),
            pose("elevator_left", AuthoredLiftingSurfacePoses.Status.HIDDEN, true, false, 0.1, 0),
            pose("elevator_right", AuthoredLiftingSurfacePoses.Status.AUTHORED, true, false, 0.1, 0)
        );
        near(LiftingSurfaceAdapter.wingControlAreaForWingPatch(
            model, wingLeft, coupling, areas, SurfaceKind.AILERON, detachedLeft, intact),
            0, 1e-12, "retained detached aileron remains excluded from parent area");
        near(LiftingSurfaceAdapter.coupledParentControlRadians(
            model, areas, detachedLeft, intact, tailLeft, null, SurfaceKind.ELEVATOR,
            new LiftingSurfaceAdapter.ControlDeflection(1, 0, 0, 0, 0, 0), .4),
            0, 1e-12, "retained detached elevator remains excluded from parent control");
    }

    private static void checkPartialControlSectionCoupling() {
        LiftingPatch innerWing = sectionPatch("inner_wing", SurfaceKind.WING, 1.75, 1.0, 2.5);
        LiftingPatch middleWing = sectionPatch("middle_wing", SurfaceKind.WING, 3.15, 2.5, 3.8);
        LiftingPatch wing = sectionPatch("tip_wing", SurfaceKind.WING, 4.3, 3.8, 4.8);
        LiftingPatch fullControl = sectionPatch("full_aileron", SurfaceKind.AILERON, 2.5, 1.0, 4.0);
        LiftingPatch partialControl = sectionPatch("partial_aileron", SurfaceKind.AILERON, 4.2, 3.0, 5.0);
        List<LiftingPatch> patches = List.of(innerWing, middleWing, wing, fullControl, partialControl);
        Bounds bounds = new Bounds(new Vec3d(-5, -1, -5), new Vec3d(5, 1, 5), true);
        PreparedModel model = new PreparedModel("partial-visibility-fixture", true, "fixture", bounds,
            bounds, List.of(), patches, 0, 0, 0, 0);
        AirframeGeometry.Geometry geometry = new AirframeGeometry.Geometry(
            2, 1, 10, 10, 10, 2, 0, 4, 4, 1, 4, 2, 0.5, 0, 10
        );
        AirframeGeometry.SurfaceAreaPlan areas = AirframeGeometry.SurfaceAreaPlan.from(model, geometry);
        var coupling = new LiftingSurfaceAdapter.AileronWingCoupling(true, 1.0, 4.0, .25, .5, 5.0);
        StructuralDamageMask.Snapshot intact = StructuralDamageMask.Snapshot.EMPTY;
        var visible = poseSnapshot(
            pose("full_aileron", AuthoredLiftingSurfacePoses.Status.AUTHORED, false, true, .1, 0),
            pose("partial_aileron", AuthoredLiftingSurfacePoses.Status.AUTHORED, false, true, .2, 0)
        );
        double fullArea = areas.areaFor(fullControl);
        double partialArea = areas.areaFor(partialControl);
        near(fullArea, 1.0, 1e-12, "partial-section fixture allocates equal authored control patches");
        near(partialArea, 1.0, 1e-12, "partial-section fixture retains immutable global area budget");
        double parentOverlapFraction = .2 / 3.0;
        double representedControlArea = fullArea + partialArea * .5;
        near(LiftingSurfaceAdapter.wingControlAreaForWingPatch(
            model, wing, coupling, areas, SurfaceKind.AILERON, visible, intact),
            representedControlArea * parentOverlapFraction, 1e-12,
            "moving area uses control-span and parent-section overlaps together");
        double representedAcrossParent = LiftingSurfaceAdapter.wingControlAreaForWingPatch(
            model, innerWing, coupling, areas, SurfaceKind.AILERON, visible, intact)
            + LiftingSurfaceAdapter.wingControlAreaForWingPatch(
                model, middleWing, coupling, areas, SurfaceKind.AILERON, visible, intact)
            + LiftingSurfaceAdapter.wingControlAreaForWingPatch(
                model, wing, coupling, areas, SurfaceKind.AILERON, visible, intact);
        near(representedAcrossParent, representedControlArea, 1e-12,
            "control area is conserved across parent sections covering the coupled band");
        var frozenWing = pose("tip_wing", AuthoredLiftingSurfacePoses.Status.FROZEN_NO_SOURCE,
            false, false, 0, 0);
        Vec3d rate = LiftingSurfaceAdapter.relativeWingControlVelocityForStrip(
            model, areas, visible, wing, frozenWing, coupling, SurfaceKind.AILERON, intact);
        near(rate.y(), (fullArea * 2.0 + partialArea * .5 * 4.0) / representedControlArea,
            1e-12, "strip rate uses exactly the control area represented in the section");
        double controlAngle = LiftingSurfaceAdapter.wingControlStripControlRadians(
            model, areas, visible, intact, wing, coupling, SurfaceKind.AILERON,
            new LiftingSurfaceAdapter.ControlDeflection(0, .2, 0, 0, 0, 0));
        near(controlAngle, .2 * .5 * .2, 1e-12,
            "a parent section centered outside the band receives its overlap-scaled control");
        require(rate.y() > 0,
            "section overlap contributes motion when its centroid lies outside the control band");

        var hiddenPartial = poseSnapshot(
            pose("full_aileron", AuthoredLiftingSurfacePoses.Status.AUTHORED, false, true, .1, 0),
            pose("partial_aileron", AuthoredLiftingSurfacePoses.Status.HIDDEN, false, true, .2, 0)
        );
        near(LiftingSurfaceAdapter.wingControlAreaForWingPatch(
            model, wing, coupling, areas, SurfaceKind.AILERON, hiddenPartial, intact),
            fullArea * parentOverlapFraction, 1e-12,
            "hidden partial control drops its own represented area without redistribution");
        double visibleSubsetAcrossParent = LiftingSurfaceAdapter.wingControlAreaForWingPatch(
            model, innerWing, coupling, areas, SurfaceKind.AILERON, hiddenPartial, intact)
            + LiftingSurfaceAdapter.wingControlAreaForWingPatch(
                model, middleWing, coupling, areas, SurfaceKind.AILERON, hiddenPartial, intact)
            + LiftingSurfaceAdapter.wingControlAreaForWingPatch(
                model, wing, coupling, areas, SurfaceKind.AILERON, hiddenPartial, intact);
        near(visibleSubsetAcrossParent, fullArea, 1e-12,
            "hidden partial control does not redistribute its conserved parent-band share");
        Vec3d remainingRate = LiftingSurfaceAdapter.relativeWingControlVelocityForStrip(
            model, areas, hiddenPartial, wing, frozenWing, coupling, SurfaceKind.AILERON, intact);
        near(remainingRate.y(), 2.0, 1e-12,
            "hidden partial control does not dilute the remaining visible control rate");
        near(LiftingSurfaceAdapter.wingControlStripControlRadians(
            model, areas, hiddenPartial, intact, wing, coupling, SurfaceKind.AILERON,
            new LiftingSurfaceAdapter.ControlDeflection(0, .2, 0, 0, 0, 0)),
            .2 * .5 * .2 * (fullArea / representedControlArea), 1e-12,
            "hidden partial control loses only its represented scalar authority");
    }

    private static LiftingPatch sectionPatch(String name, SurfaceKind kind, double x,
            double minimumRadius, double maximumRadius) {
        ModelSurfaceMap.SymmetryRole role = x < 0
            ? ModelSurfaceMap.SymmetryRole.MIRRORED_LEFT
            : ModelSurfaceMap.SymmetryRole.MIRRORED_RIGHT;
        return new LiftingPatch(name, kind, new Vec3d(x, 0, 0), 1, 1, role,
            0, 1, minimumRadius, maximumRadius);
    }

    private static AuthoredLiftingSurfacePoses.Pose pose(String name,
            AuthoredLiftingSurfacePoses.Status status, boolean elevator, boolean aileron,
            double currentY, double previousY) {
        AuthoredLiftingSurfacePoses.Binding binding = new AuthoredLiftingSurfacePoses.Binding(
            name, elevator, aileron, false, false, false, status
        );
        return new AuthoredLiftingSurfacePoses.Pose(
            translated(currentY), binding, status,
            status == AuthoredLiftingSurfacePoses.Status.AUTHORED ? translated(previousY) : null
        );
    }

    private static AnimatedWingGeometry.RigidTransform translated(double y) {
        return new AnimatedWingGeometry.RigidTransform(
            1, 0, 0, 0, 1, 0, 0, 0, 1, new Vec3d(0, y, 0)
        );
    }

    private static AuthoredLiftingSurfacePoses.Snapshot poseSnapshot(
            AuthoredLiftingSurfacePoses.Pose... poses) {
        Map<String, AuthoredLiftingSurfacePoses.Pose> patches = new java.util.LinkedHashMap<>();
        for (AuthoredLiftingSurfacePoses.Pose pose : poses) {
            patches.put(pose.binding().objectName(), pose);
        }
        return new AuthoredLiftingSurfacePoses.Snapshot(patches);
    }

    private static void checkMeasuredTailIncidence() {
        LiftingPatch wing = fittedPatch("wing", SurfaceKind.WING, 5, 1);
        LiftingPatch tail = fittedPatch("tail", SurfaceKind.HORIZONTAL_TAIL, -1, 1);
        LiftingPatch elevator = fittedPatch("elevator", SurfaceKind.ELEVATOR, 3, 1);
        LiftingPatch taileron = fittedPatch("all_moving", SurfaceKind.TAILERON, 2, 1);
        List<LiftingPatch> fitted = LiftingSurfaceFrames.regularizeHorizontalTailParentFrames(
            List.of(wing, tail, elevator, taileron));
        near(fitted.get(0).geometryIncidenceDegrees(), 5, 1e-12, "wing mesh incidence retained");
        near(fitted.get(1).geometryIncidenceDegrees(), -1, 1e-12, "tail follows measured stabilizer, not wing minus 2.5 degrees");
        near(fitted.get(2).geometryIncidenceDegrees(), -1, 1e-12, "attached elevator inherits measured parent neutral plane");
        near(fitted.get(3).geometryIncidenceDegrees(), 2, 1e-12, "all-moving tail retains its own neutral geometry");
        List<LiftingPatch> noParent = LiftingSurfaceFrames.regularizeHorizontalTailParentFrames(List.of(wing, elevator));
        near(noParent.get(1).geometryIncidenceDegrees(), 3, 1e-12, "standalone control retains its own incidence");
        LiftingPatch fallbackTail = fittedPatch("missing_tail", SurfaceKind.HORIZONTAL_TAIL, 0, 0);
        List<LiftingPatch> missing = LiftingSurfaceFrames.regularizeHorizontalTailParentFrames(List.of(fallbackTail, elevator));
        near(missing.get(1).geometryIncidenceDegrees(), 3, 1e-12, "missing geometry cannot overwrite measured control plane");
        LiftingPatch neutral = fittedPatch("neutral_tail", SurfaceKind.HORIZONTAL_TAIL, 0, 1);
        near(LiftingSurfaceFrames.regularizeHorizontalTailParentFrames(List.of(wing, neutral)).get(1)
            .geometryIncidenceDegrees(), 0, 1e-12, "geometrically neutral tail has no universal pitch incidence");
    }

    private static LiftingPatch fittedPatch(String name, SurfaceKind kind, double incidenceDegrees, double confidence) {
        double angle = Math.toRadians(incidenceDegrees);
        Vec3d point = new Vec3d(1, 0, kind == SurfaceKind.WING ? 1 : -4);
        return new LiftingPatch(name, kind, point, 1, 0, ModelSurfaceMap.SymmetryRole.MIRRORED_RIGHT,
            0, 1, 0, 0, new Vec3d(1, 0, 0), new Vec3d(0, Math.sin(angle), Math.cos(angle)),
            new Vec3d(0, Math.cos(angle), -Math.sin(angle)), confidence, point);
    }

    private static Fixture fixture(String configuration, double datumZ) {
        SurfaceKind wing = configuration.equals("tailless") ? SurfaceKind.ELEVON : SurfaceKind.WING;
        List<LiftingPatch> patches = new ArrayList<>();
        patches.add(new LiftingPatch("left_panel", wing, new Vec3d(-2, 0, 1.4 + datumZ), 1));
        patches.add(new LiftingPatch("right_panel", wing, new Vec3d(2, 0, 1.4 + datumZ), 1));
        if (configuration.equals("conventional")) {
            patches.add(new LiftingPatch("rear_left", SurfaceKind.HORIZONTAL_TAIL,
                new Vec3d(-1, 0, -4.6 + datumZ), 1));
            patches.add(new LiftingPatch("rear_right", SurfaceKind.HORIZONTAL_TAIL,
                new Vec3d(1, 0, -4.6 + datumZ), 1));
        } else if (configuration.equals("canard")) {
            patches.add(new LiftingPatch("front_left", SurfaceKind.CANARDERON,
                new Vec3d(-1, 0, 5.4 + datumZ), 1));
            patches.add(new LiftingPatch("front_right", SurfaceKind.CANARDERON,
                new Vec3d(1, 0, 5.4 + datumZ), 1));
        }
        Bounds bounds = new Bounds(new Vec3d(-5, -1, -8 + datumZ),
            new Vec3d(5, 1, 8 + datumZ), true);
        PreparedModel model = new PreparedModel("synthetic", true, "fixture", bounds, bounds,
            List.of(), List.copyOf(patches), 0, 0, 0, 0);
        AirframeGeometry.Geometry geometry = new AirframeGeometry.Geometry(
            2, 1, 12, 20, 10, 0, configuration.equals("tailless") ? 20 : 0,
            6, 4, 0, 3, 0, 0, 3, 5);
        AirframeGeometry.SurfaceAreaPlan areas = AirframeGeometry.SurfaceAreaPlan.from(model, geometry);
        double acZ = LiftingSurfaceAdapter.mainWingAerodynamicCenterReferenceZ(model, areas);
        near(acZ, 1.4 + datumZ, 1e-12, configuration + " area-weighted AC remains in model frame");
        LiftingSurfaceAdapter.PreparedFixedWingPlan plan = new LiftingSurfaceAdapter.PreparedFixedWingPlan(
            LiftingSurfaceAdapter.AileronWingCoupling.DISABLED,
            LiftingSurfaceAdapter.AileronWingCoupling.DISABLED, Map.of(), AnimatedWingGeometry.empty(),
            acZ, 2, 1, 1, .5, .5, .5);
        Vec3d center = AirframePreparation.deriveCenterOfMass(null, model, geometry, plan, false).position();
        double expectedCenterZ = acZ + (configuration.equals("conventional") ? 0 : .1);
        near(center.z(), expectedCenterZ, 1e-12, configuration + " retains existing generic CG prior");
        return new Fixture(model, geometry, areas, center);
    }

    private static void checkConfiguration(String name, Fixture fixture) {
        AirframePreparation.DerivedCenterOfMassConfiguration expected = name.equals("conventional")
            ? AirframePreparation.DerivedCenterOfMassConfiguration.CONVENTIONAL_HORIZONTAL_TAIL
            : name.equals("tailless")
                ? AirframePreparation.DerivedCenterOfMassConfiguration.TAILLESS_ELEVON_MODEL
                : AirframePreparation.DerivedCenterOfMassConfiguration.UNRESOLVED_SAFE_FORWARD_MARGIN;
        require(AirframePreparation.centerOfMassConfiguration(fixture.model()) == expected,
            name + " uses production configuration classification");
        if (!name.equals("conventional")) {
            near(accumulator(fixture.center()).momentArm(fixture.model().liftingPatches().getFirst()
                .aerodynamicCenterLocal()).z(), -.1, 1e-12,
                name + " forward static margin survives as a physical wing lever");
        }
        for (double speed : new double[] {20, 40, 80}) {
            double authority = moment(fixture, 0, 1, 0, speed) - moment(fixture, 0, -1, 0, speed);
            require(Math.abs(authority) > 1e-6, name + " has nonzero trim moment authority");
            for (double alpha : new double[] {-.03, 0, .03}) {
                double atZero = moment(fixture, alpha, 0, 0, speed);
                double perDegree = moment(fixture, alpha, 1, 0, speed) - atZero;
                double trimDegrees = -atZero / perDegree;
                require(Math.abs(trimDegrees) <= 5, name + " analytical equilibrium fits bounded trim");
                near(moment(fixture, alpha, trimDegrees, 0, speed), 0, 1e-8,
                    name + " trim can balance physical lever moments");
            }
            for (double rate : new double[] {-.2, -.02, .02, .2}) {
                double dampingMoment = moment(fixture, 0, 0, rate, speed);
                require(dampingMoment * rate < 0, name + " lever-based pitch damping opposes rate");
            }
        }
    }

    private static void checkDatumInvariance(String name, Fixture base, Fixture translated) {
        for (double trim : new double[] {-2, 0, 2}) {
            near(moment(base, .02, trim, .1, 40), moment(translated, .02, trim, .1, 40),
                1e-8, name + " common model-origin translation leaves moments unchanged");
        }
    }

    private static double moment(Fixture fixture, double alpha, double trimDegrees, double pitchRate, double speed) {
        AirframeLoads.Accumulator accumulator = accumulator(fixture.center());
        double trim = LiftingSurfaceAdapter.boundedAuthoredControlRadians(0, trimDegrees, 45, 5);
        Vec3d omega = new Vec3d(pitchRate, 0, 0);
        double pressure = .5 * 1.225 * speed * speed;
        double pitchMoment = 0;
        for (LiftingPatch patch : fixture.model().liftingPatches()) {
            Vec3d lever = accumulator.momentArm(patch.aerodynamicCenterLocal());
            double verticalPointVelocity = omega.cross(lever).y();
            boolean controlled = patch.kind() == SurfaceKind.HORIZONTAL_TAIL
                || patch.kind() == SurfaceKind.CANARDERON || patch.kind() == SurfaceKind.ELEVON;
            // A linear, symmetric section supplies a known sign to the real lever kernel.
            // This isolates geometry/control moment authority from uncalibrated PMAero coefficients.
            double incidence = alpha - verticalPointVelocity / speed - (controlled ? trim : 0);
            double lift = pressure * fixture.areas().areaFor(patch) * (2 * Math.PI) * incidence;
            pitchMoment += lever.cross(new Vec3d(0, lift, 0)).x();
        }
        return pitchMoment;
    }

    private static void checkSweptForceLever() {
        Vec3d center = new Vec3d(0, 0, .5);
        Vec3d ac = new Vec3d(4, 0, 1.4);
        AnimatedWingGeometry.RigidTransform sweep = AnimatedWingGeometry.RigidTransform.rotationAround(
            new Vec3d(1, 0, 1), new Vec3d(0, 1, 0), .4);
        Vec3d liveCenter = sweep.point(ac);
        double moment = accumulator(center).momentArm(liveCenter).cross(new Vec3d(0, 100, 0)).x();
        near(moment, -100 * (liveCenter.z() - center.z()), 1e-12,
            "wing motion changes pitch lever at its actual transformed AC");
        require(Math.abs(moment) > 1e-6, "live wing lever is not recentered onto CG");
    }
}
