package org.firstinspires.ftc.teamcode.pedro;

//import com.bylazar.configurables.annotations.Configurable;

//import com.bylazar.configurables.annotations.IgnoreConfigurable;

//import com.bylazar.field.FieldManager;
//import com.bylazar.field.PanelsField;
//import com.bylazar.field.Style;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.bylazar.ftcontrol.panels.configurables.annotations.IgnoreConfigurable;
//import com.bylazar.ftcontrol.panels.configurables.annotations.Configurable;
//import com.bylazar.ftcontrol.panels.Panels;
//import com.bylazar.ftcontrol.panels.integration.TelemetryManager;
//import com.bylazar.ftcontrol.panels.json.Canvas;
//import com.bylazar.ftcontrol.panels.json.CanvasPreset;
//import com.bylazar.ftcontrol.panels.json.CanvasRotation;
//import com.bylazar.ftcontrol.panels.json.Circle;
//import com.bylazar.ftcontrol.panels.json.Line;
//import com.bylazar.ftcontrol.panels.json.Look;
//import com.bylazar.ftcontrol.panels.json.Point;
//import com.bylazar.ftcontrol.panels.json.Rectangle;
//import com.bylazar.ftcontrol.panels.Panels;
//import com.bylazar.ftcontrol.panels.integration.TelemetryManager;
//import com.bylazar.ftcontrol.panels.json.Look;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.Canvas;
//import com.bylazar.field.CanvasRotation;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Pedro Pathing 3.x tuning suite.
 *
 * CHANGES FROM THE 2.x VERSION
 *  - SelectableOpMode is gone in 3.0. Replaced with a D-pad menu driven by the
 *    normal OpMode lifecycle, so this file depends on nothing unverified.
 *  - Follower.startTeleopDrive() is gone. Calling follower.manual(...) is itself
 *    what puts the Follower into MANUAL mode, so those calls were simply removed.
 *  - Velocity has public fields vx/vy/omega (world-frame), not getXComponent().
 *    Forward/lateral components are resolved below with the same formula the
 *    library uses internally in Velocity.toTwist(heading).
 *  - The Translational / Heading / Drive tuners were dropped. Their whole purpose
 *    was activateTranslational() / activateHeading() / activateDrive(), which have
 *    no equivalent in the Foresight algorithm. Without those calls all three were
 *    identical to the Line test, so only Line remains.
 *
 * BEFORE RUNNING THE ZERO-POWER-ACCELERATION TUNERS
 *    Set c.manualBrakeMode.set(false) in your MecanumConfig. Those two tuners
 *    measure how the robot coasts down under zero power. With brake mode on
 *    (the 3.0 default) you measure braking instead and the number will be wrong.
 */
@Configurable
@TeleOp(name = "Pedro Tuning", group = "Pedro Pathing")
public class Tuning extends OpMode {

    public static Follower follower;

    /* ---- live-editable in Panels ---- */
    public static double PUSH_DISTANCE     = 36;          // Forward / Lateral push tuners
    public static double PUSH_ANGLE        = 2 * Math.PI; // Turn tuner, radians
    public static double VELOCITY_DISTANCE = 36;          // Velocity tuners
    public static int    VELOCITY_SAMPLES  = 10;
    public static double ZPA_VELOCITY      = 30;          // Zero-power-accel trigger speed
    public static double LINE_DISTANCE     = 40;
    public static double CURVE_DISTANCE    = 20;
    public static double CIRCLE_RADIUS     = 10;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    @IgnoreConfigurable
    static final ArrayList<String> changes = new ArrayList<>();

    static final PoseFactory POSE = PoseFactory.degrees();
    static final Pose START = POSE.of(72, 72, 0);

    private final List<Tuner> tuners = Arrays.asList(
            new LocalizationTest(),
            new ForwardTuner(),
            new LateralTuner(),
            new TurnTuner(),
            new ForwardVelocityTuner(),
            new LateralVelocityTuner(),
            new ForwardZeroPowerAccelerationTuner(),
            new LateralZeroPowerAccelerationTuner(),
            new LineTest(),
            new CurveTest(),
            new TriangleTest(),
            new CircleTest());

    private int index = 0;
    private Tuner selected = null;
    private boolean prevUp, prevDown, prevA;
//    Canvas canvas = new Canvas(CanvasPreset.PEDRO_PATHING);
//    private static Canvas canvas = new Canvas()
//            .withOffsetX(-72.0)
//            .withOffsetY(72.0)
//            .withRotation(CanvasRotation.DEG_270);
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
//    private static final Style robotLook = new Style("", "#3F51B5", 0.75);
//    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    //    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
//    private static final Style robotLook = new Style("", "#3F51B5", 0.75);

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(START);
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//        telemetryM = Panels.getTelemetry();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        Drawing.init();
        changes.clear();
    }

    @Override
    public void init_loop() {
        follower.update();

        if (selected == null) {
            boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down, a = gamepad1.a;
            if (up && !prevUp)     index = (index - 1 + tuners.size()) % tuners.size();
            if (down && !prevDown) index = (index + 1) % tuners.size();
            if (a && !prevA) {
                selected = tuners.get(index);
                selected.init(this);
            }
            prevUp = up; prevDown = down; prevA = a;

            telemetryM.debug("D-pad up/down to choose, A to select, then PLAY.");
            telemetryM.debug("");
            for (int i = 0; i < tuners.size(); i++) {
                telemetryM.debug((i == index ? " > " : "   ") + tuners.get(i).name());
            }
        } else {
            telemetryM.debug("Selected: " + selected.name());
            telemetryM.debug("");
            selected.initLoop(this);
        }

        telemetryM.update(telemetry);
        drawOnlyCurrent();
    }

    @Override
    public void start() {
        if (selected == null) selected = tuners.get(index);
        selected.start(this);
    }

    @Override
    public void loop() {
        if (gamepad1.b) {
            stopRobot();
            requestOpModeStop();
            return;
        }

        follower.update();
        selected.loop(this);

        if (!changes.isEmpty()) {
            telemetryM.debug("");
            telemetryM.debug("--- recorded ---");
            for (String c : changes) telemetryM.debug(c);
        }

        telemetryM.update(telemetry);
        draw();
    }

    @Override
    public void stop() {
        if (follower != null) follower.stop();
    }

    /* ------------------------------------------------------------------ */
    /*  shared helpers                                                     */
    /* ------------------------------------------------------------------ */

    /** Forward (robot-frame) velocity, resolved exactly as Velocity.toTwist does. */
    static double forwardVelocity() {
        Velocity v = follower.velocity();
        double h = follower.pose().heading();
        return v.vx * Math.cos(h) + v.vy * Math.sin(h);
    }

    /** Lateral (robot-frame) velocity. */
    static double lateralVelocity() {
        Velocity v = follower.velocity();
        double h = follower.pose().heading();
        return -v.vx * Math.sin(h) + v.vy * Math.cos(h);
    }

    static void record(String field, double value) {
        String line = field + " = " + value;
        if (!changes.contains(line)) changes.add(line);
    }

    public static void drawOnlyCurrent() {
        Drawing.drawRobot(follower.pose());
        Drawing.sendPacket();
    }

    public static void draw() {
        Drawing.drawDebug(follower);
    }

    public static void stopRobot() {
        follower.stop();
    }

    /* ------------------------------------------------------------------ */
    /*  tuner contract                                                     */
    /* ------------------------------------------------------------------ */

    interface Tuner {
        String name();
        default void init(Tuning t)     {}
        default void initLoop(Tuning t) {}
        default void start(Tuning t)    {}
        void loop(Tuning t);
    }

    /* ------------------------------------------------------------------ */
    /*  localization                                                       */
    /* ------------------------------------------------------------------ */

    static class LocalizationTest implements Tuner {
        public String name() { return "Localization Test"; }

        public void init(Tuning t) { follower.setPose(START); }

        public void initLoop(Tuning t) {
            telemetryM.debug("Drive with gamepad 1; the pose is printed live.");
        }

        public void loop(Tuning t) {
            follower.manual(-t.gamepad1.left_stick_y,
                    -t.gamepad1.left_stick_x,
                    -t.gamepad1.right_stick_x);
            telemetryM.debug("x: " + follower.pose().x());
            telemetryM.debug("y: " + follower.pose().y());
            telemetryM.debug("heading (deg): " + Math.toDegrees(follower.pose().heading()));
        }
    }

    static class ForwardTuner implements Tuner {
        public String name() { return "Forward Tuner (push)"; }

        public void init(Tuning t) { follower.setPose(START); }

        public void initLoop(Tuning t) {
            telemetryM.debug("Push the robot forward " + PUSH_DISTANCE + " inches.");
        }

        public void loop(Tuning t) {
            double moved = follower.pose().x() - START.x();
            telemetryM.debug("Distance moved: " + moved);
            telemetryM.debug("Scale your forward ticks-to-inches by "
                    + (moved == 0 ? "n/a" : (PUSH_DISTANCE / moved)));
        }
    }

    static class LateralTuner implements Tuner {
        public String name() { return "Lateral Tuner (push)"; }

        public void init(Tuning t) { follower.setPose(START); }

        public void initLoop(Tuning t) {
            telemetryM.debug("Push the robot to the right " + PUSH_DISTANCE + " inches.");
        }

        public void loop(Tuning t) {
            double moved = follower.pose().y() - START.y();
            telemetryM.debug("Distance moved: " + moved);
            telemetryM.debug("Scale your strafe ticks-to-inches by "
                    + (moved == 0 ? "n/a" : (PUSH_DISTANCE / moved)));
        }
    }

    static class TurnTuner implements Tuner {
        public String name() { return "Turn Tuner (push)"; }

        public void init(Tuning t) { follower.setPose(START); }

        public void initLoop(Tuning t) {
            telemetryM.debug("Turn the robot " + PUSH_ANGLE + " radians.");
        }

        public void loop(Tuning t) {
            double turned = follower.pose().heading() - START.heading();
            telemetryM.debug("Angle turned (rad): " + turned);
            telemetryM.debug("Scale your turn ticks by "
                    + (turned == 0 ? "n/a" : (PUSH_ANGLE / turned)));
        }
    }

    /* ------------------------------------------------------------------ */
    /*  velocity  ->  ForesightConfig.maxAchievable*Velocity               */
    /* ------------------------------------------------------------------ */

    static class ForwardVelocityTuner implements Tuner {
        private final ArrayList<Double> samples = new ArrayList<>();
        private boolean end;

        public String name() { return "Forward Velocity Tuner"; }

        public void init(Tuning t) { follower.setPose(START); }

        public void initLoop(Tuning t) {
            telemetryM.debug("Runs forward at full power for " + VELOCITY_DISTANCE + " in.");
            telemetryM.debug("Clear at least that much space ahead. B stops.");
        }

        public void start(Tuning t) {
            samples.clear();
            for (int i = 0; i < VELOCITY_SAMPLES; i++) samples.add(0.0);
            end = false;
        }

        public void loop(Tuning t) {
            if (!end) {
                if (follower.pose().x() - START.x() > VELOCITY_DISTANCE) {
                    end = true;
                    follower.manual(0, 0, 0);
                } else {
                    follower.manual(1, 0, 0);
                    samples.add(forwardVelocity());
                    samples.remove(0);
                }
                telemetryM.debug("Running... " + (follower.pose().x() - START.x()) + " in");
            } else {
                follower.manual(0, 0, 0);
                double avg = 0;
                for (double v : samples) avg += v;
                avg /= samples.size();
                telemetryM.debug("maxAchievableForwardVelocity = " + avg);
                telemetryM.debug("Press A to record.");
                if (t.gamepad1.a) record("maxAchievableForwardVelocity", avg);
            }
        }
    }

    static class LateralVelocityTuner implements Tuner {
        private final ArrayList<Double> samples = new ArrayList<>();
        private boolean end;

        public String name() { return "Lateral Velocity Tuner"; }

        public void init(Tuning t) { follower.setPose(START); }

        public void initLoop(Tuning t) {
            telemetryM.debug("Strafes at full power for " + VELOCITY_DISTANCE + " in.");
            telemetryM.debug("Clear that much space to the side. B stops.");
        }

        public void start(Tuning t) {
            samples.clear();
            for (int i = 0; i < VELOCITY_SAMPLES; i++) samples.add(0.0);
            end = false;
        }

        public void loop(Tuning t) {
            if (!end) {
                if (Math.abs(follower.pose().y() - START.y()) > VELOCITY_DISTANCE) {
                    end = true;
                    follower.manual(0, 0, 0);
                } else {
                    follower.manual(0, 1, 0);
                    samples.add(Math.abs(lateralVelocity()));
                    samples.remove(0);
                }
                telemetryM.debug("Running... "
                        + Math.abs(follower.pose().y() - START.y()) + " in");
            } else {
                follower.manual(0, 0, 0);
                double avg = 0;
                for (double v : samples) avg += v;
                avg /= samples.size();
                telemetryM.debug("maxAchievableStrafeVelocity = " + avg);
                telemetryM.debug("Press A to record.");
                if (t.gamepad1.a) record("maxAchievableStrafeVelocity", avg);
            }
        }
    }

    /* ------------------------------------------------------------------ */
    /*  zero-power coast-down  ->  ForesightConfig.natural*Deceleration    */
    /*  REQUIRES c.manualBrakeMode.set(false) in MecanumConfig.            */
    /* ------------------------------------------------------------------ */

    abstract static class ZeroPowerTuner implements Tuner {
        final ArrayList<Double> accels = new ArrayList<>();
        double previousVelocity;
        long previousTimeNano;
        boolean stopping, end;

        abstract double velocity();
        abstract void drive();
        abstract String field();

        public void init(Tuning t) { follower.setPose(START); }

        public void initLoop(Tuning t) {
            telemetryM.debug("Accelerates to " + ZPA_VELOCITY + " in/s, then coasts.");
            telemetryM.debug("REQUIRES manualBrakeMode = false in MecanumConfig,");
            telemetryM.debug("otherwise you measure braking, not coasting. B stops.");
        }

        public void start(Tuning t) {
            accels.clear();
            stopping = false;
            end = false;
            drive();
        }

        public void loop(Tuning t) {
            double v = velocity();

            if (!end) {
                if (!stopping) {
                    if (v > ZPA_VELOCITY) {
                        stopping = true;
                        follower.manual(0, 0, 0);
                        previousVelocity = v;
                        previousTimeNano = System.nanoTime();
                    } else {
                        drive();
                    }
                    telemetryM.debug("Accelerating... " + v + " in/s");
                } else {
                    follower.manual(0, 0, 0);
                    long now = System.nanoTime();
                    double dt = (now - previousTimeNano) / 1e9;
                    if (dt > 0) accels.add((v - previousVelocity) / dt);
                    previousVelocity = v;
                    previousTimeNano = now;
                    if (v < 0.1) end = true;
                    telemetryM.debug("Coasting... " + v + " in/s");
                }
            } else {
                follower.manual(0, 0, 0);
                double avg = 0;
                for (double a : accels) avg += a;
                avg /= Math.max(1, accels.size());
                // 3.0 validates these as positive magnitudes; 2.x stored them negative.
                double magnitude = Math.abs(avg);
                telemetryM.debug(field() + " = " + magnitude);
                telemetryM.debug("Press A to record.");
                if (t.gamepad1.a) record(field(), magnitude);
            }
        }
    }

    static class ForwardZeroPowerAccelerationTuner extends ZeroPowerTuner {
        public String name() { return "Forward Zero Power Acceleration Tuner"; }
        double velocity()    { return forwardVelocity(); }
        void drive()         { follower.manual(1, 0, 0); }
        String field()       { return "naturalForwardDeceleration"; }
    }

    static class LateralZeroPowerAccelerationTuner extends ZeroPowerTuner {
        public String name() { return "Lateral Zero Power Acceleration Tuner"; }
        double velocity()    { return Math.abs(lateralVelocity()); }
        void drive()         { follower.manual(0, 1, 0); }
        String field()       { return "naturalStrafeDeceleration"; }
    }

    /* ------------------------------------------------------------------ */
    /*  path-following checks                                              */
    /* ------------------------------------------------------------------ */

    abstract static class PathTest implements Tuner {
        Path forwards, backwards;
        boolean forward = true;

        abstract void build();
        abstract String description();

        public void init(Tuning t) { follower.setPose(START); }

        public void initLoop(Tuning t) { telemetryM.debug(description()); }

        public void start(Tuning t) {
            build();
            forward = true;
            follower.follow(forwards);
        }

        public void loop(Tuning t) {
            if (!follower.isBusy()) {
                forward = !forward;
                follower.follow(forward ? forwards : backwards);
            }
            telemetryM.debug("Outbound leg?: " + forward);
            telemetryM.debug("Distance to endpoint: " + follower.distanceToEndpoint());
        }
    }

    static class LineTest extends PathTest {
        public String name() { return "Line (back and forth)"; }
        String description() { return "Runs a straight line " + LINE_DISTANCE + " in and back."; }

        void build() {
            Pose a = START;
            Pose b = POSE.of(START.x() + LINE_DISTANCE, START.y(), 0);
            forwards  = Paths.line(a, b).linear(a, b);
            backwards = Paths.line(b, a).linear(b, a);
        }
    }

    static class CurveTest extends PathTest {
        public String name() { return "Centripetal / Curve"; }
        String description() {
            return "Runs a curve " + CURVE_DISTANCE + " in forward and the same to the side.";
        }

        void build() {
            double d = Math.abs(CURVE_DISTANCE);
            Pose a = START;
            Pose c = POSE.of(START.x() + d, START.y(), 0);
            Pose b = POSE.of(START.x() + d, START.y() + CURVE_DISTANCE, 0);
            forwards  = Paths.curve(a, c, b).linear(a, b);
            backwards = Paths.curve(b, c, a).linear(b, a);
        }
    }

    static class TriangleTest implements Tuner {
        private Path triangle;

        public String name() { return "Triangle"; }

        public void init(Tuning t) { follower.setPose(START); }

        public void initLoop(Tuning t) {
            telemetryM.debug("Runs a triangle starting at the bottom-middle point.");
        }

        public void start(Tuning t) {
            Pose start = START;
            Pose inter = POSE.of(START.x() + 24, START.y() - 24, 90);
            Pose end   = POSE.of(START.x() + 24, START.y() + 24, 45);
            follower.setPose(start);
            triangle = Paths.path(
                    Paths.line(start, inter).linear(start, inter),
                    Paths.line(inter, end).linear(inter, end),
                    Paths.line(end, start).linear(end, start));
            follower.follow(triangle);
        }

        public void loop(Tuning t) {
            if (!follower.isBusy()) follower.follow(triangle);
            telemetryM.debug("Distance to endpoint: " + follower.distanceToEndpoint());
        }
    }

    static class CircleTest implements Tuner {
        private Path circle;

        public String name() { return "Circle"; }

        public void init(Tuning t) { follower.setPose(START); }

        public void initLoop(Tuning t) {
            telemetryM.debug("Runs a circle of radius " + CIRCLE_RADIUS + ".");
        }

        public void start(Tuning t) {
            double r = CIRCLE_RADIUS, x = START.x(), y = START.y();
            Pose p0 = POSE.of(x, y, 0);
            Pose p1 = POSE.of(x + r, y, 0);
            Pose p2 = POSE.of(x + r, y + r, 0);
            Pose p3 = POSE.of(x + r, y + 2 * r, 0);
            Pose p4 = POSE.of(x, y + 2 * r, 0);
            Pose p5 = POSE.of(x - r, y + 2 * r, 0);
            Pose p6 = POSE.of(x - r, y + r, 0);
            Pose p7 = POSE.of(x - r, y, 0);

            circle = Paths.path(
                    Paths.curve(p0, p1, p2).linear(p0, p2),
                    Paths.curve(p2, p3, p4).linear(p2, p4),
                    Paths.curve(p4, p5, p6).linear(p4, p6),
                    Paths.curve(p6, p7, p0).linear(p6, p0));
            follower.follow(circle);
        }

        public void loop(Tuning t) {
            if (!follower.isBusy()) follower.follow(circle);
            telemetryM.debug("Distance to endpoint: " + follower.distanceToEndpoint());
        }
    }

    /* ------------------------------------------------------------------ */
    /*  Panels field drawing                                               */
    /* ------------------------------------------------------------------ */

    static class Drawing {
        public static final double ROBOT_RADIUS = 9;

//        private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
//        private static final TelemetryManager panelsTelemetry = Panels.getTelemetry();
        private static final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        //        private static final Style robotLook   = new Style("", "#3F51B5", 0.75);
//        private static final Look robotLook = new Look("",0.0, "#3F51B5", 0.75);
        // Parameters: outlineColor, fillColor, outlineWidth, opacity
//        private static final Look robotLook = new Look("", "#3F51B5", 0.0, 0.75);
        private static final Style robotLook = new Style("", "#3F51B5", 0.75);

        //        private static final Style historyLook = new Style("", "#4CAF50", 0.75);
//        private static final Look historyLook = new Look("","#4CAF50", 0.0, 0.75);
//        private static final Style robotLook   = new Style("", "#3F51B5", 0.75);
        private static final Style historyLook = new Style("", "#4CAF50", 0.40);
//        private static final Style robotLook = new Style("", "#4CAF50", 0.75);
//        public static void init() {
////            panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
////            panelsTelemetry.setOffsets(CanvasPreset.PEDRO_PATHING);
//            panelsTelemetry.setOffsets(-72.0, 72.0, CanvasRotation.DEG_270);
//        }

        public static void init() {
            panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        }


        public static void drawDebug(Follower follower) {
            drawRobot(follower.pose(), historyLook);
            sendPacket();
        }

//        public static void drawRobot(Pose pose, Look look) {
//            if (pose == null) return;
//            double x = pose.x(), y = pose.y(), heading = pose.heading();
//            if (Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(heading)) return;
////            panelsField.setStyle(style);
////            panelsField.moveCursor(x, y);
////            panelsField.circle(ROBOT_RADIUS);//
////            panelsField.moveCursor(x, y);
////            panelsField.line(x + Math.cos(heading) * ROBOT_RADIUS,
////                    y + Math.sin(heading) * ROBOT_RADIUS);
////            // 1. Create a Canvas using the Pedro Pathing field preset
////            Canvas canvas = new Canvas(CanvasPreset.PEDRO_PATHING);
//// 2. Define the robot center and heading endpoint
//            Point robotCenter = new Point(x, y);
//            Point headingEnd = new Point(
//                    x + Math.cos(heading) * ROBOT_RADIUS,
//                    y + Math.sin(heading) * ROBOT_RADIUS
//            );
//// 3. Add the robot body circle and heading line with your Look style
//            // Declare the Canvas variable first
//// Now add your shapes to it
//            canvas.add(new Circle(robotCenter, ROBOT_RADIUS).withLook(robotLook));
//            canvas.add(new Line(robotCenter, headingEnd).withLook(robotLook));
//// Send the canvas to Panels
//            Panels.getTelemetry().sendCanvas(canvas);
//        }
public static void drawRobot(Pose pose, Style style) {
    if (pose == null) return;
    double x = pose.x(), y = pose.y(), heading = pose.heading();
    if (Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(heading)) return;

    panelsField.setStyle(style);
    panelsField.moveCursor(x, y);
    panelsField.circle(ROBOT_RADIUS);

    panelsField.moveCursor(x, y);
    panelsField.line(x + Math.cos(heading) * ROBOT_RADIUS,
            y + Math.sin(heading) * ROBOT_RADIUS);
}



        public static void drawRobot(Pose pose) {
            drawRobot(pose, robotLook);
        }

        public static void sendPacket() {
//            panelsField.update();
        }
    }
}

//package org.firstinspires.ftc.teamcode.pedroPathing;
//
//import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.changes;
//import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawOnlyCurrent;
//import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;
//import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
//import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.stopRobot;
//import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.telemetryM;
//
//import com.bylazar.configurables.PanelsConfigurables;
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.configurables.annotations.IgnoreConfigurable;
//import com.bylazar.field.FieldManager;
//import com.bylazar.field.PanelsField;
//import com.bylazar.field.Style;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.math.Pose;
//import com.pedropathing.math.Vector;
//import com.pedropathing.api.Paths;
//import com.pedropathing.api.PoseFactory;
//import com.pedropathing.paths.Path;
//import com.pedropathing.telemetry.SelectableOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import java.util.ArrayList;
//import java.util.List;
//
///**
// * Tuning selection menu (Pedro Pathing 3.x compatible).
// */
//@Configurable
//@TeleOp(name = "Tuning", group = "Pedro Pathing")
//public class Tuning extends SelectableOpMode {
//    public static Follower follower;
//
//    @IgnoreConfigurable
//    static TelemetryManager telemetryM;
//
//    @IgnoreConfigurable
//    static ArrayList<String> changes = new ArrayList<>();
//
//    public Tuning() {
//        super("Select a Tuning OpMode", s -> {
//            s.folder("Localization", l -> {
//                l.add("Localization Test", LocalizationTest::new);
//                l.add("Forward Tuner", ForwardTuner::new);
//                l.add("Lateral Tuner", LateralTuner::new);
//                l.add("Turn Tuner", TurnTuner::new);
//            });
//            s.folder("Automatic", a -> {
//                a.add("Forward Velocity Tuner", ForwardVelocityTuner::new);
//                a.add("Lateral Velocity Tuner", LateralVelocityTuner::new);
//                a.add("Forward Zero Power Acceleration Tuner", ForwardZeroPowerAccelerationTuner::new);
//                a.add("Lateral Zero Power Acceleration Tuner", LateralZeroPowerAccelerationTuner::new);
//            });
//            s.folder("Manual", p -> {
//                p.add("Translational Tuner", TranslationalTuner::new);
//                p.add("Heading Tuner", HeadingTuner::new);
//                p.add("Drive Tuner", DriveTuner::new);
//                p.add("Line Tuner", Line::new);
//                p.add("Centripetal Tuner", CentripetalTuner::new);
//            });
//            s.folder("Tests", p -> {
//                p.add("Line", Line::new);
//                p.add("Triangle", Triangle::new);
//                p.add("Circle", Circle::new);
//            });
//        });
//    }
//
//    @Override
//    public void onSelect() {
//        // ✅ 3.x：使用 Constants.create 替代 createFollower
//        follower = Constants.create(hardwareMap);
//
//        // ✅ 用 PoseFactory 创建 Pose，替代 new Pose()
//        PoseFactory poseFactory = PoseFactory.degrees();
//        follower.setPose(poseFactory.of(0, 0, 0));
//
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//
//        Drawing.init();
//    }
//
//    @Override
//    public void onLog(List<String> lines) {}
//
//    public static void drawOnlyCurrent() {
//        try {
//            Drawing.drawRobot(follower.pose());
//            Drawing.sendPacket();
//        } catch (Exception e) {
//            throw new RuntimeException("Drawing failed " + e);
//        }
//    }
//
//    public static void draw() {
//        Drawing.drawDebug(follower);
//    }
//
//    public static void stopRobot() {
//        follower.startTeleopDrive(true);
//        // ✅ 3.x：使用 manual() 替代 setTeleOpDrive()
//        follower.manual(0, 0, 0);
//    }
//}
//
///**
// * LocalizationTest
// */
//class LocalizationTest extends OpMode {
//    private final PoseFactory poseFactory = PoseFactory.degrees();
//
//    @Override
//    public void init() {
//        follower.setPose(poseFactory.of(72, 72, 0));
//    }
//
//    @Override
//    public void init_loop() {
//        telemetryM.debug("This will print your robot's position to telemetry while "
//                + "allowing robot control through a basic mecanum drive on gamepad 1.");
//        telemetryM.update(telemetry);
//        follower.update();
//        drawOnlyCurrent();
//    }
//
//    @Override
//    public void start() {
//        follower.startTeleopDrive();
//        follower.update();
//    }
//
//    @Override
//    public void loop() {
//        // ✅ 3.x：manual(forward, lateral, turn)
//        follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
//        follower.update();
//
//        telemetryM.debug("x:" + follower.pose().x());
//        telemetryM.debug("y:" + follower.pose().y());
//        telemetryM.debug("heading:" + follower.pose().heading());
//        telemetryM.update(telemetry);
//
//        draw();
//    }
//}
//
///**
// * ForwardTuner
// */
//class ForwardTuner extends OpMode {
//    public static double DISTANCE = 36;
//    private final PoseFactory poseFactory = PoseFactory.degrees();
//
//    @Override
//    public void init() {
//        follower.setPose(poseFactory.of(72, 72, 0));
//        follower.update();
//        drawOnlyCurrent();
//    }
//
//    @Override
//    public void init_loop() {
//        telemetryM.debug("Pull your robot forward " + DISTANCE + " inches. Your forward ticks to inches will be shown on the telemetry.");
//        telemetryM.update(telemetry);
//        drawOnlyCurrent();
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        telemetryM.debug("Distance Moved: " + (follower.pose().x() - 72));
//        telemetryM.debug("The multiplier will display what your forward ticks to inches should be to scale your current distance to " + DISTANCE + " inches.");
//        telemetryM.update(telemetry);
//        draw();
//    }
//}
//
///**
// * LateralTuner
// */
//class LateralTuner extends OpMode {
//    public static double DISTANCE = 36;
//    private final PoseFactory poseFactory = PoseFactory.degrees();
//
//    @Override
//    public void init() {
//        follower.setPose(poseFactory.of(72, 72, 0));
//        follower.update();
//        drawOnlyCurrent();
//    }
//
//    @Override
//    public void init_loop() {
//        telemetryM.debug("Pull your robot to the right " + DISTANCE + " inches. Your strafe ticks to inches will be shown on the telemetry.");
//        telemetryM.update(telemetry);
//        drawOnlyCurrent();
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        telemetryM.debug("Distance Moved: " + (follower.pose().y() - 72));
//        telemetryM.debug("The multiplier will display what your strafe ticks to inches should be to scale your current distance to " + DISTANCE + " inches.");
//        telemetryM.update(telemetry);
//        draw();
//    }
//}
//
///**
// * TurnTuner
// */
//class TurnTuner extends OpMode {
//    public static double ANGLE = 2 * Math.PI;
//    private final PoseFactory poseFactory = PoseFactory.degrees();
//
//    @Override
//    public void init() {
//        follower.setPose(poseFactory.of(72, 72, 0));
//        follower.update();
//        drawOnlyCurrent();
//    }
//
//    @Override
//    public void init_loop() {
//        telemetryM.debug("Turn your robot " + ANGLE + " radians. Your turn ticks to inches will be shown on the telemetry.");
//        telemetryM.update(telemetry);
//        drawOnlyCurrent();
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        telemetryM.debug("Current heading (rad): " + follower.pose().heading());
//        telemetryM.debug("The multiplier will display what your turn ticks to inches should be to scale your current angle to " + ANGLE + " radians.");
//        telemetryM.update(telemetry);
//        draw();
//    }
//}
//
///**
// * ForwardVelocityTuner
// */
//class ForwardVelocityTuner extends OpMode {
//    private final ArrayList<Double> velocities = new ArrayList<>();
//    public static double DISTANCE = 36;
//    public static double RECORD_NUMBER = 10;
//    private final PoseFactory poseFactory = PoseFactory.degrees();
//    private boolean end;
//
//    @Override
//    public void init() {
//        follower.setPose(poseFactory.of(72, 72, 0));
//    }
//
//    @Override
//    public void init_loop() {
//        telemetryM.debug("The robot will run at 1 power until it reaches " + DISTANCE + " inches forward.");
//        telemetryM.debug("Press B on game pad 1 to stop.");
//        telemetryM.update(telemetry);
//        follower.update();
//        drawOnlyCurrent();
//    }
//
//    @Override
//    public void start() {
//        for (int i = 0; i < RECORD_NUMBER; i++) velocities.add(0.0);
//        follower.startTeleopDrive(true);
//        follower.update();
//        end = false;
//    }
//
//    @Override
//    public void loop() {
//        if (gamepad1.bWasPressed()) {
//            stopRobot();
//            requestOpModeStop();
//        }
//
//        follower.update();
//        draw();
//
//        if (!end) {
//            if (Math.abs(follower.pose().x()) > (DISTANCE + 72)) {
//                end = true;
//                stopRobot();
//            } else {
//                follower.manual(1, 0, 0); // ✅ 使用 manual
//                // ⚠️ Velocity 类可能不再提供 getXComponent()，暂时使用标量速度或注释
//                // double currentVelocity = Math.abs(follower.velocity().getXComponent());
//                // velocities.add(currentVelocity);
//                // velocities.remove(0);
//            }
//        } else {
//            stopRobot();
//            double average = 0;
//            for (double velocity : velocities) average += velocity;
//            average /= velocities.size();
//            telemetryM.debug("Forward Velocity: " + average);
//            telemetryM.debug("Press A to set the Forward Velocity temporarily.");
//            telemetryM.update(telemetry);
//
//            if (gamepad1.aWasPressed()) {
//                // follower.setXVelocity(average); // 3.x 中可能已变更
//                changes.add("XMovement: " + average);
//            }
//        }
//    }
//}
//
///**
// * LateralVelocityTuner
// */
//class LateralVelocityTuner extends OpMode {
//    private final ArrayList<Double> velocities = new ArrayList<>();
//    public static double DISTANCE = 36;
//    public static double RECORD_NUMBER = 10;
//    private final PoseFactory poseFactory = PoseFactory.degrees();
//    private boolean end;
//
//    @Override
//    public void init() {
//        follower.setPose(poseFactory.of(72, 72, 0));
//    }
//
//    @Override
//    public void init_loop() {
//        telemetryM.debug("The robot will run at 1 power until it reaches " + DISTANCE + " inches to the left.");
//        telemetryM.debug("Press B on Gamepad 1 to stop.");
//        telemetryM.update(telemetry);
//        follower.update();
//        drawOnlyCurrent();
//    }
//
//    @Override
//    public void start() {
//        for (int i = 0; i < RECORD_NUMBER; i++) velocities.add(0.0);
//        follower.startTeleopDrive(true);
//        follower.update();
//    }
//
//    @Override
//    public void loop() {
//        if (gamepad1.bWasPressed()) {
//            stopRobot();
//            requestOpModeStop();
//        }
//
//        follower.update();
//        draw();
//
//        if (!end) {
//            if (Math.abs(follower.pose().y()) > (DISTANCE + 72)) {
//                end = true;
//                stopRobot();
//            } else {
//                follower.manual(0, 1, 0); // ✅ 使用 manual
//                // ⚠️ Velocity 类的 dot() 可能已变更，暂时注释
//                // double currentVelocity = Math.abs(follower.velocity().dot(new Vector(1, Math.PI / 2)));
//                // velocities.add(currentVelocity);
//                // velocities.remove(0);
//            }
//        } else {
//            stopRobot();
//            double average = 0;
//            for (double velocity : velocities) average += velocity;
//            average /= velocities.size();
//
//            telemetryM.debug("Strafe Velocity: " + average);
//            telemetryM.debug("Press A to set the Lateral Velocity temporarily.");
//            telemetryM.update(telemetry);
//
//            if (gamepad1.aWasPressed()) {
//                // follower.setYVelocity(average); // 3.x 中可能已变更
//                changes.add("YMovement: " + average);
//            }
//        }
//    }
//}
//
///**
// * ForwardZeroPowerAccelerationTuner
// */
//class ForwardZeroPowerAccelerationTuner extends OpMode {
//    private final ArrayList<Double> accelerations = new ArrayList<>();
//    public static double VELOCITY = 30;
//    private final PoseFactory poseFactory = PoseFactory.degrees();
//    private double previousVelocity;
//    private long previousTimeNano;
//    private boolean stopping;
//    private boolean end;
//
//    @Override
//    public void init() {
//        follower.setPose(poseFactory.of(72, 72, 0));
//    }
//
//    @Override
//    public void init_loop() {
//        telemetryM.debug("The robot will run forward until it reaches " + VELOCITY + " inches per second.");
//        telemetryM.debug("Press B on Gamepad 1 to stop.");
//        telemetryM.update(telemetry);
//        follower.update();
//        drawOnlyCurrent();
//    }
//
//    @Override
//    public void start() {
//        follower.startTeleopDrive(false);
//        follower.update();
//        follower.manual(1, 0, 0); // ✅ 使用 manual
//    }
//
//    @Override
//    public void loop() {
//        if (gamepad1.bWasPressed()) {
//            stopRobot();
//            requestOpModeStop();
//        }
//
//        follower.update();
//        draw();
//
//        // ⚠️ Velocity 类的 dot() 可能已变更，此处逻辑暂时简化或注释
//        Vector heading = new Vector(1.0, follower.pose().heading());
//        // if (!end) {
//        //     if (!stopping) {
//        //         if (follower.velocity().dot(heading) > VELOCITY) {
//        //             ...
//        //         }
//        //     } else {
//        //         ...
//        //     }
//        // } else {
//        //     ...
//        // }
//        telemetryM.debug("Forward Zero Power Acceleration Tuner (requires manual velocity handling)");
//        telemetryM.update(telemetry);
//    }
//}
//
///**
// * LateralZeroPowerAccelerationTuner
// */
//class LateralZeroPowerAccelerationTuner extends OpMode {
//    private final ArrayList<Double> accelerations = new ArrayList<>();
//    public static double VELOCITY = 30;
//    private final PoseFactory poseFactory = PoseFactory.degrees();
//    private double previousVelocity;
//    private long previousTimeNano;
//    private boolean stopping;
//    private boolean end;
//
//    @Override
//    public void init() {
//        follower.setPose(poseFactory.of(72, 72, 0));
//    }
//
//    @Override
//    public void init_loop() {
//        telemetryM.debug("The robot will run to the left until it reaches " + VELOCITY + " inches per second.");
//        telemetryM.debug("Press B on game pad 1 to stop.");
//        telemetryM.update(telemetry);
//        follower.update();
//        drawOnlyCurrent();
//    }
//
//    @Override
//    public void start() {
//        follower.startTeleopDrive(false);
//        follower.update();
//        follower.manual(0, 1, 0); // ✅ 使用 manual
//    }
//
//    @Override
//    public void loop() {
//        if (gamepad1.bWasPressed()) {
//            stopRobot();
//            requestOpModeStop();
//        }
//
//        follower.update();
//        draw();
//
//        // ⚠️ Velocity 类的 dot() 可能已变更，逻辑暂时简化
//        telemetryM.debug("Lateral Zero Power Acceleration Tuner (requires manual velocity handling)");
//        telemetryM.update(telemetry);
//    }
//}
//
///**
// * TranslationalTuner
// */
//class TranslationalTuner extends OpMode {
//    public static double DISTANCE = 40;
//    private final PoseFactory poseFactory = PoseFactory.degrees();
//    private boolean forward = true;
//    private Path forwards;
//    private Path backwards;
//
//    @Override
//    public void init() {
//        follower.setPose(poseFactory.of(72, 72, 0));
//    }
//
//    @Override
//    public void init_loop() {
//        telemetryM.debug("This will activate the translational PIDF(s)");
//        telemetryM.update(telemetry);
//        follower.update();
//        drawOnlyCurrent();
//    }
//
//    @Override
//    public void start() {
//        // follower.deactivateAllPIDFs(); // 3.x 中可能已变更，暂时注释
//        // follower.activateTranslational(); // 3.x 中可能已变更，暂时注释
//        forwards = Paths.line(poseFactory.of(72, 72, 0), poseFactory.of(DISTANCE + 72, 72, 0))
//                .linear(poseFactory.of(72, 72, 0), poseFactory.of(DISTANCE + 72, 72, 0));
//        backwards = Paths.line(poseFactory.of(DISTANCE + 72, 72, 0), poseFactory.of(72, 72, 0))
//                .linear(poseFactory.of(DISTANCE + 72, 72, 0), poseFactory.of(72, 72, 0));
//        follower.follow(forwards);
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        draw();
//
//        if (!follower.isBusy()) {
//            if (forward) {
//                forward = false;
//                follower.follow(backwards);
//            } else {
//                forward = true;
//                follower.follow(forwards);
//            }
//        }
//
//        telemetryM.debug("Push the robot laterally to test the Translational PIDF(s).");
//        telemetryM.update(telemetry);
//    }
//}
//
///**
// * HeadingTuner
// */
//class HeadingTuner extends OpMode {
//    public static double DISTANCE = 40;
//    private final PoseFactory poseFactory = PoseFactory.degrees();
//    private boolean forward = true;
//    private Path forwards;
//    private Path backwards;
//
//    @Override
//    public void init() {
//        follower.setPose(poseFactory.of(72, 72, 0));
//    }
//
//    @Override
//    public void init_loop() {
//        telemetryM.debug("This will activate the heading PIDF(s).");
//        telemetryM.update(telemetry);
//        follower.update();
//        drawOnlyCurrent();
//    }
//
//    @Override
//    public void start() {
//        // follower.deactivateAllPIDFs(); // 暂时注释
//        // follower.activateHeading(); // 暂时注释
//        forwards = Paths.line(poseFactory.of(72, 72, 0), poseFactory.of(DISTANCE + 72, 72, 0))
//                .linear(poseFactory.of(72, 72, 0), poseFactory.of(DISTANCE + 72, 72, 0));
//        backwards = Paths.line(poseFactory.of(DISTANCE + 72, 72, 0), poseFactory.of(72, 72, 0))
//                .linear(poseFactory.of(DISTANCE + 72, 72, 0), poseFactory.of(72, 72, 0));
//        follower.follow(forwards);
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        draw();
//
//        if (!follower.isBusy()) {
//            if (forward) {
//                forward = false;
//                follower.follow(backwards);
//            } else {
//                forward = true;
//                follower.follow(forwards);
//            }
//        }
//
//        telemetryM.debug("Turn the robot manually to test the Heading PIDF(s).");
//        telemetryM.update(telemetry);
//    }
//}
//
///**
// * DriveTuner
// */
//class DriveTuner extends OpMode {
//    public static double DISTANCE = 40;
//    private final PoseFactory poseFactory = PoseFactory.degrees();
//    private boolean forward = true;
//    private Path forwards;
//    private Path backwards;
//
//    @Override
//    public void init() {
//        follower.setPose(poseFactory.of(72, 72, 0));
//    }
//
//    @Override
//    public void init_loop() {
//        telemetryM.debug("This will run the robot in a straight line going " + DISTANCE + " inches forward.");
//        telemetryM.update(telemetry);
//        follower.update();
//        drawOnlyCurrent();
//    }
//
//    @Override
//    public void start() {
//        // follower.deactivateAllPIDFs(); // 暂时注释
//        // follower.activateDrive(); // 暂时注释
//        forwards = Paths.line(poseFactory.of(72, 72, 0), poseFactory.of(DISTANCE + 72, 72, 0))
//                .linear(poseFactory.of(72, 72, 0), poseFactory.of(DISTANCE + 72, 72, 0));
//        backwards = Paths.line(poseFactory.of(DISTANCE + 72, 72, 0), poseFactory.of(72, 72, 0))
//                .linear(poseFactory.of(DISTANCE + 72, 72, 0), poseFactory.of(72, 72, 0));
//        follower.follow(forwards);
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        draw();
//
//        if (!follower.isBusy()) {
//            if (forward) {
//                forward = false;
//                follower.follow(backwards);
//            } else {
//                forward = true;
//                follower.follow(forwards);
//            }
//        }
//
//        telemetryM.debug("Driving forward?: " + forward);
//        telemetryM.update(telemetry);
//    }
//}
//
///**
// * Line
// */
//class Line extends OpMode {
//    public static double DISTANCE = 40;
//    private final PoseFactory poseFactory = PoseFactory.degrees();
//    private boolean forward = true;
//    private Path forwards;
//    private Path backwards;
//
//    @Override
//    public void init() {
//        follower.setPose(poseFactory.of(72, 72, 0));
//    }
//
//    @Override
//    public void init_loop() {
//        telemetryM.debug("This will activate all the PIDF(s)");
//        telemetryM.update(telemetry);
//        follower.update();
//        drawOnlyCurrent();
//    }
//
//    @Override
//    public void start() {
//        // follower.activateAllPIDFs(); // 暂时注释
//        forwards = Paths.line(poseFactory.of(72, 72, 0), poseFactory.of(DISTANCE + 72, 72, 0))
//                .linear(poseFactory.of(72, 72, 0), poseFactory.of(DISTANCE + 72, 72, 0));
//        backwards = Paths.line(poseFactory.of(DISTANCE + 72, 72, 0), poseFactory.of(72, 72, 0))
//                .linear(poseFactory.of(DISTANCE + 72, 72, 0), poseFactory.of(72, 72, 0));
//        follower.follow(forwards);
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        draw();
//
//        if (!follower.isBusy()) {
//            if (forward) {
//                forward = false;
//                follower.follow(backwards);
//            } else {
//                forward = true;
//                follower.follow(forwards);
//            }
//        }
//
//        telemetryM.debug("Driving Forward?: " + forward);
//        telemetryM.update(telemetry);
//    }
//}
//
///**
// * CentripetalTuner
// */
//class CentripetalTuner extends OpMode {
//    public static double DISTANCE = 20;
//    private final PoseFactory poseFactory = PoseFactory.degrees();
//    private boolean forward = true;
//    private Path forwards;
//    private Path backwards;
//
//    @Override
//    public void init() {
//        follower.setPose(poseFactory.of(72, 72, 0));
//    }
//
//    @Override
//    public void init_loop() {
//        telemetryM.debug("This will run the robot in a curve going " + DISTANCE + " inches to the left and the same number of inches forward.");
//        telemetryM.update(telemetry);
//        follower.update();
//        drawOnlyCurrent();
//    }
//
//    @Override
//    public void start() {
//        // follower.activateAllPIDFs(); // 暂时注释
//        forwards = Paths.curve(poseFactory.of(72, 72, 0), poseFactory.of(Math.abs(DISTANCE) + 72, 72, 0), poseFactory.of(Math.abs(DISTANCE) + 72, DISTANCE + 72, 0))
//                .linear(poseFactory.of(72, 72, 0), poseFactory.of(Math.abs(DISTANCE) + 72, DISTANCE + 72, 0));
//        backwards = Paths.curve(poseFactory.of(Math.abs(DISTANCE) + 72, DISTANCE + 72, 0), poseFactory.of(Math.abs(DISTANCE) + 72, 72, 0), poseFactory.of(72, 72, 0))
//                .linear(poseFactory.of(Math.abs(DISTANCE) + 72, DISTANCE + 72, 0), poseFactory.of(72, 72, 0));
//        follower.follow(forwards);
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        draw();
//
//        if (!follower.isBusy()) {
//            if (forward) {
//                forward = false;
//                follower.follow(backwards);
//            } else {
//                forward = true;
//                follower.follow(forwards);
//            }
//        }
//
//        telemetryM.debug("Driving away from the origin along the curve?: " + forward);
//        telemetryM.update(telemetry);
//    }
//}
//
///**
// * Triangle
// */
//class Triangle extends OpMode {
//    private final PoseFactory poseFactory = PoseFactory.degrees();
//    private final Pose startPose = poseFactory.of(72, 72, 0);
//    private final Pose interPose = poseFactory.of(24 + 72, -24 + 72, 90);
//    private final Pose endPose = poseFactory.of(24 + 72, 24 + 72, 45);
//    private Path triangle;
//
//    @Override
//    public void init() {
//        follower.setPose(poseFactory.of(72, 72, 0));
//    }
//
//    @Override
//    public void init_loop() {
//        telemetryM.debug("This will run in a roughly triangular shape, starting on the bottom-middle point.");
//        telemetryM.update(telemetry);
//        follower.update();
//        drawOnlyCurrent();
//    }
//
//    @Override
//    public void start() {
//        follower.setPose(startPose);
//        triangle = Paths.path(
//                Paths.line(startPose, interPose).linear(startPose, interPose),
//                Paths.line(interPose, endPose).linear(interPose, endPose),
//                Paths.line(endPose, startPose).linear(endPose, startPose)
//        );
//        follower.follow(triangle);
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        draw();
//
//        if (!follower.isBusy()) {
//            follower.follow(triangle);
//        }
//    }
//}
//
///**
// * Circle
// */
//class Circle extends OpMode {
//    public static double RADIUS = 10;
//    private final PoseFactory poseFactory = PoseFactory.degrees();
//    private Path circle;
//
//    @Override
//    public void init() {
//        follower.setPose(poseFactory.of(72, 72, 0));
//    }
//
//    @Override
//    public void init_loop() {
//        telemetryM.debug("This will run in a roughly circular shape of radius " + RADIUS + ".");
//        telemetryM.update(telemetry);
//        follower.update();
//        drawOnlyCurrent();
//    }
//
//    @Override
//    public void start() {
//        Pose center = poseFactory.of(72, 72, 0);
//        circle = Paths.path(
//                Paths.curve(center, poseFactory.of(RADIUS + 72, 72, 0), poseFactory.of(RADIUS + 72, RADIUS + 72, 0))
//                        .linear(center, poseFactory.of(RADIUS + 72, RADIUS + 72, 0)),
//                Paths.curve(poseFactory.of(RADIUS + 72, RADIUS + 72, 0), poseFactory.of(RADIUS + 72, (2 * RADIUS) + 72, 0), poseFactory.of(72, (2 * RADIUS) + 72, 0))
//                        .linear(poseFactory.of(RADIUS + 72, RADIUS + 72, 0), poseFactory.of(72, (2 * RADIUS) + 72, 0)),
//                Paths.curve(poseFactory.of(72, (2 * RADIUS) + 72, 0), poseFactory.of(-RADIUS + 72, (2 * RADIUS) + 72, 0), poseFactory.of(-RADIUS + 72, RADIUS + 72, 0))
//                        .linear(poseFactory.of(72, (2 * RADIUS) + 72, 0), poseFactory.of(-RADIUS + 72, RADIUS + 72, 0)),
//                Paths.curve(poseFactory.of(-RADIUS + 72, RADIUS + 72, 0), poseFactory.of(-RADIUS + 72, 72, 0), poseFactory.of(72, 72, 0))
//                        .linear(poseFactory.of(-RADIUS + 72, RADIUS + 72, 0), poseFactory.of(72, 72, 0))
//        );
//        follower.follow(circle);
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        draw();
//
//        if (!follower.isBusy()) {
//            follower.follow(circle);
//        }
//    }
//}
//
///**
// * Drawing —— 精简版，仅绘制当前机器人位姿
// */
//class Drawing {
//    public static final double ROBOT_RADIUS = 9;
//    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
//
//    private static final Style robotLook = new Style("", "#3F51B5", 0.75);
//    private static final Style historyLook = new Style("", "#4CAF50", 0.75);
//
//    public static void init() {
//        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
//    }
//
//    public static void drawDebug(Follower follower) {
//        drawRobot(follower.pose(), historyLook);
//        sendPacket();
//    }
//
//    public static void drawRobot(Pose pose, Style style) {
//        if (pose == null) return;
//        double x = pose.x();
//        double y = pose.y();
//        double heading = pose.heading();
//        if (Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(heading)) return;
//
//        panelsField.setStyle(style);
//        panelsField.moveCursor(x, y);
//        panelsField.circle(ROBOT_RADIUS);
//
//        double hx = x + Math.cos(heading) * ROBOT_RADIUS;
//        double hy = y + Math.sin(heading) * ROBOT_RADIUS;
//        panelsField.moveCursor(x, y);
//        panelsField.line(hx, hy);
//    }
//
//    public static void drawRobot(Pose pose) {
//        drawRobot(pose, robotLook);
//    }
//
//    public static void sendPacket() {
//        panelsField.update();
//    }
//}


//package org.firstinspires.ftc.teamcode.pedroPathing;
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.changes;
//import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawOnlyCurrent;
//import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;
//import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
//import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.stopRobot;
//import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.telemetryM;
//
//import com.bylazar.configurables.PanelsConfigurables;
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.configurables.annotations.IgnoreConfigurable;
//import com.bylazar.field.FieldManager;
//import com.bylazar.field.PanelsField;
//import com.bylazar.field.Style;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.ErrorCalculator;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.*;
//import com.pedropathing.math.*;
//import com.pedropathing.paths.*;
//import com.pedropathing.telemetry.SelectableOpMode;
//import com.pedropathing.util.*;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import java.util.ArrayList;
//import java.util.List;
//
///**
// * This is the Tuning class. It contains a selection menu for various tuning OpModes.
// *
// * @author Baron Henderson - 20077 The Indubitables
// * @version 1.0, 6/26/2025
// */
//@Configurable
//@TeleOp(name = "Tuning", group = "Pedro Pathing")
//public class Tuning extends SelectableOpMode {
//    public static Follower follower;
//
//    @IgnoreConfigurable
//    static PoseHistory poseHistory;
//
//    @IgnoreConfigurable
//    static TelemetryManager telemetryM;
//
//    @IgnoreConfigurable
//    static ArrayList<String> changes = new ArrayList<>();
//
//    public Tuning() {
//        super("Select a Tuning OpMode", s -> {
//            s.folder("Localization", l -> {
//                l.add("Localization Test", LocalizationTest::new);
//                l.add("Forward Tuner", ForwardTuner::new);
//                l.add("Lateral Tuner", LateralTuner::new);
//                l.add("Turn Tuner", TurnTuner::new);
//            });
//            s.folder("Automatic", a -> {
//                a.add("Forward Velocity Tuner", ForwardVelocityTuner::new);
//                a.add("Lateral Velocity Tuner", LateralVelocityTuner::new);
//                a.add("Forward Zero Power Acceleration Tuner", ForwardZeroPowerAccelerationTuner::new);
//                a.add("Lateral Zero Power Acceleration Tuner", LateralZeroPowerAccelerationTuner::new);
//            });
//            s.folder("Manual", p -> {
//                p.add("Translational Tuner", TranslationalTuner::new);
//                p.add("Heading Tuner", HeadingTuner::new);
//                p.add("Drive Tuner", DriveTuner::new);
//                p.add("Line Tuner", Line::new);
//                p.add("Centripetal Tuner", CentripetalTuner::new);
//            });
//            s.folder("Tests", p -> {
//                p.add("Line", Line::new);
//                p.add("Triangle", Triangle::new);
//                p.add("Circle", Circle::new);
//            });
//        });
//    }
//
//    @Override
//    public void onSelect() {
//        if (follower == null) {
//            follower = Constants.createFollower(hardwareMap);
//            PanelsConfigurables.INSTANCE.refreshClass(this);
//        } else {
//            follower = Constants.createFollower(hardwareMap);
//        }
//
//        follower.setStartingPose(new Pose());
//
//        poseHistory = follower.getPoseHistory();
//
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//
//        Drawing.init();
//    }
//
//    @Override
//    public void onLog(List<String> lines) {}
//
//    public static void drawOnlyCurrent() {
//        try {
//            Drawing.drawRobot(follower.getPose());
//            Drawing.sendPacket();
//        } catch (Exception e) {
//            throw new RuntimeException("Drawing failed " + e);
//        }
//    }
//
//    public static void draw() {
//        Drawing.drawDebug(follower);
//    }
//
//    /** This creates a full stop of the robot by setting the drive motors to run at 0 power. */
//    public static void stopRobot() {
//        follower.startTeleopDrive(true);
//        follower.setTeleOpDrive(0,0,0,true);
//    }
//}
//
///**
// * This is the LocalizationTest OpMode. This is basically just a simple mecanum drive attached to a
// * PoseUpdater. The OpMode will print out the robot's pose to telemetry as well as draw the robot.
// * You should use this to check the robot's localization.
// *
// * @author Anyi Lin - 10158 Scott's Bots
// * @author Baron Henderson - 20077 The Indubitables
// * @version 1.0, 5/6/2024
// */
//class LocalizationTest extends OpMode {
//    @Override
//    public void init() {
//        follower.setStartingPose(new Pose(72,72));
//    }
//
//    /** This initializes the PoseUpdater, the mecanum drive motors, and the Panels telemetry. */
//    @Override
//    public void init_loop() {
//        telemetryM.debug("This will print your robot's position to telemetry while "
//                + "allowing robot control through a basic mecanum drive on gamepad 1.");
//        telemetryM.update(telemetry);
//        follower.update();
//        drawOnlyCurrent();
//    }
//
//    @Override
//    public void start() {
//        follower.startTeleopDrive();
//        follower.update();
//    }
//
//    /**
//     * This updates the robot's pose estimate, the simple mecanum drive, and updates the
//     * Panels telemetry with the robot's position as well as draws the robot's position.
//     */
//    @Override
//    public void loop() {
//        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
//        follower.update();
//
//        telemetryM.debug("x:" + follower.getPose().getX());
//        telemetryM.debug("y:" + follower.getPose().getY());
//        telemetryM.debug("heading:" + follower.getPose().getHeading());
//        telemetryM.debug("total heading:" + follower.getTotalHeading());
//        telemetryM.update(telemetry);
//
//        draw();
//    }
//}
//
///**
// * This is the ForwardTuner OpMode. This tracks the forward movement of the robot and displays the
// * necessary ticks to inches multiplier. This displayed multiplier is what's necessary to scale the
// * robot's current distance in ticks to the specified distance in inches. So, to use this, run the
// * tuner, then pull/push the robot to the specified distance using a ruler on the ground. When you're
// * at the end of the distance, record the ticks to inches multiplier. Feel free to run multiple trials
// * and average the results. Then, input the multiplier into the forward ticks to inches in your
// * localizer of choice.
// *
// * @author Anyi Lin - 10158 Scott's Bots
// * @author Baron Henderson - 20077 The Indubitables
// * @version 1.0, 5/6/2024
// */
//class ForwardTuner extends OpMode {
//    public static double DISTANCE = 36;
//
//    @Override
//    public void init() {
//        follower.setStartingPose(new Pose(72,72));
//        follower.update();
//        drawOnlyCurrent();
//    }
//
//    /** This initializes the PoseUpdater as well as the Panels telemetry. */
//    @Override
//    public void init_loop() {
//        telemetryM.debug("Pull your robot forward " + DISTANCE + " inches. Your forward ticks to inches will be shown on the telemetry.");
//        telemetryM.update(telemetry);
//        drawOnlyCurrent();
//    }
//
//    /**
//     * This updates the robot's pose estimate, and updates the Panels telemetry with the
//     * calculated multiplier and draws the robot.
//     */
//    @Override
//    public void loop() {
//        follower.update();
//
//        telemetryM.debug("Distance Moved: " + (follower.getPose().getX() - 72));
//        telemetryM.debug("The multiplier will display what your forward ticks to inches should be to scale your current distance to " + DISTANCE + " inches.");
//        telemetryM.debug("Multiplier: " + (DISTANCE / ((follower.getPose().getX() - 72) / follower.getPoseTracker().getLocalizer().getForwardMultiplier())));
//        telemetryM.update(telemetry);
//
//        draw();
//    }
//}
//
///**
// * This is the LateralTuner OpMode. This tracks the strafe movement of the robot and displays the
// * necessary ticks to inches multiplier. This displayed multiplier is what's necessary to scale the
// * robot's current distance in ticks to the specified distance in inches. So, to use this, run the
// * tuner, then pull/push the robot to the specified distance using a ruler on the ground. When you're
// * at the end of the distance, record the ticks to inches multiplier. Feel free to run multiple trials
// * and average the results. Then, input the multiplier into the strafe ticks to inches in your
// * localizer of choice.
// *
// * @author Anyi Lin - 10158 Scott's Bots
// * @author Baron Henderson - 20077 The Indubitables
// * @version 2.0, 6/26/2025
// */
//class LateralTuner extends OpMode {
//    public static double DISTANCE =36;
//
//    @Override
//    public void init() {
//        follower.setStartingPose(new Pose(72,72));
//        follower.update();
//        drawOnlyCurrent();
//    }
//
//    /** This initializes the PoseUpdater as well as the Panels telemetry. */
//    @Override
//    public void init_loop() {
//        telemetryM.debug("Pull your robot to the right " + DISTANCE + " inches. Your strafe ticks to inches will be shown on the telemetry.");
//        telemetryM.update(telemetry);
//        drawOnlyCurrent();
//    }
//
//    /**
//     * This updates the robot's pose estimate, and updates the Panels telemetry with the
//     * calculated multiplier and draws the robot.
//     */
//    @Override
//    public void loop() {
//        follower.update();
//
//        telemetryM.debug("Distance Moved: " + (follower.getPose().getY() - 72));
//        telemetryM.debug("The multiplier will display what your strafe ticks to inches should be to scale your current distance to " + DISTANCE + " inches.");
//        telemetryM.debug("Multiplier: " + (DISTANCE / ((follower.getPose().getY() - 72) / follower.getPoseTracker().getLocalizer().getLateralMultiplier())));
//        telemetryM.update(telemetry);
//
//        draw();
//    }
//}
//
///**
// * This is the TurnTuner OpMode. This tracks the turning movement of the robot and displays the
// * necessary ticks to inches multiplier. This displayed multiplier is what's necessary to scale the
// * robot's current angle in ticks to the specified angle in radians. So, to use this, run the
// * tuner, then pull/push the robot to the specified angle using a protractor or lines on the ground.
// * When you're at the end of the angle, record the ticks to inches multiplier. Feel free to run
// * multiple trials and average the results. Then, input the multiplier into the turning ticks to
// * radians in your localizer of choice.
// *
// * @author Anyi Lin - 10158 Scott's Bots
// * @author Baron Henderson - 20077 The Indubitables
// * @version 1.0, 5/6/2024
// */
//class TurnTuner extends OpMode {
//    public static double ANGLE = 2 * Math.PI;
//
//    @Override
//    public void init() {
//        follower.setStartingPose(new Pose(72,72));
//        follower.update();
//        drawOnlyCurrent();
//    }
//
//    /** This initializes the PoseUpdater as well as the Panels telemetry. */
//    @Override
//    public void init_loop() {
//        telemetryM.debug("Turn your robot " + ANGLE + " radians. Your turn ticks to inches will be shown on the telemetry.");
//        telemetryM.update(telemetry);
//
//        drawOnlyCurrent();
//    }
//
//    /**
//     * This updates the robot's pose estimate, and updates the Panels telemetry with the
//     * calculated multiplier and draws the robot.
//     */
//    @Override
//    public void loop() {
//        follower.update();
//
//        telemetryM.debug("Total Angle: " + follower.getTotalHeading());
//        telemetryM.debug("The multiplier will display what your turn ticks to inches should be to scale your current angle to " + ANGLE + " radians.");
//        telemetryM.debug("Multiplier: " + (ANGLE / (follower.getTotalHeading() / follower.getPoseTracker().getLocalizer().getTurningMultiplier())));
//        telemetryM.update(telemetry);
//
//        draw();
//    }
//}
//
///**
// * This is the ForwardVelocityTuner autonomous follower OpMode. This runs the robot forwards at max
// * power until it reaches some specified distance. It records the most recent velocities, and on
// * reaching the end of the distance, it averages them and prints out the velocity obtained. It is
// * recommended to run this multiple times on a full battery to get the best results. What this does
// * is, when paired with StrafeVelocityTuner, allows FollowerConstants to create a Vector that
// * empirically represents the direction your mecanum wheels actually prefer to go in, allowing for
// * more accurate following.
// *
// * @author Anyi Lin - 10158 Scott's Bots
// * @author Aaron Yang - 10158 Scott's Bots
// * @author Harrison Womack - 10158 Scott's Bots
// * @author Baron Henderson - 20077 The Indubitables
// * @version 1.0, 3/13/2024
// */
//class ForwardVelocityTuner extends OpMode {
//    private final ArrayList<Double> velocities = new ArrayList<>();
//    public static double DISTANCE = 36;
//    public static double RECORD_NUMBER = 10;
//
//    private boolean end;
//
//    @Override
//    public void init() {
//        follower.setStartingPose(new Pose(72, 72));
//    }
//
//    /** This initializes the drive motors as well as the cache of velocities and the Panels telemetry. */
//    @Override
//    public void init_loop() {
//        telemetryM.debug("The robot will run at 1 power until it reaches " + DISTANCE + " inches forward.");
//        telemetryM.debug("Make sure you have enough room, since the robot has inertia after cutting power.");
//        telemetryM.debug("After running the distance, the robot will cut power from the drivetrain and display the forward velocity.");
//        telemetryM.debug("Press B on game pad 1 to stop.");
//        telemetryM.debug("pose", follower.getPose());
//        telemetryM.update(telemetry);
//        follower.update();
//        drawOnlyCurrent();
//    }
//
//    /** This starts the OpMode by setting the drive motors to run forward at full power. */
//    @Override
//    public void start() {
//        for (int i = 0; i < RECORD_NUMBER; i++) {
//            velocities.add(0.0);
//        }
//        follower.startTeleopDrive(true);
//        follower.update();
//        end = false;
//    }
//
//    /**
//     * This runs the OpMode. At any point during the running of the OpMode, pressing B on
//     * game pad 1 will stop the OpMode. This continuously records the RECORD_NUMBER most recent
//     * velocities, and when the robot has run forward enough, these last velocities recorded are
//     * averaged and printed.
//     */
//    @Override
//    public void loop() {
//        if (gamepad1.bWasPressed()) {
//            stopRobot();
//            requestOpModeStop();
//        }
//
//        follower.update();
//        draw();
//
//
//        if (!end) {
//            if (Math.abs(follower.getPose().getX()) > (DISTANCE + 72)) {
//                end = true;
//                stopRobot();
//            } else {
//                follower.setTeleOpDrive(1,0,0,true);
//                //double currentVelocity = Math.abs(follower.getVelocity().getXComponent());
//                double currentVelocity = Math.abs(follower.poseTracker.getLocalizer().getVelocity().getX());
//                velocities.add(currentVelocity);
//                velocities.remove(0);
//            }
//        } else {
//            stopRobot();
//            double average = 0;
//            for (double velocity : velocities) {
//                average += velocity;
//            }
//            average /= velocities.size();
//            telemetryM.debug("Forward Velocity: " + average);
//            telemetryM.debug("\n");
//            telemetryM.debug("Press A to set the Forward Velocity temporarily (while robot remains on).");
//
//            for (int i = 0; i < velocities.size(); i++) {
//                telemetry.addData(String.valueOf(i), velocities.get(i));
//            }
//
//            telemetryM.update(telemetry);
//            telemetry.update();
//
//            if (gamepad1.aWasPressed()) {
//                follower.setXVelocity(average);
//                String message = "XMovement: " + average;
//                changes.add(message);
//            }
//        }
//    }
//}
//
///**
// * This is the StrafeVelocityTuner autonomous follower OpMode. This runs the robot left at max
// * power until it reaches some specified distance. It records the most recent velocities, and on
// * reaching the end of the distance, it averages them and prints out the velocity obtained. It is
// * recommended to run this multiple times on a full battery to get the best results. What this does
// * is, when paired with ForwardVelocityTuner, allows FollowerConstants to create a Vector that
// * empirically represents the direction your mecanum wheels actually prefer to go in, allowing for
// * more accurate following.
// *
// * @author Anyi Lin - 10158 Scott's Bots
// * @author Aaron Yang - 10158 Scott's Bots
// * @author Harrison Womack - 10158 Scott's Bots
// * @author Baron Henderson - 20077 The Indubitables
// * @version 1.0, 3/13/2024
// */
//class LateralVelocityTuner extends OpMode {
//    private final ArrayList<Double> velocities = new ArrayList<>();
//
//    public static double DISTANCE = 36;
//    public static double RECORD_NUMBER = 10;
//
//    private boolean end;
//
//    @Override
//    public void init() {
//        follower.setStartingPose(new Pose(72, 72));
//    }
//
//    /**
//     * This initializes the drive motors as well as the cache of velocities and the Panels
//     * telemetryM.
//     */
//    @Override
//    public void init_loop() {
//        telemetryM.debug("The robot will run at 1 power until it reaches " + DISTANCE + " inches to the left.");
//        telemetryM.debug("Make sure you have enough room, since the robot has inertia after cutting power.");
//        telemetryM.debug("After running the distance, the robot will cut power from the drivetrain and display the strafe velocity.");
//        telemetryM.debug("Press B on Gamepad 1 to stop.");
//        telemetryM.update(telemetry);
//        follower.update();
//        drawOnlyCurrent();
//    }
//
//    /** This starts the OpMode by setting the drive motors to run left at full power. */
//    @Override
//    public void start() {
//        for (int i = 0; i < RECORD_NUMBER; i++) {
//            velocities.add(0.0);
//        }
//        follower.startTeleopDrive(true);
//        follower.update();
//    }
//
//    /**
//     * This runs the OpMode. At any point during the running of the OpMode, pressing B on
//     * game pad1 will stop the OpMode. This continuously records the RECORD_NUMBER most recent
//     * velocities, and when the robot has run sideways enough, these last velocities recorded are
//     * averaged and printed.
//     */
//    @Override
//    public void loop() {
//        if (gamepad1.bWasPressed()) {
//            stopRobot();
//            requestOpModeStop();
//        }
//
//        follower.update();
//        draw();
//
//        if (!end) {
//            if (Math.abs(follower.getPose().getY()) > (DISTANCE + 72)) {
//                end = true;
//                stopRobot();
//            } else {
//                follower.setTeleOpDrive(0,1,0,true);
//                double currentVelocity = Math.abs(follower.getVelocity().dot(new Vector(1, Math.PI / 2)));
//                velocities.add(currentVelocity);
//                velocities.remove(0);
//            }
//        } else {
//            stopRobot();
//            double average = 0;
//            for (double velocity : velocities) {
//                average += velocity;
//            }
//            average /= velocities.size();
//
//            telemetryM.debug("Strafe Velocity: " + average);
//            telemetryM.debug("\n");
//            telemetryM.debug("Press A to set the Lateral Velocity temporarily (while robot remains on).");
//            telemetryM.update(telemetry);
//
//            if (gamepad1.aWasPressed()) {
//                follower.setYVelocity(average);
//                String message = "YMovement: " + average;
//                changes.add(message);
//            }
//        }
//    }
//}
//
///**
// * This is the ForwardZeroPowerAccelerationTuner autonomous follower OpMode. This runs the robot
// * forward until a specified velocity is achieved. Then, the robot cuts power to the motors, setting
// * them to zero power. The deceleration, or negative acceleration, is then measured until the robot
// * stops. The accelerations across the entire time the robot is slowing down is then averaged and
// * that number is then printed. This is used to determine how the robot will decelerate in the
// * forward direction when power is cut, making the estimations used in the calculations for the
// * drive Vector more accurate and giving better braking at the end of Paths.
// *
// * @author Anyi Lin - 10158 Scott's Bots
// * @author Baron Henderson - 20077 The Indubitables
// * @author Aaron Yang - 10158 Scott's Bots
// * @author Harrison Womack - 10158 Scott's Bots
// * @version 1.0, 3/13/2024
// */
//class ForwardZeroPowerAccelerationTuner extends OpMode {
//    private final ArrayList<Double> accelerations = new ArrayList<>();
//    public static double VELOCITY = 30;
//
//    private double previousVelocity;
//    private long previousTimeNano;
//
//    private boolean stopping;
//    private boolean end;
//
//    @Override
//    public void init() {
//        follower.setStartingPose(new Pose(72, 72));
//    }
//
//    /** This initializes the drive motors as well as the Panels telemetryM. */
//    @Override
//    public void init_loop() {
//        telemetryM.debug("The robot will run forward until it reaches " + VELOCITY + " inches per second.");
//        telemetryM.debug("Then, it will cut power from the drivetrain and roll to a stop.");
//        telemetryM.debug("Make sure you have enough room.");
//        telemetryM.debug("After stopping, the forward zero power acceleration (natural deceleration) will be displayed.");
//        telemetryM.debug("Press B on Gamepad 1 to stop.");
//        telemetryM.update(telemetry);
//        follower.update();
//        drawOnlyCurrent();
//    }
//
//    /** This starts the OpMode by setting the drive motors to run forward at full power. */
//    @Override
//    public void start() {
//        follower.startTeleopDrive(false);
//        follower.update();
//        follower.setTeleOpDrive(1,0,0,true);
//    }
//
//    /**
//     * This runs the OpMode. At any point during the running of the OpMode, pressing B on
//     * game pad 1 will stop the OpMode. When the robot hits the specified velocity, the robot will
//     * record its deceleration / negative acceleration until it stops. Then, it will average all the
//     * recorded deceleration / negative acceleration and print that value.
//     */
//    @Override
//    public void loop() {
//        if (gamepad1.bWasPressed()) {
//            stopRobot();
//            requestOpModeStop();
//        }
//
//        follower.update();
//        draw();
//
//        Vector heading = new Vector(1.0, follower.getPose().getHeading());
//        if (!end) {
//            if (!stopping) {
//                if (follower.getVelocity().dot(heading) > VELOCITY) {
//                    previousVelocity = follower.getVelocity().dot(heading);
//                    previousTimeNano = System.nanoTime();
//                    stopping = true;
//                    follower.setTeleOpDrive(0,0,0,true);
//                }
//            } else {
//                double currentVelocity = follower.getVelocity().dot(heading);
//                accelerations.add((currentVelocity - previousVelocity) / ((System.nanoTime() - previousTimeNano) / Math.pow(10.0, 9)));
//                previousVelocity = currentVelocity;
//                previousTimeNano = System.nanoTime();
//                if (currentVelocity < follower.getConstraints().getVelocityConstraint()) {
//                    end = true;
//                }
//            }
//        } else {
//            double average = 0;
//            for (double acceleration : accelerations) {
//                average += acceleration;
//            }
//            average /= accelerations.size();
//
//            telemetryM.debug("Forward Zero Power Acceleration (Deceleration): " + average);
//            telemetryM.debug("\n");
//            telemetryM.debug("Press A to set the Forward Zero Power Acceleration temporarily (while robot remains on).");
//            telemetryM.update(telemetry);
//
//            if (gamepad1.aWasPressed()) {
//                follower.getConstants().setForwardZeroPowerAcceleration(average);
//                String message = "Forward Zero Power Acceleration: " + average;
//                changes.add(message);
//            }
//        }
//    }
//}
//
///**
// * This is the LateralZeroPowerAccelerationTuner autonomous follower OpMode. This runs the robot
// * to the left until a specified velocity is achieved. Then, the robot cuts power to the motors, setting
// * them to zero power. The deceleration, or negative acceleration, is then measured until the robot
// * stops. The accelerations across the entire time the robot is slowing down is then averaged and
// * that number is then printed. This is used to determine how the robot will decelerate in the
// * forward direction when power is cut, making the estimations used in the calculations for the
// * drive Vector more accurate and giving better braking at the end of Paths.
// *
// * @author Anyi Lin - 10158 Scott's Bots
// * @author Aaron Yang - 10158 Scott's Bots
// * @author Harrison Womack - 10158 Scott's Bots
// * @author Baron Henderson - 20077 The Indubitables
// * @version 1.0, 3/13/2024
// */
//class LateralZeroPowerAccelerationTuner extends OpMode {
//    private final ArrayList<Double> accelerations = new ArrayList<>();
//    public static double VELOCITY = 30;
//    private double previousVelocity;
//    private long previousTimeNano;
//    private boolean stopping;
//    private boolean end;
//
//    @Override
//    public void init() {
//        follower.setStartingPose(new Pose(72, 72));
//    }
//
//    /** This initializes the drive motors as well as the Panels telemetry. */
//    @Override
//    public void init_loop() {
//        telemetryM.debug("The robot will run to the left until it reaches " + VELOCITY + " inches per second.");
//        telemetryM.debug("Then, it will cut power from the drivetrain and roll to a stop.");
//        telemetryM.debug("Make sure you have enough room.");
//        telemetryM.debug("After stopping, the lateral zero power acceleration (natural deceleration) will be displayed.");
//        telemetryM.debug("Press B on game pad 1 to stop.");
//        telemetryM.update(telemetry);
//        follower.update();
//        drawOnlyCurrent();
//    }
//
//    /** This starts the OpMode by setting the drive motors to run forward at full power. */
//    @Override
//    public void start() {
//        follower.startTeleopDrive(false);
//        follower.update();
//        follower.setTeleOpDrive(0,1,0,true);
//    }
//
//    /**
//     * This runs the OpMode. At any point during the running of the OpMode, pressing B on
//     * game pad 1 will stop the OpMode. When the robot hits the specified velocity, the robot will
//     * record its deceleration / negative acceleration until it stops. Then, it will average all the
//     * recorded deceleration / negative acceleration and print that value.
//     */
//    @Override
//    public void loop() {
//        if (gamepad1.bWasPressed()) {
//            stopRobot();
//            requestOpModeStop();
//        }
//
//        follower.update();
//        draw();
//
//        Vector heading = new Vector(1.0, follower.getPose().getHeading() - Math.PI / 2);
//        if (!end) {
//            if (!stopping) {
//                if (Math.abs(follower.getVelocity().dot(heading)) > VELOCITY) {
//                    previousVelocity = Math.abs(follower.getVelocity().dot(heading));
//                    previousTimeNano = System.nanoTime();
//                    stopping = true;
//                    follower.setTeleOpDrive(0,0,0,true);
//                }
//            } else {
//                double currentVelocity = Math.abs(follower.getVelocity().dot(heading));
//                accelerations.add((currentVelocity - previousVelocity) / ((System.nanoTime() - previousTimeNano) / Math.pow(10.0, 9)));
//                previousVelocity = currentVelocity;
//                previousTimeNano = System.nanoTime();
//                if (currentVelocity < follower.getConstraints().getVelocityConstraint()) {
//                    end = true;
//                }
//            }
//        } else {
//            double average = 0;
//            for (double acceleration : accelerations) {
//                average += acceleration;
//            }
//            average /= accelerations.size();
//
//            telemetryM.debug("Lateral Zero Power Acceleration (Deceleration): " + average);
//            telemetryM.debug("\n");
//            telemetryM.debug("Press A to set the Lateral Zero Power Acceleration temporarily (while robot remains on).");
//            telemetryM.update(telemetry);
//
//            if (gamepad1.aWasPressed()) {
//                follower.getConstants().setLateralZeroPowerAcceleration(average);
//                String message = "Lateral Zero Power Acceleration: " + average;
//                changes.add(message);
//            }
//        }
//    }
//}
//
///**
// * This is the Translational PIDF Tuner OpMode. It will keep the robot in place.
// * The user should push the robot laterally to test the PIDF and adjust the PIDF values accordingly.
// *
// * @author Baron Henderson - 20077 The Indubitables
// * @author Anyi Lin - 10158 Scott's Bots
// * @author Aaron Yang - 10158 Scott's Bots
// * @author Harrison Womack - 10158 Scott's Bots
// * @version 1.0, 3/12/2024
// */
//class TranslationalTuner extends OpMode {
//    public static double DISTANCE = 40;
//    private boolean forward = true;
//
//    private Path forwards;
//    private Path backwards;
//
//    @Override
//    public void init() {
//        follower.setStartingPose(new Pose(72, 72));
//    }
//
//    /** This initializes the Follower and creates the forward and backward Paths. */
//    @Override
//    public void init_loop() {
//        telemetryM.debug("This will activate the translational PIDF(s)");
//        telemetryM.debug("The robot will try to stay in place while you push it laterally.");
//        telemetryM.debug("You can adjust the PIDF values to tune the robot's translational PIDF(s).");
//        telemetryM.update(telemetry);
//        follower.update();
//        drawOnlyCurrent();
//    }
//
//    @Override
//    public void start() {
//        follower.deactivateAllPIDFs();
//        follower.activateTranslational();
//        forwards = new Path(new BezierLine(new Pose(72,72), new Pose(DISTANCE + 72,72)));
//        forwards.setConstantHeadingInterpolation(0);
//        backwards = new Path(new BezierLine(new Pose(DISTANCE + 72,72), new Pose(72,72)));
//        backwards.setConstantHeadingInterpolation(0);
//        follower.followPath(forwards);
//    }
//
//    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry */
//    @Override
//    public void loop() {
//        follower.update();
//        draw();
//
//        if (!follower.isBusy()) {
//            if (forward) {
//                forward = false;
//                follower.followPath(backwards);
//            } else {
//                forward = true;
//                follower.followPath(forwards);
//            }
//        }
//
//        telemetryM.debug("Push the robot laterally to test the Translational PIDF(s).");
//        telemetryM.addData("Zero Line", 0);
//        telemetryM.addData("Error X", follower.errorCalculator.getTranslationalError().getXComponent());
//        telemetryM.addData("Error Y", follower.errorCalculator.getTranslationalError().getYComponent());
//        telemetryM.update(telemetry);
//    }
//}
//
///**
// * This is the Heading PIDF Tuner OpMode. It will keep the robot in place.
// * The user should try to turn the robot to test the PIDF and adjust the PIDF values accordingly.
// * It will try to keep the robot at a constant heading while the user tries to turn it.
// *
// * @author Baron Henderson - 20077 The Indubitables
// * @author Anyi Lin - 10158 Scott's Bots
// * @author Aaron Yang - 10158 Scott's Bots
// * @author Harrison Womack - 10158 Scott's Bots
// * @version 1.0, 3/12/2024
// */
//class HeadingTuner extends OpMode {
//    public static double DISTANCE = 40;
//    private boolean forward = true;
//
//    private Path forwards;
//    private Path backwards;
//
//    @Override
//    public void init() {
//        follower.setStartingPose(new Pose(72, 72));
//    }
//
//    /**
//     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
//     * initializes the Panels telemetry.
//     */
//    @Override
//    public void init_loop() {
//        telemetryM.debug("This will activate the heading PIDF(s).");
//        telemetryM.debug("The robot will try to stay at a constant heading while you try to turn it.");
//        telemetryM.debug("You can adjust the PIDF values to tune the robot's heading PIDF(s).");
//        telemetryM.update(telemetry);
//        follower.update();
//        drawOnlyCurrent();
//    }
//
//    @Override
//    public void start() {
//        follower.deactivateAllPIDFs();
//        follower.activateHeading();
//        forwards = new Path(new BezierLine(new Pose(72,72), new Pose(DISTANCE + 72,72)));
//        forwards.setConstantHeadingInterpolation(0);
//        backwards = new Path(new BezierLine(new Pose(DISTANCE + 72,72), new Pose(72,72)));
//        backwards.setConstantHeadingInterpolation(0);
//        follower.followPath(forwards);
//    }
//
//    /**
//     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
//     * the Telemetry, as well as the Panels.
//     */
//    @Override
//    public void loop() {
//        follower.update();
//        draw();
//
//        if (!follower.isBusy()) {
//            if (forward) {
//                forward = false;
//                follower.followPath(backwards);
//            } else {
//                forward = true;
//                follower.followPath(forwards);
//            }
//        }
//
//        telemetryM.debug("Turn the robot manually to test the Heading PIDF(s).");
//        telemetryM.addData("Zero Line", 0);
//        telemetryM.addData("Error", follower.errorCalculator.getHeadingError());
//        telemetryM.update(telemetry);
//    }
//}
//
///**
// * This is the Drive PIDF Tuner OpMode. It will run the robot in a straight line going forward and back.
// *
// * @author Baron Henderson - 20077 The Indubitables
// * @author Anyi Lin - 10158 Scott's Bots
// * @author Aaron Yang - 10158 Scott's Bots
// * @author Harrison Womack - 10158 Scott's Bots
// * @version 1.0, 3/12/2024
// */
//class DriveTuner extends OpMode {
//    public static double DISTANCE = 40;
//    private boolean forward = true;
//
//    private PathChain forwards;
//    private PathChain backwards;
//
//    @Override
//    public void init() {
//        follower.setStartingPose(new Pose(72, 72));
//    }
//
//    /**
//     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
//     * initializes the Panels telemetry.
//     */
//    @Override
//    public void init_loop() {
//        telemetryM.debug("This will run the robot in a straight line going " + DISTANCE + "inches forward.");
//        telemetryM.debug("The robot will go forward and backward continuously along the path.");
//        telemetryM.debug("Make sure you have enough room.");
//        telemetryM.update(telemetry);
//        follower.update();
//        drawOnlyCurrent();
//    }
//
//    @Override
//    public void start() {
//        follower.deactivateAllPIDFs();
//        follower.activateDrive();
//
//        forwards = follower.pathBuilder()
//                .setGlobalDeceleration()
//                .addPath(new BezierLine(new Pose(72,72), new Pose(DISTANCE + 72,72)))
//                .setConstantHeadingInterpolation(0)
//                .build();
//
//        backwards = follower.pathBuilder()
//                .setGlobalDeceleration()
//                .addPath(new BezierLine(new Pose(DISTANCE + 72,72), new Pose(72,72)))
//                .setConstantHeadingInterpolation(0)
//                .build();
//
//        follower.followPath(forwards);
//    }
//
//    /**
//     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
//     * the Telemetry, as well as the Panels.
//     */
//    @Override
//    public void loop() {
//        follower.update();
//        draw();
//
//        if (!follower.isBusy()) {
//            if (forward) {
//                forward = false;
//                follower.followPath(backwards);
//            } else {
//                forward = true;
//                follower.followPath(forwards);
//            }
//        }
//
//        telemetryM.debug("Driving forward?: " + forward);
//        telemetryM.addData("Zero Line", 0);
//        telemetryM.addData("Error", follower.errorCalculator.getDriveErrors()[1]);
//        telemetryM.update(telemetry);
//    }
//}
//
///**
// * This is the Line Test Tuner OpMode. It will drive the robot forward and back
// * The user should push the robot laterally and angular to test out the drive, heading, and translational PIDFs.
// *
// * @author Baron Henderson - 20077 The Indubitables
// * @author Anyi Lin - 10158 Scott's Bots
// * @author Aaron Yang - 10158 Scott's Bots
// * @author Harrison Womack - 10158 Scott's Bots
// * @version 1.0, 3/12/2024
// */
//class Line extends OpMode {
//    public static double DISTANCE = 40;
//    private boolean forward = true;
//
//    private Path forwards;
//    private Path backwards;
//
//    @Override
//    public void init() {
//        follower.setStartingPose(new Pose(72, 72));
//    }
//
//    /** This initializes the Follower and creates the forward and backward Paths. */
//    @Override
//    public void init_loop() {
//        telemetryM.debug("This will activate all the PIDF(s)");
//        telemetryM.debug("The robot will go forward and backward continuously along the path while correcting.");
//        telemetryM.debug("You can adjust the PIDF values to tune the robot's drive PIDF(s).");
//        telemetryM.update(telemetry);
//        follower.update();
//        drawOnlyCurrent();
//    }
//
//    @Override
//    public void start() {
//        follower.activateAllPIDFs();
//        forwards = new Path(new BezierLine(new Pose(72,72), new Pose(DISTANCE + 72,72)));
//        forwards.setConstantHeadingInterpolation(0);
//        backwards = new Path(new BezierLine(new Pose(DISTANCE + 72,72), new Pose(72,72)));
//        backwards.setConstantHeadingInterpolation(0);
//        follower.followPath(forwards);
//    }
//
//    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry */
//    @Override
//    public void loop() {
//        follower.update();
//        draw();
//
//        if (!follower.isBusy()) {
//            if (forward) {
//                forward = false;
//                follower.followPath(backwards);
//            } else {
//                forward = true;
//                follower.followPath(forwards);
//            }
//        }
//
//        telemetryM.debug("Driving Forward?: " + forward);
//        telemetryM.update(telemetry);
//    }
//}
//
///**
// * This is the Centripetal Tuner OpMode. It runs the robot in a specified distance
// * forward and to the left. On reaching the end of the forward Path, the robot runs the backward
// * Path the same distance back to the start. Rinse and repeat! This is good for testing a variety
// * of Vectors, like the drive Vector, the translational Vector, the heading Vector, and the
// * centripetal Vector.
// *
// * @author Baron Henderson - 20077 The Indubitables
// * @author Anyi Lin - 10158 Scott's Bots
// * @author Aaron Yang - 10158 Scott's Bots
// * @author Harrison Womack - 10158 Scott's Bots
// * @version 1.0, 3/13/2024
// */
//class CentripetalTuner extends OpMode {
//    public static double DISTANCE = 20;
//    private boolean forward = true;
//
//    private Path forwards;
//    private Path backwards;
//
//    @Override
//    public void init() {
//        follower.setStartingPose(new Pose(72, 72));
//    }
//
//    /**
//     * This initializes the Follower and creates the forward and backward Paths.
//     * Additionally, this initializes the Panels telemetry.
//     */
//    @Override
//    public void init_loop() {
//        telemetryM.debug("This will run the robot in a curve going " + DISTANCE + " inches to the left and the same number of inches forward.");
//        telemetryM.debug("The robot will go continuously along the path.");
//        telemetryM.debug("Make sure you have enough room.");
//        telemetryM.update(telemetry);
//        follower.update();
//        drawOnlyCurrent();
//    }
//
//    @Override
//    public void start() {
//        follower.activateAllPIDFs();
//        forwards = new Path(new BezierCurve(new Pose(72,72), new Pose(Math.abs(DISTANCE) + 72,72), new Pose(Math.abs(DISTANCE) + 72,DISTANCE + 72)));
//        backwards = new Path(new BezierCurve(new Pose(Math.abs(DISTANCE) + 72,DISTANCE + 72), new Pose(Math.abs(DISTANCE) + 72,72), new Pose(72,72)));
//
//        backwards.setTangentHeadingInterpolation();
//        backwards.reverseHeadingInterpolation();
//
//        follower.followPath(forwards);
//    }
//
//    /**
//     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
//     * the Telemetry, as well as the Panels.
//     */
//    @Override
//    public void loop() {
//        follower.update();
//        draw();
//        if (!follower.isBusy()) {
//            if (forward) {
//                forward = false;
//                follower.followPath(backwards);
//            } else {
//                forward = true;
//                follower.followPath(forwards);
//            }
//        }
//
//        telemetryM.debug("Driving away from the origin along the curve?: " + forward);
//        telemetryM.update(telemetry);
//    }
//}
//
///**
// * This is the Triangle autonomous OpMode.
// * It runs the robot in a triangle, with the starting point being the bottom-middle point.
// *
// * @author Baron Henderson - 20077 The Indubitables
// * @author Samarth Mahapatra - 1002 CircuitRunners Robotics Surge
// * @version 1.0, 12/30/2024
// */
//class Triangle extends OpMode {
//
//    private final Pose startPose = new Pose(72, 72, Math.toRadians(0));
//    private final Pose interPose = new Pose(24 + 72, -24 + 72, Math.toRadians(90));
//    private final Pose endPose = new Pose(24 + 72, 24 + 72, Math.toRadians(45));
//
//    private PathChain triangle;
//
//    /**
//     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
//     * the Telemetry, as well as the Panels.
//     */
//    @Override
//    public void loop() {
//        follower.update();
//        draw();
//
//        if (follower.atParametricEnd()) {
//            follower.followPath(triangle, true);
//        }
//    }
//
//    @Override
//    public void init() {
//        follower.setStartingPose(new Pose(72, 72));
//    }
//
//    @Override
//    public void init_loop() {
//        telemetryM.debug("This will run in a roughly triangular shape, starting on the bottom-middle point.");
//        telemetryM.debug("So, make sure you have enough space to the left, front, and right to run the OpMode.");
//        telemetryM.update(telemetry);
//        follower.update();
//        drawOnlyCurrent();
//    }
//
//    /** Creates the PathChain for the "triangle".*/
//    @Override
//    public void start() {
//        follower.setStartingPose(startPose);
//
//        triangle = follower.pathBuilder()
//                .addPath(new BezierLine(startPose, interPose))
//                .setLinearHeadingInterpolation(startPose.getHeading(), interPose.getHeading())
//                .addPath(new BezierLine(interPose, endPose))
//                .setLinearHeadingInterpolation(interPose.getHeading(), endPose.getHeading())
//                .addPath(new BezierLine(endPose, startPose))
//                .setLinearHeadingInterpolation(endPose.getHeading(), startPose.getHeading())
//                .build();
//
//        follower.followPath(triangle);
//    }
//}
//
///**
// * This is the Circle autonomous OpMode. It runs the robot in a PathChain that's actually not quite
// * a circle, but some Bezier curves that have control points set essentially in a square. However,
// * it turns enough to tune your centripetal force correction and some of your heading. Some lag in
// * heading is to be expected.
// *
// * @author Anyi Lin - 10158 Scott's Bots
// * @author Aaron Yang - 10158 Scott's Bots
// * @author Harrison Womack - 10158 Scott's Bots
// * @version 1.0, 3/12/2024
// */
//class Circle extends OpMode {
//    public static double RADIUS = 10;
//    private PathChain circle;
//
//    public void start() {
//        circle = follower.pathBuilder()
//                .addPath(new BezierCurve(new Pose(72, 72), new Pose(RADIUS + 72, 72), new Pose(RADIUS + 72, RADIUS + 72)))
//                .setHeadingInterpolation(HeadingInterpolator.facingPoint(72, RADIUS + 72))
//                .addPath(new BezierCurve(new Pose(RADIUS + 72, RADIUS + 72), new Pose(RADIUS + 72, (2 * RADIUS) + 72), new Pose(72, (2 * RADIUS) + 72)))
//                .setHeadingInterpolation(HeadingInterpolator.facingPoint(72, RADIUS + 72))
//                .addPath(new BezierCurve(new Pose(72, (2 * RADIUS) + 72), new Pose(-RADIUS + 72, (2 * RADIUS) + 72), new Pose(-RADIUS + 72, RADIUS + 72)))
//                .setHeadingInterpolation(HeadingInterpolator.facingPoint(72, RADIUS + 72))
//                .addPath(new BezierCurve(new Pose(-RADIUS + 72, RADIUS + 72), new Pose(-RADIUS + 72, 72), new Pose(72, 72)))
//                .setHeadingInterpolation(HeadingInterpolator.facingPoint(72, RADIUS + 72))
//                .build();
//        follower.followPath(circle);
//    }
//
//    @Override
//    public void init_loop() {
//        telemetryM.debug("This will run in a roughly circular shape of radius " + RADIUS + ", starting on the right-most edge. ");
//        telemetryM.debug("So, make sure you have enough space to the left, front, and back to run the OpMode.");
//        telemetryM.debug("It will also continuously face the center of the circle to test your heading and centripetal correction.");
//        telemetryM.update(telemetry);
//        follower.update();
//        drawOnlyCurrent();
//    }
//
//    @Override
//    public void init() {
//        follower.setStartingPose(new Pose(72, 72));
//    }
//
//    /**
//     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
//     * the Telemetry, as well as the FTC Dashboard.
//     */
//    @Override
//    public void loop() {
//        follower.update();
//        draw();
//
//        if (follower.atParametricEnd()) {
//            follower.followPath(circle);
//        }
//    }
//}
//
///**
// * This is the Drawing class. It handles the drawing of stuff on Panels Dashboard, like the robot.
// *
// * @author Lazar - 19234
// * @version 1.1, 5/19/2025
// */
//class Drawing {
//    public static final double ROBOT_RADIUS = 9; // woah
//    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
//
//    private static final Style robotLook = new Style(
//            "", "#3F51B5", 0.75
//    );
//    private static final Style historyLook = new Style(
//            "", "#4CAF50", 0.75
//    );
//
//    /**
//     * This prepares Panels Field for using Pedro Offsets
//     */
//    public static void init() {
//        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
//    }
//
//    /**
//     * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
//     * a Follower as an input, so an instance of the DashbaordDrawingHandler class is not needed.
//     *
//     * @param follower Pedro Follower instance.
//     */
//    public static void drawDebug(Follower follower) {
//        if (follower.getCurrentPath() != null) {
//            drawPath(follower.getCurrentPath(), robotLook);
//            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
//            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), robotLook);
//        }
//        drawPoseHistory(follower.getPoseHistory(), historyLook);
//        drawRobot(follower.getPose(), historyLook);
//
//        sendPacket();
//    }
//
//    /**
//     * This draws a robot at a specified Pose with a specified
//     * look. The heading is represented as a line.
//     *
//     * @param pose  the Pose to draw the robot at
//     * @param style the parameters used to draw the robot with
//     */
//    public static void drawRobot(Pose pose, Style style) {
//        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
//            return;
//        }
//
//        panelsField.setStyle(style);
//        panelsField.moveCursor(pose.getX(), pose.getY());
//        panelsField.circle(ROBOT_RADIUS);
//
//        Vector v = pose.getHeadingAsUnitVector();
//        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
//        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
//        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();
//
//        panelsField.setStyle(style);
//        panelsField.moveCursor(x1, y1);
//        panelsField.line(x2, y2);
//    }
//
//    /**
//     * This draws a robot at a specified Pose. The heading is represented as a line.
//     *
//     * @param pose the Pose to draw the robot at
//     */
//    public static void drawRobot(Pose pose) {
//        drawRobot(pose, robotLook);
//    }
//
//    /**
//     * This draws a Path with a specified look.
//     *
//     * @param path  the Path to draw
//     * @param style the parameters used to draw the Path with
//     */
//    public static void drawPath(Path path, Style style) {
//        double[][] points = path.getPanelsDrawingPoints();
//
//        for (int i = 0; i < points[0].length; i++) {
//            for (int j = 0; j < points.length; j++) {
//                if (Double.isNaN(points[j][i])) {
//                    points[j][i] = 0;
//                }
//            }
//        }
//
//        panelsField.setStyle(style);
//        panelsField.moveCursor(points[0][0], points[0][1]);
//        panelsField.line(points[1][0], points[1][1]);
//    }
//
//    /**
//     * This draws all the Paths in a PathChain with a
//     * specified look.
//     *
//     * @param pathChain the PathChain to draw
//     * @param style     the parameters used to draw the PathChain with
//     */
//    public static void drawPath(PathChain pathChain, Style style) {
//        for (int i = 0; i < pathChain.size(); i++) {
//            drawPath(pathChain.getPath(i), style);
//        }
//    }
//
//    /**
//     * This draws the pose history of the robot.
//     *
//     * @param poseTracker the PoseHistory to get the pose history from
//     * @param style       the parameters used to draw the pose history with
//     */
//    public static void drawPoseHistory(PoseHistory poseTracker, Style style) {
//        panelsField.setStyle(style);
//
//        int size = poseTracker.getXPositionsArray().length;
//        for (int i = 0; i < size - 1; i++) {
//
//            panelsField.moveCursor(poseTracker.getXPositionsArray()[i], poseTracker.getYPositionsArray()[i]);
//            panelsField.line(poseTracker.getXPositionsArray()[i + 1], poseTracker.getYPositionsArray()[i + 1]);
//        }
//    }
//
//    /**
//     * This draws the pose history of the robot.
//     *
//     * @param poseTracker the PoseHistory to get the pose history from
//     */
//    public static void drawPoseHistory(PoseHistory poseTracker) {
//        drawPoseHistory(poseTracker, historyLook);
//    }
//
//    /**
//     * This tries to send the current packet to FTControl Panels.
//     */
//    public static void sendPacket() {
//        panelsField.update();
//    }
//}