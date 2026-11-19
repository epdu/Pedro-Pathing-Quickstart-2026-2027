package org.firstinspires.ftc.teamcode.pedro;

import static com.pedropathing.api.Paths.*;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
@Autonomous(name = "AAA AUTO DecodeRobotBlueBioBuzzSeason V1")
public class AAAAutoDecodeRobotBlueBioBuzzSeason extends LinearOpMode {

    HardwareDecodeRobotBlueBioBuzzSeason robot = new HardwareDecodeRobotBlueBioBuzzSeason();

    // 蓝方目标角坐标（Pedro Pathing 0-144 坐标系，可在 Panels 实时微调）
    public static double BLUE_GOAL_X = 0.0;
    public static double BLUE_GOAL_Y = 144.0;
    public static double TURRET_ANGLE_OFFSET_DEG = 0.0;

    private static final double Med_SHOOTER_TARGET_Velocity = 1100;
    public float intakePowerIntake = 0.95f;
    public float intakePowerShoot = 0.90f;
    public float intakePowerOff = 0.0f;

    public static final double blockageblockposition = 0.10;
    public static final double blockagereleaseposition = 0.18;

    private Follower follower;
    private TelemetryManager panelsTelemetry;
    private final ElapsedTime pathTimer = new ElapsedTime();
    private final ElapsedTime shootTimer = new ElapsedTime();

    private PathState pathState;
    private AutoShootState autoShootState;
    private boolean firstPickupCompleted = false;
    private double cachedShooterVeloL = 0.0;
    private double cachedShooterVeloR = 0.0;

    private final PoseFactory mirroredPoseFactory = PoseFactory.degrees().mirrorY(72);

    public enum PathState {
        DRIVE_START_POS_SHOOT_POS,
        DRIVE_TO_SHOOT_WAIT,
        SHOOT_PRELOAD,
        DRIVE_READY_SECOND_SPIKE_PICKUPT_POS,
        SECOND_SPIKE_PICKUP,
        DRIVE_READY_OPEN_GATE_POS,
        OPEN_GATE,
        DRIVE_BACK_SECOND_SPIKE_PICKUPT_POS,
        SHOOT_SECOND_SPIKE_PICKUP,
        DRIVE_READY_THIRD_SPIKE_PICKUPT_POS,
        THIRD_SPIKE_PICKUP,
        DRIVE_BACK_THIRD_SPIKE_PICKUPT_POS,
        SHOOT_THIRD_SPIKE_PICKUP,
        DRIVE_READY_FIRST_SPIKE_PICKUP_POS,
        FIRST_SPIKE_PICKUP,
        DRIVE_BACK_FIRST_SPIKE_SHOOT_POS,
        SHOOT_FIRST_SPIKE_PICKUP,
        DRIVE_OFFLINE,
        END
    }

    enum AutoShootState {
        IDLE,
        SPINNING_UP,
        FEEDING,
        DONE
    }

    @Configurable
    public static class ShooterPIDFConfig {
        public static double kP = 300.0;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double kF = 15.0;
        public static double targetVelocity = Med_SHOOTER_TARGET_Velocity;
        public static double toleranceofVelocity = 80.0;
    }

    // 路径坐标定义
    private final Pose startPose = mirroredPoseFactory.of(127.5, 118.75, 0);
    private final Pose shootPose = mirroredPoseFactory.of(91, 92.25, 45);
    private final Pose readyThirdSpikePickupPose = mirroredPoseFactory.of(92, 86.25, 0);
    private final Pose readySecondSpikePickupPose = mirroredPoseFactory.of(92, 62.25, 0);
    private final Pose readyFirstSpikePickupPose = mirroredPoseFactory.of(92, 36.25, 0);
    private final Pose readyOpenGatePickupPose = mirroredPoseFactory.of(124.5, 67.5, 0);
    private final Pose thirdSpikePickupPose = mirroredPoseFactory.of(127, 86.25, 0);
    private final Pose secondSpikePickupPose = mirroredPoseFactory.of(133, 62.25, 0);
    private final Pose secondSpikePickupCP = mirroredPoseFactory.of(72, 76, 0);
    private final Pose openGatePickupCP = mirroredPoseFactory.of(72, 76, 0);
    private final Pose firstSpikePickupPose = mirroredPoseFactory.of(134, 36.25, 0);
    private final Pose openGatePickupPose = mirroredPoseFactory.of(132.5, 69, 0);
    private final Pose offlinePose = mirroredPoseFactory.of(112, 92.25, 0);

    private Path driveStartShoot;
    private Path driveReadyThirdSpikePickup, driveThirdSpikePickup, driveThirdSpikePickupShoot;
    private Path driveReadyOpenGatePickup, driveOpenGatePickup, driveOpenGatePickupShoot;
    private Path driveReadySecondSpikePickup, driveSecondSpikePickup;
    private Path driveReadyFirstSpikePickup, driveFirstSpikePickup, driveFirstSpikePickupShoot;
    private Path driveOffline;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.alliance = Alliance.BLUE;
        robot.initVision(false); // 自动比赛关闭画面预览以节省 CPU

        robot.BlockageArmL.setPosition(blockageblockposition);
        robot.BlockageArmR.setPosition(blockageblockposition);

        initShooterPIDF();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        buildPaths();

        pathState = PathState.DRIVE_START_POS_SHOOT_POS;
        autoShootState = AutoShootState.IDLE;

        panelsTelemetry.debug("Auto Status", "Initialized & Ready");
        panelsTelemetry.update(telemetry);

        waitForStart();
        pathTimer.reset();
        shootTimer.reset();

        while (opModeIsActive() && pathState != PathState.END) {
            // 1. 批量刷新硬件缓存并单次读取电机速度
            robot.clearBulkCache();
            cachedShooterVeloL = Math.abs(robot.MasterShooterMotorL.getVelocity());
            cachedShooterVeloR = Math.abs(robot.SlaveShooterMotorR.getVelocity());

            // 2. 路径跟踪与自动炮台自瞄同步更新
            follower.update();
            updateAutoTurretAim();
            robot.updateTurretAxons();

            // 3. 状态机步进
            statePathUpdate();

            // 4. 双端同步刷新遥测
            updateShooterTelemetry();
            panelsTelemetry.update(telemetry);
        }

        // 自动结束前保存最终位姿供 TeleOp 继承
        saveCurrentPosition();
    }

    /**
     * 自动阶段炮台持续自瞄计算（根据机器人实时位姿对准蓝方目标点）
     */
    private void updateAutoTurretAim() {
        Pose currentPose = follower.pose();
        if (currentPose == null) return;

        double dx = BLUE_GOAL_X - currentPose.x();
        double dy = BLUE_GOAL_Y - currentPose.y();
        double fieldTargetAngle = Math.atan2(dy, dx);

        double relativeAngleRad = fieldTargetAngle - currentPose.heading();
        // 归一化到 [-PI, PI]
        while (relativeAngleRad > Math.PI) relativeAngleRad -= 2 * Math.PI;
        while (relativeAngleRad < -Math.PI) relativeAngleRad += 2 * Math.PI;

        double targetDeg = Math.toDegrees(relativeAngleRad) + TURRET_ANGLE_OFFSET_DEG;
        // 限制炮台安全旋转区间 [-90°, +90°]，防止拉扯线缆
        targetDeg = Math.max(-90.0, Math.min(90.0, targetDeg));

        robot.axonTurretArmL.trackTargetRotation(targetDeg);
//        robot.axonTurretArmR.trackTargetRotation(targetDeg);
    }

    private void initShooterPIDF() {
        robot.shooterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(
                ShooterPIDFConfig.kP,
                ShooterPIDFConfig.kI,
                ShooterPIDFConfig.kD,
                ShooterPIDFConfig.kF
        );
        robot.shooterL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        robot.shooterR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        robot.shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void autoshoot() {
        switch (autoShootState) {
            case IDLE:
                robot.BlockageArmL.setPosition(blockageblockposition);
                robot.BlockageArmR.setPosition(blockageblockposition);
                startShooter();
                shootTimer.reset();
                autoShootState = AutoShootState.SPINNING_UP;
                break;

            case SPINNING_UP:
                boolean speedReady = Math.abs(cachedShooterVeloL - ShooterPIDFConfig.targetVelocity) <= ShooterPIDFConfig.toleranceofVelocity;
                boolean turretReady = robot.axonTurretArmL.isAtTarget(4.0);
                // 当飞轮达速且炮台对准目标（或超时 1.5s 防卡死）时开始推球
                if ((speedReady && turretReady) || shootTimer.seconds() > 1.5) {
                    robot.BlockageArmL.setPosition(blockagereleaseposition);
                    robot.BlockageArmR.setPosition(blockagereleaseposition);
                    robot.IntakeMotorL.setPower(intakePowerShoot);
                    robot.IntakeMotorR.setPower(intakePowerShoot);
                    shootTimer.reset();
                    autoShootState = AutoShootState.FEEDING;
                }
                break;

            case FEEDING:
                if (shootTimer.seconds() >= 1.0) {
                    stopShooter();
                    stopIntake();
                    robot.BlockageArmL.setPosition(blockageblockposition);
                    robot.BlockageArmR.setPosition(blockageblockposition);
                    autoShootState = AutoShootState.DONE;
                }
                break;

            case DONE:
                break;
        }
    }

    private void startShooter() {
        robot.IntakeMotorL.setPower(0);
        robot.IntakeMotorR.setPower(0);
        robot.setHoodPosition(0.48);
        robot.shooterL.setVelocity(Math.abs(ShooterPIDFConfig.targetVelocity));
        robot.shooterR.setVelocity(Math.abs(ShooterPIDFConfig.targetVelocity));
    }

    private void stopShooter() {
        robot.MasterShooterMotorL.setVelocity(0);
        robot.SlaveShooterMotorR.setVelocity(0);
        robot.IntakeMotorL.setPower(0);
        robot.IntakeMotorR.setPower(0);
    }

    public void autoIntake() {
        robot.IntakeMotorL.setPower(intakePowerIntake);
        robot.IntakeMotorR.setPower(intakePowerIntake);
        robot.BlockageArmL.setPosition(blockageblockposition);
        robot.BlockageArmR.setPosition(blockageblockposition);
    }
    private void stopIntake() {
        robot.IntakeMotorL.setPower(intakePowerOff);
        robot.IntakeMotorR.setPower(intakePowerOff);
    }
    private void saveCurrentPosition() {
        if (follower != null) {
            Pose currentPose = follower.pose();
            robot.finalAutoX = currentPose.x();
            robot.finalAutoY = currentPose.y();
            robot.finalAutoHeading = currentPose.heading();
        }
    }
    private void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.reset(); // 修复：原先误写为 .seconds() 导致从不重置！
    }
    private void statePathUpdate() {
        switch (pathState) {
            case DRIVE_START_POS_SHOOT_POS:
                follower.follow(driveStartShoot);
                setPathState(PathState.DRIVE_TO_SHOOT_WAIT);
                break;
            case DRIVE_TO_SHOOT_WAIT:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;
            case SHOOT_PRELOAD:
                autoshoot();
                if (autoShootState == AutoShootState.DONE) {
                    follower.follow(driveReadySecondSpikePickup);
                    setPathState(PathState.DRIVE_READY_SECOND_SPIKE_PICKUPT_POS);
                }
                break;
            case DRIVE_READY_SECOND_SPIKE_PICKUPT_POS:
                if (!follower.isBusy()) {
                    follower.follow(driveSecondSpikePickup);
                    setPathState(PathState.SECOND_SPIKE_PICKUP);
                }
                break;
            case SECOND_SPIKE_PICKUP:
                if (pathTimer.seconds() < 1.3) {
                    autoIntake();
                } else {
                    follower.follow(driveReadyOpenGatePickup);
                    stopIntake();
                    setPathState(PathState.DRIVE_READY_OPEN_GATE_POS);
                }
                break;
            case DRIVE_READY_OPEN_GATE_POS:
                if (!follower.isBusy()) {
                    follower.follow(driveOpenGatePickup);
                    setPathState(PathState.OPEN_GATE);
                }
                break;
            case OPEN_GATE:
                if (pathTimer.seconds() >= 1.2) {
                    follower.follow(driveOpenGatePickupShoot);
                    setPathState(PathState.DRIVE_BACK_SECOND_SPIKE_PICKUPT_POS);
                }
                break;
            case DRIVE_BACK_SECOND_SPIKE_PICKUPT_POS:
                if (!follower.isBusy()) {
                    autoShootState = AutoShootState.IDLE;
                    setPathState(PathState.SHOOT_SECOND_SPIKE_PICKUP);
                }
                break;
            case SHOOT_SECOND_SPIKE_PICKUP:
                autoshoot();
                if (autoShootState == AutoShootState.DONE) {
                    follower.follow(driveReadyThirdSpikePickup);
                    setPathState(PathState.DRIVE_READY_THIRD_SPIKE_PICKUPT_POS);
                }
                break;
            case DRIVE_READY_THIRD_SPIKE_PICKUPT_POS:
                if (!follower.isBusy()) {
                    follower.follow(driveThirdSpikePickup);
                    setPathState(PathState.THIRD_SPIKE_PICKUP);
                }
                break;
            case THIRD_SPIKE_PICKUP:
                if (!firstPickupCompleted && pathTimer.seconds() < 1.3) {
                    autoIntake();
                } else {
                    follower.follow(driveThirdSpikePickupShoot);
                    stopIntake();
                    firstPickupCompleted = true;
                    setPathState(PathState.DRIVE_BACK_THIRD_SPIKE_PICKUPT_POS);
                }
                break;
            case DRIVE_BACK_THIRD_SPIKE_PICKUPT_POS:
                if (!follower.isBusy()) {
                    autoShootState = AutoShootState.IDLE;
                    setPathState(PathState.SHOOT_THIRD_SPIKE_PICKUP);
                }
                break;
            case SHOOT_THIRD_SPIKE_PICKUP:
                autoshoot();
                if (autoShootState == AutoShootState.DONE) {
                    follower.follow(driveReadyFirstSpikePickup);
                    setPathState(PathState.DRIVE_READY_FIRST_SPIKE_PICKUP_POS);
                }
                break;
            case DRIVE_READY_FIRST_SPIKE_PICKUP_POS:
                if (!follower.isBusy()) {
                    follower.follow(driveFirstSpikePickup);
                    setPathState(PathState.FIRST_SPIKE_PICKUP);
                }
                break;
            case FIRST_SPIKE_PICKUP:
                if (pathTimer.seconds() < 1.3) {
                    autoIntake();
                } else {
                    follower.follow(driveFirstSpikePickupShoot);
                    stopIntake();
                    setPathState(PathState.DRIVE_BACK_FIRST_SPIKE_SHOOT_POS);
                }
                break;
            case DRIVE_BACK_FIRST_SPIKE_SHOOT_POS:
                if (!follower.isBusy()) {
                    autoShootState = AutoShootState.IDLE;
                    setPathState(PathState.SHOOT_FIRST_SPIKE_PICKUP);
                }
                break;
            case SHOOT_FIRST_SPIKE_PICKUP:
                autoshoot();
                if (autoShootState == AutoShootState.DONE) {
                    follower.follow(driveOffline);
                    setPathState(PathState.DRIVE_OFFLINE);
                }
                break;
            case DRIVE_OFFLINE:
                if (!follower.isBusy()) {
                    setPathState(PathState.END);
                }
                break;
        }
    }
    private void buildPaths() {
        driveStartShoot = line(startPose, shootPose).linear(startPose.heading(), shootPose.heading());
        driveReadyFirstSpikePickup = line(shootPose, readyFirstSpikePickupPose).linear(shootPose.heading(), readyFirstSpikePickupPose.heading());
        driveFirstSpikePickup = line(readyFirstSpikePickupPose, firstSpikePickupPose).linear(readyFirstSpikePickupPose.heading(), firstSpikePickupPose.heading());
        driveFirstSpikePickupShoot = line(firstSpikePickupPose, shootPose).linear(firstSpikePickupPose.heading(), shootPose.heading());
        driveSecondSpikePickup = line(readySecondSpikePickupPose, secondSpikePickupPose).linear(readySecondSpikePickupPose.heading(), secondSpikePickupPose.heading());
        driveReadyThirdSpikePickup = line(shootPose, readyThirdSpikePickupPose).linear(shootPose.heading(), readyThirdSpikePickupPose.heading());
        driveThirdSpikePickup = line(readyThirdSpikePickupPose, thirdSpikePickupPose).linear(readyThirdSpikePickupPose.heading(), thirdSpikePickupPose.heading());
        driveThirdSpikePickupShoot = line(thirdSpikePickupPose, shootPose).linear(thirdSpikePickupPose.heading(), shootPose.heading());
        driveReadyOpenGatePickup = line(secondSpikePickupPose, readyOpenGatePickupPose).linear(secondSpikePickupPose.heading(), readyOpenGatePickupPose.heading());
        driveOpenGatePickup = line(readyOpenGatePickupPose, openGatePickupPose).linear(readyOpenGatePickupPose.heading(), openGatePickupPose.heading());
        driveOffline = line(shootPose, offlinePose).linear(shootPose.heading(), offlinePose.heading());
        driveReadySecondSpikePickup = curve(shootPose, secondSpikePickupCP, readySecondSpikePickupPose).linear(shootPose.heading(), readySecondSpikePickupPose.heading());
        driveOpenGatePickupShoot = curve(openGatePickupPose, openGatePickupCP, shootPose).linear(openGatePickupPose.heading(), shootPose.heading());
    }
    private void updateShooterTelemetry() {
        if (panelsTelemetry != null) {
            panelsTelemetry.debug("Path State", pathState.name());
            panelsTelemetry.debug("Shoot State", autoShootState.name());
            panelsTelemetry.debug("Shooter Left Velo", cachedShooterVeloL);
            panelsTelemetry.debug("Shooter Right Velo", cachedShooterVeloR);
            panelsTelemetry.debug("Turret Target (deg)", robot.axonTurretArmL.getTargetRotation());
            panelsTelemetry.debug("Turret Current (deg)", robot.axonTurretArmL.getTotalRotation());
        }
        telemetry.addData("State", "%s | %s", pathState, autoShootState);
        telemetry.addData("Shooter (L / R)", "%.0f / %.0f", cachedShooterVeloL, cachedShooterVeloR);
        telemetry.addData("Turret (Set / Cur)", "%.1f° / %.1f°",
                robot.axonTurretArmL.getTargetRotation(), robot.axonTurretArmL.getTotalRotation());
    }
}