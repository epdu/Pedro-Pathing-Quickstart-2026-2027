package org.firstinspires.ftc.teamcode.pedro;

//import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOpQualifier.blockageblockTele;
//import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOpQualifier.blockageblockposition;
//import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOpQualifier.blockagereleaseTele;
//import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOpQualifier.blockagereleaseposition;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
//import com.bylazar.telemetry.TelemetryManager;
//import com.bylazar.ftcontrol.panels.Panels;
//import com.bylazar.ftcontrol.panels.integration.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;


import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Pose;
import com.pedropathing.api.PoseFactory;

import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "AAA match one BLUE Near. open Gate. NO Gate intake. all spikes TWELVE park 03092026 V1")
// anyupdate of auto start with this one
// change name style to be FirstSpike,SecondSpike,ThirdSpike backup 03032026
// intake second role with open gate , park at  136, 36.25 with three loaded
//. 3 ARTIFACTS on each SPIKE MARK arranged as follows:
//i.Near (audience side): GPP
//ii. Middle: PGP
//iii. Far (GOAL side): PPG
public class AAABLUE12nearopengatenogateintake extends LinearOpMode {
    HardwareQualifier robot = new HardwareQualifier();
    private Limelight3A limelight;
    private volatile boolean isRunning = true;
    private boolean shooterStarted=false;
    // Initialize elapsed timer
    private final ElapsedTime runtime = new ElapsedTime();
//    private Timer autoTimer, pathTimer,shootTimer;
    private ElapsedTime pathTimer = new ElapsedTime();
    private ElapsedTime autoTimer = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime();

    private static final double VELOCITY_TOLERANCE = 30; // RPM容差，可根据测试调整
    // 状态变量
    private boolean isShooterAtSpeed = false;
    private boolean wasShooterAtSpeed = false; // 用于检测状态变化
    private boolean firstPickupCompleted = false;
    private boolean fireRequested = false;
    // LED颜色常量（根据你的LED库调整）
    private final String LED_COLOR_READY = "GREEN";
    private final String LED_COLOR_ACCELERATING = "YELLOW";
    private final String LED_COLOR_OFF = "RED";
    //  RPM = (TPS * 60秒) / 每转ticks数
//  return (tps * 60.0) / ticksPerRevolution;  28*13.7     28*13.7-----28 6000RPM
//  TPS=RPM/60*ticksPerRevolution=RPM*28*13.7/60；
    private static final double Close_SHOOTER_TARGET_RPM = 100;//  400RPM---2,557.33333333333333
    //    private static final double Med_SHOOTER_TARGET_RPM = 204;   //1598 white tri a little bit too far//  250RPM---1586.67
    private static final double Med_SHOOTER_TARGET_RPM = 2785;   //1598 white tri a little bit too far//  250RPM---1586.67//150-100 too big
    //    private static final double Med_SHOOTER_TARGET_Velocity = 1300;
    private static final double Far_SHOOTER_TARGET_RPM = 350;  //  350RPM---2237
//   private static final double Close_SHOOTER_TARGET_RPM = 800;//  400RPM---2,557.33333333333333
//    private static final double Med_SHOOTER_TARGET_RPM = 1300;   //1598 white tri a little bit too far//  250RPM---1586.67
//    private static final double Far_SHOOTER_TARGET_RPM = 2237;  //  350RPM---2237
    ///////////////////////////////
    private static final double Med_SHOOTER_TARGET_Velocity = 1100;
    public static double flyWheelIdleSpeed=Med_SHOOTER_TARGET_Velocity*0.6;//1598 white tri a little bit too far//  250RPM---1586.67//150-100 too big
    public float  intakePowerIntake=0.95f;//0.95
    public float  intakePowerShoot=0.9f;//0.9
    public float DriveTrains_ReducePOWER=0.75f;
    public float DriveTrains_smoothTurn=0.55f;
    //    public String fieldOrRobotCentric = "robot";
    public String fieldOrRobotCentric = "field";
    private double powerMultiplier = 0.9;
    boolean move = false;
    int controlMode = 1;
//    public float  intakePowerIntake=0.75f;//0.95
//    public float  intakePowerShoot=0.85f;//0.9

    //    public float  intakePowerShoot=0.8f;//0.9
    public float  intakePowerDump=-0.6f;
    public float  intakePowerOff=0.0f;
    public float  ShooterMotorShootFar=0.95f;
    public float  ShooterMotorShootMed=-0.8f;
    public float  ShooterMotorShootClose=-0.8f;
    public float  ShooterMotorHold=-0.2f;
    public float  ShooterMotorClean=-0.8f;
    public float  ShooterMotorOff=0.0f;
    public static final double HoodArmPositionInit = 0.1;
    public static final double HoodArmPositionCloseShoot = 0.3;
    public static final double HoodArmPositionMedShoot = 0.2;
    public static final double blockageblockposition=0.10; //for auto
    public static final double blockagereleaseposition=0.18;


    // Other variables
    private Pose currentPose; // Current pose of the robot
    private Follower follower; // Pedro Pathing follower
    private TelemetryManager panelsTelemetry; // Panels telemetry
    private int pathStatePreload;
    private int pathStatePPG; // Current state machine value
    private int pathStatePGP; // Current state machine value
    private int pathStateGPP; // Current state machine value
    private boolean hasPathStarted = false;
    private PathState pathState;
    private final PoseFactory mirroredPoseFactory = PoseFactory.degrees().mirrorY(72);
    private AutoShootState autoShootState;
    //    ShooterSubsystem shooterSubsystem;
//    FlywheelSubsystem flywheelSubsystem;
//    FeederSubsystem feederSubsystem;
//    IntakeSubsystem intakeSubsystem;

    private Path driveStartShoot;
    private Path driveReadyThirdSpikePickup, driveThirdSpikePickup, driveThirdSpikePickupShoot;
    private Path driveReadyOpenGatePickup,  driveOpenGatePickup,  driveOpenGatePickupShoot;
    private Path driveReadySecondSpikePickup, driveSecondSpikePickup, driveSecondSpikePickupShoot;
    private Path driveReadyFirstSpikePickup,  driveFirstSpikePickup,  driveFirstSpikePickupShoot;
    private Path driveOffline, driveOfflineofFirst;

//    private Path driveStartShoot;
//    private Path driveReadyThirdSpikePickup;
//    private Path driveThirdSpikePickup;
//    private Path driveThirdSpikePickupShoot;
//    private Path driveReadyOpenGatePickup;
//    private Path driveOpenGatePickup;
//    private Path driveOpenGatePickupShoot;
//    private Path driveReadySecondSpikePickup;
//    private Path driveSecondSpikePickup;
//    private Path driveSecondSpikePickupShoot;
//    private Path driveReadyFirstSpikePickup;
//    private Path driveFirstSpikePickup;
//    private Path driveFirstSpikePickupShoot;
//    private Path driveOffline;
//    private Path driveOfflineofFirst;
    public enum PathState {
        DRIVE_START_POS_SHOOT_POS,
        DRIVE_TO_SHOOT_WAIT,
        SHOOT_PRELOAD,
        DRIVE_READY_THIRD_SPIKE_PICKUPT_POS,
        THIRD_SPIKE_PICKUP,
        DRIVE_BACK_THIRD_SPIKE_PICKUPT_POS,
        SHOOT_THIRD_SPIKE_PICKUP,
        DRIVE_READY_SECOND_SPIKE_PICKUPT_POS,
        SECOND_SPIKE_PICKUP,
        DRIVE_BACK_SECOND_SPIKE_PICKUPT_POS,
        SHOOT_SECOND_SPIKE_PICKUP,
        DRIVE_READY_OPEN_GATE_POS,
        OPEN_GATE,
        DRIVE_BACK_OPEN_GATE_PICKUPT_POS,
        SHOOT_OPEN_GATE_PICKUP,
        DRIVE_READY_FIRST_SPIKE_PICKUP_POS,
        FIRST_SPIKE_PICKUP,
        DRIVE_BACK_FIRST_SPIKE_SHOOT_POS,
        SHOOT_FIRST_SPIKE_PICKUP,
        DRIVE_OFFLINE,
        END
    }


    // Start Pose further zone of our robot.
    private final Pose startPose =
            mirroredPoseFactory.of(127.5, 118.75, 0);

    // x from 92 to 91, 03142026 15:32 Scoring Pose of our robot. It is facing the goal at a 115 degree angle.
    private final Pose shootPose =
            mirroredPoseFactory.of(91, 92.25, 45);

    // Scoring Pose of our robot. It is facing the goal at a 115 degree angle.
    private final Pose scoreEnd =
            mirroredPoseFactory.of(92, 92.25, 0);

    // PPG  Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose readyThirdSpikePickupPose =
            mirroredPoseFactory.of(92, 86.25, 0);

    // PGP Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose readySecondSpikePickupPose =
            mirroredPoseFactory.of(92, 62.25, 0);

    // GPP Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose readyFirstSpikePickupPose =
            mirroredPoseFactory.of(92, 36.25, 0);

    // 63--64  PGP Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose readyOpenGatePickupPose =
            mirroredPoseFactory.of(124.5, 67.5, 0);

    // PPG  Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose thirdSpikePickupPose =
            mirroredPoseFactory.of(127, 86.25, 0);

    // PGP Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose secondSpikePickupPose =
            mirroredPoseFactory.of(133, 62.25, 0);

    private final Pose secondSpikePickupCP =
            mirroredPoseFactory.of(72, 76, 0);

    private final Pose openGatePickupCP =
            mirroredPoseFactory.of(72, 76, 0);

    // PGP Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose secondSpikePickupPoseControlPoint =
            mirroredPoseFactory.of(125, 64.25, 0);

    // GPP Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose firstSpikePickupPose =
            mirroredPoseFactory.of(134, 36.25, 0);

    // 63--61
    private final Pose openGatePickupPose =
            mirroredPoseFactory.of(132.5, 69, 0);

    // GPP Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose offlinePose =
            mirroredPoseFactory.of(112, 92.25, 0);
//    private final Pose startPose = new Pose(127.5,118.75, Math.toRadians(0)).mirror(); // Start Pose further zone of our robot.
//    private final Pose shootPose = new Pose(91, 92.25, Math.toRadians(45)).mirror(); // x from 92 to 91, 03142026 15:32 Scoring Pose of our robot. It is facing the goal at a 115 degree angle.
//    private final Pose scoreEnd = new Pose(92, 92.25, Math.toRadians(0)).mirror(); // Scoring Pose of our robot. It is facing the goal at a 115 degree angle.
//    private final Pose readyThirdSpikePickupPose = new Pose(92, 86.25, Math.toRadians(0)).mirror(); // PPG  Highest (First Set) of Artifacts from the Spike Mark.
//    private final Pose readySecondSpikePickupPose = new Pose(92, 62.25, Math.toRadians(0)).mirror(); // PGP Middle (Second Set) of Artifacts from the Spike Mark.
//    private final Pose readyFirstSpikePickupPose = new Pose(92, 36.25, Math.toRadians(0)).mirror(); // GPP Lowest (Third Set) of Artifacts from the Spike Mark.
//    private final Pose readyOpenGatePickupPose = new Pose(124.5,67.5,Math.toRadians(0)).mirror(); // 63--64  PGP Middle (Second Set) of Artifacts from the Spike Mark.
//    private final Pose thirdSpikePickupPose = new Pose(127, 86.25, Math.toRadians(0)).mirror(); // PPG  Highest (First Set) of Artifacts from the Spike Mark.
//    private final Pose secondSpikePickupPose = new Pose(133, 62.25, Math.toRadians(0)).mirror(); // PGP Middle (Second Set) of Artifacts from the Spike Mark.
//    private final Pose secondSpikePickupCP = new Pose(72, 76, Math.toRadians(0)).mirror();
//    private final Pose openGatePickupCP = new Pose(72, 76, Math.toRadians(0)).mirror();
//    private final Pose secondSpikePickupPoseControlPoint = new Pose(125, 64.25, Math.toRadians(0)).mirror(); // PGP Middle (Second Set) of Artifacts from the Spike Mark.
//    private final Pose firstSpikePickupPose = new Pose(134, 36.25, Math.toRadians(0)).mirror(); // GPP Lowest (Third Set) of Artifacts from the Spike Mark.
//    private final Pose openGatePickupPose = new Pose(132.5, 69, Math.toRadians(0)).mirror();  //63--61
//    //    private final Pose PARKPose = new Pose(120, 92.25, Math.toRadians(0)); // GPP Lowest (Third Set) of Artifacts from the Spike Mark.
//    private final Pose offlinePose = new Pose(112, 92.25, Math.toRadians(0)).mirror(); // GPP Lowest (Third Set) of Artifacts from the Spike Mark.
//    // Initialize variables for paths
//    private final Pose offlinePoseofFirst = new Pose(136, 36.25, Math.toRadians(0)); // GPP Lowest (Third Set) of Artifacts from the Spike Mark.
    // Initialize variables for paths
    @Override
    public void runOpMode() {

        /* ---------------- INIT ---------------- */
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
///////////////////////////////////////////////////////
//
//        Robot.alliance = Alliance.RED;
//        Robot.sendHardwareMap(hardwareMap);

        robot.init(hardwareMap);
//        PanelsTelemetry.INSTANCE.init(hardwareMap);
//        robot.imu.resetYaw();
        robot.MasterShooterMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MasterShooterMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.SlaveShooterMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.SlaveShooterMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.BlockageArm.setPosition(blockageblockposition);////switch with blockage case with blockage
//        robot.BlockageArm.setPosition(blockageblockTele);

        robot.BlockageArmL.setPosition(blockageblockposition);
        robot.BlockageArmR.setPosition(blockageblockposition);
        //switch with blockage case with blockage need command
//        follower.setPose(startPose);

//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        telemetry.setMsTransmissionInterval(11);
//        limelight.pipelineSwitch(0);
//        /*
//         * Starts polling for data.
//         */
//        limelight.start();// 需要手动开启吗？
/// ///////////////////////////////////////////////debug PIDF////////////////////
        initShooterPIDF();
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        Telemetry dashboardTelemetry = dashboard.getTelemetry();
//        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
        TelemetryManager dashboardTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();






        // 设置 Dashboard 更新频率
//        dashboard.setTelemetryTransmissionInterval(100); // 100ms 更新一次
//        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
//        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
//        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());



        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

//        telemetry = new MultipleTelemetry(
//                telemetry,
//                dashboardTelemetry
//        );

//        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        pathTimer = new ElapsedTime();
        autoTimer = new ElapsedTime();
        shootTimer= new ElapsedTime();

        follower = Constants.create(hardwareMap);

            pathState = PathState.DRIVE_START_POS_SHOOT_POS;
        autoShootState = AutoShootState.IDLE;
        buildPaths();
//        shooterSubsystem = ShooterSubsystem.getInstance(hardwareMap, gamepad1, gamepad2);
//        flywheelSubsystem = FlywheelSubsystem.getInstance(hardwareMap, gamepad1);
//        feederSubsystem = FeederSubsystem.getInstance(hardwareMap, gamepad1);
//        intakeSubsystem = IntakeSubsystem.getInstance(hardwareMap, gamepad1);
//        shooterSubsystem.init();
//        flywheelSubsystem.init();
//        feederSubsystem.init();
//        intakeSubsystem.init();
//        telemetry.addLine("Ready");
//        telemetry.update();

        /* ---------------- WAIT ---------------- */
        waitForStart();

        autoTimer.seconds();
        shootTimer.seconds();
        follower.setPose(startPose);
        /* ---------------- MAIN LOOP ---------------- */
        while (opModeIsActive() && pathState != PathState.END) {
//            panelsTelemetry.update();
            telemetry.update();
            follower.update();
            updateShooterTelemetry();
//            autoshoot();
            statePathUpdate();

//////////////////////////////////////////////////debug PIDF////////////////////
//            YawPitchRollAngles orientation =robot.imu.getRobotYawPitchRollAngles();
//            limelight.updateRobotOrientation(orientation.getYaw());
//            LLResult llResult= limelight.getLatestResult();
//            if(llResult!=null && llResult.isValid()){
//                Pose3D botPose=llResult.getBotpose_MT2();
//                telemetry.addData("Tx",llResult.getTx());
//                telemetry.addData("Ty",llResult.getTy());
//                telemetry.addData("Ta",llResult.getTa());
//            }




        }
    }

    ////////////////////////////////////////////////////
    private void saveCurrentPosition() {
        if (follower != null) {
            Pose currentPose = follower.pose();
            robot.finalAutoX = currentPose.x();
            robot.finalAutoY = currentPose.y();
            robot.finalAutoHeading = currentPose.heading();
        }
    }
    public void autoIntake() {
        // 吸入
        robot.IntakeMotorL.setPower(intakePowerIntake);
        robot.IntakeMotorR.setPower(intakePowerIntake);

//            robot.BlockageArm.setPosition(blockageblockTele);
        robot.BlockageArmL.setPosition(blockageblockposition);
        robot.BlockageArmR.setPosition(blockageblockposition);


//            robot.MasterShooterMotorL.setPower(ShooterMotorHold);//switch with blockage case with blockage need command
//            robot.SlaveShooterMotorR.setPower(ShooterMotorHold); //switch with blockage case with blockage need command
        telemetry.update();
    }

    enum AutoShootState {
        IDLE,
        SPINNING_UP,
        FEEDING,
        DONE
    }

    public void autoshoot() {
        switch (autoShootState) {

            case IDLE:
//                robot.BlockageArm.setPosition(blockageblockTele);
                robot.BlockageArmL.setPosition(blockageblockposition);
                robot.BlockageArmR.setPosition(blockageblockposition);
                //switch with blockage case with blockage
                startShooter();
                shooterStarted = true;
                isShooterAtSpeed = false;
                shootTimer.seconds();
                autoShootState = AutoShootState.SPINNING_UP;
                break;

            case SPINNING_UP:
//                  robot.BlockageArm.setPosition(blockagereleaseTele);
//release blockage move to start spin
                //switch with blockage case with blockage
//                double currentVelocity = Math.abs(robot.MasterShooterMotorL.getVelocity());//60/(28)
//                double targetVelocity = ShooterPIDFConfig.targetVelocity;
//                if ((!isShooterAtSpeed) && (Math.abs(currentVelocity - targetVelocity) <= ShooterPIDFConfig.toleranceofVelocity)) {
                if ((Math.abs(Math.abs(robot.MasterShooterMotorL.getVelocity()) - ShooterPIDFConfig.targetVelocity) <= ShooterPIDFConfig.toleranceofVelocity)) {
                    robot.BlockageArmL.setPosition(blockagereleaseposition);
                    robot.BlockageArmR.setPosition(blockagereleaseposition);
                    robot.IntakeMotorL.setPower(intakePowerShoot);
                    robot.IntakeMotorR.setPower(intakePowerShoot);
//                    isShooterAtSpeed = true;
                    shootTimer.seconds();
                    autoShootState = AutoShootState.FEEDING;
                }

                break;

            case FEEDING:
                if (shootTimer.seconds()  >= 1.0 ) {
                    // from 1.9---1.6
                    stopShooter();
                    stopIntake();
//                    robot.BlockageArm.setPosition(blockageblockTele);
                    robot.BlockageArmL.setPosition(blockageblockposition);
                    robot.BlockageArmR.setPosition(blockageblockposition);

                    //switch with blockage case with blockage
                    autoShootState = AutoShootState.DONE;
                }
                break;


//            case FEEDING:
//                if (shootTimer.seconds()  >= 2.2) {
//                    stopShooter();
//                    stopIntake();
//                    robot.BlockageArm.setPosition(blockageblockposition);
////                    isShooterAtSpeed = false;
//                    autoShootState = AutoShootState.DONE;
//                }
//                break;

            case DONE:
                // 什么都不做，防止重复执行
                break;
        }
    }

    private void startShooter() {
        robot.IntakeMotorL.setPower(0);
        robot.IntakeMotorR.setPower(0);
        /// /////////////////////////////////double check hood angle////////////
        robot.HoodArmL.setPosition(0.48);
        robot.HoodArmR.setPosition(0.48);
        if (robot.MasterShooterMotorL instanceof DcMotorEx) {
            DcMotorEx shooter = (DcMotorEx) robot.MasterShooterMotorL;
            DcMotorEx shooterR = (DcMotorEx) robot.SlaveShooterMotorR;
            // 直接使用setVelocity，它会使用已配置的PIDF
            shooter.setVelocity(Math.abs(ShooterPIDFConfig.targetVelocity));
            shooterR.setVelocity(Math.abs(ShooterPIDFConfig.targetVelocity));
        }
        shooterStarted=true;

    }


    /// ///////////////////////////////
    @Configurable
    public static class ShooterPIDFConfig {
        public static double kP = 300;     // 比例增益0.10.350.651.0655
        public static double kI = 0.0;      // 积分增益
        public static double kD = 0.0;      // 微分增益
        public static double kF = 15;      // 前馈增益0.050.08
        //        P: 20–60
//        I: 0–0.002
//        D: 0–0.5
//        F: 10–30
//        public static double targetRPM =Close_SHOOTER_TARGET_RPM;
//        public static double targetRPM =Med_SHOOTER_TARGET_RPM; // 目标转速
        public static double targetVelocity =Med_SHOOTER_TARGET_Velocity;
        //        public static double toleranceofRPM = 100;    // 转速容差 5RPM---30TPS
        public static double toleranceofVelocity = 80;
        public static double tolerance = 0;

    }

    private void initShooterPIDF() {
        // 初始化时调用一次
        if (robot.MasterShooterMotorL instanceof DcMotorEx) {
            DcMotorEx shooter = (DcMotorEx) robot.MasterShooterMotorL;

            // 设置PIDF参数
            PIDFCoefficients pidf = new PIDFCoefficients(
                    ShooterPIDFConfig.kP,
                    ShooterPIDFConfig.kI,
                    ShooterPIDFConfig.kD,
                    ShooterPIDFConfig.kF
            );
            shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

            telemetry.addData("Shooter PIDF", "Initialized");
            telemetry.addLine("Shooter PIDF");
            telemetry.addData("kP", pidf.p);
            telemetry.addData("kI", pidf.i);
            telemetry.addData("kD", pidf.d);
            telemetry.addData("kF", pidf.f);
            telemetry.update();

        }
    }


    /**
     * 检查射击电机速度（使用 Dashboard 调整的容差）
     */
//    private void checkShooterVelocity() {
//
//            double currentVelocity = Math.abs(robot.MasterShooterMotorL.getVelocity());
//            double targetVelocity = (ShooterPIDFConfig.targetVelocity); //28*13.7/60
//            // targetVelocity
////            double targetVelocity = (ShooterPIDFConfig.targetRPM)*28/60; //28*13.7/60
//            double toleranceofVelocity = ShooterPIDFConfig.tolerance;
//            double shooterPower = robot.MasterShooterMotorL.getPower();
//            double shooterCurrent = robot.MasterShooterMotorL.getCurrent(CurrentUnit.AMPS); // 如果有电流传感器
//
//            // 检查是否在容差范围内
//            if (Math.abs(Math.abs(currentVelocity) - targetVelocity) <= toleranceofVelocity) {
//                isShooterAtSpeed = true;
//            } else {
//                isShooterAtSpeed = false;
//                fireRequested = false;
//            }
//
//    }


    /**
     * 更新射击系统遥测数据（用于 PIDF 调参）
     */
    private void updateShooterTelemetry() {
        // 射击电机状态
        telemetry.addLine("=== SHOOTER PIDF TUNING ===");
        double currentVelocity = Math.abs(robot.MasterShooterMotorL.getVelocity());
        double currentVelocityR = Math.abs(robot.SlaveShooterMotorR.getVelocity());
//        double targetVelocity = ShooterPIDFConfig.targetVelocity;
//        double tolerance = ShooterPIDFConfig.tolerance;
//        telemetry.addData("VEL raw", robot.MasterShooterMotorL.getVelocity());
//        telemetry.addData("VEL target", ShooterPIDFConfig.targetVelocity);
        telemetry.addData("VEL error", ShooterPIDFConfig.targetVelocity - robot.MasterShooterMotorL.getVelocity());
        telemetry.addData("currentVelocity", currentVelocity);
        telemetry.addData("  shootTimer ", shootTimer.seconds());
        telemetry.addData("currentVelocityR", currentVelocityR);
        telemetry.addData("ShooterPIDFConfig.targetVelocity", ShooterPIDFConfig.targetVelocity);
        telemetry.addData("ShooterPIDFConfig.toleranceofVelocity", ShooterPIDFConfig.toleranceofVelocity);
        telemetry.addData("ShooterPIDFConfig.tolerance", ShooterPIDFConfig.tolerance);
//        double shooterPower = robot.MasterShooterMotorL.getPower();
//        double shooterPowerr = robot.SlaveShooterMotorR.getPower();
//        telemetry.addData("At Speed?", isShooterAtSpeed ? "YES" : "NO");
//        telemetry.addData("ShooterPower", "%.2f", shooterPower);
//        telemetry.addData("ShooterPowerR", "%.2f", shooterPowerr);
        telemetry.addData("state AutoShootState",  autoShootState);
        telemetry.addData("State", pathState);
        telemetry.addData("isShooterAtSpeed", isShooterAtSpeed);
        telemetry.addData("State", pathState);
//        telemetry.addData("Path Time", pathTimer.seconds());
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));

        //        telemetry.addData(" robot.SlaveShooterMotorR.setPower", "%.2f",  (robot.MasterShooterMotorL.getPower()));


        /// /////////////////////////////////////////////////
//        // PIDF 参数值
//        telemetry.addLine("=== PIDF PARAMETERS ===");
//        telemetry.addData("kP", "%.4f", TeleOpQualifier.ShooterPIDFConfig.kP);
////        telemetry.addData("kI", "%.4f", TeleOpQualifier.ShooterPIDFConfig.kI);
////        telemetry.addData("kD", "%.4f", TeleOpQualifier.ShooterPIDFConfig.kD);
//        telemetry.addData("kF", "%.4f", TeleOpQualifier.ShooterPIDFConfig.kF);
//        telemetry.addData("Tolerance", "%.0f RPM", TeleOpQualifier.ShooterPIDFConfig.tolerance);
//        telemetry.addData("MasterShooterMPower", "%.0f", robot.MasterShooterMotorL.getPower());
        // 从电机状态
//        telemetry.addLine("=== SLAVE MOTOR ===");
//        telemetry.addData("Slave Power", "%.2f", robot.SlaveShooterMotorR.getPower());
//        telemetry.addData("Slave RPM", "%.0f", robot.SlaveShooterMotorR.getVelocity());

//        // 射击状态
//        telemetry.addLine("=== SHOOTING STATUS ===");
//        telemetry.addData("Fire Requested", fireRequested ? "YES" : "NO");
//        telemetry.addData("LED Color", isShooterAtSpeed ? "GREEN" : "RED");
    }


    /**
     * 停止射击电机
     */
    private void stopShooter() {
        robot.MasterShooterMotorL.setVelocity(0);
        robot.SlaveShooterMotorR.setPower(0);
        robot.IntakeMotorL.setPower(0);
        robot.IntakeMotorR.setPower(0);
        isShooterAtSpeed = false;
        fireRequested = false;
        shooterStarted=false;
    }

    private void stopIntake() {
        robot.IntakeMotorL.setPower(intakePowerOff);
        robot.IntakeMotorR.setPower(intakePowerOff);
        robot.MasterShooterMotorL.setPower(ShooterMotorOff);
        robot.SlaveShooterMotorR.setPower(ShooterMotorOff);
        // Non-blocking delay to prevent rapid mode switching
        telemetry.addData("intakePowerOff", intakePowerOff);
        telemetry.update();

        shooterStarted=false;
    }



    /// ///////////

    /**
     * 执行射击序列
     */
    private void executeFireSequence() {
        //       if (!isShooterAtSpeed) return; // 安全检查
        telemetry.addData("Shooter", "FIRING! RPM: %.0f", robot.MasterShooterMotorL.getVelocity());
        telemetry.update();
        // 启动拾取电机将球推入射击器
        robot.IntakeMotorL.setPower(intakePowerShoot); // 全功率推出
        robot.IntakeMotorR.setPower(intakePowerShoot); // 全功率推出

        // 保持推球一段时间（根据实际调整）
        ElapsedTime fireTimer = new ElapsedTime();
        while (fireTimer.milliseconds() < 500 && opModeIsActive()) { // 推球500ms
            // 空循环，等待时间到达
            telemetry.addData("Shooter", "Firing... Time: %.0f/500", fireTimer.milliseconds());
            telemetry.update();
        }

        // 停止拾取电机
        robot.IntakeMotorL.setPower(0);
        robot.IntakeMotorR.setPower(0);
        fireRequested = false;

        telemetry.addData("Shooter", "Fire sequence completed");
        telemetry.update();
    }

    /**
     * 更新LED状态显示
     */
    private void updateLEDs() {
        // 检测状态变化，只在变化时更新LED以减少通信负载
        if (isShooterAtSpeed != wasShooterAtSpeed) {
            if (isShooterAtSpeed) {
                setLEDColor(LED_COLOR_READY);
                telemetry.addData("LED Status", "READY (GREEN) - Press A to Fire");
            }  else {
                setLEDColor(LED_COLOR_OFF);
                telemetry.addData("LED Status", "OFF (RED)");
            }
            wasShooterAtSpeed = isShooterAtSpeed;
        }
    }

    /**
     * 设置LED颜色（需要根据你的具体LED硬件实现）
     */
    private void setLEDColor(String color) {
        try {
            // 示例：使用REV Blinkin LED驱动
            if (color.equals("GREEN")) {
                robot.greenLED.setState(true);
                robot.redLED.setState(false);
//                robot.greenLED = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            } else if (color.equals("RED")) {
                robot.greenLED.setState(false);
                robot.redLED.setState(true);
            }

        } catch (Exception e) {
            telemetry.addData("LED Error", "Failed to set color: " + color);
        }
    }

    /**
     * 设置射击电机PIDF系数
     */
//    private void setShooterPIDFCoefficients() {
//        PIDFCoefficients pidf = new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
//        robot.MasterShooterMotorL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
//    }

    private double calculateOptimalSlavePower(double masterPower) {

        double masterSpeed = robot.MasterShooterMotorL.getVelocity();
        double slaveSpeed = robot.SlaveShooterMotorR.getVelocity();
        double speedRatio =Math.abs(slaveSpeed / (masterSpeed + 0.001)) ; // 避免除零

        // 如果从电机速度明显落后，增加功率补偿
        if (speedRatio < 0.9) {
            return masterPower * 1.1;
        }
        // 如果从电机速度明显超前，减少功率
        else if (speedRatio > 1.1) {
            return masterPower * 0.9;
        }
        // 正常范围内使用相同功率
        else {
            return masterPower;
        }

    }


    ///
    ///
    ///
    /// ///////////////////////////////////////////////////
    /* ---------------- PATH BUILDING ---------------- */


        private void buildPaths() {
            // 使用 Paths.line() 创建直线，并用 .linear() 设置航向插值
            // ===== 直线路径（BezierLine → line）=====

            driveStartShoot = line(startPose, shootPose)
                    .linear(startPose.heading(), shootPose.heading());

            driveReadyFirstSpikePickup = line(shootPose, readyFirstSpikePickupPose)
                    .linear(shootPose.heading(), readyFirstSpikePickupPose.heading());

            driveFirstSpikePickup = line(readyFirstSpikePickupPose, firstSpikePickupPose)
                    .linear(readyFirstSpikePickupPose.heading(), firstSpikePickupPose.heading());

            driveFirstSpikePickupShoot = line(firstSpikePickupPose, shootPose)
                    .linear(firstSpikePickupPose.heading(), shootPose.heading());

            driveSecondSpikePickup = line(readySecondSpikePickupPose, secondSpikePickupPose)
                    .linear(readySecondSpikePickupPose.heading(), secondSpikePickupPose.heading());

            driveReadyThirdSpikePickup = line(shootPose, readyThirdSpikePickupPose)
                    .linear(shootPose.heading(), readyThirdSpikePickupPose.heading());

            driveThirdSpikePickup = line(readyThirdSpikePickupPose, thirdSpikePickupPose)
                    .linear(readyThirdSpikePickupPose.heading(), thirdSpikePickupPose.heading());

            driveThirdSpikePickupShoot = line(thirdSpikePickupPose, shootPose)
                    .linear(thirdSpikePickupPose.heading(), shootPose.heading());

            driveReadyOpenGatePickup = line(secondSpikePickupPose, readyOpenGatePickupPose)
                    .linear(secondSpikePickupPose.heading(), readyOpenGatePickupPose.heading());

            driveOpenGatePickup = line(readyOpenGatePickupPose, openGatePickupPose)
                    .linear(readyOpenGatePickupPose.heading(), openGatePickupPose.heading());

            driveOffline = line(shootPose, offlinePose)
                    .linear(shootPose.heading(), offlinePose.heading());

            driveOfflineofFirst = line(shootPose, offlinePose)
                    .linear(shootPose.heading(), offlinePose.heading());

            // ===== 曲线路径（BezierCurve → curve）=====

            driveReadySecondSpikePickup = curve(shootPose, secondSpikePickupCP, readySecondSpikePickupPose)
                    .linear(shootPose.heading(), readySecondSpikePickupPose.heading());

            driveOpenGatePickupShoot = curve(openGatePickupPose, openGatePickupCP, shootPose)
                    .linear(openGatePickupPose.heading(), shootPose.heading());





//    private void buildPaths() {
//        driveStartShoot = follower.pathBuilder()
//                .addPath(new BezierLine(startPose, shootPose))
//                .setLinearHeadingInterpolation(startPose.heading(), shootPose.heading())
//                .build();


//        driveReadyFirstSpikePickup = follower.pathBuilder()
//                .addPath(new BezierLine(shootPose, readyFirstSpikePickupPose))
//                .setLinearHeadingInterpolation(shootPose.heading(), readyFirstSpikePickupPose.heading())
//                .build();
////        driveReadyFirstPickup = follower.pathBuilder()
////                .addPath(new BezierCurve(shootPose, firstPickupCP, readyFirstPickupPose))
////                .setLinearHeadingInterpolation(shootPose.heading(), readyFirstPickupPose.heading())
////                .build();
//        driveFirstSpikePickup = follower.pathBuilder()
//                .addPath(new BezierLine(readyFirstSpikePickupPose, firstSpikePickupPose))
//                .setLinearHeadingInterpolation(readyFirstSpikePickupPose.heading(), firstSpikePickupPose.heading())
//                .build();
//        driveFirstSpikePickupShoot = follower.pathBuilder()
//                .addPath(new BezierLine(firstSpikePickupPose, shootPose))
//                .setLinearHeadingInterpolation(firstSpikePickupPose.heading(), shootPose.heading())
//                .build();
//        driveReadySecondSpikePickup = follower.pathBuilder()
//                .addPath(new BezierCurve(shootPose, secondSpikePickupCP,readySecondSpikePickupPose))
//                .setLinearHeadingInterpolation(shootPose.heading(), readySecondSpikePickupPose.heading())
//                .build();
//        driveSecondSpikePickup = follower.pathBuilder()
//                .addPath(new BezierLine(readySecondSpikePickupPose, secondSpikePickupPose))
//                .setLinearHeadingInterpolation(readySecondSpikePickupPose.heading(), secondSpikePickupPose.heading())
//                .build();
////        driveSecondPickupShoot = follower.pathBuilder()
////                .addPath(new BezierCurve(secondPickupPose, secondPickupCP,shootPose))
////                .setLinearHeadingInterpolation(secondPickupPose.heading(), shootPose.heading())
////                .build();
////        driveSecondPickupOpengate = follower.pathBuilder()
////                .addPath(new BezierCurve(secondPickupPose, secondPickupCP,shootPose))
////                .setLinearHeadingInterpolation(secondPickupPose.heading(), shootPose.heading())
////                .build();
//
//        driveReadyThirdSpikePickup = follower.pathBuilder()
//                .addPath(new BezierLine(shootPose, readyThirdSpikePickupPose))
//                .setLinearHeadingInterpolation(shootPose.heading(), readyThirdSpikePickupPose.heading())
//                .build();
//        driveThirdSpikePickup = follower.pathBuilder()
//                .addPath(new BezierLine(readyThirdSpikePickupPose, thirdSpikePickupPose))
//                .setLinearHeadingInterpolation(readyThirdSpikePickupPose.heading(), thirdSpikePickupPose.heading())
//                .build();
//        driveThirdSpikePickupShoot = follower.pathBuilder()
//                .addPath(new BezierLine(thirdSpikePickupPose, shootPose))
//                .setLinearHeadingInterpolation(thirdSpikePickupPose.heading(), shootPose.heading())
//                .build();
//
//        driveReadyOpenGatePickup = follower.pathBuilder()
//                .addPath(new BezierLine(secondSpikePickupPose, readyOpenGatePickupPose))
//                .setLinearHeadingInterpolation(secondSpikePickupPose.heading(), readyOpenGatePickupPose.heading())
//                .build();
//        driveOpenGatePickup = follower.pathBuilder()
//                .addPath(new BezierLine(readyOpenGatePickupPose, openGatePickupPose))
//                .setLinearHeadingInterpolation(readyOpenGatePickupPose.heading(), openGatePickupPose.heading())
//                .build();
//        driveOpenGatePickupShoot = follower.pathBuilder()
//                .addPath(new  BezierCurve(openGatePickupPose,openGatePickupCP,shootPose))
//                .setLinearHeadingInterpolation(openGatePickupPose.heading(), shootPose.heading())
//                .build();
//
//        driveOffline = follower.pathBuilder()
//                .addPath(new BezierLine(shootPose, offlinePose))
//                .setLinearHeadingInterpolation(shootPose.heading(), offlinePose.heading())
//                .build();
//
//        driveOfflineofFirst = follower.pathBuilder()
//                .addPath(new BezierLine(shootPose, offlinePose))
//                .setLinearHeadingInterpolation(shootPose.heading(), offlinePose.heading())
//                .build();
    }

    /* ---------------- STATE MACHINE ---------------- */
    /// /////////////////////////keep working here/////////////////////////12172025///////////
    private void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.seconds();
    }
    private void statePathUpdate() {
        switch (pathState) {
            case DRIVE_START_POS_SHOOT_POS:
                follower.follow(driveStartShoot);
                setPathState(PathState.DRIVE_TO_SHOOT_WAIT);
                saveCurrentPosition();
                break;

            case DRIVE_TO_SHOOT_WAIT:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                saveCurrentPosition();
                break;

            case SHOOT_PRELOAD:
                autoshoot();
                if (autoShootState == AutoShootState.DONE) {
                    follower.follow(driveReadySecondSpikePickup);
                    isShooterAtSpeed = false;
                    setPathState(PathState.DRIVE_READY_SECOND_SPIKE_PICKUPT_POS);
                }
                saveCurrentPosition();
                break;

            case DRIVE_READY_SECOND_SPIKE_PICKUPT_POS:
                if (!follower.isBusy()) {
                    follower.follow(driveSecondSpikePickup);
                    setPathState(PathState.SECOND_SPIKE_PICKUP);
                }
                saveCurrentPosition();
                break;
/// /////////////
            case SECOND_SPIKE_PICKUP:
                if (pathTimer.seconds() < 1.3) {//from 1.5-1.4 1.8-- 1.5 03122026
                    autoIntake();
                } else {
                    follower.follow(driveReadyOpenGatePickup);
                    stopShooter();
                    stopIntake();
                    setPathState(PathState.DRIVE_READY_OPEN_GATE_POS);
                }
                saveCurrentPosition();
                break;

            case DRIVE_READY_OPEN_GATE_POS:
                if (!follower.isBusy()) {
                    follower.follow(driveOpenGatePickup);
                    setPathState(PathState.OPEN_GATE);
                }
                saveCurrentPosition();
                break;

            case OPEN_GATE:
                if (pathTimer.seconds() < 1.2) {
//                    autoIntake();
                } else {
                    follower.follow(driveOpenGatePickupShoot);
//                    stopShooter();
//                    stopIntake();
                    setPathState(PathState.DRIVE_BACK_SECOND_SPIKE_PICKUPT_POS);
                }
                saveCurrentPosition();
                break;

            case DRIVE_BACK_SECOND_SPIKE_PICKUPT_POS:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_SECOND_SPIKE_PICKUP);
                    autoShootState = AutoShootState.IDLE;
                }
                saveCurrentPosition();
                break;

            case SHOOT_SECOND_SPIKE_PICKUP:
                autoshoot();
                if (autoShootState == AutoShootState.DONE) {
                    follower.follow(driveReadyThirdSpikePickup);
                    isShooterAtSpeed = false;
                    setPathState(PathState.DRIVE_READY_THIRD_SPIKE_PICKUPT_POS);
                }
                saveCurrentPosition();
                break;
/// ///////////////////////////
            case DRIVE_READY_THIRD_SPIKE_PICKUPT_POS:
                if (!follower.isBusy()) {
                    follower.follow(driveThirdSpikePickup);
                    setPathState(PathState.THIRD_SPIKE_PICKUP);
                }
                saveCurrentPosition();
                break;

            case THIRD_SPIKE_PICKUP:
                if (!firstPickupCompleted && pathTimer.seconds() < 1.3) {
                    autoIntake();
                } else {
                    follower.follow(driveThirdSpikePickupShoot);
                    stopShooter();
                    stopIntake();
                    setPathState(PathState.DRIVE_BACK_THIRD_SPIKE_PICKUPT_POS);
                    firstPickupCompleted = true;
                }
                saveCurrentPosition();
                break;

            case DRIVE_BACK_THIRD_SPIKE_PICKUPT_POS:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_THIRD_SPIKE_PICKUP);
                    autoShootState = AutoShootState.IDLE;
                }
                saveCurrentPosition();
                break;

            case SHOOT_THIRD_SPIKE_PICKUP:
                autoshoot();
                if (autoShootState == AutoShootState.DONE) {
                    follower.follow(driveReadyFirstSpikePickup);
                    isShooterAtSpeed = false;
                    setPathState(PathState.DRIVE_READY_FIRST_SPIKE_PICKUP_POS);
                }
                saveCurrentPosition();
                break;

            case DRIVE_READY_FIRST_SPIKE_PICKUP_POS:
                if (!follower.isBusy()) {
                    follower.follow(driveFirstSpikePickup);
                    setPathState(PathState.FIRST_SPIKE_PICKUP);
                }
                saveCurrentPosition();
                break;

            case FIRST_SPIKE_PICKUP:
                if (pathTimer.seconds() < 1.3) {
                    autoIntake();
                } else {
                    follower.follow(driveFirstSpikePickupShoot);
                    stopShooter();
                    stopIntake();
                    setPathState(PathState.DRIVE_BACK_FIRST_SPIKE_SHOOT_POS);
                }
                saveCurrentPosition();
                break;

            case DRIVE_BACK_FIRST_SPIKE_SHOOT_POS:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_FIRST_SPIKE_PICKUP);
                    autoShootState = AutoShootState.IDLE;
                }
                saveCurrentPosition();
                break;

            case SHOOT_FIRST_SPIKE_PICKUP:
                autoshoot();
                if (autoShootState == AutoShootState.DONE) {
                    follower.follow(driveOffline);
                    isShooterAtSpeed = false;
                    setPathState(PathState.DRIVE_OFFLINE);
                }
                saveCurrentPosition();
                break;

            case DRIVE_OFFLINE:
                if (!follower.isBusy()) {
                    setPathState(PathState.END);
                }
                saveCurrentPosition();
                break;


        }
    }
}
