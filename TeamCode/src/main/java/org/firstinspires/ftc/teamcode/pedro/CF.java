//package org.firstinspires.ftc.teamcode.pedroPathing;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.controller.PIDFController;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
////import com.qualcomm.robotcore.hardware.Gamepad;
////import com.qualcomm.robotcore.hardware.HardwareMap;
////import com.qualcomm.robotcore.hardware.CRServoImplEx;
////import com.arcrobotics.ftclib.controller.PIDFController;
////import com.arcrobotics.ftclib.hardware.motors.Motor;
////import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
////import org.firstinspires.ftc.teamcode.commandbase.commands.CancelCommand;
////import org.firstinspires.ftc.teamcode.commandbase.commands.SetIntake;
////import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
////import org.firstinspires.ftc.teamcode.globals.SolverLogger;
////import org.firstinspires.ftc.teamcode.commandbase.commands.StationaryAimbotFullLaunch;
////import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
////import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
////import org.firstinspires.ftc.teamcode.globals.Constants;
//
//////import org.firstinspires.ftc.teamcode.globals.Robot;
////import org.firstinspires.ftc.teamcode.commandbase.commands.ClearLaunch;
////import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;
////import static org.firstinspires.ftc.teamcode.globals.Constants.*;
////import com.acmerobotics.dashboard.FtcDashboard;
////import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
////import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
////import com.qualcomm.robotcore.util.ElapsedTime;
//
//
//@Config  // 添加这个注解，让 Dashboard 可以调整参数
//@TeleOp(name = "AAA Unionville Community Fair Chassis")
//// backuo did not changed it at all working on turret testing
////Y shooter idle of 0.6 target speed 03122026
////From LEGION
//// working on turret, and hood check speed fixed
//public class CF extends LinearOpMode {
//
//
//    public float DriveTrains_ReducePOWER=0.5f;
//    public float DriveTrains_smoothTurn=0.5f;
//    private boolean useFieldCentric = true;
//    Gamepad currentGamepad1 = new Gamepad();
//    Gamepad currentGamepad2 = new Gamepad();
//    Gamepad previousGamepad1 = new Gamepad();
//    Gamepad previousGamepad2 = new Gamepad();
//    private boolean lastY = false;
//    private boolean lastX = false;
//    private boolean servoToggle=false;
//    HardwareCF robot = new HardwareCF();
////    TelemetryData telemetryData = new TelemetryData(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
//
//    //    private final Robot solverrobot = Robot.getInstance();
//    //    public String fieldOrRobotCentric = "robot";
//    public String fieldOrRobotCentric = "field";
//    private double powerMultiplier = 0.9;
//    boolean move = false;
//    private static boolean far = false;
//    private static boolean near = false;
//
//    boolean PIDFTimerStart=true;
//    int controlMode = 1;
//
//    private static double Med_SHOOTER_TARGET_SPEED = 1100;  // 1100 is good for near shoot
//    private static double Med_SHOOTER_TARGET_SPEED_I = 1050; // 1.2,3
//    private static double Med_SHOOTER_TARGET_SPEED_IV = 1100; //4   1100 is good for near shoot
//    public float  intakePowerShoot=0.85f;
//    public double servoPosition=robot.HoodArmsetPosition;
//    //////////////////////////////////////////////////////////////////////////////////////////////////
//    ///     private static final double Far_SHOOTER_TARGET_Velocity = 1400;
//    public float  intakePowerIntake=0.95f;//push blocker too much from 99-90
//    public float  intakePowerDump=-0.65f;
//    public float  intakePowerOff=0.0f;
//    public float  ShooterMotorShootFar=0.95f;
//    public float  ShooterMotorShootMed=-0.8f;
//    public float  ShooterMotorShootClose=-0.8f;
//    public float  ShooterMotorHold=-0.2f;
//    public float  ShooterMotorClean=-0.8f;
//    public float  ShooterMotorOff=0.0f;
//    public static final double blockageblockposition=0.10; //for auto
//    public static final double blockagereleaseposition=0.18;
//
//    public static final double SERVO_STEP=-0.005;
//    public static final double blockageblockTele=0.1; // from .18 -0.1 for tele
//    public static final double blockagereleaseTele=0.24;
//
//    ///////////////turret///////////
//    private boolean previousDpadUp = false;
//    private double changeTargetRotationAngle;
//
//    private double turretAngle = Math.PI / 2.0;
//    private double turretSetpoint = 0.0;
//    private double turretPower = 0.0;
//    private Follower follower;
//    //    private PinpointLocalizer pinpointLocalizer;
//    private PIDFController pidfController;
//    private Pose robotPose;
//    private Pose  currentPose;
////    private Methods methods;
////    private RTPAxon.Direction direction;
////    private final Gamepad gamepad1;
////    public static final double GEAR_RATIO = .5;
////    private DriveSubsystem driveSubsystem;
////    private static TurretSubsystem instance;
//    /// ////////////////////////////////////////////////////////////////
////    public static final double HoodArmfarposition=0.3;
////    public static final double HoodArmcloseposition=0.35;
//    public static final double HoodArmPositionInit = 0.1;
//    public static final double HoodArmPositionCloseShoot = 0.3;
//    public static final double HoodArmPositionMedShoot = 0.2;
//    public static final double HoodArmPositionFarShoot = 0.1;
//    private static final double VELOCITY_TOLERANCE = 30; // RPM容差，可根据测试调整
//    // 状态变量
//    private boolean isShooterAtSpeed = false;
//    private boolean shooterIsOn = false; //1
//    private boolean intakeIsOn = false;
//    private boolean shooterIsOnSpeend= false;
//    private boolean wasShooterAtSpeed = false; // 用于7检测状态变化
//    private boolean fireRequested = false;
//    // LED颜色常量（根据你的LED库调整）
//    private final String LED_COLOR_READY = "GREEN";
//    private final String LED_COLOR_ACCELERATING = "YELLOW";
//    private final String LED_COLOR_OFF = "RED";
//    // RPM = (TPS * 60秒) / 每转ticks数  return (tps * 60.0) / ticksPerRevolution;  28*13.7
//    private static final double Close_SHOOTER_TARGET_RPM = 800;//  400RPM---2,557.33333333333333
//    private static final double Far_SHOOTER_TARGET_RPM = 2237;  //  350RPM---2237
//    public static double toleranceforShoot = 100;        // 转速容差
//    //  private static final double Med_SHOOTER_TARGET_RPM = 1300;   //1598 white tri a little bit too far//  250RPM---1586.67
//    //    private static final double Med_SHOOTER_TARGET_RPM = 1300;   //1598 white tri a little bit too far//  250RPM---1586.67
//    // use rpm as speed, the real name should be speed = 1300      //1300
//
//    private ElapsedTime imuResetTimer = new ElapsedTime();
//    private boolean imuResetInCooldown = false;
//    private static final long IMU_RESET_COOLDOWN_MS = 300; // 1秒冷却时间
//    //    public DcMotorEx shooter =  robot.MasterShooterMotorL;
////    public DcMotorEx shooterR = (DcMotorEx) robot.SlaveShooterMotorR;
////    ButtonHandler dpadDownHandler = new ButtonHandler();
////    ButtonHandler dpadUpHandler = new ButtonHandler();
////    ButtonHandler dpadLeftHandler = new ButtonHandler();
////    ButtonHandler dpadRightHandler = new ButtonHandler();
////    ButtonHandler leftBumperHandler = new ButtonHandler();
////    ButtonHandler rightBumperHandler = new ButtonHandler();
//    ////    ButtonHandler leftTriggerHandler = new ButtonHandler();
//    ////    ButtonHandler rightTriggerHandler = new ButtonHandler();
////    ButtonHandler gamepad1XHandler = new ButtonHandler();
////    ButtonHandler gamepad1BHandler = new ButtonHandler();
////    ButtonHandler gamepad1YHandler = new ButtonHandler();
////    ButtonHandler gamepad1AHandler = new ButtonHandler();
////    ButtonHandler gamepad1BackHandler = new ButtonHandler();
//    private volatile boolean isRunning = true;
//    ElapsedTime delayTimer = new ElapsedTime();
//    ElapsedTime PIDFTimer = new ElapsedTime();
//    public enum shooterstatus {
//        OFF,
//        IDLE,
//        SHOOT_SPEED
//    }
//
//    // 计时器
//    private ElapsedTime runtime = new ElapsedTime();
//    @Override
//    public void runOpMode() {
//        robot.init(hardwareMap);
////        initShooterPIDF();
////////////////////////////////////////////////Alliance changing here
////        robot.alliance = Alliance.BLUE;
////        robot.alliance = Alliance.RED;
////        follower = Constants.createFollower(hardwareMap);
////        PinpointConstants localizerConstants = new PinpointConstants();
////        pinpointLocalizer = new PinpointLocalizer(hardwareMap,localizerConstants);
//        robot.leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//
//
////        Constants.driveConstants.setUseBrakeModeInTeleOp(true);
////        follower = Constants.createFollower(hardwareMap);
////        Pose startPose = new Pose(70, 70, 0);  // 或其他起始坐标
////        Pose startPose = new Pose(112, 92.25, 0);//Red near
////        Pose startPose = new Pose(104, 12.25, 0);//Red far
////        Pose startPose = new Pose(32, 92.25, 0);//Blue near
////        Pose startPose = new Pose(40, 12.25, 0);//Blue far
//
//        //auto off line near 112, 92.25  far 104, 12.25,//near oor far
//
////        follower.setStartingPose(startPose == null ? new Pose() : startPose);
////        follower.update();
//////        pinpointLocalizer.update();
////
////        pidfController = new PIDFController(
////                TurretConstants.kP,
////                TurretConstants.kI,
////                TurretConstants.kD,
////                TurretConstants.kF
////        );
//
////        if (gamepad1.dpad_right) {
////            solverrobot.turret.setTurret(Turret.TurretState.GOAL_LOCK_CONTROL, 0);
////        } else if(gamepad1.dpad_left){
////            solverrobot.turret.setTurret(Turret.TurretState.OFF, 0);
////        }
////        robot.axonTurretArmL.setTargetRotation(0);
////        robot.axonTurretArmL.setMaxPower(0.5);
////        robot.axonTurretArmL.setPidCoeffs(0.000002, 0.0000, 0.000);
////        robot.axonTurretArmL.setTargetRotation(0);
////        robot.axonTurretArmL.setMaxPower(0.5);  // Limit max power to 50%
////        robot.axonTurretArmR.setMaxPower(0.5);  // Limit max power to 50%
////        robot.axonTurretArmR.setPidCoeffs(0.02, 0.0005, 0.0025);
//
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        FtcDashboard Dashboard = FtcDashboard.getInstance();
//        Telemetry dashboardTelemetry = Dashboard.getTelemetry();
//        // 设置 Dashboard 更新频率
//        Dashboard.setTelemetryTransmissionInterval(100); // 100ms 更新一次
//
//        waitForStart();
//        runtime.reset();
//        delayTimer.reset();
//        PIDFTimer.reset();
//
//        while (opModeIsActive()) {
//
//            boolean currentY = gamepad1.y;
//            boolean currentX = gamepad1.x;
//            // Store the gamepad values from the previous loop iteration in
//            // previousGamepad1/2 to be used in this loop iteration.
//            // This is equivalent to doing this at the end of the previous
//            // loop iteration, as it will run in the same order except for
//            // the first/last iteration of the loop.
//            previousGamepad1.copy(currentGamepad1);
//            previousGamepad2.copy(currentGamepad2);
//
//            // Store the gamepad values from this loop iteration in
//            // currentGamepad1/2 to be used for the entirety of this loop iteration.
//            // This prevents the gamepad values from changing between being
//            // used and stored in previousGamepad1/2.
//            currentGamepad1.copy(gamepad1);
//            currentGamepad2.copy(gamepad2);
//
//            handleDriveModeToggle();
//            handlePowerAdjustment();
////            handleIntakeToggle();
//
//            telemetry.update();
//
//
//
//        } //end of while loop
//
//    } //end of run mode
//
//    /// //////////////////////////////////////////////////////////////////////////
//
//
////    private void updateLEDs() {
////        // 检测状态变化，只在变化时更新LED以减少通信负载
//////        if (isShooterAtSpeed) {
////        if (isShooterAtSpeed) {
////            setLEDColor(LED_COLOR_READY);
////            telemetry.addData("LED Status", "READY (GREEN) - Press A to Fire");
////        }  else {
////            setLEDColor(LED_COLOR_OFF);
////            telemetry.addData("LED Status", "OFF (RED)");
////        }
//////            wasShooterAtSpeed = false;
//////        }
////    }
//
//    /**
//     * 设置LED颜色（需要根据你的具体LED硬件实现）
//     */
////    private void setLEDColor(String color) {
////        try {
////            // 示例：使用REV Blinkin LED驱动
////            if (color.equals("GREEN")) {
////                robot.greenLED.setState(false);
////                robot.redLED.setState(true);
//////                robot.greenLED = RevBlinkinLedDriver.BlinkinPattern.GREEN;
////            } else if (color.equals("RED")) {
////                robot.greenLED.setState(true);
////                robot.redLED.setState(false);
////            }
////
////        } catch (Exception e) {
////            telemetry.addData("LED Error", "Failed to set color: " + color);
////        }
////    }
//
//
//    private void handleDriveModeToggle() {
//        if (currentGamepad1.y && !previousGamepad1.y) {
//            useFieldCentric = !useFieldCentric;
//        }
//
//        if (useFieldCentric) {
//            updateDrivetrain_FieldCentric();
//            telemetry.addData("Mode", "FieldCentric");
//        } else {
//            updateDrivetrain_RobotCentric();
//            telemetry.addData("Mode", "RobotCentric");
//        }
//
//    }
//
//    // ========== 功能2：Dpad 上下调节 DriveTrains_ReducePOWER ==========
//    private void handlePowerAdjustment() {
//        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
//            DriveTrains_ReducePOWER += 0.1;
//        }
//        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
//            DriveTrains_ReducePOWER -= 0.1;
//        }
//        telemetry.addData("DriveTrains_ReducePOWER", DriveTrains_ReducePOWER);
//
//    }
//
//    // ========== 功能3：Left Trigger 切换 IntakeServo ==========
////    private void handleIntakeToggle() {
////        if (currentGamepad1.left_trigger > 0.3 && !(previousGamepad1.left_trigger > 0.3)) {
////            servoToggle = !servoToggle;
////        }
////
////        if (servoToggle) {
////            robot.IntakeServo.setPower(0.8);
////            telemetry.addData("IntakeServo", "ON (0.8)");
////        } else {
////            robot.IntakeServo.setPower(0.0);
////            telemetry.addData("IntakeServo", "OFF (0.0)");
////        }
////
////    }
//
//    /// ///////////////////////joy stick reversed for no reason//// on 03112026/////////////
//    public void updateDrivetrain_FieldCentric() {
//        /// ////////////follower issue/////////
//        /// ////////////////03102026 switch left and right reverse for no reason， after auto everything reversed。
//
//        double y = -gamepad1.left_stick_y * (1); // Remember, Y stick value is reversed
//        double x = +gamepad1.left_stick_x * (1);
//        double rx = +gamepad1.right_stick_x * DriveTrains_smoothTurn; //*(0.5) is fine
//
//        // This button choice was made so that it is hard to hit on accident,
//        // it can be freely changed based on preference.
//        // The equivalent button is start on Xbox-style controllers.
//// ******************************************temp
////        if (gamepad1.back) {
////            robot.imu.resetYaw();
////        }
////******************************************temp
//
//        double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//
//        // Rotate the movement direction counter to the bot's rotation
//        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//
//        rotX = rotX * 1.1;  // Counteract imperfect strafing
//
//        // Denominator is the largest motor power (absolute value) or 1
//        // This ensures all the powers maintain the same ratio,
//        // but only if at least one is out of the range [-1, 1]
//        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//        double frontLeftPower = (rotY + rotX + rx) / denominator;
//        double backLeftPower = (rotY - rotX + rx) / denominator;
//        double frontRightPower = (rotY - rotX - rx) / denominator;
//        double backRightPower = (rotY + rotX - rx) / denominator;
//
//        robot.leftFrontMotor.setPower(frontLeftPower * DriveTrains_ReducePOWER);
//        robot.leftRearMotor.setPower(backLeftPower * DriveTrains_ReducePOWER);
//        robot.rightFrontMotor.setPower(frontRightPower * DriveTrains_ReducePOWER);
//        robot.rightRearMotor.setPower(backRightPower * DriveTrains_ReducePOWER);
//    }
//
//////////////////////////////////////////////////////////////////////////////////////////////////
////    public void updateDrivetrain_FieldCentric() {
////        double y = gamepad1.left_stick_y * (1); // Remember, Y stick value is reversed
////        double x = -gamepad1.left_stick_x * (1);
////        double rx = -gamepad1.right_stick_x * DriveTrains_smoothTurn; //*(0.5) is fine
////
////        // This button choice was made so that it is hard to hit on accident,
////        // it can be freely changed based on preference.
////        // The equivalent button is start on Xbox-style controllers.
////// ******************************************temp
//////        if (gamepad1.back) {
//////            robot.imu.resetYaw();
//////        }
//////******************************************temp
////
////        double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
////
////        // Rotate the movement direction counter to the bot's rotation
////        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
////        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
////
////        rotX = rotX * 1.1;  // Counteract imperfect strafing
////
////        // Denominator is the largest motor power (absolute value) or 1
////        // This ensures all the powers maintain the same ratio,
////        // but only if at least one is out of the range [-1, 1]
////        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
////        double frontLeftPower = (rotY + rotX + rx) / denominator;
////        double backLeftPower = (rotY - rotX + rx) / denominator;
////        double frontRightPower = (rotY - rotX - rx) / denominator;
////        double backRightPower = (rotY + rotX - rx) / denominator;
////
////        robot.leftFrontMotor.setPower(frontLeftPower * DriveTrains_ReducePOWER);
////        robot.leftRearMotor.setPower(backLeftPower * DriveTrains_ReducePOWER);
////        robot.rightFrontMotor.setPower(frontRightPower * DriveTrains_ReducePOWER);
////        robot.rightRearMotor.setPower(backRightPower * DriveTrains_ReducePOWER);
////    }
////
////
//    /////////////////////////////////////////////////////////////////////////////////////////
//    public void updateDrivetrain_RobotCentric() {
//        double robot_y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
//        double robot_x = gamepad1.left_stick_x;
//        double robot_rx = gamepad1.right_stick_x*DriveTrains_smoothTurn; // If a smooth turn is required 0.5
//
//        double fl = robot_y - robot_x - robot_rx;
//        double bl = robot_y + robot_x - robot_rx;
//        double fr = robot_y + robot_x + robot_rx;
//        double br = robot_y - robot_x + robot_rx;
//
//        robot.leftFrontMotor.setPower(fl * DriveTrains_ReducePOWER);
//        robot.leftRearMotor.setPower(bl * DriveTrains_ReducePOWER);
//        robot.rightFrontMotor.setPower(fr * DriveTrains_ReducePOWER);
//        robot.rightRearMotor.setPower(br * DriveTrains_ReducePOWER);
//
//    }
//
////    private void handleIMUReset() {
////        // 检查冷却状态
////        if (imuResetInCooldown) {
////            if (imuResetTimer.milliseconds() > IMU_RESET_COOLDOWN_MS) {
////                imuResetInCooldown = false;
////            } else {
////                return; // 冷却中，不处理
////            }
////        }
////
////        // 使用 gamepad1.back 键重设IMU
////        if (gamepad1.back) {
////            robot.imu.resetYaw();
////
////            // 记录状态和开始冷却
////            imuResetInCooldown = true;
////            imuResetTimer.reset();
////
////            // 提供视觉反馈
////            telemetry.addLine("=== IMU RESET ===");
////            telemetry.addData("Status", "Yaw angle reset to 0°");
////            telemetry.addData("Current Heading", "%.1f°",
////                    robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
////            telemetry.addData("Cooldown", "%.1fs", IMU_RESET_COOLDOWN_MS / 1000.0);
////
////            // 震动反馈
////            gamepad1.rumble(300);
////        }
////    }
//
////Begin Definition and Initialization of steptestservo()
//// Begin debugging with a step increment of 0.05  SGC - servoGamepadControl
////        public void servoGamepadControl() {
//
///**
// * This code snippet controls the position of a servo motor using the gamepad triggers.
// *
// * **Purpose**:
// * - The left trigger (`gamepad1.left_trigger`) increases the servo's position by a fixed step (`SERVO_STEP`).
// * - The right trigger (`gamepad1.right_trigger`) decreases the servo's position by a fixed step (`SERVO_STEP`).
// * - The servo position is constrained between 0.01 (minimum) and 0.99 (maximum) to prevent invalid values.
// * - The current servo position is displayed on the telemetry for real-time monitoring.
// *
// * **Usage Instructions**:
// * 1. Press the **left trigger** (`gamepad1.left_trigger`) to move the servo incrementally towards its maximum position.
// * 2. Press the **right trigger** (`gamepad1.right_trigger`) to move the servo incrementally towards its minimum position.
// * 3. The servo's position is updated with a small delay (`sleep(200)` milliseconds) to prevent rapid changes from multiple trigger presses.
// * 4. Adjust `SERVO_STEP` as needed to control the increment size for finer or coarser adjustments.
// *
// * **Setup**:
// * - Ensure the servo is connected to the correct port and initialized in the `robot.TServo` variable.
// * - Configure the `SERVO_STEP` variable to determine how much the position changes with each trigger press.
// * - Calibrate the servo movement range (e.g., 0.01 to 0.99) based on your servo's physical limits to avoid damage.
// */
//
//
////            if (gamepad1.left_trigger > 0.3) {
////                servoPosition = servoPosition + SERVO_STEP;
////                if (servoPosition >= 1.0) {
////                    servoPosition = 0.99; // 限制最大值
////                }
////                robot.TServo.setPosition(servoPosition);
////                telemetry.addData("Servo Position", servoPosition);
////                telemetry.update();
////                sleep(200);
////            }
////            if (gamepad1.right_trigger > 0.3) {
////                servoPosition = servoPosition - SERVO_STEP;
////                if (servoPosition <= 0.0) {
////                    servoPosition = 0.01; // 限制最小值
////                }
////                robot.TServo.setPosition(servoPosition);
////                telemetry.addData("Servo Position", servoPosition);
////                telemetry.update();
////                sleep(200);
////            }
//
////End debugging with a step increment of 0.05
//
////        }
//
/////////////////////End Definition and Initialization of steptestservo()
////    Begin Definition and Initialization of steptestservo()
//// Begin debugging with a step increment of 0.05  SGC - servoGamepadControl
////        public void servoGamepadControl() {
////            if (gamepad1.dpadLeftWasPressed()) {
////                servoPosition = servoPosition + SERVO_STEP;
////                if (servoPosition >= 1.0) {
////                    servoPosition = 0.99; // 限制最大值
////                }
////                robot.HoodArmL.setPosition(servoPosition);
////                robot.HoodArmR.setPosition(servoPosition);
////                telemetry.addData("Servo Position", servoPosition);
////                telemetry.update();
////                sleep(200);
////            }
////            if (gamepad1.dpadRightWasPressed()) {
////                servoPosition = servoPosition - SERVO_STEP;
////                if (servoPosition <= 0.0) {
////                    servoPosition = 0.01; // 限制最小值
////                }
////                robot.HoodArmL.setPosition(servoPosition);
////                robot.HoodArmR.setPosition(servoPosition);
////                telemetry.addData("Servo Position", servoPosition);
////                telemetry.update();
////                sleep(200);
////            }
////
//////End debugging with a step increment of 0.05
////
////        }
//
//
//
//
//
//}
//
//
//
//
//
//
//
//
