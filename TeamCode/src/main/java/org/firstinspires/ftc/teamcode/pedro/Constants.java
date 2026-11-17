
package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;




//package org.firstinspires.ftc.teamcode.pedroPathing;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.follower.FollowerConstants;
//import com.pedropathing.ftc.FollowerBuilder;
//import com.pedropathing.ftc.drivetrains.MecanumConstants;
//import com.pedropathing.ftc.localization.constants.PinpointConstants;
//import com.pedropathing.paths.PathConstraints;
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("LFMotor");
        c.backLeftName.set("LBMotor");
        c.frontRightName.set("RFMotor");
        c.backRightName.set("RBMotor");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("odo");
        c.xPodOffset.set(-8.033);
        c.yPodOffset.set(-1.9685);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        c.maxAchievableForwardVelocity.set(64.599);
        c.maxAchievableStrafeVelocity.set(51.33016);
        c.naturalForwardDeceleration.set(32.17968);   // was -32.17968 in 2.x
        c.naturalStrafeDeceleration.set(59.39382);    // was -59.39382 in 2.x

        // Required, and with no 2.x equivalent — get these from AutoTune:
        // c.headingFeedback.set(...);
        // c.forwardTranslational.set(...);
        // c.strafeTranslational.set(...);
        // c.brake.set(...);
        // c.coast.set(...);
        // c.linearBrakeCoefficients.set(...);
        // c.quadraticBrakeCoefficients.set(...);
        // c.headingBrakeCoefficients.set(...);
    });

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
//package org.firstinspires.ftc.teamcode.pedroPathing;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.follower.FollowerConstants;
//import com.pedropathing.ftc.FollowerBuilder;
//import com.pedropathing.ftc.drivetrains.MecanumConstants;
//import com.pedropathing.ftc.localization.constants.PinpointConstants;
//import com.pedropathing.paths.PathConstraints;
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//
//
//public class Constants {
//
//    // 1. Follower 算法参数（Foresight 调参结果）
//    public static FollowerConstants followerConstants = new FollowerConstants()
//            .mass(12.11)
//            .lateralZeroPowerAcceleration(-59.39382)
//            .forwardZeroPowerAcceleration(-32.17968);
//
//    // 2. 路径约束
//    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
//
//    // 3. Mecanum 底盘配置
//    public static MecanumConstants driveConstants = new MecanumConstants()
//            .maxPower(0.8)
//            .rightFrontMotorName("RFMotor")
//            .rightRearMotorName("RBMotor")
//            .leftRearMotorName("LBMotor")
//            .leftFrontMotorName("LFMotor")
//            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
//            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
//            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
//            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
//            .manualBrakeMode(true)        // 原 useBrakeModeInTeleOp(true)，3.x 中已更名
//            .xMovement(64.599)            // 原 xVelocity，3.x 中已更名
//            .yMovement(51.33016);         // 原 yVelocity，3.x 中已更名
//
//    // 4. Pinpoint 定位器配置
//    public static PinpointConstants localizerConstants = new PinpointConstants()
//            .forwardPodY(-1.9685)
//            .strafePodX(-8.033)
//            .distanceUnit(DistanceUnit.INCH)
//            .hardwareMapName("odo")
//            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
//            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
//            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
//
//    // 5. 构建 Follower（方法名改为 create，与 3.x 官方文档一致）
//    public static Follower create(HardwareMap hardwareMap) {
//        return new FollowerBuilder(followerConstants, hardwareMap)
//                .pathConstraints(pathConstraints)
//                .mecanumDrivetrain(driveConstants)
//                .pinpointLocalizer(localizerConstants)
//                .build();
//    }
//}
//
////package org.firstinspires.ftc.teamcode.pedroPathing;
////
////import com.pedropathing.follower.Follower;
////import com.pedropathing.follower.FollowerConstants;
////import com.pedropathing.ftc.FollowerBuilder;
////import com.pedropathing.ftc.drivetrains.MecanumConstants;
////import com.pedropathing.ftc.localization.constants.PinpointConstants;
////import com.pedropathing.paths.PathConstraints;
////import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
////import com.qualcomm.robotcore.hardware.DcMotor;
////import com.qualcomm.robotcore.hardware.DcMotorSimple;
////import com.qualcomm.robotcore.hardware.HardwareMap;
////
////import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
////
////public class Constants {
////    public static FollowerConstants followerConstants = new FollowerConstants()
////            .mass(12.11)
////            .lateralZeroPowerAcceleration(-59.39382)
////            .forwardZeroPowerAcceleration(-32.17968);
////
////
//////    public static FollowerConstants followerConstants = new FollowerConstants()
//////            .mass(9.117);
////    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
////
////    public static Follower createFollower(HardwareMap hardwareMap) {
////        return new FollowerBuilder(followerConstants, hardwareMap)
////                .pathConstraints(pathConstraints)
////                .mecanumDrivetrain(driveConstants)
////                .pinpointLocalizer(localizerConstants)
////                .build();
////    }
////    public static MecanumConstants driveConstants = new MecanumConstants()
////            .maxPower(0.8)
////            .rightFrontMotorName("RFMotor")
////            .rightRearMotorName("RBMotor")
////            .leftRearMotorName("LBMotor")
////            .leftFrontMotorName("LFMotor")
////            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
////            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
////            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
////            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
////            .useBrakeModeInTeleOp(true)
////            .xVelocity(64.599)
////            .yVelocity(51.33016);          ;
//////useBrakeModeInTeleOp(true) added after using follower for teleOp
////    public static PinpointConstants localizerConstants = new PinpointConstants()
////            .forwardPodY(-1.9685)
////            .strafePodX(-8.033)
////            .distanceUnit(DistanceUnit.INCH)
////            .hardwareMapName("odo")
////            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
////            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
////            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);// changed from forward tobe reversed 0130843pm
//////    public static PinpointConstants localizerConstants = new PinpointConstants()
//////            .forwardPodY(-1.9685)
//////            .strafePodX(-7.08661)
//////            .distanceUnit(DistanceUnit.INCH)
//////            .hardwareMapName("odo")
//////            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
//////            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
//////            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
////
////}
