package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    // XAVIER NOTE:  While tuning for Autonomous, the Pedro Pathing documentation will tell you to copy and paste over a "MecanumConstants" declaration statement.
    //   We already did that below, so just add on to it or modify it as necessary. You will still need to copy and paste certain things over, though,
    //   such as a localizerConstants declaration statement

    final static double lbPerKg = 2.205;
    final static double robotWeightInPounds = 28; //
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(robotWeightInPounds / lbPerKg)
            .forwardZeroPowerAcceleration(-34.036282596)
            .lateralZeroPowerAcceleration(-60.040102549)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.06, 0, 0.005, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(1,0,0.08,0.02))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.014,0,0.0002,0,0.03))
            .centripetalScaling(0.005)
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("front_right")
            .rightRearMotorName("back_right")
            .leftRearMotorName("back_left")
            .leftFrontMotorName("front_left")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)

            .xVelocity(60.773636)
            .yVelocity(48.310272457092765)
            ;

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1,
            1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();

    }

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0)
            .strafePodX(-1)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

}