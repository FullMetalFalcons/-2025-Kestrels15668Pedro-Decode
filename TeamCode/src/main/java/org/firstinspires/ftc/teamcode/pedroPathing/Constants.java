package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
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

@Configurable
public class Constants {

    // XAVIER NOTE:  While tuning for Autonomous, the Pedro Pathing documentation will tell you to copy and paste over a "MecanumConstants" declaration statement.
    //   We already did that below, so just add on to it or modify it as necessary. You will still need to copy and paste certain things over, though,
    //   such as a localizerConstants declaration statement
    // TODO Update constants

    final static double lbPerKg = 2.205;
    final static double robotWeightInPounds = 22; //
    public static double offsetX = -5.51177863558, offsetY = 2.33294452;
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(robotWeightInPounds / lbPerKg)
            //.forwardZeroPowerAcceleration(77.25213094395916)
            //.lateralZeroPowerAcceleration(57.2398710476132)
            //.translationalPIDFCoefficients(new PIDFCoefficients(0.08, 0, 0.01, 0.08))
            .headingPIDFCoefficients(new PIDFCoefficients(1,0,0.11,0.038))
            //.drivePIDFCoefficients(new FilteredPIDFCoefficients(0.004,0,0.0006,0,0.024))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.13, 0.09568919477896312, 0.001302114196862323)) // (kP, kLinear, kQuadratic)
            .centripetalScaling(0.0009)
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rb")
            .leftRearMotorName("lb")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)

            .xVelocity(82.31240580401084)
            .yVelocity(63.72572098379062)
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
            .forwardPodY(offsetY)
            .strafePodX(offsetX)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

}