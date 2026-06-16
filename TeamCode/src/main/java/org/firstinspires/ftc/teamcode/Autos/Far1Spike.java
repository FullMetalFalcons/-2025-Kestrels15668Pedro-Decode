package org.firstinspires.ftc.teamcode.Autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.FalconsTeleOp;
import org.firstinspires.ftc.teamcode.Mechanisms.OuttakeFR;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "Far1Spike", group = "Auto")
public class Far1Spike extends OpMode {

    public Follower follower;
    private int pathState;
    private boolean red;

    ElapsedTime delayTimer = new ElapsedTime();
    double delaySeconds = 0.0;
    final double AUTO_LENGTH_SECONDS = 30.0;
    final double AUTO_END_BUFFER_SECONDS = 1.0;
    //int intakeFromTunnel = 4;


    Servo indicatorLight;
    OuttakeFR outtake = new OuttakeFR();
    private ElapsedTime timer = new ElapsedTime();


    // *************     POSES    *************
    private Pose startPose = new Pose(53.5, 8.8, Math.toRadians(90));
    private Pose launchPosePreload = new Pose(53.5,17, Math.toRadians(112.2));


    private Pose intake3ControlPoint =  new Pose(48, 29);
    private Pose intake3Pose = new Pose(23, 36.5, Math.toRadians(170));
    private Pose launch3ControlPoint =  new Pose(51, 31.5);
    private Pose launchPose3 = new Pose(53.5,17,Math.toRadians(112.0));


    private Pose intakeCornerControlPoint = new Pose(25.5, 23);
    private Pose intakeCornerReadyPose = new Pose(17, 24, Math.toRadians(235));
    private Pose intakeCornerPose = new Pose(13, 10, Math.toRadians(185));
    private Pose launchCornerControlPoint = new Pose(25.5, 24);
    private Pose launchPoseCorner = new Pose(53.5,17,Math.toRadians(112.0));

    private Pose intakeTunnelControlPoint = new Pose(26.6, 9);
    private Pose intakeTunnelReadyPose = new Pose(17, 14.5, Math.toRadians(150));
    private Pose intakeTunnelReadyControlPoint = new Pose(10, 18);
    private Pose intakeTunnelPose = new Pose(10, 32, Math.toRadians(90));
    private Pose launchTunnelControlPoint = new Pose(29, 24);
    private Pose launchPoseTunnel = new Pose(53.5,17,Math.toRadians(112.0));


    private Pose leavePose = new Pose(50, 20, Math.toRadians(135));


    private PathChain launchPathPreload, intakeSpike3, launchPath3, intakeCornerReady, intakeCorner, launchPathCorner, intakeTunnelReady, intakeTunnel, launchPathTunnel, leavePath;



    @Override
    public void init() {

        indicatorLight = hardwareMap.servo.get("light");

        // Mirror coordinates across the x-Axis if the autonomous is run on the Red side
        if (gamepad1.dpad_right || gamepad2.dpad_right) {

            startPose = startPose.mirror();
            launchPosePreload = launchPosePreload.mirror();

            intake3ControlPoint = intake3ControlPoint.mirror();
            intake3Pose = intake3Pose.mirror();
            launch3ControlPoint = launch3ControlPoint.mirror();
            launchPose3 = launchPose3.mirror();

            intakeCornerControlPoint = intakeCornerControlPoint.mirror();
            intakeCornerReadyPose = intakeCornerReadyPose.mirror();
            intakeCornerPose = intakeCornerPose.mirror();
            launchCornerControlPoint = launchCornerControlPoint.mirror();
            launchPoseCorner = launchPoseCorner.mirror();

            intakeTunnelControlPoint = intakeTunnelControlPoint.mirror();
            intakeTunnelReadyPose = intakeTunnelReadyPose.mirror();
            intakeTunnelReadyControlPoint = intakeTunnelReadyControlPoint.mirror();
            intakeTunnelPose = intakeTunnelPose.mirror();
            launchTunnelControlPoint = launchTunnelControlPoint.mirror();
            launchPoseTunnel = launchPoseTunnel.mirror();

            leavePose = leavePose.mirror();
            red = true;
            FalconsTeleOp.blue = false;
        } else {
            red = false;
            FalconsTeleOp.blue = true;
        }
        telemetry.update();

        follower = Constants.createFollower(hardwareMap); // Make sure you create the follower before building paths
        buildPaths();
        follower.setStartingPose(startPose);

        // Initialize external systems
        outtake.init(hardwareMap);
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (red) {
            telemetry.addLine("red");
            indicatorLight.setPosition(0.290);
        } else {
            telemetry.addLine("blue");
            indicatorLight.setPosition(0.611);
        }

        // Modify the delay before the autonomous begins
        if (gamepad1.dpadUpWasPressed()) {
            delaySeconds += 0.5;
        }
        if (gamepad1.dpadDownWasPressed()) {
            delaySeconds -= 0.5;
        }
        telemetry.addData("Delay in seconds", delaySeconds);
        telemetry.update();

    }

    @Override
    public void start() {
        // Reset any timers
        delayTimer.reset();
        indicatorLight.setPosition(0);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing - will also cause the robot to follow the current path
        outtake.update();
        autonomousPathUpdate(); // Update autonomous state machine
        telemetry.addData("busy?", outtake.isBusy());
        telemetry.addData("busyfollower?", follower.isBusy());
        telemetry.addData("path?",pathState);
        telemetry.addData("rpm?",outtake.getFlywheelVelocity());
        telemetry.addData("pose?", follower.getPose());
        telemetry.update();

        FalconsTeleOp.startingPose = new Pose2D(DistanceUnit.INCH, follower.getPose().getX(), follower.getPose().getY(), AngleUnit.DEGREES, Math.toDegrees(follower.getHeading()));
    }

    public void buildPaths() {
        // ....... Launch Preload
        launchPathPreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, launchPosePreload))
                .setLinearHeadingInterpolation(startPose.getHeading(), launchPosePreload.getHeading())
                .setTimeoutConstraint(150)
                .setVelocityConstraint(.05)
                .build();

        // ....... Intake/Launch 3
        intakeSpike3 = follower.pathBuilder()
                .addPath(new BezierCurve(  launchPosePreload, intake3ControlPoint, intake3Pose  ))
                .setLinearHeadingInterpolation(launchPosePreload.getHeading(), intake3Pose.getHeading()).build();
        launchPath3 = follower.pathBuilder()
                .addPath(new BezierCurve(  intake3Pose, launch3ControlPoint, launchPose3  ))
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), launchPose3.getHeading())
                .setTimeoutConstraint(150)
                .setVelocityConstraint(.05)
                .build();


        // ....... Intake/Launch Corner
        intakeCornerReady = follower.pathBuilder()
                .addPath(new BezierCurve(  launchPose3, intakeCornerControlPoint, intakeCornerReadyPose  ))
                .setLinearHeadingInterpolation(launchPose3.getHeading(), intakeCornerReadyPose.getHeading()).build();
        intakeCorner = follower.pathBuilder()
                .addPath(new BezierLine(  intakeCornerReadyPose, intakeCornerPose  ))
                .setLinearHeadingInterpolation(intakeCornerReadyPose.getHeading(), intakeCornerPose.getHeading()).build();
        launchPathCorner = follower.pathBuilder()
                .addPath(new BezierCurve(  intakeCornerPose, launchCornerControlPoint, launchPoseCorner  ))
                .setLinearHeadingInterpolation(intakeCornerPose.getHeading(), launchPoseCorner.getHeading())
                .setTimeoutConstraint(150)
                .setVelocityConstraint(.05)
                .build();

        // ....... Intake/Launch Tunnel
        intakeTunnelReady = follower.pathBuilder()
                .addPath(new BezierCurve(  launchPoseCorner, intakeTunnelControlPoint, intakeTunnelReadyPose  ))
                .setLinearHeadingInterpolation(launchPoseCorner.getHeading(), intakeTunnelReadyPose.getHeading()).build();
        intakeTunnel = follower.pathBuilder()
                .addPath(new BezierCurve(  intakeTunnelReadyPose, intakeTunnelReadyControlPoint, intakeTunnelPose  ))
                .setLinearHeadingInterpolation(intakeTunnelReadyPose.getHeading(), intakeTunnelPose.getHeading()).build();
        launchPathTunnel = follower.pathBuilder()
                .addPath(new BezierCurve(  intakeTunnelPose, launchTunnelControlPoint, launchPoseTunnel  ))
                .setLinearHeadingInterpolation(intakeTunnelPose.getHeading(), launchPoseTunnel.getHeading()).build();


        // ....... Leave Points
        leavePath = follower.pathBuilder()
                .addPath(new BezierLine(  launchPoseTunnel, leavePose  ))
                .setLinearHeadingInterpolation(launchPoseTunnel.getHeading(), leavePose.getHeading()).build();
    }

    public void autonomousPathUpdate() {

        // Autonomous state machine
        switch (pathState) {
            case 0:
                if (delayTimer.seconds() > delaySeconds) {
                    follower.followPath(launchPathPreload, true);
                    outtake.setOuttakeVelocity(true);
                    pathState = 1;
                    OuttakeFR.close = false;
                    OuttakeFR.far = true;
                    timer.reset();
                }
                break;

            case 1:
                if (!follower.isBusy()) {
                    outtake.fireShots(3);
                    pathState = 2;
                }
                break;

            case 2:
                if (!outtake.isBusy()) {
                    follower.followPath(intakeSpike3, 0.8, true);
                    outtake.setIntakePower(true);
                    pathState = 3;
                    timer.reset();
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(launchPath3, true);
                    pathState = 4;
                    timer.reset();
                }
                break;

            case 4:
                if (timer.seconds() > 0.5) {
                    outtake.setIntakePower(false);
                    outtake.setOuttakeVelocity(true);
                }
                if (!follower.isBusy()) {
                    outtake.fireShots(3);
                    pathState = 5;
                }
                break;

            case 5:
                if (!outtake.isBusy()) {
                    follower.followPath(intakeCornerReady,true);
                    outtake.setIntakePower(true);
                    pathState = 6;
                    timer.reset();
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(intakeCorner, 0.7,true);
                    pathState = 7;
                }
                break;

            case 7:
                if (timer.seconds() > 4) {
                    follower.followPath(launchPathCorner, true);
                    pathState = 8;
                    timer.reset();
                }
                break;

            case 8:
                if (timer.seconds() > 0.5) {
                    outtake.setIntakePower(false);
                    outtake.setOuttakeVelocity(true);
                }
                if (!follower.isBusy()) {
                    outtake.fireShots(3);
                    pathState = 91;
                }
                break;

            case 91:
                if (!outtake.isBusy()) {
                    follower.followPath(intakeTunnelReady,true);
                    outtake.setIntakePower(true);
                    pathState = 92;
                    timer.reset();
                }
                break;

            case 92:
                if (!follower.isBusy()) {
                    follower.followPath(intakeTunnel, 0.7,true);
                    pathState = 93;
                }
                break;

            case 93:
                if (timer.seconds() > 4) {
                    follower.followPath(launchPathTunnel, true);
                    outtake.setOuttakeVelocity(true);
                    pathState = 94;
                    timer.reset();
                }
                break;

            case 94:
                if (timer.seconds() > 0.5) {
                    outtake.setIntakePower(false);
                }
                if (!follower.isBusy()) {
                    outtake.fireShots(3);
                    //if (intakeFromTunnel == 0) {
                        pathState = 10;
                    //} else {
                        //pathState = 91;
                    //}
                }
                break;

            case 10:
                if (!outtake.isBusy()) {
                    follower.followPath(intakeCornerReady,true);
                    outtake.setIntakePower(true);
                    pathState = 11;
                    timer.reset();
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(intakeCorner, 0.7,true);
                    pathState = 12;
                }
                break;

            case 12:
                if (timer.seconds() > 4) {
                    follower.followPath(launchPathCorner, true);
                    pathState = 13;
                    timer.reset();
                }
                break;

            case 13:
                if (timer.seconds() > 0.5) {
                    outtake.setIntakePower(false);
                    outtake.setOuttakeVelocity(true);
                }
                if (!follower.isBusy()) {
                    outtake.fireShots(3);
                    pathState = 14;
                }
                break;

            case 14:
                if (!outtake.isBusy()) {
                    follower.followPath(intakeCornerReady,true);
                    outtake.setIntakePower(true);
                    pathState = 15;
                    timer.reset();
                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    follower.followPath(intakeCorner, 0.7,true);
                    pathState = 16;
                }
                break;

            case 16:
                if (timer.seconds() > 4) {
                    outtake.setOuttakeVelocity(0);
                    outtake.setIntakePower(0);
                    pathState = -1;
                }
                break;

            /*case 18:
                if (!outtake.isBusy()) {
                    outtake.setOuttakeVelocity(0);
                    follower.followPath(leavePath, true);
                    pathState = -1;
                }
                break;*/
        }
    }
}
