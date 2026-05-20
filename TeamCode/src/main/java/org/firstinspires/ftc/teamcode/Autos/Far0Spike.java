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
@Autonomous(name = "Far0Spike", group = "Auto")
public class Far0Spike extends OpMode {

    public Follower follower;
    private int pathState;
    private boolean red;

    ElapsedTime delayTimer = new ElapsedTime();
    double delaySeconds = 0.0;
    final double AUTO_LENGTH_SECONDS = 30.0;
    final double AUTO_END_BUFFER_SECONDS = 1.0;
    int intakeFromTunnel = 2;


    Servo indicatorLight;
    OuttakeFR outtake = new OuttakeFR();
    private ElapsedTime timer = new ElapsedTime();


    // *************     POSES    *************
    private Pose startPose = new Pose(53.5, 8.8, Math.toRadians(90));
    private Pose launchPosePreload = new Pose(53.5,17, Math.toRadians(112.1));


    private Pose intakeCornerControlPoint = new Pose(25.5, 23);
    private Pose intakeCornerReadyPose = new Pose(16.3, 20, Math.toRadians(235));
    private Pose intakeCornerPose = new Pose(13, 10, Math.toRadians(185));
    private Pose launchCornerControlPoint = new Pose(24, 21);
    private Pose launchPoseCorner = new Pose(53.5,17,Math.toRadians(112.1));

    private Pose intakeTunnelControlPoint = new Pose(26.6, 9);
    private Pose intakeTunnelReadyPose = new Pose(17, 14.5, Math.toRadians(150));
    private Pose intakeTunnelReadyControlPoint = new Pose(10, 18);
    private Pose intakeTunnelPose = new Pose(10, 32, Math.toRadians(90));
    private Pose launchTunnelControlPoint = new Pose(29, 24);
    private Pose launchPoseTunnel = new Pose(53.5,17,Math.toRadians(112.1));

    private Pose leavePose = new Pose(50, 20, Math.toRadians(135));


    private PathChain launchPathPreload, intakeCornerReady, intakeCorner, launchPathCorner, intakeTunnelReady, intakeTunnel, launchPathTunnel, leavePath;



    @Override
    public void init() {

        indicatorLight = hardwareMap.servo.get("light");

        // Mirror coordinates across the x-Axis if the autonomous is run on the Red side
        if (gamepad1.dpad_right || gamepad2.dpad_right) {

            startPose = startPose.mirror();
            launchPosePreload = launchPosePreload.mirror();

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
                .setLinearHeadingInterpolation(startPose.getHeading(), launchPosePreload.getHeading()).build();

        // ....... Intake/Launch Corner
        intakeCornerReady = follower.pathBuilder()
                .addPath(new BezierCurve(  launchPosePreload, intakeCornerControlPoint, intakeCornerReadyPose  ))
                .setLinearHeadingInterpolation(launchPosePreload.getHeading(), intakeCornerReadyPose.getHeading()).build();
        intakeCorner = follower.pathBuilder()
                .addPath(new BezierLine(  intakeCornerReadyPose, intakeCornerPose  ))
                .setLinearHeadingInterpolation(intakeCornerReadyPose.getHeading(), intakeCornerPose.getHeading()).build();
        launchPathCorner = follower.pathBuilder()
                .addPath(new BezierCurve(  intakeCornerPose, launchCornerControlPoint, launchPoseCorner  ))
                .setLinearHeadingInterpolation(intakeCornerPose.getHeading(), launchPoseCorner.getHeading()).build();

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
                .addPath(new BezierLine(  launchPoseCorner, leavePose  ))
                .setLinearHeadingInterpolation(launchPoseCorner.getHeading(), leavePose.getHeading()).build();
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
                    follower.followPath(intakeCornerReady,true);
                    outtake.setIntakePower(true);
                    pathState = 31;
                    timer.reset();
                }
                break;

            case 31:
                if (!follower.isBusy()) {
                    follower.followPath(intakeCorner, 0.7,true);
                    pathState = 32;
                }
                break;

            case 32:
                if (timer.seconds() > 4) {
                    follower.followPath(launchPathCorner, true);
                    pathState = 33;
                    timer.reset();
                }
                break;

            case 33:
                if (timer.seconds() > 0.5) {
                    outtake.setIntakePower(false);
                    outtake.setOuttakeVelocity(true);
                }
                if (!follower.isBusy()) {
                    outtake.fireShots(3);
                    pathState = 41;
                }
                break;

            case 41:
                if (!outtake.isBusy()) {
                    follower.followPath(intakeTunnelReady,true);
                    outtake.setIntakePower(true);
                    pathState = 42;
                    timer.reset();
                }
                break;

            case 42:
                if (!follower.isBusy()) {
                    follower.followPath(intakeTunnel, 0.7,true);
                    pathState = 43;
                }
                break;

            case 43:
                if (timer.seconds() > 4) {
                    follower.followPath(launchPathTunnel, true);
                    outtake.setOuttakeVelocity(true);
                    pathState = 44;
                    timer.reset();
                }
                break;

            case 44:
                if (timer.seconds() > 0.5) {
                    outtake.setIntakePower(false);
                }
                if (!follower.isBusy()) {
                    outtake.fireShots(3);
                    //if (intakeFromTunnel == 0) {
                    pathState = 5;
                    //} else {
                      //  pathState = 41;
                    //}
                }
                break;

            case 5:
                if (!outtake.isBusy()) {
                    follower.followPath(intakeCornerReady,true);
                    outtake.setIntakePower(true);
                    pathState = 61;
                    timer.reset();
                }
                break;

            case 61:
                if (!follower.isBusy()) {
                    follower.followPath(intakeCorner, 0.7,true);
                    pathState = 62;
                }
                break;

            case 62:
                if (timer.seconds() > 4) {
                    follower.followPath(launchPathCorner, true);
                    pathState = 63;
                    timer.reset();
                }
                break;

            case 63:
                if (timer.seconds() > 0.5) {
                    outtake.setIntakePower(false);
                    outtake.setOuttakeVelocity(true);
                }
                if (!follower.isBusy()) {
                    outtake.fireShots(3);
                    pathState = 71;
                }
                break;

            case 71:
                if (!outtake.isBusy()) {
                    follower.followPath(intakeCornerReady,true);
                    outtake.setIntakePower(true);
                    pathState = 72;
                    timer.reset();
                }
                break;

            case 72:
                if (!follower.isBusy()) {
                    follower.followPath(intakeCorner, 0.7,true);
                    pathState = 73;
                }
                break;

            case 73:
                if (timer.seconds() > 4) {
                    follower.followPath(launchPathCorner, true);
                    pathState = 74;
                    timer.reset();
                }
                break;

            case 74:
                if (timer.seconds() > 0.5) {
                    outtake.setIntakePower(false);
                    outtake.setOuttakeVelocity(true);
                }
                if (!follower.isBusy()) {
                    outtake.fireShots(3);
                    pathState = 81;
                }
                break;

            case 81:
                if (!outtake.isBusy()) {
                    follower.followPath(intakeCornerReady,true);
                    outtake.setIntakePower(true);
                    pathState = 82;
                    timer.reset();
                }
                break;

            case 82:
                if (!follower.isBusy()) {
                    follower.followPath(intakeCorner, 0.7,true);
                    pathState = 83;
                    timer.reset();
                }
                break;

            case 83:
                if (timer.seconds() > 4) {
                    outtake.setIntakePower(0);
                    pathState = -1;
                    timer.reset();
                }
                break;


            /*case 6:
                if (!outtake.isBusy()) {
                    outtake.setOuttakeVelocity(0);
                    follower.followPath(leavePath, true);
                    pathState = -1;
                }
                break;*/
        }
    }
}
