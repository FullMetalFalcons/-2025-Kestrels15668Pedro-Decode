package org.firstinspires.ftc.teamcode.Autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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


    OuttakeFR outtake = new OuttakeFR();
    private ElapsedTime timer = new ElapsedTime();


    // *************     POSES    *************
    private Pose startPose = new Pose(53.5, 12, Math.toRadians(110)); //TODO find real value


    private Pose intake3ControlPoint =  new Pose(48, 29);
    private Pose intake3Pose = new Pose(23, 36.5, Math.toRadians(170));
    private Pose launch3ControlPoint =  new Pose(51, 31.5);
    private Pose launchPose3 = new Pose(53.5,17,Math.toRadians(110));


    private Pose intakeCornerControlPoint = new Pose(25.5, 23);
    private Pose intakeCornerPose = new Pose(10, 12, Math.toRadians(190));
    private Pose launchCornerControlPoint = new Pose(25.5, 24);
    private Pose launchPoseCorner = new Pose(53.5,17,Math.toRadians(110));


    private Pose leavePose = new Pose(50, 20, Math.toRadians(135));


    private PathChain intakeSpike3, launchPath3, intakeCorner, launchPathCorner, leavePath;



    @Override
    public void init() {


        // Mirror coordinates across the x-Axis if the autonomous is run on the Red side
        if (gamepad1.dpad_right || gamepad2.dpad_right) {

            startPose = startPose.mirror();

            intake3ControlPoint = intake3ControlPoint.mirror();
            intake3Pose = intake3Pose.mirror();
            launch3ControlPoint = launch3ControlPoint.mirror();
            launchPose3 = launchPose3.mirror();

            intakeCornerControlPoint = intakeCornerControlPoint.mirror();
            intakeCornerPose = intakeCornerPose.mirror();
            launchCornerControlPoint = launchCornerControlPoint.mirror();
            launchPoseCorner = launchPoseCorner.mirror();

            leavePose = leavePose.mirror();
            red = true;
        } else {
            red = false;
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
        } else {
            telemetry.addLine("blue");
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
        telemetry.update();
    }

    public void buildPaths() {

        // ....... Intake/Launch 3
        intakeSpike3 = follower.pathBuilder()
                .addPath(new BezierCurve(  startPose, intake3ControlPoint, intake3Pose  ))
                .setLinearHeadingInterpolation(startPose.getHeading(), intake3Pose.getHeading()).build();
        launchPath3 = follower.pathBuilder()
                .addPath(new BezierCurve(  intake3Pose, launch3ControlPoint, launchPose3  ))
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), launchPose3.getHeading()).build();

        // ....... Intake/Launch Corner
        intakeCorner = follower.pathBuilder()
                .addPath(new BezierCurve(  launchPose3, intakeCornerControlPoint, intakeCornerPose  ))
                .setLinearHeadingInterpolation(launchPose3.getHeading(), intakeCornerPose.getHeading()).build();
        launchPathCorner = follower.pathBuilder()
                .addPath(new BezierCurve(  intakeCornerPose, launchCornerControlPoint, launchPoseCorner  ))
                .setLinearHeadingInterpolation(intakeCornerPose.getHeading(), launchPoseCorner.getHeading()).build();

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
                    outtake.setOuttakeVelocity(false);
                    outtake.fireShots(3);
                    pathState = 1;
                }
                break;

            case 1:
                if (!outtake.isBusy()) {
                    follower.followPath(intakeSpike3, true);
                    outtake.setIntakePower(true);
                    pathState = 2;
                    timer.reset();
                }
                break;

            case 2:
                if (timer.seconds() > 0.5) {
                    outtake.setIntakePower(false);
                }
                if (!follower.isBusy()) {
                    follower.followPath(launchPath3, true);
                    outtake.setOuttakeVelocity(false);
                    pathState = 3;
                    timer.reset();
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    outtake.fireShots(3);
                    pathState = 41;
                }
                break;

            case 41:
                if (!outtake.isBusy()) {
                    follower.followPath(intakeCorner,true);
                    outtake.setIntakePower(true);
                    pathState = 42;
                }
                break;

            case 42:
                if (!follower.isBusy() && timer.seconds() > 1.6) {
                    follower.followPath(launchPathCorner, true);
                    outtake.setOuttakeVelocity(false);
                    pathState = 43;
                    timer.reset();
                }
                break;

            case 43:
                if (timer.seconds() > 0.8) {
                    outtake.setIntakePower(false);
                }
                if (!follower.isBusy()) {
                    outtake.fireShots(3);
                    pathState = 5;
                }
                break;

            case 5:
                if (!outtake.isBusy()) {
                    outtake.setOuttakeVelocity(0);
                    follower.followPath(leavePath, true);
                    pathState = -1;
                }
                break;
        }
    }
}
