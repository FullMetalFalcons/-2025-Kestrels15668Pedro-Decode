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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.FalconsTeleOp;
import org.firstinspires.ftc.teamcode.Mechanisms.OuttakeFR;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "Close2SpikesConsistent", group = "Auto")
public class Close2SpikesConsistent extends OpMode {

    public Follower follower;
    private int pathState;
    private boolean red;

    ElapsedTime delayTimer = new ElapsedTime();
    double delaySeconds = 0.0;
    int intakeFromGate = 3;


    OuttakeFR outtake = new OuttakeFR();
    private ElapsedTime timer = new ElapsedTime();


    // *************     POSES    *************
    private Pose startPose = new Pose(22, 122.8, Math.toRadians(142));
    private Pose startControlPoint = new Pose(46.4, 102.4);
    private Pose launchPosePreload = new Pose(60, 82,Math.toRadians(134));


    private Pose intake1ControlPoint =  new Pose(51.7, 83.8);
    private Pose intake1ReadyPose = new Pose(42, 84, Math.toRadians(180));
    private Pose intake1Pose = new Pose(14,84,Math.toRadians(180));
    private Pose hitGate1ControlPoint = new Pose(24,80);
    private Pose hitGate1Pose = new Pose(15,75.4,Math.toRadians(180));
    private Pose launch1ControlPoint =  new Pose(40, 75);
    private Pose launchPose1 = new Pose(60,84,Math.toRadians(134));


    private Pose intake2ControlPoint = new Pose(56.8, 60);
    private Pose intake2ReadyPose = new Pose(42, 60, Math.toRadians(180));
    private Pose intake2Pose =  new Pose(16, 60, Math.toRadians(180));
    private Pose hitGate2ControlPoint = new Pose(23.4,63.4);
    private Pose hitGate2Pose = new Pose(15.2,65.8,Math.toRadians(160));
    private Pose launch2ControlPoint = new Pose(42, 66);
    private Pose launchPose2 = new Pose(60,75,Math.toRadians(132));


    private Pose intakeRampControlPoint = new Pose(28, 48);
    private Pose intakeRampReadyPose = new Pose(22, 56, Math.toRadians(140));
    private Pose intakeRampPose = new Pose(14,59.6, Math.toRadians(140.9));
    private Pose launchRampControlPoint = new Pose(25.5, 53);
    private Pose launchPoseRamp = new Pose(60,75,Math.toRadians(132));


    private Pose leavePose = new Pose(60, 96, Math.toRadians(142));


    private PathChain launchPathPreload, intakeSpike1Ready, intakeSpike1, hitGate1, launchPath1, intakeSpike2Ready, intakeSpike2, hitGate2, launchPath2, intakeRampReady, intakeRamp, launchPathRamp, launchPathEnd;



    @Override
    public void init() {


        // Mirror coordinates across the x-Axis if the autonomous is run on the Red side
        if (gamepad1.dpad_right || gamepad2.dpad_right) {

            startPose = startPose.mirror();
            startControlPoint = startControlPoint.mirror();
            launchPosePreload = launchPosePreload.mirror();

            intake1ControlPoint = intake1ControlPoint.mirror();
            intake1ReadyPose = intake1ReadyPose.mirror();
            hitGate1ControlPoint = hitGate1ControlPoint.mirror();
            hitGate1Pose = hitGate1Pose.mirror();
            intake1Pose = intake1Pose.mirror();
            launch1ControlPoint = launch1ControlPoint.mirror();
            launchPose1 = launchPose1.mirror();

            intake2ControlPoint = intake2ControlPoint.mirror();
            intake2ReadyPose = intake2ReadyPose.mirror();
            hitGate2ControlPoint = hitGate2ControlPoint.mirror();
            hitGate2Pose = hitGate2Pose.mirror();
            intake2Pose = intake2Pose.mirror();
            launch2ControlPoint = launch2ControlPoint.mirror();
            launchPose2 = launchPose2.mirror();

            intakeRampControlPoint = intakeRampControlPoint.mirror();
            intakeRampReadyPose = intakeRampReadyPose.mirror();
            intakeRampPose = intakeRampPose.mirror();
            launchRampControlPoint = launchRampControlPoint.mirror();
            launchPoseRamp = launchPoseRamp.mirror();

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
        telemetry.addData("busyFollower?", follower.isBusy());
        telemetry.addData("path?",pathState);
        telemetry.addData("rpm?",outtake.getFlywheelVelocity());
        telemetry.addData("pose?", follower.getPose());
        telemetry.addData("timer", timer.seconds());
        telemetry.addData("takeFromGate", intakeFromGate);
        telemetry.update();

        FalconsTeleOp.startingPose = new Pose2D(DistanceUnit.INCH, follower.getPose().getX(), follower.getPose().getY(), AngleUnit.DEGREES, Math.toDegrees(follower.getHeading()));
    }

    public void buildPaths() {
        // ....... Launch Preload
        launchPathPreload = follower.pathBuilder()
                .addPath(new BezierCurve(  startPose, startControlPoint, launchPosePreload  ))
                .setLinearHeadingInterpolation(startPose.getHeading(), launchPosePreload.getHeading()).build();

        // ....... Intake/Launch 1
        intakeSpike1Ready = follower.pathBuilder()
                .addPath(new BezierCurve(  launchPosePreload, intake1ControlPoint, intake1ReadyPose  ))
                .setLinearHeadingInterpolation(launchPosePreload.getHeading(), intake1ReadyPose.getHeading()).build();
        intakeSpike1 = follower.pathBuilder()
                .addPath(new BezierCurve(  intake1ReadyPose, intake1ControlPoint, intake1Pose  ))
                .setTangentHeadingInterpolation().build();
        hitGate1 = follower.pathBuilder()
                .addPath(new BezierCurve(  intake1Pose, hitGate1ControlPoint, hitGate1Pose  ))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), hitGate1Pose.getHeading()).build();
        launchPath1 = follower.pathBuilder()
                .addPath(new BezierCurve(  hitGate1Pose, launch1ControlPoint, launchPose1  ))
                .setLinearHeadingInterpolation(hitGate1Pose.getHeading(), launchPose1.getHeading()).build();

        // ....... Intake/Launch 2
        intakeSpike2Ready = follower.pathBuilder()
                .addPath(new BezierCurve(  launchPose1, intake2ControlPoint, intake2ReadyPose  ))
                .setLinearHeadingInterpolation(launchPose1.getHeading(), intake2ReadyPose.getHeading()).build();
        intakeSpike2 = follower.pathBuilder()
                .addPath(new BezierLine(  intake2ReadyPose, intake2Pose  ))
                .setTangentHeadingInterpolation().build();
        hitGate2 = follower.pathBuilder()
                .addPath(new BezierCurve(  intake2Pose, hitGate2ControlPoint, hitGate2Pose  ))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), hitGate2Pose.getHeading()).build();
        launchPath2 = follower.pathBuilder()
                .addPath(new BezierCurve(  hitGate2Pose, launch2ControlPoint, launchPose2  ))
                .setLinearHeadingInterpolation(hitGate2Pose.getHeading(), launchPose2.getHeading()).build();

        // ....... Intake/Launch Ramp
        intakeRampReady = follower.pathBuilder()
                .addPath(new BezierCurve(  launchPose2, intakeRampControlPoint, intakeRampReadyPose  ))
                .setLinearHeadingInterpolation(launchPose2.getHeading(), intakeRampReadyPose.getHeading()).build();
        intakeRamp = follower.pathBuilder()
                .addPath(new BezierLine(  intakeRampReadyPose, intakeRampPose  ))
                .setLinearHeadingInterpolation(intakeRampReadyPose.getHeading(), intakeRampPose.getHeading()).build();
        launchPathRamp = follower.pathBuilder()
                .addPath(new BezierCurve(  intakeRampPose, launchRampControlPoint, launchPoseRamp  ))
                .setLinearHeadingInterpolation(intakeRampPose.getHeading(), launchPoseRamp.getHeading()).build();

        // ....... Leave Points
        launchPathEnd = follower.pathBuilder()
                .addPath(new BezierCurve(  intakeRampPose, launchRampControlPoint, leavePose  ))
                .setLinearHeadingInterpolation(intakeRampPose.getHeading(), leavePose.getHeading()).build();
    }

    public void autonomousPathUpdate() {

        // Autonomous state machine
        switch (pathState) {
            case 0:
                if (delayTimer.seconds() > delaySeconds) {
                    follower.followPath(launchPathPreload,true);
                    outtake.setOuttakeVelocity(false);
                    pathState = 1;
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
                    follower.followPath(intakeSpike1Ready, true);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(intakeSpike1, 0.85,true);
                    outtake.setIntakePower(true);
                    pathState = 4;
                    timer.reset();
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(hitGate1, 0.8,true);
                    pathState = 5;
                    timer.reset();
                }
                break;

            case 5:
                if (timer.seconds() > 0.5) {
                    outtake.setIntakePower(false);
                }
                if (!follower.isBusy()) {
                    follower.followPath(launchPath1, true);
                    pathState = 6;
                    timer.reset();
                }
                break;

            case 6:
                if (timer.seconds() > 0.5) {
                    outtake.setOuttakeVelocity(false);
                }
                if (!follower.isBusy()) {
                    outtake.fireShots(3);
                    pathState = 7;
                    timer.reset();
                }
                break;

            case 7:
                if (!outtake.isBusy()) {
                    follower.followPath(intakeSpike2Ready,true);
                    pathState = 8;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(intakeSpike2,0.85,true);
                    outtake.setIntakePower(true);
                    pathState = 9;
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(hitGate2, 0.8, true);
                    pathState = 10;
                    timer.reset();
                }
                break;

            case 10:
                if (timer.seconds() > 0.5) {
                    outtake.setIntakePower(false);
                }
                if (!follower.isBusy()) {
                    follower.followPath(launchPath2, true);
                    pathState = 11;
                    timer.reset();
                }
                break;

            case 11:
                if (timer.seconds() > 0.5) {
                    outtake.setOuttakeVelocity(false);
                }
                if (!follower.isBusy()) {
                    outtake.fireShots(3);
                    pathState = 91;
                    timer.reset();
                }
                break;

            case 91:
                if (!outtake.isBusy()) {
                    follower.followPath(intakeRampReady,true);
                    intakeFromGate = intakeFromGate - 1;
                    pathState = 92;
                }
                break;

            case 92:
                if (!follower.isBusy()) {
                    follower.followPath(intakeRamp,0.6,true);
                    outtake.setIntakePower(true);
                    pathState = 93;
                }
                break;

            case 93:
                if (timer.seconds() > 5) {
                    if (intakeFromGate == 0) {
                        follower.followPath(launchPathEnd, true);
                        pathState = 12;
                    } else {
                        follower.followPath(launchPathRamp, true);
                        pathState = 11;
                    }
                    timer.reset();
                }
                break;


            case 12:
                if (timer.seconds() > 0.8) {
                    outtake.setOuttakeVelocity(false);
                    outtake.setIntakePower(false);
                }
                if (!follower.isBusy()) {
                    outtake.fireShots(3);
                    pathState = 13;
                }
                break;

            case 13:
                if (!outtake.isBusy()) {
                    outtake.setOuttakeVelocity(0);
                    pathState = -1;
                }
                break;
        }
    }
}
