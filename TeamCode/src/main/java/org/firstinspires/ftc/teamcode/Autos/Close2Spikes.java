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
@Autonomous(name = "Close2Spikes", group = "Auto")
public class Close2Spikes extends OpMode {

    public Follower follower;
    private int pathState;
    private boolean red;

    ElapsedTime delayTimer = new ElapsedTime();
    double delaySeconds = 0.0;
    final double AUTO_LENGTH_SECONDS = 30.0;
    final double AUTO_END_BUFFER_SECONDS = 1.0;
    public static int INTAKE_FROM_GATE = 3;
    int intakeFromGate = INTAKE_FROM_GATE;


    OuttakeFR outtake = new OuttakeFR();
    private ElapsedTime timer = new ElapsedTime();


    // *************     POSES    *************
    private Pose startPose = new Pose(22, 122.6, Math.toRadians(145)); //TODO find real value
    private Pose startControlPoint = new Pose(53, 92);
    private Pose launchPosePreload = new Pose(58,78,Math.toRadians(137));


    private Pose intake1ControlPoint =  new Pose(36, 76);
    private Pose intake1Pose = new Pose(22, 84, Math.toRadians(180));
    private Pose launch1ControlPoint =  new Pose(45, 78);
    private Pose launchPose1 = new Pose(61,84,Math.toRadians(137));


    private Pose intake2ControlPoint = new Pose(50, 55);
    private Pose intake2Pose =  new Pose(22, 60, Math.toRadians(190));
    private Pose launch2ControlPoint = new Pose(42, 63);
    private Pose launchPose2 = new Pose(58,80.5,Math.toRadians(137));


    private Pose intakeRampControlPoint = new Pose(28, 48);
    private Pose intakeRampPose = new Pose(13, 60, Math.toRadians(145)); //TODO find real value
    private Pose launchRampControlPoint = new Pose(25.5, 53);
    private Pose launchPoseRamp = new Pose(58,80.5,Math.toRadians(137));


    private Pose leavePose = new Pose(44, 80, Math.toRadians(140));


    private PathChain launchPathPreload, intakeSpike1, launchPath1, intakeSpike2, launchPath2, intakeRamp, launchPathRamp, leavePath;



    @Override
    public void init() {


        // Mirror coordinates across the x-Axis if the autonomous is run on the Red side
        if (gamepad1.dpad_right || gamepad2.dpad_right) {

            startPose = startPose.mirror();
            startControlPoint = startControlPoint.mirror();
            launchPosePreload = launchPosePreload.mirror();

            intake1ControlPoint = intake1ControlPoint.mirror();
            intake1Pose = intake1Pose.mirror();
            launch1ControlPoint = launch1ControlPoint.mirror();
            launchPose1 = launchPose1.mirror();

            intake2ControlPoint = intake2ControlPoint.mirror();
            intake2Pose = intake2Pose.mirror();
            launch2ControlPoint = launch2ControlPoint.mirror();
            launchPose2 = launchPose2.mirror();

            intakeRampControlPoint = intakeRampControlPoint.mirror();
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
        telemetry.addData("busyfollower?", follower.isBusy());
        telemetry.addData("path?",pathState);
        telemetry.addData("rpm?",outtake.getFlywheelVelocity());
        telemetry.update();
    }

    public void buildPaths() {
        // ....... Launch Preload
        launchPathPreload = follower.pathBuilder()
                .addPath(new BezierCurve(  startPose, startControlPoint, launchPosePreload  ))
                .setLinearHeadingInterpolation(startPose.getHeading(), launchPosePreload.getHeading()).build();

        // ....... Intake/Launch 1
        intakeSpike1 = follower.pathBuilder()
                .addPath(new BezierCurve(  launchPosePreload, intake1ControlPoint, intake1Pose  ))
                .setLinearHeadingInterpolation(launchPosePreload.getHeading(), intake1Pose.getHeading()).build();
        launchPath1 = follower.pathBuilder()
                .addPath(new BezierCurve(  intake1Pose, launch1ControlPoint, launchPose1  ))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), launchPose1.getHeading()).build();

        // ....... Intake/Launch 2
        intakeSpike2 = follower.pathBuilder()
                .addPath(new BezierCurve(  launchPose1, intake2ControlPoint, intake2Pose  ))
                .setLinearHeadingInterpolation(launchPose1.getHeading(), intake2Pose.getHeading()).build();
        launchPath2 = follower.pathBuilder()
                .addPath(new BezierCurve(  intake2Pose, launch2ControlPoint, launchPose2  ))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), launchPose2.getHeading()).build();

        // ....... Intake/Launch Ramp
        intakeRamp = follower.pathBuilder()
                .addPath(new BezierCurve(  launchPose2, intakeRampControlPoint, intakeRampPose  ))
                .setLinearHeadingInterpolation(launchPose2.getHeading(), intakeRampPose.getHeading()).build();
        launchPathRamp = follower.pathBuilder()
                .addPath(new BezierCurve(  intakeRampPose, launchRampControlPoint, launchPoseRamp  ))
                .setLinearHeadingInterpolation(intakeRampPose.getHeading(), launchPoseRamp.getHeading()).build();

        // ....... Leave Points
        leavePath = follower.pathBuilder()
                .addPath(new BezierLine(  launchPoseRamp, leavePose  ))
                .setLinearHeadingInterpolation(launchPoseRamp.getHeading(), leavePose.getHeading()).build();
    }

    public void autonomousPathUpdate() {

        // Autonomous state machine
        switch (pathState) {
            case 0:
                if (delayTimer.seconds() > delaySeconds) {
                    outtake.setOuttakeVelocity(false);
                    follower.followPath(launchPathPreload,true);
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
                    follower.followPath(intakeSpike1, true);
                    outtake.setIntakePower(true);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(launchPath1, true);
                    outtake.setOuttakeVelocity(false);
                    pathState = 4;
                    timer.reset();
                }
                break;

            case 4:
                if (timer.seconds() > 0.5) {
                    outtake.setIntakePower(false);
                }
                if (!follower.isBusy()) {
                    outtake.fireShots(3);
                    pathState = 5;
                    timer.reset();
                }
                break;

            case 5:
                if (!outtake.isBusy()) {
                    follower.followPath(intakeSpike2,true);
                    outtake.setIntakePower(true);
                    pathState = 6;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(launchPath2, true);
                    outtake.setOuttakeVelocity(false);
                    pathState = 7;
                }
                break;

            case 7:
                if (timer.seconds() > 0.5) {
                    outtake.setIntakePower(false);
                }
                if (!follower.isBusy()) {
                    outtake.fireShots(3);
                    pathState = 81;
                    timer.reset();
                }
                break;

            case 81:
                if (!outtake.isBusy()) {
                    follower.followPath(intakeRamp,true);
                    outtake.setIntakePower(true);
                    pathState = 82;
                }
                break;

            case 82:
                if (!follower.isBusy() && timer.seconds() > 1.6) {
                    follower.followPath(launchPathRamp, true);
                    outtake.setOuttakeVelocity(false);
                    pathState = 83;
                    timer.reset();
                }
                break;

            case 83:
                if (timer.seconds() > 0.8) {
                    outtake.setIntakePower(false);
                }
                if (!follower.isBusy()) {
                    outtake.fireShots(3);
                    if (intakeFromGate == 0) {
                        pathState = 9;
                    } else {
                        intakeFromGate -= intakeFromGate;
                        pathState = 81;
                    }
                }
                break;

            case 9:
                if (!outtake.isBusy()) {
                    outtake.setOuttakeVelocity(0);
                    follower.followPath(leavePath, true);
                    pathState = -1;
                }
                break;
        }
    }
}
