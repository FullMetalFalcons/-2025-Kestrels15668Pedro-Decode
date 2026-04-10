package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.OuttakeFR;
import org.firstinspires.ftc.teamcode.Mechanisms.Webcam;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Far1Spike", group = "Auto")
public class Far1Spike extends OpMode {

    public Follower follower;
    private int pathState;
    private boolean red;

    ElapsedTime delayTimer = new ElapsedTime();
    ElapsedTime autoTimer = new ElapsedTime();
    double delaySeconds = 0.0;
    final double AUTO_LENGTH_SECONDS = 30.0;
    final double AUTO_END_BUFFER_SECONDS = 1.0;

    private OuttakeFR outtake = new OuttakeFR();
    Webcam webcam = new Webcam(hardwareMap);

    // Define important coordinate locations for the Blue side of the field
    private Pose startPose = new Pose(56, 9.5, Math.toRadians(270));
    private Pose launchPose = new Pose(56,14,Math.toRadians(292));

    // 1st spike mark
    private Pose intake1ControlPoint = new Pose(56, 36);
    private Pose intake1ReadyPose =  new Pose(48, 36, Math.toRadians(180));
    private Pose intake1FinishPose = new Pose(18, 36, Math.toRadians(180));


    // overflow pickup
    private Pose intakeTunnelControlPoint = new Pose(9, 20);
    private Pose intakeTunnelReadyPose =  new Pose(14, 20, Math.toRadians(135));
    private Pose intakeTunnelFinishPose = new Pose(9, 38, Math.toRadians(135));


    // corner pickup
    private Pose intakeCornerControlPoint = new Pose(7.5, 20);
    private Pose intakeCornerReadyPose =  new Pose(14, 28, Math.toRadians(225));
    private Pose intakeCornerFinishPose = new Pose(7.5, 12, Math.toRadians(185));
    private Pose launchCornerControlPoint = new Pose(30, 20);


    private Pose leavePose = new Pose(36, 12, Math.toRadians(270));

    private PathChain launchPath1, intakePathReady1,intakePath1, launchPath2, intakePathReadyTunnel, intakePathTunnel, launchPathTunnel, intakePathReadyCorner, intakePathCorner, launchPathCorner, leavePath, hitLever1;

    private ElapsedTime timer = new ElapsedTime();


    @Override
    public void init() {


        // Mirror coordinates across the x-Axis if the autonomous is run on the Red side
        if (gamepad1.dpad_right) {
            startPose = startPose.mirror();
            launchPose = new Pose(144-56,14,Math.toRadians(270-24.1));
            intake1ReadyPose = intake1ReadyPose.mirror();
            intake1ControlPoint = intake1ControlPoint.mirror();
            intake1FinishPose = intake1FinishPose.mirror();
            intakeTunnelReadyPose = intakeTunnelReadyPose.mirror();
            intakeTunnelControlPoint = intakeTunnelControlPoint.mirror();
            intakeTunnelFinishPose = intakeTunnelFinishPose.mirror();
            intakeCornerReadyPose = intakeCornerReadyPose.mirror();
            launchCornerControlPoint = launchCornerControlPoint.mirror();
            intakeCornerControlPoint = intakeCornerControlPoint.mirror();
            intakeCornerFinishPose = intakeCornerFinishPose.mirror();
            leavePose = leavePose.mirror();
            red = true;
        } else {
            red = false;
        }

        follower = Constants.createFollower(hardwareMap); // Make sure you create the follower before building paths
        buildPaths();
        follower.setStartingPose(startPose);

        // Initialize external systems
        webcam = new Webcam(hardwareMap);
        webcam.init(hardwareMap, telemetry);
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
        autoTimer.reset();

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
        // ....... Launch 1
        launchPath1 = follower.pathBuilder()
                .addPath(new BezierLine(  startPose, launchPose  ))
                .setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading()).build();

        // ....... Intake 1
        intakePathReady1 = follower.pathBuilder()
                .addPath(new BezierCurve(  launchPose, intake1ControlPoint, intake1ReadyPose  ))
                .setLinearHeadingInterpolation(launchPose.getHeading(), intake1ReadyPose.getHeading()).build();
        intakePath1 = follower.pathBuilder()
                .addPath(new BezierLine(  intake1ReadyPose, intake1FinishPose  ))
                .setTangentHeadingInterpolation().build();

        // ....... Launch 2
        launchPath2 = follower.pathBuilder()
                .addPath(new BezierLine(  intake1FinishPose, launchPose  ))
                .setLinearHeadingInterpolation(intake1FinishPose.getHeading(), launchPose.getHeading())
                .build();

        // ....... Intake 2
        intakePathReadyTunnel = follower.pathBuilder()
                .addPath(new BezierLine(  launchPose, intakeTunnelReadyPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), intakeTunnelReadyPose.getHeading())
                .build();
        intakePathTunnel = follower.pathBuilder()
                .addPath(new BezierCurve(intakeTunnelReadyPose, intakeTunnelControlPoint, intakeTunnelFinishPose))
                .setLinearHeadingInterpolation(intakeTunnelReadyPose.getHeading(), intakeTunnelFinishPose.getHeading())
                .build();

        // ....... Launch 3
        launchPathTunnel = follower.pathBuilder()
                .addPath(new BezierLine(intakeTunnelFinishPose, launchPose  ))
                .setLinearHeadingInterpolation(intakeTunnelFinishPose.getHeading(), launchPose.getHeading())
                .build();

        // ....... Intake 3
        intakePathReadyCorner = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, intakeCornerReadyPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), intakeCornerReadyPose.getHeading())
                .build();
        intakePathCorner = follower.pathBuilder()
                .addPath(new BezierCurve(intakeCornerReadyPose, intakeCornerControlPoint, intakeCornerFinishPose))
                .setLinearHeadingInterpolation(intakeCornerReadyPose.getHeading(), intakeCornerFinishPose.getHeading())
                .build();

        // ....... Launch 4
        launchPathCorner = follower.pathBuilder()
                .addPath(new BezierCurve(intakeCornerFinishPose, launchCornerControlPoint, launchPose))
                .setLinearHeadingInterpolation(intakeCornerFinishPose.getHeading(), launchPose.getHeading())
                .build();

        // ....... Leave Points
        leavePath = follower.pathBuilder()
                .addPath(new BezierLine(  launchPose, leavePose  ))
                .setConstantHeadingInterpolation(leavePose.getHeading())
                .build();
    }

    //TODO add outtake delay
    public void autonomousPathUpdate() {

        // Autonomous state machine
        switch (pathState) {
            case 0:
                // Wait for the starting delay to expire
                if (delayTimer.seconds() > delaySeconds) {
                    // Begin the whole route
                    outtake.setOuttakeVelocity(true);
                    follower.followPath(launchPath1, true);
                    pathState = 1;
                }
                break;
            case 1:
                /* Let the robot get to launch position */

                if (!follower.isBusy()) {
                    // Begin the first launch sequence
                    outtake.fireShots(3);
                    pathState = 2;
                }
                break;
            case 2:
                /* Let the first launch sequence play out */

                if (!outtake.isBusy()) {
                    // drive to the first line of balls
                    follower.followPath(intakePathReady1, true);
                    outtake.setIntakePower(1);
                    pathState = 3;
                }
                break;
            case 3:
                /* Let the robot get to the first line of balls */

                if (!follower.isBusy()) {
                    // Intake the first line of balls
                    follower.followPath(intakePath1, 0.6, true);
                    pathState = 5;
                    timer.reset();
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    // Stop the intake and drive back to launch position
                    if (timer.seconds() > 0.4) {
                        outtake.setIntakePower(-0.3);
                    } else {
                        outtake.setIntakePower(0);
                    }
                    outtake.setOuttakeVelocity(true);
                    follower.followPath(launchPath2, true);
                    pathState = 6;
                }
                break;
            case 6:
                /* Let the robot get back to launch position */

                if (!follower.isBusy()) {
                    // Begin the second launch sequence
                    outtake.setIntakePower(0);
                    outtake.fireShots(3);
                    pathState = 7;
                }
                break;
            case 7:
                /* Let the second launch sequence play out */

                if (!outtake.isBusy())
                {
                    // Drive to the second line of balls of balls
                    outtake.setIntakePower(1);
                    follower.followPath(intakePathReadyCorner);
                    pathState = 8;
                }
                break;
            case 8:
                /* Let the robot get to the second line of balls of balls */

                if (!follower.isBusy()) {
                    // Intake the second line of balls of balls
                    follower.followPath(intakePathCorner, 0.6, true);
                    pathState = 9;
                    timer.reset();
                }
                break;
            case 9:
                /* Let the intake sequence play out */

                if (timer.seconds() > 3.5) {
                    // Stop the intake and drive back to launch position
                    if (timer.seconds() > 3.9) {
                        outtake.setIntakePower(-0.3);
                    } else {
                        outtake.setIntakePower(0);
                    }
                    outtake.setOuttakeVelocity(true);
                    follower.followPath(launchPathCorner, true);
                    pathState = 10;
                }
                break;
            case 10:
                /* Let the robot get back to launch position */

                if (!follower.isBusy()) {
                    // Begin the third launch sequence
                    outtake.setIntakePower(0);
                    outtake.fireShots(3);
                    pathState = 11;
                    timer.reset();
                }
                break;
            case 11:
                /* Let the robot get to the third line of balls */

                if (!outtake.isBusy()) {
                    outtake.setIntakePower(1);
                    follower.followPath(intakePathReadyTunnel, true);
                    pathState = 12;
                }
                break;
            case 12:
                /* Let the intake sequence play out */

                if (!follower.isBusy()) {
                    // intake third line of balls
                    follower.followPath(intakePathTunnel, 0.65, true);
                    timer.reset();
                    pathState = 13;
                }
                break;
            case 13:
                if (timer.seconds() > 4) {
                    // stop intake and goto launch
                    if (timer.seconds() > 4.4) {
                        outtake.setIntakePower(-0.3);
                    } else {
                        outtake.setIntakePower(0);
                    }
                    outtake.setOuttakeVelocity(true);
                    follower.followPath(launchPathTunnel, true);
                    pathState = 14;
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    // launch
                    outtake.setIntakePower(0);
                    outtake.fireShots(3);
                    pathState = 15;
                    timer.reset();
                }
                break;
            case 15:
                /* Let the robot get to the third line of balls */

                if (!outtake.isBusy()) {
                    outtake.setIntakePower(1);
                    follower.followPath(intakePathReadyTunnel, true);
                    pathState = 16;
                }
                break;
            case 16:
                /* Let the intake sequence play out */

                if (!follower.isBusy()) {
                    // intake third line of balls
                    follower.followPath(intakePathTunnel, 0.65, true);
                    timer.reset();
                    pathState = 17;
                }
                break;
            case 17:
                if (!follower.isBusy() || timer.seconds() > 4) {
                    // stop intake and goto launch
                    if (timer.seconds() > 4.4) {
                        outtake.setIntakePower(-0.3);
                    } else {
                        outtake.setIntakePower(0);
                    }
                    outtake.setOuttakeVelocity(true);
                    follower.followPath(launchPathTunnel, true);
                    pathState = 18;
                }
                break;
            case 18:
                if (!follower.isBusy()) {
                    // launch
                    outtake.setIntakePower(0);
                    outtake.fireShots(3);
                    pathState = 19;
                }
                break;

            case 19:
                /* Let the third launch sequence play out */

                // If the launch sequence is finished, or autonomous is about to end, move sideways for the Leave points
                if (/*autoTimer.seconds() > AUTO_LENGTH_SECONDS - AUTO_END_BUFFER_SECONDS ||*/ !outtake.isBusy()
                ) {

                    // Quit out of the state machine and move off of the Launch line
                    follower.followPath(leavePath, true);
                    pathState = -1;
                }
                break;
        }
    }
}