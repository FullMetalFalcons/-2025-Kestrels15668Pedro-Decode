package org.firstinspires.ftc.teamcode;

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


    private Pose intake1ControlPoint = new Pose(56, 36);
    private Pose intake1ReadyPose =  new Pose(48, 36, Math.toRadians(180));
    private Pose intake1FinishPose = new Pose(18, 36, Math.toRadians(180));



    private Pose intake2ControlPoint = new Pose(9, 20);
    private Pose intake2ReadyPose =  new Pose(14, 20, Math.toRadians(135));
    private Pose intake2FinishPose = new Pose(9, 38, Math.toRadians(135));


    private Pose intake3ControlPoint = new Pose(7.5, 20);
    private Pose intake3ReadyPose =  new Pose(14, 28, Math.toRadians(225));
    private Pose intake3FinishPose = new Pose(7.5, 12, Math.toRadians(185));
    private Pose launch3ControlPoint = new Pose(30, 20);


    private Pose leavePose = new Pose(36, 12, Math.toRadians(270));

    private PathChain launchPath1, intakePathReady1,intakePath1, launchPath2, intakePathReady2,intakePath2, launchPath3, intakePathReady3, intakePath3, launchPath4, leavePath, hitLever1;

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
            intake2ReadyPose = intake2ReadyPose.mirror();
            intake2ControlPoint = intake2ControlPoint.mirror();
            intake2FinishPose = intake2FinishPose.mirror();
            intake3ReadyPose = intake3ReadyPose.mirror();
            launch3ControlPoint = launch3ControlPoint.mirror();
            intake3ControlPoint = intake3ControlPoint.mirror();
            intake3FinishPose = intake3FinishPose.mirror();
            leavePose = leavePose.mirror();
            telemetry.addLine("red");
        } else {
            telemetry.addLine("blue");
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
        intakePathReady2 = follower.pathBuilder()
                .addPath(new BezierLine(  launchPose, intake2ReadyPose  ))
                .setLinearHeadingInterpolation(launchPose.getHeading(), intake2ReadyPose.getHeading())
                .build();
        intakePath2 = follower.pathBuilder()
                .addPath(new BezierCurve(  intake2ReadyPose, intake2ControlPoint, intake2FinishPose  ))
                .setLinearHeadingInterpolation(intake2ReadyPose.getHeading(), intake2FinishPose.getHeading())
                .build();

        // ....... Launch 3
        launchPath3 = follower.pathBuilder()
                .addPath(new BezierLine(  intake2FinishPose, launchPose  ))
                .setLinearHeadingInterpolation(intake2FinishPose.getHeading(), launchPose.getHeading())
                .build();

        // ....... Intake 3
        intakePathReady3 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, intake3ReadyPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), intake3ReadyPose.getHeading())
                .build();
        intakePath3 = follower.pathBuilder()
                .addPath(new BezierCurve(intake3ReadyPose, intake3ControlPoint, intake3FinishPose))
                .setLinearHeadingInterpolation(intake3ReadyPose.getHeading(), intake3FinishPose.getHeading())
                .build();

        // ....... Launch 4
        launchPath4 = follower.pathBuilder()
                .addPath(new BezierCurve(intake3FinishPose, launch3ControlPoint, launchPose))
                .setLinearHeadingInterpolation(intake3FinishPose.getHeading(), launchPose.getHeading())
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
                    outtake.isFar = true;
                    outtake.setOuttakeVelocity(true);
                    outtake.setServoPosition(0.4);
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
                    outtake.setServoPosition(0.48);
                    pathState = 3;
                }
                break;
            case 3:
                /* Let the robot get to the first line of balls */

                if (!follower.isBusy()) {
                    // Intake the first line of balls
                    outtake.setIntakePower(1);
                    follower.followPath(intakePath1, 0.6, true);
                    pathState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    // Stop the intake and drive back to launch position
                    outtake.setIntakePower(-0.3);
                    outtake.setServoPosition(0.4);
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
                    outtake.setServoPosition(0.48);
                    follower.followPath(intakePathReady3);
                    pathState = 8;
                }
                break;
            case 8:
                /* Let the robot get to the second line of balls of balls */

                if (!follower.isBusy()) {
                    // Intake the second line of balls of balls
                    outtake.setIntakePower(1);
                    follower.followPath(intakePath3, 0.6, true);
                    pathState = 9;
                    timer.reset();
                }
                break;
            case 9:
                /* Let the intake sequence play out */

                if (!follower.isBusy()  || timer.seconds() > 3.5) {
                    // Stop the intake and drive back to launch position
                    outtake.setIntakePower(-0.3);
                    outtake.setServoPosition(0.4);
                    outtake.setOuttakeVelocity(true);
                    follower.followPath(launchPath4, true);
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
                    outtake.setServoPosition(0.48);
                    if (timer.seconds() > 0.6) {
                        outtake.setIntakePower(1);
                    }
                    follower.followPath(intakePathReady2, true);
                    pathState = 12;
                }
                break;
            case 12:
                /* Let the intake sequence play out */

                if (!follower.isBusy()) {
                    // intake third line of balls
                    outtake.setServoPosition(0.48);
                    follower.followPath(intakePath2, 0.65, true);
                    timer.reset();
                    pathState = 13;
                }
                break;
            case 13:
                if (!follower.isBusy() || timer.seconds() > 4.5) {
                    // stop intake and goto launch
                    outtake.setIntakePower(-0.3);
                    outtake.setServoPosition(0.4);
                    outtake.setOuttakeVelocity(true);
                    follower.followPath(launchPath3, true);
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
                    outtake.setServoPosition(0.48);
                    if (timer.seconds() > 0.6) {
                        outtake.setIntakePower(1);
                    }
                    follower.followPath(intakePathReady2, true);
                    pathState = 16;
                }
                break;
            case 16:
                /* Let the intake sequence play out */

                if (!follower.isBusy()) {
                    // intake third line of balls
                    outtake.setServoPosition(0.48);
                    follower.followPath(intakePath2, 0.65, true);
                    timer.reset();
                    pathState = 17;
                }
                break;
            case 17:
                if (!follower.isBusy() || timer.seconds() > 4.5) {
                    // stop intake and goto launch
                    outtake.setIntakePower(-0.3);
                    outtake.setServoPosition(0.4);
                    outtake.setOuttakeVelocity(true);
                    follower.followPath(launchPath3, true);
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