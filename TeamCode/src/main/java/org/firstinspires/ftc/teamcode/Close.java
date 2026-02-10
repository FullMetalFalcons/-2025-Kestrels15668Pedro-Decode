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

@Autonomous(name = "Close", group = "Auto")
public class Close extends OpMode {

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
    private Pose startPose = new Pose(18, 118.5, Math.toRadians(-50));
    private Pose launchPose = new Pose(48,84,Math.toRadians(-50));

    //private Pose intake1ControlPoint = new Pose(48, 104);
    private Pose intake1ReadyPose =  new Pose(44, 60, Math.toRadians(180));
    private Pose intake1FinishPose = new Pose(18, 60, Math.toRadians(180));
    private Pose launch1ControlPoint = new Pose(48, 60, Math.toRadians(180));


    private Pose intake2ControlPoint = new Pose(48, 61);
    private Pose intake2ReadyPose =  new Pose(18, 61, Math.toRadians(145));
    private Pose intake2FinishPose = new Pose(12, 61, Math.toRadians(145));


    private Pose launch3ControlPoint = new Pose(48, 60);
    //private Pose hitLever = new Pose(17, 75, Math.toRadians(180));
    //private Pose hitLeverControlPoint = new Pose(40, 80);

    private Pose intake3ReadyPose = new Pose(48, 84, Math.toRadians(180));
    private Pose intake3FinishPose = new Pose(18, 84, Math.toRadians(180));
    //private Pose launch4ControlPoint = new Pose(55, 58);

    private Pose leavePose = new Pose(44, 84, Math.toRadians(-50));

    private PathChain launchPath1, intakePathReady1,intakePath1, launchPath2, intakePathReady2,intakePath2, launchPath3, intakePathReady3, intakePath3, launchPath4, leavePath, hitLever1;


    @Override
    public void init() {


        // Mirror coordinates across the x-Axis if the autonomous is run on the Red side
        if (gamepad1.dpad_right) {
            startPose = startPose.mirror();
            launchPose = launchPose.mirror();
            intake1ReadyPose = intake1ReadyPose.mirror();
            intake1FinishPose = intake1FinishPose.mirror();
            intake2ReadyPose = intake2ReadyPose.mirror();
            intake2ControlPoint = intake2ControlPoint.mirror();
            intake2FinishPose = intake2FinishPose.mirror();
            intake3ReadyPose = intake3ReadyPose.mirror();
            intake3FinishPose = intake3FinishPose.mirror();
            launch1ControlPoint = launch1ControlPoint.mirror();
            launch3ControlPoint = launch3ControlPoint.mirror();
            leavePose = leavePose.mirror();
        }

        follower = Constants.createFollower(hardwareMap); // Make sure you create the follower before building paths
        buildPaths();
        follower.setStartingPose(startPose);

        // Initialize external systems
        webcam = new Webcam(hardwareMap);
        webcam.init(hardwareMap, telemetry);
        outtake.init(hardwareMap);
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
                .addPath(new BezierLine(  launchPose, intake1ReadyPose  ))
                .setLinearHeadingInterpolation(launchPose.getHeading(), intake1ReadyPose.getHeading()).build();
        intakePath1 = follower.pathBuilder()
                .addPath(new BezierLine(  intake1ReadyPose, intake1FinishPose  ))
                .setTangentHeadingInterpolation().build();

        // ....... Launch 2
        launchPath2 = follower.pathBuilder()
                .addPath(new BezierCurve(  intake1FinishPose, launch1ControlPoint, launchPose  ))
                .setLinearHeadingInterpolation(intake1FinishPose.getHeading(), launchPose.getHeading()).build();

        // ....... Intake 2
        intakePathReady2 = follower.pathBuilder()
                .addPath(new BezierCurve(  launchPose, intake2ControlPoint, intake2ReadyPose  ))
                .setLinearHeadingInterpolation(launchPose.getHeading(), intake2ReadyPose.getHeading()).build();
        intakePath2 = follower.pathBuilder()
                .addPath(new BezierLine(  intake2ReadyPose, intake2FinishPose  ))
                .setLinearHeadingInterpolation(intake2ReadyPose.getHeading(), intake2FinishPose.getHeading()).build();

        // ....... Launch 3
        launchPath3 = follower.pathBuilder()
                .addPath(new BezierCurve(  intake2FinishPose, launch3ControlPoint, launchPose  ))
                .setLinearHeadingInterpolation(intake2FinishPose.getHeading(), launchPose.getHeading()).build();

        // ....... Intake 3
        intakePathReady3 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, intake3ReadyPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), intake3ReadyPose.getHeading())
                .build();
        intakePath3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3ReadyPose, intake3FinishPose))
                .setLinearHeadingInterpolation(intake3ReadyPose.getHeading(), intake3FinishPose.getHeading())
                .build();

        // ....... Launch 4
        launchPath4 = follower.pathBuilder()
                .addPath(new BezierLine(intake3FinishPose, launchPose))
                .setLinearHeadingInterpolation(intake3FinishPose.getHeading(), launchPose.getHeading())
                .build();

        // ....... Leave Points
        leavePath = follower.pathBuilder()
                .addPath(new BezierLine(  launchPose, leavePose  ))
                .setConstantHeadingInterpolation(leavePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {

        // Autonomous state machine
        switch (pathState) {
            case 0:
                // Wait for the starting delay to expire
                if (delayTimer.seconds() > delaySeconds) {
                    // Begin the whole route
                    outtake.setOuttakeVelocity(1680);
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
                    follower.followPath(intakePathReady1);
                    outtake.setServoPosition(0.48);
                    pathState = 3;
                }
                break;
            case 3:
                /* Let the robot get to the first line of balls */

                if (!follower.isBusy()) {
                    // Intake the first line of balls
                    outtake.setIntakePower(1);
                    follower.followPath(intakePath1, 0.8, true);
                    pathState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    // Stop the intake and drive back to launch position
                    outtake.setIntakePower(-0.1);
                    outtake.setServoPosition(0.4);
                    outtake.setOuttakeVelocity(1680);
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
                    // Drive to the gate of balls
                    outtake.setServoPosition(0.48);
                    follower.followPath(intakePathReady2);
                    pathState = 8;
                }
                break;
            case 8:
                /* Let the robot get to the gate of balls */

                if (!follower.isBusy()) {
                    // Intake the gate of balls
                    outtake.setIntakePower(1);
                    follower.followPath(intakePath2, 0.8, true);
                    pathState = 9;
                }
                break;
            case 9:
                /* Let the intake sequence play out */

                if (!follower.isBusy()) {
                    // Stop the intake and drive back to launch position
                    outtake.setIntakePower(-0.1);
                    outtake.setServoPosition(0.4);
                    outtake.setOuttakeVelocity(1680);
                    follower.followPath(launchPath3, true);
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
                }
                break;
            case 11:
                /* Let the robot get to the second line of balls */

                if (!outtake.isBusy()) {
                    outtake.setServoPosition(0.48);
                    follower.followPath(intakePathReady3, true);
                    pathState = 12;
                }
                break;
            case 12:
                /* Let the intake sequence play out */

                if (!follower.isBusy()) {
                    // intake second line of balls
                    outtake.setIntakePower(1);
                    follower.followPath(intakePath3, 0.8, true);
                    pathState = 13;
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    // stop intake and goto launch
                    outtake.setIntakePower(-0.1);
                    outtake.setServoPosition(0.4);
                    outtake.setOuttakeVelocity(1680);
                    follower.followPath(launchPath4, true);
                    pathState = 14;
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    // launch
                    outtake.setIntakePower(0);
                    outtake.fireShots(3);
                    pathState = 15;
                }
                break;

            case 15:
                /* Let the third launch sequence play out */

                // If the launch sequence is finished, or autonomous is about to end, move sideways for the Leave points
                if (autoTimer.seconds() > AUTO_LENGTH_SECONDS - AUTO_END_BUFFER_SECONDS || !outtake.isBusy()
                ) {

                    // Quit out of the state machine and move off of the Launch line
                    follower.followPath(leavePath, true);
                    pathState = -1;
                }
                break;
        }
    }
}