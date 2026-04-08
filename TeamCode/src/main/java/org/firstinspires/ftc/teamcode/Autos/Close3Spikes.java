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

@Autonomous(name = "Close3Spikes", group = "Auto")
public class Close3Spikes extends OpMode {

    public Follower follower;
    private int pathState;
    private boolean red;

    ElapsedTime delayTimer = new ElapsedTime();
    ElapsedTime autoTimer = new ElapsedTime();
    double delaySeconds = 0.0;
    final double AUTO_LENGTH_SECONDS = 30.0;
    final double AUTO_END_BUFFER_SECONDS = 1.0;

    OuttakeFR outtake = new OuttakeFR();
    Webcam webcam = new Webcam(hardwareMap);

    // Define important coordinate locations for the Blue side of the field
    private Pose startPose = new Pose(22, 122, Math.toRadians(360-45-180));
    //private Pose startControlPoint = new Pose(60, 110);
    private Pose launchPose = new Pose(54,84,Math.toRadians(310-180));
    private Pose launchPose2 = new Pose(54,84,Math.toRadians(310-180));




    private Pose intake1ReadyPose =  new Pose(47, 84, Math.toRadians(180));
    private Pose intake1FinishPose = new Pose(20, 84, Math.toRadians(180));


    private Pose intake2ControlPoint = new Pose(64, 56);
    private Pose intake2ReadyPose =  new Pose(48, 60, Math.toRadians(180));
    private Pose intake2FinishPose = new Pose(16, 60, Math.toRadians(180));


    private Pose intakeRampControlPoint = new Pose(48, 52);
    private Pose intakeRampReadyPose =  new Pose(18, 55, Math.toRadians(145));
    private Pose intakeRampFinishPose = new Pose(10, 56, Math.toRadians(145));


    private Pose intake3ControlPoint = new Pose(56, 30);
    private Pose intake3ReadyPose =  new Pose(50, 36, Math.toRadians(180));
    private Pose intake3FinishPose = new Pose(16, 36, Math.toRadians(180));

    private Pose leavePose = new Pose(44, 80, Math.toRadians(310));

    private PathChain launchPath1, intakePathReady1,intakePath1, launchPath2, intakePathReady2,intakePath2,  launchPath3, launchPathRamp, intakePathReadyRamp,intakePathRamp, intakePathReady3, intakePath3, launchPath4, leavePath, hitLever1;

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {


        // Mirror coordinates across the x-Axis if the autonomous is run on the Red side
        if (gamepad1.dpad_right || gamepad2.dpad_right) {
            startPose = startPose.mirror();
            //startControlPoint = startControlPoint.mirror();
            launchPose = new Pose(90, 84, Math.toRadians(225-180));
            launchPose2 = new Pose(90, 84, Math.toRadians(228-180));
            intake1ReadyPose = intake1ReadyPose.mirror();
            intake1FinishPose = intake1FinishPose.mirror();
            intake2ReadyPose = intake2ReadyPose.mirror();
            intake2ControlPoint = intake2ControlPoint.mirror();
            intake2FinishPose = intake2FinishPose.mirror();
            intakeRampReadyPose = intakeRampReadyPose.mirror();
            intakeRampControlPoint = intakeRampControlPoint.mirror();
            intakeRampFinishPose = intakeRampFinishPose.mirror();
            intake3ReadyPose = intake3ReadyPose.mirror();
            intake3ControlPoint = intake3ControlPoint.mirror();
            intake3FinishPose = intake3FinishPose.mirror();
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
                .addPath(new BezierLine(  startPose, launchPose2  ))
                .setLinearHeadingInterpolation(startPose.getHeading(), launchPose2.getHeading()).build();

        // ....... Intake 1
        intakePathReady1 = follower.pathBuilder()
                .addPath(new BezierLine(  launchPose, intake1ReadyPose  ))
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
                .addPath(new BezierCurve(  launchPose, intake2ControlPoint, intake2ReadyPose  ))
                .setLinearHeadingInterpolation(launchPose.getHeading(), intake2ReadyPose.getHeading())
                .build();
        intakePath2 = follower.pathBuilder()
                .addPath(new BezierLine(  intake2ReadyPose, intake2FinishPose  ))
                .setTangentHeadingInterpolation()
                .build();

        // ....... Launch 3
        launchPath3 = follower.pathBuilder()
                .addPath(new BezierLine(  intake2FinishPose, launchPose  ))
                .setLinearHeadingInterpolation(intake2FinishPose.getHeading(), launchPose.getHeading())
                .build();

        // ....... Intake Ramp
        intakePathReadyRamp = follower.pathBuilder()
                .addPath(new BezierCurve(  launchPose, intakeRampControlPoint, intakeRampReadyPose  ))
                .setLinearHeadingInterpolation(launchPose.getHeading(), intakeRampReadyPose.getHeading())
                .build();
        intakePathRamp = follower.pathBuilder()
                .addPath(new BezierLine(  intakeRampReadyPose, intakeRampFinishPose  ))
                .setLinearHeadingInterpolation(intakeRampReadyPose.getHeading(), intakeRampFinishPose.getHeading())
                .build();

        // ....... Launch Ramp
        launchPathRamp = follower.pathBuilder()
                .addPath(new BezierLine(  intakeRampFinishPose, launchPose  ))
                .setLinearHeadingInterpolation(intakeRampFinishPose.getHeading(), launchPose.getHeading())
                .build();

        // ....... Intake 3
        intakePathReady3 = follower.pathBuilder()
                .addPath(new BezierCurve(launchPose, intake3ControlPoint, intake3ReadyPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), intake3ReadyPose.getHeading())
                .build();
        intakePath3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3ReadyPose, intake3FinishPose))
                .setTangentHeadingInterpolation()
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
                    outtake.isFar = false;
                    outtake.setOuttakeVelocity(false);
                    outtake.setServoPosition(0.4);
                    follower.followPath(launchPath1,true);
                    pathState = 1;
                }
                break;
            case 1:
                /* Let the robot get to launch position */

                if (!follower.isBusy()) {
                    // Begin the first launch sequence
                    outtake.fireShots(3);
                    pathState = 2;
                    timer.reset();
                }
                break;
            case 2:
                /* Let the first launch sequence play out */

                if (!outtake.isBusy()) {
                    // drive to the first line of balls
                    follower.followPath(intakePathReady1, true);
                    outtake.setIntakePower(1);
                    outtake.setServoPosition(0.48);
                    pathState = 3;
                }
                break;
            case 3:
                /* Let the robot get to the first line of balls */

                if (!follower.isBusy()) {
                    // Intake the first line of balls
                    follower.followPath(intakePath1, 0.85, true);
                    pathState = 5;
                    timer.reset();
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    // Stop the intake and drive back to launch position
                    outtake.setServoPosition(0.4);
                    outtake.setOuttakeVelocity(false);
                    follower.followPath(launchPath2, true);
                    pathState = 6;
                    timer.reset();
                }
                break;
            case 6:
                /* Let the robot get back to launch position */
                if (timer.seconds() > 0.4) {
                    outtake.setIntakePower(-0.3);
                } else {
                    outtake.setIntakePower(0);
                }
                if (!follower.isBusy()) {
                    // Begin the second launch sequence
                    outtake.setIntakePower(0);
                    outtake.fireShots(3);
                    pathState = 7;
                    timer.reset();
                }
                break;
            case 7:
            /* Let the second launch sequence play out */

                if (!outtake.isBusy()) {
                    // Drive to the second line of balls of balls
                    outtake.setServoPosition(0.48);
                    outtake.setIntakePower(1);
                    follower.followPath(intakePathReady2,true);
                    pathState = 8;
                }
                break;
            case 8:
                /* Let the robot get to the second line of balls of balls */

                if (!follower.isBusy()) {
                    // Intake the second line of balls of balls
                    follower.followPath(intakePath2,0.85, true);
                    pathState = 9;
                    timer.reset();
                }
                break;
            case 9:
                /* Let the intake sequence play out */

                if (!follower.isBusy()) {
                    // Stop the intake and drive back to launch position
                    outtake.setServoPosition(0.4);
                    outtake.setOuttakeVelocity(false);
                    follower.followPath(launchPath3, true);
                    pathState = 10;
                    timer.reset();
                }
                break;
            case 10:
                /* Let the robot get back to launch position */
                if (timer.seconds() > 0.4) {
                    outtake.setIntakePower(-0.3);
                } else {
                    outtake.setIntakePower(0);
                }
                if (!follower.isBusy()) {
                    // Begin the second launch sequence
                    outtake.setIntakePower(0);
                    outtake.fireShots(3);
                    pathState = 101;
                    timer.reset();
                }
                break;
            case 101:
                /* Let the second launch sequence play out */

                if (!outtake.isBusy()) {
                    // Drive to the ramp of balls
                    outtake.setServoPosition(0.48);
                    follower.followPath(intakePathReadyRamp,true);
                    pathState = 102;
                }
                break;
            case 102:
                /* Let the robot get to the ramp of balls */

                if (!follower.isBusy()) {
                    // Intake the ramp of balls
                    outtake.setIntakePower(1);
                    follower.followPath(intakePathRamp,0.8, true);
                    pathState = 103;
                    timer.reset();
                }
                break;
            case 103:
                /* Let the intake sequence play out */

                if (timer.seconds() > 2.2) {
                    // Stop the intake and drive back to launch position
                    outtake.setServoPosition(0.4);
                    outtake.setOuttakeVelocity(false);
                    follower.followPath(launchPathRamp, true);
                    pathState = 104;
                    timer.reset();
                }
                break;
            case 104:
                /* Let the robot get back to launch position */
                if (timer.seconds() > 0.6) {
                    outtake.setIntakePower(-0.3);
                } else {
                    outtake.setIntakePower(0);
                }
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
                    outtake.setIntakePower(1);
                    follower.followPath(intakePathReady3,true);
                    pathState = 12;
                }
                break;
            case 12:
                /* Let the intake sequence play out */

                if (!follower.isBusy()) {
                    // intake third line of balls
                    follower.followPath(intakePath3, 0.85, true);
                    pathState = 13;
                    timer.reset();

                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    // stop intake and goto launch

                    outtake.setServoPosition(0.4);
                    outtake.setOuttakeVelocity(false);
                    follower.followPath(launchPath4, true);
                    pathState = 14;
                    timer.reset();
                }
                break;
            case 14:
                if (timer.seconds() > 0.4) {
                    outtake.setIntakePower(-0.3);
                } else {
                    outtake.setIntakePower(0);
                }
                if (!follower.isBusy()) {
                    // launch
                    outtake.setIntakePower(0);
                    timer.reset();
                    outtake.fireShots(3);
                    pathState = 15;
                }
                break;

            case 15:
                /* Let the third launch sequence play out */

                // If the launch sequence is finished, or autonomous is about to end, move sideways for the Leave points
                if (!outtake.isBusy()) {
                    outtake.setOuttakeVelocity(0);
                    // Quit out of the state machine and move off of the Launch line
                    follower.followPath(leavePath, true);
                    pathState = -1;
                }
                break;
        }
    }
}