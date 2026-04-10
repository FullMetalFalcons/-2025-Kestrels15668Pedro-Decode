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

@Autonomous(name = "Close2Spikes18", group = "Auto")
public class Close2Spikes18 extends OpMode {

    public Follower follower;
    private int pathState;
    private boolean red;

    ElapsedTime delayTimer = new ElapsedTime();
    ElapsedTime autoTimer = new ElapsedTime();
    double delaySeconds = 0.0;

    OuttakeFR outtake = new OuttakeFR();
    Webcam webcam = new Webcam(hardwareMap);

    // Define important coordinate locations for the Blue side of the field
    private Pose startPose = new Pose(22, 122, Math.toRadians(360-45));
    private Pose launchPose = new Pose(54,84,Math.toRadians(310));
    private Pose launchPose2 = new Pose(54,84,Math.toRadians(310));



    private Pose intake1ReadyPose =  new Pose(48, 84, Math.toRadians(180));
    private Pose intake1FinishPose = new Pose(21, 84, Math.toRadians(180));


    private Pose intake2ControlPoint = new Pose(72, 52);
    private Pose intake2Pose = new Pose(16, 61, Math.toRadians(180));
    private Pose launch2ControlPoint = new Pose(61, 55);


    private Pose intakeRampControlPoint = new Pose(48, 52);
    private Pose intakeRampReadyPose =  new Pose(18, 55, Math.toRadians(145));
    private Pose intakeRampFinishPose = new Pose(10, 56, Math.toRadians(145));
    private Pose launchRampControlPoint = new Pose(61,45);


    private Pose launchLeavePose = new Pose(54, 104, Math.toRadians(330));

    private PathChain launchPath1, intakePathReady1,intakePath1, launchPath2, intakePath2, launchPathRamp, intakePathReadyRamp, intakePathRamp;

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {


        // Mirror coordinates across the x-Axis if the autonomous is run on the Red side
        if (gamepad1.dpad_right || gamepad2.dpad_right) {
            startPose = startPose.mirror();
            launchPose = new Pose(90, 84, Math.toRadians(225));
            launchPose2 = new Pose(90, 84, Math.toRadians(228));
            intake1ReadyPose = intake1ReadyPose.mirror();
            intake1FinishPose = intake1FinishPose.mirror();
            intake2ControlPoint = intake2ControlPoint.mirror();
            intake2Pose = intake2Pose.mirror();
            launch2ControlPoint = launch2ControlPoint.mirror();
            intakeRampReadyPose = intakeRampReadyPose.mirror();
            intakeRampControlPoint = intakeRampControlPoint.mirror();
            intakeRampFinishPose = intakeRampFinishPose.mirror();
            launchRampControlPoint = launchRampControlPoint.mirror();
            launchLeavePose = launchLeavePose.mirror();
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

        // ....... Intake 2
        intakePath2 = follower.pathBuilder()
                .addPath(new BezierCurve(  launchPose, intake2ControlPoint, intake2Pose  ))
                .setLinearHeadingInterpolation(launchPose.getHeading(), intake2Pose.getHeading()).build();

        // ....... Launch 2
        launchPath2 = follower.pathBuilder()
                .addPath(new BezierCurve(  intake2Pose, launch2ControlPoint, launchPose  ))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), launchPose.getHeading())
                .build();

        // ....... Intake 1
        intakePathReady1 = follower.pathBuilder()
                .addPath(new BezierLine(  launchPose, intake1ReadyPose  ))
                .setLinearHeadingInterpolation(launchPose.getHeading(), intake1ReadyPose.getHeading())
                .build();
        intakePath1 = follower.pathBuilder()
                .addPath(new BezierLine(  intake1ReadyPose, intake1FinishPose  ))
                .setTangentHeadingInterpolation()
                .build();

        // ....... Launch 1
        launchPath1 = follower.pathBuilder()
                .addPath(new BezierLine(  intake1FinishPose, launchLeavePose  ))
                .setLinearHeadingInterpolation(intake1FinishPose.getHeading(), launchLeavePose.getHeading())
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
                .addPath(new BezierCurve(  intakeRampFinishPose, launchRampControlPoint, launchPose  ))
                .setLinearHeadingInterpolation(intakeRampFinishPose.getHeading(), launchPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {

        // Autonomous state machine
        switch (pathState) {
            case 0:
                // Wait for the starting delay to expire
                if (delayTimer.seconds() > delaySeconds) {
                    // Begin the whole route
                    outtake.setOuttakeVelocity(false);

                    follower.followPath(launchPath1,true);
                    outtake.fireShots(3);
                    pathState = 1;
                }
                break;
            case 1:
                /* Let the first launch sequence play out */

                if (!outtake.isBusy() && !follower.isBusy()) {
                    // drive to the first line of balls
                    follower.followPath(intakePath2, true);
                    outtake.setIntakePower(1);
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    // Stop the intake and drive back to launch position

                    follower.followPath(launchPath2, true);
                    pathState = 3;
                    timer.reset();
                }
                break;
            case 3:
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
                    pathState = 4;
                    timer.reset();
                }
                break;
            case 4:
                /* Let the second launch sequence play out */

                if (!outtake.isBusy()) {
                    // Drive to the ramp of balls
                    follower.followPath(intakePathReadyRamp,true);
                    pathState = 5;
                }
                break;
            case 5:
                /* Let the robot get to the ramp of balls */

                if (!follower.isBusy()) {
                    // Intake the ramp of balls
                    outtake.setIntakePower(1);
                    follower.followPath(intakePathRamp,0.8, true);
                    pathState = 6;
                    timer.reset();
                }
                break;
            case 6:
                /* Let the intake sequence play out */

                if (timer.seconds() > 2.2) {
                    // Stop the intake and drive back to launch position

                    follower.followPath(launchPathRamp, true);
                    pathState = 7;
                    timer.reset();
                }
                break;
            case 7:
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
                    pathState = 8;
                    timer.reset();
                }
                break;
            case 8:
                /* Let the second launch sequence play out */

                if (!outtake.isBusy()) {
                    // Drive to the ramp of balls
                    follower.followPath(intakePathReadyRamp,true);
                    pathState = 9;
                }
                break;
            case 9:
                /* Let the robot get to the ramp of balls */

                if (!follower.isBusy()) {
                    // Intake the ramp of balls
                    outtake.setIntakePower(1);
                    follower.followPath(intakePathRamp,0.8, true);
                    pathState = 10;
                    timer.reset();
                }
                break;
            case 10:
                /* Let the intake sequence play out */

                if (timer.seconds() > 2.2) {
                    // Stop the intake and drive back to launch position

                    follower.followPath(launchPathRamp, true);
                    pathState = 11;
                    timer.reset();
                }
                break;
            case 11:
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
                    pathState = 12;
                    timer.reset();
                }
                break;
            case 12:
                /* Let the second launch sequence play out */

                if (!outtake.isBusy()) {
                    // Drive to the ramp of balls
                    follower.followPath(intakePathReadyRamp,true);
                    pathState = 13;
                }
                break;
            case 13:
                /* Let the robot get to the ramp of balls */

                if (!follower.isBusy()) {
                    // Intake the ramp of balls
                    outtake.setIntakePower(1);
                    follower.followPath(intakePathRamp,0.8, true);
                    pathState = 14;
                    timer.reset();
                }
                break;
            case 14:
                /* Let the intake sequence play out */

                if (timer.seconds() > 2.2) {
                    // Stop the intake and drive back to launch position

                    follower.followPath(launchPathRamp, true);
                    pathState = 15;
                    timer.reset();
                }
                break;
            case 15:
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
                    pathState = 16;
                    timer.reset();
                }
                break;
            case 16:
                /* Let the second launch sequence play out */

                if (!outtake.isBusy()) {
                    // Drive to the ramp of balls
                    follower.followPath(intakePathReady1,true);
                    pathState = 17;
                }
                break;
            case 17:
                /* Let the robot get to the ramp of balls */

                if (!follower.isBusy()) {
                    // Intake the ramp of balls
                    outtake.setIntakePower(1);
                    follower.followPath(intakePath1, true);
                    pathState = 18;
                    timer.reset();
                }
                break;
            case 18:
                /* Let the intake sequence play out */

                if (timer.seconds() > 2.2) {
                    // Stop the intake and drive back to launch position

                    follower.followPath(launchPath1, true);
                    pathState = 19;
                    timer.reset();
                }
                break;
            case 19:
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
                    pathState = 20;
                    timer.reset();
                }
                break;
            case 20:
                /* Let the launch sequence play out */

                // If the launch sequence is finished, or autonomous is about to end, move sideways for the Leave points
                if (!outtake.isBusy()) {
                    outtake.setOuttakeVelocity(0);
                    // Quit out of the state machine and move off of the Launch line
                    pathState = -1;
                }
                break;
        }
    }
}