package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.OuttakeFR;
import org.firstinspires.ftc.teamcode.Mechanisms.Webcam;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "TestAuto", group = "Auto")
public class TestAuto extends OpMode {

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
    private Pose startPose = new Pose(56, 84, Math.toRadians(-90));
    private Pose launchPose = new Pose(56,84,Math.toRadians(-90));

    //private Pose intake1ControlPoint = new Pose(48, 104);
    private Pose intake1ReadyPose =  new Pose(44, 84, Math.toRadians(180));
    private Pose intake1FinishPose = new Pose(18, 84, Math.toRadians(180));




    private PathChain launchPath1, intakePathReady1, intakePath1, launchPath2;

    @Override
    public void init() {


        follower = Constants.createFollower(hardwareMap); // Make sure you create the follower before building paths
        buildPaths();
        follower.setStartingPose(startPose);

        // Initialize external systems
        webcam = new Webcam(hardwareMap);
        webcam.init(hardwareMap, telemetry);
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
    }

    public void buildPaths() {
        // ....... Launch 1
        launchPath1 = follower.pathBuilder()
                .addPath(new BezierLine(  startPose, launchPose  ))
                .setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading()).build();

        // ....... Intake 1
        intakePathReady1 = follower.pathBuilder()
                .addPath(new BezierLine(  launchPose, intake1ReadyPose  ))
                .setTangentHeadingInterpolation().build();
        intakePath1 = follower.pathBuilder()
                .addPath(new BezierLine(  intake1ReadyPose, intake1FinishPose  ))
                .setTangentHeadingInterpolation().build();

        // ....... Launch 2
        launchPath2 = follower.pathBuilder()
                .addPath(new BezierLine(  intake1FinishPose, launchPose  ))
                .setLinearHeadingInterpolation(intake1FinishPose.getHeading(), launchPose.getHeading()).build();

    }

    public void autonomousPathUpdate() {

        // Autonomous state machine
        switch (pathState) {
            case 0:
                // Wait for the starting delay to expire
                if (delayTimer.seconds() > delaySeconds) {
                    // Begin the whole route
                    outtake.setOuttakeVelocity(1800);
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

                if (!outtake.isBusy())
                {
                    // drive to the first line of balls
                    follower.followPath(intakePathReady1);
                    pathState = 3;
                }
                break;
            case 3:
                /* Let the robot get to the first line of balls */

                if (!follower.isBusy()) {
                    // Intake the first line of balls
                    outtake.setIntakePower(0.8);
                    follower.followPath(intakePath1, 0.4, true);
                    pathState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    // Stop the intake and drive back to launch position
                    outtake.setIntakePower(0);
                    outtake.setOuttakeVelocity(1800);
                    follower.followPath(launchPath2, true);
                    pathState = 5;
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    outtake.fireShots(3);
                    pathState = 6;
                }
                break;
            case 6:
                if(!outtake.isBusy()) {
                    //pathState = 7
                }
                break;
        }
    }
}