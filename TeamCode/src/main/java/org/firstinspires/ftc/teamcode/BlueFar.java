package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.Mechanisms.Outtake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class BlueFar extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    // -------- LAUNCH SETUP --------
    private Outtake shooter = new Outtake();

    private boolean shotsTriggered = false;


    public enum PathState {
        // START POSITION_END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT > ATTEMPT TO SCORE ARTIFACT
        DRIVESTART_SHOOT,
        SHOOT_PRELOAD,
        DRIVESHOOT_GOTO1,
        DRIVEGOTO1_PICKUP1,
        SHOOT_1,
        DRIVESHOOT1_PARK

    }

    PathState pathState;

    private final Pose startPose = new Pose(56,10,Math.toRadians(-76.5));
    private final Pose shootPose = new Pose(54,18,Math.toRadians(-76.5));
    private final Pose goto1Pose = new Pose(44,36,Math.toRadians(-180));
    private final Pose pickup1Pose = new Pose(9,36,Math.toRadians(-180));
    private final Pose parkPose = new Pose(36,12,Math.toRadians(-90));



    private PathChain driveStart_Shoot, driveShoot_Goto1, driveGoto1_Pickup1, drivePickup1_Shoot1, driveShoot1_Park;

    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
        driveStart_Shoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        driveShoot_Goto1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, goto1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), goto1Pose.getHeading())
                .build();
        driveGoto1_Pickup1 = follower.pathBuilder()
                .addPath(new BezierLine(goto1Pose, pickup1Pose))
                .setLinearHeadingInterpolation(goto1Pose.getHeading(), pickup1Pose.getHeading())
                .build();
        drivePickup1_Shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, shootPose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), shootPose.getHeading())
                .build();
        driveShoot1_Park = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVESTART_SHOOT:
                follower.followPath(driveStart_Shoot, true);
                setPathState(PathState.SHOOT_PRELOAD); // resets timer and makes new state at same time
                //pathState = PathState.SHOOT_PRELOAD); // rather than this
                break;
            case SHOOT_PRELOAD:
                // check if follower done it's path
                if (!follower.isBusy()) {
                    // requested shots yet?
                    if (!shotsTriggered) {
                        shooter.fireShots(3);
                        shotsTriggered = true;
                    } else if (shotsTriggered && !shooter.isBusy()) {
                        // shots are done free to transition
                        follower.followPath(driveShoot_Goto1, true);
                        setPathState(PathState.DRIVESHOOT_GOTO1);
                    }

                }

                break;
            case DRIVESHOOT_GOTO1:
                if (!follower.isBusy()) {
                    // TODO add intake stuff

                    follower.followPath(driveGoto1_Pickup1, true);
                    setPathState(PathState.DRIVEGOTO1_PICKUP1);
                }
                break;
            case DRIVEGOTO1_PICKUP1:
                if (!follower.isBusy()) {
                    // TODO add intake stuff

                    follower.followPath(drivePickup1_Shoot1, true);
                    setPathState(PathState.SHOOT_1);
                }
                break;
            case SHOOT_1:
                if (!follower.isBusy()) {
                    // requested shots yet?
                    if (!shotsTriggered) {
                        shooter.fireShots(3);
                        shotsTriggered = true;
                    } else if (shotsTriggered && !shooter.isBusy()) {
                        // shots are done free to transition
                        follower.followPath(driveShoot_Goto1, true);
                        setPathState(PathState.DRIVESHOOT_GOTO1);
                    }

                }
                break;
            case DRIVESHOOT1_PARK:
                if (!follower.isBusy()) {
                    telemetry.addLine("Done all Paths");
                }
            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();

        shotsTriggered = false;
    }

    @Override
    public void init() {
        pathState = PathState.DRIVESTART_SHOOT;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        // TODO add in any other init mechanisms
        shooter.init(hardwareMap);

        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update(); //must always include at start
        shooter.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());


    }
}
