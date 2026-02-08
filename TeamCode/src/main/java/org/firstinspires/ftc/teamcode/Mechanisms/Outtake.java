package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Outtake {
    private Servo servoTrigger;
    private DcMotorEx motorLaunch, motorRamp1, motorRamp2, motorIntake;

    private ElapsedTime stateTimer = new ElapsedTime();

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        RESET_TRIGGER
    }
    private LaunchState launchState;

    // ------------------- GATE LOGIC -----------------------
    private double TRIGGER_CLOSE_POS = 0.48;
    private double TRIGGER_OPEN_POS = 0.4;
    private double TRIGGER_OPEN_TIME = 0.5;
    private double TRIGGER_CLOSE_TIME= 0.5;


    // ----------------- LAUNCHER CONSTANTS ------------------
    private int shotsRemaining = 0;
    private double launchVelocity = 0;
    private double LAUNCH_MIN_VEL = 1950;
    private double LAUNCH_TARGET_VEL = 2100;
    private double LAUNCH_MAX_SPINUP_TIME = 1.5;

    public void init(HardwareMap hwMap) {
        servoTrigger = hwMap.get(Servo.class, "trigga");
        motorLaunch = hwMap.get(DcMotorEx.class, "launch1");
        motorRamp1 = hwMap.get(DcMotorEx.class, "Intake1");
        motorRamp2 = hwMap.get(DcMotorEx.class,"Intake2");
        motorIntake = hwMap.get(DcMotorEx.class,"intake");

        motorLaunch.setDirection(DcMotorSimple.Direction.REVERSE);
        motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLaunch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLaunch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLaunch.setVelocityPIDFCoefficients(500,0,0,0);

        launchState = LaunchState.IDLE;

        servoTrigger.setPosition(0.48);
        motorLaunch.setPower(0);
        motorRamp1.setPower(0);
        motorRamp2.setPower(0);
        motorIntake.setPower(0);
    }

    public void update() {
        switch (launchState) {
            case IDLE:
                if (shotsRemaining > 0) {
                    servoTrigger.setPosition(TRIGGER_OPEN_POS);
                    motorLaunch.setVelocity(LAUNCH_TARGET_VEL);

                    stateTimer.reset();
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                if (launchVelocity > LAUNCH_MIN_VEL || stateTimer.seconds() > LAUNCH_MAX_SPINUP_TIME) {
                    servoTrigger.setPosition(TRIGGER_OPEN_POS);
                    stateTimer.reset();

                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                if (stateTimer.seconds() > TRIGGER_OPEN_TIME) {
                    shotsRemaining--; // incrament by 1
                    servoTrigger.setPosition(TRIGGER_CLOSE_POS);
                    stateTimer.reset();

                    launchState = LaunchState.RESET_TRIGGER;
                }
            case RESET_TRIGGER:
                if (stateTimer.seconds() > TRIGGER_CLOSE_TIME) {
                    if (shotsRemaining > 0) {
                        stateTimer.reset();
                        launchState = LaunchState.SPIN_UP;
                    } else {
                        motorLaunch.setVelocity(0);
                        launchState = LaunchState.IDLE;
                    }
                }
                break;
        }
    }
    public void fireShots(int numberOfShots) {
        if (launchState == LaunchState.IDLE) {
            shotsRemaining = numberOfShots;
        }
    }
    public boolean isBusy(){
        return launchState != LaunchState.IDLE;

    }

}
