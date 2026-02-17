package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class OuttakeFR {
    private Servo servoTrigger;
    private DcMotorEx motorLaunch, motorRamp1, motorRamp2, motorIntake;

    private ElapsedTime stateTimer = new ElapsedTime();

    public boolean isFar;

    private enum LaunchState {
        IDLE,
        LAUNCH
    }
    private LaunchState launchState = LaunchState.IDLE;


    // ----------------- LAUNCHER CONSTANTS ------------------
    private int shotsRemaining = 0;

    public void init(HardwareMap hwMap) {
        servoTrigger = hwMap.get(Servo.class, "trigga");
        motorLaunch = hwMap.get(DcMotorEx.class, "Launch1");
        motorRamp1 = hwMap.get(DcMotorEx.class, "Intake1");
        motorRamp2 = hwMap.get(DcMotorEx.class,"Intake2");
        motorIntake = hwMap.get(DcMotorEx.class,"intake");

        motorLaunch.setDirection(DcMotorSimple.Direction.REVERSE);
        motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLaunch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLaunch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLaunch.setVelocityPIDFCoefficients(500,0,0,0);


        servoTrigger.setPosition(0.4);
        motorLaunch.setPower(0);
        motorRamp1.setPower(0);
        motorRamp2.setPower(0);
        motorIntake.setPower(0);
    }

    public void update() {
        switch (launchState) {
            case IDLE:
                if ((motorLaunch.getVelocity() > 1540 && !isFar) || (motorLaunch.getVelocity() > 1960 && isFar)) {
                    if (shotsRemaining > 0) {
                        stateTimer.reset();
                        launchState = LaunchState.LAUNCH;
                    }
                }
                break;
            case LAUNCH:
                if (shotsRemaining > 0) {
                    if (!isFar) {
                        if (stateTimer.seconds() < 0.26) {
                            motorRamp1.setPower(1);
                            motorRamp2.setPower(-1);
                            motorIntake.setPower(1);
                        } else {
                            shotsRemaining -= 1;
                            stateTimer.reset();
                        }
                    } else {
                        if (stateTimer.seconds() < 0.36) {
                            motorRamp1.setPower(1);
                            motorRamp2.setPower(-0.72);
                            motorIntake.setPower(0.72);
                        } else {
                            shotsRemaining -= 1;
                            stateTimer.reset();
                        }
                    }
                } else {
                    motorRamp1.setPower(0);
                    motorRamp2.setPower(0);
                    motorIntake.setPower(0);
                    //motorLaunch.setPower(0);
                    stateTimer.reset();
                    launchState = LaunchState.IDLE;
                }
        }
    }
    public void fireShots(int numBalls) {
        if (!isBusy()) {  shotsRemaining = numBalls;  }
    }
    public boolean isBusy() {
        return (launchState != LaunchState.IDLE) || (shotsRemaining > 0);
    }


    // Outtake Logic
    public void setOuttakeVelocity(boolean launchFar) {
        if (launchFar) {
            motorLaunch.setVelocity(2040);
        } else {
            motorLaunch.setVelocity(1580);
        }
    }
    public void setOuttakeVelocity(double velocity) {
        motorLaunch.setVelocity(velocity);
    }


    // Intake Logic
    public void setIntakePower(double power) {
        motorIntake.setPower(Math.abs(power*1000000));
        motorRamp1.setPower(power);
        motorRamp2.setPower(-power);
    }


    public void setServoPosition(double position) {
        servoTrigger.setPosition(position);
    }

    public double getFlywheelVelocity(){
        return motorLaunch.getVelocity();
    }
}


