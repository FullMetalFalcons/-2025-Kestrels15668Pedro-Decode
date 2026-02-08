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

    /*private ElapsedTime stateTimer = new ElapsedTime();

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH
    }
    private LaunchState launchState; */


    // ----------------- LAUNCHER CONSTANTS ------------------
    /*private int shotsRemaining = 0;
    private double launchVelocity = 0;
    public double LAUNCH_TARGET_VEL = 2100;
    private double LAUNCH_MIN_VEL = LAUNCH_TARGET_VEL - 100;
    private double LAUNCH_MAX_SPINUP_TIME = 1.5;*/

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

        //launchState = LaunchState.IDLE;

        servoTrigger.setPosition(0.4);
        motorLaunch.setPower(0);
        motorRamp1.setPower(0);
        motorRamp2.setPower(0);
        motorIntake.setPower(0);
    }

    /*public void update() {
        switch (launchState) {
            case IDLE:
                if (shotsRemaining > 0) {
                    motorLaunch.setVelocity(LAUNCH_TARGET_VEL);

                    stateTimer.reset();
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                if (launchVelocity > LAUNCH_MIN_VEL || stateTimer.seconds() > LAUNCH_MAX_SPINUP_TIME) {
                    stateTimer.reset();

                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                if (shotsRemaining > 0) {
                    shotsRemaining -= 1;
                    motorRamp1.setPower(0.6);
                    motorRamp2.setPower(-0.6);
                    motorIntake.setPower(1);
                    stateTimer.reset();
                } else {
                    motorRamp1.setPower(0);
                    motorRamp2.setPower(0);
                    motorIntake.setPower(0);
                    motorLaunch.setPower(0);
                }
                break;
        }
    }
    public void fireShots(int numberOfShots) {
        if (stateTimer.seconds() >= 0.5) {
            shotsRemaining = numberOfShots;
        }
    }*/
    public boolean isBusy(){
        return launchState != LaunchState.IDLE;
    }


    // Outtake Logic
    public void setOuttakeVelocity(double velocity) {
        motorLaunch.setVelocity(velocity);
    }


    // Intake Logic
    public void setIntakePower(double power) {
        motorIntake.setPower(1);
        motorRamp1.setPower(power);
        motorRamp2.setPower(-power);
    }

    public void setServoPosition(double position) {
        servoTrigger.setPosition(position);
    }

}


