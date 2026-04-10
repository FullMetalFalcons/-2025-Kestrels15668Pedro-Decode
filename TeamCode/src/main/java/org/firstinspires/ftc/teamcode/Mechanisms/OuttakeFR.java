package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class OuttakeFR {
    private Servo servoTrigger;
    private DcMotorEx motorLaunch1, motorLaunch2, motorIntake;
    PIDFCoefficients  launcherPIDF;
    private ElapsedTime stateTimer = new ElapsedTime();

    public double launchVel;


    private enum LaunchState {
        IDLE,
        LAUNCH
    }
    private LaunchState launchState = LaunchState.IDLE;



    // ----------------- LAUNCHER INIT ------------------
    private int shotsRemaining = 0;
    double SERVO_CLOSE = 0.27, SERVO_OPEN = 0.45;

    public void init(HardwareMap hwMap) {
        motorIntake = (DcMotorEx) hwMap.dcMotor.get("intake");
        motorLaunch1 = (DcMotorEx) hwMap.dcMotor.get("launch1");
        motorLaunch2 = (DcMotorEx) hwMap.dcMotor.get("launch2");

        servoTrigger = (Servo) hwMap.servo.get("trigga");

        motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLaunch1.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLaunch2.setDirection(DcMotorSimple.Direction.REVERSE);

        motorIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLaunch1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLaunch2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motorLaunch1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLaunch2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLaunch1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLaunch2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcherPIDF = new PIDFCoefficients(60,0,0,13.8);

        motorLaunch1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherPIDF);
        motorLaunch2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherPIDF);


        servoTrigger.setPosition(0.4);
        motorLaunch1.setPower(0);
        motorLaunch2.setPower(0);
        motorIntake.setPower(0);

    }

    public void update() {
        switch (launchState) {
            case IDLE:
                if (motorLaunch1.getVelocity() > launchVel - 100) {
                    if (shotsRemaining > 0) {
                        stateTimer.reset();
                        servoTrigger.setPosition(SERVO_OPEN);
                        launchState = LaunchState.LAUNCH;
                    }
                }
                break;
            case LAUNCH:
                if (shotsRemaining > 0) {
                    if (motorLaunch1.getVelocity() > launchVel - 100) {
                        if (stateTimer.seconds() < 0.145) {
                            motorIntake.setPower(1);
                        } else {
                            shotsRemaining -= 1;
                            stateTimer.reset();
                        }
                    }
                } else {
                    motorIntake.setPower(0);
                    servoTrigger.setPosition(SERVO_CLOSE);
                    motorLaunch1.setVelocity(0);
                    motorLaunch2.setVelocity(0);
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
            motorLaunch1.setVelocity(2060);
            motorLaunch2.setVelocity(2060);
            launchVel = 2060;
        } else {
            motorLaunch1.setVelocity(1480);
            motorLaunch2.setVelocity(1480);
            launchVel = 1480;
        }
    }

    public void setOuttakeVelocity(double launchvel) {
        motorLaunch1.setVelocity(launchvel);
        motorLaunch2.setVelocity(launchvel);
        launchVel = launchvel;
    }


    // Intake Logic
    public void setIntakePower(double power) {
        motorIntake.setPower(power);
    }

    public void setIntakePower(boolean intaking) {
        if (intaking) {
            motorIntake.setPower(1);
            servoTrigger.setPosition(SERVO_CLOSE);
        } else {
            motorIntake.setPower(0);
            servoTrigger.setPosition(SERVO_OPEN);
        }
    }



    public void setServoPosition(boolean open) {
        if (open) {
            servoTrigger.setPosition(SERVO_OPEN);
        } else {
            servoTrigger.setPosition(SERVO_CLOSE);
        }
    }

    public double getFlywheelVelocity(){
        return motorLaunch1.getVelocity();
    }
}


