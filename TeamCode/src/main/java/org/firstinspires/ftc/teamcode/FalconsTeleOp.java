package org.firstinspires.ftc.teamcode;

import static com.pedropathing.math.MathFunctions.normalizeAngle;

import android.graphics.Point;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class FalconsTeleOp extends OpMode {
    //Initialize motors, servos, sensors, imus, etc.
    DcMotorEx motorLF, motorRF, motorLB, motorRB, motorIntake, motorLaunch1, motorLaunch2;
    Servo servoTrigger;
    GoBildaPinpointDriver pinpoint;
    
    Follower follower;

    Pose currentPose;
    Boolean close, far;

    // The following code will run as soon as "INIT" is pressed on the Driver Station
    @Override
    public void init() {
        // Set up drive motors
        // The names for each motor are taken from the driveConstants in the Constants file
        motorLF = (DcMotorEx) hardwareMap.dcMotor.get("lf");
        motorRF = (DcMotorEx) hardwareMap.dcMotor.get("rf");
        motorLB = (DcMotorEx) hardwareMap.dcMotor.get("lb");
        motorRB = (DcMotorEx) hardwareMap.dcMotor.get("rb");
        motorIntake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        motorLaunch1 = (DcMotorEx) hardwareMap.dcMotor.get("launch1");
        motorLaunch2 = (DcMotorEx) hardwareMap.dcMotor.get("launch2");

        servoTrigger = (Servo) hardwareMap.servo.get("trigga");

        // Reverse certain drive motors so that positive power to all motors makes the robot move forwards
        // TODO: Update "Constants" with the proper directions of your drive motors
        motorLF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLB.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRF.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRB.setDirection(DcMotorSimple.Direction.FORWARD);
        motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLaunch1.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLaunch2.setDirection(DcMotorSimple.Direction.REVERSE);

        // This makes the wheels tense up and stay in position when it is not moving, opposite is FLOAT
        motorLF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLaunch1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLaunch2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // This lets you look at encoder values while the OpMode is active
        // If you have a STOP_AND_RESET_ENCODER, make sure to put this below it
        motorLaunch1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLaunch2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLaunch1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLaunch2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLaunch1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(250,0,0,0));
        motorLaunch2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(250,0,0,0));


        // *************    FOLLOWER STUFF    *************
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(72,72,0));

        tarBlue = new Point(10,140);
        tarRed = new Point(134,140);

        double headingError = 0;
        double targetHeading = 0;
    }

    // This code runs repeatedly until the Stop button is pressed on the Driver Station
    // Replaces the old  while(OpModeIsActive())  loop
    @Override
    public void loop() {

        readFromPinpoint();
        currentPose = follower.getPose();

        // Mecanum drive code
        double powerX = 0.0;  // Desired power for strafing           (-1 to 1)
        double powerY = 0.0;  // Desired power for forward/backward   (-1 to 1)
        double powerAng = 0.0;  // Desired power for turning          (-1 to 1)

        // Set the desired powers based on joystick inputs (-1 to 1)
        powerX = applyExpo(gamepad1.left_stick_x, 0.5);
        powerY = applyExpo(-gamepad1.left_stick_y, 0.5);

        // Calculate target heading
        targetHeading = Math.atan2(
                tarBlue.y - currentPose.getY(),
                tarBlue.x - currentPose.getX()
        );
        
        // Turn on heading track if trigger
        if (gamepad1.left_trigger > 0.2 || gamepad2.left_trigger > 0.2) {
            headingError = normalizeAngle(targetHeading - currentPose.getHeading());
            powerAng = headingError * 2.0; // kP TODO create PID tuner
            powerAng = Math.max(-1.0, Math.min(1.0,powerAng));
        } else {
            powerAng = applyExpo(-gamepad1.right_stick_x, 0.6);
        }

        // Perform vector math to determine the desired powers for each wheel
        double powerLF = powerX + powerY - powerAng;
        double powerLB = -powerX + powerY - powerAng;
        double powerRF = -powerX + powerY + powerAng;
        double powerRB = powerX + powerY + powerAng;

        // Determine the greatest wheel power and set it to max
        double max = Math.max(1.0, Math.abs(powerLF));
        max = Math.max(max, Math.abs(powerRF));
        max = Math.max(max, Math.abs(powerLB));
        max = Math.max(max, Math.abs(powerRB));

        // Scale all power variables down to a number between 0 and 1 (so that setPower will accept them)
        powerLF /= max;
        powerLB /= max;
        powerRF /= max;
        powerRB /= max;

        motorLF.setPower(powerLF);
        motorLB.setPower(powerLB);
        motorRF.setPower(powerRF);
        motorRB.setPower(powerRB);



        // *************    INTAKE LOGIC    *************
        if (gamepad1.right_bumper || gamepad2.right_bumper) {
            motorIntake.setPower(1);
        } else if (gamepad1.y) {
            motorIntake.setPower(-1);
        } else {
            motorIntake.setPower(0);
        }



        // *************    TRIGGER LOGIC    *************
        if (gamepad2.right_trigger > 0.2) {
            servoTrigger.setPosition(0.5);
        } else {
            servoTrigger.setPosition(0.4);
        }



        // *************    LAUNCHER LOGIC    *************
        int closeVel = 1800;
        int farVel = 2100;
        int launchVel;

        if (gamepad2.a) {
            launchVel = closeVel;
        } else if (gamepad2.b) {
            launchVel = farVel;
        } else {
            launchVel = 0;
        }

        motorLaunch1.setVelocity(launchVel);
        motorLaunch2.setVelocity(launchVel);

        if (gamepad2.dpad_up) {
            closeVel += 20;
        }
        if (gamepad2.dpad_down) {
            closeVel -=20;
        }
        if (gamepad2.dpad_right) {
            farVel += 20;
        }
        if (gamepad2.dpad_left) {
            farVel -=20;
        }



        // *************    TELEMETRY    *************
        telemetry.addData("launchVel1", motorLaunch1.getVelocity());
        telemetry.addData("launchVel2", motorLaunch2.getVelocity());
        telemetry.addData("closeVel", closeVel);
        telemetry.addData("farVel", farVel);

        telemetry.addData("currentPos", follower.getPose());
        telemetry.addData("targetHeading", targetHeading);
        telemetry.addData("errorHeading", headingError);
    }


    private double applyExpo(double input, double expo) {
        return input * (1 - expo) + Math.pow(input, 3) * expo;
    }
    // Any additional methods go here

}
