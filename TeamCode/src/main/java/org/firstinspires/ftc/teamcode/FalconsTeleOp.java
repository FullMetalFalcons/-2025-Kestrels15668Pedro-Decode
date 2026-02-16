package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.Mechanisms.Webcam;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class FalconsTeleOp extends OpMode {
    //Initialize motors, servos, sensors, imus, etc.
    DcMotorEx motorLF, motorRF, motorLB, motorRB, motorLaunch, motorRamp1, motorRamp2, motorIntake;;
    Servo servoTrigger, lightLauncher;
    Webcam webcam;

    double reverse;
    boolean lastRB, intakeRun, red, blue;

    // The following code will run as soon as "INIT" is pressed on the Driver Station
    @Override
    public void init() {
        webcam = new Webcam(hardwareMap);
        webcam.init(hardwareMap, telemetry);

        // Set up drive motors
        // The names for each motor are taken from the driveConstants in the Constants file
        motorLF = (DcMotorEx) hardwareMap.dcMotor.get( Constants.driveConstants.leftFrontMotorName );
        motorLB = (DcMotorEx) hardwareMap.dcMotor.get( Constants.driveConstants.leftRearMotorName );
        motorRF = (DcMotorEx) hardwareMap.dcMotor.get( Constants.driveConstants.rightFrontMotorName );
        motorRB = (DcMotorEx) hardwareMap.dcMotor.get( Constants.driveConstants.rightRearMotorName );

        motorLaunch = (DcMotorEx) hardwareMap.dcMotor.get("Launch1");
        motorRamp1 = (DcMotorEx) hardwareMap.dcMotor.get("Intake1");
        motorRamp2 = (DcMotorEx) hardwareMap.dcMotor.get("Intake2");
        motorIntake = (DcMotorEx) hardwareMap.dcMotor.get("intake");

        servoTrigger = (Servo) hardwareMap.servo.get("trigga"); // rly chud
        lightLauncher = (Servo) hardwareMap.servo.get("light");


        // Use the following line as a template for defining new servos
        //claw = (Servo) hardwareMap.servo.get("claw");

        // Reverse certain drive motors so that positive power to all motors makes the robot move forwards
        // TODO: Update "Constants" with the proper directions of your drive motors
        motorLF.setDirection( Constants.driveConstants.leftFrontMotorDirection );
        motorLB.setDirection( Constants.driveConstants.leftRearMotorDirection );
        motorRF.setDirection( Constants.driveConstants.rightFrontMotorDirection );
        motorRB.setDirection( Constants.driveConstants.rightRearMotorDirection );

        motorLaunch.setDirection(DcMotorSimple.Direction.REVERSE);
        motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        // This resets the encoder values when the code is initialized
        motorLF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorRB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // This makes the wheels tense up and stay in position when it is not moving, opposite is FLOAT
        motorLF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLaunch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // This lets you look at encoder values while the OpMode is active
        // If you have a STOP_AND_RESET_ENCODER, make sure to put this below it
        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLaunch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLaunch.setVelocityPIDFCoefficients(500, 0, 0, 0);
        if (gamepad1.dpad_left || gamepad2.dpad_left) {
            blue = true;
            red = false;
        }
        if (gamepad1.dpad_right || gamepad2.dpad_right) {
            blue = false;
            red = true;
        }
    }

    // This code runs repeatedly until the Stop button is pressed on the Driver Station
    // Replaces the old  while(OpModeIsActive())  loop
    @Override
    public void loop() {

        // Mecanum drive code
        double powerX = 0.0;  // Desired power for strafing           (-1 to 1)
        double powerY = 0.0;  // Desired power for forward/backward   (-1 to 1)
        double powerAng = 0.0;  // Desired power for turning          (-1 to 1)

        if (gamepad1.left_trigger > 0.25 || gamepad2.left_trigger > 0.25) {
            AprilTagDetection tag = null;
            for (AprilTagDetection d : webcam.getDetectedTags()) {
                if (d.id == 20) {
                    tag = d;
                    break;
                }
                if (d.id == 24) {
                    tag = d;
                    break;
                }
            }

            if (tag != null) {
                double x = tag.ftcPose.x;

                if (Math.abs(x-4) > 0.5) {
                    powerAng = (x-4) * 0.012;
                }
            } else {
                powerAng = -gamepad1.right_stick_x;
            }
        } else {
            powerAng = -gamepad1.right_stick_x;
        }

        // Set the desired powers based on joystick inputs (-1 to 1)
        powerX = gamepad1.left_stick_x * reverse;
        powerY = -gamepad1.left_stick_y * reverse;
        //powerAng = -gamepad1.right_stick_x;

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


        if (gamepad1.right_trigger > 0.25) {
            reverse = -1;
        } else {
            reverse = 1;
        }



        if (gamepad2.b) {
            motorLaunch.setVelocity(2020);
        } else if (gamepad2.a) {
            motorLaunch.setVelocity(1720);
        } else if (gamepad2.x) {
            motorLaunch.setVelocity(1490);
        } else {
            motorLaunch.setVelocity(0);
        }


        if (gamepad1.right_bumper && !lastRB) {
            intakeRun = !intakeRun;
        }

        lastRB = gamepad1.right_bumper;


        if (gamepad2.right_bumper && gamepad2.b) {
            motorRamp1.setPower(0.73);
            motorRamp2.setPower(-0.73);
            motorIntake.setPower(1);
        } else if (gamepad2.right_bumper) {
            motorRamp1.setPower(1);
            motorRamp2.setPower(-1);
            motorIntake.setPower(1);
        } else if (intakeRun) {
            motorRamp1.setPower(1);
            motorRamp2.setPower(-1);
            motorIntake.setPower(1);
        } else if (gamepad1.y) {
            motorRamp1.setPower(-1);
            motorRamp2.setPower(1);
            motorIntake.setPower(-1);
        } else if (gamepad1.left_bumper || gamepad2.left_bumper) {
            motorRamp1.setPower(-0.55);
            motorRamp2.setPower(0.55);
            motorIntake.setPower(1);
        } else {
            motorRamp1.setPower(0);
            motorRamp2.setPower(0);
            motorIntake.setPower(0);
        }

        if (gamepad2.right_trigger > 0.25) {
            servoTrigger.setPosition(0.4);
        } else {
            servoTrigger.setPosition(0.48);
        }

        if ((motorLaunch.getVelocity() > 1750 && gamepad2.b) || (motorLaunch.getVelocity() > 1550 && gamepad2.a) || (motorLaunch.getVelocity() > 1350 && gamepad2.x)) {
            lightLauncher.setPosition(0.6);
        } else if (gamepad2.b || gamepad2.a || gamepad2.x) {
            lightLauncher.setPosition(0.279);
        } else {
            lightLauncher.setPosition(0);
        }



        // This type of boolean is a new addition to the FTC SDK
        // It will be true ONLY when the specified button changes state from not being pressed to being pressed
        //   Useful for toggle systems and as a replacement for the old (button && !lastButton) approach

        //if (gamepad1.rightBumperWasPressed()) { /* CODE */ }



        // If you want to print information to the Driver Station, use telemetry
        // addData() lets you give a string which is automatically followed by a ":" when printed
        //     the variable that you list after the comma will be displayed next to the label
        // update() only needs to be run once and will "push" all of the added data
        webcam.update();
        if (red && !blue){
            AprilTagDetection id24 = webcam.getTagBySpecificId(24);
            webcam.displayDetectionTelemetry(id24);
        }
        if (!red && blue){
            AprilTagDetection id20 = webcam.getTagBySpecificId(20);
            webcam.displayDetectionTelemetry(id20);
        }


        telemetry.addData("rpm", motorLaunch.getVelocity());
        telemetry.addData("servoPos", servoTrigger.getPosition());
        telemetry.update();

    }

    // Any additional methods go here

}