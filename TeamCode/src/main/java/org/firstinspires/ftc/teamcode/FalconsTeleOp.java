package org.firstinspires.ftc.teamcode;

import static com.pedropathing.math.MathFunctions.normalizeAngle;


import android.graphics.Point;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp
@Configurable
public class FalconsTeleOp extends OpMode {
    //Initialize motors, servos, sensors, imus, etc.
    DcMotorEx motorLF, motorRF, motorLB, motorRB, motorIntake, motorLaunch1, motorLaunch2;
    Servo servoTrigger;
    GoBildaPinpointDriver pinpoint;

    private TelemetryManager telemetryManager;


    double currentX, currentY, distance;
    double currentHeading, targetHeading, headingError;
    boolean blue, launchRun = false;

    public static double closeVel =  1800, farVel = 2040;
    public static double SERVO_CLOSE = 0.27, SERVO_OPEN = 0.45;
    public static double expoX = 0.4, expoY = 0.4, expoAng = 0.5;

    PIDFCoefficients  launcherPIDF;
    com.pedropathing.control.PIDFCoefficients headingPIDF;
    public static double launch_p = 60, launch_d = 0, launch_f = 13.88, heading_p = 1.7, heading_d = 0.2, heading_f = 0.0;
    PIDFController headingPIDF_Controller = new PIDFController(new com.pedropathing.control.PIDFCoefficients(1.7,0,0.2,0));

    Point tarBlue, tarRed, tarCurrent;

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
        motorLaunch1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLaunch2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLaunch1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLaunch2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcherPIDF = new PIDFCoefficients(250,0,0,0);
        headingPIDF = new com.pedropathing.control.PIDFCoefficients(1.7,0,0.2,0);
        headingPIDF_Controller.setCoefficients(headingPIDF);

        motorLaunch1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherPIDF);
        motorLaunch2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherPIDF);


        // *************    PINPOINT STUFF    *************
        // Pinpoint setup
        String pinpointName = Constants.localizerConstants.hardwareMapName;
        GoBildaPinpointDriver.EncoderDirection forwardDirection = Constants.localizerConstants.forwardEncoderDirection;
        GoBildaPinpointDriver.EncoderDirection strafeDirection = Constants.localizerConstants.strafeEncoderDirection;
        GoBildaPinpointDriver.GoBildaOdometryPods resolution = Constants.localizerConstants.encoderResolution;

        double xOffset = Constants.localizerConstants.forwardPodY;
        double yOffset = Constants.localizerConstants.strafePodX;

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, pinpointName);
        pinpoint.setEncoderDirections(forwardDirection, strafeDirection);
        pinpoint.setOffsets(xOffset, yOffset, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(resolution);

        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,72,72,AngleUnit.DEGREES,0));

        tarBlue = new Point(10,140);
        tarRed = new Point(134,140);
        tarCurrent = new Point(10, 140);

        double headingError = 0.0;
        double targetHeading = 0.0;



        // *************    PANELS    *************
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    // This code runs repeatedly until the Stop button is pressed on the Driver Station
    // Replaces the old  while(OpModeIsActive())  loop
    @Override
    public void loop() {

        // *************    ODOMETRY    *************
        pinpoint.update();

        currentX = pinpoint.getPosX(DistanceUnit.INCH);
        currentY = pinpoint.getPosY(DistanceUnit.INCH);
        currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);


        // Calculate target heading
        double targetHeading = Math.atan2(
                tarCurrent.y - currentY,
                tarCurrent.x - currentX
        );

        if (gamepad1.dpadUpWasPressed()) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 72, 72, AngleUnit.DEGREES, 0));
        }


        // *************    MECANUM    *************
        double powerX = 0.0;  // Desired power for strafing           (-1 to 1)
        double powerY = 0.0;  // Desired power for forward/backward   (-1 to 1)
        double powerAng = 0.0;  // Desired power for turning          (-1 to 1)

        headingPIDF = new com.pedropathing.control.PIDFCoefficients(heading_p, 0, heading_d, heading_f);
        headingPIDF_Controller.setCoefficients(headingPIDF);

        if (gamepad1.left_stick_x > 0) {
            powerX = 0.12 + applyExpo(gamepad1.left_stick_x, expoAng);
        } else if (gamepad1.left_stick_x < 0) {
            powerX = -0.12 + applyExpo(gamepad1.left_stick_x, expoAng);
        }
        if (-gamepad1.left_stick_y > 0) {
            powerY = 0.05 + applyExpo(-gamepad1.left_stick_y, expoAng);
        } else if (-gamepad1.left_stick_y < 0) {
            powerY = -0.05 + applyExpo(-gamepad1.left_stick_y, expoAng);
        }

        // Turn on heading track if trigger
        if (gamepad1.left_trigger > 0.2 || gamepad2.left_trigger > 0.2) {
            headingError = determineRotationDirection(pinpoint.getHeading(AngleUnit.RADIANS), targetHeading);
            headingPIDF_Controller.updateError(headingError);
            powerAng = headingPIDF_Controller.run();
            powerAng = Math.max(-1.0, Math.min(1.0, powerAng));

        } else {
                    /*if (-gamepad1.right_stick_x > 0) {
                        powerAng = 0.09 + applyExpo(-gamepad1.right_stick_x, expoAng);
                    } else if (-gamepad1.right_stick_x < 0) {
                        powerAng = -0.09 + applyExpo(-gamepad1.right_stick_x, expoAng);
                    }*/
            powerAng = applyExpo(-gamepad1.right_stick_x, expoAng);
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
        } else if (gamepad1.left_bumper || gamepad2.left_bumper) {
            motorIntake.setPower(-1);
        } else {
            motorIntake.setPower(0);
        }


        // *************    TRIGGER LOGIC    *************
        if (gamepad2.right_trigger > 0.2 || gamepad1.right_trigger > 0.2) {
            servoTrigger.setPosition(SERVO_OPEN);
        } else {
            servoTrigger.setPosition(SERVO_CLOSE);
        }


        // *************    LAUNCHER LOGIC    *************
        double launchVel;

        launcherPIDF = new PIDFCoefficients(launch_p, 0, launch_d, launch_f);

        motorLaunch1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherPIDF);
        motorLaunch2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherPIDF);

        if (gamepad2.yWasPressed() || gamepad1.yWasPressed()) {
            launchRun = !launchRun;
        }


        if (launchRun) {
            launchVel = 0.08945 * Math.pow(distance, 2) - 10.85 * distance + 1873.23;
        } else if (gamepad2.a) {
            launchVel = closeVel;
        } else if (gamepad2.b) {
            launchVel = farVel;
        } else if (gamepad2.x){
            launchVel = -1000;
        } else {
            launchVel = 0;
        }

        motorLaunch1.setVelocity(launchVel);
        motorLaunch2.setVelocity(launchVel);



        // *************    TARGET LOGIC    *************
        if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed()) {
            blue = !blue;
        }

        if (blue) {
            tarCurrent = tarBlue;
        } else {
            tarCurrent = tarRed;
        }

        distance = calculateDistance(currentX, currentY, tarCurrent.x, tarCurrent.y);



        // *************    TELEMETRY    *************
        telemetry.addData("launchVel",  motorLaunch1.getVelocity());
        telemetry.addData("launchPow",  motorLaunch1.getPower());
        telemetry.addLine();
        telemetry.addData("X", currentX);
        telemetry.addData("Y", currentY);
        telemetry.addData("H", pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addLine();
        if (blue) {
            telemetry.addLine("BLUE BLUE BLUE = yes");
        } else {
            telemetry.addLine("RED RED RED = meow");
        }
        telemetry.addLine();
        telemetry.addData("targetHeading", Math.toDegrees(targetHeading));
        telemetry.addData("distance", distance);

        telemetry.update();



        telemetryManager.addData("X", pinpoint.getPosX(DistanceUnit.INCH));
        telemetryManager.addData("Y", pinpoint.getPosY(DistanceUnit.INCH));
        telemetryManager.addData("H", pinpoint.getHeading(AngleUnit.DEGREES));

        telemetryManager.addData("distance",distance);

        telemetryManager.addData("targetHeading", Math.toDegrees(targetHeading));
        telemetryManager.addData("headingError", Math.toDegrees(headingError));

        telemetryManager.addData("launchVel", motorLaunch1.getVelocity());
        telemetryManager.addData("targetVel", launchVel);
        telemetryManager.addData("velocityError", launchVel - motorLaunch1.getVelocity());

        telemetryManager.update();
    }



    // *************    Any additional methods go here   *************

    private double applyExpo(double input, double expo) {
        return input * (1 - expo) + Math.pow(input, 3) * expo;
    }
    public double determineRotationDirection(double current, double target) {
        double currentCircularHeading = normalizeAngle(current);
        double targetHeading = normalizeAngle(target);
        double clockwiseRadians;
        double counterclockwiseRadians;

        // Determine the larger of the two headings
        if (targetHeading > currentCircularHeading) {
            // Subtract the smaller (current) heading from the larger (target) heading
            //   to find the radians needed to turn to get to the target going counterclockwise
            counterclockwiseRadians = targetHeading - currentCircularHeading;
            // Find the alternative
            clockwiseRadians = 2*Math.PI - counterclockwiseRadians;
        } else {
            // Subtract the smaller (target) heading from the larger (current) heading
            //   to find the radians needed to turn to get to the target doing clockwise
            clockwiseRadians = currentCircularHeading - targetHeading;
            // Find the alternative
            counterclockwiseRadians = 2*Math.PI - clockwiseRadians;
        }
        // Determine the most efficient direction and return the proper multiplier
        if (clockwiseRadians < counterclockwiseRadians) {
            return -clockwiseRadians;
        } else {
            return counterclockwiseRadians;
        }
    }

    /** Performs a mod operation that can only return positive results */
    public double modPositive(double number, double divisor) {
        return ((number % divisor) + divisor) % divisor;
    }

    public static double calculateDistance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }

}
