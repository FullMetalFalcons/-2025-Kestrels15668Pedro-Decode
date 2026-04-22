package org.firstinspires.ftc.teamcode;

import static com.pedropathing.math.MathFunctions.normalizeAngle;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.UtilityOctoQuadConfigMenu;
import org.firstinspires.ftc.teamcode.Mechanisms.ColorSensor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp
@Configurable
public class FalconsTeleOp extends OpMode {
    //Initialize motors, servos, sensors, imus, etc.
    DcMotorEx motorLF, motorRF, motorLB, motorRB, motorIntake, motorLaunch1, motorLaunch2;
    Servo servoGayte, lightIndicator;
    GoBildaPinpointDriver pinpoint;
    ColorRangeSensor colorSensor;
    ColorSensor counter = new ColorSensor();
    TelemetryManager telemetryManager;
    public static double heading_p = 1.6, heading_d = 0.2, heading_f = 0;
    PIDFCoefficients  launcherPIDF;
    com.pedropathing.control.PIDFCoefficients headingPIDF = new com.pedropathing.control.PIDFCoefficients(heading_p, 0, heading_d, heading_f);
    PIDFController headingPIDF_Controller = new PIDFController(headingPIDF);

    // Set toggles
    boolean blue = false, launchRun = false, autoLaunchToggle = true, correctedTargetToggle = false;

    // Configurables
    public static double launcherVel =  1800;
    public static double SERVO_CLOSE = 0.27, SERVO_OPEN = 0.45;
    public static double expoX = 0.3, expoY = 0.3, expoAng = 0.3;
    public static double launch_p = 60, launch_d = 0, launch_f = 13.88;
    public static double timeOfFlight;

    double headingError;

    double targetBlueX = 10, targetBlueY = 140;
    double targetRedX = 140, targetRedY = 140;
    double targetCurrentX = 140, targetCurrentY = 140;

    public static Pose2D startingPose = new Pose2D(DistanceUnit.INCH, 72,72,AngleUnit.DEGREES, 0);


    // The following code will run as soon as "INIT" is pressed on the Driver Station
    @Override
    public void init() {
        // Init motors/servos
        initDriveMotors(DcMotor.ZeroPowerBehavior.BRAKE);
        initLaunchMotors(DcMotor.ZeroPowerBehavior.FLOAT);
        initIntakeMotor();

        servoGayte = (Servo) hardwareMap.servo.get("gayte");
        lightIndicator = (Servo) hardwareMap.servo.get("light");

        // Init any other systems
        initPinpoint();
        colorSensor = hardwareMap.get(ColorRangeSensor.class, "sensorColorRange");
        counter.init(colorSensor);

        headingPIDF = new com.pedropathing.control.PIDFCoefficients(heading_p,0,heading_d,heading_f);
        headingPIDF_Controller.setCoefficients(headingPIDF);

        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
    }


    @Override
    public void loop() {

        // *************    ODOMETRY    *************
        if (gamepad1.dpadUpWasPressed()) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 72, 72, AngleUnit.DEGREES, 0));
        }

        pinpoint.update();

        double currentX = pinpoint.getPosX(DistanceUnit.INCH);
        double currentY = pinpoint.getPosY(DistanceUnit.INCH);
        double currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);
        double velX = pinpoint.getVelX(DistanceUnit.INCH);
        double velY = pinpoint.getVelY(DistanceUnit.INCH);


        // *************    COLOR SENSOR    *************
        double artifacts = counter.getCount();
        counter.update();


        // *************    TARGET LOGIC    *************
        if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed()) {
            blue = !blue;
        }
        if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed()) {
            correctedTargetToggle = !correctedTargetToggle;
        }

        if (blue) {
            targetCurrentX = targetBlueX;
            targetCurrentY = targetBlueY;
        } else {
            targetCurrentX = targetRedX;
            targetCurrentY = targetRedY;
        }

        double distance = calculateDistance(currentX, currentY, targetCurrentX, targetCurrentY);

        double[] targetCurrentAdjusted = getAdjustedTarget(
                targetCurrentX, targetCurrentY,
                velX, velY,
                timeOfFlight = calculateTimeOfFlight(1.7142857143*launcherVel, distance)
        );
        if (correctedTargetToggle) {
            targetCurrentX = targetCurrentAdjusted[0];
            targetCurrentY = targetCurrentAdjusted[1];
        }

        double targetHeading = Math.atan2(
                targetCurrentY - currentY,
                targetCurrentX - currentX
        );

        distance = calculateDistance(currentX, currentY, targetCurrentX, targetCurrentY);


        // *************    MECANUM    *************
        double powerX = 0.0;  // Desired power for strafing           (-1 to 1)
        double powerY = 0.0;  // Desired power for forward/backward   (-1 to 1)
        double powerAng = 0.0;  // Desired power for turning          (-1 to 1)

        boolean targetTrack;
        double powerAngle;

        headingPIDF = new com.pedropathing.control.PIDFCoefficients(heading_p, 0, heading_d, heading_f);
        headingPIDF_Controller.setCoefficients(headingPIDF);

        headingError = determineRotationDirection(pinpoint.getHeading(AngleUnit.RADIANS), targetHeading);
        headingPIDF_Controller.updateError(headingError);

        if (gamepad1.left_trigger > 0.2 || gamepad2.left_trigger > 0.2) {
            powerAngle = headingPIDF_Controller.run();
            powerAngle = Math.max(-1.0, Math.min(1.0, powerAngle));
            targetTrack = true;
        } else {
            powerAngle = applyExpo(-gamepad1.right_stick_x, expoAng);
            targetTrack = false;
        }

        powerX = applyDeadZone(applyExpo(gamepad1.left_stick_x,expoX), 0.11);
        powerY = applyDeadZone(applyExpo(-gamepad1.left_stick_y,expoY), 0.045);
        powerAng = powerAngle;

        mecanumDriveCode(powerY, powerX, powerAng, 1.0);


        // *************    LAUNCH ZONE    **************
        // Find 4 corners pos
        double cosH = Math.cos(currentHeading);
        double sinH = Math.sin(currentHeading);
        double axX = cosH * 9;
        double axY = sinH * 9;
        double ayX = -sinH * 8;
        double ayY = cosH * 8;

        double[][] corners = {
                {currentX + axX + ayX, currentY + axY + ayY},  // front left
                {currentX + axX - ayX, currentY + axY - ayY},  // front right
                {currentX - axX + ayX, currentY - axY + ayY},  // back left
                {currentX - axX - ayX, currentY - axY - ayY},  // back right
        };


        boolean inFar = false, inClose = false;

        // Check to see if corners are within zones
        for (double[] p : corners) {
            double px = p[0], py = p[1];

            double nd1 = (144 - 0) * (py - 144) - (144 - 144) * (px - 144);
            double nd2 = (72 - 144) * (py - 144) - (72 - 144) * (px - 144);
            double nd3 = (0 - 72) * (py - 72) - (144 - 72) * (px - 72);
            if ((nd1 >= 0 && nd2 >= 0 && nd3 >= 0) || (nd1 <= 0 && nd2 <= 0 && nd3 <= 0)) {
                inClose = true;
            }

            double fd1 = (96 - 48) * (py - 0) - (0 - 0) * (px - 48);
            double fd2 = (72 - 96) * (py - 0) - (24 - 0) * (px - 96);
            double fd3 = (48 - 72) * (py - 24) - (0 - 24) * (px - 72);
            if ((fd1 >= 0 && fd2 >= 0 && fd3 >= 0) || (fd1 <= 0 && fd2 <= 0 && fd3 <= 0)) {
                inFar = true;
            }
        }


        // *************    INTAKE LOGIC    *************
        if (gamepad1.right_bumper) {
            motorIntake.setPower(1);
            servoGayte.setPosition(SERVO_CLOSE);
        } else if (gamepad2.right_bumper) {
            motorIntake.setPower(1);
            servoGayte.setPosition(SERVO_OPEN);
            counter.resetCount();
        } else if (gamepad1.left_bumper || gamepad2.left_bumper) {
            motorIntake.setPower(-1);
        } else {
            motorIntake.setPower(0);
            servoGayte.setPosition(SERVO_OPEN);
        }

        if (gamepad2.right_trigger > 0.2 || gamepad1.right_trigger > 0.2) {
            servoGayte.setPosition(SERVO_OPEN);
        }


        // *************    LAUNCHER LOGIC    *************
        double launchVel;
        double targetVel = runLaunchFormula(distance);

        launcherPIDF = new PIDFCoefficients(launch_p, 0, launch_d, launch_f);

        motorLaunch1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherPIDF);
        motorLaunch2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherPIDF);

        if (gamepad2.yWasPressed() || gamepad1.yWasPressed()) {
            launchRun = !launchRun;
        }

        if (launchRun) {
            launchVel = targetVel;
        } else if (gamepad2.a) {
            launchVel = launcherVel;
        } else if (gamepad2.x) {
            launchVel = -1000;
        } else {
            launchVel = 0;
        }

        motorLaunch1.setVelocity(launchVel * 0.95);
        motorLaunch2.setVelocity(launchVel * 0.95);

        if (inFar) {
            motorLaunch1.setVelocity(launchVel * 0.97);
            motorLaunch2.setVelocity(launchVel * 0.97);
        }


        // *************    AUTO LAUNCH    *************
        boolean goingSlow = Math.hypot(velX, velY) < 2;

        if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed()) {
            autoLaunchToggle = !autoLaunchToggle;
        }

        if (autoLaunchToggle && (inClose || inFar) && targetTrack && Math.abs(targetHeading) < 10 && (motorLaunch1.getVelocity() > targetVel - 500) && (goingSlow || correctedTargetToggle)) {
            motorIntake.setPower(1);
            servoGayte.setPosition(SERVO_OPEN);
            counter.resetCount();
        }



        // *************    INDICATOR LIGHT    *************
        double RED = 0.277, YELLOW = 0.388, GREEN = 0.500;

        lightIndicator.setPosition(0);
        if (artifacts == 1) {
            lightIndicator.setPosition(RED);
        } else if (artifacts == 2) {
            lightIndicator.setPosition(YELLOW);
        } else if (artifacts >= 3) {
            lightIndicator.setPosition(GREEN);
        }



        // *************    TELEMETRY    *************
        telemetry.addData("launchVel",  motorLaunch1.getVelocity());
        telemetry.addData("timeOfFlight", timeOfFlight);
        telemetry.addData("headingPIDF", headingPIDF);
        telemetry.addData("headingPIDFController", headingPIDF_Controller.getCoefficients());
        telemetry.addLine();
        telemetry.addData("X", currentX);
        telemetry.addData("Y", currentY);
        telemetry.addData("H", pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addLine();
        telemetry.addData("move-n-shoot", correctedTargetToggle);
        telemetry.addLine();
        if (autoLaunchToggle) {
            telemetry.addLine("AUTO LAUNCH");
        } else {
            telemetry.addLine("NO LAUNCH FOR YOU");
        }
        telemetry.addLine();
        if (blue) {
            telemetry.addLine("BLUE BLUE BLUE = yes");
        } else {
            telemetry.addLine("RED RED RED = meow");
        }
        if (inFar) {
            telemetry.addLine("IN FAR ZONE");
        } else if (inClose) {
            telemetry.addLine("IN CLOSE ZONE");
        } else {
            telemetry.addLine("OUTSIDE ZONES");
        }
        if (goingSlow) {
            telemetry.addLine("SLOW SLOW SLOW");
        } else {
            telemetry.addLine("SUPER FAST VROOM VROOM");
        }
        telemetry.addLine();
        telemetry.addData("targetHeading", Math.toDegrees(targetHeading));
        telemetry.addData("distance", distance);
        telemetry.addData("balls Counted", counter.getCount());
        telemetry.addLine();
        telemetry.addData("targetX", targetCurrentX);
        telemetry.addData("targetY", targetCurrentY);

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
    // Methods to easily set the attributes of all drive motors at once
    public void initDriveMotors(DcMotor.ZeroPowerBehavior behavior){
        motorLF = (DcMotorEx) hardwareMap.dcMotor.get(Constants.driveConstants.leftFrontMotorName);
        motorRF = (DcMotorEx) hardwareMap.dcMotor.get(Constants.driveConstants.rightFrontMotorName);
        motorLB = (DcMotorEx) hardwareMap.dcMotor.get(Constants.driveConstants.leftRearMotorName);
        motorRB = (DcMotorEx) hardwareMap.dcMotor.get(Constants.driveConstants.rightRearMotorName);

        motorLF.setDirection(Constants.driveConstants.leftFrontMotorDirection);
        motorLB.setDirection(Constants.driveConstants.leftRearMotorDirection);
        motorRF.setDirection(Constants.driveConstants.rightFrontMotorDirection);
        motorRB.setDirection(Constants.driveConstants.rightRearMotorDirection);

        motorLF.setZeroPowerBehavior(behavior);
        motorLB.setZeroPowerBehavior(behavior);
        motorRF.setZeroPowerBehavior(behavior);
        motorRB.setZeroPowerBehavior(behavior);
    }

    public void initLaunchMotors(DcMotor.ZeroPowerBehavior behavior){
        motorLaunch1 = (DcMotorEx) hardwareMap.dcMotor.get("launch1");
        motorLaunch2 = (DcMotorEx) hardwareMap.dcMotor.get("launch2");

        motorLaunch1.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLaunch2.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLaunch1.setZeroPowerBehavior(behavior);
        motorLaunch2.setZeroPowerBehavior(behavior);

        motorLaunch1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLaunch2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLaunch1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLaunch2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcherPIDF = new PIDFCoefficients(launch_p,0,launch_d,launch_f);
        motorLaunch1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherPIDF);
        motorLaunch2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherPIDF);
    }

    public void initIntakeMotor() {
        motorIntake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        motorIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void initPinpoint() {
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

        pinpoint.setPosition(startingPose);
    }

    public void mecanumDriveCode(double forward, double strafe, double angular, double speedPercent) {
        // Perform vector math to determine the desired powers for each wheel
        double powerLF = strafe + forward - angular;
        double powerLB = -strafe + forward - angular;
        double powerRF = -strafe + forward + angular;
        double powerRB = strafe + forward + angular;

        // Determine the greatest wheel power and set it to max
        double max = Math.max(1.0, Math.abs(powerLF));
        max = Math.max(max, Math.abs(powerRF));
        max = Math.max(max, Math.abs(powerLB));
        max = Math.max(max, Math.abs(powerRB));

        // Scale all power variables down to a number between 0 and 1 (so that setPower will accept them)
        motorLF.setPower(powerLF /max * speedPercent);
        motorLB.setPower(powerLB /max * speedPercent);
        motorRF.setPower(powerRF /max * speedPercent);
        motorRB.setPower(powerRB /max * speedPercent);
    }

    public static double calculateTimeOfFlight(double inputRPM, double distanceInches) {
        // Ball and gear constants
        double diameterIn = 96.0 / 25.4;
        double circumferenceIn = Math.PI * diameterIn;
        double launchSpeed = (inputRPM * (8.0 / 10.0) * circumferenceIn) / 60.0;

        // Velocity components at 38 degrees
        double vx = launchSpeed * Math.cos(Math.toRadians(38.0));
        double vy = launchSpeed * Math.sin(Math.toRadians(38.0));

        // Time to reach target horizontally: t = distance / vx
        double t = distanceInches / vx;

        // Verify ball is still airborne (above 36in goal height) at that time
        // y(t) = vy*t - 0.5*g*t²
        double height = vy * t - 0.5 * 386.09 * t * t;
        if (height < 36.0) return -1; // ball lands short of goal height

        return t;
    }


    private double applyExpo(double input, double expo) {
        return input * (1 - expo) + Math.pow(input, 3) * expo;
    }

    public double applyDeadZone(double input, double deadZone) {
        if (-input > 0) {
            return deadZone + input;
        } else if (-input < 0) {
            return -deadZone + input;
        } else {
            return 0;
        }
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

    public static double runLaunchFormula(double distance) {
        return 0.08945 * Math.pow(distance, 2) - 10.85 * distance + 1873.23;
    }

    public static double[] getAdjustedTarget( 
        double targetX, double targetY,
        double velX,    double velY,
        double timeOfFlight) 
    {

    double adjustedX = targetX - velX * timeOfFlight;
    double adjustedY = targetY - velY * timeOfFlight;

    return new double[] { adjustedX, adjustedY };

    }
}
