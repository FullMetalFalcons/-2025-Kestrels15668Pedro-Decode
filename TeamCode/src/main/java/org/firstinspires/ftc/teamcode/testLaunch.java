package org.firstinspires.ftc.teamcode;

import static com.pedropathing.math.MathFunctions.normalizeAngle;

import android.graphics.Point;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
@Configurable
public class testLaunch extends OpMode {
    DcMotorEx motorLaunch1, motorLaunch2;
    double launchVel;

    PIDFCoefficients  launcherPIDF;

    public static double launch_p = 60, launch_f = 13.2;

    public static double closeVel =  1800, farVel = 2140;


    @Override
    public void init() {
        motorLaunch1 =(DcMotorEx)hardwareMap.dcMotor.get("launch1");
        motorLaunch2 =(DcMotorEx)hardwareMap.dcMotor.get("launch2");

        motorLaunch1.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLaunch2.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLaunch1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLaunch2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motorLaunch1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLaunch2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLaunch1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLaunch2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLaunch1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLaunch2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcherPIDF = new PIDFCoefficients(250,0,0,0);

        motorLaunch1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherPIDF);
        motorLaunch2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherPIDF);
    }

    @Override
    public void loop() {
        // *************    LAUNCHER LOGIC    *************

        launcherPIDF = new PIDFCoefficients(launch_p, 0, 0, launch_f);

        motorLaunch1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherPIDF);
        motorLaunch2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherPIDF);


         if (gamepad2.dpad_left) {
            motorLaunch1.setPower(1);
            motorLaunch2.setPower(1);
        } else if (gamepad2.a) {
            launchVel = closeVel;
        } else if (gamepad2.b) {
            launchVel = farVel;
        } else if (gamepad2.x){
            launchVel = -1000;
        } else {
            launchVel = 0;
        }

        //motorLaunch1.setVelocity(launchVel);
        //motorLaunch2.setVelocity(launchVel);


        // *************    TELEMETRY    *************
        telemetry.addData("launchVel1",  motorLaunch1.getVelocity());
        telemetry.addData("launchVel2", motorLaunch2.getVelocity());
        telemetry.addData("launchPow1",  motorLaunch1.getPower());
        telemetry.addData("launchPow2", motorLaunch2.getPower());
        telemetry.addData("closeVel", closeVel);
        telemetry.addData("farVel", farVel);
    }
}
