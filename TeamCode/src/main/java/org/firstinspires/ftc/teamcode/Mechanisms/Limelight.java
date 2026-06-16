package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.FalconsTeleOp;

public class Limelight {
    private static Limelight3A limelight;

    public static void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
    }

    public static void start() {
        limelight.start();
    }

    public static Pose3D llPose = new Pose3D(
            new Position(DistanceUnit.INCH, 0, 0, 0, 0),
            new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));
    public static double llTx = 0, llTy = 0, llTa = 0;

    public static Pose3D update(double headingDegrees) {
        //limelight.updateRobotOrientation(headingDegrees);

        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            llPose = llResult.getBotpose();
            llTx = llResult.getTx();
            llTy = llResult.getTy();
            llTa = llResult.getTa();
        }
        return llPose;
    }
}