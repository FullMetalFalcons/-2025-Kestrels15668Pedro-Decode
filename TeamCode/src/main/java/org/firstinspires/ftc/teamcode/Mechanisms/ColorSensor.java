package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensor {

    ColorRangeSensor colorSensor;

    // Tune this to match your sensor placement
    private static final double DETECT_DISTANCE_IN = 2.0;

    int artifactCount = 0;
    boolean ballWasDetected = false;

    public void init(ColorRangeSensor sensor) {
        this.colorSensor = sensor;
    }

    public void update() {
        boolean ballInRange = colorSensor.getDistance(DistanceUnit.INCH) < DETECT_DISTANCE_IN;

        if (ballInRange && !ballWasDetected) {
            artifactCount++;
            ballWasDetected = true;
        }

        if (!ballInRange) {
            ballWasDetected = false;
        }
    }

    public int getCount() {
        return artifactCount;
    }

    public void resetCount() {
        artifactCount = 0;
    }
}
