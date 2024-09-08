package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import org.firstinspires.ftc.teamcode.AprilTagLocation;

public final class Constants {
    public static final double TICKS_PER_REVOLUTION = 2000;
    public static final double DIAMETER = 1.889764;
    public static final double CIRCUMFERENCE = PI * DIAMETER;
    public static final AprilTagLocation[] APRIL_TAG_LOCATIONS = new AprilTagLocation[] {
            new AprilTagLocation(0, "April 1", 2.5F, new float[] {0, 0, 0}, new float[] {0, 1, 0}),
            new AprilTagLocation(1, "April 2", 2.5F, new float[] {24, 0, 0}, new float[] {0, 1, 0})
    };
}
