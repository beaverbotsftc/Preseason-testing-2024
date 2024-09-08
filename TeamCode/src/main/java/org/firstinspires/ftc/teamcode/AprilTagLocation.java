package org.firstinspires.ftc.teamcode;

public class AprilTagLocation {
    public int id = 0;
    public String name = null;
    public float size = 0;
    public float[] location = null;
    public float[] direction = null;
    public AprilTagLocation(int id, String name, float size, float[] location, float[] direction) {
        this.id = id;
        this.name = name;
        this.size = size;
        this.location = location;
        this.direction = direction;
    }
}
