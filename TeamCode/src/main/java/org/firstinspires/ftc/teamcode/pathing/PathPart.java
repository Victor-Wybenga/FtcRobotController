package org.firstinspires.ftc.teamcode.pathing;

import org.firstinspires.ftc.teamcode.mechanics.DcMotors;

public class PathPart {
    public double throttle;
    public double rotation;
    public double seconds;

    private PathPart(double throttle, double rotation, double seconds) {
        this.throttle = throttle;
        this.rotation = rotation;
        this.seconds = seconds;
    }

    public static PathPart Rotate(PathRotation rotation, double degrees) {
        return new PathPart(
            0.0,
            (rotation == PathRotation.RIGHT) ? 1.0 : -1.0,
            DcMotors.ROBOT_SECONDS_PER_DEGREE * degrees
        );
    }

    public static PathPart Drive(PathDirection direction, double feet) {
        return new PathPart(
            (direction == PathDirection.FORWARD) ? 1.0 : -1.0,
            0.0,
            DcMotors.ROBOT_SECONDS_PER_FEET * feet
        );
    }
}