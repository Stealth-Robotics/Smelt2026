package org.firstinspires.ftc.teamcode.tools;

import com.pedropathing.math.Pose;

import org.stealthrobotics.library.Alliance;

public class Targets {

    // Hive cell target poses
    private static final Pose BLUE_RIGHT_CELL = new Pose(84,80);
    private static final Pose BLUE_LEFT_CELL = new Pose(84, 61.5);
    private static final Pose RED_RIGHT_CELL = new Pose(57.8,61.5);
    private static final Pose RED_LEFT_CELL = new Pose(57.8, 80);
    private static final double HEIGHT = 60.0;

    private static Pose getTarget(Pose robotPose) {
        return Alliance.isRed() ?
                (robotPose.y() > 72 ? RED_LEFT_CELL : RED_RIGHT_CELL) :
                (robotPose.y() < 72 ? BLUE_LEFT_CELL : BLUE_RIGHT_CELL);
    }

    public static double getDistanceToTarget(Pose drivePosition) {
        Pose targetPosition = getTarget(drivePosition);
        // From https://en.wikipedia.org/wiki/Euclidean_distance#Higher_dimensions
        return Math.sqrt(Math.pow(drivePosition.x() - targetPosition.x(),2) + Math.pow(drivePosition.y() - targetPosition.y(),2) + Math.pow(HEIGHT, 2));
    }

    public static double getAngleToTarget(Pose drivePosition) {
        Pose targetPosition = getTarget(drivePosition);
        // Gets field relative angle to target
        return Math.atan2(targetPosition.y() - drivePosition.y(), targetPosition.x() - drivePosition.x());
    }
}
