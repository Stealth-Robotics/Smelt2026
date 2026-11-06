package org.firstinspires.ftc.teamcode.tools;

import com.pedropathing.math.Pose;

import org.stealthrobotics.library.Alliance;

public class Targets {

    // Hive cell target poses (in inches) center of the April tags to align with vision data
    private static final Pose BLUE_SCORE_CELL = new Pose(83.3,89);
    private static final Pose BLUE_AUDIENCE_CELL = new Pose(83.3, 52.5);
    private static final Pose RED_AUDIENCE_CELL = new Pose(57.8,52.5);
    private static final Pose RED_SCORE_CELL = new Pose(57.8, 89);
    private static final double HEIGHT = 40.0;

    /**
     * Gets the distance to the target in inches. Calculates for height of target
     * @param drivePosition The current pose of the robot.
     * @return The distance to the target in inches.
     */
    public static double getDistanceToTarget(Pose drivePosition) {
        Pose targetPosition = getTarget(drivePosition);
        // From https://en.wikipedia.org/wiki/Euclidean_distance#Higher_dimensions
        return Math.sqrt(Math.pow(drivePosition.x() - targetPosition.x(),2) + Math.pow(drivePosition.y() - targetPosition.y(),2) + Math.pow(HEIGHT, 2));
    }


    /**
     * Gets the angle error to the target in degrees.
     * @param drivePosition The current pose of the robot.
     * @return The angle error to the target in degrees.
     */
    public static double getAngleErrorToTarget(Pose drivePosition) {
        Pose targetPosition = getTarget(drivePosition);

        // Get target angle in RADIANS
        double targetAngleRad = Math.atan2(targetPosition.y() - drivePosition.y(), targetPosition.x() - drivePosition.x());

        // Smelts shooter is on the back
        double targetHeadingRad = targetAngleRad + Math.PI;

        // Calculate raw error in RADIANS
        double errorRad =  drivePosition.heading() - targetHeadingRad;

        // Normalize error to [-PI, PI] to get shortest path
        double normalizedErrorRad = Math.atan2(Math.sin(errorRad), Math.cos(errorRad));

        // Convert the final normalized error to degrees
        return Math.toDegrees(normalizedErrorRad);
    }


    /**
     * Based on robot position, return the target position
     * @param robotPose The current pose of the robot.
     * @return The target position.
     */
    private static Pose getTarget(Pose robotPose) {
        return Alliance.isRed() ?
                (robotPose.y() > FieldConstants.FIELD_Y_CENTER ? RED_SCORE_CELL : RED_AUDIENCE_CELL) :
                (robotPose.y() < FieldConstants.FIELD_Y_CENTER ? BLUE_AUDIENCE_CELL : BLUE_SCORE_CELL);
    }
}
