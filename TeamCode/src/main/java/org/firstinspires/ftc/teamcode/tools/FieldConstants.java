package org.firstinspires.ftc.teamcode.tools;

import com.pedropathing.math.Pose;

public class FieldConstants {
    public static final double FIELD_X_MAX = 141.5;
    public static final double FIELD_Y_MAX = 141.5;

    public static final double FIELD_X_CENTER = FIELD_X_MAX / 2;

    public static final double FIELD_Y_CENTER = FIELD_Y_MAX / 2;

    public static final double ROBOT_CENTER_Y = 17.99 / 2;
    public static final double ROBOT_CENTER_X = 16.95 / 2;

    public static final double ROBOT_BLUE_HEADING = Math.toRadians(180);
    public static final double ROBOT_RED_HEADING = Math.toRadians(0);

    // Rests to the Scoring table side Blue wall intake facing away from driver
    public static final Pose BLUE_SCORE_POSE = new Pose(
            FieldConstants.FIELD_X_MAX - FieldConstants.ROBOT_CENTER_X,
            FieldConstants.FIELD_Y_MAX -
                    FieldConstants.ROBOT_CENTER_Y,
            FieldConstants.ROBOT_BLUE_HEADING);

    // Rests to the Audience side Blue wall intake facing away from driver
    public static final Pose BLUE_AUDIENCE_POSE = new Pose(
            FieldConstants.FIELD_X_MAX - FieldConstants.ROBOT_CENTER_X,
            FieldConstants.ROBOT_CENTER_Y,
            FieldConstants.ROBOT_BLUE_HEADING);

    // Resets to the Audience side Red wall intake facing away from driver
    public static final Pose RED_AUDIENCE_POSE = new Pose(
            FieldConstants.ROBOT_CENTER_X,
            FieldConstants.ROBOT_CENTER_Y,
            FieldConstants.ROBOT_RED_HEADING);

    // Resets to the scoring table side red wall
    public static final Pose RED_SCORE_POSE = new Pose(
            FieldConstants.ROBOT_CENTER_X,
            FieldConstants.FIELD_Y_MAX -
                    FieldConstants.ROBOT_CENTER_Y,
            FieldConstants.ROBOT_RED_HEADING);

}
