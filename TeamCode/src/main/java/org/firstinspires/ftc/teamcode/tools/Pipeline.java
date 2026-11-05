package org.firstinspires.ftc.teamcode.tools;

/**
 * Enum for Limelight pipelines.
 *
 * <p>One pipeline per AprilTag cell grouping. Update the numeric {@code id} values
 * below to match the pipeline indices configured on your robot's Limelight.
 */
public enum Pipeline {

    // All Red Alliance Tags
    RED_ALL(0),

    // ALL Blue Alliance tags
    BLUE_ALL(1),

    // Red CELL – Score table side of field opposite the audience
    RED_SCORE(2),

    // Red CELL – audience side
    RED_AUDIENCE(3),

    // Blue CELL – audience side
    BLUE_AUDIENCE(4),

    // Blue CELL – Score table side of field opposite the audience
    BLUE_SCORE(5);

    public final int id;

    Pipeline(int id) {
        this.id = id;
    }

}
