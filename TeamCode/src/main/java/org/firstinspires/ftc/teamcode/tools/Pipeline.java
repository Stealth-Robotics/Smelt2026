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

    // Red CELL – side of field opposite the audience
    RED_FAR(2),

    // Red CELL – audience side
    RED_NEAR(3),

    // Blue CELL – audience side
    BLUE_NEAR(4),

    // Blue CELL – side of field opposite the audience
    BLUE_FAR(5);

    public final int id;

    Pipeline(int id) {
        this.id = id;
    }

}
