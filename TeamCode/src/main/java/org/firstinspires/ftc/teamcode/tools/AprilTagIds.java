package org.firstinspires.ftc.teamcode.tools;

public enum AprilTagIds {

    TAG_ID_NONE(-1),

    // Red CELL – side of field opposite the audience
    RED_FAR_30(30), RED_FAR_31(31), RED_FAR_32(32), RED_FAR_33(33),

    // Red CELL – audience side
    RED_NEAR_34(34), RED_NEAR_35(35), RED_NEAR_36(36), RED_NEAR_37(37),

    // Blue CELL – audience side
    BLUE_NEAR_38(38), BLUE_NEAR_39(39), BLUE_NEAR_40(40), BLUE_NEAR_41(41),

    // Blue CELL – side of field opposite the audience
    BLUE_FAR_42(42), BLUE_FAR_43(43), BLUE_FAR_44(44), BLUE_FAR_45(45);

    public final int id;
    AprilTagIds(int id) {
        this.id = id;
    }

    public static AprilTagIds fromId(int id) {
        for (AprilTagIds tagId : AprilTagIds.values()) {
            if (tagId.id == id) {
                return tagId;
            }
        }

        return AprilTagIds.TAG_ID_NONE;
    }
}
