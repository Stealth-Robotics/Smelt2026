package org.firstinspires.ftc.teamcode.tools;

public enum AprilTagIds {

    TAG_ID_NONE(-1),

    // Red CELL – Score table side of the field opposite the audience
    RED_SCORE_30(30), RED_SCORE_31(31), RED_SCORE_32(32), RED_SCORE_33(33),

    // Red CELL – audience side
    RED_AUDIENCE_34(34), RED_AUDIENCE_35(35), RED_AUDIENCE_36(36), RED_AUDIENCE_37(37),

    // Blue CELL – audience side
    BLUE_AUDIENCE_38(38), BLUE_AUDIENCE_39(39), BLUE_AUDIENCE_40(40), BLUE_AUDIENCE_41(41),

    // Blue CELL – Score table side of field opposite the audience
    BLUE_SCORE_42(42), BLUE_SCORE_43(43), BLUE_SCORE_44(44), BLUE_SCORE_45(45);

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
