package org.firstinspires.ftc.teamcode.tools;

public enum AprilTagIds {

    TAG_ID_NONE(-1),

    RED_FAR_30(30), RED_FAR_31(31), RED_FAR_32(32), RED_FAR_33(33),
    // red, opposite of audience

    RED_NEAR_34(34), RED_NEAR_35(35), RED_NEAR_36(36), RED_NEAR_37(37),
    // red, audience side

    BLUE_NEAR_38(38), BLUE_NEAR_39(39), BLUE_NEAR_40(40), BLUE_NEAR_41(41),
    // blue, audience side

    BLUE_FAR_42(42), BLUE_FAR_43(43), BLUE_FAR_44(44), BLUE_FAR_45(45);
    // blue, opposite of audience

    public final int id;

    AprilTagIds(int id) {this.id = id;}

    public static AprilTagIds fromId(int id) {
        for (AprilTagIds tagIds : AprilTagIds.values()) {
            if (tagIds.id == id) {
                return tagIds;
            }
        }
        return AprilTagIds.TAG_ID_NONE;
    }
}
