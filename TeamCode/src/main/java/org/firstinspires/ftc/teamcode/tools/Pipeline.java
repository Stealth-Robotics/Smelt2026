package org.firstinspires.ftc.teamcode.tools;

public enum Pipeline {
    RED_ALL(0),
    BLUE_ALL(1),
    RED_FAR(2),
    RED_NEAR(3),
    BLUE_NEAR(4),
    BLUE_FAR(5);


    public final int id;
    Pipeline(int id) {this.id = id;}
}
