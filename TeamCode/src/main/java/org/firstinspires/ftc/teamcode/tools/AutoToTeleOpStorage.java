package org.firstinspires.ftc.teamcode.tools;

import com.pedropathing.math.Pose;

public class AutoToTeleOpStorage {

    private static boolean poseValid = false;
    private static Pose lastAutoPose = null;

    public static void setPose(Pose pose){
        lastAutoPose = pose;
        poseValid = true;
    }

    public static Pose getPose(){
        return lastAutoPose;
    }

    public static boolean isPoseValid(){
        return poseValid;
    }

    public static void reset(){
        poseValid = false;
    }

}
