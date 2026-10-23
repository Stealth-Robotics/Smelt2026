package org.firstinspires.ftc.teamcode.tools;

import com.pedropathing.math.Pose;

public class AutoToTeleOpStorage {

    private static boolean poseValid = false;
    private static Pose lastAutoPose = null;

    public static void setPose(Pose pose){

        if (pose == null || (pose.x() == 0 && pose.y() == 0) && pose.heading() == 0){
            return;
        }

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
        lastAutoPose = null;
        poseValid = false;
    }

}
