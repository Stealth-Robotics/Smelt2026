package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    // Dev BOT
//    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
//        c.frontLeftName.set("frontLeft");
//        c.frontRightName.set("frontRight");
//        c.backLeftName.set("backLeft");
//        c.backRightName.set("backRight");
//        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
//        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
//        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
//        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
//    });

//    public static final PinpointConfig localizerConfig = new PinpointConfig(c -> {
//        c.name.set("goBildaPP");
//        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        c.xPodOffset.set(-6.595);
//        c.yPodOffset.set(-2.42);
//        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
//        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        c.globalDistanceUnit.set(DistanceUnit.INCH);
//        c.offsetUnits.set(DistanceUnit.INCH);
//    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("goBildaPP");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(0.0);
        c.yPodOffset.set(0.0);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

//    public static ForesightConfig foresightConfig = new ForesightConfig(
//            c -> {
//                Controller primaryTranslationalForward = Controller.proportional(0.18614280116058035);
//                Controller secondaryTranslationalForward = Controller.proportional(0.06877479120158314);
//                Controller primaryTranslationalLateral = Controller.proportional(0.24390111828559322);
//                Controller secondaryTranslationalLateral = Controller.proportional(0.09011494604861789);
//
//                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
//                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
//
//                c.coast.set(Controller.proportionalFeedforward(0.017968640190348804));
//                c.brake.set(Controller.proportionalFeedforward(0.015273344161796482));
//
//                c.headingFeedback.set(Controller.proportional(3.4713961089931877));
//                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05325312903703271, 0.0033392145928464495));
//
//                c.linearBrakeCoefficients.set(Matrix.diag(-0.05054132266130609, 0.06884677014422538));
//                c.quadraticBrakeCoefficients.set(Matrix.diag(0.006239061770360558, 0.0010564801658569842));
//
//                c.maxAchievableForwardVelocity.set(60.59014535413563);
//                c.maxAchievableStrafeVelocity.set(47.7181892603203);
//                c.naturalForwardDeceleration.set(40.52049303930872);
//                c.naturalStrafeDeceleration.set(66.62516998821941);
//            }
//    );

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.19100033036370356);
                Controller secondaryTranslationalForward = Controller.proportional(0.07056951844656639);
                Controller primaryTranslationalLateral = Controller.proportional(0.30627664907629604);
                Controller secondaryTranslationalLateral = Controller.proportional(0.11316103797090371);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.017820596811895698));
                c.brake.set(Controller.proportionalFeedforward(0.015147507290111342));

                c.headingFeedback.set(Controller.proportional(2.985052201708149));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.048693829024679544, 0.0035870698273965194));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07120785514794362, 0.07741693246347461));
                c.quadraticBrakeCoefficients.set(Matrix.diag(9.363281536152803E-4, 6.771385630068499E-4));

                c.maxAchievableForwardVelocity.set(62.90944731456953);
                c.maxAchievableStrafeVelocity.set(51.60461789601366);
                c.naturalForwardDeceleration.set(34.380951976730046);
                c.naturalStrafeDeceleration.set(65.5720858423836);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}