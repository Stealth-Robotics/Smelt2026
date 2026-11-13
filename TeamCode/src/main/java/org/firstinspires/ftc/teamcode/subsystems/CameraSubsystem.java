package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.tools.FieldConstants;
import org.firstinspires.ftc.teamcode.tools.Pipeline;
import org.firstinspires.ftc.teamcode.tools.Targets;
import org.firstinspires.ftc.teamcode.tools.Trajectories;
import org.stealthrobotics.library.StealthSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

import java.util.ArrayList;
import java.util.List;

public class CameraSubsystem extends StealthSubsystem
{

    private final Telemetry telemetry = StealthOpMode.telemetry;
    public static final String NAME = "limelight";
    private  final Limelight3A limelight;
    /** The length of time to keep LLResults in the queue before removing them, in milliseconds. */
    private static final long QUEUE_DEFAULT_TIMEOUT_MS = 500;

    /** Rotational difference in degrees between the Limelight's yaw and Pedro's IMU yaw. */
    private static final double LL_ANGLE_DELTA = 90;

    /** How many degrees back the Limelight is rotated from perfectly vertical. */
    private static final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 40;

    /** Distance from the center of the Limelight lens to the floor, in inches. */
    private static final double LIMELIGHT_FLOOR_HEIGHT_INCHES = 11;

    /** The height of the center of an AprilTag from the floor, in inches. */
    private static final double APRIL_TAG_CENTER_HEIGHT_INCHES = 49.65551;

    /** Physical Y offset of the Limelight lens relative to the shooter/robot center (in inches).
     *  Positive value indicates the camera is mounted to the LEFT of the shooter. */
    public static final double LIMELIGHT_Y_OFFSET_INCHES = 0;

    /** The last valid result received from the Limelight. */
    private LLResult lastResult;

    private boolean areTagsVisible = false;

    /**
     * Lookup table mapping tag visibility bitmask (0 to 15) to offset in tag-spacing units.
     * This is for limelight "smart" grouping of AprilTags as the aimpoint will shift base on tags seen
     */
    private static final double[] SPACING_OFFSET_LOOKUP = {
            0.0,            // 0: None
            7.4,            // 1: {0} -> target center is 1.5 spacings right
            2.5,            // 2: {1} -> target center is 0.5 spacings right
            5.6,            // 3: {0,1}
            -2.5,           // 4: {2} -> target center is 0.5 spacings left
            3.5,            // 5: {0,2}
            0.0,            // 6: {1,2}
            2.2,            // 7: {0,1,2}
            -7.4,           // 8: {3} -> target center is 1.5 spacings left
            0.0,            // 9: {0,3}
            -2.5,           // 10: {1,3}
            0.0,           // 11: {0,1,3}
            -5.5,           // 12: {2,3}
            -0.0,          // 13: {0,2,3}
            -2.2,           // 14: {1,2,3}
            0.0             // 15: {0,1,2,3}
    };

    // Used to compute per tag spacing offsets based on distance to target
    private static final double TUNING_TA = 0.0058;
    private static final double TUNING_SCALE = 0.45;

    public CameraSubsystem (HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, NAME);
        // setPollRate must be called before start
        limelight.setPollRateHz(250);
        limelight.start();
        limelight.pipelineSwitch(Pipeline.RED_AUDIENCE.id);
    }

    /**
     * Gets the last valid result received from the Limelight within that last 40ms
     * @return The last valid result received from the Limelight or null
     */
    @Nullable
    public LLResult getLastResult() {
        // TODO: Do we want to add averaging and standard deviation?
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            this.lastResult = result;
        }

        return this.lastResult != null && lastResult.getStaleness() < 40 ? lastResult : null;
    }
    public boolean canUpdate (LLResult result, Pose pose) {
        if (result != null && result.isValid()) {
            if (result.getFiducialResults().size() > 3) {
                Pose camPose = limelightToPedroPose(result.getBotpose());
                double taDistance = getAverageTa(result);
                double camDistance = Targets.getDistanceToTarget(camPose);
                if (Math.abs(taDistance - camDistance) < 1 && taDistance < 62 && taDistance > 40) {
                    return true;
                }
            }
        }
        return false;
    }
    @Override
    public void periodic(){
        LLResult result = getLastResult();
        telemetry.addData("Pipeline:", getCurrentPipeline());
        if (result != null && result.isValid()) {
            areTagsVisible = true;

            telemetry.addData("Tags:", getTagIds(result));
            Pose pose = limelightToPedroPose(result.getBotpose());
            telemetry.addData("Camera Pose: ",  "X: %.2f, Y:%.2f, Z:%.2f", pose.x(),pose.y(), Math.toDegrees(pose.heading()));
            pose = limelightToPedroPose(result.getBotpose_MT2());
            telemetry.addData("Camera MT2 Pose","X: %.2f, Y:%.2f, Z:%.2f", pose.x(),pose.y(), Math.toDegrees(pose.heading()));
            telemetry.addData("MT2 Distance to Target", "%.2f", Targets.getDistanceToTarget(pose));

        } else {
            areTagsVisible = false;
            telemetry.addData("Tags:", "None");
        }
    }

    public Limelight3A getLimelight() {
        return limelight;
    }

    /**
     * Determine if any tags are visible
     * @return true if any tags are visible otherwise false
     */
    public boolean areTagsVisible() {
        return areTagsVisible;
    }

    /**
     * updates limelight MT2 orientation based on Pedro heading
     * @param pedroHeading RADIAN heading from Pedro
     */
    public void updateRobotOrientation(double pedroHeading){
        double yaw = getCorrectedYaw(pedroHeading);
        limelight.updateRobotOrientation(yaw);
    }

    /**
     * Gets the currently active pipeline on the Limelight.
     *
     * @return The current {@link Pipeline}.
     */
    public Pipeline getCurrentPipeline() {
        return Pipeline.values()[limelight.getStatus().getPipelineIndex()];
    }

    /**
     * Switches the Limelight's active pipeline.
     *
     * @param pipeline The {@link Pipeline} to switch to.
     */
    public void setPipeline(@NonNull Pipeline pipeline) {

        if (pipeline == getCurrentPipeline()) {
            return;
        }

        limelight.pipelineSwitch(pipeline.id);
        this.lastResult = null;
    }

    /**
     * Calculates distance to a goal by using the cameras Ta value using 4th order polynomial
     * @param result The LLResult from Limelight
     * @return distance in inches
     */
    public double calcGoalDistanceByTa(LLResult result) {
        double ta = getAverageTa(result);

        double distance = 112.8561 - 29484.76 * ta + 5687858.0 * Math.pow(ta, 2) - 582954800.0 * Math.pow(ta, 3) + 24568860000.0 * Math.pow(ta, 4);
        return Math.round(distance * 10) / 10.0; // round to 1 decimal places)
    }


    /**
     * Gets the adjusted tx angle in degrees aimed directly from the shooter
     * to the center of the hive target, compensating for both:
     * 1) Imbalanced AprilTag grouping (hive center offset)
     * 2) Physical camera offset (4.5 inches left of shooter)
     *
     * @param result The LLResult from Limelight.
     * @return The corrected tx angle for the shooter.
     */
    public double getAdjustedTx(@Nullable LLResult result) {
        if (result == null || !result.isValid()) {
            return 0.0;
        }

        double hiveGroupOffset = getAimingOffsetDegrees(result);
        // telemetry.addData("Hive Group Offset", hiveGroupOffset);
        double physicalOffset = getPhysicalCameraOffsetDegrees(result);

        return result.getTx() + hiveGroupOffset + physicalOffset;
    }

    /**
     * Converts the limelight coordinate system to the pedro coordinate system.
     * @param botPose The pose of the bot in the limelight coordinate system.
     * @return The pose of the bot in the Pedro coordinate system.
     */
    @NonNull
    public static Pose limelightToPedroPose(@NonNull Pose3D botPose) {

        Position p = botPose.getPosition().toUnit(DistanceUnit.INCH);
        double llYawRad = botPose.getOrientation().getYaw(AngleUnit.RADIANS);

        // Limelight uses a center field 0,0 and x and y are swapped. Yaw is also rotated by 90 degrees
        return new Pose(
                FieldConstants.FIELD_X_CENTER + p.y, // intentional field center is swapped with x and y
                FieldConstants.FIELD_Y_CENTER - p.x,    // intentional field center is swapped with x and y
                AngleUnit.normalizeRadians(llYawRad - Math.toRadians(LL_ANGLE_DELTA)));
    }

    /**
     * Calculates distance to a goal by using the cameras Ty value
     * NOTE: Not useful for BioBuzz
     * @param ty The Ty value from the camera
     *
     * @return The distance to the goal in inches
     */
    public static double calcGoalDistanceByTy(double ty) {
        // This is the total angle from the camera to the target, relative to a horizontal line from the camera.
        double totalAngleDegrees = LIMELIGHT_MOUNT_ANGLE_DEGREES + ty;

        // Convert the total angle to radians for use in Math.tan()
        double totalAngleRadians = Math.toRadians(totalAngleDegrees);

        // Calculate the height difference between the AprilTag and the Limelight
        double heightDifference = APRIL_TAG_CENTER_HEIGHT_INCHES - LIMELIGHT_FLOOR_HEIGHT_INCHES;

        // Using trigonometry (tangent) to find the horizontal distance.
        // distance = opposite / tan(angle)

        return heightDifference / Math.tan(totalAngleRadians);
    }

    /**
     * Gets the average Ta value from a LLResult
     * @param result The LLResult
     * @return The average Ta value
     */
    private double getAverageTa(@Nullable LLResult result) {
        if (result == null || !result.isValid()) {
            return 0.0;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return 0.0;
        }
        List<Double> areas = new ArrayList<>();
        for (LLResultTypes.FiducialResult fr : fiducials) {
            areas.add(fr.getTargetArea());
        }
        if (areas.isEmpty()) {
            return 0.0;
        }

        return calculateAverage(areas);
    }

    /**
     * Gets the tag IDs from a LLResult
     * @param result The LLResult
     * @return List of tag IDs
     */
    private List<Integer> getTagIds(LLResult result ) {
        List<Integer> tags = new ArrayList<>();
        if (result == null || !result.isValid()) {
            return tags;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        for (int i = 0; i < fiducials.size(); i++) {
            tags.add(fiducials.get(i).getFiducialId());
        }

        return tags;
    }

    /**
     * Returns the relative index (0 to 3, from left to right facing the hive)
     * of an AprilTag within its hive group.
     * Subtracts the hive's base AprilTag ID:
     * - RED_SCORE:   IDs 30..33 -> base 30
     * - RED_AUDIENCE:  IDs 34..37 -> base 34
     * - BLUE_AUDIENCE: IDs 38..41 -> base 38
     * - BLUE_SCORE:  IDs 42..45 -> base 42
     *
     * @param tagId The AprilTag ID.
     * @return Relative index 0..3, or -1 if not a hive tag.
     */
    private static int getRelativeTagIndex(int tagId) {
        if (tagId >= 30 && tagId <= 33) return tagId - 30; // RED_SCORE
        if (tagId >= 34 && tagId <= 37) return tagId - 34; // RED_AUDIENCE
        if (tagId >= 38 && tagId <= 41) return tagId - 38; // BLUE_AUDIENCE
        if (tagId >= 42 && tagId <= 45) return tagId - 42; // BLUE_SCORE
        return -1;
    }

    /**
     * Helper method that calculates the tag visibility bitmask (0..15) for a given LLResult.
     *
     * @param result The LLResult.
     * @return The bitmask of detected tags.
     */
    private int getHiveBitmask(@Nullable LLResult result) {
        if (result == null || !result.isValid()) {
            return 0;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return 0;
        }

        int bitmask = 0;
        for (LLResultTypes.FiducialResult fr : fiducials) {
            int relIdx = getRelativeTagIndex(fr.getFiducialId());
            if (relIdx >= 0 && relIdx <= 3) {
                bitmask |= (1 << relIdx);
            }
        }

        return bitmask;
    }

    /**
     * Calculates the aiming offset in degrees using the lookup table.
     *
     * @param result The LLResult from Limelight.
     * @param distanceScale Default degrees per tag spacing to use if only 1 tag is detected.
     * @return The offset in degrees to add to result.getTx().
     */
    private double getAimingOffsetDegrees(@Nullable LLResult result, double distanceScale) {
        int bitmask = getHiveBitmask(result);
        if (bitmask == 0) {
            return 0.0;
        }

        return SPACING_OFFSET_LOOKUP[bitmask] * distanceScale;
    }

    /**
     * Calculates the aiming offset in degrees using the lookup table.
     *
     * @param result The LLResult from Limelight.
     * @return The offset in degrees to add to result.getTx().
     */
    private double getAimingOffsetDegrees(@Nullable LLResult result) {
        if (result == null || !result.isValid()) {
            return 0.0;
        }

        double avgTa = getAverageTa(result);
        double scale = Math.pow(avgTa / TUNING_TA, TUNING_SCALE);

        return getAimingOffsetDegrees(result, scale);
    }

    /**
     * Calculates the angular correction in degrees needed to compensate for the Limelight's
     * physical offset relative to the shooter centerline.
     * Uses distance calculated from calcGoalDistanceByTa.
     *
     * @param result The LLResult from Limelight.
     * @return The angular offset in degrees to add to tx.
     */
    private double getPhysicalCameraOffsetDegrees(@Nullable LLResult result) {
        if (result == null || !result.isValid() || LIMELIGHT_Y_OFFSET_INCHES == 0) {
            return 0.0;
        }

        double distanceInches = calcGoalDistanceByTa(result);
        if (distanceInches <= 0.0 || Double.isNaN(distanceInches)) {
            return 0.0;
        }

        // Camera is to the left of shooter (+4.5 inches).
        // Target appears to the right of camera crosshair when shooter is aligned.
        // Therefore, we subtract atan(4.5 / distance) from camera tx.
        double offsetAngleRadians = Math.atan2(LIMELIGHT_Y_OFFSET_INCHES, distanceInches);
        return -Math.toDegrees(offsetAngleRadians);
    }

    /**
     * Extracts the robot's yaw angle corrected for Limelight mounting offset.
     * This is the yaw value suitable for robot localizer updates.
     *
     * @param pedroYaw The base yaw (in radians) from the Pedro system/IMU.
     * @return The corrected yaw in degrees (0-360).
     */
    private double getCorrectedYaw(double pedroYaw) {
        return Math.toDegrees(
                AngleUnit.normalizeRadians(pedroYaw + Math.toRadians(LL_ANGLE_DELTA)));
    }

    /**
     * Calculates the average of a list of values
     * @param values list of values
     * @return average
     */
    private static double calculateAverage(List<Double> values) {
        if (values.isEmpty()) {
            return 0.0;
        }
        double sum = 0.0;
        for (Double value : values) {
            sum += value;
        }
        return sum / values.size();
    }

    /**
     * Calculates the standard deviation of a list of values
     * @param values list of values
     * @param average average of the values
     * @return standard deviation
     */
    private static double calculateStandardDeviation(List<Double> values, double average) {
        if (values.isEmpty()) {
            return 0.0;
        }
        double sumOfSquaredDeviations = 0.0;
        for (Double value : values) {
            sumOfSquaredDeviations += Math.pow(value - average, 2);
        }
        return Math.sqrt(sumOfSquaredDeviations / (values.size() - 1));
    }

}
