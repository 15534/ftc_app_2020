package org.firstinspires.ftc.teamcode.drive.localizer;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.teamcode.hardware.HardwareNames;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 * Note: this could be optimized significantly with REV bulk reads
 */
@Config
public class VertexLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 360 * 4;
    public static double WHEEL_RADIUS = 1.14173; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 13.9; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -3.0; // in; offset of the lateral wheel //vertical offset between the center and side wheels are -2.78
    public static double SIDE_WHEEL_FORWARD_OFFSET = -0.312;// vertical offset of the side odemetry pods/wheels

    private ExpansionHubMotor leftEncoder, rightEncoder, frontEncoder;
    private ExpansionHubEx hub;

    public VertexLocalizer(HardwareMap hardwareMap) {

        // TODO check if these positions are correct

        super(Arrays.asList(
                new Pose2d(SIDE_WHEEL_FORWARD_OFFSET, LATERAL_DISTANCE / 2, Math.toRadians(180)), // left
                new Pose2d(SIDE_WHEEL_FORWARD_OFFSET, -LATERAL_DISTANCE / 2, Math.toRadians(180)), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");

        leftEncoder = hardwareMap.get(ExpansionHubMotor.class, HardwareNames.left_encoder);
        rightEncoder = hardwareMap.get(ExpansionHubMotor.class, HardwareNames.right_encoder);
        frontEncoder = hardwareMap.get(ExpansionHubMotor.class, HardwareNames.horizontal_encoder);
    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
//        return Arrays.asList(
//                encoderTicksToInches(leftEncoder.getCurrentPosition()),
//                encoderTicksToInches(rightEncoder.getCurrentPosition()),
//                encoderTicksToInches(frontEncoder.getCurrentPosition())
//        );

        RevBulkData bulkData = hub.getBulkInputData();
//
//        if (bulkData == null) {
//            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
//        }

        List<ExpansionHubMotor> encoders = Arrays.asList(leftEncoder, rightEncoder, frontEncoder);
        List<Double> wheelPositions = new ArrayList<>();
        for (ExpansionHubMotor motor : encoders) {
            wheelPositions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(motor)));
        }
        return wheelPositions;
    }
}
