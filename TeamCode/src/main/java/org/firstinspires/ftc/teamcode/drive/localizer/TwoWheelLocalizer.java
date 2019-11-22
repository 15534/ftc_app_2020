package org.firstinspires.ftc.teamcode.drive.localizer;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.HardwareNames;

import java.util.Arrays;
import java.util.List;

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
public class TwoWheelLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 360 * 4;
    public static double WHEEL_RADIUS = 1.14173; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 14.4; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -3.25; // in; offset of the lateral wheel

    private DcMotor leftEncoder, rightEncoder, frontEncoder;
    private BNO055IMU imu;

    public TwoWheelLocalizer(HardwareMap hardwareMap, BNO055IMU imu) {
        super(Arrays.asList(
//                new Pose2d(0, LATERAL_DISTANCE / 2, Math.toRadians(180)), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, Math.toRadians(180)), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = hardwareMap.dcMotor.get(HardwareNames.left_encoder);
        rightEncoder = hardwareMap.dcMotor.get(HardwareNames.right_encoder);
        frontEncoder = hardwareMap.dcMotor.get(HardwareNames.horizontal_encoder);
        this.imu = imu;
    }

    public double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
//                encoderTicksToInches(leftEncoder.getCurrentPosition()),
                encoderTicksToInches(rightEncoder.getCurrentPosition()),
                encoderTicksToInches(frontEncoder.getCurrentPosition())
        );
    }
}
