package org.firstinspires.ftc.teamcode.drive.opmode;

import android.graphics.Path;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.mecanum.VertexDrive;
import org.firstinspires.ftc.teamcode.hardware.HardwareDrivetrain;

/*
 * Op mode for tuning follower PID coefficients (located in the drive base classes). The robot
 * drives in a DISTANCE-by-DISTANCE square indefinitely.
 */
@Config
@Autonomous(group = "drive", name = "intake stone center")
public class IntakeStoneCenter extends LinearOpMode {

    HardwareDrivetrain robot = new HardwareDrivetrain();

    @Override
    public void runOpMode() throws InterruptedException {
        // starts on blue side
        VertexDrive drive = new VertexDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        robot.init(hardwareMap);

        // reset lift encoders
        robot.lift_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.lift_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lift_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .addMarker(() -> {
                            Operations.reset(robot);
                            return null;
                        })
                        .lineTo(new Vector2d(10, -26), new SplineInterpolator(0, Math.toRadians(-135)))
                        .addMarker(() -> {
                            Operations.startIntake(robot);
                            return null;
                            })
                        .lineTo(new Vector2d(15, -26), new ConstantInterpolator(Math.toRadians(-135)))
                        .lineTo(new Vector2d(15, -29.5), new ConstantInterpolator(Math.toRadians(-135)))
                        .lineTo(new Vector2d(10.5, -34.5), new ConstantInterpolator(Math.toRadians(-135)))
                        .lineTo(new Vector2d(24, -25.5), new SplineInterpolator(Math.toRadians(-135), Math.toRadians(-180)))
                        .build()
        );
        robot.left_intake.setPower(-0.2);
        robot.right_intake.setPower(-0.2);
        sleep(500);
        Operations.startIntake(robot);
        sleep(800);
        robot.left_intake.setPower(0);
        robot.right_intake.setPower(0);
    }
}
