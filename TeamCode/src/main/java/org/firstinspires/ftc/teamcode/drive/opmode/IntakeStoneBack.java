package org.firstinspires.ftc.teamcode.drive.opmode;

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
@Autonomous(group = "drive", name = "intake stone back")
public class IntakeStoneBack extends LinearOpMode {

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
                        .addMarker(() -> {
                            Operations.startIntake(robot);
                            return null;
                        })
                        .lineTo(new Vector2d(11, -29), new SplineInterpolator(0, Math.toRadians(-135)))
                        .lineTo(new Vector2d(4, -33), new ConstantInterpolator(Math.toRadians(-135)))
                        .lineTo(new Vector2d(22, -25), new SplineInterpolator(Math.toRadians(-135), Math.toRadians(-180)))
                        .build()
        );

    }
}
