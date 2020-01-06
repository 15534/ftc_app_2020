package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.VertexDrive;

/*
 * Op mode for tuning follower PID coefficients (located in the drive base classes). The robot
 * drives in a DISTANCE-by-DISTANCE square indefinitely.
 */
@Config
@Autonomous(group = "drive", name = "intake stone test")
public class IntakeStoneTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // starts on blue side
        VertexDrive drive = new VertexDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(22.9, 0, 0));

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(3)
                        .splineTo(new Pose2d(22.9+3.9, -23.6, Math.toRadians(270)))
                        .build()
        );
    }
}
