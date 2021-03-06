package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.mecanum.VertexDrive;
import org.firstinspires.ftc.teamcode.hardware.HardwareDrivetrain;


/*
 * Op mode for tuning follower PID coefficients (located in the drive base classes). The robot
 * drives in a DISTANCE-by-DISTANCE square indefinitely.
 */

@Autonomous(group = "drive", name = "Init Auto")
public class JustInitAuto extends LinearOpMode {
    HardwareDrivetrain robot = new HardwareDrivetrain();

    @Override
    public void runOpMode() throws InterruptedException {
        // starts on blue side
        VertexDrive drive = new VertexDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(80, -31, Math.toRadians(90)));
        robot.init(hardwareMap);

        // reset lift encoders
        robot.lift_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.lift_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lift_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        sleep(5000);
        robot.left_v4b.setPosition(0.6);
        robot.right_v4b.setPosition(0.6);
        sleep(500);
        robot.left_intake.setPower(0.8);
        robot.right_intake.setPower(0.8);
        sleep(1000);
        robot.left_intake.setPower(0);
        robot.right_intake.setPower(0);
        sleep(500);
        robot.push_servo.setPosition(0.35);
        robot.gripper_servo.setPosition(1);
        robot.foundation_right.setPosition(0.2);
        robot.foundation_left.setPosition(0.46);
        sleep(1000);
    }
}

