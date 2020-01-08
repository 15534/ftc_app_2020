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

import java.util.function.Function;

import kotlin.Unit;
import kotlin.jvm.functions.Function0;

/*
 * Op mode for tuning follower PID coefficients (located in the drive base classes). The robot
 * drives in a DISTANCE-by-DISTANCE square indefinitely.
 */
@Config
@Autonomous(group = "drive", name = "intake stone front")
public class IntakeStoneFront extends LinearOpMode {

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
                        .lineTo(new Vector2d(-2, -24), new SplineInterpolator(0, Math.toRadians(-45)))
                        .addMarker(() -> {
                            Operations.startIntake(robot);
                            return null;
                        })
                        .lineTo(new Vector2d(3, -32), new ConstantInterpolator(Math.toRadians(-45)))
                        .lineTo(new Vector2d(6, -35), new ConstantInterpolator(Math.toRadians(-45)))
                        .lineTo(new Vector2d(22, -25), new LinearInterpolator(Math.toRadians(-45), Math.toRadians(-135)))
                        .addMarker(() -> {
                            new Operations(robot, "INTAKE_OUTTAKE_STONE").run();
                            return null;
                        })
                        .build()
        );

        Thread.sleep(30*1000);
    }

}

class Operations implements Runnable {

    String action;
    HardwareDrivetrain robot;

    Operations(HardwareDrivetrain robot, String action) {
        this.robot = robot;
        this.action = action;
    }

    public void intakeOuttakeStone() throws InterruptedException {
        robot.left_intake.setPower(-0.2);
        robot.right_intake.setPower(-0.2);
        Thread.sleep(500);
        startIntake(robot);
        Thread.sleep(800);
        robot.left_intake.setPower(0);
        robot.right_intake.setPower(0);
    }

    public static void startIntake(HardwareDrivetrain robot) {
        robot.left_intake.setPower(0.6);
        robot.right_intake.setPower(0.6);
    }

    public static void initServos(HardwareDrivetrain robot) {
        // initialize servos
        robot.left_v4b.setPosition(0.6);
        robot.right_v4b.setPosition(0.6);
        robot.push_servo.setPosition(0.35);
        robot.gripper_servo.setPosition(1);
    }

    public static void reset(HardwareDrivetrain robot) {
        // initialize servos
        robot.left_v4b.setPosition(0.6);
        robot.right_v4b.setPosition(0.6);
        robot.push_servo.setPosition(0.35);
        robot.gripper_servo.setPosition(1);

    }

    public void run(){
        try {
            if (this.action.equals("INTAKE_OUTTAKE_STONE")) {
                this.intakeOuttakeStone();
            }
        } catch (InterruptedException e) {}
    }
}
