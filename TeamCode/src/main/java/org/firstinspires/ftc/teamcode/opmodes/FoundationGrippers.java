package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.mecanum.VertexDrive;
import org.firstinspires.ftc.teamcode.hardware.HardwareDrivetrain;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.HashMap;

/*
 * Op mode for tuning follower PID coefficients (located in the drive base classes). The robot
 * drives in a DISTANCE-by-DISTANCE square indefinitely.
 */
@Config
@Autonomous(group = "drive", name = "FoundationGrippers")
public class FoundationGrippers extends LinearOpMode {
    private OpenCvInternalCamera phoneCam;
    private SkystoneDetector skyStoneDetector;

    HardwareDrivetrain robot = new HardwareDrivetrain();

    enum State {
        INTAKE_OUT_AND_IN, RESET_SERVOS, STOP_INTAKE, START_INTAKE, REVERSE_INTAKE, DROP_GRIPPERS_HALFWAY
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // starts on blue side
        VertexDrive drive = new VertexDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(90, -31, Math.toRadians(-90)));
        robot.init(hardwareMap);

        ElapsedTime timer = new ElapsedTime();
        HashMap<State, Double> stateTimes = new HashMap<>();  // map of state -> start time

        waitForStart();
        robot.foundation_left.setPosition(1);
        robot.foundation_right.setPower(0.5);
        sleep(500);
        robot.foundation_right.setPower(0);

//        sleep(500);
//        robot.foundation_left.setPower(0);
//        robot.foundation_right.setPower(0);


//        while (opModeIsActive()) {
//            drive.update();
//
//            double currentTime = timer.milliseconds();
//
//            // set all start times if they don't exist
//            for (State key : stateTimes.keySet()) {
//                if (stateTimes.get(key) == null) {
//                    stateTimes.put(key, currentTime);
//                }
//            }
//
//            if (stateTimes.containsKey(State.INTAKE_OUT_AND_IN)) {
//                // intake out and in case stone gets stuck
//                Double startTime = stateTimes.get(State.INTAKE_OUT_AND_IN);
//                double elapsedTime = 0;
//                if (startTime != null) {
//                    elapsedTime = currentTime - startTime;
//                }
//
//                telemetry.addData("INTAKE_OUT_AND_IN", elapsedTime);
//
//                if (elapsedTime < 500) {
//                    robot.left_intake.setPower(-0.2);
//                    robot.right_intake.setPower(-0.2);
//                } else if (elapsedTime < 1300) {
//                    robot.left_intake.setPower(0.6);
//                    robot.right_intake.setPower(0.6);
//                } else {
//                    robot.left_intake.setPower(0);
//                    robot.right_intake.setPower(0);
//                    stateTimes.remove(State.INTAKE_OUT_AND_IN);
//                }
//            }
//
//            // start/stop/reverse intake
//            if (stateTimes.containsKey(State.START_INTAKE)) {
//                robot.left_intake.setPower(0.6);
//                robot.right_intake.setPower(0.6);
//                telemetry.addData("START_INTAKE", 0);
//                stateTimes.remove(State.START_INTAKE);
//            } else if (stateTimes.containsKey(State.STOP_INTAKE)) {
//                robot.left_intake.setPower(0);
//                robot.right_intake.setPower(0);
//                telemetry.addData("STOP_INTAKE", 0);
//                stateTimes.remove(State.STOP_INTAKE);
//            } else if (stateTimes.containsKey(State.REVERSE_INTAKE)) {
//                robot.left_intake.setPower(-0.6);
//                robot.right_intake.setPower(-0.6);
//                telemetry.addData("REVERSE_INTAKE", 0);
//                stateTimes.remove(State.REVERSE_INTAKE);
//            }
//
//            // reset servos
//            if (stateTimes.containsKey(State.RESET_SERVOS)) {
//                // initialize servos
//                Double startTime = stateTimes.get(State.RESET_SERVOS);
//                double elapsedTime = 0;
//                if (startTime != null) {
//                    elapsedTime = currentTime - startTime;
//                }
//
//                telemetry.addData("RESET_SERVOS", elapsedTime);
//
//                if (elapsedTime < 500) {
//                    robot.left_v4b.setPosition(0.6);
//                    robot.right_v4b.setPosition(0.6);
//                    robot.push_servo.setPosition(0.35);
//                    robot.gripper_servo.setPosition(1);
//                    robot.foundation_left.setPower(-1);
//                    robot.foundation_right.setPower(-1);
//                } else if (elapsedTime < 1000) {
//                    robot.left_intake.setPower(0.8);
//                    robot.right_intake.setPower(0.8);
//                    robot.foundation_left.setPower(0);
//                    robot.foundation_right.setPower(0);
//                } else {
//                    // start intake
//                    robot.left_intake.setPower(0.6);
//                    robot.right_intake.setPower(0.6);
//                    stateTimes.remove(State.RESET_SERVOS);
//                }
//            }
//
//            // drop grippers
//            if (stateTimes.containsKey(State.DROP_GRIPPERS_HALFWAY)) {
//                // initialize servos
//                Double startTime = stateTimes.get(State.DROP_GRIPPERS_HALFWAY);
//                double elapsedTime = 0;
//                if (startTime != null) {
//                    elapsedTime = timer.milliseconds() - startTime;
//                }
//                telemetry.addData("START_TIME", startTime);
//                telemetry.addData("CURRENT TIME", currentTime);
//                telemetry.addData("DROP_GRIPPERS_HALFWAY", elapsedTime);
//
//                if (elapsedTime < 2000) {
//                    telemetry.addLine("GOING UP");
//                    robot.foundation_left.setPower(-1);
//                    robot.foundation_right.setPower(-1);
//                } else {
//                    telemetry.addLine("STOPPING");
//                    robot.foundation_left.setPower(0);
//                    robot.foundation_right.setPower(0);
////                    stateTimes.remove(State.DROP_GRIPPERS_HALFWAY);
//                }
//            }
//            telemetry.addData("stateTimes", stateTimes);
//
//            telemetry.update();
//        }
    }
}

