package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
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

import static org.firstinspires.ftc.teamcode.opmodes.LiftPIDTest.k_G;
import static org.firstinspires.ftc.teamcode.opmodes.LiftPIDTest.k_d;
import static org.firstinspires.ftc.teamcode.opmodes.LiftPIDTest.k_i;
import static org.firstinspires.ftc.teamcode.opmodes.LiftPIDTest.k_p;

/*
 * Op mode for tuning follower PID coefficients (located in the drive base classes). The robot
 * drives in a DISTANCE-by-DISTANCE square indefinitely.
 */
@Config
@Autonomous(group = "drive", name = "Foundation Autonomous")
public class FoundationAuto extends LinearOpMode {
    HardwareDrivetrain robot = new HardwareDrivetrain();

    enum State {
        INTAKE_OUT_AND_IN, RESET_SERVOS, STOP_INTAKE, START_INTAKE, REVERSE_INTAKE,
        DROP_GRIPPERS_HALFWAY, DROP_GRIPPERS_FULLY, LIFT_GRIPPERS, GO_TO_STACK_POSITION,
        GO_TO_LIFT_POSITION
    }

    static PIDCoefficients liftPidCoefficients = new PIDCoefficients(k_p, k_i, k_d);
    PIDFController controller = new PIDFController(liftPidCoefficients, 0, 0,
            0, x -> k_G);
    private double target = 0;

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

        int stonePosition = 0;

        waitForStart();
        if (isStopRequested()) return;

        ElapsedTime timer = new ElapsedTime();
        HashMap<State, Double> stateTimes = new HashMap<>();  // map of state -> start time

        Trajectory trajectory = drive.trajectoryBuilder()
                .addMarker(() -> {
                    stateTimes.put(State.DROP_GRIPPERS_FULLY, null);
//                    stateTimes.put(State.GO_TO_STACK_POSITION, null);
                    return null;
                })
                .lineTo(new Vector2d(80, -35), new ConstantInterpolator(Math.toRadians(90)))
                .addMarker(() -> {
//                    stateTimes.put(State.GO_TO_LIFT_POSITION, null);
                    return null;
                })
                .lineTo(new Vector2d(56, -19), new SplineInterpolator(Math.toRadians(90), Math.toRadians(180)))
                .addMarker(() -> {
                    stateTimes.put(State.LIFT_GRIPPERS, null);
                    return null;
                })
                .lineTo(new Vector2d(76, -19), new ConstantInterpolator(Math.toRadians(180)))
//                .lineTo(new Vector2d(76, -29), new ConstantInterpolator(Math.toRadians(180)))
                .splineTo(new Pose2d(40, -29,Math.toRadians(180)))
                .build();

        drive.followTrajectory(trajectory);

        while (opModeIsActive()) {
            drive.update();

            double currentTime = timer.milliseconds();

            // set all start times if they don't exist
            for (State key : stateTimes.keySet()) {
                if (stateTimes.get(key) == null) {
                    stateTimes.put(key, currentTime);
                }
            }

            if (stateTimes.containsKey(State.INTAKE_OUT_AND_IN)) {
                // intake out and in case stone gets stuck
                Double startTime = stateTimes.get(State.INTAKE_OUT_AND_IN);
                double elapsedTime = 0;
                if (startTime != null) {
                    elapsedTime = currentTime - startTime;
                }

                telemetry.addData("INTAKE_OUT_AND_IN", elapsedTime);

                if (elapsedTime < 500) {
                    robot.left_intake.setPower(-0.2);
                    robot.right_intake.setPower(-0.2);
                } else if (elapsedTime < 1300) {
                    robot.left_intake.setPower(0.6);
                    robot.right_intake.setPower(0.6);
                } else {
                    robot.left_intake.setPower(0);
                    robot.right_intake.setPower(0);
                    stateTimes.remove(State.INTAKE_OUT_AND_IN);
                }
            }

            // start/stop/reverse intake
            if (stateTimes.containsKey(State.START_INTAKE)) {
                robot.left_intake.setPower(0.6);
                robot.right_intake.setPower(0.6);
                telemetry.addData("START_INTAKE", 0);
                stateTimes.remove(State.START_INTAKE);
            } else if (stateTimes.containsKey(State.STOP_INTAKE)) {
                robot.left_intake.setPower(0);
                robot.right_intake.setPower(0);
                telemetry.addData("STOP_INTAKE", 0);
                stateTimes.remove(State.STOP_INTAKE);
            } else if (stateTimes.containsKey(State.REVERSE_INTAKE)) {
                robot.left_intake.setPower(-0.6);
                robot.right_intake.setPower(-0.6);
                telemetry.addData("REVERSE_INTAKE", 0);
                stateTimes.remove(State.REVERSE_INTAKE);
            }

            // reset servos
            if (stateTimes.containsKey(State.RESET_SERVOS)) {
                // initialize servos
                Double startTime = stateTimes.get(State.RESET_SERVOS);
                double elapsedTime = 0;
                if (startTime != null) {
                    elapsedTime = currentTime - startTime;
                }

                telemetry.addData("RESET_SERVOS", elapsedTime);

                if (elapsedTime < 500) {
                    robot.left_v4b.setPosition(0.6);
                    robot.right_v4b.setPosition(0.6);
                    robot.push_servo.setPosition(0.35);
                    robot.gripper_servo.setPosition(1);
                    robot.foundation_right.setPower(-1);
                } else if (elapsedTime < 1000) {
                    robot.left_intake.setPower(0.8);
                    robot.right_intake.setPower(0.8);
                    robot.foundation_right.setPower(0);
                } else {
                    // start intake
                    robot.left_intake.setPower(0.6);
                    robot.right_intake.setPower(0.6);
                    stateTimes.remove(State.RESET_SERVOS);
                }
            }

            // drop grippers
            if (stateTimes.containsKey(State.DROP_GRIPPERS_HALFWAY)) {
                // initialize servos
                Double startTime = stateTimes.get(State.DROP_GRIPPERS_HALFWAY);
                double elapsedTime = 0;
                if (startTime != null) {
                    elapsedTime = timer.milliseconds() - startTime;
                }

                if (elapsedTime < 250) {
                    robot.foundation_left.setPosition(0.5);
                    robot.foundation_right.setPower(0.5);
                } else {
                    telemetry.addLine("STOPPING");
                    robot.foundation_right.setPower(0);
                    stateTimes.remove(State.DROP_GRIPPERS_HALFWAY);
                }
            }

            // drop grippers fully
            if (stateTimes.containsKey(State.DROP_GRIPPERS_FULLY)) {
                robot.foundation_left.setPosition(1);
                robot.foundation_right.setPower(0.5);
                stateTimes.remove(State.DROP_GRIPPERS_FULLY);
            }

            if (stateTimes.containsKey(State.LIFT_GRIPPERS)) {
                // initialize servos
                Double startTime = stateTimes.get(State.LIFT_GRIPPERS);
                double elapsedTime = 0;
                if (startTime != null) {
                    elapsedTime = timer.milliseconds() - startTime;
                }

                if (elapsedTime < 1000) {
                    robot.foundation_left.setPosition(0);
                    robot.foundation_right.setPower(-0.5);
                } else {
                    robot.foundation_right.setPower(0);
                    stateTimes.remove(State.LIFT_GRIPPERS);
                }
            }

            if (stateTimes.containsKey(State.GO_TO_STACK_POSITION)) {
                // initialize servos
                Double startTime = stateTimes.get(State.GO_TO_STACK_POSITION);
                double elapsedTime = 0;
                if (startTime != null) {
                    elapsedTime = timer.milliseconds() - startTime;
                }

                if (elapsedTime < 300) {
                    robot.push_servo.setPosition(0.35);
                    robot.gripper_servo.setPosition(1);
                } else if (elapsedTime < 600) {
                    robot.push_servo.setPosition(1);
                } else if (elapsedTime < 900) {
                    robot.left_v4b.setPosition(0.75);
                    robot.right_v4b.setPosition(0.75);
                } else if (elapsedTime < 1200) {
                    robot.gripper_servo.setPosition(0.6);
                } else {
                    stateTimes.remove(State.GO_TO_STACK_POSITION);
                }
            }

            if (stateTimes.containsKey(State.GO_TO_LIFT_POSITION)) {
                // initialize servos
                Double startTime = stateTimes.get(State.GO_TO_LIFT_POSITION);
                double elapsedTime = 0;
                if (startTime != null) {
                    elapsedTime = timer.milliseconds() - startTime;
                }

                if (elapsedTime < 300) {
                    // get pusher out of the way
                    robot.push_servo.setPosition(0.35);
                } else if (elapsedTime < 500){
                    target = -100;
                } else if (Math.abs(robot.lift_left.getCurrentPosition() - target) < 5) {
                    stateTimes.remove(State.GO_TO_LIFT_POSITION);
                }
            }

            // use PID to hold lift position
            controller.setTargetPosition(target);
            double correction = controller.update(robot.lift_left.getCurrentPosition());
            robot.lift_left.setPower(correction);
            robot.lift_right.setPower(correction);

            telemetry.addData("stateTimes", stateTimes);
            telemetry.update();
        }
    }
}

