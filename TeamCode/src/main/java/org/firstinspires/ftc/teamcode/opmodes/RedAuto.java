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

import org.firstinspires.ftc.teamcode.NewStoneDetector;
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
@Autonomous(group = "drive", name = "Red Auto")
public class RedAuto extends LinearOpMode {
    private OpenCvInternalCamera phoneCam;
    private NewStoneDetector skyStoneDetector;

    HardwareDrivetrain robot = new HardwareDrivetrain();

    enum State {
        INTAKE_OUT_AND_IN, RESET_SERVOS, STOP_INTAKE, START_INTAKE, REVERSE_INTAKE,
        DROP_GRIPPERS_HALFWAY, DROP_GRIPPERS_FULLY, LIFT_GRIPPERS, GO_TO_STACK_POSITION,
        GO_TO_LIFT_POSITION, DROP_BLOCK
    }
    @Override
    public void runOpMode() throws InterruptedException {
        // starts on blue side
        VertexDrive drive = new VertexDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(180)));
        robot.init(hardwareMap);

        // reset lift encoders
        robot.lift_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.lift_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lift_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.push_servo.setPosition(0);
        robot.gripper_servo.setPosition(1);
        robot.foundation_right.setPosition(0);
        robot.foundation_left.setPosition(0);

        PIDCoefficients liftPidCoefficients = new PIDCoefficients(k_p, k_i, k_d);
        PIDFController controller = new PIDFController(liftPidCoefficients, 0, 0,
                0, x -> k_G);
        double target = 0;  // target lift position

        // TODO detect stone position here
        // 142 (front) 85 (center) 30 (back)

        // setup camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        skyStoneDetector = new NewStoneDetector(true);
        phoneCam.setPipeline(skyStoneDetector);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        double xPosition;
        int stonePosition = 0;  // 0: front, 1: center, 2: back

        boolean exposureLocked = false;
        boolean xPressed = true;
        int exposureCompensation = 0;

        while(!isStarted()) {
            if (gamepad2.x) {
                if (!xPressed)
                    exposureLocked = !exposureLocked;
                xPressed = true;
            } else {
                xPressed = false;
            }

            if (!exposureLocked) {
                if (gamepad2.a) {
                    exposureCompensation = Math.max(exposureCompensation - 1, phoneCam.getMinSupportedExposureCompensation());
                } else if (gamepad2.b) {
                    exposureCompensation = Math.min(exposureCompensation + 1, phoneCam.getMaxSupportedExposureCompensation());
                }
            } else {
                telemetry.addLine("EXPOSURE LOCKED");
            }
            phoneCam.setExposureCompensation(exposureCompensation);
            phoneCam.setExposureLocked(exposureLocked);  // TODO check if this works

            telemetry.addData("EXPOSURE COMPENSATION", exposureCompensation);

            stonePosition = skyStoneDetector.position;
            if (stonePosition == 0) {
                telemetry.addData("position", "front");
            } else if (stonePosition == 1) {
                telemetry.addData("position", "center");
            } else if (stonePosition == 2) {
                telemetry.addData("position", "back");
            }
            telemetry.update();
        }

        phoneCam.pauseViewport();
        phoneCam.stopStreaming();

        ElapsedTime timer = new ElapsedTime();
        HashMap<State, Double> stateTimes = new HashMap<>();  // map of state -> start time

        Trajectory trajectory = null;

//        drive.setPoseEstimate(new Pose2d(84, -28, Math.toRadians(90)));

        if (stonePosition == 0) {
            telemetry.addLine("FRONT"); telemetry.update();
            trajectory = drive.trajectoryBuilder()
                    .addMarker(() -> {
                        stateTimes.put(State.RESET_SERVOS, null);
                        return null;
                    })
                    .lineTo(new Vector2d(0, 6), new ConstantInterpolator(Math.toRadians(180)))
                    .lineTo(new Vector2d(-8, 30), new LinearInterpolator(Math.toRadians(180), Math.toRadians(-135)))
                    .lineTo(new Vector2d(0, 38), new ConstantInterpolator(Math.toRadians(45)))
                    .lineTo(new Vector2d(12, 26), new LinearInterpolator(Math.toRadians(45), Math.toRadians(135)))
                    .addMarker(() -> {
                        stateTimes.put(State.INTAKE_OUT_AND_IN, null);
                        stateTimes.put(State.DROP_GRIPPERS_HALFWAY, null);
                        return null;
                    })
                    .addMarker(new Vector2d(30, 26), () -> {
                        stateTimes.put(State.INTAKE_OUT_AND_IN, null);
                        return null;
                    })
                    // from here it is the same for all three positions
                    .lineTo(new Vector2d(70, 26), new ConstantInterpolator(Math.toRadians(-180)))
                    .addMarker(() -> {
                        stateTimes.put(State.STOP_INTAKE, null);
                        return null;
                    })
                    .lineTo(new Vector2d(94, 28), new ConstantInterpolator(Math.toRadians(-90)))
                    .addMarker(new Vector2d(94, 32), () -> {
                        stateTimes.put(State.DROP_GRIPPERS_FULLY, null);
                        return null; // go to stack pos
                    })
                    .lineTo(new Vector2d(94, 34), new ConstantInterpolator(Math.toRadians(-90)))
                    .build();
        } else if (stonePosition == 1) {
            telemetry.addLine("CENTER");
            telemetry.update();
            trajectory = drive.trajectoryBuilder()
                    .addMarker(() -> {
                        stateTimes.put(State.RESET_SERVOS, null);
                        return null;
                    })
                    .lineTo(new Vector2d(0, 6), new ConstantInterpolator(Math.toRadians(180)))
                    .lineTo(new Vector2d(8, 34), new SplineInterpolator(Math.toRadians(180), Math.toRadians(135)))
                    .lineTo(new Vector2d(2, 28), new ConstantInterpolator(Math.toRadians(135)))
                    .lineTo(new Vector2d(12, 26), new SplineInterpolator(Math.toRadians(135), Math.toRadians(180)))
                    .addMarker(new Vector2d(30, 26), () -> {
                        stateTimes.put(State.INTAKE_OUT_AND_IN, null);
                        return null;
                    })
                    // from here it is the same for all three positions
                    .lineTo(new Vector2d(82, 26), new ConstantInterpolator(Math.toRadians(-180)))
                    .addMarker(() -> {
                        stateTimes.put(State.STOP_INTAKE, null);
                        return null;
                    })
                    .lineTo(new Vector2d(94, 28), new ConstantInterpolator(Math.toRadians(-90)))
                    .addMarker(new Vector2d(94, 32), () -> {
                        stateTimes.put(State.DROP_GRIPPERS_FULLY, null);
                        return null; // go to stack pos
                    })
                    .lineTo(new Vector2d(94, 34), new ConstantInterpolator(Math.toRadians(-90)))
                    .build();
        } else {
            telemetry.addLine("BACK"); telemetry.update();
            trajectory = drive.trajectoryBuilder()
                    .addMarker(() -> {
                        stateTimes.put(State.RESET_SERVOS, null);
                        return null;
                    })
                    .lineTo(new Vector2d(0, 6), new ConstantInterpolator(Math.toRadians(180)))
                    .lineTo(new Vector2d(0, 32), new SplineInterpolator(Math.toRadians(180), Math.toRadians(135)))
                    .lineTo(new Vector2d(-2, 34), new ConstantInterpolator(Math.toRadians(135)))
                    .lineTo(new Vector2d(12, 26), new SplineInterpolator(Math.toRadians(135), Math.toRadians(180)))
                    .addMarker(new Vector2d(30, 26), () -> {
                        stateTimes.put(State.INTAKE_OUT_AND_IN, null);
                        return null;
                    })
                    // from here it is the same for all three positions
                    .lineTo(new Vector2d(70, 26), new ConstantInterpolator(Math.toRadians(-180)))
                    .addMarker(() -> {
                        stateTimes.put(State.STOP_INTAKE, null);
                        return null;
                    })
                    .lineTo(new Vector2d(94, 28), new ConstantInterpolator(Math.toRadians(-90)))
                    .addMarker(new Vector2d(94, 32), () -> {
                        stateTimes.put(State.DROP_GRIPPERS_FULLY, null);
                        return null; // go to stack pos
                    })
                    .lineTo(new Vector2d(94, 34), new ConstantInterpolator(Math.toRadians(-90)))
                    .build();        }

        drive.followTrajectory(trajectory);
//        robot.foundation_right.setPosition(0.2);
//        robot.foundation_left.setPosition(0.46);
//        sleep(500);
//        stateTimes.put(State.INTAKE_OUT_AND_IN, null);

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
                } else if (elapsedTime < 2000) {
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
                    robot.push_servo.setPosition(0);
                    robot.gripper_servo.setPosition(1);
                    robot.foundation_right.setPosition(0.2);
                    robot.foundation_left.setPosition(0.46);
                } else if (elapsedTime < 1000) {
                    robot.left_intake.setPower(0.8);
                    robot.right_intake.setPower(0.8);
                } else {
                    // start intake
                    robot.left_intake.setPower(0.6);
                    robot.right_intake.setPower(0.6);
                    robot.push_servo.setPosition(0.35);
                    stateTimes.remove(State.RESET_SERVOS);
                }
            }

            // drop grippers fully
            if (stateTimes.containsKey(State.DROP_GRIPPERS_FULLY)) {
                robot.foundation_left.setPosition(1);
                robot.foundation_right.setPosition(1);
                drive.setMotorPowers(0,0,0,0);
                sleep(1000);
                stateTimes.remove(State.DROP_GRIPPERS_FULLY);

                Trajectory trajectory2 = drive.trajectoryBuilder()
                        .addMarker(() -> {
                            stateTimes.put(State.GO_TO_STACK_POSITION, null);
                            return null;
                        })
                        .lineTo(new Vector2d(52, 16), new LinearInterpolator(Math.toRadians(-90), Math.toRadians(-90)))
                        .addMarker(() -> {
                            stateTimes.put(State.LIFT_GRIPPERS, null);
                            return null;
                        })
                        .lineTo(new Vector2d(50, 16), new ConstantInterpolator(Math.toRadians(180)))
                        .addMarker(new Vector2d(70, 16), () -> {
                            stateTimes.put(State.GO_TO_LIFT_POSITION, null);
                            return null;
                        })
                        .lineTo(new Vector2d(72, 16), new ConstantInterpolator(Math.toRadians(180)))
                        .build();
                drive.followTrajectory(trajectory2);
            }

            if (stateTimes.containsKey(State.LIFT_GRIPPERS)) {
                robot.foundation_right.setPosition(0.2);
                robot.foundation_left.setPosition(0.46);
                stateTimes.remove(State.LIFT_GRIPPERS);
            }

            if (stateTimes.containsKey(State.GO_TO_STACK_POSITION)) {
                // initialize servos
                Double startTime = stateTimes.get(State.GO_TO_STACK_POSITION);
                double elapsedTime = 0;
                if (startTime != null) {
                    elapsedTime = timer.milliseconds() - startTime;
                }

                if (elapsedTime < 300) {
                    robot.push_servo.setPosition(1);
                } else if (elapsedTime < 600) {
                    robot.push_servo.setPosition(0.35);
                    robot.gripper_servo.setPosition(1);
                } else if (elapsedTime < 900) {
                    robot.push_servo.setPosition(1);
                } else if (elapsedTime < 1200) {
                    robot.left_v4b.setPosition(0.75);
                    robot.right_v4b.setPosition(0.75);
                } else if (elapsedTime < 1500) {
                    robot.gripper_servo.setPosition(0.4);
                } else {
                    stateTimes.remove(State.GO_TO_STACK_POSITION);
//                    stateTimes.put(State.GO_TO_LIFT_POSITION, null);
                }
            }

            if (stateTimes.containsKey(State.DROP_BLOCK)) {
                // initialize servos
                Double startTime = stateTimes.get(State.DROP_BLOCK);
                double elapsedTime = 0;
                if (startTime != null) {
                    elapsedTime = timer.milliseconds() - startTime;
                }

                if (elapsedTime < 300) {
                    robot.left_v4b.setPosition(0);
                    robot.right_v4b.setPosition(0);
                } else if (elapsedTime < 700) {
                    robot.gripper_servo.setPosition(1);
                } else if (elapsedTime < 1000) {
                    // reset v4b's to grab position
                    robot.left_v4b.setPosition(0.75);
                    robot.right_v4b.setPosition(0.75);
                } else if (elapsedTime < 1500) {
                    target = 0;
                } else if (elapsedTime < 2000) {
                    // reset v4b's to wait position
                    robot.left_v4b.setPosition(0.6);
                    robot.right_v4b.setPosition(0.6);
                } else {
                    stateTimes.remove(State.DROP_BLOCK);
                    Trajectory trajectory2 = drive.trajectoryBuilder()
                        .lineTo(new Vector2d(76, 26), new ConstantInterpolator(Math.toRadians(180)))
                        .build();
                    drive.followTrajectorySync(trajectory2);
                    Trajectory trajectory3 = drive.trajectoryBuilder()
                            .lineTo(new Vector2d(32, 26), new ConstantInterpolator(Math.toRadians(180)))
                            .build();
                    drive.followTrajectorySync(trajectory3);
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
                    robot.gripper_servo.setPosition(0.4);
                    robot.push_servo.setPosition(0.35);
                } else if (elapsedTime < 500){
                    target = -100;
                } else if (Math.abs(robot.lift_left.getCurrentPosition() - target) < 5) {
                    stateTimes.remove(State.GO_TO_LIFT_POSITION);
                    stateTimes.put(State.DROP_BLOCK, null);
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

