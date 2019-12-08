package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.HardwareDrivetrain;
import org.firstinspires.ftc.teamcode.hardware.HardwareNames;
import org.firstinspires.ftc.teamcode.math.MathFunctions;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalCoordinatePosition;

import static org.firstinspires.ftc.teamcode.opmodes.LiftPIDTest.k_d;
import static org.firstinspires.ftc.teamcode.opmodes.LiftPIDTest.k_i;
import static org.firstinspires.ftc.teamcode.opmodes.LiftPIDTest.k_p;
import static org.firstinspires.ftc.teamcode.opmodes.LiftPIDTest.k_G;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwareTwoMotors class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Tele-op", group="Competition")
public class TeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareDrivetrain robot = new HardwareDrivetrain();
    private double THRESHOLD = 0.05;
    ElapsedTime accel = new ElapsedTime();
    ElapsedTime rotate_accel = new ElapsedTime();
    ElapsedTime strafe_accel = new ElapsedTime();
    private double millisecondsToFullSpeed = 600;
    private double speedAdjust = 0;

    private double leftBackSpeed, rightBackSpeed, leftFrontSpeed, rightFrontSpeed, intakeSpeed;
    private int desiredLiftPosition = 0;
    private double manualLiftPower = 0;
    private OdometryGlobalCoordinatePosition globalPositionUpdate;

    private int drivetrainSpeedAdjust = 5;
    ElapsedTime left_trigger_time = null;
    ElapsedTime right_trigger_time = null;
    boolean start_button_pressed = false;
    private double DPAD_SPEED = 0.35;
    private double BUMPER_ROTATION_SPEED = 0.4;
    private double FCD_ROTATION_SPEED = 0.8;
    ElapsedTime dpad_accel = new ElapsedTime();
    ElapsedTime bumper_rotate_accel = new ElapsedTime();

    private String lastIntakeButton = "x";

    ElapsedTime stack_routine_time = null;
    ElapsedTime drop_routine_time = null;
    ElapsedTime lift_routine_time = null;
    private double target = 0;

    static PIDCoefficients liftPidCoefficients = new PIDCoefficients(k_p, k_i, k_d);
    PIDFController controller = new PIDFController(liftPidCoefficients, 0, 0,
            0, x -> k_G);

    private String mode = "MODE_TANK";
    private boolean stickControllingPusher = false;
    private boolean gamepad2XYPressed = false;

    @Override
    public void runOpMode() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // reset lift encoders
        robot.lift_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.lift_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lift_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addLine("Ready");
        telemetry.update();

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight,
                robot.horizontal, HardwareNames.COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        // initialize servos
        robot.left_v4b.setPosition(0.6);
        robot.right_v4b.setPosition(0.6);
        robot.push_servo.setPosition(0.35);
        robot.gripper_servo.setPosition(1);

        // lift PID controller
        controller.setTargetPosition(0);
        controller.setOutputBounds(-1, 1);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // mode switch
            if (!start_button_pressed && gamepad1.start) {
                start_button_pressed = true;
                if (mode.equals("MODE_FCD")) {
                    mode = "MODE_TANK";
                } else if (mode.equals("MODE_TANK")) {
                    mode = "MODE_FCD";
                }
            }
            if (!gamepad1.start) start_button_pressed = false;

            // initialize all speeds to 0
            leftBackSpeed = leftFrontSpeed = rightBackSpeed = rightFrontSpeed = 0;

            // speed adjust
            // decrease speed
            if (gamepad1.left_trigger > 0.2) {
                drivetrainSpeedAdjust = 3;
            } else {
                drivetrainSpeedAdjust = 5;
            }
//
//            // increase speed
//            if (gamepad1.right_trigger > 0.3) {
//                if (right_trigger_time == null) {
//                    right_trigger_time = new ElapsedTime();
//                } else if (right_trigger_time.milliseconds() > 500) {
//                    drivetrainSpeedAdjust = 5;
//                }
//            } else {
//                if (right_trigger_time != null) {
//                    drivetrainSpeedAdjust = Math.min(5, drivetrainSpeedAdjust + 1);
//                    right_trigger_time = null;
//                }
//            }

            telemetry.addData("speed", drivetrainSpeedAdjust * 20);

            // rotate using bumpers
            if (gamepad1.left_bumper || gamepad1.right_bumper) {
                double bumperRotationSpeed = Math.min(BUMPER_ROTATION_SPEED,
                        bumper_rotate_accel.milliseconds() / millisecondsToFullSpeed);

                // make bumper rotation speed independent of drivetrain speed adjust
                bumperRotationSpeed = bumperRotationSpeed / drivetrainSpeedAdjust * 5;

                if (gamepad1.left_bumper) {
                    telemetry.addLine("left bumper");
                    leftBackSpeed = leftFrontSpeed = -bumperRotationSpeed;
                    rightBackSpeed = rightFrontSpeed = bumperRotationSpeed;
                } else if (gamepad1.right_bumper) {
                    telemetry.addLine("right bumper");
                    leftBackSpeed = leftFrontSpeed = bumperRotationSpeed;
                    rightBackSpeed = rightFrontSpeed = -bumperRotationSpeed;
                }

            } else {
                bumper_rotate_accel.reset();
            }


            if (mode.equals("MODE_FCD")) {
                // field centric driving (right joystick)
                if (Math.abs(gamepad1.right_stick_y) > THRESHOLD || Math.abs(gamepad1.right_stick_x) > THRESHOLD) {
                    speedAdjust = Math.min(1, accel.milliseconds() / millisecondsToFullSpeed);

                    double[] maxSpeeds = getMaxSpeeds(gamepad1.right_stick_x, gamepad1.right_stick_y,
                            globalPositionUpdate.returnOrientation());

                    rightFrontSpeed = speedAdjust * maxSpeeds[0];
                    leftFrontSpeed  = speedAdjust * maxSpeeds[1];
                    rightBackSpeed  = speedAdjust * maxSpeeds[2];
                    leftBackSpeed   = speedAdjust * maxSpeeds[3];

                } else {
                    accel.reset();
                }

                // rotating (left joystick)
                if (Math.abs(gamepad1.left_stick_x) > THRESHOLD) {
                    double rotationSpeed = Math.min(Math.abs(FCD_ROTATION_SPEED * gamepad1.left_stick_x),
                            rotate_accel.milliseconds() / millisecondsToFullSpeed) * Math.signum(gamepad1.left_stick_x);

                    leftFrontSpeed += rotationSpeed;
                    leftBackSpeed += leftFrontSpeed;
                    rightBackSpeed -= leftFrontSpeed;
                    rightFrontSpeed -= rightBackSpeed;
                } else {
                    rotate_accel.reset();
                }

                // scale all speeds so the max is 1, if needed
                double maxSpeed = MathFunctions.absMax(leftBackSpeed, leftFrontSpeed, rightBackSpeed, rightFrontSpeed);
                if (maxSpeed > 1) {
                    leftBackSpeed /= maxSpeed;
                    rightBackSpeed /= maxSpeed;
                    leftFrontSpeed /= maxSpeed;
                    rightFrontSpeed /= maxSpeed;
                }

                // multiply by speed adjust
                leftFrontSpeed *= drivetrainSpeedAdjust / 5.0;
                rightFrontSpeed *= drivetrainSpeedAdjust / 5.0;
                leftBackSpeed *= drivetrainSpeedAdjust / 5.0;
                rightBackSpeed *= drivetrainSpeedAdjust / 5.0;


            } else if (mode.equals("MODE_TANK")) {
                // left - left joystick
                // right - right joystick
                leftFrontSpeed = leftBackSpeed = -drivetrainSpeedAdjust * gamepad1.left_stick_y / 5;
                rightFrontSpeed = rightBackSpeed = -drivetrainSpeedAdjust * gamepad1.right_stick_y / 5;

                // strafe
                if (gamepad1.left_bumper || gamepad1.right_bumper) {
//                    double strafeSpeed = Math.min(drivetrainSpeedAdjust / 5.0, strafe_accel.milliseconds() / millisecondsToFullSpeed);
                    double strafeSpeed = drivetrainSpeedAdjust / 5.0;

                    if (gamepad1.left_bumper) {
                        leftFrontSpeed = rightBackSpeed = -strafeSpeed;
                        leftBackSpeed = rightFrontSpeed = strafeSpeed;
                    } else if (gamepad1.right_bumper) {
                        leftFrontSpeed = rightBackSpeed = strafeSpeed;
                        leftBackSpeed = rightFrontSpeed = -strafeSpeed;
                    }

                } else {
                    strafe_accel.reset();
                }
            }

            // dpad drive controls
            if (gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_left || gamepad1.dpad_down) {
                double dpadSpeed = Math.min(DPAD_SPEED, dpad_accel.milliseconds() / millisecondsToFullSpeed);

                if (gamepad1.dpad_up) {
                    leftFrontSpeed = leftBackSpeed = rightFrontSpeed = rightBackSpeed = dpadSpeed;
                } else if (gamepad1.dpad_down) {
                    leftFrontSpeed = leftBackSpeed = rightFrontSpeed = rightBackSpeed = -dpadSpeed;
                } else if (gamepad1.dpad_left) {
                    leftFrontSpeed = rightBackSpeed = -dpadSpeed;
                    leftBackSpeed = rightFrontSpeed = dpadSpeed;
                } else if (gamepad1.dpad_right) {
                    leftFrontSpeed = rightBackSpeed = dpadSpeed;
                    leftBackSpeed = rightFrontSpeed = -dpadSpeed;
                }

            } else {
                dpad_accel.reset();
            }

            // intake/outtake
            if (gamepad1.a || gamepad2.a) {
                // intake
                intakeSpeed = 0.6;
                lastIntakeButton = "on";
            } else if (gamepad1.b || gamepad2.b) {
                // outtake
                intakeSpeed = -0.3;
                lastIntakeButton = "temp";
            } else if (false) {
                // intake fast
                intakeSpeed = 1;
                lastIntakeButton = "temp";
            } else if (gamepad1.x || lastIntakeButton.equals("temp")) {
                // stop intake
                intakeSpeed = 0;
                lastIntakeButton = "stop";
            }

            // reset orientation
            if (gamepad1.y) {
                globalPositionUpdate.setOrientation(0);
            }

            // gamepad 2

            // manually adjust lift
            // TODO change to modify a target position rather than directly set powers
            // TODO except when the position is close to 0, then automatically flush with the robot
            if (gamepad2.left_bumper) {
                manualLiftPower = 0.2;
            } else if (gamepad2.right_bumper) {
                manualLiftPower = -0.2;
            } else {
                manualLiftPower = 0;
            }

            // set desired position (x to decrease position, y to increase)
            if (gamepad2.y) {
                if (!gamepad2XYPressed) {
                    desiredLiftPosition++;
                }
                gamepad2XYPressed = true;
            } else if (gamepad2.x) {
                if (!gamepad2XYPressed) {
                    desiredLiftPosition = Math.max(0, desiredLiftPosition - 1);
                }
                gamepad2XYPressed = true;
            } else {
                gamepad2XYPressed = false;
            }


            // go to stacking position routine
            if (stack_routine_time == null && gamepad2.dpad_right) {
                stack_routine_time = new ElapsedTime();
            }
            if (stack_routine_time != null) {
                if (stack_routine_time.milliseconds() < 500) {
                    robot.push_servo.setPosition(0.35);
                    robot.gripper_servo.setPosition(1);
                } else if (stack_routine_time.milliseconds() < 1000) {
                    robot.push_servo.setPosition(1);
                } else if (stack_routine_time.milliseconds() < 1600) {
                    robot.left_v4b.setPosition(0.75);
                    robot.right_v4b.setPosition(0.75);
                } else if (stack_routine_time.milliseconds() < 2300) {
                    robot.gripper_servo.setPosition(0.6);
                    intakeSpeed = 0;
                    lastIntakeButton = "stop";
                }  else {
                    stack_routine_time = null;
                }
            }

            // lift + flip routine
            if (lift_routine_time == null && gamepad2.dpad_up) {
                lift_routine_time = new ElapsedTime();
            }
            if (lift_routine_time != null) {
                if (lift_routine_time.milliseconds() < 500) {
                    // get pusher out of the way
                    robot.push_servo.setPosition(0.35);
                } else if (lift_routine_time.milliseconds() < 600){
                    target = -50 + -155 * desiredLiftPosition;
                    controller.setTargetPosition(target);
                } else if (Math.abs(robot.lift_left.getCurrentPosition() - target) > 5) {
                    // we're still far away from the target
                    // do nothing
                } else {
                    // extend v4b servos to drop position
                    robot.left_v4b.setPosition(0);
                    robot.right_v4b.setPosition(0);
                    desiredLiftPosition++;
                    lift_routine_time = null;
                }
            }

            // drop routine
            if (drop_routine_time == null && gamepad2.dpad_down) {
                drop_routine_time = new ElapsedTime();
            }
            if (drop_routine_time != null) {
                if (drop_routine_time.milliseconds() < 400) {
                    // release the gripper
                    robot.gripper_servo.setPosition(1);
                } else if (drop_routine_time.milliseconds() < 800) {
                    // raise the lift
                    controller.setTargetPosition(-20 + robot.lift_left.getCurrentPosition());
                } else if (drop_routine_time.milliseconds() < 1200) {
                    // reset v4b's to grab position
                    robot.left_v4b.setPosition(0.75);
                    robot.right_v4b.setPosition(0.75);
                } else if (drop_routine_time.milliseconds() < 1600) {
                    // lower the lift
                    controller.setTargetPosition(0);
                } else if (drop_routine_time.milliseconds() < 2000) {
                    // reset v4b's to wait position
                    robot.left_v4b.setPosition(0.6);
                    robot.right_v4b.setPosition(0.6);
                } else {
                    drop_routine_time = null;
                }
            }

//            // drop routine routine
//
//            if (drop_routine_time == null && gamepad2.dpad_down) {
//                drop_routine_time = new ElapsedTime();
//            }
//            if (drop_routine_time != null) {
//                if (drop_routine_time.milliseconds() < 500) {
//                    robot.gripper_servo.setPosition(1);
//                } else if (drop_routine_time.milliseconds() < 1000) {
//                    robot.left_v4b.setPosition(0.5);
//                    robot.right_v4b.setPosition(0.5);
//                } else if (drop_routine_time.milliseconds() < 1500) {
//                    robot.push_servo.setPosition(0.3);
//                } else if (drop_routine_time.milliseconds() < 2000) {
//                    robot.left_v4b.setPosition(0.6);
//                    robot.right_v4b.setPosition(0.6);
//                    intakeSpeed = 0.6;
//                    lastIntakeButton = "on";
//                } else {
//                    drop_routine_time = null;
//                }
//            }

//            // gf drop routine
//            if (gf_drop_routine_time == null && gamepad2.dpad_right) {
//                gf_drop_routine_time = new ElapsedTime();
//            }
//            if (gf_drop_routine_time != null) {
//                if (gf_drop_routine_time.milliseconds() < 300) {
//                    robot.push_servo.setPosition(0.35);
//                    robot.gripper_servo.setPosition(1);
//                } else if (gf_drop_routine_time.milliseconds() < 600) {
//                    robot.push_servo.setPosition(1);
//                } else if (gf_drop_routine_time.milliseconds() < 900) {
//                    robot.left_v4b.setPosition(0.75);
//                    robot.right_v4b.setPosition(0.75);
//                } else if (gf_drop_routine_time.milliseconds() < 1200) {
//                    robot.gripper_servo.setPosition(0.6);
//                } else if (gf_drop_routine_time.milliseconds() < 1500) {
//                    robot.left_v4b.setPosition(0);
//                    robot.right_v4b.setPosition(0);
//                } else if (gf_drop_routine_time.milliseconds() < 1800) {
//                    robot.gripper_servo.setPosition(1);
//                } else if (gf_drop_routine_time.milliseconds() < 2100) {
//                    robot.left_v4b.setPosition(0.5);
//                    robot.right_v4b.setPosition(0.5);
//                } else if (gf_drop_routine_time.milliseconds() < 3000) {
//                    robot.push_servo.setPosition(0.3);
//                } else if (gf_drop_routine_time.milliseconds() < 3500) {
//                    robot.left_v4b.setPosition(0.6);
//                    robot.right_v4b.setPosition(0.6);
//                    intakeSpeed = 0.6;
//                    lastIntakeButton = "a";
//                } else {
//                    gf_drop_routine_time = null;
//                }
//            }

            // left joystick - control pusher
            if (gamepad2.left_stick_y > 0.1) {
                double pusherPosition = 0.7 * gamepad2.left_stick_y + 0.3;
                robot.push_servo.setPosition(pusherPosition);
            }
            if (gamepad2.left_stick_y < -0.1) {
                double pusherPosition = 1- (0.7 * (-gamepad2.left_stick_y) + 0.3);
                robot.push_servo.setPosition(pusherPosition);
            }

            // set motor powers
            robot.left_back.setPower(leftBackSpeed);
            robot.left_front.setPower(leftFrontSpeed);
            robot.right_back.setPower(rightBackSpeed);
            robot.right_front.setPower(rightFrontSpeed);

            robot.right_intake.setPower(intakeSpeed);
            robot.left_intake.setPower(intakeSpeed);

            double lift_position = robot.lift_left.getCurrentPosition();

            if (manualLiftPower == 0) {
                // use PID to hold position
                double correction = controller.update(lift_position);

                robot.lift_left.setPower(correction);
                robot.lift_right.setPower(correction);
            } else {
                robot.lift_left.setPower(manualLiftPower);
                robot.lift_right.setPower(manualLiftPower);
                controller.setTargetPosition(lift_position);
            }

            // update telemetry
//            telemetry.addData("leftBack", leftBackSpeed);
//            telemetry.addData("rightBack", rightBackSpeed);
//            telemetry.addData("leftFront", leftFrontSpeed);
            telemetry.addData("current", robot.lift_left.getCurrentPosition());
            telemetry.addData("desired", (target));
            telemetry.addData("error", (Math.abs(robot.lift_left.getCurrentPosition() - target) < 5));
            telemetry.addData("desiredLiftPosition", desiredLiftPosition);
            telemetry.update();
        }
    }

    /**
     * Get maximum possible speeds that the robot can travel at for the specific orientation
     */
    private double[] getMaxSpeeds(double joystick_x, double joystick_y, double orientation) {
        // rf, lf, rb, lb
        double[] speeds = new double[4];
        // transform joystick to specific orientation
        double[] new_joystick_position = MathFunctions.rotatePointCounterClockwise(joystick_x,
                joystick_y, orientation);

        speeds[1] = new_joystick_position[0] - new_joystick_position[1];
        speeds[0] = -new_joystick_position[0] - new_joystick_position[1];


        double maxSpeed = Math.max(Math.abs(speeds[0]), Math.abs(speeds[1]));
        speeds[0] /= maxSpeed;
        speeds[1] /= maxSpeed;

        speeds[2] = speeds[1];
        speeds[3] = speeds[0];
        return speeds;
    }
}
