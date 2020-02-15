package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drive.mecanum.VertexDrive;
import org.firstinspires.ftc.teamcode.hardware.HardwareNames;
import org.firstinspires.ftc.teamcode.math.MathFunctions;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalCoordinatePosition;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TELE_OP_CONSTRAINTS;


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
    ServoController robot = new ServoController();
    LiftController lift = null;

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
//    private double DPAD_SPEED = 0.35;
    private double BUMPER_ROTATION_SPEED = 0.4;
    private double FCD_ROTATION_SPEED = 0.8;
    ElapsedTime dpad_accel = new ElapsedTime();
    ElapsedTime bumper_rotate_accel = new ElapsedTime();

    private String lastIntakeButton = "x";

    private ElapsedTime stack_routine_time = null;
    private ElapsedTime drop_routine_time = null;
    private ElapsedTime drop_routine_2_time = null;
    private ElapsedTime lift_routine_time = null;
    private ElapsedTime foundation_gripper_routine_time = null;
    private ElapsedTime last_time = null;
    private double foundationGripperSpeed = 0;
    private double target = 0;

    private boolean gripperIsDown = false;
    private boolean stickControllingPusher = false;
    private boolean leftTriggerPressed;
    private boolean rightTriggerPressed = false;
    private boolean gamepad2XYPressed = false;
    private boolean gamepad1YPressed = false;
    private boolean gamepad2RightButtonPressed = false;
    private boolean capPosition = false;
    private boolean blue = false;

    ElapsedTime lagTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        VertexDrive drive = new VertexDrive(hardwareMap);

        robot.init(hardwareMap);
        lift = new LiftController(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addLine("Ready");
        telemetry.update();

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight,
                robot.horizontal, HardwareNames.COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        // initialize servos
        double lastGripperPosition = 1;

        robot.v4bWait();
        robot.pushServoUp();
        robot.gripper_servo.setPosition(lastGripperPosition);
        robot.foundationUp();
        robot.parkOff();

        // save last auto in file
        String fname = AppUtil.ROOT_FOLDER + "/lastAuto.txt";
        try {
            BufferedReader br = new BufferedReader(new FileReader(fname));
            String line = br.readLine();
            if (line.startsWith("Blue")) {
                blue = true;
            }
            br.close();
        } catch (IOException exception) {

        }

        if (blue) {
            drive.setPoseEstimate(new Pose2d(11.5, -26.6, Math.toRadians(180)));
        } else {
            drive.setPoseEstimate(new Pose2d(11.5, 26.6, Math.toRadians(180)));
        }

        // extra vars
        boolean goneUp = false;
        boolean autoDriving = false;
        boolean gamepad1LeftStickButtonPressed = false;
        boolean gamepad1RightStickButtonPressed = false;
        boolean motionSignal = false;

        // set drive contraints
        drive.setConstraints(TELE_OP_CONSTRAINTS);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            motionSignal = false;
            lift.update();
            telemetry.addData("lag (ms)", lagTimer.milliseconds());
            lagTimer.reset();

            // initialize all speeds to 0
            leftBackSpeed = leftFrontSpeed = rightBackSpeed = rightFrontSpeed = 0;

            // speed adjust
            // decrease speed
            if (gamepad1.left_trigger > 0.2) {
                // change on new click
                if (!leftTriggerPressed) {
                    if (drivetrainSpeedAdjust != 2) {
                        drivetrainSpeedAdjust = 2;
                    } else {
                        drivetrainSpeedAdjust = 5;
                    }
                }
                leftTriggerPressed = true;
            } else {
                leftTriggerPressed = false;
            }

            if (gamepad1.right_trigger > 0.2) {
                // change on new click
                if (!rightTriggerPressed) {
                    if (drivetrainSpeedAdjust != 3) {
                        drivetrainSpeedAdjust = 3;
                    } else {
                        drivetrainSpeedAdjust = 5;
                    }
                }
                rightTriggerPressed = true;
            } else {
                rightTriggerPressed = false;
            }

            telemetry.addData("speed", drivetrainSpeedAdjust * 20);


            // tank driving

            // left - left joystick
            // right - right joystick
//            leftFrontSpeed = leftBackSpeed = -drivetrainSpeedAdjust * gamepad1.left_stick_y / 5;
//            rightFrontSpeed = rightBackSpeed = -drivetrainSpeedAdjust * gamepad1.right_stick_y / 5;

            if (Math.abs(gamepad1.left_stick_x) > 0.05 || (Math.abs(gamepad1.left_stick_y) > 0.05)
                    || (Math.abs(gamepad1.right_stick_x) > 0.05)) {
                motionSignal = true;
                if (Math.abs(gamepad1.right_stick_x) < 0.05) {
                    drive.setDriveSignal(new DriveSignal(
                            new Pose2d(-300*gamepad1.left_stick_y,-300*gamepad1.left_stick_x, -3*gamepad1.right_stick_x)));
                } else {
                    drive.setDriveSignal(new DriveSignal(
                            new Pose2d(-100*gamepad1.left_stick_y,-100*gamepad1.left_stick_x, -3*gamepad1.right_stick_x)));
                }

            }

            // rotate using bumpers
            if (gamepad1.left_bumper || gamepad1.right_bumper) {
                motionSignal = true;
                double bumperRotationSpeed = Math.min(BUMPER_ROTATION_SPEED,
                        bumper_rotate_accel.milliseconds() / millisecondsToFullSpeed);

                // make bumper rotation speed independent of drivetrain speed adjust
                bumperRotationSpeed = bumperRotationSpeed / drivetrainSpeedAdjust * 5;

                if (gamepad1.left_bumper) {
                    leftBackSpeed = leftFrontSpeed = -bumperRotationSpeed;
                    rightBackSpeed = rightFrontSpeed = bumperRotationSpeed;
                } else if (gamepad1.right_bumper) {
                    leftBackSpeed = leftFrontSpeed = bumperRotationSpeed;
                    rightBackSpeed = rightFrontSpeed = -bumperRotationSpeed;
                }
                drive.setMotorPowers(leftFrontSpeed, leftBackSpeed, rightBackSpeed, rightFrontSpeed);

            } else {
                bumper_rotate_accel.reset();
            }

            // dpad drive controls
            if (gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_left || gamepad1.dpad_down) {
                double DPAD_SPEED;
                if (drivetrainSpeedAdjust == 2) {
                    // speed up slightly when left trigger pressed
                    DPAD_SPEED = 0.45;
                } else {
                    DPAD_SPEED = 0.35;
                }

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
                motionSignal = true;
                drive.setMotorPowers(leftFrontSpeed, leftBackSpeed, rightBackSpeed, rightFrontSpeed);
            } else {
                dpad_accel.reset();
            }

            // auto drive controls
            if (gamepad1.left_stick_button && !gamepad1LeftStickButtonPressed) {
                gamepad1LeftStickButtonPressed = true;

                // to depot
                if (autoDriving) {
                    // stop auto-driving
                    drive.followTrajectory(drive.trajectoryBuilder().build());
                    autoDriving = false;
                } else {
                    // start auto-driving
                    if (blue) {
                        drive.followTrajectory(drive.trajectoryBuilder()
//                            .lineTo(new Vector2d(45, 70), new SplineInterpolator(drive.getRawExternalHeading(),
//                                    Math.toRadians(90)))
                                .splineTo(new Pose2d(45, 70, Math.toRadians(90)))
                                .build());
                    } else {
                        drive.followTrajectory(drive.trajectoryBuilder()
//                            .lineTo(new Vector2d(45, 70), new SplineInterpolator(drive.getRawExternalHeading(),
//                                    Math.toRadians(90)))
                                .splineTo(new Pose2d(45, -70, Math.toRadians(-90)))
                                .build());
                    }
                    autoDriving = true;
                }

            } else if (!gamepad1.left_stick_button) {
                gamepad1LeftStickButtonPressed = false;
            }

            // auto drive controls
            if (gamepad1.right_stick_button && !gamepad1RightStickButtonPressed) {
                gamepad1RightStickButtonPressed = true;

                // to depot
                if (autoDriving) {
                    // stop auto-driving
                    drive.followTrajectory(drive.trajectoryBuilder().build());
                    autoDriving = false;
                } else {
                    if (blue) {
                        drive.followTrajectory(drive.trajectoryBuilder()
//                                .lineTo(new Vector2d(16, -14), new SplineInterpolator(drive.getRawExternalHeading(),
//                                        Math.toRadians(180)))
                                .splineTo(new Pose2d(16, -14, Math.toRadians(180)))
                                .build());
                    } else {
                        drive.followTrajectory(drive.trajectoryBuilder()
//                                .lineTo(new Vector2d(16, 14), new SplineInterpolator(drive.getRawExternalHeading(),
//                                        Math.toRadians(180)))
                                .splineTo(new Pose2d(16, 14, Math.toRadians(180)))
                                .build());
                    }
                    // start auto-driving

                    autoDriving = true;
                }

            } else if (!gamepad1.right_stick_button) {
                gamepad1RightStickButtonPressed = false;
            }


            // intake/outtake
            if (gamepad1.a || gamepad2.a) {
                // intake
                intakeSpeed = 0.65;
                lastIntakeButton = "on";
            } else if (gamepad1.b || gamepad2.b) {
                // outtake
                intakeSpeed = -0.3;
                lastIntakeButton = "temp";
            } else if (false) {
                // intake fast
                intakeSpeed = 1;
                lastIntakeButton = "temp";
            } else if (lastIntakeButton.equals("temp")) {
                // stop intake
                intakeSpeed = 0;
                lastIntakeButton = "stop";
            }

            // capstone
            if (gamepad1.y && !gamepad1YPressed) {
                gamepad1YPressed = true;
                double currentGripperPosition = robot.gripper_servo.getPosition();
                if (currentGripperPosition < 0.01) {
                    robot.gripper_servo.setPosition(lastGripperPosition);
                } else {
                    lastGripperPosition = currentGripperPosition;
                    robot.gripCap();
                }
            } else if (!gamepad1.y) {
                gamepad1YPressed = false;
            }

            // reset orientation
            if (gamepad1.x) {
                drive.setPoseEstimate(new Pose2d(10.42, -25.68, Math.toRadians(180)));
            }

            // gamepad 2

            // manually adjust lift
            if (gamepad2.right_bumper && (lift.mode != LiftController.Mode.CONTROLLING_VELOCITY)) {
                lift.moveAtVelocity(1);
            } else if (gamepad2.left_bumper && (lift.mode != LiftController.Mode.CONTROLLING_VELOCITY)) {
                lift.moveAtVelocity(-1);
            } else if (!gamepad2.right_bumper && !gamepad2.left_bumper && lift.mode == LiftController.Mode.CONTROLLING_VELOCITY) {
                lift.stopMoving();
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
            // TODO add automatic pusher routine
            if (stack_routine_time == null && gamepad2.dpad_right) {
                stack_routine_time = new ElapsedTime();
            }
            if (stack_routine_time != null) {
                if (stack_routine_time.milliseconds() < 300) {
                    robot.pushServoUp();
                } else if (stack_routine_time.milliseconds() < 600) {
                    robot.pushServoDown();
                } else if (stack_routine_time.milliseconds() < 900) {
                    robot.pushServoUp();
                    robot.gripRelease();
                } else if (stack_routine_time.milliseconds() < 1200) {
                    robot.pushServoDown();
                } else if (stack_routine_time.milliseconds() < 1500) {
                    robot.v4bStack();
                } else if (stack_routine_time.milliseconds() < 1800) {
                    robot.grip();
                    intakeSpeed = 0;
                    lastIntakeButton = "stop";
                }  else {
                    stack_routine_time = null;
                }
            }

            // lift + flip routine
            if (lift_routine_time == null && gamepad2.dpad_up) {
                lift_routine_time = new ElapsedTime();
                drop_routine_2_time = null;
            }
            if (lift_routine_time != null) {
                if (lift_routine_time.milliseconds() < 500) {
                    // get pusher out of the way
                    robot.pushServoUp();
                } else if (lift_routine_time.milliseconds() < 600){
                    if (desiredLiftPosition < 8) {
//                        target = -50 + -127 * desiredLiftPosition;
                        lift.moveToPosition(1.69 + 4.28 * desiredLiftPosition);
                    } else {
                        // capstone (go less high)
//                        target = -970;
                        lift.moveToPosition(32);
                    }
                } else if (lift.mode != LiftController.Mode.STOPPED) {
                    // we're still far away from the target
                    // do nothing
                } else {
                    // extend v4b servos to drop position
                    if (capPosition) {
                        // capstone level
                        robot.v4bCapstone();
                        robot.foundationUp();
                    } else {
                        robot.v4bDown();
                    }
                    if (desiredLiftPosition < 8) {
                        desiredLiftPosition++;
                    }
                    lift_routine_time = null;
                }
            }

            if (gamepad2.dpad_left && lift_routine_time == null) {
                desiredLiftPosition = 0;
                lift_routine_time = new ElapsedTime();
            }

            // drop routine
            if (drop_routine_time == null && gamepad2.dpad_down) {
                drop_routine_time = new ElapsedTime();
            }
            if (drop_routine_time != null) {
                if (drop_routine_time.milliseconds() < 400) {
                    //release the gripper
                    robot.gripRelease();
                    goneUp = false;
                } else if (drop_routine_time.milliseconds() < 500) {
                    if (!goneUp) {
//                        if (desiredLiftPosition < 8) {
//
//                        } else {
//                            target = -1005;
//                        }
                        if (capPosition) {
                            lift.moveToPosition(lift.currentPosition + 4);
                        } else {
                            lift.moveToPosition(lift.currentPosition + 2);
                        }
                        goneUp = true;
                    }
                } else if (lift.mode != LiftController.Mode.STOPPED) {
                    // we're still far away from the target
                    // do nothing
                } else {
                    // start new routine timer
                    if (capPosition) {
                        robot.v4bCapstone();
                    } else {
                        drop_routine_2_time = new ElapsedTime();
                    }
                    drop_routine_time = null;
                }
            }

            // second part of drop routine
            if (drop_routine_2_time != null) {
                if (drop_routine_2_time.milliseconds() < 400) {
                    // reset v4b's to grab position
                    robot.v4bGrab();
                    telemetry.addLine("grab position");
                } else if (drop_routine_2_time.milliseconds() < 800) {
                    // lower the lift
                    lift.moveToPosition(0);
                    telemetry.addLine("lower lift");
                } else if (drop_routine_2_time.milliseconds() < 1600) {
                    telemetry.addLine("wait position");
                    // reset v4b's to wait position
                    robot.v4bWait();
                    intakeSpeed = 0.6;
                    lastIntakeButton = "on";
                } else {
                    drop_routine_2_time = null;
                }
            }

            // left joystick - control pusher
            if (gamepad2.right_stick_y > 0.1) {
                double pusherPosition = 0.7 * gamepad2.right_stick_y + 0.3;
                robot.push_servo.setPosition(pusherPosition);
            }
            if (gamepad2.right_stick_y < -0.1) {
                double pusherPosition = 1- (0.7 * (-gamepad2.right_stick_y) + 0.3);
                robot.push_servo.setPosition(pusherPosition);
            }

            //
            if (gamepad2.right_stick_button && !gamepad2RightButtonPressed) {
                gamepad2RightButtonPressed = true;
                capPosition = !capPosition;

            } else if (!gamepad2.right_stick_button) {
                gamepad2RightButtonPressed = false;
            }

            // emergency re-initialization
            if (gamepad2.left_stick_button) {
                telemetry.addLine("RE-INITIALIZE!");
                // initialize servos
                robot.v4bWait();
                robot.pushServoUp();
            }

            if (capPosition) {
                telemetry.addLine("CAP");
            }

            double lift_position = robot.lift_left.getCurrentPosition();

//            if (target > 0) {
//                robot.lift_left.setPower(1);
//                robot.lift_right.setPower(1);
//                sleep(200);
//                target = 0;
//            }
//
//            if (target < 10 && target > 3) {
//                // go all the way down
//                robot.lift_left.setPower(1);
//                robot.lift_right.setPower(1);
//            }

//            if (manualLiftPower == 0) {


//            } else {
//                robot.lift_left.setPower(manualLiftPower);
//                robot.lift_right.setPower(manualLiftPower);
//                target = lift_position;
//            }

            // park position v4b
            if (gamepad1.x && (gamepad2.left_trigger > 0.85) && (gamepad2.right_trigger > 0.85)) {
                robot.v4bDown();
            }


            // control foundation grippers
            if (foundation_gripper_routine_time != null) {
                telemetry.addLine("field gripper routine");
                // lift foundation grippers for 400ms
                if (foundation_gripper_routine_time.milliseconds() < 400) {
                    // up
                    foundationGripperSpeed = 1;
                } else {
                    // stop the routine
                    foundationGripperSpeed = 0;
                    gripperIsDown = false;
                    foundation_gripper_routine_time = null;
                }
            }

            // foundation gripper
            if (gamepad2.left_stick_y > 0.05) {
                // down
                robot.foundationDown();
            } else if (gamepad2.left_stick_y < -0.05) {
                // up
                robot.foundationUp();
            }

            // set motor powers
//            robot.left_back.setPower(leftBackSpeed);
//            robot.left_front.setPower(leftFrontSpeed);
//            robot.right_back.setPower(rightBackSpeed);
//            robot.right_front.setPower(rightFrontSpeed);

            if (!drive.isBusy()) {
                autoDriving = false;
            }

            if ((leftFrontSpeed != 0 || leftBackSpeed != 0 || rightBackSpeed != 0 || rightFrontSpeed != 0) && autoDriving) {
                drive.followTrajectory(drive.trajectoryBuilder().build());
                autoDriving = false;
            }

            if (!autoDriving) {
//                drive.setMotorPowers(leftFrontSpeed, leftBackSpeed, rightBackSpeed, rightFrontSpeed);
            } else {
                telemetry.addLine("AUTO-DRIVING");
            }

            if (!motionSignal) {
                drive.setMotorPowers(0,0,0,0);
            }

            drive.update();

            robot.right_intake.setPower(intakeSpeed);
            robot.left_intake.setPower(intakeSpeed);
            if (Math.abs(intakeSpeed) < 1e-5) {
                robot.intake_wheel.setPower(0);
            } else if (intakeSpeed > 0) {
                robot.intake_wheel.setPower(1);
            } else {
                robot.intake_wheel.setPower(-1);
            }

            if(blue) {
                telemetry.addLine("Blue");
            } else {
                telemetry.addLine("Red");
            }

            telemetry.addData("lift position", desiredLiftPosition);

//            telemetry.addData("lift mode", lift.mode.toString());
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
