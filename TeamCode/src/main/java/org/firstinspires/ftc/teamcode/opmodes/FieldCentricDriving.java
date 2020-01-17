/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.HardwareDrivetrain;
import org.firstinspires.ftc.teamcode.hardware.HardwareNames;
import org.firstinspires.ftc.teamcode.math.MathFunctions;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalCoordinatePosition;

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

@TeleOp(name="Field Centric Driving", group="TwoMotor")
@Disabled
public class FieldCentricDriving extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareDrivetrain robot = new HardwareDrivetrain();
    double THRESHOLD = 0.05;
    ElapsedTime accel = new ElapsedTime();
    ElapsedTime rotate_accel = new ElapsedTime();
    ElapsedTime strafe_right_accel = new ElapsedTime();
    ElapsedTime strafe_left_accel = new ElapsedTime();
    double millisecondsToFullSpeed = 600;
    double speedAdjust = 0;
    double leftSpeed, rightSpeed, leftBackSpeed, rightBackSpeed, leftFrontSpeed, rightFrontSpeed,
            intakeSpeed;
    private OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight,
                robot.horizontal, HardwareNames.COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            leftSpeed = rightSpeed = leftBackSpeed = leftFrontSpeed = rightBackSpeed = rightFrontSpeed = 0;

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
                leftFrontSpeed = Math.min(Math.abs(gamepad1.left_stick_x),
                        rotate_accel.milliseconds() / millisecondsToFullSpeed) * Math.signum(gamepad1.left_stick_x);

                leftBackSpeed = leftFrontSpeed;
                rightBackSpeed = -leftFrontSpeed;
                rightFrontSpeed = rightBackSpeed;
            } else {
                rotate_accel.reset();
            }

            // intake/outtake
            if (gamepad1.a) {
                // intake
                intakeSpeed = 1;
            } else if (gamepad1.b) {
                // outtake
                intakeSpeed = -1;
            } else if (gamepad1.x) {
                intakeSpeed = 0;
            }

            // set motor powers
            robot.left_back.setPower(leftBackSpeed);
            robot.left_front.setPower(leftFrontSpeed);
            robot.right_back.setPower(rightBackSpeed);
            robot.right_front.setPower(rightFrontSpeed);

            robot.right_intake.setPower(intakeSpeed);
            robot.left_intake.setPower(intakeSpeed);

            // update telemetry
            telemetry.addData("leftBack", leftBackSpeed);
            telemetry.addData("rightBack", rightBackSpeed);
            telemetry.addData("leftFront", leftFrontSpeed);
            telemetry.addData("rightFront", rightFrontSpeed);
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
