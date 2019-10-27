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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.HardwareDrivetrain;
import org.firstinspires.ftc.teamcode.hardware.HardwareEncoder;

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

@TeleOp(name="Simple Drivetrain Motion", group="TwoMotor")
public class SimpleDrivetrainMotion extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareDrivetrain robot = new HardwareDrivetrain();
    double THRESHOLD = 0.05;
    ElapsedTime left_accel = new ElapsedTime();
    ElapsedTime right_accel = new ElapsedTime();
    ElapsedTime strafe_right_accel = new ElapsedTime();
    ElapsedTime strafe_left_accel = new ElapsedTime();
    double millisecondsToFullSpeed = 600;
    double leftSpeed, rightSpeed, leftBackSpeed, rightBackSpeed, leftFrontSpeed, rightFrontSpeed;

    @Override
    public void runOpMode() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            leftSpeed = rightSpeed = leftBackSpeed = leftFrontSpeed = rightBackSpeed = rightFrontSpeed = 0;

            if (Math.abs(gamepad1.left_stick_y) > THRESHOLD) {
                leftSpeed = Math.min(Math.abs(gamepad1.left_stick_y),
                        left_accel.milliseconds() / millisecondsToFullSpeed);
                leftBackSpeed = leftSpeed * Math.signum(gamepad1.left_stick_y);
                leftFrontSpeed = leftBackSpeed;
            } else {
                left_accel.reset();
            }


            if (Math.abs(gamepad1.right_stick_y) > THRESHOLD) {
                rightSpeed = Math.min(Math.abs(gamepad1.right_stick_y),
                        right_accel.milliseconds() / millisecondsToFullSpeed);
                rightBackSpeed = rightSpeed * Math.signum(gamepad1.right_stick_y);
                rightFrontSpeed = rightBackSpeed;
            } else {
                right_accel.reset();
            }

            if (gamepad1.right_bumper) {
                rightBackSpeed = -Math.min(1, strafe_right_accel.milliseconds() / millisecondsToFullSpeed);
                rightFrontSpeed = -rightBackSpeed;
                leftBackSpeed = -rightBackSpeed;
                leftFrontSpeed = rightBackSpeed;
            } else {
                strafe_right_accel.reset();
            }

            if (gamepad1.left_bumper) {
                rightBackSpeed = Math.min(1, strafe_left_accel.milliseconds() / millisecondsToFullSpeed);
                rightFrontSpeed = -rightBackSpeed;
                leftBackSpeed = -rightBackSpeed;
                leftFrontSpeed = rightBackSpeed;
            } else {
                strafe_left_accel.reset();
            }

            robot.left_back.setPower(leftBackSpeed);
            robot.left_front.setPower(leftFrontSpeed);
            robot.right_back.setPower(rightBackSpeed);
            robot.right_front.setPower(rightFrontSpeed);

            telemetry.addData("leftBack", leftBackSpeed);
            telemetry.addData("rightBack", rightBackSpeed);
            telemetry.addData("leftFront", leftFrontSpeed);
            telemetry.addData("rightFront", rightFrontSpeed);

            telemetry.update();
        }
    }
}
