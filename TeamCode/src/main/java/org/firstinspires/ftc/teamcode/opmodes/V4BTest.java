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
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.hardware.HardwareDrivetrain;

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

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="V4B Test", group="Competition")
public class V4BTest extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareDrivetrain robot = new HardwareDrivetrain();


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

            double currentLeftPosition = robot.left_v4b.getPosition();
            double currentRightPosition = robot.right_v4b.getPosition();

//
            if (gamepad1.dpad_up) {
//                double newLeftPosition = Math.min(currentLeftPosition + 0.01, 0);
//                double newRightPosition = Math.min(currentLeftPosition + 0.01, 0);
//                robot.left_v4b.setPosition(newLeftPosition);
//                robot.right_v4b.setPosition(newRightPosition);
                robot.left_v4b.setPosition(0.75);
                robot.right_v4b.setPosition(0.75);
            } else if (gamepad1.dpad_down) {
//                double newLeftPosition = Math.min(currentLeftPosition - 0.01, 0);
//                double newRightPosition = Math.min(currentLeftPosition - 0.01, 0);
//                robot.left_v4b.setPosition(newLeftPosition);
//                robot.right_v4b.setPosition(newRightPosition);
//                robot.left_v4b.setPosition(0.68);
//                robot.right_v4b.setPosition(0.68);
                robot.left_v4b.setPosition(0.5);
                robot.right_v4b.setPosition(0.5);

            }
//
            telemetry.addData("left position:", currentLeftPosition);
            telemetry.addData("right position:", currentRightPosition);
            telemetry.update();
        }
    }
}
