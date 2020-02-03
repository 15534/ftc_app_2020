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

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="PID Lift Test")
public class LiftPIDTest extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareDrivetrain robot = new HardwareDrivetrain();
    public static double k_p = 0.003;
    public static double k_i = 0;
    public static double k_d = 0;
    public static double k_V = 0.001;
    public static double k_G = -0.005;
    public static double maxVel = 100;
    public static double maxAccel = 200;
    public static double maxJerk = 200;
//    public static double target = -800;


    @Override
    public void runOpMode() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        robot.lift_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.lift_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lift_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.left_v4b.setPosition(0.75);
        robot.right_v4b.setPosition(0.75);

        PIDCoefficients liftPidCoefficients = new PIDCoefficients(k_p, k_i, k_d);
        PIDFController controller = new PIDFController(liftPidCoefficients, k_V, 0, k_G);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        controller.setOutputBounds(-1, 1);

        MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(0,0,0),
                new MotionState(-500, 0,0), maxVel, maxAccel, maxJerk);

        ElapsedTime t = new ElapsedTime();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double lift_position = robot.lift_left.getCurrentPosition();
//            double correction = controller.update(lift_position);
            MotionState desiredMotionState = profile.get(t.seconds());
            controller.setTargetPosition(desiredMotionState.getX());
            double correction = controller.update(lift_position, desiredMotionState.getV(), desiredMotionState.getA());
            robot.lift_left.setPower(correction);
            robot.lift_right.setPower(correction);

//            telemetry.addData("right", robot.lift_right.getCurrentPosition());
            telemetry.addData("position", lift_position);
            telemetry.addData("correction", correction);
            telemetry.addData("x", desiredMotionState.getX());
            telemetry.addData("v", desiredMotionState.getV());
            telemetry.addData("a", desiredMotionState.getA());

//            telemetry.addData("power", gamepad1.right_stick_y);
            telemetry.update();
        }
    }
}
