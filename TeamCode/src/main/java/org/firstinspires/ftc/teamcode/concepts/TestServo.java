package org.firstinspires.ftc.teamcode.concepts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.HardwareServo;

@TeleOp(name="Test Servo", group="TwoMotor")
public class TestServo extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareServo robot = new HardwareServo();

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
            // check to see if we need to move the servo.

            if(gamepad1.a) {
                // move to 0 degrees.
                robot.left_servo.setPosition(0);
            } else if (gamepad1.b) {
                // move to 180 degrees.
                robot.left_servo.setPosition(1);
            }
            telemetry.addData("Servo Position", robot.left_servo.getPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }

}


