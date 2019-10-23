package org.firstinspires.ftc.teamcode.concepts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.HardwareDrivetrain;
import org.firstinspires.ftc.teamcode.hardware.HardwareNames;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalCoordinatePosition;

/**
 * Created by Sarthak on 10/4/2019.
 */
@TeleOp(name = "Odometry OpMode", group = "Concept")
public class ConceptOdometryPosition extends LinearOpMode {

    private OdometryGlobalCoordinatePosition globalPositionUpdate;
    private HardwareDrivetrain robot = new HardwareDrivetrain();

    @Override
    public void runOpMode() throws InterruptedException {
        final double COUNTS_PER_INCH = HardwareNames.COUNTS_PER_INCH;
        robot.init(hardwareMap);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();

        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight,
                robot.horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();

        while (opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", robot.verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", robot.verticalRight.getCurrentPosition());
            telemetry.addData("Horizontal encoder position", robot.horizontal.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        // Stop the thread
        globalPositionUpdate.stop();
    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }
}
