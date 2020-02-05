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

import static org.firstinspires.ftc.teamcode.opmodes.LiftController.k_G;
import static org.firstinspires.ftc.teamcode.opmodes.LiftController.k_d;
import static org.firstinspires.ftc.teamcode.opmodes.LiftController.k_i;
import static org.firstinspires.ftc.teamcode.opmodes.LiftController.k_p;

/*
 * Op mode for tuning follower PID coefficients (located in the drive base classes). The robot
 * drives in a DISTANCE-by-DISTANCE square indefinitely.
 */
@Config
@Autonomous(name = "Detector Test")
public class DetectorTest extends LinearOpMode {
    private OpenCvInternalCamera phoneCam;
    private NewStoneDetector detector;

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

        drive.setPoseEstimate(new Pose2d(0, 0, 0));
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
        // 65 (back), 96 (middle) 187 (front), 20 (front)

        // setup camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        detector = new NewStoneDetector(false);
        phoneCam.setPipeline(detector);
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

            telemetry.addData("leftPosition", detector.frontTotal);
            telemetry.addData("middlePosition", detector.middleTotal);
            telemetry.addData("rightPosition", detector.backTotal);
            telemetry.update();
        }


        phoneCam.pauseViewport();
        phoneCam.stopStreaming();
    }
}

