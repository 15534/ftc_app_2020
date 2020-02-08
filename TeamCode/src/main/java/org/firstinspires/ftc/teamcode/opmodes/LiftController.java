package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.HardwareNames;
import org.openftc.revextensions2.ExpansionHubMotor;

@Config
public class LiftController {
    private DcMotor lift_left, lift_right;
    public static double k_p = 0.5;
    public static double k_i = 0;
    public static double k_d = 0;
    public static double k_V = 0.0027;
    public static double k_G = 0.0027;
    public static double k_static = 0;
    public static double k_A = 0;
    public static double maxVel = 1000;
    public static double maxAccel = 3000;
    public static double manualAccel = 500;
    public static double maxJerk = 2000;
    public static double manualVel = 500;
    public static double encoderTicksPerInch = 29.625;
    private PIDFController controller;
    public double currentPosition = 0;
    public double correction = 0;
    public double error = 0;
    private double manualPower = 0;

    enum Mode {STOPPED, FOLLOWING_TRAJECTORY, CONTROLLING_VELOCITY}
    public Mode mode = Mode.STOPPED;
    public MotionProfile profile = null;
    public ElapsedTime timer = new ElapsedTime();

    public LiftController(HardwareMap hwMap) {
        lift_left = hwMap.get(ExpansionHubMotor.class, HardwareNames.lift_left);
        lift_right = hwMap.get(ExpansionHubMotor.class, HardwareNames.lift_right);

        lift_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift_left.setDirection(DcMotor.Direction.REVERSE);
        lift_right.setDirection(DcMotor.Direction.FORWARD);

        lift_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lift_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        PIDCoefficients liftPidCoefficients = new PIDCoefficients(k_p, k_i, k_d);
        controller = new PIDFController(liftPidCoefficients, k_V, k_A, k_static, x -> k_G);
        controller.setOutputBounds(-1, 1);
        controller.setTargetPosition(0);
    }

    public void update() {
        currentPosition = getDistance();
        error = Math.abs(currentPosition - controller.getTargetPosition());

        if (mode == Mode.STOPPED) {
            correction = controller.update(currentPosition);
        } else if (mode == Mode.FOLLOWING_TRAJECTORY) {
            MotionState desiredMotionState = profile.get(timer.seconds());
            controller.setTargetPosition(desiredMotionState.getX());
            correction = controller.update(currentPosition, desiredMotionState.getV(), desiredMotionState.getX());
        }

        if (mode == Mode.FOLLOWING_TRAJECTORY) {
            if (profile != null && profile.duration() +0.3 <= timer.seconds()) {
                mode = Mode.STOPPED;
                controller.setTargetPosition(profile.end().getX());
            }
        }
        if (mode == Mode.CONTROLLING_VELOCITY) {
            lift_left.setPower(manualPower);
            lift_right.setPower(manualPower);
        } else {
            lift_left.setPower(correction);
            lift_right.setPower(correction);
        }

    }

    public void moveToPosition(double distance) {
        timer.reset();
        mode = Mode.FOLLOWING_TRAJECTORY;
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(currentPosition,0,0),
                new MotionState(distance, 0,0), maxVel, maxAccel, maxJerk);
    }

    private double getDistance() {
        double lift_position = lift_left.getCurrentPosition();
        return lift_position/encoderTicksPerInch;
    }

    public void moveAtVelocity(int velocity) {
        timer.reset();
        mode = Mode.CONTROLLING_VELOCITY;
        if (velocity > 0) {
            manualPower = 0.4;
        } else if (velocity < 0) {
            manualPower = -0.1;
        }
    }

    public void stopMoving() {
        controller.setTargetPosition(currentPosition);
        mode = Mode.STOPPED;
    }
}
