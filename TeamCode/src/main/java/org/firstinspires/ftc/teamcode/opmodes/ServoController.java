package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.teamcode.hardware.HardwareDrivetrain;
import org.firstinspires.ftc.teamcode.hardware.HardwareNames;
import org.openftc.revextensions2.ExpansionHubMotor;

@Config
public class ServoController extends HardwareDrivetrain {

    HardwareDrivetrain robot = new HardwareDrivetrain();

    //PUSH SERVO
    public void pushServoReset() {
        robot.push_servo.setPosition(0);
    }
    public void pushServoDown() {
        robot.push_servo.setPosition(0.35);
    }

    public void pushServoUp() {
        robot.push_servo.setPosition(1);
    }

    //GRIPPER SERVO
    public void grip() {
        robot.gripper_servo.setPosition(1);
    }

    public void midgrip() {
        robot.gripper_servo.setPosition(0.4);
    }

    public void ungrip() {
        robot.gripper_servo.setPosition(0);
    }

    //V4B
    public void v4bStack() {
        robot.left_v4b.setPosition(0.72);
        robot.right_v4b.setPosition(0.72);
    }

    public void v4bGrab() {
        robot.left_v4b.setPosition(0.75);
        robot.right_v4b.setPosition(0.75);
    }

    public void v4bWait() {
        robot.left_v4b.setPosition(0.6);
        robot.right_v4b.setPosition(0.6);
    }

    public void v4bCapstone() {
        robot.left_v4b.setPosition(0.5);
        robot.right_v4b.setPosition(0.5);
    }

    public void v4bDown() {
        robot.left_v4b.setPosition(0);
        robot.right_v4b.setPosition(0);
    }

    //Foundation
    public void foundationReset() {
        robot.foundation_right.setPosition(0);
        robot.foundation_left.setPosition(0);
    }

    public void foundationUp() {
        robot.foundation_right.setPosition(0.2);
        robot.foundation_left.setPosition(0.4);
    }

    public void foundationDown() {
        robot.foundation_left.setPosition(1);
        robot.foundation_right.setPosition(1);
    }

    //Park Servo
    public void parkOn() {
        robot.park_servo.setPosition(1);
    }

    public void parkOff() {
        robot.park_servo.setPosition(0);
    }

}
