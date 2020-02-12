package org.firstinspires.ftc.teamcode.opmodes;
import org.firstinspires.ftc.teamcode.hardware.HardwareDrivetrain;

public class ServoController extends HardwareDrivetrain {

    //PUSH SERVO
    public void pushServoReset() {
        this.push_servo.setPosition(0);
    }
    public void pushServoUp() {
        this.push_servo.setPosition(0.35);
    }

    public void pushServoDown() {
        this.push_servo.setPosition(1);
    }

    //GRIPPER SERVO
    public void gripRelease() {
        this.gripper_servo.setPosition(1);
    }

    public void grip() {
        this.gripper_servo.setPosition(0.4);
    }

    public void gripCap() {
        this.gripper_servo.setPosition(0);
    }

    //V4B
    public void v4bStack() {
        this.left_v4b.setPosition(0.69);
        this.right_v4b.setPosition(0.69);
    }

    public void v4bGrab() {
        this.left_v4b.setPosition(0.75);
        this.right_v4b.setPosition(0.75);
    }

    public void v4bWait() {
        this.left_v4b.setPosition(0.6);
        this.right_v4b.setPosition(0.6);
    }

    public void v4bCapstone() {
        this.left_v4b.setPosition(0.5);
        this.right_v4b.setPosition(0.5);
    }

    public void v4bDown() {
        this.left_v4b.setPosition(0);
        this.right_v4b.setPosition(0);
    }

    //Foundation
    public void foundationReset() {
        this.foundation_right.setPosition(0);
        this.foundation_left.setPosition(0);
    }

    public void foundationUp() {
        this.foundation_right.setPosition(0.3);
        this.foundation_left.setPosition(0.5);
    }

    public void foundationDown() {
        this.foundation_left.setPosition(1);
        this.foundation_right.setPosition(1);
    }

    //Park Servo
    public void parkOn() {
        this.park_servo.setPosition(1);
    }

    public void parkOff() {
        this.park_servo.setPosition(0);
    }
}
