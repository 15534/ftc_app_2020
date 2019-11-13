package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareServo {

    /* Public OpMode members. */
    public Servo left_servo   = null;
    //public Servo right_servo  = null;

    /* local OpMode members. */
    HardwareMap hardwareMap = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        hardwareMap = hwMap;
        // TODO disable this hardware
        left_servo  = hardwareMap.get(Servo.class, HardwareNames.push_servo);
    }
}

