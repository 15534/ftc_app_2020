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

package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

public class HardwareDrivetrain
{
    /* Public OpMode members. */
    public DcMotor left_front, right_front, right_back, left_back;
    public DcMotor verticalLeft, verticalRight, horizontal;
    public DcMotor left_intake, right_intake;

    public Servo push_servo, gripper_servo, left_v4b, right_v4b;
    public CRServo foundation_left, foundation_right;
    public PwmControl left_v4b_pwm, right_v4b_pwm;

    public DcMotor  lift_left   = null;
    public DcMotor  lift_right  = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period  = new ElapsedTime();

    public HardwareDrivetrain() {


    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hardwareMap) {
        // servos
        push_servo = hardwareMap.servo.get(HardwareNames.push_servo);
        gripper_servo = hardwareMap.servo.get(HardwareNames.gripper_servo);

        foundation_left = hardwareMap.crservo.get(HardwareNames.foundation_left);
        foundation_right = hardwareMap.crservo.get(HardwareNames.foundation_right);
        foundation_right.setDirection(CRServo.Direction.FORWARD);
        foundation_left.setDirection(CRServo.Direction.REVERSE);

        left_v4b = hardwareMap.servo.get(HardwareNames.left_v4b);
        right_v4b = hardwareMap.servo.get(HardwareNames.right_v4b);
        left_v4b.setDirection(Servo.Direction.REVERSE);
        right_v4b.setDirection(Servo.Direction.FORWARD);

        left_v4b.scaleRange(0, 1);
        right_v4b.scaleRange(0, 1);

        left_v4b_pwm = (PwmControl) left_v4b;
        right_v4b_pwm = (PwmControl) right_v4b;

        PwmControl.PwmRange range = new PwmControl.PwmRange(800, 2200);
        left_v4b_pwm.setPwmRange(range);
        right_v4b_pwm.setPwmRange(range);

        lift_left  = hardwareMap.get(DcMotor.class, HardwareNames.lift_left);
        lift_right = hardwareMap.get(DcMotor.class, HardwareNames.lift_right);

        // drive motors
        right_front = hardwareMap.dcMotor.get(HardwareNames.right_front);
        right_back = hardwareMap.dcMotor.get(HardwareNames.right_back);
        left_front = hardwareMap.dcMotor.get(HardwareNames.left_front);
        left_back = hardwareMap.dcMotor.get(HardwareNames.left_back);

        left_intake = hardwareMap.dcMotor.get(HardwareNames.left_intake);
        right_intake = hardwareMap.dcMotor.get(HardwareNames.right_intake);

        verticalLeft = hardwareMap.dcMotor.get(HardwareNames.left_encoder);
        verticalRight = hardwareMap.dcMotor.get(HardwareNames.right_encoder);
        horizontal = hardwareMap.dcMotor.get(HardwareNames.horizontal_encoder);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);
        right_front.setDirection(DcMotorSimple.Direction.FORWARD);
        right_back.setDirection(DcMotorSimple.Direction.FORWARD);

        // intake
        left_intake.setDirection(DcMotorSimple.Direction.FORWARD);
        right_intake.setDirection(DcMotorSimple.Direction.REVERSE);
        left_intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right_intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // lift
        lift_left.setDirection(DcMotor.Direction.FORWARD);
        lift_right.setDirection(DcMotor.Direction.REVERSE);

        lift_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
 }

