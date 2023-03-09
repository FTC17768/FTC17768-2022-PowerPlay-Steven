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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Mecanum: Linear OpMode 22-23", group="Iterative Opmode")

public class MecanumDrive extends OpMode
{
    // Declare OpMode members.
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor lift;
    private Servo lFinger;
    private Servo rFinger;
    private double powerMod = 1;


    /*double lFingerPos = 1.4;
    double rFingerPos = 0.5;*/
    /*
     * Code to run ONCE when the driver hits INIT
     */
    //@Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftMotor = hardwareMap.get(DcMotor .class, "FLMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FRMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "BRMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "BLMotor");
        rFinger = hardwareMap.get(Servo.class, "lFinger");
        lFinger = hardwareMap.get(Servo.class, "rFinger");
        //arm hardware mapping and PID controller creation
        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //Read the GamePad joystick values, use left stick y for Drive, left stick x for strafe and right stick x for rotate
        //want the negative of the Y stick value as pushing up is a negative value and we want it to be positive up and negative going down

        float drive = gamepad1.left_stick_y;
        float strafe = gamepad1.left_stick_x;
        float rotate = gamepad1.right_stick_x;
        float lTrigger = gamepad2.left_trigger;
        float rTrigger = gamepad2.right_trigger;

        //Since motor power setting need to be between -1 and 1, if the sum of the absolute values of the 3 inputs is great than one, will need to divide by that sum to keep the values between -1 and 1, if the sum is less than 1, then we divide by 1

        float normalize = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(rotate), 1);
        float frontLeftPower = (-drive + strafe + rotate) / normalize;
        float backLeftPower = (drive - strafe + rotate) / normalize;
        float frontRightPower = (drive + strafe + rotate) / normalize;
        float backRightPower = (-drive - strafe + rotate) / normalize;

        //then just set each of the motor powers to the corresponding motor power variable, you will need to use the name for each of the motors that you have in your configuration file.

        frontLeftMotor.setPower(frontLeftPower*powerMod);
        backLeftMotor.setPower(backLeftPower*powerMod);
        frontRightMotor.setPower(frontRightPower*powerMod);
        backRightMotor.setPower(backRightPower*powerMod);

        if (lTrigger>0.1) { //Lefty-loosey
            lFinger.setPosition(0.25);
            rFinger.setPosition(0.75);
        } else if (rTrigger>0.1) { //Righty-tighty
            lFinger.setPosition(0);
            rFinger.setPosition(1);
        }

        //ADJUSTABLE SERVO SCRIPT FOR TESTING VALUES
        //COMMENT OUT BEFORE USE
        /*lFingerPos += gamepad2.left_stick_x;
        rFingerPos += gamepad2.right_stick_x;
        lFinger.setPosition(lFingerPos);
        rFinger.setPosition(rFingerPos);*/
        lift.setPower(-gamepad2.right_stick_y);

        telemetry.addData("FR", frontRightPower);
        telemetry.addData("FL", frontLeftPower);
        telemetry.addData("BR", backRightPower);
        telemetry.addData("BL", backLeftPower);
        telemetry.addData("NORMALIZE", normalize);
        telemetry.addData("LiftPosition", lift.getCurrentPosition());
        telemetry.addData("LiftPower", Math.abs(gamepad2.right_stick_y));
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
