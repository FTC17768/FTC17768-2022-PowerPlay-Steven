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

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 */

@TeleOp(name="ARCADE: IterativeOpMode 21-22", group="Iterative Opmode")
public class Drivetrain extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor top;
    private DcMotorEx arm;
    private Servo fingersR;
    private Servo fingersL;
    private PIDController armController;

    double maxPosition = 160;
    double minPosition = 0;
    //constants for pid control
    //increasing kP will increase overshoot, increase rise time, and increase instability
    //increasing kI will reduce steady state error(how off from the target that you settle at), increase rise time & increase overshoot a little
    //increasing kD will reduce overshoot, slightly slow down rise time, and if it gets too high will cause the mechanism to jitter. If you have problems with overshoot, bring this up.
    double kP = 0.025;
    double kI = 0;
    double kD = 0.25;

    //constants for arm motion
    double finalSetPoint = 50;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        //drivetrain hardware mapping and directional config
        //right drive is set backwards because the sides are mirrored
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        //motor to spin the carousel
        top = hardwareMap.get(DcMotor.class, "top");
        top.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        top.setDirection(DcMotorSimple.Direction.REVERSE);

        //arm hardware mapping and PID controller creation
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        armController = new PIDController(kP,kI,kD);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //finger hardware mapping.
        //these are the things that grabbed the game pieces for 2022 game
        fingersR = hardwareMap.get(Servo.class, "fingersA");
        fingersL = hardwareMap.get(Servo.class, "fingersB");

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
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("ArmPosition", arm.getCurrentPosition());
        telemetry.addData("setPosition", finalSetPoint);
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        boolean dpadDown = gamepad1.dpad_down;
        boolean dpadUp = gamepad1.dpad_up;
        float rTrigger = gamepad1.right_trigger;
        float lTrigger = gamepad1.left_trigger;
        boolean B = gamepad1.b;
        boolean X = gamepad1.x;

// Servos are ALMOST opposite (meet in middle at R0.5, L1)
        if (lTrigger>0.1) { //Lefty-loosey
            fingersR.setPosition(1);
            fingersL.setPosition(0.5);
        } else if (rTrigger>0.1) { //Righty-tighty
            fingersR.setPosition(0.6);
            fingersL.setPosition(1);
        }

        if (B) {
            top.setPower(0.8);
        } else if (X) {
            top.setPower(-0.8);
        } else {
            top.setPower(0);
        }

        //loop to adjust set point based on current position
        if(dpadUp && arm.getCurrentPosition()<maxPosition){
            finalSetPoint++;
        } else if(dpadDown && arm.getCurrentPosition()>minPosition){
            finalSetPoint--;
        }

        //code to set in case it goes over
        if(arm.getCurrentPosition()<minPosition){
            finalSetPoint = minPosition;
        } else if(arm.getCurrentPosition()>maxPosition){
            finalSetPoint = maxPosition;
        }

        setArmPID(finalSetPoint);

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.right_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    private void setArmPID(double setpoint) {
        double output = armController.calculate(arm.getCurrentPosition(), setpoint);
        arm.setPower(output);
    }
}
