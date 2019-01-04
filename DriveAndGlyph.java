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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="DriveAndGlyph", group="Iterative Opmode")
//@Disabled
public class DriveAndGlyph extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor glyphGripper = null;
    private DcMotor lifter = null;
    //private Servo arm = null; //servo that controls the arm for the relic
    //private Servo claw = null; //servo that controls the claw for the relic
    boolean lowPower = false;
    boolean highPower = true;
    private Servo swinger = null; //servo motor that controls the arm to knock off colored balls


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "lf_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lb_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb_drive");
        glyphGripper = hardwareMap.get(DcMotor.class, "glyph_gripper");
        lifter = hardwareMap.get(DcMotor.class, "lifter");
        swinger = hardwareMap.servo.get("swinger");
        //arm = hardwareMap.servo.get("arm");
        //claw = hardwareMap.servo.get("claw");



        // Set motor direction
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        glyphGripper.setDirection(DcMotor.Direction.FORWARD);
        lifter.setDirection(DcMotor.Direction.FORWARD);
        swinger.setDirection(Servo.Direction.FORWARD); //MIGHT NEED TO CHANGE
        //arm.setDirection(Servo.Direction.REVERSE);
        //claw.setDirection(Servo.Direction.REVERSE);




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
        swinger.setPosition(.5); //set the color sensor arm up out of the way


    }



    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level
        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;
        double gripperPower = 0;
        double lifterPower = 0;
        //double armPos;
        //double clawPos;
        boolean strafe = gamepad1.left_bumper;
        double drive = gamepad1.right_stick_y;
        double turn  =  -gamepad1.right_stick_x;
        swinger.setPosition(.5);







        // - This uses basic math to combine motions and is easier to drive straight.
        if (strafe) { //Strafing mode: if left bumper is pressed and held down on gamepad 1, the driver can only strafe left and right
            double speed = gamepad1.right_stick_x;
            telemetry.addData("Drive Mode", "Strafing");
            leftFrontPower = Range.clip(speed, -.5, .5); //If joystick is moved to the left, then speed will be negative, and the left front wheel needs to spin forward
            leftBackPower = Range.clip(-speed, -.7,.7);
            rightFrontPower = Range.clip(-speed, -.5, .5);
            rightBackPower = Range.clip(speed, -.7, .7);
        }
        else {
            telemetry.addData("Drive Mode", "Regular");
            leftFrontPower = Range.clip(drive + turn, -.7, .7);
            leftBackPower = Range.clip(drive + turn, -.7, .7);
            rightFrontPower = Range.clip(drive - turn, -.7, .7);
            rightBackPower = Range.clip(drive - turn, -.7, .7);
        }

        //Controls for the glyph gripper
        if (gamepad2.right_bumper) {
            gripperPower = -.5;
        } else if (gamepad2.left_bumper) {
            gripperPower = .5;
        }

        //In order to get more control over the glyph grpper, the 'a' button sets the motors to "low power" mode and the 'b' button sets it back to regular or "high power" mode
        if (gamepad2.a) {
            lowPower = true;
            highPower = false;
        } else if (gamepad2.b) {
            lowPower = false;
            highPower = true;
        }

        if (lowPower) {
            gripperPower = gripperPower * .3;
            telemetry.addData("Gripper Mode:", "Low Power");
        } else {
            telemetry.addData("Gripper Mode:", "High Power");
        }

        //This allows the driver with gamepad 1 to move the servos for the relic arm; the left trigger controls the arm (pressing it down all the way moves it over the robot) and the right trigger controls the claw (pressing it down all the way opens the claw)
        //armPos = Range.clip(gamepad1.left_trigger, 0, 1);
        //clawPos = Range.clip(gamepad1.right_trigger + .3, 0, 1);

        //arm.setPosition(armPos);
        //claw.setPosition(clawPos);



        //Controls for the winch connected to the gearbox
        lifterPower = Range.clip(gamepad2.left_stick_y, -1.0, 1.0);


        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower * .5);
        leftBackDrive.setPower(leftBackPower * .5);
        rightBackDrive.setPower(rightBackPower * .5);
        rightFrontDrive.setPower(rightFrontPower * .5);
        glyphGripper.setPower(gripperPower);
        lifter.setPower(lifterPower);


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Left Front Encoder", "Current Position: " + leftFrontDrive.getCurrentPosition());
        telemetry.addData("Left Back Encoder", "Current Position: " + leftBackDrive.getCurrentPosition());
        telemetry.addData("Right Front Encoder", "Current Position: " + rightFrontDrive.getCurrentPosition());
        telemetry.addData("Right Back Encoder", "Current Position: " + rightBackDrive.getCurrentPosition());
        telemetry.addData("Glyph Gripper", "Current Position: " + glyphGripper.getCurrentPosition());
        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
