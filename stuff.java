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
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="The Grind", group="Iterative Opmode")
//@Disabled
public class stuff extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor verticalSlider = null;
    private DcMotor horizontalSlider = null;
    private DcMotor belt = null;
    private DcMotor basketFlip = null;
    private Servo backBasketRelease = null;
    private Servo frontBasketRelease = null;



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "lf_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lb_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb_drive");
        //verticalSlider = hardwareMap.get(DcMotor.class, "vertical_slider");
        horizontalSlider = hardwareMap.get(DcMotor.class, "horizontal_slider");
        belt = hardwareMap.get(DcMotor.class, "belt");
       /* basketFlip = hardwareMap.get(DcMotor.class, "basket_flip");
        backBasketRelease = hardwareMap.get(Servo.class, "back_basket");
        frontBasketRelease = hardwareMap.get(Servo.class, "front_basket");*/



        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        horizontalSlider.setDirection(DcMotor.Direction.REVERSE);

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
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;
        double basketPower;
        double verticalPower;
        double horizontalPower;
        double beltPower;
        double backReleasePosition;
        double frontReleasePosition;

        boolean bPrevState = false;
        boolean bCurrState = false;

        boolean strafe = gamepad1.x;
        double drive = gamepad1.right_stick_y;
        double turn = -gamepad1.right_stick_x;
        boolean basketup = gamepad1.dpad_up;
        boolean basketdown = gamepad1.dpad_down;
        boolean basketFlipforward = gamepad1.right_bumper;
        boolean basketFlipback = gamepad1.left_bumper;
        boolean beltOut = gamepad2.b;
        boolean beltIn = gamepad2.a;
        boolean frontRelease = gamepad2.left_bumper;
        boolean backRelease = gamepad2.right_bumper;
        boolean horizontalOut = gamepad2.dpad_up;
        boolean horizontalIn = gamepad2.dpad_down;



        if (strafe) { //Strafing mode: if left bumper is pressed and held down, the driver can only strafe left and right
            double speed = gamepad1.right_stick_x;
            //MIGHT HAVE TO REVISE WHICH ONES ARE NEGATIVE
            telemetry.addData("Drive Mode", "Strafing");
            leftFrontPower = Range.clip(-speed, -1, 1); //If joystick is moved to the left, then speed will be negative, and the left front wheel needs to spin forward
            leftBackPower = Range.clip(speed, -1, 1);
            rightFrontPower = Range.clip(speed, -1, 1);
            rightBackPower = Range.clip(-speed, -1, 1);
        } else {
            telemetry.addData("Drive Mode", "Regular");
            leftFrontPower = Range.clip(drive + turn, -.8, .8);
            leftBackPower = Range.clip(drive + turn, -.8, .8);
            rightFrontPower = Range.clip(drive - turn, -.8, .8);
            rightBackPower = Range.clip(drive - turn, -.8, .8);
        }

        //Vertical slider controlled by dpad on gamepad1
        if (basketup) {
            verticalPower = 1;
        } else if (basketdown) {
            verticalPower = -1;
        } else {
            verticalPower = 0;
        }

        //Basket flip
        if (basketFlipforward) {
            basketPower = 1;
        } else if (basketFlipback) {
            basketPower = -1;
        } else {
            basketPower = 0;
        }

        //Servos controlling release on basket
        //if (frontRelease) {
            //frontReleasePosition = 0;
       // }

        //Horizontal slider controlled by dpad on gamepad2
        if (horizontalOut) {
            horizontalPower = 1;
        } else if (horizontalIn) {
            horizontalPower = -1;
        } else {
            horizontalPower = 0;
        }

        if (beltIn){
            beltPower = 1;
        } else if (beltOut){
            beltPower = -1;
        } else {
            beltPower = 0;
        }








        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
        rightFrontDrive.setPower(rightFrontPower);
        //verticalSlider.setPower(verticalPower);
        horizontalSlider.setPower(horizontalPower);
        belt.setPower(beltPower);


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Left Front Encoder", "Current Position: " + leftFrontDrive.getCurrentPosition());
        telemetry.addData("Left Back Encoder", "Current Position: " + leftBackDrive.getCurrentPosition());
        telemetry.addData("Right Front Encoder", "Current Position: " + rightFrontDrive.getCurrentPosition());
        telemetry.addData("Right Back Encoder", "Current Position: " + rightBackDrive.getCurrentPosition());

        /*
         * Code to run ONCE after the driver hits STOP
         */
    }
    @Override
    public void stop() {
    }

}
