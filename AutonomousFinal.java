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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;



/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Autonomous Blue", group="Linear Opmode")
//@Disabled
public class AutonomousFinal extends LinearOpMode { //THIS IS FOR IF WE ARE BLUE TEAM

    //Note: Circumference of mecanum wheel is approx. 12.566 inches, 115 encoder ticks per inch


    public boolean anyBusy() {
        return (leftBackDrive.isBusy() || leftFrontDrive.isBusy() || rightBackDrive.isBusy() || rightFrontDrive.isBusy());
    } //Easy, simple method call to see if any of the driving motors are still busy; returns true is any of them are

    public boolean checkTargetRange(double tar) {
        int lfStart = leftFrontDrive.getCurrentPosition();
        int lbStart = leftBackDrive.getCurrentPosition();
        int rfStart = rightFrontDrive.getCurrentPosition();
        int rbStart = rightBackDrive.getCurrentPosition();
        return ((Math.abs((tar + lfStart) - leftFrontDrive.getCurrentPosition()) < 100) && (Math.abs((tar + lbStart) - leftBackDrive.getCurrentPosition()) < 100) && (Math.abs((tar + rfStart) - rightFrontDrive.getCurrentPosition()) < 100) && (Math.abs((tar + rbStart) - rightBackDrive.getCurrentPosition()) < 100));
    } //returns true if all of the motors have reached within 100 encoder ticks of their target position to prevent

    public void moveForBack(int targ) { //This method allows the robot to simply run forwards and backwards by giving the robot a target number of inches to move to; POS is back, NEG is for

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(0.3);
        leftBackDrive.setPower(0.3);
        rightFrontDrive.setPower(0.3);
        rightBackDrive.setPower(0.3);

        int lfStart = leftFrontDrive.getCurrentPosition(); //this accounts for the starting encoder positions
        int lbStart = leftBackDrive.getCurrentPosition();
        int rfStart = rightFrontDrive.getCurrentPosition();
        int rbStart = rightBackDrive.getCurrentPosition();

        leftFrontDrive.setTargetPosition(targ + lfStart); //Might need to negate left values
        leftBackDrive.setTargetPosition(targ + lbStart);
        rightFrontDrive.setTargetPosition(targ + rfStart);
        rightBackDrive.setTargetPosition(targ + rbStart);

        while (anyBusy()) {
            if (checkTargetRange(targ)){
                leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition());
                leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition());
                rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition());
                rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition());
            }
        }
    }

    public void turn90(int LR) { //1 for left, 0 for right
        int target = 1600; //This value may change with further testing of the robot to get a more precise turn
        if (LR == 1) { //an input of 1 corresponds to turning LEFT
            target = target * -1;
        }

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(0.3);
        leftBackDrive.setPower(0.3);
        rightFrontDrive.setPower(0.3);
        rightBackDrive.setPower(0.3);


        int lfStart = leftFrontDrive.getCurrentPosition();
        int lbStart = leftBackDrive.getCurrentPosition();
        int rfStart = rightFrontDrive.getCurrentPosition();
        int rbStart = rightBackDrive.getCurrentPosition();

        leftFrontDrive.setTargetPosition(-target + lfStart); //Might need to negate left values
        leftBackDrive.setTargetPosition(-target + lbStart);
        rightFrontDrive.setTargetPosition(target + rfStart);
        rightBackDrive.setTargetPosition(target + rbStart);

        if (checkTargetRange(target)){
            leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition());
            leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition());
            rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition());
            rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition());
        }
    }

    public void gripper(int value) { //value will be necessary encoder ticks to fully close the gripper mechanism; positive will close, negative will open
        glyphGripper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        glyphGripper.setPower(0.40);
        int ggStart = glyphGripper.getCurrentPosition();
        glyphGripper.setTargetPosition(ggStart + value);

        while (glyphGripper.isBusy()) {
            if (Math.abs((ggStart + value) - glyphGripper.getCurrentPosition()) < 50) { //if the gripper motor si within 50 encoder marks of where it's supposed to be, say that it's reached its target
                glyphGripper.setTargetPosition(glyphGripper.getCurrentPosition());
            }
        }
    }



    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive; //Motors for wheels
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private DcMotor glyphGripper; //Glyph gripper motor
    private DcMotor lifter;
    private Servo swinger = null; //servo motor that controls the arm to knock off colored balls

    //private Servo turner = null;
    private ColorSensor color_sensor;  //color sensor implementation
    boolean colorworks = false;
    boolean left = false;
    boolean center = false;
    boolean right = false;
    boolean visible = false;
    int count = 0;

    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        //Wheel motors
        /*leftFrontDrive  = hardwareMap.get(DcMotor.class, "lf_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lb_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb_drive");
        //Grabber motor
        glyphGripper = hardwareMap.get(DcMotor.class, "glyph_gripper");
        //Winch motor
        lifter = hardwareMap.get(DcMotor.class, "lifter");
        //Color sensor on arm*/
        color_sensor = hardwareMap.colorSensor.get("color");
        //Servo that controls arm position
        swinger = hardwareMap.servo.get("swinger");



        // Set directions of motors
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        glyphGripper.setDirection(DcMotor.Direction.FORWARD);
        swinger.setDirection(Servo.Direction.FORWARD);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "ATOzBz3/////AAAAGRMgkMSWKEfdlf7+ytKRogZUttffAG5ut8vff2t5WwX6k+kV9zsrBFJM8TkRmB9RHvjojerOovvTw6AHQaBJkwuVSXQknu6a33MFK+Nvc1WOd1YMjNszhIV1tfXpkxEJ7ZHALTSsgGz/eB7BBMmX4vgmH148dEB64L39glx36CflP8PZoPYz6zszYRZTK7ioKOKiGE40QoHXaBCBju08fVIJ6dAWyeZSbEnXIr0ylJvaAobEjSXJ7227gFIDigR+QVM/2FtNJYNcNxZFOW8JDjDvxf0PNuWic81Bxd8slokuYXZpq3SRW17yGEeq55nY1uSxEdDGoSsCLqi47Jrq26kbW4Bnr9P9xTSl27WIvfe4\n";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary





        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //closes the glyph gripper
        gripper(-400);



        // The following code controls the servo motors to knock off the appropriate jewel

        // This code brings the arm with the color sensor down into position to sense the jewel color
        swinger.setPosition(.9);
        color_sensor.enableLed(true);
        while (colorworks == false && runtime.milliseconds() < 10000) {
            telemetry.addData("Part", "Jewels");
            int rValue = color_sensor.red(); //Amount of red in ball
            int bValue = color_sensor.blue(); //Amount of blue in bal
            telemetry.addData("Clear", color_sensor.alpha());
            telemetry.addData("Red  ", color_sensor.red());
            telemetry.addData("Green", color_sensor.green());
            telemetry.addData("Blue ", color_sensor.blue());

            if (bValue > 2) { //If the ball in back is blue, move forward then back onto the balancing stone
                telemetry.addData("Color", "blue");
                moveForBack(-700);
                swinger.setPosition(.55);
                moveForBack(700);
                colorworks = true;
            } else if (rValue > 2) { //If the ball in back is red, move backward then back onto the balancing stone
                telemetry.addData("Color", "red");
                moveForBack(700);
                swinger.setPosition(.55);
                moveForBack(-700);
                colorworks = true;
            }
            telemetry.update();
    }
    swinger.setPosition(.5); //Resets the position to upright so that the arm for the jewel doesn't get in the way in case it couldn't sense the colors

        //relicTrackables.activate();

        //This code repeats while we have more than 8 seconds left until the robot identifies the VuForia code
        /*while (!visible && runtime.milliseconds() > 22000) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                visible = true;
                if (vuMark == RelicRecoveryVuMark.CENTER) {
                    telemetry.addData("VuMark", "%s visible", vuMark);
                    center = true;
                } else if (vuMark == RelicRecoveryVuMark.LEFT) {
                    telemetry.addData("VuMark", "%s visible", vuMark);
                    left = true;
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    telemetry.addData("VuMark", "%s visible", vuMark);
                    right = true;
                }
                telemetry.update();

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();

                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);


                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            }
            else { //if it doesn't find Vuforia, it moves forward a little bit to see if it's there
                telemetry.addData("VuMark", "not visible");
                telemetry.update();
                moveForBack(-200);
                count = count + 1;
                sleep(100);
            }
        } */

        //This is the code to move to the correct cryptobox based on Vuforia, but we are still working on exact encoder values to get it to move to the correct one and not get it stuck
        /*if (center) {
            turn90(1);
            turn90(1);
            moveForBack(-3000 - (count * -200);
            turn90(1);
            gripper(200);
            moveForBack(300); //moves forward approx. 36 inches to cryptobox
            moveForBack(300);
        } else if (left) {
            turn90(1);
            moveForBack(-600);
            turn90(1);
            moveForBack(-4050);
            gripper(150);
            moveForBack(300);
        } else if (right) {
            moveForBack(-4050); //moves forward approx. 36 inches to cryptobox
            moveForBack(300);
        } else {
            turn90(1);
            moveForBack(-600);
            turn90(1);
            moveForBack(-4050);
            gripper(150);
            moveForBack(300);
        }*/


        //Our code to just move the robot to put in the glyph in the cryptobox and be in the safe zone
        //turn90(1);
        //turn90(1);
        telemetry.addData("Part", "Moving to Cryptobox");
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(1);
        leftBackDrive.setPower(1);
        rightFrontDrive.setPower(1);
        rightBackDrive.setPower(1);

        int lfStart = leftFrontDrive.getCurrentPosition(); //this accounts for the starting encoder positions
        int lbStart = leftBackDrive.getCurrentPosition();
        int rfStart = rightFrontDrive.getCurrentPosition();
        int rbStart = rightBackDrive.getCurrentPosition();

        leftFrontDrive.setTargetPosition(4800 + lfStart); //Might need to negate left values
        leftBackDrive.setTargetPosition(-4800 + lbStart);
        rightFrontDrive.setTargetPosition(-4800 + rfStart);
        rightBackDrive.setTargetPosition(4800 + rbStart);

        /*while (anyBusy()) {
            if (checkTargetRange(1600)){
                leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition());
                leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition());
                rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition());
                rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition());
            } */
        moveForBack(4000);
        //gripper(200);
        moveForBack(-400);




        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        telemetry.addData("Part", "YAY ALL DONE!!!!");
    }}