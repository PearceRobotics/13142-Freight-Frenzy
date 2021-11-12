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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Teleop", group="Drive code")
public class BasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor carouselDrive = null;
    private DcMotor leftArmDrive = null;
    private DcMotor rightArmDrive = null;
    private DcMotor intakeDrive = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        carouselDrive = hardwareMap.get(DcMotor.class, "carousel_drive");
        leftArmDrive = hardwareMap.get(DcMotor.class, "left_arm_drive");
        rightArmDrive = hardwareMap.get(DcMotor.class, "right_arm_drive");
        intakeDrive = hardwareMap.get(DcMotor.class, "intake_drive");
        
        leftArmDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int armPosition = 0;
        leftArmDrive.setTargetPosition(0);
        rightArmDrive.setTargetPosition(0);
        leftArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftArmDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        carouselDrive.setDirection(DcMotor.Direction.REVERSE);
        leftArmDrive.setDirection(DcMotor.Direction.REVERSE);
        rightArmDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeDrive.setDirection(DcMotor.Direction.FORWARD);

        leftArmDrive.setPower(1);
        rightArmDrive.setPower(1);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double BackLeftPower;
            double rightPower;
            double BackRightPower;

            //Driving and turning(direct drive)
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            BackLeftPower = Range.clip(drive + turn, -1.0,1.0);
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            BackRightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            //two carousels-one forward(left button) and one backward(left trigger)
            if (gamepad1.dpad_up)
                carouselDrive.setPower(1);
            else if (gamepad1.dpad_down)
                carouselDrive.setPower(-1);
            else
                carouselDrive.setPower(0);

            //arm
            if (gamepad1.left_trigger > 0.25)
                armPosition = leftArmDrive.getCurrentPosition() - 45;
            else if (gamepad1.left_bumper)
                armPosition = leftArmDrive.getCurrentPosition() + 45;
            /*else
                armPosition = leftArmDrive.getCurrentPosition();*/

            if (gamepad1.x)
                armPosition = -600; // pickup from the ground
            if (gamepad1.y)
                armPosition = -25; // lift to safety position
            if (gamepad1.a)
                armPosition = -330; // score on level 2
            if (gamepad1.b)
                armPosition = -550; //pickup slightly higher

            /*if (armPosition > 1) {
                leftArmDrive.setPower(0);
                rightArmDrive.setPower(0);
            }
            else if (armPosition > -600) {
                leftArmDrive.setPower(0);
                rightArmDrive.setPower(0);
            }
            else {
                leftArmDrive.setPower(1);
                rightArmDrive.setPower(1);
            }*/

            leftArmDrive.setTargetPosition(armPosition);
            rightArmDrive.setTargetPosition(armPosition);

            // intake
            if (gamepad1.right_bumper)
                intakeDrive.setPower(1);
            else if (gamepad1.right_trigger > 0)
                intakeDrive.setPower(-1);
            else
                intakeDrive.setPower(0);

            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            backLeftDrive.setPower(BackLeftPower);
            rightDrive.setPower(rightPower);
            backRightDrive.setPower(BackRightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f), backleft (%.2f), backright (%.2f), armposition (%d), rightjoystick_y (%.3f))",
                    leftPower, rightPower, BackLeftPower, BackRightPower, armPosition, gamepad1.right_stick_y);
            telemetry.update();
        }
    }
}