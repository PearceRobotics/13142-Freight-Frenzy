package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "AutonBlueCarousel", group = "Drive Code")
public class AutonBlueCarousel extends LinearOpMode {

    private DcMotor leftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor carouselDrive = null;
    private DcMotor leftArmDrive = null;
    private DcMotor rightArmDrive = null;
    private DcMotor intakeDrive = null;

    private int lFPos;
    private int rFPos;
    private int lBPos;
    private int rBPos;
    private int carPos;

    // operational constants
    private double fast = 1;
    private double slow = 0.5;
    private int angle;
    private double clicksPerInch = 75.53;
    private double clicksPerDeg = 9.64;

    @Override
    public void runOpMode() {
        telemetry.setAutoClear(true);

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        carouselDrive = hardwareMap.get(DcMotor.class, "carousel_drive");
        leftArmDrive = hardwareMap.get(DcMotor.class, "left_arm_drive");
        rightArmDrive = hardwareMap.get(DcMotor.class, "right_arm_drive");
        intakeDrive = hardwareMap.get(DcMotor.class, "intake_drive");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        carouselDrive.setDirection(DcMotor.Direction.REVERSE);
        leftArmDrive.setDirection(DcMotor.Direction.REVERSE);
        rightArmDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeDrive.setDirection(DcMotor.Direction.FORWARD);

        leftArmDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArmDrive.setTargetPosition(0);
        rightArmDrive.setTargetPosition(0);
        leftArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftArmDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        // *****************Dead reckoning list*************
        // Distances in inches, angles in deg, speed 0.0 to 1
        moveForward(12, fast);
        turnClockwise(75, fast);
        moveForward(-18.75, slow);
        turnClockwise(80, fast);
        moveForward(-10, slow);
        carouselClockwise(15, fast);
        moveForward(10, fast);
    }

    private void moveForward(double howFar, double speed) {
        // howLong is in seconds

        leftDrive.setTargetPosition(0);
        rightDrive.setTargetPosition(0);
        backLeftDrive.setTargetPosition(0);
        backRightDrive.setTargetPosition(0);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // fetch motor positions
        lFPos = leftDrive.getCurrentPosition();
        rFPos = rightDrive.getCurrentPosition();
        lBPos = backLeftDrive.getCurrentPosition();
        rBPos = backRightDrive.getCurrentPosition();

        // calculate new targets
        lFPos += howFar * clicksPerInch;
        rFPos += howFar * clicksPerInch;
        lBPos += howFar * clicksPerInch;
        rBPos += howFar * clicksPerInch;

        // move robot to new position
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
        backLeftDrive.setPower(speed);
        backRightDrive.setPower(speed);
        leftDrive.setTargetPosition(lFPos);
        rightDrive.setTargetPosition(rFPos);
        backLeftDrive.setTargetPosition(lBPos);
        backRightDrive.setTargetPosition(rBPos);

        // wait for move to complete
        while (leftDrive.isBusy() || rightDrive.isBusy() ||
                backLeftDrive.isBusy() || backRightDrive.isBusy()) {

            // Display it for the driver.
            telemetry.addLine("Move Foward");
            telemetry.addData("Target", "%7d :%7d", lFPos, rFPos, lBPos, rBPos);
            telemetry.addData("Actual", "%7d :%7d", leftDrive.getCurrentPosition(),
                    rightDrive.getCurrentPosition(), backLeftDrive.getCurrentPosition(),
                    backRightDrive.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }


    private void turnClockwise(int whatAngle, double speed) {
        // whatAngle is in degrees. A negative whatAngle turns counterclockwise.

        // fetch motor positions
        lFPos = leftDrive.getCurrentPosition();
        rFPos = rightDrive.getCurrentPosition();
        lBPos = backLeftDrive.getCurrentPosition();
        rBPos = backRightDrive.getCurrentPosition();

        // calculate new targets
        lFPos += whatAngle * clicksPerDeg;
        rFPos -= whatAngle * clicksPerDeg;
        lBPos += whatAngle * clicksPerDeg;
        rBPos -= whatAngle * clicksPerDeg;

        // move robot to new position
        leftDrive.setTargetPosition(lFPos);
        rightDrive.setTargetPosition(rFPos);
        backLeftDrive.setTargetPosition(lBPos);
        backRightDrive.setTargetPosition(rBPos);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
        backLeftDrive.setPower(speed);
        backRightDrive.setPower(speed);

        // wait for move to complete
        while (leftDrive.isBusy() || rightDrive.isBusy() ||
                backLeftDrive.isBusy() || backRightDrive.isBusy()) {

            // Display it for the driver.
            telemetry.addLine("Turn Clockwise");
            telemetry.addData("Target", "%7d :%7d", lFPos, rFPos, lBPos, rBPos);
            telemetry.addData("Actual", "%7d :%7d", leftDrive.getCurrentPosition(),
                    rightDrive.getCurrentPosition(), backLeftDrive.getCurrentPosition(),
                    backRightDrive.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    private void carouselClockwise(double howFar, double speed) {
        carouselDrive.setTargetPosition(0);
        carouselDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        carPos = carouselDrive.getCurrentPosition();
        carPos += howFar * clicksPerInch;
        carouselDrive.setTargetPosition(carPos);
        carouselDrive.setPower(speed);

        //while (Math.abs(carPos - carouselDrive.getCurrentPosition()) > 0.1) {

        try {
            Thread.sleep(5000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        //}

    }
}