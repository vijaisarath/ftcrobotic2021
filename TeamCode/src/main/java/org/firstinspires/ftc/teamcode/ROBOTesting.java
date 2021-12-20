package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This program provides driver station control of  Mecanum Drive Prototype.
 * <p>
 * This robot uses four VEX Mecanum wheels, each direct driven by Neverest 20 motors.
 * It is designed as a linear op mode, and uses RUN_WITH_ENCODER motor operation.
 * <p>
 * The gamepad1 right joystick is used for translation movement, while the left joystick x-axis controls rotation.
 */

@Autonomous(name = "ROBOTesting Autonomous", group = "Linear Opmode")
// @Disabled
public class ROBOTesting extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    Hardware7Creator myRobot = new Hardware7Creator();   // Use a Pushbot's hardware

    // declare motor speed variables
    int FR, FL, BR, BL;
    int ms = 1000;

    // operational constants
    double motorMax = 0.5; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP //https://www.wikihow.com/Determine-Gear-Ratio
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        myRobot.init(hardwareMap);

        myRobot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myRobot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myRobot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myRobot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Reset speed variables
        FL = 0;
        FR = 0;
        BL = 0;
        BR = 0;

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Red1", "Starting at %7d : %7d : %7d : %7d",
                myRobot.frontLeftMotor.getCurrentPosition(),
                myRobot.frontRightMotor.getCurrentPosition(),
                myRobot.backLeftMotor.getCurrentPosition(),
                myRobot.backRightMotor.getCurrentPosition()
        );
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        while (opModeIsActive() && runtime.seconds() < 10.0) {

            moveFront(1.25, 2f);
            moveBack(1.25, 2f);
            moveLeft(1.25, 2f);
            moveRight(1.25, 2f);

            rotateClockWise(1.2, 9f);
            rotateAntiClockWise(1.2, 9f);

            armPositionLow();
            armPositionMid();
            armPositionHigh();


            armPositionMid();
            armPositionLow();
            armPositionDown();

            carouselMotorRightON();
            sleep(1000);
            carouselMotorOFF();
            sleep(1000);
            carouselMotorRightON();
            sleep(1000);
            carouselMotorOFF();

            stopRobot();
            idle();
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            idle();
        }
        if (isStopRequested()) {
            stopRobot();
            armPositionDown();
            idle();
        }
        stopRobot();
        idle();
    }

//    private void clawClose() {
//        myRobot.clawLeftServo.setPosition(0.35);
//        myRobot.clawRightServo.setPosition(0.51);
//    }
//
//    private void clawOpen() {
//        myRobot.clawLeftServo.setPosition(0.75);
//        myRobot.clawRightServo.setPosition(0.25);
//    }
//
//    private void clawExpand() {
//        myRobot.clawLeftServo.setPosition(0.95);
//        myRobot.clawRightServo.setPosition(0.15);
//    }
//
//    private void moveClawLeftServoToPosition(double targetPstn) {
//        targetPstn = Range.clip(targetPstn, myRobot.MIN_POS_SERVO, myRobot.MAX_POS_SERVO);
//        myRobot.clawLeftServo.setPosition(targetPstn);
//    }
//
//    private void moveClawRightServoToPosition(double targetPstn) {
//        targetPstn = Range.clip(targetPstn, myRobot.MIN_POS_SERVO, myRobot.MAX_POS_SERVO);
//        myRobot.clawRightServo.setPosition(targetPstn);
//    }

//    ///region Carousel code
//    private void carouselMotorLeftON() {
//        myRobot.carouselMotorLeft.setPower(myRobot.CAROSULE_SPEED);
//        sendTelemetryData("carouselMotorLeftON");
//    }

    private void carouselMotorRightON() {
        myRobot.carouselMotorRight.setPower(myRobot.CAROSULE_SPEED);
        sendTelemetryData("carouselMotorRightON");
    }

    private void carouselMotorOFF() {
//        myRobot.carouselMotorLeft.setPower(0);
 //       myRobot.carouselMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        myRobot.carouselMotorRight.setPower(0);
        myRobot.carouselMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    ///endregion

    ///region Arm code

    private void armPositionHigh() {
        moveArm(myRobot.ARM_SPEED, 2.1f);
    }

    private void armPositionMid() {
        moveArm(myRobot.ARM_SPEED, 1.34f);
    }

    private void armPositionLow() {
        moveArm(myRobot.ARM_SPEED, 0.97f);
    }

    private void armPositionDown() {
        moveArm(myRobot.ARM_SPEED, 0);
    }


    public void moveArm(double speed, float steps) {
        if (opModeIsActive()) {
            int distTick = (int) (steps * COUNTS_PER_INCH);
            distTick = Range.clip(distTick, 20, 650);
            myRobot.armMotor.setTargetPosition(distTick);
            myRobot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            myRobot.armMotor.setPower(Math.abs(speed));

            while (opModeIsActive() && (myRobot.armMotor.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Arm", "Running to %7d ", distTick);
                telemetry.addData("arm busy", myRobot.armMotor.isBusy());
                telemetry.addData("arm cp", "Running at %7d ", myRobot.armMotor.getCurrentPosition());
                telemetry.addData("arm tp", "Running at %7d ", myRobot.armMotor.getTargetPosition());
                telemetry.update();
            }
            // myRobot.armMotor.setPower(0); // If you enable the arm is NOT standing in required position
            myRobot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    ///endregion

    ///region Robot Wheel code
    private void stopRobot() {
        FL = BL = FR = BR = 0;
        assignPowerToMotors(0);
        sendTelemetryData("stopRobot");
    }

    private void moveFront(int power) {
        FL = power;
        BL = power;
        FR = power;
        BR = power;
        assignPowerToMotors(power);
        sendTelemetryData("moveFront");
    }

    private void moveFront(double power, float distanceInInch) {
        int distTick = (int) (distanceInInch * COUNTS_PER_INCH);
        FL = BL = FR = BR = distTick;
        setTargetToMotors(power, "moveFront");
    }

    private void moveBack(double power, float distanceInInch) {
        int distTick = (int) (distanceInInch * COUNTS_PER_INCH);
        FL = BL = FR = BR = -distTick;
        setTargetToMotors(power, "moveBack");
    }

    private void moveRight(double power, float distanceInInch) {
        int distTick = (int) (distanceInInch * COUNTS_PER_INCH);
        FL = +distTick;
        BL = -distTick;

        FR = -distTick;
        BR = +distTick;
        setTargetToMotors(power, "moveRight");
    }

    private void moveLeft(double power, float distanceInInch) {
        int distTick = (int) (distanceInInch * COUNTS_PER_INCH);
        FL = -distTick;
        BL = +distTick;

        FR = +distTick;
        BR = -distTick;

        setTargetToMotors(power, "moveLeft");
    }

    //Pivot Movements
    private void moveFrontLeft(int power, float distanceInInch) {
        FL = BR = 0;
        BL = FR = power;
        setTargetToMotors(distanceInInch, "moveFrontLeft");
    }

    private void moveBackLeft(int power, float distanceInInch) {
        FL = BR = 0;
        BL = FR = -power;
        setTargetToMotors(distanceInInch, "moveBackLeft");
    }

    private void moveBackRight(int power, float distanceInInch) {
        FL = BR = -power;
        BL = FR = 0;
        setTargetToMotors(distanceInInch, "moveBackRight");
    }

    private void moveFrontRight(int power, float distanceInInch) {
        FL = BR = power;
        BL = FR = 0;
        setTargetToMotors(distanceInInch, "moveFrontRight");
    }

    private void rotateClockWise(double power, float distanceInInch) {
        int distTick = (int) (distanceInInch * COUNTS_PER_INCH);
        FL = BL = distTick;
        FR = BR = -distTick;
        setTargetToMotors(power, "rotateClockWise");
    }

    private void rotateAntiClockWise(double power, float distanceInInch) {
        int distTick = (int) (distanceInInch * COUNTS_PER_INCH);
        FL = BL = -distTick;
        FR = BR = distTick;
        setTargetToMotors(power, "rotateClockWise");
    }

    private void assignPowerToMotors(double power) {
        // Clip motor power values to +-motorMax
//        FL = Math.max(-motorMax, Math.min(FL, motorMax));
//        FR = Math.max(-motorMax, Math.min(FR, motorMax));
//        BL = Math.max(-motorMax, Math.min(BL, motorMax));
//        BR = Math.max(-motorMax, Math.min(BR, motorMax));

        power = Math.max(-motorMax, Math.min(power, motorMax));

        // Send values to the motors
        myRobot.frontLeftMotor.setPower(power);
        myRobot.frontRightMotor.setPower(power);
        myRobot.backLeftMotor.setPower(power);
        myRobot.backRightMotor.setPower(power);
    }

    private void setTargetToMotors(double power, String commandName) {
        myRobot.frontLeftMotor.setTargetPosition(FL);
        myRobot.frontRightMotor.setTargetPosition(FR);
        myRobot.backLeftMotor.setTargetPosition(BL);
        myRobot.backRightMotor.setTargetPosition(BR);

        myRobot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myRobot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myRobot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myRobot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        assignPowerToMotors(power);

        while (opModeIsActive() &&
                (myRobot.frontLeftMotor.isBusy() && myRobot.frontRightMotor.isBusy() && myRobot.backRightMotor.isBusy() && myRobot.backLeftMotor.isBusy())) {
            sendTelemetryData(commandName);
        }

        stopRobot();

        myRobot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myRobot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myRobot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myRobot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        myRobot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        myRobot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        myRobot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        myRobot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //endregion

    private void sendTelemetryData(String param) {

        telemetry.addData("Movement", param);
//        telemetry.addData("FL | FR", "%.3f | %.3f", FL, FR);
//        telemetry.addData("BL | BR", "%.3f | %.3f", BL, BR);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData(param, "Current post %7d : %7d : %7d : %7d",
                myRobot.frontLeftMotor.getCurrentPosition(),
                myRobot.frontRightMotor.getCurrentPosition(),
                myRobot.backLeftMotor.getCurrentPosition(),
                myRobot.backRightMotor.getCurrentPosition()
        );
        telemetry.addData(param, "Target post %7d : %7d : %7d : %7d",
                myRobot.frontLeftMotor.getTargetPosition(),
                myRobot.frontRightMotor.getTargetPosition(),
                myRobot.backLeftMotor.getTargetPosition(),
                myRobot.backRightMotor.getTargetPosition()
        );
        //telemetry.addData("Carosule L Pwr :", myRobot.carouselMotorLeft.getPower());
        //telemetry.addData("Carosule L Dir :", myRobot.carouselMotorLeft.getDirection());

        telemetry.addData("Carosule R Pwr :", myRobot.carouselMotorRight.getPower());
        telemetry.addData("Carosule R Dir :", myRobot.carouselMotorRight.getDirection());

        telemetry.addData("Arm Power: ", myRobot.armMotor.getPower());
        telemetry.addData("Arm C  position: ", myRobot.armMotor.getCurrentPosition());
        telemetry.addData("Arm T position: ", myRobot.armMotor.getTargetPosition());
        telemetry.update();
    }
}