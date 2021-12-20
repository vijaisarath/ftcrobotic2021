package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Manual Drive", group = "Linear Opmode")
// @Disabled
public class ManualDrive extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    Hardware7Creator myRobot = new Hardware7Creator();

    // declare motor speed variables
    double FR;
    double FL;
    double BR;
    double BL;
    // declare joystick position variables
    double X1;
    double Y1;
    double X2;
    // operational constants

    static final int ARM_MAX_POS = 1099;
    static final int ARM_MIN_POS = 16;

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP //https://www.wikihow.com/Determine-Gear-Ratio
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415); //229

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        myRobot.init(hardwareMap);

        moveArm(myRobot.ARM_SPEED, 32);

        telemetry.addData("Arm C position: ", myRobot.armMotor.getCurrentPosition());
        telemetry.addData("Arm T position: ", myRobot.armMotor.getTargetPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            // Reset speed variables
            FL = 0;
            FR = 0;
            BL = 0;
            BR = 0;

            moveRobotWheels();

            //Game Pad 1 controls
            if (gamepad1.x) {
                myRobot.carouselMotorRight.setDirection(DcMotor.Direction.FORWARD);
                myRobot.carouselMotorRight.setPower(myRobot.CAROSULE_SPEED);
            } else if (gamepad1.b) {
                myRobot.carouselMotorRight.setDirection(DcMotor.Direction.REVERSE);
                myRobot.carouselMotorRight.setPower(myRobot.CAROSULE_SPEED);
            } else {
                myRobot.carouselMotorRight.setPower(0);
            }

            if (gamepad1.a) {
                myRobot.carouselMotorRight.setPower(0);
                myRobot.carouselMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            if (gamepad1.dpad_left) {
                moveLeft(0.08f);
            } else if (gamepad1.dpad_right) {
                moveRight(0.08f);
            } else if (gamepad1.dpad_up) {
                moveFront(0.08f);
            } else if (gamepad1.dpad_down) {
                moveBack(0.08f);
            }


            //Game Pad 2 controls
            //Arm movements
            if (gamepad2.a) {
                moveArm(myRobot.ARM_SPEED, 25);
            } else if (gamepad2.b) {
                moveArm(myRobot.ARM_SPEED, 126);
            } else if (gamepad2.x) {
                moveArm(myRobot.ARM_SPEED, 221);
            } else if (gamepad2.y) {
                moveArm(myRobot.ARM_SPEED, 344);
            }

            //Backward arm position
            if (gamepad2.dpad_left) {
                moveArm(myRobot.ARM_SPEED, 892);
            } else if (gamepad2.dpad_up) {
                moveArm(myRobot.ARM_SPEED, 750);
            } else if (gamepad2.dpad_right) {
                moveArm(myRobot.ARM_SPEED, 970);
            } else if (gamepad2.dpad_down) {
                moveArm(myRobot.ARM_SPEED, 1133);
            }

            if (gamepad2.right_stick_y < 0) {
                moveArm_freely(myRobot.ARM_FREELY_MOVE_SPEED, 60);
            } else if (gamepad2.right_stick_y > 0) {
                moveArm_freely(myRobot.ARM_FREELY_MOVE_SPEED, -60);
            }

            if (gamepad2.left_bumper) {
                clawOpen();
            } else if (gamepad2.right_bumper) {
                clawClose();
            } else {
                myRobot.crServo.setPower(0);
            }

            // Send some useful parameters to the driver station
            telemetry.addData("Arm Power: ", myRobot.armMotor.getPower());
            telemetry.addData("Arm C->T position: ", "%4d -> %4d", myRobot.armMotor.getCurrentPosition(), myRobot.armMotor.getTargetPosition());

            telemetry.addData("Gripper Servo Direction", myRobot.crServo.getDirection());

            telemetry.addData("FL | FR", "%.3f | %.3f", FL, FR);
            telemetry.addData("BL | BR", "%.3f | %.3f", BL, BR);

            telemetry.addData("Carousel R Pwr | Dir :", myRobot.carouselMotorRight.getPower() + "|" + myRobot.carouselMotorRight.getDirection());
            idle();
        }

        if (isStopRequested()) {
            myRobot.frontLeftMotor.setPower(0);
            myRobot.frontRightMotor.setPower(0);
            myRobot.backLeftMotor.setPower(0);
            myRobot.backRightMotor.setPower(0);
        }
    }

    private void clawOpen() {
        myRobot.crServo.setDirection(DcMotorSimple.Direction.REVERSE);
        myRobot.crServo.setPower(myRobot.INTAKE_SPEED);
    }

    private void clawClose() {
        myRobot.crServo.setDirection(DcMotorSimple.Direction.FORWARD);
        myRobot.crServo.setPower(myRobot.INTAKE_SPEED);
    }

    public void moveArm(double speed, int ticks) {
        if (opModeIsActive()) {
            ticks = Range.clip(ticks, ARM_MIN_POS, ARM_MAX_POS);
            myRobot.armMotor.setTargetPosition(ticks);
            myRobot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            myRobot.armMotor.setPower(Math.abs(speed));
            myRobot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void moveArm_freely(double speed, int steps) {
        if (opModeIsActive()) {
            int target = myRobot.armMotor.getCurrentPosition() + steps;
            target = Range.clip(target, ARM_MIN_POS, ARM_MAX_POS);

            myRobot.armMotor.setTargetPosition(target);
            myRobot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            myRobot.armMotor.setPower(Math.abs(speed));
            myRobot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    private void moveRight(float power) {
        FL = +power;
        BL = -power;

        FR = -power;
        BR = +power;
        // Clip motor power values to +-motorMax
        FL = Math.max(-myRobot.MOTOR_MAX_MANUAL, Math.min(FL, myRobot.MOTOR_MAX_MANUAL));
        FR = Math.max(-myRobot.MOTOR_MAX_MANUAL, Math.min(FR, myRobot.MOTOR_MAX_MANUAL));
        BL = Math.max(-myRobot.MOTOR_MAX_MANUAL, Math.min(BL, myRobot.MOTOR_MAX_MANUAL));
        BR = Math.max(-myRobot.MOTOR_MAX_MANUAL, Math.min(BR, myRobot.MOTOR_MAX_MANUAL));

        // Send values to the motors
        myRobot.frontLeftMotor.setPower(FL);
        myRobot.frontRightMotor.setPower(FR);
        myRobot.backLeftMotor.setPower(BL);
        myRobot.backRightMotor.setPower(BR);
        sleep(200);
    }

    private void moveLeft(float power) {
        FL = -power;
        BL = +power;

        FR = +power;
        BR = -power;
        // Clip motor power values to +-motorMax
        FL = Math.max(-myRobot.MOTOR_MAX_MANUAL, Math.min(FL, myRobot.MOTOR_MAX_MANUAL));
        FR = Math.max(-myRobot.MOTOR_MAX_MANUAL, Math.min(FR, myRobot.MOTOR_MAX_MANUAL));
        BL = Math.max(-myRobot.MOTOR_MAX_MANUAL, Math.min(BL, myRobot.MOTOR_MAX_MANUAL));
        BR = Math.max(-myRobot.MOTOR_MAX_MANUAL, Math.min(BR, myRobot.MOTOR_MAX_MANUAL));

        // Send values to the motors
        myRobot.frontLeftMotor.setPower(FL);
        myRobot.frontRightMotor.setPower(FR);
        myRobot.backLeftMotor.setPower(BL);
        myRobot.backRightMotor.setPower(BR);
        sleep(200);
    }

    private void moveFront(float power) {
        FL = BL = FR = BR = power;
        // Clip motor power values to +-motorMax
        FL = Math.max(-myRobot.MOTOR_MAX_MANUAL, Math.min(FL, myRobot.MOTOR_MAX_MANUAL));
        FR = Math.max(-myRobot.MOTOR_MAX_MANUAL, Math.min(FR, myRobot.MOTOR_MAX_MANUAL));
        BL = Math.max(-myRobot.MOTOR_MAX_MANUAL, Math.min(BL, myRobot.MOTOR_MAX_MANUAL));
        BR = Math.max(-myRobot.MOTOR_MAX_MANUAL, Math.min(BR, myRobot.MOTOR_MAX_MANUAL));

        // Send values to the motors
        myRobot.frontLeftMotor.setPower(FL);
        myRobot.frontRightMotor.setPower(FR);
        myRobot.backLeftMotor.setPower(BL);
        myRobot.backRightMotor.setPower(BR);
        sleep(200);
    }

    private void moveBack(float power) {
        FL = BL = FR = BR = -power;
        // Clip motor power values to +-motorMax
        FL = Math.max(-myRobot.MOTOR_MAX_MANUAL, Math.min(FL, myRobot.MOTOR_MAX_MANUAL));
        FR = Math.max(-myRobot.MOTOR_MAX_MANUAL, Math.min(FR, myRobot.MOTOR_MAX_MANUAL));
        BL = Math.max(-myRobot.MOTOR_MAX_MANUAL, Math.min(BL, myRobot.MOTOR_MAX_MANUAL));
        BR = Math.max(-myRobot.MOTOR_MAX_MANUAL, Math.min(BR, myRobot.MOTOR_MAX_MANUAL));

        // Send values to the motors
        myRobot.frontLeftMotor.setPower(FL);
        myRobot.frontRightMotor.setPower(FR);
        myRobot.backLeftMotor.setPower(BL);
        myRobot.backRightMotor.setPower(BR);
        sleep(200);
    }

    private void moveRobotWheels() {
        // Get joystick values
        Y1 = -gamepad1.right_stick_y;// * joyScale; // invert so up is positive
        X1 = gamepad1.right_stick_x;// * joyScale;
        X2 = gamepad1.left_stick_x * 0.45f;// * joyScale;

        //Combined code of all the above movements
        FL = Y1 + X1 + X2;
        BL = Y1 - X1 + X2;

        FR = Y1 - X1 - X2;
        BR = Y1 + X1 - X2;

        // Clip motor power values to +-motorMax
        FL = Math.max(-myRobot.MOTOR_MAX_MANUAL, Math.min(FL, myRobot.MOTOR_MAX_MANUAL));
        FR = Math.max(-myRobot.MOTOR_MAX_MANUAL, Math.min(FR, myRobot.MOTOR_MAX_MANUAL));
        BL = Math.max(-myRobot.MOTOR_MAX_MANUAL, Math.min(BL, myRobot.MOTOR_MAX_MANUAL));
        BR = Math.max(-myRobot.MOTOR_MAX_MANUAL, Math.min(BR, myRobot.MOTOR_MAX_MANUAL));

        // Send values to the motors
        myRobot.frontLeftMotor.setPower(FL);
        myRobot.frontRightMotor.setPower(FR);
        myRobot.backLeftMotor.setPower(BL);
        myRobot.backRightMotor.setPower(BR);
    }
}