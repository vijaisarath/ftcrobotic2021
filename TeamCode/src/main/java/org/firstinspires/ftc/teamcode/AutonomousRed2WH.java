package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "Autonomous Red Warehouse", group = "Linear Opmode")
// @Disabled
public class AutonomousRed2WH extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    Hardware7Creator myRobot = new Hardware7Creator();   // Use a Pushbot's hardware

    // declare motor speed variables
    int FR, FL, BR, BL;
    int ms = 1000;

    // operational constants
    double motorMax = 0.89; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP //https://www.wikihow.com/Determine-Gear-Ratio
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    //      "Ball",            "Cube",            "Duck",            "Marker"
    private static final String VUFORIA_KEY =
            "AaNE+N//////AAABmX8rTMa7Ikg+pRZR4cwSqpwDeN9zILuYkC9xNWPlqCmYG8tjh0BjaOrqou/+pPjH42YpyRrR92pSBpcBas66hKsGhXnfqhAuas2bXjOtH5+PQKIW8iNbO5UHhnwtgPey7SYe4dOkhE7hbScbI5Qj0tdSJqvUyowH47+cBeB5dstxutcJlQSF3xMlaGmV1qIGd4pX1zKgLfZZDCoKHGdrpiuh51d9ADUoulWBUbUhv8CiQhzhxW3ui6TP+eOFKXM7zaaUEta0da5I0Ra659xgLkoq2XZ7dDH++VGp33hs32swppTx1fOGSxrdEq0zkSYa8fVKjGXQCyhoRUjXvKvqBnZ7sKGBTEHAXLINcmIUY9yi";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        myRobot.init(hardwareMap);
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.3, 20.0 / 11.0);
        }

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

        //Red-Warehouse
        float duckLeftPosition = detectObject();
        telemetry.addData("DUck Left Position", "%.03f", duckLeftPosition);

        if (tfod != null) {
            tfod.shutdown();
        }

        String position = "";
        if (duckLeftPosition == 0 || duckLeftPosition < myRobot.DUCK_LEFT_POST) {
            telemetry.addData("Duck  Position", "Left");
            telemetry.update();
            position = "L";
            armPositionLow();
        } else if (duckLeftPosition <= myRobot.DUCK_MID_POST) {
            telemetry.addData("Duck  Position", "Middle");
            telemetry.update();
            position = "M";
            armPositionMid();
        } else if (duckLeftPosition <= myRobot.DUCK_RIGHT_POST) {
            telemetry.addData("Duck  Position", "Right");
            telemetry.update();
            position = "H";
            armPositionHigh();
        }
        sleep(300);

        moveFront(0.25, 11.15f);
        rotateAntiClockWise(0.2, 4.1f);

        //moveBack(0.4, 1.57f);
        if (position.equals("H")) {
            moveFront(0.4, 1.37f);
        } else if (position.equals("M")) {
            moveFront(0.4, 1.0f);
        } else {//low
            moveFront(0.4, 0.5f);
        }

        intakeClockwise();
        sleep(2000);

        intakeStop();

        moveBack(0.3, 0.90f);
        rotateClockWise(0.2, 4.2f);

        moveBack(0.3, 12.9f);
        moveRight(0.35, 12.5f);
        moveFront(0.25, 6f);
        rotateClockWise(0.2, 4.2f);

        armPositionDown();

        stopRobot();
        idle();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            idle();
        }
        if (isStopRequested()) {
            armPositionDown();
        }
    }

    private float detectObject() {
        String objectDetected = "";
        float duckLeftPosition = 0;
        int round = 1;
        boolean isDuckOrCubeDetected = false;
        if (opModeIsActive()) {
            while (opModeIsActive())// && !isDuckOrCubeDetected)// && round < 200000)
            {
                round++;
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        //telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            i++;

                            // check label to see if the camera now sees a Duck  
                            if (recognition.getLabel().equals("Duck") || recognition.getLabel().equals("Cube")) {
                                isDuckOrCubeDetected = true;
                                duckLeftPosition = recognition.getLeft();
                                objectDetected = recognition.getLabel();
                                // telemetry.addData("Object Detected", objectDetected);
                            } else {
                                duckLeftPosition = recognition.getLeft();
                                objectDetected = recognition.getLabel();
                                // telemetry.addData("Object Detected", objectDetected);      
                                isDuckOrCubeDetected = false;
                            }
                        }
                        if (isDuckOrCubeDetected) {
                            break;
                        } else if (round > 250000) {
                            break;
                        }
                    }
                }
            }
        }
        telemetry.addData("What Detected", objectDetected);
        telemetry.addData("Round", round);
        telemetry.addData("Object Detected", isDuckOrCubeDetected);
        return duckLeftPosition;
    }

    private void intakeClockwise() {
        myRobot.crServo.setDirection(DcMotorSimple.Direction.REVERSE);
        myRobot.crServo.setPower(myRobot.INTAKE_SPEED);
    }

    private void intakeAntiClockwise() {
        myRobot.crServo.setDirection(DcMotorSimple.Direction.FORWARD);
        myRobot.crServo.setPower(myRobot.INTAKE_SPEED);
    }

    private void intakeStop() {
        myRobot.crServo.setPower(0);
    }

    private void carouselMotorRightON() {
        myRobot.carouselMotorRight.setPower(myRobot.CAROSULE_SPEED);
        sendTelemetryData("carouselMotorRightON");
    }

    private void carouselMotorOFF() {
        myRobot.carouselMotorRight.setPower(0);
        myRobot.carouselMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    ///endregion

    ///region Arm code

    private void armPositionHigh() {
        moveArm(myRobot.ARM_SPEED, myRobot.ARM_HIGH_POST);
    }

    private void armPositionMid() {
        moveArm(myRobot.ARM_SPEED, myRobot.ARM_MID_POST);
    }

    private void armPositionLow() {
        moveArm(myRobot.ARM_SPEED, myRobot.ARM_LOW_POST);
    }

    private void armPositionDown() {
        moveArm(myRobot.ARM_SPEED, 0);
    }


    public void moveArm(double speed, int steps) {
        if (opModeIsActive()) {
            steps = Range.clip(steps, myRobot.ARM_MIN_POS_AUTO, myRobot.ARM_MAX_POS_AUTO);
            myRobot.armMotor.setTargetPosition(steps);
            myRobot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            myRobot.armMotor.setPower(Math.abs(speed));
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
        telemetry.addData("Gripper Pwr :", myRobot.crServo.getPower());
        telemetry.addData("Gripper Dir :", myRobot.crServo.getDirection());

        telemetry.addData("Carosule R Pwr :", myRobot.carouselMotorRight.getPower());
        telemetry.addData("Carosule R Dir :", myRobot.carouselMotorRight.getDirection());
        telemetry.addData("Distance :", myRobot.distanceSensor.getDistance(DistanceUnit.CM));


        telemetry.addData("Arm Power: ", myRobot.armMotor.getPower());
        telemetry.addData("Arm C  position: ", myRobot.armMotor.getCurrentPosition());
        telemetry.addData("Arm T position: ", myRobot.armMotor.getTargetPosition());
        telemetry.update();
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}