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

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p>
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class Hardware7Creator {
    /* Public OpMode members. */
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor backLeftMotor = null;
    public DcMotor backRightMotor = null;

    // public DcMotor carouselMotorLeft = null;
    public DcMotor carouselMotorRight = null;

    public DcMotor armMotor = null;

    //public Servo clawRightServo;
    //public Servo clawLeftServo;
    public CRServo crServo;

    public DistanceSensor distanceSensor;
    public ColorSensor colorSensor;

    //Speed control
    public static final double ARM_SPEED = 0.07; // 0-1
    public static final double ARM_FREELY_MOVE_SPEED = 0.045; // 0-1

    public static final double CAROSULE_SPEED = 0.35;
    public static final double INTAKE_SPEED = 0.30;

    public double MOTOR_MAX = 0.6; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode
    public double MOTOR_MAX_MANUAL = 0.50; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode

    public static final double MAX_POS_SERVO = 1.0;     // Maximum rotational position
    public static final double MIN_POS_SERVO = 0;     // Minimum rotational position
    public static final double SERVO_INCREMENT = 0.01;     // Minimum rotational position

    public static final int ARM_MAX_POS_AUTO = 650;
    public static final int ARM_MIN_POS_AUTO = 20;

    //Autonomus contants
    public static final int ARM_HIGH_POST = 345;
    public static final int ARM_MID_POST = 235;
    public static final int ARM_LOW_POST = 135;

    public static final float DUCK_LEFT_POST = 90;
    public static final float DUCK_MID_POST = 340;
    public static final float DUCK_RIGHT_POST = 500;

    double INIT_POST_SERVO = (MAX_POS_SERVO - MIN_POS_SERVO) / 2; // Start at halfway position

    /* local OpMode members. */
    HardwareMap hardwareMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public Hardware7Creator() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        // Save reference to Hardware map
        hardwareMap = hwMap;

        /* Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */

        frontLeftMotor = hardwareMap.dcMotor.get("frontleft_motor");
        frontRightMotor = hardwareMap.dcMotor.get("frontright_motor");
        backLeftMotor = hardwareMap.dcMotor.get("rearleft_motor");
        backRightMotor = hardwareMap.dcMotor.get("rearright_motor");//rearright_motor

        // carouselMotorLeft = hardwareMap.dcMotor.get("carousel_left_motor");
        carouselMotorRight = hardwareMap.dcMotor.get("carousel_right_motor");

        armMotor = hardwareMap.dcMotor.get("arm_motor");//arm_motor

        // clawLeftServo = hardwareMap.get(Servo.class, "claw_left_servo");//wrist_servo
        //clawRightServo = hardwareMap.get(Servo.class, "claw_right_servo");//claw_servo

        crServo = hardwareMap.get(CRServo.class, "cr_servo");//claw_servo

        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_range");
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        // Set the drive motor direction:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // These polarities are for the Neverest 20 motors

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set the drive motor run modes:
        // "RUN_USING_ENCODER" causes the motor to try to run at the specified fraction of full velocity
        // Note: We were not able to make this run mode work until we switched Channel A and B encoder wiring into
        // the motor controllers. (Neverest Channel A connects to MR Channel B input, and vice versa.)
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // carouselMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carouselMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //carouselMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        carouselMotorRight.setDirection(DcMotor.Direction.FORWARD);

        //armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //try
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}