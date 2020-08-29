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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.CancellationException;
import java.util.concurrent.ExecutorService;
import com.qualcomm.robotcore.util.ThreadPool;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class K9856_robot
{
    /* Public OpMode members. */
    public DcMotor  leftFront   = null;
    public DcMotor  rightFront  = null;
    public DcMotor  leftBack    = null;
    public DcMotor  rightBack   = null;
    public Servo    tailThing   = null;
    public DcMotor  armThing    = null;
    public CRServo  grabber     = null;
    public Servo    yeeter      = null;

    public static final double START_POSITION    =  0    ;
    public static final double DOWN_POSITION     =  0.9  ;
    public static final double UP_POSITION       =  0.1  ;
    public static final double TAIL_POSITION     =  0.4  ;
    public static final double TAIL_UP_POSITION  =  0.25 ;

    public static final double YEETER_DOWN = 0.9 ;
    public static final double YEETER_UP   = 0   ;


    public static final double SPEED_ROBOT = 1.0;
    public static final double SLOW_ROBOT  = 0.5;
    public double speedLimit = 0.5;

    public int armPosition = 0;
    public static final int ARM_TINY_STEP = 5;
    public static final int ARM_STEP = 10;
    public static final int ARM_MAX_HEIGHT = 850;

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 0.75;     // This is < 1.0 if geared UP 15 to 20 in gears
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    double forwardLeftPower  = 0.0;
    double forwardRightPower = 0.0;
    double backLeftPower     = 0.0;
    double backRightPower    = 0.0;

    private BNO055IMU imu;

    public RevBlinkinLedDriver blinkinLedDriver;
    public RevBlinkinLedDriver.BlinkinPattern pattern;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public K9856_robot(){

    }
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFront  = hwMap.get(DcMotor.class, "leftFront") ;
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        leftBack   = hwMap.get(DcMotor.class, "leftBack")  ;
        rightBack  = hwMap.get(DcMotor.class, "rightBack") ;
        tailThing  = hwMap.get(Servo.class,   "tailThing") ;

        armThing   = hwMap.get(DcMotor.class, "armThing");
        grabber    = hwMap.get(CRServo.class, "grabber");

        yeeter     = hwMap.get(Servo.class, "yeeter");

        // Set all motors to zero power
        leftFront.setPower (0);
        rightFront.setPower(0);
        leftBack.setPower  (0);
        rightBack.setPower (0);
     //   tailThing.setPosition(START_POSITION);
        armThing.setPower  (0);
        grabber.setPower   (0);


        leftFront.setMode (DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode  (DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode (DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armThing.setMode  (DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFront.setMode (DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode  (DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode (DcMotor.RunMode.RUN_USING_ENCODER);
        armThing.setMode  (DcMotor.RunMode.RUN_USING_ENCODER);

        //set wheel direction (reverse if on the right)
        leftFront.setDirection (DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection  (DcMotor.Direction.FORWARD);
        rightBack.setDirection (DcMotor.Direction.REVERSE);
        armThing.setDirection  (DcMotor.Direction.REVERSE);
        grabber.setDirection   (CRServo.Direction.REVERSE);

        armThing.setTargetPosition(armPosition);
        armThing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armThing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json";
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
  //        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu = ahwMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

        blinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        blinkinLedDriver.setPattern(pattern);
    }

    //finds the angle of each motor relative to the direction of movement
    public double relativeAngle (double motorAngle, double movementAngle, int motorPosition){
        //1440: number of ticks per rotation on the encoder
        //3.75: diameter of wheels
        //18.75: approximation of the diameter of the bot's path
        return
                motorAngle + movementAngle - (2*Math.PI * ((motorPosition/1440)*(Math.PI*4.15))/Math.PI*20.75);
    }

    public void mechanumDrive(double movX, double movY, double rotation) {
        /* *********************************************************************
                     FORWARD(+Y)   SIDEWAYS RIGHT(+X)   TURN RIGHT(+r)
        front right     +                 -                  -
        back left       +                 -                  +
        front left      +                 +                  +
        back right      +                 +                  -

        frPower = + y - x - r
        blPower = + y - x + r
        flPower = + y + x + r
        brPower = + y + x - r
        ************************************************************************
        */

        forwardRightPower = Range.clip((-1*movX + movY), -1, 1)  - rotation;
        backLeftPower     = Range.clip((-1*movX + movY), -1, 1) + rotation ;

        forwardLeftPower  = Range.clip((movX + movY),    -1, 1) + rotation ;
        backRightPower    = Range.clip((movX + movY),    -1, 1) - rotation ;

        leftFront.setPower(forwardLeftPower   * speedLimit);
        rightFront.setPower(forwardRightPower * speedLimit);
        leftBack.setPower(backLeftPower       * speedLimit);
        rightBack.setPower(backRightPower     * speedLimit);
    }

    public void stop() {
        mechanumDrive(0, 0, 0);
    }


    public void encoderplainMovement(double speed,
                                     double inches,
                                     double timeoutS) {
        int newLeftFront;
        int newRightFront;
        int newLeftBack;
        int newRightBack;

        // Ensure that the opmode is still active

            // Determine new target position, and pass to motor controller
            newLeftFront  = leftFront.getCurrentPosition () + (int) (inches * COUNTS_PER_INCH);
            newRightFront = rightFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newLeftBack  = leftBack.getCurrentPosition () + (int) (inches * COUNTS_PER_INCH);
            newRightBack = rightBack.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            leftFront.setTargetPosition (newLeftFront);
            rightFront.setTargetPosition(newRightFront);
            leftBack.setTargetPosition  (newLeftBack);
            rightBack.setTargetPosition (newRightBack);

            // Turn On RUN_TO_POSITION
            leftFront.setMode (DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode  (DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode (DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFront.setPower (Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftBack.setPower  (Math.abs(speed));
            rightBack.setPower (Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (
                    (runtime.seconds() < timeoutS) &&
                    (leftFront.isBusy() && rightFront.isBusy())) {


            }

            // Stop all motion;
            leftFront.setPower (0);
            rightFront.setPower(0);
            leftBack.setPower  (0);
            rightBack.setPower (0);


            //  sleep(250);   // optional pause after each move

    }


    public void encoderStrafe(double speed,
                              double inches, boolean isRight,
                              double timeoutS) {
        int leftfront;
        int rightfront;
        int leftback;
        int rightback;

        // Ensure that the opmode is still active

            if (isRight == true) {
                // Determine new target position, and pass to motor controller
                leftfront  = leftFront.getCurrentPosition () - (int) (inches * COUNTS_PER_INCH);
                rightfront = rightFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                leftback   = leftBack.getCurrentPosition  () + (int) (inches * COUNTS_PER_INCH);
                rightback  = rightBack.getCurrentPosition () - (int) (inches * COUNTS_PER_INCH);

            }
            else {          // Determine new target position, and pass to motor controller
                leftfront  = leftFront.getCurrentPosition () + (int) (inches * COUNTS_PER_INCH);
                rightfront = rightFront.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
                leftback   = leftBack.getCurrentPosition  () - (int) (inches * COUNTS_PER_INCH);
                rightback  = rightBack.getCurrentPosition () + (int) (inches * COUNTS_PER_INCH);
            }

            leftFront.setTargetPosition (leftfront);
            rightFront.setTargetPosition(rightfront);
            leftBack.setTargetPosition  (leftback);
            rightBack.setTargetPosition (rightback);

            // Turn On RUN_TO_POSITION
            leftFront.setMode (DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode  (DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode (DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFront.setPower (Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftBack.setPower  (Math.abs(speed));
            rightBack.setPower (Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (
                    (runtime.seconds() < timeoutS) &&
           //         (leftFront.isBusy() && rightFront.isBusy())) {

                            (Math.abs(leftFront.getCurrentPosition()- leftfront) > (2.0*COUNTS_PER_INCH))){


            }

            // Stop all motion;
            leftFront.setPower (0);
            rightFront.setPower(0);
            leftBack.setPower  (0);
            rightBack.setPower (0);


            //  sleep(250);   // optional pause after each move


    }
    public void rotateByTime   (double speed,
                              double degrees,
                              double timeoutS) {
        int leftfront;
        int rightfront;
        int leftback;
        int rightback;
        double targetAngle = getCurrentHeading() + degrees;

        //telemetry.addData("Status", "Robot Initialized");    //
        //telemetry.update();
        // reset the timeout time and start motion.
        runtime.reset();

        leftFront.setMode (DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode  (DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode (DcMotor.RunMode.RUN_USING_ENCODER);

        while (
                (runtime.seconds() < timeoutS)
        ){
            if (degrees < 0) {
                leftFront.setPower (-1*Math.abs(speed));
                rightFront.setPower(Math.abs(speed))   ;
                leftBack.setPower  (-1*Math.abs(speed));
                rightBack.setPower (Math.abs(speed))   ;

            }
            else {          // Determine new target position, and pass to motor controller
                leftFront.setPower (Math.abs(speed))   ;
                rightFront.setPower(-1*Math.abs(speed));
                leftBack.setPower  (Math.abs(speed))   ;
                rightBack.setPower (-1*Math.abs(speed));

            }
            sleep(100);

        }

        // Stop all motion;
        leftFront.setPower (0);
        rightFront.setPower(0);
        leftBack.setPower  (0);
        rightBack.setPower (0);


        //  sleep(250);   // optional pause after each move

    }
    public void rotateGyro   (double speed,
                                     double degrees,
                                     double timeoutS) {
        int leftfront;
        int rightfront;
        int leftback;
        int rightback;
        double targetAngle = getCurrentHeading() + degrees;

        //telemetry.addData("Status", "Robot Initialized");    //
        //telemetry.update();
        // reset the timeout time and start motion.
        runtime.reset();

        leftFront.setMode (DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode  (DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode (DcMotor.RunMode.RUN_USING_ENCODER);

        while (
                (runtime.seconds() < timeoutS) &&
                    Math.abs(getCurrentHeading() - targetAngle) > 5.0 ) {
            if (degrees < 0) {
                leftFront.setPower (-1*Math.abs(speed));
                rightFront.setPower(Math.abs(speed))   ;
                leftBack.setPower  (-1*Math.abs(speed));
                rightBack.setPower (Math.abs(speed))   ;

            }
            else {          // Determine new target position, and pass to motor controller
                leftFront.setPower (Math.abs(speed))   ;
                rightFront.setPower(-1*Math.abs(speed));
                leftBack.setPower  (Math.abs(speed))   ;
                rightBack.setPower (-1*Math.abs(speed));

            }
            sleep(50);

        }

        // Stop all motion;
        leftFront.setPower (0);
        rightFront.setPower(0);
        leftBack.setPower  (0);
        rightBack.setPower (0);


        //  sleep(250);   // optional pause after each move

    }


    public void encoderRotateTime   (double speed,
                                     double degrees,
                                     double timeoutS) {
        int leftfront;
        int rightfront;
        int leftback;
        int rightback;
        double inches = Math.abs(degrees*0.1396263402);

        // Ensure that the opmode is still active

            if (degrees < 0) {
                // Determine new target position, and pass to motor controller
                leftfront  = leftFront.getCurrentPosition () - (int) (inches * COUNTS_PER_INCH);
                rightfront = rightFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                leftback   = leftBack.getCurrentPosition  () - (int) (inches * COUNTS_PER_INCH);
                rightback  = rightBack.getCurrentPosition () + (int) (inches * COUNTS_PER_INCH);

            }
            else {          // Determine new target position, and pass to motor controller
                leftfront  = leftFront.getCurrentPosition () + (int) (inches * COUNTS_PER_INCH);
                rightfront = rightFront.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
                leftback   = leftBack.getCurrentPosition  () + (int) (inches * COUNTS_PER_INCH);
                rightback  = rightBack.getCurrentPosition () - (int) (inches * COUNTS_PER_INCH);
            }

            leftFront.setTargetPosition (leftfront);
            rightFront.setTargetPosition(rightfront);
            leftBack.setTargetPosition  (leftback);
            rightBack.setTargetPosition (rightback);

            // Turn On RUN_TO_POSITION
            leftFront.setMode (DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode  (DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode (DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFront.setPower (Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftBack.setPower  (Math.abs(speed));
            rightBack.setPower (Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (
                    (runtime.seconds() < timeoutS) &&
                    (leftFront.isBusy() && rightFront.isBusy())) {


            }

            // Stop all motion;
            leftFront.setPower (0);
            rightFront.setPower(0);
            leftBack.setPower  (0);
            rightBack.setPower (0);


            //  sleep(250);   // optional pause after each move

    }
    public double getCurrentHeading()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        // need to use rotation about X for heading, since our expansion hub is mounted vertically
  //      return (angles.secondAngle + 360) % 360;
        return (angles.secondAngle);
    }

    public double getCurrentPitch()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        // need to use rotation about X for heading, since our expansion hub is mounted vertically
   //     return (angles.firstAngle + 360) % 360;
        return (angles.firstAngle);
    }

    public double getCurrentRoll()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        // need to use rotation about X for heading, since our expansion hub is mounted vertically
  //      return (angles.thirdAngle + 360) % 360;
        return (angles.thirdAngle);
    }

    public double getAngleErrorFromTarget(double targetAngle) {
        double angleError = 0;

        angleError = (targetAngle - getCurrentHeading());
        angleError -= (360 * Math.floor(0.5 + ((angleError) / 360.0)));

        return angleError;
    }
}

