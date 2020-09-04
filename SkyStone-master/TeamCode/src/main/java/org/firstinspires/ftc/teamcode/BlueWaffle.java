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

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Blue Waffle Bridge", group="Pushbot")
@Disabled
public class BlueWaffle extends LinearOpMode {

    /* Declare OpMode members. */
    K9856_robot robot = new K9856_robot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();



    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE;
        robot.blinkinLedDriver.setPattern(robot.pattern);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Robot Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        // Robot moves forward towards stone
        robot.encoderplainMovement(K9856_robot.DRIVE_SPEED,   16, 4.0);  // S1: Forward 47 Inches with 5 Sec timeout
        sleep(100);

        robot.encoderStrafe(K9856_robot.DRIVE_SPEED, 10, true, 4.0);
        sleep(100);

        robot.encoderplainMovement(K9856_robot.DRIVE_SPEED,   16, 4.0);  // S1: Forward 47 Inches with 5 Sec timeout
        sleep(100);

        // Tail goes down on stone
        robot.tailThing.setPosition (K9856_robot.DOWN_POSITION);
        sleep(1700);

        robot.encoderplainMovement(K9856_robot.DRIVE_SPEED,   -22, 4.00);  // S1: Forward 47 Inches with 5 Sec timeout
        sleep(100);

        // Gyro telemetry
        double targetAngle = robot.getCurrentHeading();
        telemetry.addData("Current Heading-1",      "%.2f", targetAngle);
        telemetry.update();

        // Rotate 90 degrees
        robot .rotateGyro(K9856_robot.DRIVE_SPEED,           -45, 0.85);
        sleep(100);

        robot.encoderplainMovement(K9856_robot.DRIVE_SPEED,   -10, 4.0);  // S1: Forward 47 Inches with 4 Sec timeout
        sleep(100);

        // Robot delivers waffle
        robot.tailThing.setPosition(K9856_robot.UP_POSITION);
        sleep(1000);

        // Rotate 90 degrees
        robot .rotateGyro(K9856_robot.DRIVE_SPEED,           -50, 0.9);
        sleep(100);

        //Gyro telemetry
        targetAngle = robot.getCurrentHeading();
        telemetry.addData("Current Heading-2",       "%.2f", targetAngle);
        telemetry.update();

        //    robot.encoderplainMovement(K9856_robot.DRIVE_SPEED,     3, 30.0);

        //    robot.encoderStrafe(K9856_robot.DRIVE_SPEED,3,false,30.0);
       // sleep(100);

        robot.encoderplainMovement(K9856_robot.DRIVE_SPEED,   3, 4.0);
        sleep(100);

        robot.encoderplainMovement(K9856_robot.DRIVE_SPEED,   -15, 4.0);
        sleep(100);

        robot.encoderStrafe(K9856_robot.DRIVE_SPEED, 23, false, 4.0);
        sleep(100);

        // Robot moves back onto the line
        robot.encoderplainMovement(K9856_robot.DRIVE_SPEED,   -24, 4.0);
        sleep(100);

        robot.encoderStrafe(K9856_robot.DRIVE_SPEED, 17, false, 4.0);

        robot.tailThing.setPosition(K9856_robot.TAIL_POSITION);
        sleep(100);

        //robot.encoderStrafe(K9856_robot.DRIVE_SPEED, 48, true, 30.0);
        // robot.encoderStrafe(K9856_robot.DRIVE_SPEED, 48, false, 30.0);
        // robot.rotateGyro(K9856_robot.DRIVE_SPEED, 90, 30.0);
        // robot.rotateGyro(K9856_robot.DRIVE_SPEED, -90, 30.0);


        telemetry.addData("Path", "Complete");
        telemetry.update();

    }


    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */

}

