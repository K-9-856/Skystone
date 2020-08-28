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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

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

@Autonomous(name="Red Skystone_vuforia Wall", group="Pushbot")
public class RedSkystone_vuforiaWall extends LinearOpMode {

    /* Declare OpMode members. */
    K9856_robot robot = new K9856_robot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    WebcamName webcamName = null;
    // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
    // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
    // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)
    //
    // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    //
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = true  ;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "ASM3iXv/////AAABmSYZ0S7EjkZjk3AKUr7v12J5DwwMX/nV7sI4vkBK8uehINzffoMu/EgVuF6ODY0Xen73Be0hhA41XHBEB5rqj/f4+vCwjG6aTGYC/fVrbvyUag4DP2ULDhUrsSn7oOeVK0CBqpuMCCW1P+NtEPWcMrDIs7MPUj/DHewoDFiyL/ZdQkimgXWreLXngq+vQF/TdcX94+V3fAz2chXkuA34iVuiD67zrJoeuR+CK1CoTSRkOqtuZ2m5ZGFtTl9+FkLJEZqh3w7m9128dAb0r0EvvSfBhLTwfmI9x7bFpxKNmxRGQcvY80AYqMXYuFye/8NhuIVfvzzzIaQvU8nRs4qfrlyu0ZnmYURxxNIKwqumGFQz";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ    = 6.42f * mmPerInch;
    private static final float bridgeY    = 23 * mmPerInch;
    private static final float bridgeX    = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField  = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia  = null;
    private boolean targetVisible     = false;
    private float phoneXRotate        = 0;
    private float phoneYRotate        = 0;
    private float phoneZRotate        = 0;

    VuforiaTrackable stoneTarget;
    VuforiaTrackables targetsSkyStone;
    @Override
    public void runOpMode() {

        double extraTravel = 0.0;

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        init_vuforia();
        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE;
        robot.blinkinLedDriver.setPattern(robot.pattern);

        //Set room of the camera
        CameraDevice.getInstance().setField( "opti-zoom","opti-zoom-on");
        CameraDevice.getInstance().setField( "zoom",     "30");


        // turn on vuforia
        targetsSkyStone.activate();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Robot Initialized");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        // Robot moves forward towards stone
        robot.encoderplainMovement(K9856_robot.DRIVE_SPEED,17, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        sleep(100);

        //Turn the flash on
        CameraDevice.getInstance().setFlashTorchMode(true);

        sleep(1000);

        if (isSkystoneVisible() == false)
        {
            robot.encoderStrafe(K9856_robot.DRIVE_SPEED, 12, true, 5.0);
            sleep(1000);
            extraTravel += 8;

            if (isSkystoneVisible() == false)
            {
                telemetry.addData("Status", "Assuming Block3 is Skystone");    //
                telemetry.update();
                robot.encoderStrafe(K9856_robot.DRIVE_SPEED, 12, true, 5.0);
                sleep(100);
                extraTravel += 8;
            }
            else
            {
                telemetry.addData("Status", "Block2 is Skystone!");    //
                telemetry.update();
                robot.pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                robot.blinkinLedDriver.setPattern(robot.pattern);
            }
        }
        else
        {
            telemetry.addData("Status", "Block1 is Skystone!");    //
            telemetry.update();
            robot.pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
            robot.blinkinLedDriver.setPattern(robot.pattern);

        }
        sleep(100);
        telemetry.addData("Action","Disabling Vuforia");    //
        telemetry.update();
        // Disable Tracking when we are done;
        targetsSkyStone.deactivate();
        //Turn the flash on
        CameraDevice.getInstance().setFlashTorchMode(false);

        //sleep(2000);

        robot.encoderStrafe(K9856_robot.DRIVE_SPEED, 9.5, true, 5.0);
        sleep(100);

        telemetry.addData("Action","Drive Forward till almost at stone");    //
        telemetry.update();
        robot.encoderplainMovement(K9856_robot.DRIVE_SPEED,20, 10.0);  // S1: Forward 47 Inches with 5 Sec timeout
        sleep(100);

      //  telemetry.addData("Action","Drive Forward little bit more");    //
      //  telemetry.update();
      //  robot.encoderplainMovement(K9856_robot.DRIVE_SPEED,4, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
      //  sleep(300);

        // Tail goes down on stone
        telemetry.addData("Action","Tail Down");    //
        telemetry.update();
        robot.tailThing.setPosition (K9856_robot.DOWN_POSITION);
        sleep(2000);

        // Robot moves back to avoid collisions with other stones
        telemetry.addData("Action","Forward 4.5 inches");    //
        telemetry.update();
        robot.encoderplainMovement(K9856_robot.DRIVE_SPEED,4.5,5.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        sleep(100);

        // Robot moves back to avoid collisions with other stones
        //telemetry.addData("Action","Backup 10 inches");    //
        //telemetry.update();
        robot.encoderplainMovement(K9856_robot.DRIVE_SPEED,-36,5.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        sleep(1000);

        // Gyro telemetry
        double targetAngle = robot.getCurrentHeading();
        telemetry.addData("Current Heading-1",     "%.2f", targetAngle);
        telemetry.update();

        // Rotate 90 degrees
        rotateGyro2(K9856_robot.DRIVE_SPEED,        95, 2.0);
        sleep(100);
        double targetAngle2 = robot.getCurrentHeading();
        telemetry.addData("Current Heading-1",     "%.2f", targetAngle);
        telemetry.addData("Current Heading-2",     "%.2f", targetAngle2);
        telemetry.update();
        sleep(100);

        robot.encoderStrafe(K9856_robot.DRIVE_SPEED, 15, false, 5.0);
        sleep(100);

        // Robot moves forwards past the line
        telemetry.addData("Action", "Drive Forward across the line");    //
        telemetry.update();
        robot.encoderplainMovement(K9856_robot.DRIVE_SPEED,(44 + extraTravel), 30.0);
        sleep(100);

        // Robot delivers stone
        telemetry.addData("Action", "Tail Up");    //
        telemetry.update();
        robot.tailThing.setPosition(K9856_robot.UP_POSITION);
        sleep(100);

        // Robot moves back onto the line
        telemetry.addData("Action", "Backup to the line");    //
        telemetry.update();
        robot.encoderplainMovement(K9856_robot.DRIVE_SPEED,-17, 30.0);
        sleep(100);

        robot.encoderStrafe(K9856_robot.DRIVE_SPEED, 20, false, 4.0);

        robot.tailThing.setPosition(K9856_robot.TAIL_POSITION);
        sleep(100);

        //robot.encoderStrafe(K9856_robot.DRIVE_SPEED, 48, true, 30.0);
       // robot.encoderStrafe(K9856_robot.DRIVE_SPEED, 48, false, 30.0);
        // robot.rotateGyro(K9856_robot.DRIVE_SPEED, 90, 30.0);
        // robot.rotateGyro(K9856_robot.DRIVE_SPEED, -90, 30.0);


        telemetry.addData("Path", "Complete");
        telemetry.update();



    }


    private void init_vuforia()
    {
        /*
         * Retrieve the camera we are to use.
         */
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;


        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");


        stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        // waitForStart();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.


    }

    private boolean isSkystoneVisible()
    {
        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;
        if (((VuforiaTrackableDefaultListener)stoneTarget.getListener()).isVisible()) {
            telemetry.addData("Visible Target", stoneTarget.getName());
            targetVisible = true;

            // getUpdatedRobotLocation() will return null if no new information is available since
            // the last time that call was made, or if the trackable is not currently visible.
            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)stoneTarget.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
            }

            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

        }
        telemetry.update();
        return targetVisible;
    }
    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void rotateGyro2   (double speed,
                              double degrees,
                              double timeoutS) {
        int leftfront;
        int rightfront;
        int leftback;
        int rightback;
        double targetAngle = robot.getCurrentHeading() + degrees;
        if (targetAngle > 180)
        {
            targetAngle -= 360;
        }
        else if (targetAngle < -180)
        {
            targetAngle += 360;
        }

        //telemetry.addData("Status", "Robot Initialized");    //
        //telemetry.update();
        // reset the timeout time and start motion.
        runtime.reset();

        robot.leftFront.setMode (DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode  (DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode (DcMotor.RunMode.RUN_USING_ENCODER);

        double originalAngle = robot.getCurrentHeading();

         //               Math.abs(robot.getCurrentHeading() - targetAngle) > 10.0 )

        while ((runtime.seconds() < timeoutS) &&
        (((targetAngle < originalAngle) && (robot.getCurrentHeading() >= targetAngle)) ||
                ((targetAngle >= originalAngle) && (robot.getCurrentHeading() <= targetAngle))))

        {
            double currentHeading = robot.getCurrentHeading();
            telemetry.addData("Current Heading","%.2f", currentHeading);
            telemetry.addData("Target Angle",   "%.2f", targetAngle);
            telemetry.addData("Degrees",        "%.2f", degrees);
            telemetry.update();

            if (degrees < 0) {
                robot.leftFront.setPower (-1*Math.abs(speed));
                robot.rightFront.setPower(Math.abs(speed))   ;
                robot.leftBack.setPower  (-1*Math.abs(speed));
                robot.rightBack.setPower (Math.abs(speed))   ;

            }
            else {          // Determine new target position, and pass to motor controller
                robot.leftFront.setPower (Math.abs(speed))   ;
                robot.rightFront.setPower(-1*Math.abs(speed));
                robot.leftBack.setPower  (Math.abs(speed))   ;
                robot.rightBack.setPower (-1*Math.abs(speed));

            }
            sleep(50);

        }

        // Stop all motion;
        robot.leftFront.setPower (0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower  (0);
        robot.rightBack.setPower (0);


        //  sleep(250);   // optional pause after each move

    }

}

