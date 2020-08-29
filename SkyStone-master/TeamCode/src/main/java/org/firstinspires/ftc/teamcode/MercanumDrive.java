package org.firstinspires.ftc.teamcode;

        import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.hardware.CRServo;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Drive: MecanumDrive Red", group="Drive")
public class MercanumDrive extends OpMode {

    public DcMotor arm;
    public Servo Left;
    public Servo Right;
    private K9856_robot robotthing;
    private double grabberSpeed = 0.0 ;
    private final double SPEED  = 0.2 ;
    private ElapsedTime runtime = new ElapsedTime();


    public void init() {
        robotthing = new K9856_robot();
        robotthing.init(hardwareMap);
        runtime.reset();

        robotthing.pattern = RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE;
        robotthing.blinkinLedDriver.setPattern(robotthing.pattern);

    }

    public void loop() {
        if (gamepad1.a) {
            robotthing.speedLimit = K9856_robot.SPEED_ROBOT;
            robotthing.pattern = RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE;
            robotthing.blinkinLedDriver.setPattern(robotthing.pattern);
        } else if (gamepad1.b) {
            robotthing.speedLimit = K9856_robot.SLOW_ROBOT;
            robotthing.pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE;
            robotthing.blinkinLedDriver.setPattern(robotthing.pattern);
        }

        if (gamepad2.a) {
           robotthing.tailThing.setPosition (K9856_robot.DOWN_POSITION);
      //     robotthing.pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
      //     robotthing.blinkinLedDriver.setPattern(robotthing.pattern);

        } else if (gamepad2.b) {
           robotthing.tailThing.setPosition (K9856_robot.TAIL_POSITION);
      //     robotthing.pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
      //     robotthing.blinkinLedDriver.setPattern(robotthing.pattern);
        }
        else if (gamepad2.x) {
            robotthing.tailThing.setPosition(K9856_robot.TAIL_UP_POSITION);
      //      robotthing.pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
      //      robotthing.blinkinLedDriver.setPattern(robotthing.pattern);
        }
        else if (gamepad2.y)
        {
            robotthing.tailThing.setPosition(K9856_robot.UP_POSITION);
     //       robotthing.pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
     //       robotthing.blinkinLedDriver.setPattern(robotthing.pattern);
        }
        //robotthing.mechanumDrive(-Math.pow(gamepad1.left_stick_x, drive_power), -Math.pow(gamepad1.left_stick_y, drive_power), Math.pow(gamepad1.right_stick_x, drive_power));
        robotthing.mechanumDrive(gamepad1.left_stick_x,-gamepad1.left_stick_y, gamepad1.right_stick_x);

        if (gamepad1.x) {
            robotthing.yeeter.setPosition (K9856_robot.YEETER_DOWN);

        } else if (gamepad1.y) {
            robotthing.yeeter.setPosition (K9856_robot.YEETER_UP);
        }


        if (gamepad2.right_bumper) {
            if ((robotthing.armPosition < K9856_robot.ARM_MAX_HEIGHT) &&
                    (runtime.milliseconds() > 200.0))
            {
                robotthing.armPosition += K9856_robot.ARM_STEP;
                robotthing.armThing.setTargetPosition(robotthing.armPosition);
                robotthing.armThing.setPower(1.0);
                runtime.reset();
            }
        }
        else if (gamepad2.left_bumper)
        {
            if ((robotthing.armPosition > 0) &&
                    (runtime.milliseconds() > 200.0))
            {
                robotthing.armPosition -= K9856_robot.ARM_STEP;
                robotthing.armThing.setTargetPosition(robotthing.armPosition);
                robotthing.armThing.setPower(1.0);
                runtime.reset();
            }
        }
        else if (-gamepad2.right_stick_y > 0.2) {
            if (//(robotthing.armPosition < K9856_robot.ARM_MAX_HEIGHT) &&
                    (runtime.milliseconds() > 50.0)) {
                robotthing.armPosition += (40 * -gamepad2.right_stick_y);
                robotthing.armThing.setTargetPosition(robotthing.armPosition);
                robotthing.armThing.setPower(0.9);
                runtime.reset();
            }
        }
        else if (-gamepad2.right_stick_y < -0.1) {
            if (//(robotthing.armPosition > 0) &&
                    (runtime.milliseconds() > 50.0)) {
                robotthing.armPosition -= (-20 * -gamepad2.right_stick_y);
                robotthing.armThing.setTargetPosition(robotthing.armPosition);
                robotthing.armThing.setPower(1.0);
                runtime.reset();
            }
        }
        //else
        //{
        //    robotthing.armThing.setPower(0.0);
        //}

        if (gamepad2.right_trigger != 0.0) {
            //robotthing.grabber.setDirection(CRServo.Direction.REVERSE);
            robotthing.grabber.setPower(gamepad2.right_trigger);
        } else if (gamepad2.left_trigger != 0.0) {

            //robotthing.grabber.setDirection(CRServo.Direction.FORWARD);
            robotthing.grabber.setPower(-gamepad2.left_trigger);

        } else {

            robotthing.grabber.setPower(0);

        }

        telemetry.addData("Left Front Power",   "%.2f", robotthing.forwardLeftPower);
        telemetry.addData("Right Front Power",  "%.2f", robotthing.forwardRightPower);
        telemetry.addData("Left Back Power",    "%.2f", robotthing.backLeftPower);
        telemetry.addData("Right Back Power",   "%.2f", robotthing.backRightPower);

        telemetry.addData("Grabber Power",      "%.2f", robotthing.grabber.getPower());
        telemetry.addData("Arm Position",       "%d", robotthing.armThing.getCurrentPosition());
        telemetry.addData("Arm Target Position","%d", robotthing.armPosition);
        telemetry.addData("Arm Power",          "%.2f", robotthing.armThing.getPower());

        telemetry.addData("Current Heading",    "%.2f", robotthing.getCurrentHeading());
        telemetry.addData("Current Pitch",      "%.2f", robotthing.getCurrentPitch());
        telemetry.addData("Current Roll",       "%.2f", robotthing.getCurrentRoll());
    }
}
