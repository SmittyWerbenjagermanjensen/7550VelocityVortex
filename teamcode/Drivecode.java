package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */

@TeleOp(name = "Drivecode", group = "Code")
public class Drivecode extends OpMode {




    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor motorCondor;
    DcMotor motorIntake;
    Servo condorFeeder;
    Servo buttonPusher;
    TouchSensor birdBrain;
    OpticalDistanceSensor eagleEyes;
    ColorSensor color;

    int shooterTicks = 1652;
    double feederPos = 1;
    boolean shootingDelay = false;

    boolean servoPosition;
    boolean shooting = false;
    boolean buttonPos = false;
    public ElapsedTime calibrate = new ElapsedTime();
    public ElapsedTime shooterTime = new ElapsedTime();
    public ElapsedTime toggler = new ElapsedTime();




    /**
     * Constructor
     */
    public Drivecode() {

    }


    @Override
    public void init() {
        motorRight = hardwareMap.dcMotor.get("motor_right");
        motorCondor = hardwareMap.dcMotor.get("motor_condor");
        motorCondor.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeft = hardwareMap.dcMotor.get("motor_left");
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorIntake = hardwareMap.dcMotor.get("motor_intake");
        motorCondor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        condorFeeder = hardwareMap.servo.get("servo_feeder");
        birdBrain = hardwareMap.touchSensor.get("bird_brain");
        eagleEyes = hardwareMap.opticalDistanceSensor.get("eagle_eyes");
        buttonPusher = hardwareMap.servo.get("servo_button");
        color = hardwareMap.colorSensor.get("sensor_color");


        eagleEyes.enableLed(false);
        color.enableLed(false);
        buttonPusher.setPosition(1);

    }

    @Override
    public void start() {
        motorCondor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        toggler.startTime();
        shooterTime.startTime();
    }
    @Override
    public void loop() {
        // tank drive
        // note that if y equal -1 then joystick is pushed all of the way forward.
        float left = -gamepad1.left_stick_y;
        float right = -gamepad1.right_stick_y;

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        right = (float)scaleInput(right);
        left =  (float)scaleInput(left);


        motorRight.setPower(right);
        motorLeft.setPower(left);

        //Driver 2, button a: trigger's "shooting" state
        if ((gamepad2.a) && (toggler.time() > 0.5))
        {
            shooting = true;
            shooterTime.reset();
            toggler.reset();
            //motorCondor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //motorCondor.setPower(0.4);
            //motorCondor.setTargetPosition(shooterTicks);
            //motorCondor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (shooterTime.time() > 0.4)
        {
            shootingDelay = true;
        }
        else
        {
            shootingDelay = false;
        }

        if ((!shootingDelay || !birdBrain.isPressed()) && shooting)
        {
            motorCondor.setPower(0.3);
        }
        if (shootingDelay && birdBrain.isPressed())
        {
            motorCondor.setPower(0);
            shooting = false;
        }

        //if auto-shooting is diabled, return control to joystick. Else, turn one rotation
        if (!shooting) {
            motorCondor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorCondor.setPower(-gamepad2.left_stick_y);
        }

        // LSKJFLAKJSD;FLJASDLFGAJSD;LKJG;LAJSG;LKJASL'KGJASGALJL
        if (shooterTime.time() < 0.3)
        {
            feederPos = 1;
        }
        else if ((shooterTime.time() > 0.3) && (shooterTime.time() < 0.8))
        {
            feederPos = 0.42;
        }
        else
        {
            feederPos = 1;
        }
        condorFeeder.setPosition(feederPos);


        //Driver 2, button b: manual encoder reset.
        if ((gamepad2.b) && (toggler.time() > 0.5))
        {
            motorCondor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            toggler.reset();
        }

        if ((gamepad1.a) && (toggler.time() > 0.5))
        {
            buttonPos = !buttonPos;
            toggler.reset();
        }

        if (buttonPos)
        {
            buttonPusher.setPosition(0);
            motorIntake.setPower(0);
        } else {
            buttonPusher.setPosition(1);
            motorIntake.setPower(-gamepad2.right_stick_y);
        }

        //Triggers the end of auto-shooting cycle. When motors reach their position, or go over specified time limit
        /*
        if (((motorCondor.getCurrentPosition() > (shooterTicks - 5)) && (motorCondor.getCurrentPosition() < (shooterTicks + 5)) && (shooting)) || ((shooterTime.seconds() > 1.3) && (shooting)))
        {
            motorCondor.setPower(0);
            motorCondor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooting = false;
        }
        */



        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", left));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
        telemetry.addData("Shifter Servo Status",buttonPos);
        telemetry.addData("condor", motorCondor.getCurrentPosition());
        telemetry.addData("a", gamepad2.a);
        telemetry.addData("shooting", shooting);
        telemetry.addData("shooter tmier", shooterTime.time());
        telemetry.addData("toggler", toggler.time());
        telemetry.addData("feederPos", feederPos);
        telemetry.addData("eagleeyes", Math.pow(eagleEyes.getLightDetected(), 0.5));
        telemetry.addData("butts", eagleEyes.getLightDetected());
        telemetry.addData("eagleeyes raw", eagleEyes.getRawLightDetected());


    }

    @Override
    public void stop() {

    }

    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        int index = (int) (dVal * 16.0);

        if (index < 0) {
            index = -index;
        }

        if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

}