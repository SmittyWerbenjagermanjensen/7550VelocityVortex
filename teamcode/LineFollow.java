package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by jzerez17 on 4/16/16.
 */


@TeleOp(name = "TestCode", group = "code")
public class LineFollow extends OpMode {
    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;
    DcMotor motor4;
    OpticalDistanceSensor eagleEyes;

    double leftSpeed;
    double rightSpeed;
    double shooterSpeed = 0;
    boolean timer;
    public ElapsedTime rampUp = new ElapsedTime();


    public LineFollow() throws InterruptedException {

    }

    @Override
    public void init() {
        motor1 = hardwareMap.dcMotor.get("motor_left");
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor2 = hardwareMap.dcMotor.get("motor_right");
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setDirection(DcMotor.Direction.REVERSE);

        motor3 = hardwareMap.dcMotor.get("motor_condor");
        motor3.setDirection(DcMotorSimple.Direction.REVERSE);

        motor4 = hardwareMap.dcMotor.get("motor_intake");
        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        eagleEyes = hardwareMap.opticalDistanceSensor.get("eagle_eyes");
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        eagleEyes.enableLed(true);
    }

    @Override
    public void loop()
    {
        if (gamepad1.a)
        {
            rightSpeed = Math.round(((-100 * (Math.pow(eagleEyes.getLightDetected(), 0.5))) + 39.9));
            leftSpeed = Math.round(((100 * (Math.pow(eagleEyes.getLightDetected(), 0.5))) - 14.9));
            motor1.setPower(leftSpeed / 100);
            motor2.setPower(rightSpeed / 100);

            /* OLD LINE FOLLOWING NUMBERS

                    leftSpeed = Math.round(((67.5676 * (Math.pow(eagleEyes.getLightDetected(), 0.5))) - 8.78378));
                    rightSpeed = Math.round(((-67.5676 * (Math.pow(eagleEyes.getLightDetected(), 0.5))) + 33.7838));
            */

        }
        else
        {
            motor1.setPower(0);
            motor2.setPower(0);
        }

        if (gamepad2.a)
        {
            rampUp.startTime();

            if (rampUp.time() < 0.5)
            {
                motor4.setPower(0.2);
            }
            else if (rampUp.time() < 1)
            {
                motor4.setPower(0.35);
            }
            else if (rampUp.time() < 1.5)
            {
                motor4.setPower(0.4);
            }
            else
            {
                motor4.setPower(0.5);
            }

        }
        else if (gamepad2.b)
        {
            rampUp.startTime();
            rampUp.startTime();

            if (rampUp.time() < 0.5)
            {
                motor4.setPower(0.2);
            }
            else if (rampUp.time() < 1)
            {
                motor4.setPower(0.35);
            }
            else if (rampUp.time() < 1.5)
            {
                motor4.setPower(0.55);
            }
            else if (rampUp.time() < 2)
            {
                motor4.setPower(0.60);
            }
            else
            {
                motor4.setPower(0.75);
            }

        } else {
            motor4.setPower(0);
            rampUp.reset();

        }


        telemetry.addData("leftspeeed", leftSpeed);
        telemetry.addData("rightspeeed", rightSpeed);
        telemetry.addData("lightraw", eagleEyes.getLightDetected());
        telemetry.addData("light", Math.pow(eagleEyes.getLightDetected(), 0.5));
        telemetry.addData("rime", rampUp.time());
        telemetry.addData("speed", motor4.getPower());
        telemetry.addData("distance", motor4.getCurrentPosition());



    }
}
