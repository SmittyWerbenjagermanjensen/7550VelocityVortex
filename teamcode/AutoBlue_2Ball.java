package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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


@Autonomous(name = "blueAuto_2Ball", group = "code")
public class AutoBlue_2Ball extends OpMode {
    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;
    DcMotor motor4;
    Servo buttonPusher;
    Servo condorFeeder;
    ColorSensor color;
    TouchSensor birdBrain;
    OpticalDistanceSensor eagleEyes;



    int moveNumber;
    int pathSegment = 0;
    int targetPosLeft = 0;
    int targetPosRight = 0;
    int lastPosLeft = 0;
    int lastPosRight = 0;
    double targetSpeed = 0;
    double leftSpeed;
    double rightSpeed;
    int shootingNumber = 0;
    boolean redBeacon = false;
    boolean blueBeacon = false;
    boolean rampdowntown;
    int redSum = 0;
    int blueSum = 0;
    int i = 0;




    final static int left = 0;
    final static int right = 1;
    final static double oneEightyDegreeTurn = (25.875*Math.PI);
    final static double ninetyDegreeTurn = (oneEightyDegreeTurn/2);
    final static double fortyFiveDegreeTurn = (oneEightyDegreeTurn/4);
    final static double twentyTwoPointFiveDegreeTurn = (oneEightyDegreeTurn/8);

    //declare double arrays
    //{left dist, right dist, speed}

    //Shooting Path
    final static double[] a1 = {-12, -12, 50};
    final static double[] a2 = {0, -fortyFiveDegreeTurn, 45};
    final static double[] a3 = {-72,-72, 56};
    final static double[] a4 ={-3 * twentyTwoPointFiveDegreeTurn, 3 * twentyTwoPointFiveDegreeTurn, 50};
    final static double[][] shootingPath = {a1, a2, a3, a4};

    //Repair Path
    final static double[] b1 = {-35, -35, 50};
    final static double[] b2 = {0, ninetyDegreeTurn, 40};
    final static double[][] beacon1Path = {b1, b2};

    final static double[] d1 = {-6, -9, 50};
    final static double[] d2 = {-fortyFiveDegreeTurn, fortyFiveDegreeTurn, 50};
    final static double[] d3 = {25, 25, 50};
    final static double[] d4 = {-ninetyDegreeTurn, ninetyDegreeTurn, 50};
    final static double[] d5 = {-22, -22, 50};
    final static double[][] beacon2Path = {d1, d2, d3, d4, d5};

    final static double ticksPerInch = (1120*11/(Math.PI*4*16));
    final static int speedFactor = 100;
    public ElapsedTime colorReading = new ElapsedTime();
    public ElapsedTime shooterTime = new ElapsedTime();


    public AutoBlue_2Ball() throws InterruptedException
    {

    }

    @Override
    public void init ()
    {
        motor1 = hardwareMap.dcMotor.get("motor_left");
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2 = hardwareMap.dcMotor.get("motor_right");
        motor3 = hardwareMap.dcMotor.get("motor_condor");
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotorSimple.Direction.REVERSE);
        motor4 = hardwareMap.dcMotor.get("motor_intake");
        birdBrain = hardwareMap.touchSensor.get("bird_brain");
        eagleEyes = hardwareMap.opticalDistanceSensor.get("eagle_eyes");
        buttonPusher = hardwareMap.servo.get("servo_button");
        condorFeeder = hardwareMap.servo.get("servo_feeder");
        color = hardwareMap.colorSensor.get("sensor_color");

        condorFeeder.setPosition(1);
        buttonPusher.setPosition(1);


        moveNumber = 0;
        pathSegment = 0;
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void init_loop()
    {

    }

    @Override
    public void start()
    {
        moveNumber = 1;
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterTime.startTime();
        eagleEyes.enableLed(true);

        color.enableLed(false);


    }

    @Override
    public void loop()
    {
        switch (moveNumber) {
            //Go to the beacon
            case 1:
                moveMatrix(4, 1, 18, shootingPath);
                buttonPusher.setPosition(0.5);
                break;


            // Follow line, then push button
            case 2:
                colorReading.startTime();
                motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                if (colorReading.time() < 3)
                {
                    lastPosLeft = 0;
                    lastPosRight = 0;
                    pathSegment = 0;

                    rightSpeed = Math.round(((-100 * (Math.pow(eagleEyes.getLightDetected(), 0.5))) + 39.9));
                    leftSpeed = Math.round(((100 * (Math.pow(eagleEyes.getLightDetected(), 0.5))) - 14.9));
                    motor1.setPower(leftSpeed / 100);
                    motor2.setPower(rightSpeed / 100);
                    if (colorReading.time() > 2.5)
                    {
                        blueSum += color.blue();
                        redSum += color.red();
                    }

                }
                else if (colorReading.time() < 3.75)
                {
                    motor1.setPower(-0.2);
                    motor2.setPower(-0.2);

                    if (redSum > blueSum) {
                        buttonPusher.setPosition(0);
                    } else
                    {
                        buttonPusher.setPosition(1);
                    }
                }
                else if (colorReading.time() < 4.75)
                {
                    motor1.setPower(0.65);
                    motor2.setPower(0.65);
                }
                else
                {
                    motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    moveNumber++;
                }
                telemetry.addData("leftspeed", leftSpeed / 100);
                telemetry.addData("rightspeed", rightSpeed / 100);
                break;

            //Get into shooting position
            case 3:
                moveMatrix(2, 1, 17, beacon1Path);
                shooterTime.reset();
                break;

            //shoot the balls
            case 4:
                pathSegment = 0;
                motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lastPosLeft = 0;
                lastPosRight = 0;
                if (i < 2)
                {
                    if (((shooterTime.time() > 0.4) && (shooterTime.time() < 0.7)) || !birdBrain.isPressed()) {
                        motor3.setPower(0.4);
                    }

                    if (shooterTime.time() > 0.6 && birdBrain.isPressed())
                    {
                        condorFeeder.setPosition(0.42);
                        motor3.setPower(0);

                    }
                    if (shooterTime.time() > 1 && birdBrain.isPressed())
                    {
                        shooterTime.reset();
                        i++;
                    }
                }
                else
                {
                    redSum = 0;
                    blueSum = 0;
                    moveNumber++;
                    buttonPusher.setPosition(1);
                }
                break;

            // move to next beacon
            case 5:
                motor4.setPower(0);
                moveMatrix(5, 1, 20, beacon2Path);
                break;

            //follow line and push button
            case 7:
                motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                if (colorReading.time() < 5)
                {
                    lastPosLeft = 0;
                    lastPosRight = 0;
                    pathSegment = 0;

                    leftSpeed = Math.round(((67.5676 * (Math.pow(eagleEyes.getLightDetected(), 0.5))) - 8.78378));
                    rightSpeed = Math.round(((-67.5676 * (Math.pow(eagleEyes.getLightDetected(), 0.5))) + 33.7838));
                    motor1.setPower(leftSpeed / 100);
                    motor2.setPower(rightSpeed / 100);
                    blueSum += color.blue();
                    redSum += color.red();

                }
                else if (colorReading.time() < 5.5)
                {
                    motor1.setPower(-0.2);
                    motor2.setPower(-0.2);

                    if (redSum > blueSum) {
                        buttonPusher.setPosition(1);
                    } else
                    {
                        buttonPusher.setPosition(0);
                    }
                }
                else if (colorReading.time() < 6)
                {
                    motor1.setPower(0.65);
                    motor2.setPower(0.65);
                }
                else
                {
                    motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    moveNumber++;
                }
                telemetry.addData("leftspeed", leftSpeed / 100);
                telemetry.addData("rightspeed", rightSpeed / 100);
                break;

        }

        telemetry.addData("Move Number", moveNumber);
        telemetry.addData("path segment", pathSegment);
        telemetry.addData("lastPos", lastPosLeft);
        telemetry.addData("lastPosRight", lastPosRight);
        telemetry.addData("targetPosLeft", targetPosLeft);
        telemetry.addData("targetPosRight", targetPosRight);
        telemetry.addData("eagleEyes", Math.pow(eagleEyes.getRawLightDetected(), -0.5));
        telemetry.addData("left", motor1.getCurrentPosition());
        telemetry.addData("red", color.red());
        telemetry.addData("blue", color.blue());
        telemetry.addData("rampdown", rampdowntown);
    }
    @Override
    public void stop(){

    }

    private void moveMatrix(int segment, double rampDown, int range, double[][] path)
    {

        if (pathSegment < segment)
        {                                                                                 //repeat 5 times
            targetPosLeft = (lastPosLeft + (int) Math.round((path[pathSegment][left]) * ticksPerInch));    //find target position in beacon matrix
            targetPosRight = (lastPosRight + (int) Math.round((path[pathSegment][right]) * ticksPerInch));
            targetSpeed = ((path[pathSegment][2]) / speedFactor);
            motor1.setTargetPosition(targetPosLeft);                                                           //set target position from beacon matrix
            motor2.setTargetPosition(targetPosRight);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);                                                   //run to position
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            if (Math.abs(Math.abs(motor1.getCurrentPosition()) - Math.abs(lastPosLeft)) >= (targetPosLeft * 0.8)) {
                motor1.setPower(targetSpeed * rampDown);
                motor2.setPower(targetSpeed * rampDown);
                rampdowntown = true;
            } else {
                motor1.setPower(targetSpeed);                                                                  //find motor speed in beacon matrix
                motor2.setPower(targetSpeed);//find motor speed in beacon matrix
                rampdowntown = false;
            }

            if ((motor1.getCurrentPosition() < (targetPosLeft + (range / 2) ) && motor1.getCurrentPosition() > (targetPosLeft - (range / 2))) &&
                    motor2.getCurrentPosition() < (targetPosRight + (range / 2)) && motor2.getCurrentPosition() > (targetPosRight - (range /2 )))
            {
                //if the motor is within 5 degrees of the target position
                motor1.setPower(0.0);                                                                          //set the motor power to zero
                motor2.setPower(0.0);
                lastPosLeft = motor1.getCurrentPosition();
                lastPosRight = motor2.getCurrentPosition();
                pathSegment++;
            }
        }
        if (pathSegment == segment)
        {                                                     //When loops is equal to 3
            moveNumber++;                                                           //increase the move number
            pathSegment = 0;                                //reset the path segment
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            colorReading.reset();
            shooterTime.reset();

        }
    }
}



