/* Copyright (c) 2018 FIRST. All rights reserved.
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


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

public class Robot
{
    /////////////////////
    // Declare motors variables
    /////////////////////
    public DcMotor motorFrontRight      = null;
    public DcMotor motorFrontLeft       = null;
    public DcMotor motorCenter          = null;
    public DcMotor motorLift            = null;
    public DcMotor motorIntakeLeftArm   = null;
    public DcMotor motorIntakeRightArm  = null;
    public DcMotor motorIntakeExtension = null;

//    public DcMotor motorIntakeHopper   = null;
//    public DcMotor motorIntakeSlide    = null;
    public int[] extensionPosition = new int[4];
    public int extensionCounter = 0;

    /////////////////////
    // Declare vuforia tensorflow variables
    /////////////////////

    private static final String VUFORIA_KEY = "AaaD61H/////AAABmfJ7OgkkWEWVmniO8RAqZ1cEkXF6bR9ebw4Gw+hUI8s1s5iTA9Hyri+sjoSO/ISwSWxfZCI/iAzZ0RxSQyGQ7xjWaoE4AJgn4pKLKVcOsuglHJQhjQevzdFKWX6cXq4xYL6vzwX7G7zuUP6Iw3/TzZIAj7OxYl49mA30JfoXvq/kKnhDOdM531dbRyZiaNwTGibRl5Dxd4urQ5av3EU1QyFBWR04eKWBrJGffk8bdqjAbB3QDp/7ZAMi2WfkItMOP5ghc5arNRCNt5x+xX7gSq8pMt3ZoC3XPfRNNaEbf24MgaNJXlUARsfAuycgPiY83jbX0Hrctj4wZ20wqah+FNYMqvySokw6/fDmyG0mPmel";

    public static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    public enum MineralLocation
    {
        UNKNOWN,
        RIGHT,
        CENTER,
        LEFT
    };

    public enum IntakeState
    {
        INTAKE_DOWN,
        STAGE1_UP,
        STAGE2_UP,
        STAGE1_DOWN
    };

    /////////////////////
    // Declare sensors
    /////////////////////
    ModernRoboticsI2cRangeSensor rangeSensorBottom  = null;
    ModernRoboticsI2cRangeSensor rangeSensorFront   = null;
    ModernRoboticsI2cRangeSensor rangeSensorBack    = null;

    // The IMU sensor object
    BNO055IMU imu;
    public DigitalChannel digitalTouch;  // Hardware Device Object

    /////////////////////
    // Declare servos
    /////////////////////
    Servo servoPhone = null;
    Servo servoIntake = null;
    Servo servo3 = null;
    Servo servo4 = null;


    boolean forwardFacing = true;
    boolean intakeRaised  = false;
    boolean distanceAchieved = false;
    IntakeState intakeState = IntakeState.INTAKE_DOWN;


    public final boolean setforwardFacing(boolean forwardFacing)
    {
        this.forwardFacing = forwardFacing;
        return this.forwardFacing;
    }

    public final boolean getforwardFacing()
    {
        return this.forwardFacing;
    }

    private final void sleep(long milliseconds)
    {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void setPhoneStartingPostion()
    {
        servoPhone.setPosition(1);
    }

    public void setPhoneScanPosition()
    {
        servoPhone.setPosition(0.52);
    }

    public void lowerRobot()
    {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        while (runtime.seconds() < 5.0 && rangeSensorBottom.rawUltrasonic() > 5)
        {
            motorLift.setPower(1.0);
        }

        sleep(50);

        motorLift.setPower(0.0);
    }

    //----------------------------------------------------------------------------------------------
    // Initialization Methods
    //----------------------------------------------------------------------------------------------

    public void initMotors( HardwareMap hardwareMap, boolean brake )
    {
        motorFrontRight     = hardwareMap.get(DcMotor.class, "motor1");
        motorFrontLeft      = hardwareMap.get(DcMotor.class, "motor2");
        motorCenter         = hardwareMap.get(DcMotor.class, "motor3");
        motorLift           = hardwareMap.get(DcMotor.class, "motor4");

        motorIntakeLeftArm = hardwareMap.get(DcMotor.class, "motor5");
        motorIntakeRightArm = hardwareMap.get(DcMotor.class, "motor6");
        motorIntakeExtension = hardwareMap.get(DcMotor.class, "motor7");


//        motorIntakeHopper   = hardwareMap.get(DcMotor.class, "motor6");
//        motorIntakeSlide    = hardwareMap.get(DcMotor.class, "motor7");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorCenter.setDirection(DcMotor.Direction.REVERSE);
        motorLift.setDirection(DcMotor.Direction.FORWARD);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorIntakeLeftArm.setDirection(DcMotor.Direction.REVERSE);
        motorIntakeRightArm.setDirection(DcMotor.Direction.FORWARD);
        motorIntakeExtension.setDirection(DcMotor.Direction.FORWARD);

        if (brake)
        {
            motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorCenter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else
        {
            motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motorCenter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        motorIntakeLeftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorIntakeRightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int intakePos = this.motorIntakeExtension.getCurrentPosition();

        extensionPosition[0] = intakePos - 50;
        extensionPosition[1] = intakePos - 2500;
        extensionPosition[2] = intakePos - 3500;
        extensionPosition[3] = intakePos - 4500;


    }

    public void initSensors( HardwareMap hardwareMap )
    {
        rangeSensorBottom   = hardwareMap.get(ModernRoboticsI2cRangeSensor.class,"range_sensor1");
        rangeSensorFront    = hardwareMap.get(ModernRoboticsI2cRangeSensor.class,"range_sensor2");
        rangeSensorBack     = hardwareMap.get(ModernRoboticsI2cRangeSensor.class,"range_sensor3");
    }

    public void initServos( HardwareMap hardwareMap )
    {
        servoPhone  = hardwareMap.get(Servo.class, "servo1");
        servoIntake = hardwareMap.get(Servo.class, "servo2");
        //servo2  = hardwareMap.get(Servo.class, "servo2");
        //servo3  = hardwareMap.get(Servo.class, "servo3");
        //servo4  = hardwareMap.get(Servo.class, "servo4");
    }


    public void initGyroSensor( HardwareMap hardwareMap ) {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }


    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    public void initTensorFlowObjectDetection( HardwareMap hardwareMap )
    {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public void activateTensorFlowObjectDetection()
    {
        if ( tfod != null )
        {
            tfod.activate();
        }
    }

    public void activateGyroTracking()
    {
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    ////////////////////////////////////////////////////////
    public void driveForwardRotation( double rotation, double targetPower )
    {
        int initPosition = motorFrontRight.getCurrentPosition();

        boolean cont = true;
        double power = 0.05;

        motorFrontRight.setPower( power );
        motorFrontLeft.setPower( power );
        motorCenter.setPower( 0.0 );

        while (cont)
        {
            if (motorFrontRight.getCurrentPosition() - initPosition >= 1000 * rotation)
                cont = false;

            if (power < targetPower)
                power += 0.02;

            motorFrontRight.setPower( power );
            motorFrontLeft.setPower( power );
        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorCenter.setPower(0);
    }

    public void driveForwardRotationTurn( double rotation, double targetPower, double centerPower )
    {
        int initPosition = motorFrontRight.getCurrentPosition();

        boolean cont = true;
        double power = 0.05;

        motorFrontRight.setPower( power );
        motorFrontLeft.setPower( power );
        motorCenter.setPower( centerPower );

        while (cont)
        {
            if (motorFrontRight.getCurrentPosition() - initPosition >= 1000 * rotation)
                cont = false;

            if (power < targetPower)
                power += 0.02;

            motorFrontRight.setPower( power );
            motorFrontLeft.setPower( power );
        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorCenter.setPower(0);
    }

    ////////////////////////////////////////////////////////
    public void driveBackwardRotationAlignWall(double rotation, double targetPower, double distance, Telemetry telemetry) {
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorCenter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        int initPosition = motorFrontRight.getCurrentPosition();

        boolean cont = true;
        double power = -0.05;

        motorFrontRight.setPower(power);
        motorFrontLeft.setPower(power);
        motorCenter.setPower(0.0);

        while (cont) {
            if (motorFrontRight.getCurrentPosition() - initPosition <= -1000 * rotation)
                cont = false;

            if (power > -targetPower)
                power -= 0.02;

            double sensorFront = this.rangeSensorFront.rawUltrasonic();
            double sensorBack = this.rangeSensorBack.rawUltrasonic();

            double motorCenterPower = (sensorBack - sensorFront) * 0.05;

            if (sensorFront > 100)
                continue;

            double sensorDistance = Math.min(sensorFront, sensorBack);


            if (sensorDistance >= distance + 3)  {
                motorFrontRight.setPower(-0.10);
                motorFrontLeft.setPower(0.10);
                motorCenter.setPower(0.4);
            } else if (sensorDistance <= distance - 3) {
                motorFrontRight.setPower(0.10);
                motorFrontLeft.setPower(-0.10);
                motorCenter.setPower(-0.4);
            } else {
                motorFrontRight.setPower(power);
                motorFrontLeft.setPower(power);


                if (motorCenterPower > 0.20)
                    motorCenterPower = 0.20;

                motorCenter.setPower(motorCenterPower);

            }
        }
    }

    public void lockMarker(){
        this.servoIntake.setPosition(0.5);  //stops intake
    }

    public void dropMarker(){
        this.servoIntake.setPosition(0);    // out take
        sleep(1000);
        this.servoIntake.setPosition(0.5);  //stops intake
    }

    public void driveForwardRotationAlignWall(double rotation, double targetPower, double distance, Telemetry telemetry)
    {
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorCenter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        int initPosition = motorFrontRight.getCurrentPosition();

        boolean cont = true;
        double power = 0.05;

        double motorCenterPower = 0;
        double motorLeftPower = 0;
        double motorRightPower = 0;

        motorFrontRight.setPower( 0 );
        motorFrontLeft.setPower( 0 );
        motorCenter.setPower( 0.0 );

        double lastFrontSensorValue = 255;
        double lastBackSensorValue = 255;


        while (cont)
        {
            motorCenterPower = 0;
            motorLeftPower = 0;
            motorRightPower = 0;

            if (motorFrontRight.getCurrentPosition() - initPosition >= 1000 * rotation) {
                cont = false;
                continue;
            }

            if (power < targetPower)
                power += 0.02;

            double sensorFront = this.rangeSensorFront.rawUltrasonic();
            double sensorBack = this.rangeSensorBack.rawUltrasonic();

            if (sensorFront > 250)
                sensorFront = lastFrontSensorValue;
            if (sensorBack > 250)
                sensorBack = lastBackSensorValue;

            if (sensorFront == 255 && sensorBack == 255)
                continue;

            if (Math.abs(sensorBack - sensorFront) > 2)
            {
                motorCenterPower = (sensorBack > sensorFront) ? 0.25 : - 0.25;

                motorRightPower = motorCenterPower;
                motorLeftPower = -1 * motorCenterPower;
            }
            else if (Math.max(sensorFront, sensorBack) > distance + 3)
            {
                motorRightPower  = -0.10;
                motorLeftPower   =  0.10;
                motorCenterPower =  0.45;
            }
            else if (Math.min(sensorFront, sensorBack) < distance - 3)
            {
                motorRightPower  =  0.10;
                motorLeftPower   = -0.10;
                motorCenterPower = -0.40;
            }
            else
            {
                motorCenterPower = (sensorBack - sensorFront) * 0.05;
                if (motorCenterPower > 0.20)
                    motorCenterPower = 0.20;

                motorRightPower  = power;
                motorLeftPower   = power;
            }

            motorFrontRight.setPower( motorRightPower );
            motorFrontLeft.setPower( motorLeftPower );
            motorCenter.setPower( motorCenterPower );

            lastFrontSensorValue = sensorFront;
            lastBackSensorValue =  sensorBack;

            telemetry.addData("sensorFront", sensorFront);
            telemetry.addData("sensorBack", sensorBack);
            telemetry.update();
        }

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorCenter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorCenter.setPower(0);
    }

    ////////////////////////////////////////////////////////
    public void driveRightTillRotation( double rotation, double targetPower )
    {
        int initPosition = motorCenter.getCurrentPosition();

        boolean cont = true;
        double power = 0.05;

        motorFrontRight.setPower( 0 );
        motorFrontLeft.setPower( 0 );

        while (cont)
        {
            if (motorCenter.getCurrentPosition() - initPosition >= 1000 * rotation)
                cont = false;

            if (power < targetPower)
                power += 0.02;

            motorFrontRight.setPower( -0.20 );
            motorFrontLeft.setPower( 0.20 );
            motorCenter.setPower( power );
        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorCenter.setPower(0);
    }


    ////////////////////////////////////////////////////////
    public void driveLeftTillRotation( double rotation, double targetPower )
    {
        int initPosition = motorCenter.getCurrentPosition();

        boolean cont = true;
        double power = 0.05;

        motorFrontRight.setPower( 0 );
        motorFrontLeft.setPower( 0 );

        while (cont)
        {
            if (motorCenter.getCurrentPosition() - initPosition <= -1000 * rotation)
                cont = false;

            if (power < targetPower)
                power += 0.02;

            motorFrontRight.setPower( 0.10 );
            motorFrontLeft.setPower( -0.10 );
            motorCenter.setPower( power * -1 );
        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorCenter.setPower(0);
    }


    ////////////////////////////////////////////////////////
    public void driveForwardTillTime( long seconds, double targetPower )
    {
        motorCenter.setPower( 0.0 );

        motorFrontRight.setPower( targetPower * 1 );
        motorFrontLeft.setPower( targetPower * 1 );

        sleep(seconds * 1000);

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorCenter.setPower(0);
    }


    ////////////////////////////////////////////////////////
    public void driveBackwardTillTime( long seconds, double targetPower )
    {
        motorCenter.setPower( 0.0 );

        motorFrontRight.setPower( targetPower * -1 );
        motorFrontLeft.setPower( targetPower * -1 );

        sleep(seconds * 1000);

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorCenter.setPower(0);
    }



    ////////////////////////////////////////////////////////
    public void driveBackwardRotation( double rotation, double targetPower )
    {
        int initPosition = motorFrontRight.getCurrentPosition();

        boolean cont = true;
        double power = 0.05;

        motorCenter.setPower( 0.0 );

        while (cont)
        {
            if (motorFrontRight.getCurrentPosition() - initPosition <= -1000 * rotation)
                cont = false;

            if (power < targetPower)
                power += 0.02;

            motorFrontRight.setPower( power * -1 );
            motorFrontLeft.setPower( power * -1 );
        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorCenter.setPower(0);
    }



    ////////////////////////////////////////////////////////
    public void turnRightRotation( double rotation, double targetPower )
    {
        int initPosition = motorCenter.getCurrentPosition();

        boolean cont = true;
        double power = 0.10;

        while (cont)
        {
            if (motorCenter.getCurrentPosition() - initPosition <= -1000 * rotation)
                cont = false;

            if (power < targetPower)
                power += 0.01;

            motorFrontRight.setPower(power * -1);
            motorFrontLeft.setPower(power);
            motorCenter.setPower(power * -1);
        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorCenter.setPower(0);
    }


    ////////////////////////////////////////////////////////
    public void turnLeftRotation( double rotation, double power )
    {
        int initPosition = motorCenter.getCurrentPosition();

        boolean cont = true;

        motorFrontRight.setPower(power);
        motorFrontLeft.setPower(power * -1);
        motorCenter.setPower(power * 1);

        while (cont)
        {
            if (motorCenter.getCurrentPosition() - initPosition >= 1000 * rotation)
                cont = false;

        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorCenter.setPower(0);
    }


    public double getCurrentPositionInDegrees() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double fromDegrees = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);

        // degrees
        // 0, -45, -90, -180
        // convert that to 0, 45, 90, 180

        double toDegrees;

        if ( fromDegrees <= 0 )
            toDegrees = fromDegrees * -1;
        else
            toDegrees = 360 - fromDegrees;

        return toDegrees;

    }


    ///////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////

    public void turnRightTillDegrees( int targetDegrees, double targetPower, Telemetry telemetry )
    {
        boolean continueToTurn = true;
        double power = 0.05;
        double currentHeading = 0;
        double distanceToGo = 0;

        while ( continueToTurn )
        {
            currentHeading = getCurrentPositionInDegrees();
            if (currentHeading > 180 && targetDegrees < 180)
                currentHeading -= 360;

            distanceToGo = targetDegrees - currentHeading;

            if ( distanceToGo > 0.0 )
            {
                // Ramp up power by 3%
                if (power < targetPower)
                    power += 0.03;

                // Ramp down power
                if ( distanceToGo < 15 )
                    power = 0.20;

                motorFrontRight.setPower( power * -1 );
                motorFrontLeft.setPower( power );
                motorCenter.setPower( power * -0.50 ) ;
            }
            else
            {
                continueToTurn = false;
            }

            telemetry.addData("currentHeading", currentHeading);
            telemetry.addData( "distanceToGo", distanceToGo);
            telemetry.update();
        }

        // Stop motors
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorCenter.setPower(0);

        telemetry.addData("currentHeading", currentHeading);
        telemetry.addData( "distanceToGo", distanceToGo);
        telemetry.addData( "done?", "yes");
        telemetry.update();

    }
    ///////////////////////////////////////////////////////////////////////////////////////////

    public void turnLeftTillDegrees( int targetDegrees, double targetPower, Telemetry telemetry )
    {
        boolean continueToTurn = true;
        double power = 0.05;

        while ( continueToTurn )
        {
            double currentHeading = getCurrentPositionInDegrees();
            if (currentHeading < 30)
                currentHeading += 360;

            double distanceToGo = currentHeading - targetDegrees;

            if ( distanceToGo > 0.0 )
            {
                // Ramp up power by 3%
                if (power < targetPower)
                    power += 0.03;

                // Ramp down power
               // if ( distanceToGo < 15 )
                 //   power = 0.20;

                motorFrontRight.setPower(power );
                motorFrontLeft.setPower(power * -1 );
                motorCenter.setPower(power * 0.5) ;
            }
            else
            {
                continueToTurn = false;
            }
        }

        // Stop motors
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorCenter.setPower(0);

    }


    float getCurrentHeadingRightTurn()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) *-1;
    }

    float getCurrentHeadingLeftTurn()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }

    void setServoPosition( Servo servo, double targetPosition )
    {
        double currentPos = servo.getPosition();

        if (currentPos > targetPosition) {
            while (servo.getPosition() > targetPosition) {
                servo.setPosition( servo.getPosition() - 0.1);

            }
        }
        else if (currentPos < targetPosition) {
            while (servo.getPosition() < targetPosition) {

                servo.setPosition( servo.getPosition() + 0.1);

            }
        }
    }

    public void stopAllMotors()
    {
        this.motorCenter.setPower(0);
        this.motorFrontLeft.setPower(0);
        this.motorFrontRight.setPower(0);
    }

    private double extendIntakeToPos(int pos)
    {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        double rampUpPower = -0.10;

        while ((runtime.seconds() < 1.0) &&
                (this.motorIntakeExtension.getCurrentPosition() >= extensionPosition[pos]) )
        {
            this.motorIntakeExtension.setPower( rampUpPower );

            rampUpPower -= 0.02;
            if (rampUpPower <= -0.50)
                rampUpPower = -0.50;
        }

        this.motorIntakeExtension.setPower(0);

        return 0.0;
    }

    private int getNextExtensionPos()
    {
        int pos = this.motorIntakeExtension.getCurrentPosition();
        int i = 1;

        while ( i<= 2 && pos < this.extensionPosition[i] )
            i++;

        return i;
    }


    private int getPreviousExtensionPos()
    {
        int pos = this.motorIntakeExtension.getCurrentPosition();
        int i = 2;

        while ( i >  0 && this.extensionPosition[i] < pos )
            i--;

        return i;
    }


    public double extendIntake()
    {
        double intakePower = 0;

        if (extensionCounter <= 2)      // Allow at most 3 extensions
        {
            extensionCounter = getNextExtensionPos();

/*            if (extensionCounter == 2 && !intakeRaised) {
                this.motorIntakeLeftArm.setPower(-0.35);
                this.motorIntakeRightArm.setPower(-0.35);
                sleep(50);
                intakePower = -0.20;
                intakeRaised = true;
            }
            else {
                intakeRaised = false;
                this.motorIntakeLeftArm.setPower(0.0);
                this.motorIntakeRightArm.setPower(0.0);
                intakePower = 0.0;
            }
*/
            extendIntakeToPos( extensionCounter );
        }

        return intakePower;
    }

    private double retractIntakeToPos(int pos)
    {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        double rampUpPower = -0.10;

        while (runtime.seconds() < 2.5)
        {
            int posToGo = extensionPosition[pos] - this.motorIntakeExtension.getCurrentPosition();

            if (posToGo <= 0)
                break;

            if (posToGo > 200)
                this.motorIntakeExtension.setPower(0.75);
            else
                this.motorIntakeExtension.setPower(0.35);
        }

        this.motorIntakeExtension.setPower(0);

        return 0.0;
    }


    public double retractIntake()
    {
        if (extensionCounter > 0) {

            extensionCounter = this.getPreviousExtensionPos();

            return retractIntakeToPos( extensionCounter );
        }

        return 0;
    }

    public double lowerIntakeStep1( double currentPower )
    {
        if (intakeState != IntakeState.STAGE2_UP)
            return currentPower;

        ElapsedTime runtime = new ElapsedTime();

        double startingPos = this.motorIntakeLeftArm.getCurrentPosition();

        double power = 0.10;

        runtime.reset();

        while ((runtime.seconds() < 2.0) &&
                (this.motorIntakeLeftArm.getCurrentPosition() <= startingPos + 400))
        {
            this.motorIntakeLeftArm.setPower(power);
            this.motorIntakeRightArm.setPower(power);

            power += 0.02;
            if (power > 0.20)
                power = 0.20;
        }

        while ((runtime.seconds() < 2.0) &&
                (this.motorIntakeLeftArm.getCurrentPosition() <= startingPos + 900))
        {
            this.motorIntakeLeftArm.setPower(0);
            this.motorIntakeRightArm.setPower(0);
        }

        this.motorIntakeLeftArm.setPower(-0.2);
        this.motorIntakeRightArm.setPower(-0.2);

        retractIntake();

        intakeState = IntakeState.STAGE1_DOWN;

        return -0.2;
    }

    public double lowerIntakeStep2(double currentPower)
    {
        if (intakeState != IntakeState.STAGE1_DOWN)
            return currentPower;

        intakeState = IntakeState.INTAKE_DOWN;

        return 0.0;
    }

    public double raiseIntakeStep1()
    {
        if ( intakeState != IntakeState.INTAKE_DOWN )
            return 0.0;

        ElapsedTime runtime = new ElapsedTime();

        double rampUpPower = -0.20;
        double startingPos = this.motorIntakeLeftArm.getCurrentPosition();

        if (this.motorIntakeExtension.getCurrentPosition() < extensionPosition[1])
            retractIntakeToPos(1);
        else
            extendIntakeToPos(1);

        runtime.reset();

        while ((runtime.seconds() < 1.0) &&
                (this.motorIntakeLeftArm.getCurrentPosition() >= startingPos - 200))
        {
            this.motorIntakeLeftArm.setPower( rampUpPower);
            this.motorIntakeRightArm.setPower( rampUpPower );
            rampUpPower -= 0.02;

            if (rampUpPower < -0.75)
                rampUpPower = -0.75;
        }

        this.motorIntakeLeftArm.setPower(-.20);
        this.motorIntakeRightArm.setPower(-.20);

        intakeState = IntakeState.STAGE1_UP;

        return -0.20;
    }

    public void raiseIntakeStep2()
    {
        if ( intakeState != IntakeState.STAGE1_UP )
            return;

        extendIntakeToPos(2);

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        double startingPos = this.motorIntakeLeftArm.getCurrentPosition();

        this.servoIntake.setPosition(1);

        while ((runtime.seconds() < 3.0) &&
              (this.motorIntakeLeftArm.getCurrentPosition() >= startingPos - 750))
        {
            this.motorIntakeLeftArm.setPower(-0.85);
            this.motorIntakeRightArm.setPower(-0.85);
        }

        this.motorIntakeLeftArm.setPower(0);
        this.motorIntakeRightArm.setPower(0);


        sleep(250);
        this.servoIntake.setPosition(0.5);

        intakeState = IntakeState.STAGE2_UP;

    }


    public MineralLocation detectMineral(Telemetry telemetry)
    {
        MineralLocation location = MineralLocation.UNKNOWN;

        // getUpdatedRecognitions() will return null if no new information is available since
        // the last time that call was made.
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

        if (updatedRecognitions == null)
            return MineralLocation.UNKNOWN;

        int goldMineralYPos = 99999;
        int silverMineral1YPos = 99999;
        int silverMineral2YPos = 99999;

        telemetry.addData("# Object Detected", updatedRecognitions.size());

        // Determining Y coordinate of each mineral detected
        // Y position increases as it goes from right to left

        for (Recognition recognition : updatedRecognitions)
        {
            telemetry.addData(recognition.getLabel(), "Y = %.2f", recognition.getTop());

            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                goldMineralYPos = (int) recognition.getTop();
            } else if (silverMineral1YPos == 99999) {
                silverMineral1YPos = (int) recognition.getTop();
            }
            else {
                silverMineral2YPos = (int) recognition.getTop();
            }
        }

        if ( goldMineralYPos < 99999 && silverMineral1YPos < 99999 &&
                goldMineralYPos < silverMineral1YPos &&
                goldMineralYPos < silverMineral2YPos )
        {
            location = MineralLocation.RIGHT;
        }
        else if ( goldMineralYPos < 99999 && silverMineral1YPos < 99999 &&
                  goldMineralYPos > silverMineral1YPos &&
                  goldMineralYPos < silverMineral2YPos )
        {
            location = MineralLocation.CENTER;
        }
        else if ( silverMineral1YPos < 99999 && silverMineral2YPos < 99999 &&
                  goldMineralYPos > silverMineral1YPos &&
                  goldMineralYPos > silverMineral2YPos )
        {
            location = MineralLocation.LEFT;
        }

        telemetry.addData("Gold Mineral Position", location);
        return location;

    }


}
