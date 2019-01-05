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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

public class Robot
{
    /////////////////////
    // Declare motors vraiables
    /////////////////////
    public DcMotor motorFrontRight      = null;
    public DcMotor motorFrontLeft       = null;
    public DcMotor motorCenter          = null;
//    public DcMotor motorLift          = null;
    public DcMotor motorIntakeLeftArm   = null;
    public DcMotor motorIntakeRightArm  = null;
    public DcMotor motorIntakeExtension = null;

//    public DcMotor motorIntakeHopper   = null;
//    public DcMotor motorIntakeSlide    = null;


    /////////////////////
    // Declare vuforia tensorflow variables
    /////////////////////

    private static final String VUFORIA_KEY = "AaaD61H/////AAABmfJ7OgkkWEWVmniO8RAqZ1cEkXF6bR9ebw4Gw+hUI8s1s5iTA9Hyri+sjoSO/ISwSWxfZCI/iAzZ0RxSQyGQ7xjWaoE4AJgn4pKLKVcOsuglHJQhjQevzdFKWX6cXq4xYL6vzwX7G7zuUP6Iw3/TzZIAj7OxYl49mA30JfoXvq/kKnhDOdM531dbRyZiaNwTGibRl5Dxd4urQ5av3EU1QyFBWR04eKWBrJGffk8bdqjAbB3QDp/7ZAMi2WfkItMOP5ghc5arNRCNt5x+xX7gSq8pMt3ZoC3XPfRNNaEbf24MgaNJXlUARsfAuycgPiY83jbX0Hrctj4wZ20wqah+FNYMqvySokw6/fDmyG0mPmel";

    public static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    /////////////////////
    // Declare sensors
    /////////////////////
    ModernRoboticsI2cRangeSensor rangeSensorBottom  = null;
    ModernRoboticsI2cRangeSensor rangeSensorFront   = null;
    ModernRoboticsI2cRangeSensor rangeSensorBack    = null;

    // The IMU sensor object
    BNO055IMU imu;

    /////////////////////
    // Declare servos
    /////////////////////
    Servo servo1 = null;
    Servo servo2 = null;
    Servo servo3 = null;
    Servo servo4 = null;


    boolean     forwardFacing = true;

    boolean distanceAchieved = false;


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
        servo1.setPosition(1.00);
        servo2.setPosition(0.38);
    }

    public void setPhoneScanPosition() {
        // Rotate servo to positions photo in the front tiled
        setServoPosition(servo2, 0.55);
        sleep(500);
        setServoPosition(servo1, 0.40);
    }

    public void lowerRobot()
    {
     //   while (rangeSensorBottom.rawUltrasonic() > 6) {
     //       motorLift.setPower(0.4);
     //   }

       // motorLift.setPower(0.0);
       // sleep(150);
    }

    //----------------------------------------------------------------------------------------------
    // Initialization Methods
    //----------------------------------------------------------------------------------------------

    public void initMotors( HardwareMap hardwareMap, boolean brake ) {
        motorFrontRight     = hardwareMap.get(DcMotor.class, "motor1");
        motorFrontLeft      = hardwareMap.get(DcMotor.class, "motor2");
        motorCenter         = hardwareMap.get(DcMotor.class, "motor3");
//        motorLift           = hardwareMap.get(DcMotor.class, "motor4");

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
//        motorLift.setDirection(DcMotor.Direction.FORWARD);
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

    }

    public void initRangeSensors( HardwareMap hardwareMap )
    {
        rangeSensorBottom   = hardwareMap.get(ModernRoboticsI2cRangeSensor.class,"range_sensor1");
        rangeSensorFront    = hardwareMap.get(ModernRoboticsI2cRangeSensor.class,"range_sensor2");
        rangeSensorBack     = hardwareMap.get(ModernRoboticsI2cRangeSensor.class,"range_sensor3");
    }

    public void initServos( HardwareMap hardwareMap )
    {
        servo1  = hardwareMap.get(Servo.class, "servo1");
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
    public void driveForwardRotationAlignWall(double rotation, double targetPower, double distance, Telemetry telemetry)
    {
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorCenter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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

            double sensorFront = this.rangeSensorFront.rawUltrasonic();
            double sensorBack = this.rangeSensorBack.rawUltrasonic();

            double motorCenterPower = (sensorBack - sensorFront) * 0.05;

            if (sensorFront > 100)
                continue;
            /*if (sensorFront == distance || sensorBack == distance && !distanceAchieved)
                distanceAchieved = true;

            if (distanceAchieved){
                if (sensorFront >= distance + 3) {
                    motorCenter.setPower(0.4);
                } else if (sensorFront <= distance - 3) {
                    motorCenter.setPower(-0.4);
                }
                else
                {
                    motorFrontRight.setPower( power );
                    motorFrontLeft.setPower( power );


                    if (motorCenterPower > 0.20)
                        motorCenterPower = 0.20;

                    motorCenter.setPower( motorCenterPower );

                }
            }
            else {*/
                if (sensorFront >= distance + 3) {
                    motorFrontRight.setPower(-0.10);
                    motorFrontLeft.setPower(0.10);
                    motorCenter.setPower(0.4);
                } else if (sensorFront <= distance - 3) {
                    motorFrontRight.setPower(0.10);
                    motorFrontLeft.setPower(-0.10);
                    motorCenter.setPower(-0.4);
                }
                else
                {
                    motorFrontRight.setPower( power );
                    motorFrontLeft.setPower( power );


                    if (motorCenterPower > 0.20)
                        motorCenterPower = 0.20;

                    motorCenter.setPower( motorCenterPower );

                }
//            }


            telemetry.addData("sensorFront", sensorFront);
            telemetry.addData("sensorBack", sensorBack);
            telemetry.addData("motorCenterPower", motorCenterPower);
            telemetry.update();



/*
            telemetry.addData("joystick DA pos", getDirectionAwareJoystickPosition());

            if (sensorFront > 6){
                    motorCenter.setPower(-power);
            }
            else if (sensorFront == 6){
                if (sensorBack > 6){
                    motorCenter.setPower(power);
                }
                else if (sensorBack == 6){
                    motorCenter.setPower(0);
                }
                else if (sensorBack < 6){
                    motorCenter.setPower(-power);
                }
            }
            else if (sensorFront < 6){
                    motorCenter.setPower(power);
            }
*/
        }
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorCenter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorCenter.setPower(0);
    }



    ////////////////////////////////////////////////////////
    public void driveForwardTillTime( double seconds, double targetPower )
    {
        int initPosition = motorFrontRight.getCurrentPosition();

        boolean cont = true;
        double power = 0.10;

        motorFrontRight.setPower( power );
        motorFrontLeft.setPower( power );
        motorCenter.setPower( 0.0 );

        while (power < targetPower)
        {
            if (power < targetPower)
                power += 0.02;

            motorFrontRight.setPower( power );
            motorFrontLeft.setPower( power );
        }

        sleep( (long)(seconds * 1000));

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
//        this.motorLift.setPower(0);
    }

}
