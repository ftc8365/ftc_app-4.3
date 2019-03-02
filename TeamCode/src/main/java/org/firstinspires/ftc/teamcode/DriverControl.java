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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="DriverControl", group="TeleOp")
//@Disabled            robot.servoIntake.setPosition(1);

public class DriverControl extends LinearOpMode {

    Robot robot = new Robot();

    double SCALING_FACTOR       = 0.30;
    double lastJoystickPosition = 0;
    double robotPower           = 0.0;
    double robotSteerPower      = 0.0;
    boolean rampUp                  = true;
    double  initalIntakeArmPosition = 99999;

    @Override
    public void runOpMode()
    {
        robot.initMotors( hardwareMap, false );
        robot.initGyroSensor( hardwareMap );
        robot.initSensors( hardwareMap );
        robot.initServos( hardwareMap );

        robot.setPhoneStartingPostion();

        while (!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData(">", "Press Play to start");
            telemetry.update();
        }

        robot.activateGyroTracking();

        this.initalIntakeArmPosition = robot.motorIntakeLeftArm.getCurrentPosition();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() )
        {
            if (gamepad1.y)
                robot.setforwardFacing( false );
            if (gamepad1.b)
                robot.setforwardFacing( true );

            if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right)
                operateLiftMotor();
            else
                operateDriveTrain();

            operateIntake();

            telemetry.update();
        }

        robot.stopAllMotors();
    }

    public double motorIntakeExtensionPower = 0;
    public double rotatingPower = 0.0;
    public boolean useJoystick = false;

    void operateIntake()
    {
        if ( Math.abs(gamepad2.right_stick_y) > 0.01 )
        {
            rotatingPower = /*robot.getRotatingPower(gamepad2.right_stick_y)*/Math.pow(gamepad2.right_stick_y,3);
            useJoystick = true;
        }
        else if (useJoystick) {
            rotatingPower = 0;
            useJoystick = false;
        }

        double motorIntakeExtensionPower = 0;

        if ( gamepad2.left_stick_y > 0.01 )
        {
            motorIntakeExtensionPower = 0.5;
        }
        else if ( gamepad2.left_stick_y < -0.01 )
        {
            motorIntakeExtensionPower = -0.5;
        }

        if (gamepad2.dpad_up) {
            rotatingPower = robot.extendIntake( rotatingPower );
        }

        if (gamepad2.dpad_down) {
            robot.retractIntake( rotatingPower );
        }
        else if (gamepad2.b) {
            rotatingPower = robot.lowerIntakeStep1(rotatingPower);
            useJoystick  = false;
        }
        else if (gamepad2.a) {
            rotatingPower = robot.lowerIntakeStep2(rotatingPower);
            useJoystick  = false;
        }
        else if (gamepad2.x) {
            rotatingPower = robot.raiseIntakeStep1( rotatingPower );
            motorIntakeExtensionPower = 0;
            useJoystick  = false;
        } else if (gamepad2.y){
            rotatingPower = robot.raiseIntakeStep2( rotatingPower );
            useJoystick  = false;
        }

        robot.motorIntakeLeftArm.setPower(rotatingPower);
        robot.motorIntakeRightArm.setPower(rotatingPower);

        robot.motorIntakeExtension.setPower(motorIntakeExtensionPower);

//        telemetry.addData("Motor Intake Power", rotatingPower);
//        telemetry.addData("Motor Extension Position", robot.motorIntakeExtension.getCurrentPosition());

//        telemetry.addData("Motor Left Intake Position", robot.motorIntakeLeftArm.getCurrentPosition());
//        telemetry.addData("Motor Right Intake Position", robot.motorIntakeRightArm.getCurrentPosition());


        if (gamepad2.left_trigger > 0)
//            robot.servoIntake.setPosition(0);
        robot.motorIntakeSpinner.setPower(0.60);
        else if (gamepad2.right_trigger > 0 )
//            robot.servoIntake.setPosition(1);
            robot.motorIntakeSpinner.setPower(-0.60);

        else
//            robot.servoIntake.setPosition(0.5);
            robot.motorIntakeSpinner.setPower(0);

    }

    void operateDriveTrain()
    {
        double multiplier = robot.getforwardFacing() ? 1 : -1;

        double motorFrontRightPower = 0;
        double motorFrontLeftPower  = 0;
        double motorCenterPower     = 0;

        if (gamepad1.right_trigger != 0)
            SCALING_FACTOR = 0.5;
        else
            SCALING_FACTOR = 0.5;

        double OFFSET_POWER = 0.10;

        double SIDEWAYS_MULTIPLIER = 1.5;

        double x1value = gamepad1.left_stick_x;

        if (Math.abs(x1value) > 0.01)
        {
            if (this.rampUp)
            {
                // Turning right
                if (x1value > 0) {
                    if (robotSteerPower < 0)
                        robotSteerPower = 0;

                    this.robotSteerPower += 0.10;

                    if (robotSteerPower > x1value)
                        robotSteerPower = x1value;
                } else {
                    // Turning left
                    if (robotSteerPower > 0)
                        robotSteerPower = 0;

                    this.robotSteerPower -= 0.10;

                    if (robotSteerPower < x1value)
                        robotSteerPower = x1value;
                }
            }

            motorFrontRightPower    = robotSteerPower * -1.0 * SCALING_FACTOR;
            motorFrontLeftPower     = robotSteerPower *  1.0 * SCALING_FACTOR;
            motorCenterPower        = robotSteerPower * -0.5 * SCALING_FACTOR;
        }
        else if ( (Math.abs(gamepad1.right_stick_x) > 0.01) || (Math.abs(gamepad1.right_stick_y) > 0.01))
        {
            this.robotSteerPower = 0.0;

            double targetRobotPower = SCALING_FACTOR * Math.sqrt(Math.pow(gamepad1.right_stick_x, 2) + Math.pow(gamepad1.right_stick_y, 2));

            //int joystickPostion = getDirectionAwareJoystickPosition();

            int joystickPostion = getJoystickPosition();

            telemetry.addData("joystick DA pos", getDirectionAwareJoystickPosition());
            telemetry.addData("joystick    pos", getJoystickPosition());

            if (this.rampUp)
                robotPower = getRevUpPower( joystickPostion, targetRobotPower );
            else
                robotPower = targetRobotPower;

            switch (joystickPostion)
            {
                case 1:
                    motorCenterPower = 0;
                    motorFrontRightPower = multiplier * robotPower;
                    motorFrontLeftPower = multiplier * robotPower;
                    break;
                case 2:
                    motorCenterPower = multiplier * robotPower;
                    motorFrontLeftPower =  multiplier * robotPower;
                    motorFrontRightPower = OFFSET_POWER * multiplier;
                    break;
                case 3:
                    motorCenterPower = multiplier * robotPower * SIDEWAYS_MULTIPLIER;
                    motorFrontRightPower = -OFFSET_POWER * multiplier * SIDEWAYS_MULTIPLIER;
                    motorFrontLeftPower = OFFSET_POWER * multiplier * SIDEWAYS_MULTIPLIER;
                    break;
                case 4:
                    motorCenterPower = multiplier * robotPower;
                    motorFrontRightPower = -1 * multiplier * robotPower;
                    motorFrontLeftPower = -OFFSET_POWER * multiplier;
                    break;
                case 5:
                    motorCenterPower = 0;
                    motorFrontRightPower = -1 * multiplier * robotPower;
                    motorFrontLeftPower = -1 * multiplier * robotPower;
                    break;
                case 6:
                    motorCenterPower = -1 * multiplier * robotPower;
                    motorFrontLeftPower = -1 * multiplier * robotPower;
                    motorFrontRightPower = -OFFSET_POWER * multiplier;
                    break;
                case 7:
                    motorCenterPower = -1 * multiplier * robotPower * SIDEWAYS_MULTIPLIER;
                    motorFrontRightPower = OFFSET_POWER * multiplier * SIDEWAYS_MULTIPLIER;
                    motorFrontLeftPower = -OFFSET_POWER * multiplier * SIDEWAYS_MULTIPLIER;
                    break;
                case 8:
                    motorCenterPower = -1*multiplier * robotPower;
                    motorFrontRightPower = multiplier * robotPower;
                    motorFrontLeftPower = OFFSET_POWER * multiplier;
                    break;
                case 0:
                    motorCenterPower = 0;
                    motorFrontLeftPower = 0;
                    motorFrontRightPower = 0;
            }
        }

        robot.motorFrontRight.setPower(motorFrontRightPower);
        robot.motorFrontLeft.setPower(motorFrontLeftPower);
        robot.motorCenter.setPower(motorCenterPower);

        telemetry.addData("robot power", robotPower);
        telemetry.addData("Robot Facing", robot.getforwardFacing() ? "FORWARD" : "BACKWARD");
        telemetry.addData("gamepad1.right_stick_y", gamepad1.right_stick_y);
        telemetry.addData("gamepad1.right_stick_x", gamepad1.right_stick_x);
        telemetry.addData("motorFrontRightPower", motorFrontRightPower);
        telemetry.addData("motorFrontLeftPower", motorFrontLeftPower);
        telemetry.addData("motorCenterPower", motorCenterPower);
    }


    double getRevUpPower(int joystickPosition, double targetRobotPower )
    {
        double power = robotPower;

        if (targetRobotPower == 0 )
        {
            power = 0;
        }
        else if ((Math.abs(joystickPosition - lastJoystickPosition) > 1) && (Math.abs(joystickPosition - lastJoystickPosition) < 7))
        {
            power = 0;
        }
        else if (targetRobotPower > 0)
        {
            // Increment robot power by 5% power until targetPower is reached
            power += 0.10;
            if (power > targetRobotPower)
                power = targetRobotPower;
        }
        else
        {
            // Decrement robot power by 5% power until targetPower is reached
            power -= 0.10;
            if (power < targetRobotPower)
                power = targetRobotPower;
        }

        this.lastJoystickPosition = joystickPosition;
        return power;
    }

    int getDirectionAwareJoystickPosition()
    {
        int position = getJoystickPosition();

        double degrees = robot.getCurrentPositionInDegrees();

        int offset = 0;

        if (degrees >= 0 && degrees <= 22.5)
            offset = 0;
        else if (degrees > 22.5 && degrees <= 67.5)
            offset = -1;
        else if (degrees > 67.5 && degrees <= 112.5)
            offset = -2;
        else if (degrees > 112.5 && degrees <= 157.5)
            offset = -3;
        else if (degrees > 157.5 && degrees <= 202.5)
            offset = -4;
        else if (degrees > 202.5 && degrees <= 247.5)
            offset = -5;
        else if (degrees > 247.5 && degrees <= 292.5)
            offset = -6;
        else if (degrees > 337.5 && degrees <= 337.5)
            offset = -7;
        else
            offset = 0;

        if (position == 0) return  0;

        position += offset;
        if (position <= 0)
            position += 8;

        telemetry.addData("offset", offset);
        telemetry.addData("position", position);
        telemetry.addData("degrees", degrees);

        return position;
    }

    int getJoystickPosition()
    {
        double x = gamepad1.right_stick_x;
        double y = gamepad1.right_stick_y;

        if (x >= -0.1 && x < 0.1) {
            if (y < -0.05)
                return 1;

            if (y > 0.05)
                return 5;
        }

        if (x >= 0.10 && x < 0.90)
        {
            if (y >= -1.00 && y <= -0.10)
                return 2;

            if (y <= 1.00 && y >= 0.10)
                return 4;
        }

        if (x <= -0.10 && x >= -0.90) {
            if (y >= -1.00 && y <= -0.10)
                return 8;

            if (y <= 1.00 && y >= 0.10)
                return 6;
        }

        if (x >= 0.05) {
            return 3;
        }

        if (x <= -0.05) {
            return 7;
        }
        return 0;
    }

    void operateLiftMotor()
    {
       double motorPower = 0;

        if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
            if (gamepad1.right_stick_y > 0.1)
                motorPower = 1.0;

            if (gamepad1.right_stick_y < -0.1)
                motorPower = -1.0;
        }
        robot.motorLift.setPower(motorPower);
    }

}
