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
//@Disabled
public class DriverControl extends LinearOpMode {

    Robot robot = new Robot();

    double SCALING_FACTOR               = 0.30;
    boolean forwardFacing               = true;

    @Override
    public void runOpMode()
    {

        robot.initMotors( hardwareMap );
        robot.initGyroSensor( hardwareMap );
        robot.initRangeSensors( hardwareMap );

        while (!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData(">", "Press Play to start");
            telemetry.update();
        }

        robot.activateGyroTracking();

        this.forwardFacing = true;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() ) {

            operateLiftMotor();

            operatorDriveTrain();

            telemetry.addData("Gyro Degrees", robot.getCurrentPositionInDegrees());
            telemetry.addData("rangeSensorFront", robot.rangeSensorFront.rawUltrasonic());
            telemetry.addData("rangeSensorBack", robot.rangeSensorBack.rawUltrasonic());
            telemetry.update();
        }

        robot.stopAllMotors();
    }


    void operatorDriveTrain() {

        if (gamepad1.y)
            this.forwardFacing = true;
        if (gamepad1.a)
            this.forwardFacing = false;

        double multiplier = this.forwardFacing ? 1 : -1;

        double motorFrontRightPower = 0;
        double motorFrontLeftPower = 0;
        double motorCenterPower = 0;

        if (gamepad1.right_trigger != 0)
            SCALING_FACTOR = 0.60;
        else
            SCALING_FACTOR = 0.450;

        double OFFSET_POWER = 0.10;
        double robotPower = SCALING_FACTOR * Math.sqrt(Math.pow(gamepad1.right_stick_x, 2) + Math.pow(gamepad1.right_stick_y, 2));
        telemetry.addData("robot power", robotPower);

        double SIDEWAYS_MULTIPLIER = 1.5;

        double x1value = gamepad1.left_stick_x;

        if (Math.abs(x1value) > 0.1)
        {
            motorFrontRightPower = x1value * -1 * SCALING_FACTOR;
            motorFrontLeftPower = x1value * 1 * SCALING_FACTOR;
            motorCenterPower = x1value * -0.5 * SCALING_FACTOR;
        }
        else if ( (Math.abs(gamepad1.right_stick_x) > 0.01) || (Math.abs(gamepad1.right_stick_y) > 0.01))
        {
            //int joystickPostion = getDirectionAwareJoystickPosition();

            int joystickPostion = getJoystickPosition();

            telemetry.addData("joystick DA pos", getDirectionAwareJoystickPosition());
            telemetry.addData("joystick    pos", getJoystickPosition());

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

        telemetry.addData("Robot Facing", this.forwardFacing ? "FORWARD" : "BACKWARD");
        telemetry.addData("gamepad1.right_stick_y", gamepad1.right_stick_y);
        telemetry.addData("gamepad1.right_stick_x", gamepad1.right_stick_x);

        telemetry.addData("motorFrontRightPower", motorFrontRightPower);
        telemetry.addData("motorFrontLeftPower", motorFrontLeftPower);
        telemetry.addData("motorCenterPower", motorCenterPower);
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
       double motorRPPower = 0;

        if (gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right) {
            if (gamepad2.right_stick_y > 0.5)
                motorRPPower = 1.0;

            if (gamepad2.right_stick_y < -0.5)
                motorRPPower = -1.0;
        }

//        robot.motorLift.setPower(motorRPPower);

//        telemetry.addData("motorLift", motorRPPower);
    }

/*
    void operateIntake() {

        double powerSlide = 0;
        double powerHopper = 0;

        if ((gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right) == false)
        {
            if (gamepad2.right_stick_y > 0)
                powerSlide = 0.8;
            else if (gamepad2.right_stick_y < 0)
                powerSlide = -0.8;

            motorIntakeSlide.setPower(powerSlide);
        }

        if (gamepad2.left_trigger > 0)
            powerHopper = 1.0;

        if (gamepad2.right_trigger > 0 )
            powerHopper = -1.0;

        motorIntakeHopper.setPower(powerHopper);

        telemetry.addData("motorIntakeSlide", powerSlide);
        telemetry.addData("motorIntakeHopper", powerHopper);

    }*/

}
