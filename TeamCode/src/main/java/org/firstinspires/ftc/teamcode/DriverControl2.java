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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


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

@TeleOp(name="DriverControl2", group="TeleOp")
//@Disabled
public class DriverControl2 extends LinearOpMode {

    Robot robot = new Robot();

    double SCALING_FACTOR               = 0.80;
    boolean forwardFacing               = true;

    @Override
    public void runOpMode()
    {

        robot.initMotors( hardwareMap );
        robot.initGyroSensor( hardwareMap );

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
            SCALING_FACTOR = 1.0;
        else
            SCALING_FACTOR = 0.80;


        double x1value = gamepad1.left_stick_x;
        if (Math.abs(x1value) > 0.1)
        {
            motorFrontRightPower = x1value * -1 * SCALING_FACTOR;
            motorFrontLeftPower = x1value * 1 * SCALING_FACTOR;
            motorCenterPower = x1value * -0.5 * SCALING_FACTOR;
        }
        else
        {
            int joystickPostion = getJoystickPosition();
            telemetry.addData("joystick pos", joystickPostion);

            switch (joystickPostion)
            {
                case 1:
                    motorCenterPower = 0;
                    motorFrontRightPower = gamepad1.right_stick_y * -1 * multiplier * SCALING_FACTOR;
                    motorFrontLeftPower = gamepad1.right_stick_y * -1 * multiplier * SCALING_FACTOR;
                    break;
                case 2:
                    motorCenterPower = gamepad1.right_stick_x * 1 * multiplier ; //* SCALING_FACTOR;
                    motorFrontLeftPower = gamepad1.right_stick_x * 1 * multiplier ; //* SCALING_FACTOR;
                    motorFrontRightPower = 0.10;
                    break;
                case 3:
                    motorCenterPower = gamepad1.right_stick_x  * multiplier;
                    motorFrontRightPower = -0.10 * multiplier;
                    motorFrontLeftPower = 0.10 * multiplier;
                    break;
                case 4:
                    motorCenterPower = gamepad1.right_stick_x * 1 * multiplier; // * SCALING_FACTOR;
                    motorFrontRightPower = gamepad1.right_stick_y * -1 * multiplier; // * SCALING_FACTOR;
                    motorFrontLeftPower = -0.10;
                    break;
                case 5:
                    motorCenterPower = 0;
                    motorFrontRightPower = gamepad1.right_stick_y * -1 * multiplier * SCALING_FACTOR;
                    motorFrontLeftPower = gamepad1.right_stick_y * -1 * multiplier * SCALING_FACTOR;
                    break;
                case 6:
                    motorCenterPower = gamepad1.right_stick_x * 1 * multiplier; // * SCALING_FACTOR;
                    motorFrontLeftPower = gamepad1.right_stick_y * -1 * multiplier; // * SCALING_FACTOR;
                    motorFrontRightPower = -0.10;
                    break;
                case 7:
                    motorCenterPower = gamepad1.right_stick_x * multiplier;
                    motorFrontRightPower = 0.10 * multiplier;
                    motorFrontLeftPower = -0.10 * multiplier;
                    break;
                case 8:
                    motorCenterPower = gamepad1.right_stick_x * 1 * multiplier; // * SCALING_FACTOR;
                    motorFrontRightPower = gamepad1.right_stick_x * -1 * multiplier; // * SCALING_FACTOR;
                    motorFrontLeftPower = 0.10;
                    break;
            }
        }

        robot.motorFrontRight.setPower(motorFrontRightPower );
        robot.motorFrontLeft.setPower(motorFrontLeftPower );
        robot.motorCenter.setPower(motorCenterPower );

        telemetry.addData("Robot Facing", this.forwardFacing ? "FORWARD" : "BACKWARD");
        telemetry.addData("gamepad1.right_stick_y", gamepad1.right_stick_y);
        telemetry.addData("gamepad1.right_stick_x", gamepad1.right_stick_x);

        telemetry.addData("motorFrontRightPower", motorFrontRightPower);
        telemetry.addData("motorFrontLeftPower", motorFrontLeftPower);
        telemetry.addData("motorCenterPower", motorCenterPower);
    }

    int getJoystickPosition() {
        int pos = 0;
        double x = gamepad1.right_stick_x;
        double y = gamepad1.right_stick_y;

        if (x > -Math.sin(22.5) && x < Math.sin(22.5)){
            if (y < -Math.cos(22.5)) return 1;
            if (y > Math.cos(22.5)) return 5;
        }
        if (x > -Math.sin(67.5)){
            if (y < -Math.cos(67.5)) return 2;
            if (y > Math.cos(67.5)) return 4;
        }
        if (x < Math.sin(67.5)){
            if (y < -Math.cos(67.5)) return 8;
            if (y > Math.cos(67.5)) return 6;
        }
        if (y <= -Math.cos(67.5)) return 3;
        if (y >= Math.cos(67.5)) return 7;

        return pos;
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

        robot.motorLift.setPower(motorRPPower);

        telemetry.addData("motorLift", motorRPPower);
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
