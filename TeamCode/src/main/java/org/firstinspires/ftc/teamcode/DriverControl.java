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
import com.qualcomm.robotcore.util.ElapsedTime;


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
            else if (gamepad1.right_trigger > 0)
                robot.turnRightTillTime(250, 75, telemetry );
            else if (gamepad1.left_trigger > 0)
                robot.turnLeftTillTime(250, 75, telemetry );
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


    private double extendIntakeToPos(int pos)
    {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        double rampUpPower = -0.10;

        while ((runtime.seconds() < 3.0) &&
                (robot.motorIntakeExtension.getCurrentPosition() >= robot.extensionPosition[pos]) )
        {
            robot.motorIntakeExtension.setPower( rampUpPower );

            if (rampUpPower > -0.75)
                rampUpPower -= 0.05;

            this.operateDriveTrain();
        }

        robot.motorIntakeExtension.setPower(0);

        return 0.0;
    }

    public double extendIntake( double currentPower )
    {
        if ( robot.intakeState != Robot.IntakeState.INTAKE_DOWN )
            return currentPower;

        double intakePower = 0;

        if (robot.extensionCounter <= 2)      // Allow at most 3 extensions
        {
            robot.extensionCounter = robot.getNextExtensionPos();

            extendIntakeToPos( robot.extensionCounter );
        }

        return intakePower;
    }

    private double retractIntakeToPos(int pos)
    {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        double rampUpPower = -0.10;

        while (runtime.seconds() < 3.0)
        {
            int posToGo = robot.extensionPosition[pos] - robot.motorIntakeExtension.getCurrentPosition();

            if (posToGo <= 0)
                break;

            if (posToGo > 200)
                robot.motorIntakeExtension.setPower(0.75);
            else
                robot.motorIntakeExtension.setPower(0.35);

            this.operateDriveTrain();
        }

        robot.motorIntakeExtension.setPower(0);

        return 0.0;
    }


    public double retractIntake( double currentPower )
    {
        if ( robot.intakeState != Robot.IntakeState.INTAKE_DOWN )
            return currentPower;

        if (robot.extensionCounter > 0) {

            robot.extensionCounter = robot.getPreviousExtensionPos();

            return retractIntakeToPos( robot.extensionCounter );
        }

        return 0;
    }


    public double lowerIntakeStep1( double currentPower )
    {
        switch (robot.intakeState)
        {
            case INTAKE_DOWN:
            case STAGE1_UP:
//            case STAGE2_UP:
            case STAGE1_DOWN:
                return currentPower;
        }

        ElapsedTime runtime = new ElapsedTime();

        double startingPos = robot.motorIntakeLeftArm.getCurrentPosition();

        double power = 0.10;

        runtime.reset();

        while ((runtime.seconds() < 3.0) &&
                (robot.motorIntakeLeftArm.getCurrentPosition() <= robot.intakeArmPosition[1] - 150))
        {
            robot.motorIntakeLeftArm.setPower(power);
            robot.motorIntakeRightArm.setPower(power);

            if (power < 0.20)
                power += 0.02;

            this.operateDriveTrain();
        }

        robot.motorIntakeLeftArm.setPower(-0.20);
        robot.motorIntakeRightArm.setPower(-0.20);

        retractIntakeToPos(1);

        robot.intakeState = Robot.IntakeState.STAGE1_DOWN;

        return -0.2;
    }

    public double lowerIntakeStep2(double currentPower)
    {
        switch (robot.intakeState)
        {
            case INTAKE_DOWN:
            case STAGE1_UP:
//            case STAGE2_UP:
            case STAGE1_DOWN:
                robot.intakeState = Robot.IntakeState.INTAKE_DOWN;
                return 0.0;
        }

        return currentPower;
    }

    public double raiseIntakeStep1( double currentPower )
    {
        if ( robot.intakeState != Robot.IntakeState.INTAKE_DOWN )
            return currentPower;

        int prevPos = robot.motorIntakeLeftArm.getCurrentPosition();
        for (int i = 0; i < 10; ++i)
        {
            sleep(50);
            if (Math.abs(prevPos - robot.motorIntakeLeftArm.getCurrentPosition()) < 2)
            {
                robot.calcArmPositions();
                break;
            }
        }

        double rampUpPower = -0.10;

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        while ((runtime.seconds() < 3.0) &&
                (robot.motorIntakeLeftArm.getCurrentPosition() >= robot.intakeArmPosition[1]))
        {
            robot.motorIntakeLeftArm.setPower( rampUpPower);
            robot.motorIntakeRightArm.setPower( rampUpPower );

            if (rampUpPower > -0.75)
                rampUpPower -= 0.02;

            this.operateDriveTrain();
        }

        robot.motorIntakeLeftArm.setPower(-0.20);
        robot.motorIntakeRightArm.setPower(-0.20);

        if (robot.motorIntakeExtension.getCurrentPosition() < robot.extensionPosition[1])
            retractIntakeToPos(1);
        else
            extendIntakeToPos(1);

        robot.intakeState = Robot.IntakeState.STAGE1_UP;

        return -0.20;
    }

    public double raiseIntakeStep2( double currentPower )
    {
        switch (robot.intakeState)
        {
//            case INTAKE_DOWN:
//            case STAGE1_UP:
            case STAGE2_UP:
            case STAGE1_DOWN:
                return currentPower;
        }

        if (robot.intakeState == Robot.IntakeState.INTAKE_DOWN)
        {
            int prevPos = robot.motorIntakeLeftArm.getCurrentPosition();
            for (int i = 0; i < 10; ++i)
            {
                sleep(50);
                if (Math.abs(prevPos - robot.motorIntakeLeftArm.getCurrentPosition()) < 2) {
                    robot.calcArmPositions();
                    break;
                }
            }

            double rampUpPower = -0.10;

            ElapsedTime runtime1 = new ElapsedTime();
            runtime1.reset();

            while ((runtime1.seconds() < 3.0) &&
                    (robot.motorIntakeLeftArm.getCurrentPosition() >= robot.intakeArmPosition[1]))
            {
                robot.motorIntakeLeftArm.setPower( rampUpPower);
                robot.motorIntakeRightArm.setPower( rampUpPower );

                if (rampUpPower > -0.75)
                    rampUpPower -= 0.02;

                this.operateDriveTrain();
            }

            robot.motorIntakeLeftArm.setPower(-0.20);
            robot.motorIntakeRightArm.setPower(-0.20);
        }

        if (robot.motorIntakeExtension.getCurrentPosition() < robot.extensionPosition[2])
            retractIntakeToPos(2);
        else
            extendIntakeToPos(2);

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        double rampUpPower = -0.25;

        while ((runtime.seconds() < 4.0) &&
                (robot.motorIntakeLeftArm.getCurrentPosition() >= robot.intakeArmPosition[2]))
        {
            robot.motorIntakeLeftArm.setPower(rampUpPower);
            robot.motorIntakeRightArm.setPower(rampUpPower);

            if (rampUpPower > -0.75)
                rampUpPower -= 0.05;

            this.operateDriveTrain();
        }

        robot.motorIntakeSpinner.setPower(-0.35);
        robot.motorIntakeLeftArm.setPower(0);
        robot.motorIntakeRightArm.setPower(0);

        sleep(1000);

        robot.intakeState = Robot.IntakeState.STAGE2_UP;

        return 0;
    }



    void operateIntake()
    {
        if ( Math.abs(gamepad2.right_stick_y) > 0.01 )
        {
            rotatingPower = robot.getRotatingPower(gamepad2.right_stick_y); //Math.pow(gamepad2.right_stick_y,3);
            useJoystick = true;
        }
        else if (useJoystick)
        {
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

        if (gamepad2.dpad_up)
        {
            rotatingPower = extendIntake( rotatingPower );
        }
        else if (gamepad2.dpad_down)
        {
            retractIntake( rotatingPower );
        }
        else if (gamepad2.b)
        {
            rotatingPower = lowerIntakeStep1(rotatingPower);
            useJoystick  = false;
        }
        else if (gamepad2.a)
        {
            rotatingPower = lowerIntakeStep2(rotatingPower);
            useJoystick  = false;
        }
        else if (gamepad2.x)
        {
            rotatingPower = raiseIntakeStep1( rotatingPower );
            motorIntakeExtensionPower = 0;
            useJoystick  = false;
        }
        else if (gamepad2.y)
        {
            rotatingPower = raiseIntakeStep2( rotatingPower );
            useJoystick  = false;
        }

        robot.motorIntakeLeftArm.setPower(rotatingPower);
        robot.motorIntakeRightArm.setPower(rotatingPower);

        robot.motorIntakeExtension.setPower(motorIntakeExtensionPower);

//        telemetry.addData("Motor Intake Power", rotatingPower);
//        telemetry.addData("Motor Extension Position", robot.motorIntakeExtension.getCurrentPosition());

//        telemetry.addData("Motor Left Intake Position", robot.motorIntakeLeftArm.getCurrentPosition());
//        telemetry.addData("Motor Right Intake Position", robot.motorIntakeRightArm.getCurrentPosition());
//        telemetry.addData("Intake State", robot.getIntakeState());
//        telemetry.addData("Arm Position", robot.motorIntakeLeftArm.getCurrentPosition());


        if (gamepad2.left_trigger > 0)
        {
            robot.motorIntakeSpinner.setPower(0.60);
        }
        else if (gamepad2.right_trigger > 0 )
        {
            robot.motorIntakeSpinner.setPower(-0.60);
        }
        else
        {
            robot.motorIntakeSpinner.setPower(0);
        }

    }

    void operateDriveTrain()
    {
        double multiplier = robot.getforwardFacing() ? 1 : -1;

        double motorFrontRightPower = 0;
        double motorFrontLeftPower  = 0;
        double motorCenterPower     = 0;

        double MAX_STEER_POWER = 0.5;
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

                    if (this.robotSteerPower < (MAX_STEER_POWER * 0.75))
                        this.robotSteerPower += 0.02;
                    else if (this.robotSteerPower < MAX_STEER_POWER)
                        this.robotSteerPower += 0.01;

                    if (robotSteerPower > x1value)
                        robotSteerPower = x1value;
                } else {
                    // Turning left
                    if (robotSteerPower > 0)
                        robotSteerPower = 0;

                    if (this.robotSteerPower > -(MAX_STEER_POWER * 0.75))
                        this.robotSteerPower -= 0.02;
                    else if (this.robotSteerPower > -MAX_STEER_POWER)
                        this.robotSteerPower -= 0.01;

                    if (robotSteerPower < x1value)
                        robotSteerPower = x1value;
                }
            }

            motorFrontRightPower    = robotSteerPower * -1.0;
            motorFrontLeftPower     = robotSteerPower *  1.0;
            motorCenterPower        = robotSteerPower * -0.5;
        }
        else if ( (Math.abs(gamepad1.right_stick_x) > 0.01) || (Math.abs(gamepad1.right_stick_y) > 0.01))
        {
            this.robotSteerPower = 0.0;

            double targetRobotPower = SCALING_FACTOR * Math.sqrt(Math.pow(gamepad1.right_stick_x, 2) + Math.pow(gamepad1.right_stick_y, 2));

            //int joystickPostion = getDirectionAwareJoystickPosition();

            int joystickPostion = getJoystickPosition();

//            telemetry.addData("joystick DA pos", getDirectionAwareJoystickPosition());
//            telemetry.addData("joystick    pos", getJoystickPosition());

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

//        telemetry.addData("robot power", robotPower);
//        telemetry.addData("Robot Facing", robot.getforwardFacing() ? "FORWARD" : "BACKWARD");
//        telemetry.addData("gamepad1.right_stick_y", gamepad1.right_stick_y);
//        telemetry.addData("gamepad1.right_stick_x", gamepad1.right_stick_x);
//        telemetry.addData("motorFrontRightPower", motorFrontRightPower);
//        telemetry.addData("motorFrontLeftPower", motorFrontLeftPower);
//        telemetry.addData("motorCenterPower", motorCenterPower);
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
