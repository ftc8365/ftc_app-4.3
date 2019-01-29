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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@TeleOp(name = "TensorFlow Object Detection", group = "Concept")
//@Disabled
public class TensorFlowObjectDetection extends LinearOpMode {

    Robot robot = new Robot();

    @Override
    public void runOpMode() {

        robot.initTensorFlowObjectDetection( hardwareMap );
        robot.initServos( hardwareMap );
        robot.setPhoneStartingPostion();

        while (!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
        }

        if (opModeIsActive())
        {
            robot.activateTensorFlowObjectDetection();
            robot.setPhoneScanPosition();

            while (opModeIsActive()) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();

                if (updatedRecognitions != null) {
                  telemetry.addData("# Object Detected", updatedRecognitions.size());
                  if (updatedRecognitions.size() == 3) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions)
                    {
                        telemetry.addData(recognition.getLabel(), "Left %.2f", recognition.getLeft());

                        if (recognition.getLabel().equals(robot.LABEL_GOLD_MINERAL))
                        {
                            goldMineralX = (int) recognition.getTop();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getTop();
                        } else {
                            silverMineral2X = (int) recognition.getTop();
                        }
                    }

                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1)
                    {
                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X)
                        {
                            telemetry.addData("Gold Mineral Position", "Left");
                        }
                        else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X)
                        {
                            telemetry.addData("Gold Mineral Position", "Right");
                        }
                        else
                        {
                            telemetry.addData("Gold Mineral Position", "Center");
                        }
                    }
                  }
                  telemetry.update();
                }
            }

            robot.tfod.shutdown();
        }

    }

}
