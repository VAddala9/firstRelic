/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Optical Distance Sensor
 * It assumes that the ODS sensor is configured with a name of "ods sensor".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Sensor: MR_ODS", group = "Sensor")
//@Disabled
public class OpticalSensorTest_Autonomous extends LinearOpMode {
    private double ODsensorLD, ODsensorRLD;
    DcMotor leftMotor;
    DcMotor rightMotor;



  OpticalDistanceSensor odsSensor;
    OpticalDistanceSensor odsSensor2;
    // Hardware Device Object

  @Override
  public void runOpMode() throws InterruptedException {

    // get a reference to our Light Sensor object.
    odsSensor = hardwareMap.opticalDistanceSensor.get("ods");
      odsSensor2 = hardwareMap.opticalDistanceSensor.get("ods2");

    // wait for the start button to be pressed.
    waitForStart();

    // while the op mode is active, loop and read the light levels.
    // Note we use opModeIsActive() as our loop condition because it is an interruptible method.


    while (opModeIsActive()) {

        leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
      ODsensorLD = odsSensor.getLightDetected();
      ODsensorRLD = odsSensor.getRawLightDetected();
        // send the info back to driver station using telemetry function.
      telemetry.addData("Raw",    odsSensor.getRawLightDetected());
      telemetry.addData("Normal", odsSensor.getLightDetected());

        telemetry.addData("Raw",    odsSensor2.getRawLightDetected());
        telemetry.addData("Normal", odsSensor2.getLightDetected());
        telemetry.update();

     //   if(odsSensor.getRawLightDetected() < .10 )
      //  {
            //leftMotor.setPower(100);
            //rightMotor.setPower(70);
           // wait(5000);
        //            }

      //  else if(odsSensor.getRawLightDetected() >= .10)
      //  {
            //leftMotor.setPower(70);
            //rightMotor.setPower(40);
            //wait(5000);
              //     }

      idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
    }

  }
}
