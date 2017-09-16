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

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

/*
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Optical Distance Sensor
 * It assumes that the ODS sensor is configured with a name of "ods sensor".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list....8
 */
@TeleOp(name = "Sensor: MR ODS2", group = "Sensor")
//@Disabled
public class OST2 extends LinearOpMode {

    private double ODsensorLD, ODsensorRLD, dist;

    OpticalDistanceSensor odsSensor;
    OpticalDistanceSensor odsSensor2;// Hardware Device Object
    /*private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;*/

    public double returnDistanceCardboard(double sensorValue) {
        //    double distance = 0.0;

        // private double distance = Math.round(((6.7*Math.sqrt(1/ODsensorRLD))/(Math.sqrt(1/.19))-6) *10);
        //dist = (-70 * sensorValue) + 14.5;
        //dist = Math.sqrt(1/sensorValue);
       /* if(sensorValue >= .35)
        {
            distance = 2.0;
        }

        else if(sensorValue < .35 && sensorValue >=.19)
        {
            distance = 3.0;
        }
        else if(sensorValue < .19 && sensorValue >=.13)
        {
            distance = 4.0;
        }
        else if(sensorValue < .13 && sensorValue >=.08)
        {
            distance = 5.0;
        }
        else if(sensorValue < .08 && sensorValue >=.06)
        {
            distance = 6;
        }
        else if(sensorValue < .06 && sensorValue >=.05)
        {
            distance = 7.0;
        }

        else if(sensorValue < .05 && sensorValue >=.045)
        {
            distance = 8.0;
        }
        else if(sensorValue < .045 && sensorValue >=.04)
        {
            distance = 9.0;
        }
        else if(sensorValue < .04 && sensorValue >=.035)
        {
            distance = 10.0;
        }
        else if(sensorValue < .035 && sensorValue >=.032)
        {
            distance = 11.0;
        } else if(sensorValue < .032 && sensorValue >=.030)
        {
            distance = 12.0;
        }else if(sensorValue < .030 && sensorValue >=.029)
        {
            distance = 13.0;
        }else if(sensorValue < .029 && sensorValue >=.027)
        {
            distance = 14.0;
        }else if(sensorValue < .027 && sensorValue >=.024)
        {
            distance = 15.0;
        }else if(sensorValue < .024 && sensorValue >=.023)
        {
            distance = 16.0;
        }else if(sensorValue < .023 && sensorValue >=.022)
        {
            distance = 17.0;
        }else if(sensorValue < .022 && sensorValue >=.021)
        {
            distance = 18.0;
        }*/
 return dist;
    }
    public double returnDistance(double sensorValue) {
            //double distance = 0.0;

        // private double distance = Math.round(((6.7*Math.sqrt(1/ODsensorRLD))/(Math.sqrt(1/.19))-6) *10);
        //dist = (-70 * sensorValue) + 14.5;
        //dist = Math.sqrt(1/sensorValue);
        if(sensorValue >= .35)
        {
            dist = 1.0;
        }
        else if(sensorValue < .36 && sensorValue >=.132)
        {
            dist = 2.0;
        }

        else if(sensorValue < .44 && sensorValue >=.36)
        {
            dist = 3.0;
        }
        else if(sensorValue < .36 && sensorValue >=.44)
        {
            dist = 4.0;
        }
        else if(sensorValue < .33 && sensorValue >=.36)
        {
            dist = 5.0;
        }
        else if(sensorValue < .29 && sensorValue >=.33)
        {
            dist = 6.0;
        }
        else if(sensorValue < .25 && sensorValue >=.29)
        {
            dist = 7.0;
        }

        else if(sensorValue < .195 && sensorValue >=.25)
        {
            dist = 8.0;
        }
        else
        {
            dist = 9.0;
        }

        return dist;
    }
    @Override
    public void runOpMode() throws InterruptedException {

        // get a reference to our Light Sensor object.
        odsSensor = hardwareMap.opticalDistanceSensor.get("ods");
        odsSensor2 = hardwareMap.opticalDistanceSensor.get("ods2");
       /* leftMotor  = hardwareMap.dcMotor.get("left motor");
        rightMotor = hardwareMap.dcMotor.get("right motor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors*/


        // wait for the start button to be pressed.
        waitForStart();
     //   runtime.reset();

        // while the op mode is active, loop and read the light levels.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            DbgLog.msg(String.format("ODS test : inOpMode"));

            ODsensorLD = odsSensor.getLightDetected();
            ODsensorRLD = odsSensor.getRawLightDetected();

            // send the info back to driver station using telemetry function
            // telemetry.addData("Special Raw",  returnDistance(ODsensorRLD));
            telemetry.addData("Raw",    odsSensor.getRawLightDetected());
            telemetry.addData("Normal", odsSensor.getLightDetected());

            telemetry.addData("Raw",    odsSensor2.getRawLightDetected());
            telemetry.addData("Normal", odsSensor2.getLightDetected());

         //   telemetry.addData("Status", "Run Time: " + runtime.toString());

            /*if(ODsensorLD > .01){
                leftMotor.setPower(100.0);
                rightMotor.setPower(100.0);
                //Thread.sleep(5000);
                TimeUnit.SECONDS.sleep(5);
                leftMotor.setPower(0.0);
                rightMotor.setPower(0.0);
                TimeUnit.SECONDS.sleep(5);
                leftMotor.setPower(75.0);
                rightMotor.setPower(50.0);
                TimeUnit.SECONDS.sleep(5);
            }*/


          /*  dist = returnDistance(ODsensorRLD);
            if(dist < 4.0)
            {
                leftMotor.setPower(50);
                rightMotor.setPower(60);
            }
            else if(dist >= 4.0 && dist < 6.0)
            {
                leftMotor.setPower(60);
                rightMotor.setPower(60);
            }
            else if(dist >= 6.0)
            {
                leftMotor.setPower(60);
                rightMotor.setPower(50);
            }*/


            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

    }
}

