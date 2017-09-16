/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Psionics hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PsionicsAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AutoTest5", group="Psionics")
//@Disabled
public class AutoTest5 extends LinearOpMode {

    private double ODsensorLD, ODsensorRLD, dist, ODsensorLD2, ODsensorRLD2, dist2;
    Psionics_Rover_Hardware       robot   = new Psionics_Rover_Hardware();

    OpticalDistanceSensor odsSensor1;
    OpticalDistanceSensor odsSensor2;

    // OpticalDistanceSensor odsSensor;
    // OpticalDistanceSensor odsSensor2;

    public double returnDistance(double sensorValue) {

        if(sensorValue >= .38)
        {
            dist = 1.0;
        }
        else if(sensorValue < .38 && sensorValue >=.132)
        {
            dist = 2.0;
        }

        else if(sensorValue < .132 && sensorValue >=.058)
        {
            dist = 3.0;
        }
        else if(sensorValue < .058 && sensorValue >=.039)
        {
            dist = 4.0;
        }
        else if(sensorValue < .039 && sensorValue >=.029)
        {
            dist = 5.0;
        }
        else if(sensorValue < .029 && sensorValue >=.024)
        {
            dist = 6.0;
        }
        else if(sensorValue < .024 && sensorValue >=.02)
        {
            dist = 8.0;
        }
        else
        {
            dist = 9.0;
        }

        return dist;
    }

    public double returnDistance2(double sensorValue2) {

        if(sensorValue2 >= .28)
        {
            dist2 = 1.0;
        }
        else if(sensorValue2 < .28 && sensorValue2 >=.11)
        {
            dist2 = 2.0;
        }

        else if(sensorValue2 < .11 && sensorValue2 >=.05865)
        {
            dist2 = 3.0;
        }
        else if(sensorValue2 < .05865 && sensorValue2 >=.048)
        {
            dist2 = 4.0;
        }
        else if(sensorValue2 < .048 && sensorValue2 >=.039)
        {
            dist2 = 5.0;
        }
        else if(sensorValue2 < .039 && sensorValue2 >=.029)
        {
            dist2 = 6.0;
        }
        else if(sensorValue2 < .029 && sensorValue2 >=.0195)
        {
            dist2 = 8.0;
        }
        else
        {
            dist2 = 9.0;
        }

        return dist2;
    }

    /* Declare OpMode members. */
    // Use a PsionicBot's hardware
    private ElapsedTime runtime = new ElapsedTime();


    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.5;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        odsSensor1 = hardwareMap.opticalDistanceSensor.get("ods");
        odsSensor2 = hardwareMap.opticalDistanceSensor.get("ods2");
        ODsensorLD = odsSensor1.getLightDetected();
        ODsensorRLD = odsSensor1.getRawLightDetected();
        ODsensorLD2 = odsSensor2.getLightDetected();
        ODsensorRLD2 = odsSensor2.getRawLightDetected();


        robot.init(hardwareMap);



        // Send telemetry message to signify robot waiting; //not real

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
        DbgLog.msg(String.format("Psionics_Rover : Debug - Resetting Encoders"));

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        waitForStart();

        encoderDrive(DRIVE_SPEED, -12,  -12, 5.0);  // S1: Forward 6 Inches with 5 Sec timeout
        encoderDrive(TURN_SPEED,   12, -12, 5.0);  // S2: Turn Right 12 Inches with 5 Sec timeout
        encoderDrive(DRIVE_SPEED, -36,  -36, 10.0);
        encoderDrive(TURN_SPEED,   -12, 12, 5.0);
        followWall(ODsensorRLD, ODsensorRLD2);

        telemetry.addData("Psionics_Rover_Path", "Complete");
        telemetry.update();
        DbgLog.msg(String.format("Psionics_Rover : Debug - Complete"));
    }


    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            int currentLeftPosition = robot.leftMotor.getCurrentPosition();
            int currentRightPosition = robot.rightMotor.getCurrentPosition();
            int lastCurrentLeftPosition = currentLeftPosition;
            int lastCurrentRightPosition = currentRightPosition;
            // Determine new target position, and pass to motor controller
            newLeftTarget = currentLeftPosition + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = currentRightPosition + (int) (rightInches * COUNTS_PER_INCH);

            //Send telemetry message to indicate starting position
            telemetry.addData("Psionics_Rover_Path", "Starting at %7d :%7d", currentLeftPosition, currentRightPosition);
            telemetry.update();
            DbgLog.msg(String.format("Psionics_Rover : Debug - Starting at %7d :%7d", currentLeftPosition, currentRightPosition));


            // send the info back to driver station using telemetry function.
            telemetry.addData("Raw", robot.odsSensor1.getRawLightDetected());
            telemetry.addData("Normal", robot.odsSensor1.getLightDetected());

            telemetry.addData("Raw", robot.odsSensor2.getRawLightDetected());
            telemetry.addData("Normal", robot.odsSensor2.getLightDetected());
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            //initializing debug logging variables
            boolean DbgMsglogged = true;
            double TimeLastDbgMsglogged = 0.0;

            //set robot's speed
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));

            // Send telemetry message to indicate new target location
            telemetry.addData("Psionics_Rover_Path", "Running to %7d :%7d", newLeftTarget, newRightTarget);
            telemetry.update();
            DbgLog.msg(String.format("Psionics_Rover : Debug - Running to %7d :%7d", newLeftTarget, newRightTarget));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {


                currentLeftPosition = robot.leftMotor.getCurrentPosition();
                currentRightPosition = robot.rightMotor.getCurrentPosition();

                if (runtime.milliseconds() - TimeLastDbgMsglogged >= 100.0) {
                    DbgMsglogged = false;
                    TimeLastDbgMsglogged += 100.0;
                }
                if ((lastCurrentLeftPosition != currentLeftPosition) || (lastCurrentRightPosition != currentRightPosition)) {
                    DbgMsglogged = false;
                    lastCurrentLeftPosition = currentLeftPosition;
                    lastCurrentRightPosition = currentRightPosition;
                }

                if (DbgMsglogged == false) {
                    // Display it for the driver.
                    telemetry.addData("Psionics_Rover_Path", "Running at %7d :%7d", currentLeftPosition, currentRightPosition);
                    telemetry.update();
                    DbgLog.msg(String.format("Psionics_Rover : Debug - Currently at %7d :%7d", currentLeftPosition, currentRightPosition));
                    DbgMsglogged = true;
                }

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleep(2000);   // optional pause after each move
        }
    }

    public void followWall(double sensorValue, double sensorValue2) throws InterruptedException {
        ODsensorRLD = odsSensor1.getRawLightDetected();
        ODsensorRLD2 = odsSensor2.getRawLightDetected();
        sensorValue = ODsensorRLD;
        sensorValue2 = ODsensorRLD2;
        dist = returnDistance(sensorValue);
        dist2 = returnDistance2(sensorValue2);

        while(1==1)
        {
            telemetry.addData("Psionics_Rover: in while loop", + dist + " " + dist2);
            telemetry.update();
            DbgLog.msg(String.format("Psionics_Rover: in while loop"));
            // if (opModeIsActive()) {
            ODsensorRLD = odsSensor1.getRawLightDetected();
            ODsensorRLD2 = odsSensor2.getRawLightDetected();
            sensorValue = ODsensorRLD;
            sensorValue2 = ODsensorRLD2;
            telemetry.addData("Sensorvalue", + sensorValue);
            telemetry.update();
            DbgLog.msg(String.format("SensorValuesInBeg: %5.7f: %5.7f", sensorValue, sensorValue2));
            dist = returnDistance(sensorValue);
            dist2 = returnDistance2(sensorValue2);
            telemetry.addData("dists", + dist + dist2);
            telemetry.update();
            DbgLog.msg(String.format("DistanceInBeg: %5.7f: %5.7f", dist, dist2));
            if (dist < 4.0) {
                encoderDrive(DRIVE_SPEED, 1, -1, 5.0);
                encoderDrive(DRIVE_SPEED, -3, -3, 5.0);
                encoderDrive(DRIVE_SPEED, -.85, .85, 5.0);
               // sleep(500);
                sensorValue = ODsensorRLD;
                sensorValue2 = ODsensorRLD2;
                telemetry.addData("Sensorvalue", + sensorValue);
                telemetry.update();
                DbgLog.msg(String.format("SensorValueD<4: %5.7f: %5.7f", sensorValue, sensorValue2));
                dist = returnDistance(sensorValue);
                dist2 = returnDistance2(sensorValue2);
                telemetry.addData("dists", + dist + dist2);
                telemetry.update();
                DbgLog.msg(String.format("DistanceD<4: %5.7f: %5.7f", dist, dist2));

            } else if (dist >= 4.0 && dist < 6.0) {
                encoderDrive(DRIVE_SPEED, -3, -3, 5.0);
                //sleep(500);
                sensorValue = ODsensorRLD;
                sensorValue2 = ODsensorRLD2;
                telemetry.addData("Sensorvalues", + sensorValue + sensorValue2);
                telemetry.update();
                DbgLog.msg(String.format("SensorValue4<=D<6: %5.7f: %5.7f", sensorValue, sensorValue2));
                dist = returnDistance(sensorValue);
                dist2 = returnDistance2(sensorValue2);
                telemetry.addData("dists", + dist + dist2);
                telemetry.update();
                DbgLog.msg(String.format("Distance4<=D<6: %5.7f: %5.7f", dist, dist2));

            } else if (dist >= 6.0 || dist2>dist) {
                //encoderDrive(DRIVE_SPEED, 0, 0, 5.0);
                encoderDrive(DRIVE_SPEED, 2, -2, 5.0);
                encoderDrive(DRIVE_SPEED, -5, -5, 5.0);
                encoderDrive(DRIVE_SPEED, -1.75, 1.75, 5.0);
                //sleep(500);
                sensorValue = ODsensorRLD;
                sensorValue2 = ODsensorRLD2;
                telemetry.addData("Sensorvalues", + sensorValue +sensorValue2);
                telemetry.update();
                DbgLog.msg(String.format("SensorValueD>=6: %5.7f: %5.7f", sensorValue, sensorValue2));
                dist = returnDistance(sensorValue);
                dist2 = returnDistance2(sensorValue2);
                telemetry.addData("dists", + dist +dist2);
                telemetry.update();
                DbgLog.msg(String.format("DistanceD>=6: %5.7f: %5.7f", dist, dist2));

            }
            idle();
        }
    }

    /*public void followWall(double sensorValue, double sensorValue2) throws InterruptedException {
        ODsensorRLD = odsSensor1.getRawLightDetected();
        ODsensorRLD2 = odsSensor2.getRawLightDetected();
        sensorValue = ODsensorRLD;
        sensorValue2 = ODsensorRLD2;
        dist = returnDistance(sensorValue);
        dist2 = returnDistance2(sensorValue2);

        while(1==1)
        {
            telemetry.addData("Psionics_Rover: in while loop", + dist + " " + dist2);
            telemetry.update();
            DbgLog.msg(String.format("Psionics_Rover: in while loop"));
            // if (opModeIsActive()) {
            ODsensorRLD = odsSensor1.getRawLightDetected();
            ODsensorRLD2 = odsSensor2.getRawLightDetected();
            sensorValue = ODsensorRLD;
            sensorValue2 = ODsensorRLD2;
            telemetry.addData("Sensorvalue", + sensorValue);
            telemetry.update();
            DbgLog.msg(String.format("SensorValuesInBeg: %5.7f: %5.7f", sensorValue, sensorValue2));
            dist = returnDistance(sensorValue);
            dist2 = returnDistance2(sensorValue2);
            telemetry.addData("dists", + dist + dist2);
            telemetry.update();
            DbgLog.msg(String.format("DistanceInBeg: %5.7f: %5.7f", dist, dist2));
            if (dist < 2.0) {
                encoderDrive(DRIVE_SPEED, 1, -1, 5.0);
                encoderDrive(DRIVE_SPEED, -3, -3, 5.0);
                encoderDrive(DRIVE_SPEED, -1, 1, 5.0);
                sleep(1000);
                sensorValue = ODsensorRLD;
                sensorValue2 = ODsensorRLD2;
                telemetry.addData("Sensorvalue", + sensorValue);
                telemetry.update();
                DbgLog.msg(String.format("SensorValueD<2: %5.7f: %5.7f", sensorValue, sensorValue2));
                dist = returnDistance(sensorValue);
                dist2 = returnDistance2(sensorValue2);
                telemetry.addData("dists", + dist + dist2);
                telemetry.update();
                DbgLog.msg(String.format("DistanceD<2: %5.7f: %5.7f", dist, dist2));

            } else if (dist >= 2.0 && dist < 4.0) {
                encoderDrive(DRIVE_SPEED, -3, -3, 5.0);
                sleep(1000);
                sensorValue = ODsensorRLD;
                sensorValue2 = ODsensorRLD2;
                telemetry.addData("Sensorvalues", + sensorValue + sensorValue2);
                telemetry.update();
                DbgLog.msg(String.format("SensorValue2<=D<4: %5.7f: %5.7f", sensorValue, sensorValue2));
                dist = returnDistance(sensorValue);
                dist2 = returnDistance2(sensorValue2);
                telemetry.addData("dists", + dist + dist2);
                telemetry.update();
                DbgLog.msg(String.format("Distance2<=D<4: %5.7f: %5.7f", dist, dist2));

            } else if (dist >= 4.0) {
                //encoderDrive(DRIVE_SPEED, 0, 0, 5.0);
                encoderDrive(DRIVE_SPEED, 2, -2, 5.0);
                encoderDrive(DRIVE_SPEED, -5, -5, 5.0);
                encoderDrive(DRIVE_SPEED, -1.75, 1.75, 5.0);
                sleep(1000);
                sensorValue = ODsensorRLD;
                sensorValue2 = ODsensorRLD2;
                telemetry.addData("Sensorvalues", + sensorValue +sensorValue2);
                telemetry.update();
                DbgLog.msg(String.format("SensorValueD>=4: %5.7f: %5.7f", sensorValue, sensorValue2));
                dist = returnDistance(sensorValue);
                dist2 = returnDistance2(sensorValue2);
                telemetry.addData("dists", + dist +dist2);
                telemetry.update();
                DbgLog.msg(String.format("DistanceD>=4: %5.7f: %5.7f", dist, dist2));

            }
            idle();
        }
    }*/

}

