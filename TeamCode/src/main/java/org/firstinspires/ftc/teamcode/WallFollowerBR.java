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
package org.firstinspires.ftc.teamcode;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="WallFollowerBR", group="Psionics")
//@Disabled
public class WallFollowerBR  extends LinearOpMode {

    Psionics_Robot_Hardware2 robot = new Psionics_Robot_Hardware2();

    double SVLF;
    double SVLB;
    double SVRF;
    double SVRB;

    OpticalDistanceSensor odsLF;
    OpticalDistanceSensor odsLB;
    OpticalDistanceSensor odsRF;
    OpticalDistanceSensor odsRB;

    // Use a PsionicBot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 5.0;     // For figuring circumference
    static final double DISTANCE_EROR_COMPENSATION = 60 / 55.5;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (DISTANCE_EROR_COMPENSATION * WHEEL_DIAMETER_INCHES * 3.1415);
    static final double RIGHT_SPEED_COMPENSATION = 2.1;
    static final double DRIVE_SPEED = 0.4;
    static final double TURN_SPEED = 0.5;

    public double getDistLF(double sensorValue) {
        double dist = 0.0;
        if (sensorValue > 0 && sensorValue <= 1) {
            dist = 2.0;
        }
        else if (sensorValue > 1 && sensorValue <= 2) {
            dist = 4.0;
        }

        else if (sensorValue > 2 && sensorValue <= 3) {
            dist = 6.0;
        }

        else if (sensorValue > 3 && sensorValue <= 4) {
            dist = 8.0;
        }
        return dist;
    }

    public double getDistRF(double sensorValue) {
        double dist = 0.0;
        if (sensorValue > 0 && sensorValue <= 1) {
            dist = 2.0;
        }
        else if (sensorValue > 1 && sensorValue <= 2) {
            dist = 4.0;
        }

        else if (sensorValue > 2 && sensorValue <= 3) {
            dist = 6.0;
        }

        else if (sensorValue > 3 && sensorValue <= 4) {
            dist = 8.0;
        }
        return dist;
    }

    public double getDistLB(double sensorValue) {
        double dist = 0.0;
        if (sensorValue > 0 && sensorValue <= 1) {
            dist = 2.0;
        }
        else if (sensorValue > 1 && sensorValue <= 2) {
            dist = 4.0;
        }

        else if (sensorValue > 2 && sensorValue <= 3) {
            dist = 6.0;
        }

        else if (sensorValue > 3 && sensorValue <= 4) {
            dist = 8.0;
        }
        return dist;
    }

    public double getDistRB(double sensorValue) {
        double dist = 0.0;
        if (sensorValue > 0 && sensorValue <= 1) {
            dist = 2.0;
        }
        else if (sensorValue > 1 && sensorValue <= 2) {
            dist = 4.0;
        }

        else if (sensorValue > 2 && sensorValue <= 3) {
            dist = 6.0;
        }

        else if (sensorValue > 3 && sensorValue <= 4) {
            dist = 8.0;
        }
        return dist;
    }


    @Override
    public void runOpMode() throws InterruptedException {

        odsLF = hardwareMap.opticalDistanceSensor.get("odsBL1");
        odsLB = hardwareMap.opticalDistanceSensor.get("odsBL2");
        odsRF = hardwareMap.opticalDistanceSensor.get("odsBR1");
        odsRB = hardwareMap.opticalDistanceSensor.get("odsBR2");
        SVLF = odsLF.getRawLightDetected();
        SVLB = odsLB.getRawLightDetected();
        SVRF = odsRF.getRawLightDetected();
        SVRB = odsRB.getRawLightDetected();

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting; //not real

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
        DbgLog.msg(String.format("Psionics_Bot : Debug - Resetting Encoders"));

        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        //robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        // encoderDrive(DRIVE_SPEED, 60, 60, 60,  60, 60.0);
        //
        //
        // sleep(2000);
        //encoderDrive(TURN_SPEED, 12, -12, 12,  -12, 60.0);
  while(opModeIsActive()) {//sleep(2000);//encoderDrive(TURN_SPEED, -12, 12, -12,  12, 60.0);
      SVLF = odsLF.getRawLightDetected();
      SVLB = odsLB.getRawLightDetected();
      SVRF = odsRF.getRawLightDetected();
      SVRB = odsRB.getRawLightDetected();

      telemetry.addData("L1", SVLF);
      telemetry.addData("L2", SVLB);
      telemetry.addData("R1", SVRF);
      telemetry.addData("R2", SVRB);
      telemetry.update();
  }
     //   followWallR(SVRF, SVRB);

     //   followWallL(SVLF, SVLB);
        //   sleep(5000);
      //  lineFollowB(SVL1, SVR1);
        // S1: Forward 60 Inches with 60 Sec timeout
        //encoderDrive(TURN_SPEED,   12, -12, 5.0);  // S2: Turn Right 12 Inches with 5 Sec timeout
    }
    public void encoderDrive(double speed,
                             double leftFrontInches, double rightFrontInches, double leftBackInches, double rightBackInches,
                             double timeoutS) throws InterruptedException {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            int currentLeftFrontPosition = robot.leftFrontMotor.getCurrentPosition();
            int currentRightFrontPosition = robot.rightFrontMotor.getCurrentPosition();
            int currentLeftBackPosition = robot.leftBackMotor.getCurrentPosition();
            int currentRightBackPosition = robot.rightBackMotor.getCurrentPosition();

            int lastCurrentLeftFrontPosition = currentLeftFrontPosition;
            int lastCurrentRightFrontPosition = currentRightFrontPosition;
            int lastCurrentLeftBackPosition = currentLeftBackPosition;
            int lastCurrentRightBackPosition = currentRightBackPosition;
            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = currentLeftFrontPosition + (int) (leftFrontInches * COUNTS_PER_INCH);
            newRightFrontTarget = currentRightFrontPosition + (int) (rightFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget = currentLeftBackPosition + (int) (leftBackInches * COUNTS_PER_INCH);
            newRightBackTarget = currentRightBackPosition + (int) (rightBackInches * COUNTS_PER_INCH);


            //Send telemetry message to indicate starting position
            telemetry.addData("Psionics_Bot_Path", "Starting at %7d :%7d :%7d :%7d", currentLeftFrontPosition, currentRightFrontPosition, currentLeftBackPosition, currentRightBackPosition);
            telemetry.update();
            DbgLog.msg(String.format("Psionics_Bot : Debug - Starting at %7d :%7d :%7d :%7d", currentLeftFrontPosition, currentRightFrontPosition, currentLeftBackPosition, currentRightBackPosition));

            robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);
            robot.leftBackMotor.setTargetPosition(newLeftBackTarget);
            robot.rightBackMotor.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            //initializing debug logging variables
            boolean DbgMsglogged = true;
            double TimeLastDbgMsglogged = 0.0;

            //set robot's speed
            robot.leftFrontMotor.setPower(Math.abs(speed));
            robot.rightFrontMotor.setPower(Math.abs(speed) * RIGHT_SPEED_COMPENSATION);
            robot.leftBackMotor.setPower(Math.abs(speed));
            robot.rightBackMotor.setPower(Math.abs(speed) * RIGHT_SPEED_COMPENSATION);

            // Send telemetry message to indicate new target location
            telemetry.addData("Psionics_Bot_Path", "Running to %7d :%7d :%7d :%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
            telemetry.update();
            DbgLog.msg(String.format("Psionics_Bot : Debug - Running to %7d :%7d :%7d :%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy() && robot.leftBackMotor.isBusy() && robot.rightBackMotor.isBusy())) {

                currentLeftFrontPosition = robot.leftFrontMotor.getCurrentPosition();
                currentRightFrontPosition = robot.rightFrontMotor.getCurrentPosition();
                currentLeftBackPosition = robot.leftBackMotor.getCurrentPosition();
                currentRightBackPosition = robot.rightBackMotor.getCurrentPosition();


                if (runtime.milliseconds() - TimeLastDbgMsglogged >= 100.0) {
                    DbgMsglogged = false;
                    TimeLastDbgMsglogged += 100.0;
                }
                if ((lastCurrentLeftFrontPosition != currentLeftFrontPosition) || (lastCurrentRightFrontPosition != currentRightFrontPosition) || (lastCurrentLeftBackPosition != currentLeftBackPosition) || (lastCurrentRightBackPosition != currentRightBackPosition)) {
                    DbgMsglogged = false;
                    lastCurrentLeftFrontPosition = currentLeftFrontPosition;
                    lastCurrentRightFrontPosition = currentRightFrontPosition;
                    lastCurrentLeftBackPosition = currentLeftBackPosition;
                    lastCurrentRightBackPosition = currentRightBackPosition;
                }

                if (DbgMsglogged == false) {
                    // Display it for the driver.
                    telemetry.addData("Psionics_Bot_Path", "Running at %7d :%7d :%7d :%7d", currentLeftFrontPosition, currentRightFrontPosition, currentLeftBackPosition, currentRightBackPosition);
                    telemetry.update();
                    DbgLog.msg(String.format("Psionics_Bot : Debug - Currently at %7d :%7d :%7d :%7d", currentLeftFrontPosition, currentRightFrontPosition, currentLeftBackPosition, currentRightBackPosition));
                    DbgMsglogged = true;
                }

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            robot.leftFrontMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
            robot.leftBackMotor.setPower(0);
            robot.rightBackMotor.setPower(0);

            currentLeftFrontPosition = robot.leftFrontMotor.getCurrentPosition();
            currentRightFrontPosition = robot.rightFrontMotor.getCurrentPosition();
            currentLeftBackPosition = robot.leftBackMotor.getCurrentPosition();
            currentRightBackPosition = robot.rightBackMotor.getCurrentPosition();

            DbgLog.msg(String.format("Psionics_Bot : Debug - Finally at %7d :%7d :%7d :%7d", currentLeftFrontPosition, currentRightFrontPosition, currentLeftBackPosition, currentRightBackPosition));

            // Turn off RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleep(2000);   // optional pause after each move
        }
    }

    public void followWallR(double sensorValueF, double sensorValueB) throws InterruptedException {
        while (1 == 1) {

            sensorValueF = odsRF.getRawLightDetected();
            sensorValueB = odsRB.getRawLightDetected();

           double distF = getDistRF(sensorValueF); //right way to call this?
           double distB = getDistRB(sensorValueB);
            telemetry.addData("dist F:", distF);
            telemetry.addData("dist R:", distB);
            telemetry.update();

            if (distF == 2)
            {
                encoderDrive(DRIVE_SPEED, -3, 3, -3, 3, 3.0);
                encoderDrive(DRIVE_SPEED, 3, 3, 3, 3, 3.0);
                encoderDrive(DRIVE_SPEED, 3, -3, 3, -3, 3.0);
                sleep(1000);
            }
            else if (distF == 4)
            {
                encoderDrive(DRIVE_SPEED, -3, 3, -3, 3, 3.0);
                encoderDrive(DRIVE_SPEED, 3, 3, 3, 3, 3.0);
                encoderDrive(DRIVE_SPEED, 3, -3, 3, -3, 3.0);
                sleep(1000);
            }
            else if (distF == 6)
            {
                encoderDrive(DRIVE_SPEED, 3, 3, 3, 3, 3.0);
                sleep(1000);
            }
            else if (distF == 8)
            {
                encoderDrive(DRIVE_SPEED, 3, -3, 3, -3, 3.0);
                encoderDrive(DRIVE_SPEED, 3, 3, 3, 3, 3.0);
                encoderDrive(DRIVE_SPEED, -3, 3, -3, 3, 3.0);
                sleep(1000);
            }
            if(distF > distB)

            {
                encoderDrive(DRIVE_SPEED, -3, 3, -3, 3, 3.0);
                sleep(1000);
            }
            else if(distF < distB)

            {
                encoderDrive(DRIVE_SPEED, 3, -3, 3, -3, 3.0);
                sleep(1000);
            }
        }
    }

    public void followWallL(double sensorValueF, double sensorValueB) throws InterruptedException {
        while (1 == 1) {

            sensorValueF = odsLF.getRawLightDetected();
            sensorValueB = odsLB.getRawLightDetected();

            double distF = getDistRF(sensorValueF); //right way to call this?
            double distB = getDistRB(sensorValueB);
            telemetry.addData("dist F:", distF);
            telemetry.addData("dist R:", distB);
            telemetry.update();

            if (distF == 2)
            {
                encoderDrive(DRIVE_SPEED, 3, -3, 3, -3, 3.0);
                encoderDrive(DRIVE_SPEED, 3, 3, 3, 3, 3.0);
                encoderDrive(DRIVE_SPEED, -3, 3, -3, 3, 3.0);
                sleep(1000);
            }
            else if (distF == 4)
            {
                encoderDrive(DRIVE_SPEED, 3, -3, 3, -3, 3.0);
                encoderDrive(DRIVE_SPEED, 3, 3, 3, 3, 3.0);
                encoderDrive(DRIVE_SPEED, -3, 3, -3, 3, 3.0);
                sleep(1000);
            }
            else if (distF == 6)
            {
                encoderDrive(DRIVE_SPEED, 3, 3, 3, 3, 3.0);
                sleep(1000);
            }
            else if (distF == 8)
            {
                encoderDrive(DRIVE_SPEED, -3, 3, -3, 3, 3.0);
                encoderDrive(DRIVE_SPEED, 3, 3, 3, 3, 3.0);
                encoderDrive(DRIVE_SPEED, 3, -3, 3, -3, 3.0);
                sleep(1000);
            }
            if(distF > distB)

            {
                encoderDrive(DRIVE_SPEED, 3, -3, 3, -3, 3.0);
                sleep(1000);
            }
            else if(distF < distB)

            {
                encoderDrive(DRIVE_SPEED, -3, 3, -3, 3, 3.0);
                sleep(1000);
            }
        }
    }


}
