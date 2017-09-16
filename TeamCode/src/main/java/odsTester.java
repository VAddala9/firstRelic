
//FOR POTENIAL POTENTIOMETER: AnalogInput potentiometer = hardwareMap.analogInput.get("<WhateverNameYouGaveIt>");
//int value = potentiometer.getValue();
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="odsTester", group="Psionics")
//@Disabled
public class odsTester extends LinearOpMode {
    Psionics_Robot_Hardware       robot   = new Psionics_Robot_Hardware();

    OpticalDistanceSensor odsRF;
    OpticalDistanceSensor odsRB;
    OpticalDistanceSensor odsLF;
    OpticalDistanceSensor odsLB;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (1 == 1) {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
            odsRF = hardwareMap.opticalDistanceSensor.get("odsRF");
            odsRB = hardwareMap.opticalDistanceSensor.get("odsRB");
            odsLF = hardwareMap.opticalDistanceSensor.get("odsLF");
            odsLB = hardwareMap.opticalDistanceSensor.get("odsLB");
            telemetry.addData("SValue RF", odsRF.getRawLightDetected());
            telemetry.addData("SValue RB", odsRB.getRawLightDetected());
            telemetry.addData("SValue LF", odsLF.getRawLightDetected());
            telemetry.addData("SValue LB", odsLB.getRawLightDetected());
            telemetry.update();
            DbgLog.msg(String.format("Psionics_Rover : Debug - Resetting Encoders"));

        }
    }
    }





