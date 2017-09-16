import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto_R_encoder_noWall_noLine", group="Psionics")
//@Disabled
public class Auto_R_encoder_noWall_noLine extends LinearOpMode {

    //private double ODsensorLD, ODsensorRLD, dist, ODsensorLD2, ODsensorRLD2, dist2;
    private double ODSBL1,ODSBR1, ODSBL2, ODSBR2;
    //private String colorBL1, colorBR1, colorBL2, colorBR2;
    private String color, colorBL1, colorBR1, colorBL2, colorBR2;
    Psionics_Robot_Hardware       robot   = new Psionics_Robot_Hardware();

    OpticalDistanceSensor odsBL1;
    OpticalDistanceSensor odsBR1;
    OpticalDistanceSensor odsBL2;
    OpticalDistanceSensor odsBR2;

    // OpticalDistanceSensor odsSensor;
    // OpticalDistanceSensor odsSensor2;

    //public int returnColor(double sensorValue) {
    public String returnColor(double sensorValue) {

       // if(sensorValue >= .02)
        if(sensorValue >= 3)
        {
            color = "white";
        }
        else{
            color = "gray";
        }
        return color;
    }
    public void setSensors1(double sensorValueBL1, double sensorValueBR1)
    {
        ODSBL1 = odsBL1.getRawLightDetected();
        ODSBR1 = odsBR1.getRawLightDetected();
        sensorValueBL1 = ODSBL1;
        sensorValueBR1 = ODSBR1;

    }
    public void setSensors2(double sensorValueBL2, double sensorValueBR2)
    {
        ODSBL2 = odsBL2.getRawLightDetected();
        ODSBR2 = odsBR2.getRawLightDetected();
        sensorValueBL2 = ODSBL2;
        sensorValueBL2 = ODSBR2;
    }
    private ElapsedTime runtime = new ElapsedTime();


    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 5.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.125;
    static final double TURN_SPEED = 0.25;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        double back_left_motor_pwr = 0.125;
        double back_right_motor_pwr = 0.125;
        double front_left_motor_pwr = 0.125;
        double front_right_motor_pwr = 0.125;
        double time_out = 1.0;

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
        DbgLog.msg(String.format("V1DBG: Psionics_Rover : Debug - Resetting Encoders"));

        idle();
        waitForStart();

	//move forward
        telemetry.addData("forward 9(1st)", "Complete");
        telemetry.update();
        DbgLog.msg(String.format("V1DBG: before forward 9(1st) - Complete"));

        back_left_motor_pwr = -0.125;
        back_right_motor_pwr = -0.125;
        front_left_motor_pwr = -0.125;
        front_right_motor_pwr = -0.125;
	time_out =3.0;
        timedDrive(back_left_motor_pwr, back_right_motor_pwr, front_left_motor_pwr, front_right_motor_pwr, time_out);
        //timedDrive(-.125, -.125,  -.125, -.125, 1.0);  // S1: Forward 6 Inches with 5 Sec timeout

        telemetry.addData("forward 9(1st)", "Complete");
        telemetry.update();
        DbgLog.msg(String.format("V1DBG: forward 9(1st) - Complete"));

        sleep(1000);
        idle();

//rotate right
        back_left_motor_pwr = 0.125;
        back_right_motor_pwr = -0.125;
        front_left_motor_pwr = 0.125;
        front_right_motor_pwr = -0.125;
	time_out =1.0;
        timedDrive(back_left_motor_pwr, back_right_motor_pwr, front_left_motor_pwr, front_right_motor_pwr, time_out);
        //timedDrive(-6, 6, -6, 6, 1.0);

        telemetry.addData("Psionics_Rover_Path", "Complete");
        telemetry.update();
        DbgLog.msg(String.format("V1DBG: turn 12(2nd)- Complete"));

        sleep(1000);
        idle();

	//shoot
	//
	//go towards the wall
        back_left_motor_pwr = -0.125;
        back_right_motor_pwr = -0.125;
        front_left_motor_pwr = -0.125;
        front_right_motor_pwr = -0.125;
	time_out =2.0;
        timedDrive(back_left_motor_pwr, back_right_motor_pwr, front_left_motor_pwr, front_right_motor_pwr, time_out);

	
        telemetry.addData("forward 9(2nd)", "Complete");
        telemetry.update();
        DbgLog.msg(String.format("V1DBG: sleep after turn 12(2nd) - Complete"));
	//
	//left turn 
        back_left_motor_pwr = -0.125;
        back_right_motor_pwr = 0.125;
        front_left_motor_pwr = -0.125;
        front_right_motor_pwr = 0.125;
	time_out =1.0;
        timedDrive(back_left_motor_pwr, back_right_motor_pwr, front_left_motor_pwr, front_right_motor_pwr, time_out);

        //follow wall
	back_left_motor_pwr = -0.125;
        back_right_motor_pwr =-0.125;
        front_left_motor_pwr = -0.125;
        front_right_motor_pwr = -0.125;
	time_out =3.0;
        timedDrive(back_left_motor_pwr, back_right_motor_pwr, front_left_motor_pwr, front_right_motor_pwr, time_out);

        telemetry.addData("Psionics_Rover_Path", "Complete");
        telemetry.update();
        DbgLog.msg(String.format("V1DBG: forward 3(3rd) - Complete"));

        sleep(1000);

        idle();

	//left turn to face camera toward beacon
	back_left_motor_pwr = -0.125;
        back_right_motor_pwr = 0.125;
        front_left_motor_pwr = -0.125;
        front_right_motor_pwr = 0.125;
	time_out =1.0;
        timedDrive(back_left_motor_pwr, back_right_motor_pwr, front_left_motor_pwr, front_right_motor_pwr, time_out);

        telemetry.addData("forward 9(1st)", "Complete");
        telemetry.update();
        DbgLog.msg(String.format("V1DBG: sleep after forward 3(3rd) - Complete"));

	//go toward beacon and stop before beacon
	back_left_motor_pwr =0.125;
        back_right_motor_pwr = 0.125;
        front_left_motor_pwr = 0.125;
        front_right_motor_pwr = 0.125;
	time_out =1.0;
        timedDrive(back_left_motor_pwr, back_right_motor_pwr, front_left_motor_pwr, front_right_motor_pwr, time_out);


        telemetry.addData("Psionics_Rover_Path", "Complete");
        telemetry.update();
        DbgLog.msg(String.format("V1DBG: turn 12(4th)- Complete"));

        sleep(1000);

        idle();

    }

   /* 
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        //
        // Initialize the drive system variables.
        // The init() method of the hardware class does all the work here
        //

        odsBL1 = hardwareMap.opticalDistanceSensor.get("odsBL1");
        odsBR1 = hardwareMap.opticalDistanceSensor.get("odsBR1");
        odsBL2 = hardwareMap.opticalDistanceSensor.get("odsBL2");
        odsBR2 = hardwareMap.opticalDistanceSensor.get("odsBR2");
        ODSBL1 = odsBL1.getRawLightDetected();
        ODSBR1 = odsBR1.getRawLightDetected();
        ODSBL2 = odsBL2.getRawLightDetected();
        ODSBR2 = odsBR2.getRawLightDetected();


        // Send telemetry message to signify robot waiting; //not real

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
        DbgLog.msg(String.format("V1DBG: Psionics_Rover : Debug - Resetting Encoders"));

        idle();
        waitForStart();

        telemetry.addData("forward 9(1st)", "Complete");
        telemetry.update();
        DbgLog.msg(String.format("V1DBG: before forward 9(1st) - Complete"));

        encoderDrive(DRIVE_SPEED, -3,  -3, 1.0);  // S1: Forward 6 Inches with 5 Sec timeout

        telemetry.addData("forward 9(1st)", "Complete");
        telemetry.update();
        DbgLog.msg(String.format("V1DBG: forward 9(1st) - Complete"));

        sleep(1000);
        idle();

        telemetry.addData("test 9(1st)", "Complete");
        telemetry.update();
        DbgLog.msg(String.format("V1DBG: test 9(1st) - Complete"));


        telemetry.addData("forward 9(1st)", "First drive done");
        telemetry.update();
        DbgLog.msg(String.format("V1DBG: sleep after forward 9(1st) - Complete"));

        encoderDrive(TURN_SPEED, 6, -6, 1.0);

        telemetry.addData("Psionics_Rover_Path", "Complete");
        telemetry.update();
        DbgLog.msg(String.format("V1DBG: turn 12(2nd)- Complete"));

        sleep(1000);
        idle();

        telemetry.addData("forward 9(2nd)", "Complete");
        telemetry.update();
        DbgLog.msg(String.format("V1DBG: sleep after turn 12(2nd) - Complete"));

        encoderDrive(DRIVE_SPEED, -3, -3, 1.0);

        telemetry.addData("Psionics_Rover_Path", "Complete");
        telemetry.update();
        DbgLog.msg(String.format("V1DBG: forward 3(3rd) - Complete"));

        sleep(1000);

        idle();
        telemetry.addData("forward 9(1st)", "Complete");
        telemetry.update();
        DbgLog.msg(String.format("V1DBG: sleep after forward 3(3rd) - Complete"));

        encoderDrive(TURN_SPEED, -2, 2, 1.0);

        telemetry.addData("Psionics_Rover_Path", "Complete");
        telemetry.update();
        DbgLog.msg(String.format("V1DBG: turn 12(4th)- Complete"));

        sleep(1000);

        idle();

        encoderDrive(DRIVE_SPEED, -3, -3, 1.0);

        telemetry.addData("Psionics_Rover_Path", "Wall Follow start");
        telemetry.update();
        DbgLog.msg(String.format("V1DBG: forward Wall follow - Start"));

        sleep(1000);

        idle();

        telemetry.addData("forward 9(1st)", "Complete");
        telemetry.update();
        DbgLog.msg(String.format("V1DBG: sleep after wall follow - Complete"));

        encoderDrive(TURN_SPEED, 4, -4, 1.0);

        sleep(1000);

        idle();

        telemetry.addData("Psionics_Rover_Path", "Start Line Following");
        telemetry.update();
        DbgLog.msg(String.format("V1DBG: turn for Line Following "));

        encoderDrive(DRIVE_SPEED, -3, -3, 1.0);

        telemetry.addData("Psionics_Rover_Path", "Wall Follow");
        telemetry.update();
        DbgLog.msg(String.format("V1DBG: forward Wall follow"));

        sleep(1000);

        idle();


        followLine1(ODSBL1, ODSBR1);

        sleep(1000);

        idle();

        telemetry.addData("Psionics_Rover_Path", "Complete");
        telemetry.update();
        DbgLog.msg(String.format("Psionics_Rover : Debug - Complete"));
    }
*/

    public void timedDrive(double speed1, double speed2, double speed3, double speed4,
                             double timeoutS) throws InterruptedException 

    {
        telemetry.addData("Psionics_Rover_Path", "Complete");
        telemetry.update();
        DbgLog.msg(String.format("in timedDrive"));
         

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            telemetry.addData("Psionics_Rover_Path", "Complete");
            telemetry.update();
            DbgLog.msg(String.format("V1DBG: opModeIsActive -- encoderDrive"));

idle();

            // reset the timeout time and start motion.
            runtime.reset();


            //set robot's speed
            robot.leftBackMotor.setPower(speed1);
            robot.rightBackMotor.setPower(speed2);
            robot.leftFrontMotor.setPower(speed3);
            robot.rightFrontMotor.setPower(speed4);

            telemetry.addData("V1DBG: after run to position", "Complete");
            telemetry.update();
            idle();
            DbgLog.msg(String.format("V1DBG: speeds of motors assigned"));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftBackMotor.isBusy() && robot.rightBackMotor.isBusy())) {

                telemetry.addData("after run to position", "Complete");
                telemetry.update();
                idle();
                DbgLog.msg(String.format("V1DBG: while opmode active loop"));

                // Allow time for other processes to run.
                idle();
                telemetry.addData("after run to position", "Complete");
                telemetry.update();
                DbgLog.msg(String.format("V1DBG: after idle statement"));
            }

            // Stop all motion;
            robot.leftBackMotor.setPower(0);
            robot.rightBackMotor.setPower(0);

            robot.leftFrontMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);

            telemetry.addData("after run to position", "Complete");
            telemetry.update();
            idle();
            DbgLog.msg(String.format("V1DBG: stopped motors"));


            //sleep(2000);   // optional pause after each move
        }
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException 
    {
        telemetry.addData("Psionics_Rover_Path", "Complete");
        telemetry.update();
        DbgLog.msg(String.format("in encoderDrive"));
        int newLeftTarget;
        int newRightTarget;

        int newLeftTarget2;
        int newRightTarget2;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            telemetry.addData("Psionics_Rover_Path", "Complete");
            telemetry.update();
            DbgLog.msg(String.format("V1DBG: opModeIsActive -- encoderDrive"));

            int currentLeftPosition = robot.leftBackMotor.getCurrentPosition();
            int currentRightPosition = robot.rightBackMotor.getCurrentPosition();

            int currentLeftPosition2 = robot.leftFrontMotor.getCurrentPosition();
            int currentRightPosition2 = robot.rightFrontMotor.getCurrentPosition();


           /* DbgLog.msg(String.format("V1DBG: current postions left and right (front first then back): %10d : %10d : %10d : %10d :",
                    robot.leftFrontMotor.getCurrentPosition(), robot.rightFrontMotor.getCurrentPosition(),
                    robot.leftBackMotor.getCurrentPosition(), robot.rightBackMotor.getCurrentPosition()));*/
idle();
            telemetry.addData("Psionics_Rover_Path", robot.leftFrontMotor.getCurrentPosition());
            telemetry.update();
            idle();
           DbgLog.msg(String.format("V1DBG: current postion front left %10d :", robot.leftFrontMotor.getCurrentPosition()));
            sleep(1000);
            idle();

            telemetry.addData("Psionics_Rover_Path", robot.leftFrontMotor.getCurrentPosition());
            telemetry.update();
            idle();
            DbgLog.msg(String.format("V1DBG: current postion front left %10d :", robot.leftFrontMotor.getCurrentPosition()));
            sleep(1000);
            idle();

            telemetry.addData("Psionics_Rover_Path", robot.rightFrontMotor.getCurrentPosition());
            telemetry.update();
            idle();
            DbgLog.msg(String.format("V1DBG: current postion front right %10d :", robot.rightFrontMotor.getCurrentPosition()));
            sleep(1000);
            idle();

            telemetry.addData("Psionics_Rover_Path", robot.leftBackMotor.getCurrentPosition());
            telemetry.update();
            idle();
            DbgLog.msg(String.format("V1DBG: current postion back left %10d :", robot.leftBackMotor.getCurrentPosition()));
            sleep(1000);
            idle();

            telemetry.addData("Psionics_Rover_Path", robot.rightFrontMotor.getCurrentPosition());
            telemetry.update();
            idle();
            DbgLog.msg(String.format("V1DBG: current postion back right %10d :", robot.rightFrontMotor.getCurrentPosition()));
            idle();

            int lastCurrentLeftPosition = currentLeftPosition;
            int lastCurrentRightPosition = currentRightPosition;

            int lastCurrentLeftPosition2 = currentLeftPosition2;
            int lastCurrentRightPosition2 = currentRightPosition2;

            telemetry.addData("Psionics_Rover_Path", "Complete");
            telemetry.update();
            idle();
            DbgLog.msg(String.format("V1DBG: current postions left and right (front first then back) after assigning: %10d : %10d : %10d : %10d",
                   currentLeftPosition, currentRightPosition, currentLeftPosition2, currentRightPosition2));
            // Determine new target position, and pass to motor controller
            newLeftTarget = currentLeftPosition + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = currentRightPosition + (int) (rightInches * COUNTS_PER_INCH);
            newLeftTarget2 = currentLeftPosition2 + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget2 = currentRightPosition2 + (int) (rightInches * COUNTS_PER_INCH);

            telemetry.addData("Psionics_Rover_Path", "Complete");
            telemetry.update();
            idle();
            DbgLog.msg(String.format("V1DBG: new left and right targets: %10d: %10d", newLeftTarget, newRightTarget));

            robot.leftBackMotor.setTargetPosition(newLeftTarget);
            robot.rightBackMotor.setTargetPosition(newRightTarget);

            robot.leftFrontMotor.setTargetPosition(newLeftTarget);
            robot.rightFrontMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("after run to position", "Complete");
            telemetry.update();
            idle();
            DbgLog.msg(String.format("V1DBG: after run to position"));

            // reset the timeout time and start motion.
            runtime.reset();

            telemetry.addData("after run to position", "Complete");
            telemetry.update();
            idle();
            DbgLog.msg(String.format("V1DBG: past runtime reset"));

            //initializing debug logging variables
            boolean DbgMsglogged = true;
            double TimeLastDbgMsglogged = 0.0;

            //set robot's speed
            robot.leftBackMotor.setPower(speed);
            robot.rightBackMotor.setPower(speed);

            robot.leftFrontMotor.setPower(speed);
            robot.rightFrontMotor.setPower(speed);

            telemetry.addData("V1DBG: after run to position", "Complete");
            telemetry.update();
            idle();
            DbgLog.msg(String.format("V1DBG: speeds of motors assigned"));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftBackMotor.isBusy() && robot.rightBackMotor.isBusy())) {

                telemetry.addData("after run to position", "Complete");
                telemetry.update();
                idle();
                DbgLog.msg(String.format("V1DBG: while opmode active loop"));



                // Allow time for other processes to run.
                idle();
                telemetry.addData("after run to position", "Complete");
                telemetry.update();
                idle();
                DbgLog.msg(String.format("V1DBG: after idle statement"));
            }

            // Stop all motion;
            robot.leftBackMotor.setPower(0);
            robot.rightBackMotor.setPower(0);

            robot.leftFrontMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);

            telemetry.addData("after run to position", "Complete");
            telemetry.update();
            idle();
            DbgLog.msg(String.format("V1DBG: stopped motors"));

            // Turn off RUN_TO_POSITION
        }
    }

    public void followLine1(double w, double x) throws InterruptedException {
        setSensors1(w, x);

        while(1==1)
        {
            telemetry.addData("SensorValue in while", w);
            telemetry.update();
            idle();
            DbgLog.msg(String.format("Psionics_Rover: in while loop"));
            // if (opModeIsActive()) {
            setSensors1(w, x);

            telemetry.addData("SensorValue in while", w);
            telemetry.update();
            idle();
            DbgLog.msg(String.format("SensorValuesInBeg: %5.7f: %5.7f", w, x));

            colorBL1 = returnColor(w);
            colorBR1 = returnColor(x);

            telemetry.addData("color", color);
            telemetry.update();
            idle();
            DbgLog.msg(String.format("Colors in Beg: %5s: %5s", colorBL1, colorBR1));

            if (colorBR1.equals("gray") && colorBL1.equals("gray")) {
                encoderDrive(DRIVE_SPEED, 0, 0, 0.0);
                setSensors1(w, x);

                telemetry.addData("sensorx", x);
                telemetry.update();
                idle();
                DbgLog.msg(String.format("SensorValuesInGray/Gray: %5.7f: %5.7f", w, x));

                colorBL1 = returnColor(w);
                colorBR1 = returnColor(x);

                telemetry.addData("color", color);
                telemetry.update();
                idle();
                DbgLog.msg(String.format("Colors in Beg: %5s: %5s", colorBL1, colorBR1));sleep(2000);
            }
            else if(colorBR1.equals("white") && colorBL1.equals("gray"))
                {
                    encoderDrive(DRIVE_SPEED, -3, 3, 10.0);
                    setSensors1(w, x);

                    telemetry.addData("sensorx", x);
                    telemetry.update();
                    idle();
                    DbgLog.msg(String.format("SensorValuesInWhite/Gray: %5.7f: %5.7f", w, x));

                    colorBL1 = returnColor(w);
                    colorBR1 = returnColor(x);

                    telemetry.addData("color", color);
                    telemetry.update();
                    idle();
                    DbgLog.msg(String.format("Colors in White/Gray: %5s: %5s", colorBL1, colorBR1));
                    sleep(2000);

                }
            else if(colorBR1.equals("gray") && colorBL1.equals("white"))
            {
                encoderDrive(DRIVE_SPEED, 3, -3, 10.0);
                setSensors1(w, x);

                telemetry.addData("sensorx", x);
                telemetry.update();
                idle();
                DbgLog.msg(String.format("SensorValuesInGray/White: %5.7f: %5.7f", w, x));

                colorBL1 = returnColor(w);
                colorBR1 = returnColor(x);

                telemetry.addData("color", color);
                telemetry.update();
                idle();
                DbgLog.msg(String.format("Colors in Gray/White: %5s: %5s", colorBL1, colorBR1));
                sleep(1000);
            }
            else if(colorBR1.equals("white") && colorBL1.equals("white"))
            {
                telemetry.addData("color", color);
                telemetry.update();
                idle();
                DbgLog.msg(String.format("Colors in White/White - Before: %5s: %5s", colorBL1, colorBR1));

                encoderDrive(DRIVE_SPEED, -3, -3, 10.0);

                telemetry.addData("color", color);
                telemetry.update();
                idle();
                DbgLog.msg(String.format("Finished Drive in White/White - Before: %5s: %5s", colorBL1, colorBR1));

                sleep(1000);
                setSensors1(w, x);

                telemetry.addData("sensorx", x);
                telemetry.update();
                idle();
                DbgLog.msg(String.format("SensorValuesInWhite/White: %5.7f: %5.7f", w, x));

                colorBL1 = returnColor(w);
                colorBR1 = returnColor(x);

                telemetry.addData("color", color);
                telemetry.update();
                idle();
                DbgLog.msg(String.format("Colors in White/White - After: %5s: %5s", colorBL1, colorBR1));sleep(2000);
            }

            idle();
            }

        }
    }


