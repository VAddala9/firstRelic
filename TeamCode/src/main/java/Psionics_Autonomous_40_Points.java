import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.LineFollowerBR;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import java.util.ArrayList;

/**
 * Created by geeth on 11/5/2016.
 */
@Autonomous(name="Psionics_Autonomous: Shoot x2 and Cap Ball", group="Psionics")
//@Disabled
public class Psionics_Autonomous_40_Points extends LinearOpMode {
    Psionics_Robot_Hardware robot = new Psionics_Robot_Hardware();
    LineFollowerBR lineFollower = new LineFollowerBR();

    private final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    private final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    private final double WHEEL_CIRCUMFRENCE_FRONT_INCHES = 15.0; // FR:15.0, FL:15.125
    private final double WHEEL_DIAMETER_BACK_INCHES = 4.0; // For figuring circumference
    private final double DISTANCE_ERROR_COMPENSATION = 1.0 * (64.5/60.0); //(57.5 / 60) * (60.5 / 60) * (78 / 63); //orig. (57.5/60) * (60.5/60)
    private final double COUNTS_PER_INCH_FRONT = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (DISTANCE_ERROR_COMPENSATION * WHEEL_CIRCUMFRENCE_FRONT_INCHES);
    private final double COUNTS_PER_INCH_BACK = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (DISTANCE_ERROR_COMPENSATION * WHEEL_DIAMETER_BACK_INCHES * 3.1415);

    private final double FLICKER_ONE_REVOLUTION = COUNTS_PER_MOTOR_REV * 758 / 795;
    private final double FLICKER_THREE_FOURTHS_REVOLUTION = FLICKER_ONE_REVOLUTION * 0.75;

    private final double FRONT_WHEEL_BASE_WIDTH = 14.50;
    private final double FRONT_TURN_RADIUS = FRONT_WHEEL_BASE_WIDTH / 2;
    private final double BACK_WHEEl_BASE_WIDTH = 15.875;
    private final double BACK_TURN_RADIUS = BACK_WHEEl_BASE_WIDTH / 2;
    private final double BACK_TURN_COMPENSATION = BACK_WHEEl_BASE_WIDTH / FRONT_WHEEL_BASE_WIDTH;

    private final double FRONT_QUARTER_TURN = ((3.1415 * FRONT_TURN_RADIUS) / 2.0);
    private final double FRONT_EIGHTH_TURN = ((3.1415 * FRONT_TURN_RADIUS) / 4.0);
    private final double FRONT_SIXTEENTH_TURN = ((3.1415 * FRONT_TURN_RADIUS) / 8.0);
    private final double FRONT_THIRTY_SECOND_TURN = ((3.1415 * FRONT_TURN_RADIUS) / 16.0);
    private final double FRONT_SIXTY_FOURTH_TURN = ((3.1415 * FRONT_TURN_RADIUS) / 32.0);

    private final double RIGHT_SPEED_COMPENSATION = 1.00; //orig. 1.76
    private final double BACK_RIGHT_SPEED_COMPENSATION = 1.00; //orig. 1.55 stays at 1.00
    private final double FRONT_RIGHT_SPEED_COMPENSATION_BACK = 1.00; //orig. 1.50
    private final double FRONT_RIGHT_SPEED_COMPENSATION_FRONT = 1.00; //orig. 2.00


    private final double FLICKER_SPEED = 1.0;
    private final double DRIVE_SPEED = 0.5*1.37;
    private final double DRIVE_SPEED_SHORT = 0.3;
    private final double TURN_SPEED = 0.2;
    /*private final double FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_STRAIGHT_THREE = 0.50;
    private final double FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_LEFT_TURN_THREE = 0.20;
    private final double FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_RIGHT_TURN_THREE = 1.00;
    private final double FRONT_TWO_ENCODER_DISTANCE_COMPENSATION_FRONT_LEFT_SHORT = 26.0 / (31.0 * 1.2);
    private final double FRONT_TWO_ENCODER_DISTANCE_COMPENSATION_FRONT_RIGHT_SHORT = 26.0 / 27.5;*/

    private final double BACK_TWO_ENCODER_DISTANCE_COMPENSATION_FRONT_LEFT = (30.0/32.5); //orig. 26.0/31.0
    private final double BACK_TWO_ENCODER_DISTANCE_COMPENSATION_FRONT_RIGHT = (29.75/30.5) * (60.0/62.5) * (30.0/24.5); // orig. 26.0 / 27.5
    private final double BACK_TWO_ENCODER_LEFT_SPEED_COMPENSATION_FRONT = (30.625/31.875) * 0.75 * (25.5/30.0) * (29.0/30.75) * (31.0/30.0) * (29.0/31.0); //orig. 0.75
    private final double BACK_TWO_ENCODER_RIGHT_SPEED_COMPENSATION_FRONT = (31.875/30.625) * 1.00 * (30.0/23.0) * (31.0/29.0); //orig. 0.75

    private final double FRONT_TWO_ENCODER_SHORT_DISTANCE_COMPENSATION_FRONT_LEFT = (30.0/32.5); //orig. 26.0/31.0
    private final double FRONT_TWO_ENCODER_SHORT_DISTANCE_COMPENSATION_FRONT_RIGHT = (29.75/30.5) * (60.0/62.5) * (30.0/24.5); // orig. 26.0 / 27.5
    private final double FRONT_TWO_ENCODER_SHORT_LEFT_SPEED_COMPENSATION_FRONT = (30.625/31.875) * 0.75 * (25.5/30.0) * (29.0/30.75) * (31.0/30.0) * (29.0/31.0); //orig. 0.75
    private final double FRONT_TWO_ENCODER_SHORT_LEFT_SPEED_COMPENSATION_BACK = 0.5;
    private final double FRONT_TWO_ENCODER_SHORT_RIGHT_SPEED_COMPENSATION_FRONT = (31.875/30.625) * 1.00 * (30.0/23.0) * (31.0/29.0); //orig. 0.75
    private final double FRONT_TWO_ENCODER_SHORT_RIGHT_SPEED_COMPENSATION_FRONT_STRAIGHT_THREE = 1.60;

    private final double BACK_COMPENSATION_THRESHOLD_BACK = 1 * COUNTS_PER_INCH_BACK;
    private final double FRONT_COMPENSATION_THRESHOLD_BACK = 1 * COUNTS_PER_INCH_BACK;
    private final double BACK_COMPENSATION_THRESHOLD_FRONT = 1 * COUNTS_PER_INCH_FRONT;
    private final double FRONT_COMPENSATION_THRESHOLD_FRONT = 1 * COUNTS_PER_INCH_FRONT;

    private final double colorDecision_NotBlue_NotRed = 0.0;
    private final double colorDecision_Blue = 1.0;
    private final double colorDecision_Red = 2.0;
    private final double beaconState_NotSure = 0.0;
    private final double beaconState_BlueOnLeft = 1.0;
    private final double beaconState_RedOnLeft = 2.0;
    private double beaconColorState;

    private final double midBeaconRangeLeftMin = 5.5;
    private final double midBeaconRangeLeftMax = 7.0;
    private final double midBeaconRangeRightMin = 3.5;
    private final double midBeaconRangeRightMax = 5.0;

    private double cumulativeBeaconFindingForwardDistance = 0.0;

    private int lineState;

    private double sensorValueLF;
    private double sensorValueLB;
    private double sensorValueRF;
    private double sensorValueRB;
    private double sensorValueDist1;
    private double sensorValueDist2;

    private double sensorDecisionLF;
    private double sensorDecisionLB;
    private double sensorDecisionRF;
    private double sensorDecisionRB;


    private final double sensor_Threshold_LF = 0.32;
    private final double sensor_Threshold_LB = 0.20;
    private final double sensor_Threshold_RF = 1.20;
    private final double sensor_Threshold_RB = 0.70;


    private final int lineState_NONE = 0;
    private final int lineState_LEFT = 1;
    private final int lineState_LEFT_FRONT = 2;
    private final int lineState_LEFT_BACK = 3;
    private final int lineState_RIGHT = 4;
    private final int lineState_RIGHT_FRONT = 5;
    private final int lineState_RIGHT_BACK = 6;

    double SVL1;
    double SVL2;
    double SVR1;
    double SVR2;

    private enum STOP_CONDITION {
        STOP_ON_ANY_MOTOR,
        STOP_ON_ANY_FRONT,
        STOP_ON_ANY_BACK,
        STOP_ON_ALL_MOTORS,
        STOP_ON_FRONT_MOTORS,
        STOP_ON_BACK_MOTORS
    }

    private double backwardInches = 0;
    private double cumulativeAddlRightRotation_Front = 0.0;
    private double cumulativeAddlRightRotation_Back = 0.0;

    private ElapsedTime runtime = new ElapsedTime();
    private String TAG = "Psionics Autonomous Blue";
    private Scalar averageHsv;
    private boolean doOnce = false;
    private boolean firstBlueButtonPressed = false;
    private int start = 0;
    private int run = 0;
    private int mid = 0;
    private final double FTC_AUTONOMOUS_TIMEOUT = 30.0; // 30 seconds for FTC Autonomous Period
    private final double FRONT_THIRTY_TIMEOUT = 3.0;
    private final double BACK_THIRTY_TIMEOUT = 3.0;
    private final double FRONT_SIX_OR_LESS_TIMEOUT = 3.0;
    private final double BACK_SIX_OR_LESS_TIMEOUT = 3.0;
    private final double QUARTER_TURN_OR_LESS_TIMEOUT = 3.0;
    private final double FLICKER_ONE_REVOLUTION_TIMEOUT = 3.0;

    private final double blueHueNearMin = 100.0; //home : 80.0
    private final double blueHueNearMax = 105.0; //home : 142.5
    private final double blueHueFarMin = 85.0; //home : 80.0
    private final double blueHueFarMax = 125.0; //home : 142.5
    private final double redUpperHueMin = 150.0; //home : 160.0
    private final double redUpperHueMax = 180.0; //home : 180.0
    //private final double redLowerHueMin = 0.0; //home : 0.0
    //private final double redLowerHueMax = 25.0; //home : 25.0
    private final double blueSMin = 250.0;
    private final double blueSMax = 255.0;
    private final double redUpperSMin = 150.0;
    private final double redUpperSMax = 156.0;
    //private final double redLowerSMin = 0.0;
    //private final double redLowerSMax = 25.0;
    private final double blueVMin = 250.0;
    private final double blueVMax = 255.0;
    private final double redUpperVMin = 250.0;
    private final double redUpperVMax = 255.0;

    //private final double redLowerVMin = 0.0;
    //private final double redLowerVMax = 25.0;
    private final double encoderDriveLogIntervalms = 1000.0;
    private final int numColorSearchWindows = FtcRobotControllerActivity.myPsionics_Vision_OpenCV.numColorSearchWindows;
    private final int numColorSearchStartLeft = 5;
    private final int numColorSearchEndRight = 6;
    private final Scalar[] poleSplitColorHSV = FtcRobotControllerActivity.myPsionics_Vision_OpenCV.poleSplitColorHSV;
    private final Scalar[] poleSplitColorRGB = FtcRobotControllerActivity.myPsionics_Vision_OpenCV.poleSplitColorRGB;
    private final Mat[] poleSplit = FtcRobotControllerActivity.myPsionics_Vision_OpenCV.poleSplit;

    ArrayList<Double> beaconColorsDim = new ArrayList<Double>();
    ArrayList<Double> beaconColorsNotDim = new ArrayList<Double>();
    ArrayList<Scalar> beaconColorsDimOverall = new ArrayList<Scalar>();
    ArrayList<Scalar> beaconColorsNotDimOverall = new ArrayList<Scalar>();
    ArrayList<Double> beaconColorDecision = new ArrayList<Double>();

    private final int blueBeaconOneIdealLeft = (int) ((-65 - FRONT_QUARTER_TURN) * COUNTS_PER_INCH_FRONT);
    private final int blueBeaconOneIdealRight = (int) ((-65 + FRONT_QUARTER_TURN) * COUNTS_PER_INCH_FRONT);
    private final int beaconLocationThreshold = 80;
    private int beaconStart = 0;
    private int beaconRun = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.flickerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //do color detect
            //FtcRobotControllerActivity.myPsionics_Vision_OpenCV.setColorDetect(false);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Position 5 Color: ", FtcRobotControllerActivity.myPsionics_Vision_OpenCV.poleSplitColorHSV[5].val[0]);
            //telemetry.addData("Position 6 Color: ", FtcRobotControllerActivity.myPsionics_Vision_OpenCV.poleSplitColorHSV[6].val[0]);
            //telemetry.update();

            /*if (doOnce == false) {
                driveTimedDrive(-0.25,-0.25,-0.25,-0.25,1);
                flickerTimedDrive(1, 0.5);
                intakeTimedDrive(1, 7);
                flickerTimedDrive(1, 0.5);
                driveTimedDrive(-0.25,-0.25,-0.25,-0.25,2);
                driveTimedDrive(0,0,0,0,30);
                doOnce = true;
            }*/
            if (doOnce == false) {
                //move to shooting position
                //shoot first ball
                //run intake
                //shoot second ball
                //FtcRobotControllerActivity.myPsionics_Vision_OpenCV.setColorDetect(false);
                //moveLeft(2);
                //moveRight(2);
                DbgLog.msg("Psionics_Autonomous : Start Test");

                //moveLeft(6);
                //moveRight(6);
                //go to second beacon blue
                //encoderDriveBack(DRIVE_SPEED,-30.0,-30.0,FTC_AUTONOMOUS_TIMEOUT);
                //encoderDriveFront(DRIVE_SPEED,30.0,30.0,FRONT_THIRTY_TIMEOUT);
                //encoderDrive(DRIVE_SPEED,30.0,30.0,30.0,30.0,STOP_CONDITION.STOP_ON_ANY_MOTOR,FTC_AUTONOMOUS_TIMEOUT);


                //encoderDriveBack(DRIVE_SPEED,-26.0,-26.0,BACK_SIX_OR_LESS_TIMEOUT);
                //flickerTimedDrive(1,0.46);

                encoderDriveBack_TwoEnc(DRIVE_SPEED,-42.0,-42.0,STOP_CONDITION.STOP_ON_FRONT_MOTORS,5.0);
                flickerDoFullRotation(FLICKER_ONE_REVOLUTION,FLICKER_SPEED,FLICKER_ONE_REVOLUTION_TIMEOUT);
                intakeTimedDrive(1,1.5);
                flickerDoFullRotation(FLICKER_ONE_REVOLUTION,FLICKER_SPEED,FLICKER_ONE_REVOLUTION_TIMEOUT);
                //flickerTimedDrive(1,0.6);
                encoderDriveBack_TwoEnc(DRIVE_SPEED,-36.0,-36.0,STOP_CONDITION.STOP_ON_FRONT_MOTORS,5.0);
                //encoderDriveBack(TURN_SPEED,FRONT_EIGHTH_TURN,-FRONT_EIGHTH_TURN,BACK_SIX_OR_LESS_TIMEOUT);
                //encoderDriveBack(DRIVE_SPEED,-45.0,-45.0,FRONT_THIRTY_TIMEOUT);
                /*encoderDriveBack(TURN_SPEED,-FRONT_EIGHTH_TURN,FRONT_EIGHTH_TURN,BACK_SIX_OR_LESS_TIMEOUT);
                encoderDriveBack(TURN_SPEED,-FRONT_EIGHTH_TURN,FRONT_EIGHTH_TURN,BACK_SIX_OR_LESS_TIMEOUT);
                encoderDriveBack(TURN_SPEED,-FRONT_EIGHTH_TURN,FRONT_EIGHTH_TURN,BACK_SIX_OR_LESS_TIMEOUT);*/
                //straightenBlue(TURN_SPEED,blueBeaconOneIdealLeft,blueBeaconOneIdealRight,FRONT_SIX_OR_LESS_TIMEOUT);
                //encoderDriveFront(TURN_SPEED,-FRONT_QUARTER_TURN,FRONT_QUARTER_TURN,QUARTER_TURN_OR_LESS_TIMEOUT);
                //straightenBlue(TURN_SPEED,blueBeaconOneIdealLeft,blueBeaconOneIdealRight,FRONT_SIX_OR_LESS_TIMEOUT);
                //encoderDriveFront(DRIVE_SPEED,18.0,18.0,FRONT_SIX_OR_LESS_TIMEOUT);
                //goToFirstBeaconBlueLineFollow();
                //goToFirstBeaconBlue();
                //encoderDriveBack(TURN_SPEED, -FRONT_QUARTER_TURN, FRONT_QUARTER_TURN, BACK_SIX_OR_LESS_TIMEOUT);
                //encoderDriveFront(DRIVE_SPEED,54.0,54.0,FRONT_SIX_OR_LESS_TIMEOUT);*/
                doOnce = true;
            }

            /*telemetry.addData("Color 0 , 1: ", FtcRobotControllerActivity.myPsionics_Vision_OpenCV.poleSplitColorHSV[0].val[0]+ " " +  FtcRobotControllerActivity.myPsionics_Vision_OpenCV.poleSplitColorHSV[1].val[0]);
            telemetry.addData("Color 2 , 3: ", FtcRobotControllerActivity.myPsionics_Vision_OpenCV.poleSplitColorHSV[2].val[0]+ " " +  FtcRobotControllerActivity.myPsionics_Vision_OpenCV.poleSplitColorHSV[3].val[0]);
            telemetry.addData("Color 4 , 5: ", FtcRobotControllerActivity.myPsionics_Vision_OpenCV.poleSplitColorHSV[4].val[0]+ " " +  FtcRobotControllerActivity.myPsionics_Vision_OpenCV.poleSplitColorHSV[5].val[0]);
            telemetry.addData("Color 6 , 7: ", FtcRobotControllerActivity.myPsionics_Vision_OpenCV.poleSplitColorHSV[6].val[0]+ " " +  FtcRobotControllerActivity.myPsionics_Vision_OpenCV.poleSplitColorHSV[7].val[0]);
            telemetry.addData("Color 8 , 9: ", FtcRobotControllerActivity.myPsionics_Vision_OpenCV.poleSplitColorHSV[8].val[0]+ " " +  FtcRobotControllerActivity.myPsionics_Vision_OpenCV.poleSplitColorHSV[9].val[0]);
            telemetry.addData("Color 10,11: ", FtcRobotControllerActivity.myPsionics_Vision_OpenCV.poleSplitColorHSV[10].val[0]+ " " +  FtcRobotControllerActivity.myPsionics_Vision_OpenCV.poleSplitColorHSV[11].val[0]);*/

            /*telemetry.addData("Color 0 , 1: ", poleSplitColorHSV[0] + " " + poleSplitColorHSV[1]);
            telemetry.addData("Color 2 , 3: ", poleSplitColorHSV[2] + " " + poleSplitColorHSV[3]);
            telemetry.addData("Color 4 , 5: ", poleSplitColorHSV[4] + " " + poleSplitColorHSV[5]);
            telemetry.addData("Color 6 , 7: ", poleSplitColorHSV[6] + " " + poleSplitColorHSV[7]);
            telemetry.addData("Color 8 , 9: ", poleSplitColorHSV[8] + " " + poleSplitColorHSV[9]);
            telemetry.addData("Color 10,11: ", poleSplitColorHSV[10] + " " + poleSplitColorHSV[11]);
            telemetry.addData("Blue on Left: ", thereIsBlueOnLeft(blueHueFarMin,blueHueFarMax));
            telemetry.addData("Blue on Right: ", thereIsBlueOnRight(blueHueFarMin,blueHueFarMax));
            telemetry.addData("Blue Run on Left: ", findBlueRunOnLeft(blueHueFarMin,blueHueFarMax));
            telemetry.addData("Blue Run on Right: ", findBlueRunOnRight(blueHueFarMin,blueHueFarMax));
            telemetry.update();*/

            idle();
        }

        //cleanup
        //FtcRobotControllerActivity.myPsionics_Vision_OpenCV.setColorDetect(false);
    }

    public void flickerTimedDrive(double speed, double timeoutS) throws InterruptedException {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the timeout time and start motion.
            runtime.reset();

            //set robot's speed
            robot.flickerMotor.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS)) {
                robot.flickerMotor.setPower(Math.abs(speed));
            }

            // Allow time for other processes to run.
            idle();

            // Stop all motion;
            robot.flickerMotor.setPower(0);
        }
    }

    public void intakeTimedDrive(double speed, double timeoutS) throws InterruptedException {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the timeout time and start motion.
            runtime.reset();

            //set robot's speed
            robot.intakeMotor.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS)) {
                robot.intakeMotor.setPower(Math.abs(speed));
            }

            // Allow time for other processes to run.
            idle();

            // Stop all motion;
            robot.intakeMotor.setPower(0);
        }
    }

    public void driveTimedDrive(double speed, double speed1, double speed2, double speed3, double timeoutS) throws InterruptedException {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the timeout time and start motion.
            runtime.reset();

            //set robot's speed
            robot.leftBackMotor.setPower(speed);
            robot.leftFrontMotor.setPower(speed1);
            robot.rightBackMotor.setPower(speed2);
            robot.rightFrontMotor.setPower(speed3);

            while (opModeIsActive() && (runtime.seconds() < timeoutS)) {
                robot.leftBackMotor.setPower(speed);
                robot.leftFrontMotor.setPower(speed1);
                robot.rightBackMotor.setPower(speed2);
                robot.rightFrontMotor.setPower(speed3);
            }

            // Allow time for other processes to run.
            idle();

            // Stop all motion;
            robot.leftBackMotor.setPower(0);
            robot.leftFrontMotor.setPower(0);
            robot.rightBackMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
        }
    }

    public void flickerDoFullRotation(double position, double speed, double timeoutS) throws InterruptedException {
        int newFlickerTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            int currentFlickerPosition = robot.flickerMotor.getCurrentPosition();

            // Determine new target position, and pass to motor controller
            newFlickerTarget = currentFlickerPosition + (int)(position);

            DbgLog.msg(String.format("Psionics_Autonomous : Flicker starting at %7d ",currentFlickerPosition));

            robot.flickerMotor.setTargetPosition(newFlickerTarget);

            // Turn On RUN_TO_POSITION
            robot.flickerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            //set robot's speed
            robot.flickerMotor.setPower(Math.abs(speed));

            DbgLog.msg(String.format("Psionics_Autonomous : Flicker running to %7d ",newFlickerTarget));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && robot.flickerMotor.isBusy()) {
                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            robot.flickerMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.flickerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(30);

            currentFlickerPosition = robot.flickerMotor.getCurrentPosition();

            DbgLog.msg(String.format("Psionics_Autonomous : Flicker finally at %7d",currentFlickerPosition));

            //sleep(2000);   // optional pause after each move
        }
    }

    public void straightenBlue(double speed, int idealLeftTarget, int idealRightTarget, double timeoutS) throws InterruptedException {
        int currentLeftFrontPosition = robot.leftFrontMotor.getCurrentPosition();
        int currentRightFrontPosition = robot.rightFrontMotor.getCurrentPosition();
        encoderDriveFront(speed,((idealLeftTarget-currentLeftFrontPosition)/COUNTS_PER_INCH_FRONT),
                                ((idealRightTarget-currentRightFrontPosition)/COUNTS_PER_INCH_FRONT),FRONT_SIX_OR_LESS_TIMEOUT);
    }

    public void goToFirstBeaconBlue() throws InterruptedException {
        final int maxDistanceToBeacon = 24;
        final int getToBlueStep = 18;
        final int getToRunBlueForwardStep = 3;
        final int getToRunBlueLeftStep = 3;
        final int getToRunBlueRightStep = 3;
        final int getToRunBlueSize = 5;
        final int getToMidBlueStep = 2;
        int netForwardMotion = 0;
        int mid = 0;
        int newMid = 0;

        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        cumulativeAddlRightRotation_Front = 0.0;
        cumulativeAddlRightRotation_Back = 0.0;

        while (opModeIsActive() && (firstBlueButtonPressed == false)) {
            moveForward(getToBlueStep);
            netForwardMotion += getToBlueStep;
            if (!thereIsBlueOnLeft(blueHueFarMin,blueHueFarMax) && !thereIsBlueOnRight(blueHueFarMin,blueHueFarMax)) {
                DbgLog.msg("Psionics_Autonomous : Moved " + (maxDistanceToBeacon - getToBlueStep) + " but didn't see blue!");
                firstBlueButtonPressed = true;
            }
            DbgLog.msg("Psionics_Autonomous : Moved " + netForwardMotion + " to see blue!");
            if (!firstBlueButtonPressed) {
                do {
                    start = findBlueRun(blueHueFarMin,blueHueFarMax);
                    run = start / numColorSearchWindows;
                    start = start % numColorSearchWindows;
                    DbgLog.msg("Psionics_Autonomous : Run value " + run);
                    DbgLog.msg("Psionics_Autonomous : Start value " + start);
                    if (start < (numColorSearchWindows / 3)) {
                        moveLeft(getToRunBlueLeftStep);
                        moveForward(getToRunBlueForwardStep);
                    } else if (start >= (2 * numColorSearchWindows / 3)) {
                        moveRight(getToRunBlueRightStep);
                        moveForward(getToRunBlueForwardStep);
                    } else {
                        moveForward(getToRunBlueForwardStep);
                    }
                    netForwardMotion += getToRunBlueForwardStep;
                    DbgLog.msg("Psionics_Autonomous : Moved " + netForwardMotion + " to increase and center blue run");
                }
                while ((run < getToRunBlueSize) && (netForwardMotion <= (maxDistanceToBeacon - (2 * getToRunBlueForwardStep))) && !firstBlueButtonPressed);

                do {
                    mid = findBlueRunMid(blueHueFarMin,blueHueFarMax);
                    DbgLog.msg("Psionics_Autonomous : Mid value : " + mid);
                    if (mid < 0) {
                        //firstBlueButtonPressed = true;
                        DbgLog.msg("Psionics_Autonomous : Unexpected mid value"); //look into this
                        moveRight(2*getToMidBlueStep); //added
                        moveForward(getToRunBlueForwardStep); //added
                        netForwardMotion+=getToRunBlueForwardStep; //added
                    }
                    else if (mid > 3) {
                        moveLeft(getToMidBlueStep);
                        DbgLog.msg("Psionics Autonomous - Moved left " + getToMidBlueStep + "inches");
                        newMid = findBlueRunMid(blueHueFarMin,blueHueFarMax);
                        DbgLog.msg("Psionics_Autonomous : New mid : " + newMid);
                        if(newMid < 0) {
                            DbgLog.msg("Psionics_Autonomous : Unexpected mid value - blue not in range");
                            //firstBlueButtonPressed = true;
                            moveRight(2*getToMidBlueStep); // added
                            moveForward(getToRunBlueForwardStep); //added
                            netForwardMotion+=getToRunBlueForwardStep; //added
                        }
                        else if (newMid >= mid) {
                            moveLeft(2 * getToMidBlueStep);
                            DbgLog.msg("Psionics Autonomous - Moved left " + 2 * getToMidBlueStep + "inches");
                        }
                    }
                    else {
                        moveRight(getToMidBlueStep);
                        DbgLog.msg("Psionics Autonomous - Moved right " + getToMidBlueStep + "inches");
                        newMid = findBlueRunMid(blueHueFarMin,blueHueFarMax);
                        if(newMid < 0) {
                            DbgLog.msg("Psionics_Autonomous : Unexpected mid value - blue not in range");
                            firstBlueButtonPressed = true;
                        }
                        else if (newMid <= mid) {
                            moveRight(2 * getToMidBlueStep);
                            DbgLog.msg("Psionics Autonomous - Moved right " + 2 * getToMidBlueStep + "inches");
                        }
                    }
                } while (((mid >= 2) && (mid <= 4)) && (mid > 0) && !firstBlueButtonPressed);
                firstBlueButtonPressed = true;
            }
        }

                    /*while ((mid != 3) && (mid > 0) && !firstBlueButtonPressed) {
                        if (mid > 3) {
                            moveLeft(getToMidBlueStep);
                            DbgLog.msg("Psionics Autonomous - Moved left " + getToMidBlueStep + "inches");
                            newMid = findBlueRunMid();
                            if(newMid < 0) {
                                DbgLog.msg("Psionics_Autonomous : Unexpected mid value - blue not in range");
                                firstBlueButtonPressed = true;
                            }
                            else if (newMid >= mid) {
                                moveLeft(2 * getToMidBlueStep);
                                DbgLog.msg("Psionics Autonomous - Moved left " + 2 * getToMidBlueStep + "inches");
                            }
                        }
                        else {
                            moveRight(getToMidBlueStep);
                            DbgLog.msg("Psionics Autonomous - Moved right " + getToMidBlueStep + "inches");
                            newMid = findBlueRunMid();
                            if(newMid < 0) {
                                DbgLog.msg("Psionics_Autonomous : Unexpected mid value - blue not in range");
                                firstBlueButtonPressed = true;
                            }
                            else if (newMid <= mid) {
                                moveRight(2 * getToMidBlueStep);
                                DbgLog.msg("Psionics Autonomous - Moved right " + 2 * getToMidBlueStep + "inches");
                            }
                        }
                    }
                } while ((!blueSaturationInRange(mid, 1)) && (netForwardMotion <= (maxDistanceToBeacon - 2 * getToMidBlueStep)) && !firstBlueButtonPressed); */

        idle();
    }
    public void goToFirstBeaconBlueLineFollow() throws InterruptedException {
        if(thereIsBlueOnLeft(blueHueFarMin,blueHueFarMax)) {
            lineFollower.lineFollowF(0,0);
        }
        else if(thereIsBlueOnRight(blueHueFarMin,blueHueFarMax)) {
            lineFollower.lineFollowB(0,0);
        }
    }

    public void moveForward(double inches) throws InterruptedException {
        encoderDriveFront(DRIVE_SPEED, inches, inches, FRONT_THIRTY_TIMEOUT);
        idle();
    }

    public void moveBackward(double inches) throws InterruptedException {
        encoderDriveBack(DRIVE_SPEED, -inches, -inches, FRONT_THIRTY_TIMEOUT);
        idle();
    }

    public void moveLeft(double inches) throws InterruptedException {
        double b;
        double lr;
        double f;
        /*encoderDriveBack(DRIVE_SPEED,-FRONT_TURN_RADIUS,-FRONT_TURN_RADIUS,QUARTER_TURN_OR_LESS_TIMEOUT); //back turn radius
        DbgLog.msg("Psionics_Autonomous : Moved backwards " + FRONT_TURN_RADIUS + " inches");
        encoderDriveBack(TURN_SPEED, -FRONT_QUARTER_TURN, FRONT_QUARTER_TURN, QUARTER_TURN_OR_LESS_TIMEOUT); //turn 90 degrees
        DbgLog.msg("Psionics_Autonomous : Turned left " + FRONT_QUARTER_TURN + " inches");
        encoderDriveFront(DRIVE_SPEED, inches, inches, FRONT_SIX_OR_LESS_TIMEOUT); //move forward net left
        DbgLog.msg("Psionics_Autonomous : Went forward " + inches + " inches");
        encoderDriveFront(TURN_SPEED, FRONT_QUARTER_TURN, -FRONT_QUARTER_TURN, QUARTER_TURN_OR_LESS_TIMEOUT); //turn -90 degrees
        DbgLog.msg("Psionics_Autonomous : Turned right " + FRONT_QUARTER_TURN + " inches");
        encoderDriveFront(DRIVE_SPEED, FRONT_TURN_RADIUS, FRONT_TURN_RADIUS, FRONT_SIX_OR_LESS_TIMEOUT); //move forward net left
        DbgLog.msg("Psionics_Autonomous : Went forward " + inches + " inches");*/

        switch((int) inches) {
            case 1: b=2.3; lr=3; f=2.5; break;
            case 2: b=4.6; lr=3; f=5; break;
            case 3: b=5.2; lr=4; f=6; break;
            case 6: b=5.7; lr=6; f=8.3; break;
            default: b=5.7; lr=6; f=8.3; break;
        }

        encoderDriveBack(DRIVE_SPEED,-b,-b,BACK_SIX_OR_LESS_TIMEOUT);
        DbgLog.msg("Psionics_Autonomous : Moved backwards " + b + "inches");
        encoderDriveBack(TURN_SPEED, -lr, lr, BACK_SIX_OR_LESS_TIMEOUT);
        DbgLog.msg("Psionics_Autonomous : Turned left " + lr + " inches");
        encoderDriveFront(DRIVE_SPEED, f, f, FRONT_SIX_OR_LESS_TIMEOUT);
        DbgLog.msg("Psionics_Autonomous : Went forward " + f + "inches");
        encoderDriveFront(TURN_SPEED, lr, -lr, FRONT_SIX_OR_LESS_TIMEOUT);
        DbgLog.msg("Psionics_Autonomous : Turned right" + lr + "inches");

        DbgLog.msg("Psionics_Autonomous : Moved left " + inches + "inches");

        idle();
    }

    public void moveRight(double inches) throws InterruptedException {
        double b;
        double lr;
        double f;

        switch((int) inches) {
            case 1: b=2.3; lr=3; f=2.5; break;
            case 2: b=4.6; lr=3; f=5; break;
            case 3: b=5.2; lr=4; f=6; break;
            case 6: b=5.7; lr=6; f=8.3; break;
            default: b=5.7; lr=6; f=8.3; break;
        }

        encoderDriveBack(DRIVE_SPEED,-b,-b,BACK_SIX_OR_LESS_TIMEOUT);
        DbgLog.msg("Psionics_Autonomous : Moved backwards " + b + "inches");
        encoderDriveBack(TURN_SPEED, lr, -lr, BACK_SIX_OR_LESS_TIMEOUT);
        DbgLog.msg("Psionics_Autonomous : Turned right " + lr + " inches");
        encoderDriveFront(DRIVE_SPEED, f, f, FRONT_SIX_OR_LESS_TIMEOUT);
        DbgLog.msg("Psionics_Autonomous : Went forward " + f + "inches");
        encoderDriveFront(TURN_SPEED, -lr, lr, FRONT_SIX_OR_LESS_TIMEOUT);
        DbgLog.msg("Psionics_Autonomous : Turned left" + lr + "inches");

        DbgLog.msg("Psionics_Autonomous : Moved right " + inches + "inches");

        idle();
    }

    public void turnLeft(double inches) throws InterruptedException {
        encoderDriveFront(TURN_SPEED, -inches, inches, FRONT_SIX_OR_LESS_TIMEOUT);
        idle();
    }

    public void turnRight(double inches) throws InterruptedException {
        encoderDriveFront(TURN_SPEED, inches, -inches, FRONT_SIX_OR_LESS_TIMEOUT);
        idle();
    }

    public boolean thereIsBlueOnRight(double min, double max) throws InterruptedException {
        if (blueHueInRange(0,7,min,max))  {
            DbgLog.msg("Psionics_Autonomous : Found blue on right");

            return true;
        }
        else {
            DbgLog.msg("Psionics_Autonomous : Did not find blue on right");

            return false;
        }
    }

    public boolean thereIsBlueOnLeft(double min, double max) throws InterruptedException {
        if (blueHueInRange(5,7,min,max))  {
            DbgLog.msg("Psionics_Autonomous : Found blue on left");

            return true;
        }
        else {
            DbgLog.msg("Psionics_Autonomous : Did not find blue on left");

            return false;
        }
    }

    public int findBlueRunOnRight(double min, double max) throws InterruptedException {
        int blueRun = 0;
        int startBlueRun = -1;

        waitForNextFrameReady();
        for (int i=0; i<numColorSearchEndRight+1; i++) {
            if ((poleSplitColorHSV[i].val[0] >= min) && (poleSplitColorHSV[i].val[0] <= max)) {
                if (startBlueRun < 0) {startBlueRun = i;}
                blueRun++;
            }
            else if (startBlueRun >= 0) {i = 7;}
        }

        DbgLog.msg("Psionics_Autonomous : Run " + blueRun + " Start " + startBlueRun);
        return ((blueRun*numColorSearchWindows) + startBlueRun);
    }

    public int findBlueRunMidOnRight(double min, double max) throws InterruptedException {
        int blueMid = 0;
        int blueRun = 0;
        int blueStart = -1;

        waitForNextFrameReady();
        for (int i=0; i<numColorSearchEndRight+1; i++) {
            if ((poleSplitColorHSV[i].val[0] >= min) && (poleSplitColorHSV[i].val[0] <= max)) {
                if (blueStart < 0) {blueStart = i;}
                blueRun++;
            }
            else if (blueStart >= 0) {i = 7;}
        }

        blueMid = blueStart + (blueRun/2);
        DbgLog.msg("Psionics_Autonomous : Blue Right Middle: " + blueMid);
        return blueMid;
    }

    public int findBlueRunOnLeft(double min, double max) throws InterruptedException {
        int blueRun = 0;
        int startBlueRun = -1;

        waitForNextFrameReady();
        for (int i=numColorSearchStartLeft; i<numColorSearchWindows; i++) {
            if ((poleSplitColorHSV[i].val[0] >= min) && (poleSplitColorHSV[i].val[0] <= max)) {
                if (startBlueRun < 0) {startBlueRun = i;}
                blueRun++;
            }
            else if (startBlueRun >= 0) {i = numColorSearchWindows;}
        }

        DbgLog.msg("Psionics_Autonomous : Run " + blueRun + " Start " + startBlueRun);
        return ((blueRun*numColorSearchWindows) + startBlueRun);
    }

    public int findBlueRunMidOnLeft(double min, double max) throws InterruptedException {
        int blueMid = 0;
        int blueRun = 0;
        int blueStart = -1;

        waitForNextFrameReady();
        for (int i=numColorSearchStartLeft; i<numColorSearchWindows; i++) {
            if ((poleSplitColorHSV[i].val[0] >= min)&& (poleSplitColorHSV[i].val[0] <= max)) {
                if (blueStart < 0) {blueStart = i;}
                blueRun++;
            }
            else if (blueStart >= 0) {i = numColorSearchWindows;}
        }

        blueMid = blueStart + (blueRun/2);
        DbgLog.msg("Psionics_Autonomous : Blue Left Middle: " + blueMid);
        return blueMid;
    }

    public int findBlueRun(double min, double max) throws InterruptedException {
        int blueRun = 0;
        int startBlueRun = -1;

        waitForNextFrameReady();
        for (int i=0; i<numColorSearchWindows; i++) {
            if ((poleSplitColorHSV[i].val[0] >= min) && (poleSplitColorHSV[i].val[0] <= max)) {
                if (startBlueRun < 0) {startBlueRun = i;}
                blueRun++;
            }
            else if (startBlueRun >= 0) {i = numColorSearchWindows;}
        }

        DbgLog.msg("Psionics_Autonomous : Run " + blueRun + " Start " + startBlueRun);
        return ((blueRun*numColorSearchWindows) + startBlueRun);
    }

    public int findBlueRunMid(double min, double max) throws InterruptedException {
        int blueMid = 0;
        int blueRun = 0;
        int blueStart = -1;

        waitForNextFrameReady();
        for (int i=0; i<numColorSearchWindows; i++) {
            if ((poleSplitColorHSV[i].val[0] >= min)&& (poleSplitColorHSV[i].val[0] <= max)) {
                if (blueStart < 0) {blueStart = i;}
                blueRun++;
            }
            else if (blueStart >= 0) {i = numColorSearchWindows;}
        }

        blueMid = blueStart + (blueRun/2);
        DbgLog.msg("Psionics_Autonomous : Blue Middle: " + blueMid);
        return blueMid;
    }

    public boolean blueHueInRange(int start, int run, double min, double max) throws InterruptedException {
        waitForNextFrameReady();
        for(int i=start; i<(start + run); i++) {
            if((poleSplitColorHSV[i].val[0] >= min) && (poleSplitColorHSV[i].val[0] <= max)) {
                DbgLog.msg("Psionics_Autonomous : Blue in range " + poleSplitColorHSV[i].val[0]);
                return true;
            }
        }
        return false;
    }
    public boolean redHueInRange(int start, int run) throws InterruptedException{
        waitForNextFrameReady();
        for(int i=start; i<(start + run); i++) {
            if(poleSplitColorHSV[i].val[0] >= redUpperHueMin && poleSplitColorHSV[i].val[0] <= redUpperHueMax) {
                return true;
            }
        }
        return false;
    }
    public boolean blueSaturationInRange(int start, int run) throws InterruptedException {
        waitForNextFrameReady();
        for(int i=start; i<(start + run); i++) {
            if(poleSplitColorHSV[i].val[1] >= blueSMin && poleSplitColorHSV[i].val[1] <= blueSMax) {
                return true;
            }
        }
        return false;
    }
    public boolean redSaturationInRange(int start, int run) throws InterruptedException {
        waitForNextFrameReady();
        for(int i=start; i<(start + run); i++) {
            if(poleSplitColorHSV[i].val[1] >= redUpperSMin && poleSplitColorHSV[i].val[1] <= redUpperSMax) {
                return true;
            }
        }
        return false;
    }
    public boolean blueValueInRange(int start, int run) throws InterruptedException{
        waitForNextFrameReady();
        for(int i=start; i<(start + run); i++) {
            if(poleSplitColorHSV[i].val[2] >= blueVMin && poleSplitColorHSV[i].val[2] <= blueVMax) {
                return true;
            }
        }
        return false;
    }
    public boolean redValueInRange(int start, int run) throws InterruptedException {
        waitForNextFrameReady();
        for(int i=start; i<(start + run); i++) {
            if(poleSplitColorHSV[i].val[2] >= redUpperVMin && poleSplitColorHSV[i].val[2] <= redUpperVMax) {
                return true;
            }
        }
        return false;
    }

    public void waitForNextFrameReady() throws InterruptedException {
        DbgLog.msg("Psionics_Autonomous : Waiting for next camera frame");
        FtcRobotControllerActivity.myPsionics_Vision_OpenCV.setNextFrameReady(false);
        while(!(FtcRobotControllerActivity.myPsionics_Vision_OpenCV.getNextFrameReady())) {
            idle();
        }
        DbgLog.msg("Psionics_Autonomous : Next camera frame ready");
    }

    public void encoderDrive(double speed,
                             double leftFrontInches, double rightFrontInches, double leftBackInches, double rightBackInches,
                             STOP_CONDITION stop_condition, double timeoutS) throws InterruptedException {
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

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = currentLeftFrontPosition + (int) (leftFrontInches * COUNTS_PER_INCH_FRONT);
            newRightFrontTarget = currentRightFrontPosition + (int) (rightFrontInches * COUNTS_PER_INCH_FRONT);
            newLeftBackTarget = currentLeftBackPosition + (int) (leftBackInches * COUNTS_PER_INCH_BACK);
            newRightBackTarget = currentRightBackPosition + (int) (rightBackInches * COUNTS_PER_INCH_BACK);

            DbgLog.msg(String.format("Psionics_Autonomous : Starting at %7d :%7d :%7d :%7d", currentLeftFrontPosition, currentRightFrontPosition, currentLeftBackPosition, currentRightBackPosition));

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

            //set robot's speed
            robot.leftFrontMotor.setPower(Math.abs(speed));
            robot.rightFrontMotor.setPower(Math.abs(speed) * RIGHT_SPEED_COMPENSATION);
            robot.leftBackMotor.setPower(Math.abs(speed));
            robot.rightBackMotor.setPower(Math.abs(speed) * RIGHT_SPEED_COMPENSATION);

            DbgLog.msg(String.format("Psionics_Autonomous : Running to %7d :%7d :%7d :%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget));

            // keep looping while we are still active, and there is time left, and both motors are running.
            if (stop_condition == STOP_CONDITION.STOP_ON_BACK_MOTORS) {
                while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                        (robot.leftBackMotor.isBusy() || robot.rightBackMotor.isBusy())) {
                    // Allow time for other processes to run.
                    idle();
                }
            } else if (stop_condition == STOP_CONDITION.STOP_ON_ANY_MOTOR) {
                while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                        (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy() &&
                                robot.leftBackMotor.isBusy() && robot.rightBackMotor.isBusy())) {
                    // Allow time for other processes to run.
                    idle();
                }
            }

            // Stop all motion;
            robot.leftFrontMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
            robot.leftBackMotor.setPower(0);
            robot.rightBackMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(20);

            currentLeftFrontPosition = robot.leftFrontMotor.getCurrentPosition();
            currentRightFrontPosition = robot.rightFrontMotor.getCurrentPosition();
            currentLeftBackPosition = robot.leftBackMotor.getCurrentPosition();
            currentRightBackPosition = robot.rightBackMotor.getCurrentPosition();

            DbgLog.msg(String.format("Psionics_Autonomous : Finally at %7d :%7d :%7d :%7d", currentLeftFrontPosition, currentRightFrontPosition, currentLeftBackPosition, currentRightBackPosition));

            //sleep(2000);   // optional pause after each move
        }
    }

    public void encoderDriveFront(double speed, double leftFrontInches, double rightFrontInches, double timeoutS) throws InterruptedException {
        double leftDirection;
        double rightDirection;
        double directionCompensation;
        double compTargetBack;
        double compTargetFront;
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

            if(leftFrontInches >= 0) leftDirection = 1.0; else leftDirection = -1.0;
            if(rightFrontInches >= 0) rightDirection = 1.0; else rightDirection = -1.0;
            if(leftDirection != rightDirection) directionCompensation = BACK_TURN_COMPENSATION; else directionCompensation = 1.0;

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = currentLeftFrontPosition + (int)(leftFrontInches*COUNTS_PER_INCH_FRONT);
            newRightFrontTarget = currentRightFrontPosition + (int)(rightFrontInches*COUNTS_PER_INCH_FRONT);
            newLeftBackTarget  = currentLeftBackPosition + (int)(leftFrontInches*COUNTS_PER_INCH_BACK*directionCompensation);
            newRightBackTarget  = currentRightBackPosition + (int)(rightFrontInches*COUNTS_PER_INCH_BACK*directionCompensation);

            DbgLog.msg(String.format("Psionics_Autonomous : Starting at %7d :%7d :%7d :%7d",currentLeftFrontPosition,currentRightFrontPosition,currentLeftBackPosition,currentRightBackPosition));

            robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // reset the timeout time and start motion.
            runtime.reset();

            //set robot's speed
            robot.leftFrontMotor.setPower(Math.abs(speed));
            robot.rightFrontMotor.setPower(Math.abs(speed)*FRONT_RIGHT_SPEED_COMPENSATION_FRONT);

            robot.leftBackMotor.setPower(leftDirection* Math.abs(speed));
            robot.rightBackMotor.setPower(rightDirection* Math.abs(speed)*FRONT_RIGHT_SPEED_COMPENSATION_BACK);

            DbgLog.msg(String.format("Psionics_Autonomous : Running to %7d :%7d ",newLeftFrontTarget,newRightFrontTarget));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    ((robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy()))) {
                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            robot.leftBackMotor.setPower(0);
            robot.rightBackMotor.setPower(0);
            robot.leftFrontMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(30);

            currentLeftFrontPosition = robot.leftFrontMotor.getCurrentPosition();
            currentRightFrontPosition = robot.rightFrontMotor.getCurrentPosition();
            currentLeftBackPosition = robot.leftBackMotor.getCurrentPosition();
            currentRightBackPosition = robot.rightBackMotor.getCurrentPosition();

            DbgLog.msg(String.format("Psionics_Autonomous : Finally at %7d :%7d :%7d :%7d",currentLeftFrontPosition,currentRightFrontPosition,currentLeftBackPosition,currentRightBackPosition));

            cumulativeAddlRightRotation_Front += (currentLeftFrontPosition-newLeftFrontTarget)-(currentRightFrontPosition-newRightFrontTarget);
            cumulativeAddlRightRotation_Back += (currentLeftBackPosition-newLeftBackTarget)-(currentRightBackPosition-newRightBackTarget);
            DbgLog.msg(String.format("Psionics_Autonomous : Cumulative additional right rotation front : " + cumulativeAddlRightRotation_Front));
            DbgLog.msg(String.format("Psionics_Autonomous : Cumulative additional right rotation back : " + cumulativeAddlRightRotation_Back));
            DbgLog.msg(String.format("Psionics_Autonomous : Cumulative left front error : " + (newLeftFrontTarget - currentLeftFrontPosition)));
            DbgLog.msg(String.format("Psionics_Autonomous : Cumulative right front error : " + (newRightFrontTarget - currentRightFrontPosition)));
            if ((Math.abs(cumulativeAddlRightRotation_Front) >= FRONT_COMPENSATION_THRESHOLD_FRONT)
                 || (Math.abs(currentLeftFrontPosition-newLeftFrontTarget) >= FRONT_COMPENSATION_THRESHOLD_FRONT)
                 || (Math.abs(currentRightFrontPosition-newRightFrontTarget) >= FRONT_COMPENSATION_THRESHOLD_FRONT)
                 || (Math.abs(cumulativeAddlRightRotation_Back) >= FRONT_COMPENSATION_THRESHOLD_BACK)
                ) {
                compTargetFront = cumulativeAddlRightRotation_Front;
                cumulativeAddlRightRotation_Front = 0.0;
                compTargetBack = cumulativeAddlRightRotation_Back;
                cumulativeAddlRightRotation_Front = 0.0;
                encoderDrive(DRIVE_SPEED,((-compTargetFront+newLeftFrontTarget-currentLeftFrontPosition)/(2*COUNTS_PER_INCH_FRONT)),((compTargetFront+newRightFrontTarget-currentRightFrontPosition)/(2*COUNTS_PER_INCH_FRONT)),(-compTargetBack/(2*COUNTS_PER_INCH_BACK)),(compTargetBack/(2*COUNTS_PER_INCH_BACK)), STOP_CONDITION.STOP_ON_BACK_MOTORS,FRONT_SIX_OR_LESS_TIMEOUT);
                //encoderDriveFront(DRIVE_SPEED,(-compTargetFront/(2*COUNTS_PER_INCH_FRONT)),(compTargetFront/(2*COUNTS_PER_INCH_FRONT)),FTC_AUTONOMOUS_TIMEOUT);
            }

            //sleep(2000);   // optional pause after each move
        }
    }

    public void encoderDriveBack(double speed, double leftFrontInches, double rightFrontInches, double timeoutS) throws InterruptedException {
        double leftDirection;
        double rightDirection;
        double directionCompensation;
        double compTargetFront;
        double compTargetBack;
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

            if(leftFrontInches >= 0) leftDirection = 1.0; else leftDirection = -1.0;
            if(rightFrontInches >= 0) rightDirection = 1.0; else rightDirection = -1.0;
            if(leftDirection != rightDirection) directionCompensation = BACK_TURN_COMPENSATION; else directionCompensation = 1.0;

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = currentLeftFrontPosition + (int)(leftFrontInches*COUNTS_PER_INCH_FRONT);
            newRightFrontTarget = currentRightFrontPosition + (int)(rightFrontInches*COUNTS_PER_INCH_FRONT);
            newLeftBackTarget  = currentLeftBackPosition + (int)(leftFrontInches*COUNTS_PER_INCH_BACK*directionCompensation);
            newRightBackTarget  = currentRightBackPosition + (int)(rightFrontInches*COUNTS_PER_INCH_BACK*directionCompensation);

            DbgLog.msg(String.format("Psionics_Autonomous : Starting at %7d :%7d :%7d :%7d",currentLeftFrontPosition,currentRightFrontPosition,currentLeftBackPosition,currentRightBackPosition));

            robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // reset the timeout time and start motion.
            runtime.reset();

            //set robot's speed
            robot.leftFrontMotor.setPower(Math.abs(speed));
            robot.rightFrontMotor.setPower(Math.abs(speed)*BACK_RIGHT_SPEED_COMPENSATION);
            robot.leftBackMotor.setPower(leftDirection* Math.abs(speed));
            robot.rightBackMotor.setPower(rightDirection* Math.abs(speed)*BACK_RIGHT_SPEED_COMPENSATION);

            DbgLog.msg(String.format("Psionics_Autonomous : Running to %7d :%7d ",newLeftFrontTarget,newRightFrontTarget));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    ((robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy()))) {
                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            robot.leftBackMotor.setPower(0);
            robot.rightBackMotor.setPower(0);
            robot.leftFrontMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);

           // Turn off RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(30);

            currentLeftFrontPosition = robot.leftFrontMotor.getCurrentPosition();
            currentRightFrontPosition = robot.rightFrontMotor.getCurrentPosition();
            currentLeftBackPosition = robot.leftBackMotor.getCurrentPosition();
            currentRightBackPosition = robot.rightBackMotor.getCurrentPosition();

            DbgLog.msg(String.format("Psionics_Autonomous : Finally at %7d :%7d :%7d :%7d",currentLeftFrontPosition,currentRightFrontPosition,currentLeftBackPosition,currentRightBackPosition));

            cumulativeAddlRightRotation_Front += (currentLeftFrontPosition-newLeftFrontTarget)-(currentRightFrontPosition-newRightFrontTarget);
            cumulativeAddlRightRotation_Back += (currentLeftBackPosition-newLeftBackTarget)-(currentRightBackPosition-newRightBackTarget);
            DbgLog.msg(String.format("Psionics_Autonomous : Cumulative additional right rotation front : " + cumulativeAddlRightRotation_Front));
            DbgLog.msg(String.format("Psionics_Autonomous : Cumulative additional right rotation back : " + cumulativeAddlRightRotation_Back));
            if ((Math.abs(cumulativeAddlRightRotation_Front) >= BACK_COMPENSATION_THRESHOLD_FRONT) || (Math.abs(cumulativeAddlRightRotation_Back) >= BACK_COMPENSATION_THRESHOLD_BACK)) {
                compTargetFront = cumulativeAddlRightRotation_Front;
                cumulativeAddlRightRotation_Front = 0.0;
                compTargetBack = cumulativeAddlRightRotation_Back;
                cumulativeAddlRightRotation_Front = 0.0;
                encoderDrive(DRIVE_SPEED,(-compTargetFront/(2*COUNTS_PER_INCH_FRONT)),(compTargetFront/(2*COUNTS_PER_INCH_FRONT)),(-compTargetBack/(2*COUNTS_PER_INCH_BACK)),(compTargetBack/(2*COUNTS_PER_INCH_BACK)), STOP_CONDITION.STOP_ON_BACK_MOTORS,BACK_SIX_OR_LESS_TIMEOUT);
            }

            //sleep(2000);   // optional pause after each move
        }
    }
    public void encoderDriveBack_TwoEnc(double speed, double leftFrontInches, double rightFrontInches, STOP_CONDITION stop_condition, double timeoutS) throws InterruptedException {
        double leftDirection;
        double rightDirection;
        double compTargetFront;
        double compTargetBack;
        int newLeftFrontTarget;
        int newRightFrontTarget;
        boolean leftFrontMotorBusy;
        boolean rightFrontMotorBusy;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            int currentLeftFrontPosition = robot.leftFrontMotor.getCurrentPosition();
            int currentRightFrontPosition = robot.rightFrontMotor.getCurrentPosition();
            int currentLeftBackPosition = robot.leftBackMotor.getCurrentPosition();
            int currentRightBackPosition = robot.rightBackMotor.getCurrentPosition();

            if (leftFrontInches >= 0) leftDirection = 1.0;
            else leftDirection = -1.0;
            if (rightFrontInches >= 0) rightDirection = 1.0;
            else rightDirection = -1.0;

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = currentLeftFrontPosition + (int) (leftFrontInches * BACK_TWO_ENCODER_DISTANCE_COMPENSATION_FRONT_LEFT * COUNTS_PER_INCH_FRONT);
            newRightFrontTarget = currentRightFrontPosition + (int) (rightFrontInches * BACK_TWO_ENCODER_DISTANCE_COMPENSATION_FRONT_RIGHT * COUNTS_PER_INCH_FRONT);

            DbgLog.msg(String.format("Psionics_Autonomous : Starting at %7d :%7d :%7d :%7d", currentLeftFrontPosition, currentRightFrontPosition, currentLeftBackPosition, currentRightBackPosition));

            robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // reset the timeout time and start motion.
            runtime.reset();

            //set robot's speed
            robot.leftFrontMotor.setPower(Math.abs(speed) * BACK_TWO_ENCODER_LEFT_SPEED_COMPENSATION_FRONT);
            robot.rightFrontMotor.setPower(Math.abs(speed) * BACK_TWO_ENCODER_RIGHT_SPEED_COMPENSATION_FRONT);
            robot.leftBackMotor.setPower(leftDirection * Math.abs(speed));
            robot.rightBackMotor.setPower(rightDirection * Math.abs(speed));

            DbgLog.msg(String.format("Psionics_Autonomous : Running to %7d :%7d ", newLeftFrontTarget, newRightFrontTarget));

            if(stop_condition == STOP_CONDITION.STOP_ON_ANY_FRONT) {
                // keep looping while we are still active, and there is time left, and both front motors are running.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        ((robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy()))) {
                    // Allow time for other processes to run.
                    idle();
                }
            }
            else if(stop_condition == STOP_CONDITION.STOP_ON_FRONT_MOTORS) {
                // keep looping while we are still active, and there is time left, and either front motors are running.
                do {
                    leftFrontMotorBusy = robot.leftFrontMotor.isBusy();
                    rightFrontMotorBusy = robot.rightFrontMotor.isBusy();
                    if (!leftFrontMotorBusy) {
                        robot.leftBackMotor.setPower(0);
                        robot.leftFrontMotor.setPower(0);
                    } else if (!rightFrontMotorBusy) {
                        robot.rightBackMotor.setPower(0);
                        robot.rightFrontMotor.setPower(0);
                    }
                    // Allow time for other processes to run.
                    idle();
                }
                while (opModeIsActive() && (runtime.seconds() < timeoutS) && (leftFrontMotorBusy || rightFrontMotorBusy));
            }

            // Stop all motion;
            robot.rightBackMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
            robot.leftBackMotor.setPower(0);
            robot.leftFrontMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(30);

            currentLeftFrontPosition = robot.leftFrontMotor.getCurrentPosition();
            currentRightFrontPosition = robot.rightFrontMotor.getCurrentPosition();
            currentLeftBackPosition = robot.leftBackMotor.getCurrentPosition();
            currentRightBackPosition = robot.rightBackMotor.getCurrentPosition();

            DbgLog.msg(String.format("Psionics_Autonomous : Finally at %7d :%7d :%7d :%7d", currentLeftFrontPosition, currentRightFrontPosition, currentLeftBackPosition, currentRightBackPosition));

            /*cumulativeAddlRightRotation_Front += (currentLeftFrontPosition - newLeftFrontTarget) - (currentRightFrontPosition - newRightFrontTarget);
            DbgLog.msg(String.format("Psionics_Autonomous : Cumulative additional right rotation front : " + cumulativeAddlRightRotation_Front));

            if ((Math.abs(cumulativeAddlRightRotation_Front) >= BACK_COMPENSATION_THRESHOLD_FRONT)) {
                compTargetFront = cumulativeAddlRightRotation_Front;
                cumulativeAddlRightRotation_Front = 0.0;
                encoderDriveBack_TwoEnc(TURN_SPEED, (-compTargetFront / (2 * COUNTS_PER_INCH_FRONT)), (compTargetFront / (2 * COUNTS_PER_INCH_FRONT)), 2.0);
            }*/
        }
    }

    public void encoderDriveFront_TwoEnc_Short(double speed, double leftFrontInches, double rightFrontInches, double rightSpeedCompensation, STOP_CONDITION stop_condition, double timeoutS) throws InterruptedException {
        double leftDirection;
        double rightDirection;
        double compTargetFront;
        double compTargetBack;
        int newLeftFrontTarget;
        int newRightFrontTarget;
        boolean leftFrontMotorBusy;
        boolean rightFrontMotorBusy;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            int currentLeftFrontPosition = robot.leftFrontMotor.getCurrentPosition();
            int currentRightFrontPosition = robot.rightFrontMotor.getCurrentPosition();
            int currentLeftBackPosition = robot.leftBackMotor.getCurrentPosition();
            int currentRightBackPosition = robot.rightBackMotor.getCurrentPosition();

            if (leftFrontInches >= 0) leftDirection = 1.0;
            else leftDirection = -1.0;
            if (rightFrontInches >= 0) rightDirection = 1.0;
            else rightDirection = -1.0;

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = currentLeftFrontPosition + (int) (leftFrontInches * FRONT_TWO_ENCODER_SHORT_DISTANCE_COMPENSATION_FRONT_LEFT * COUNTS_PER_INCH_FRONT);
            newRightFrontTarget = currentRightFrontPosition + (int) (rightFrontInches * FRONT_TWO_ENCODER_SHORT_DISTANCE_COMPENSATION_FRONT_RIGHT * COUNTS_PER_INCH_FRONT);

            DbgLog.msg(String.format("Psionics_Autonomous : Starting at %7d :%7d :%7d :%7d", currentLeftFrontPosition, currentRightFrontPosition, currentLeftBackPosition, currentRightBackPosition));

            robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // reset the timeout time and start motion.
            runtime.reset();

            //set robot's speed
            robot.leftFrontMotor.setPower(Math.abs(speed) * FRONT_TWO_ENCODER_SHORT_LEFT_SPEED_COMPENSATION_FRONT);
            robot.rightFrontMotor.setPower(Math.abs(speed) * FRONT_TWO_ENCODER_SHORT_RIGHT_SPEED_COMPENSATION_FRONT * rightSpeedCompensation);
            robot.leftBackMotor.setPower(leftDirection * Math.abs(speed) * FRONT_TWO_ENCODER_SHORT_LEFT_SPEED_COMPENSATION_BACK);
            robot.rightBackMotor.setPower(rightDirection * Math.abs(speed) * rightSpeedCompensation);

            DbgLog.msg(String.format("Psionics_Autonomous : Running to %7d :%7d ", newLeftFrontTarget, newRightFrontTarget));

            if(stop_condition == STOP_CONDITION.STOP_ON_ANY_FRONT) {
                // keep looping while we are still active, and there is time left, and both front motors are running.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        ((robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy()))) {
                    // Allow time for other processes to run.
                    idle();
                }
            }
            else if(stop_condition == STOP_CONDITION.STOP_ON_FRONT_MOTORS) {
                // keep looping while we are still active, and there is time left, and either front motors are running.
                do {
                    leftFrontMotorBusy = robot.leftFrontMotor.isBusy();
                    rightFrontMotorBusy = robot.rightFrontMotor.isBusy();
                    if (!leftFrontMotorBusy) {
                        robot.leftBackMotor.setPower(0);
                        robot.leftFrontMotor.setPower(0);
                    } else if (!rightFrontMotorBusy) {
                        robot.rightBackMotor.setPower(0);
                        robot.rightFrontMotor.setPower(0);
                    }
                    // Allow time for other processes to run.
                    idle();
                }
                while (opModeIsActive() && (runtime.seconds() < timeoutS) && (leftFrontMotorBusy || rightFrontMotorBusy));
            }

            // Stop all motion;
            robot.leftBackMotor.setPower(0);
            robot.leftFrontMotor.setPower(0);
            robot.rightBackMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(30);

            currentLeftFrontPosition = robot.leftFrontMotor.getCurrentPosition();
            currentRightFrontPosition = robot.rightFrontMotor.getCurrentPosition();
            currentLeftBackPosition = robot.leftBackMotor.getCurrentPosition();
            currentRightBackPosition = robot.rightBackMotor.getCurrentPosition();

            DbgLog.msg(String.format("Psionics_Autonomous : Finally at %7d :%7d :%7d :%7d", currentLeftFrontPosition, currentRightFrontPosition, currentLeftBackPosition, currentRightBackPosition));

            /*cumulativeAddlRightRotation_Front += (currentLeftFrontPosition - newLeftFrontTarget) - (currentRightFrontPosition - newRightFrontTarget);
            DbgLog.msg(String.format("Psionics_Autonomous : Cumulative additional right rotation front : " + cumulativeAddlRightRotation_Front));

            if ((Math.abs(cumulativeAddlRightRotation_Front) >= BACK_COMPENSATION_THRESHOLD_FRONT)) {
                compTargetFront = cumulativeAddlRightRotation_Front;
                cumulativeAddlRightRotation_Front = 0.0;
                encoderDriveBack_TwoEnc(TURN_SPEED, (-compTargetFront / (2 * COUNTS_PER_INCH_FRONT)), (compTargetFront / (2 * COUNTS_PER_INCH_FRONT)), 2.0);
            }*/
        }
    }
}
