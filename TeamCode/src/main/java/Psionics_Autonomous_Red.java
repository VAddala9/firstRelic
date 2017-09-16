import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.LineFollowerBR;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import java.util.ArrayList;

/**
 * Created by geeth on 11/5/2016.
 */
@Autonomous(name="Psionics_Autonomous_Red", group="Psionics")
//@Disabled
public class Psionics_Autonomous_Red extends LinearOpMode {
    Psionics_Robot_Hardware_Tournament robot = new Psionics_Robot_Hardware_Tournament();
    LineFollowerBR lineFollower = new LineFollowerBR();



    private final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    private final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    private final double WHEEL_CIRCUMFRENCE_FRONT_INCHES = 15.0; // FR:15.0, FL:15.125
    private final double WHEEL_DIAMETER_BACK_INCHES = 4.0; // For figuring circumference
    private final double DISTANCE_ERROR_COMPENSATION = (57.5 / 60) * (60.5 / 60) * (78 / 63); //orig. (57.5/60) * (60.5/60)
    private final double COUNTS_PER_INCH_FRONT = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (DISTANCE_ERROR_COMPENSATION * WHEEL_CIRCUMFRENCE_FRONT_INCHES);
    private final double COUNTS_PER_INCH_BACK = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (DISTANCE_ERROR_COMPENSATION * WHEEL_DIAMETER_BACK_INCHES * 3.1415);

    private final double FLICKER_ONE_REVOLUTION = COUNTS_PER_MOTOR_REV * 758 / 795;
    private final double FLICKER_THREE_FOURTHS_REVOLUTION = FLICKER_ONE_REVOLUTION * 0.75;

    private final double FRONT_WHEEL_BASE_WIDTH = 14.625;
    private final double FRONT_TURN_RADIUS = FRONT_WHEEL_BASE_WIDTH / 2;
    private final double BACK_WHEEl_BASE_WIDTH = 15.875;
    private final double BACK_TURN_RADIUS = BACK_WHEEl_BASE_WIDTH / 2;
    private final double BACK_TURN_COMPENSATION = BACK_WHEEl_BASE_WIDTH / FRONT_WHEEL_BASE_WIDTH;

    private final double FRONT_QUARTER_TURN = ((3.1415 * FRONT_TURN_RADIUS) / 2.0);
    private final double FRONT_EIGHTH_TURN = ((3.1415 * FRONT_TURN_RADIUS) / 4.0);
    private final double FRONT_SIXTEENTH_TURN = ((3.1415 * FRONT_TURN_RADIUS) / 8.0);
    private final double FRONT_THIRTY_SECOND_TURN = ((3.1415 * FRONT_TURN_RADIUS) / 16.0);
    private final double FRONT_SIXTY_FOURTH_TURN = ((3.1415 * FRONT_TURN_RADIUS) / 32.0);

    private final double RIGHT_SPEED_COMPENSATION = 1.76;
    private final double BACK_RIGHT_SPEED_COMPENSATION = 1.00; //orig. 1.55
    private final double BACK_LEFT_SPEED_COMPENSATION_FRONT = 0.75;
    private final double FRONT_RIGHT_SPEED_COMPENSATION_BACK = 1.50;
    private final double FRONT_RIGHT_SPEED_COMPENSATION_FRONT = 2.00;


    private final double FLICKER_SPEED = 1.0;
    private final double DRIVE_SPEED = 0.5;
    private final double TURN_SPEED = 0.2;

    private final double FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_STRAIGHT_THREE = 0.50;
    private final double FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_LEFT_TURN_THREE = 0.20;
    private final double FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_RIGHT_TURN_THREE = 1.00;
    private final double FRONT_TWO_ENCODER_DISTANCE_COMPENSATION_FRONT_LEFT_SHORT = 26.0 / (31.0 * 1.2);
    private final double FRONT_TWO_ENCODER_DISTANCE_COMPENSATION_FRONT_RIGHT_SHORT = 26.0 / 27.5;

    private final double BACK_TWO_ENCODER_DISTANCE_COMPENSATION_FRONT_LEFT = 26.0 / 31.0;
    private final double BACK_TWO_ENCODER_DISTANCE_COMPENSATION_FRONT_RIGHT = 26.0 / 27.5;

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
                DbgLog.msg("Psionics_Autonomous : Start Test");

                FtcRobotControllerActivity.myPsionics_Vision_OpenCV.setOpenCVProcessing(false);
                FtcRobotControllerActivity.myPsionics_Vision_OpenCV.setLocateBeacon(false);
                FtcRobotControllerActivity.myPsionics_Vision_OpenCV.setFindBeaconColor(false);

                //encoderDriveBack_TwoEnc(DRIVE_SPEED,-29.0,-29.0,2.0);

                /*flickerDoFullRotation(FLICKER_ONE_REVOLUTION,FLICKER_SPEED,FLICKER_ONE_REVOLUTION_TIMEOUT);
                intakeTimedDrive(1,1.5);
                flickerDoFullRotation(FLICKER_ONE_REVOLUTION,FLICKER_SPEED,FLICKER_ONE_REVOLUTION_TIMEOUT);*/

                /*encoderDriveBack_TwoEnc(TURN_SPEED,1.46*1.25*(-FRONT_EIGHTH_TURN),1.46*1.25*(FRONT_EIGHTH_TURN),2.0);
                encoderDriveBack_TwoEnc(DRIVE_SPEED,-29.0,-29.0,2.5);
                encoderDriveBack_TwoEnc(DRIVE_SPEED,-30.0,-30.0,2.5);
                encoderDriveBack_TwoEnc(TURN_SPEED,1.20*(FRONT_EIGHTH_TURN),1.20*(-FRONT_EIGHTH_TURN),2.0);
                encoderDriveBack_TwoEnc(DRIVE_SPEED,-6.00,-6.00,2.0);
                encoderDriveBack_TwoEnc(TURN_SPEED,3.25*(FRONT_EIGHTH_TURN),3.25*(-FRONT_EIGHTH_TURN),3.0);*/

                positionRobotForBeacon();

                //sleep(100);

                //robot.buttonPusher.setPosition(Psionics_Robot_Hardware.BUTTONPUSHER_MAX_RANGE);

                //lineFollowF(SVL2, SVR2);
                //lineFollowB(SVL1, SVR1);

                //straightenBlue(TURN_SPEED,blueBeaconOneIdealLeft,blueBeaconOneIdealRight,FRONT_SIX_OR_LESS_TIMEOUT);

                //encoderDriveBack(DRIVE_SPEED,20.0,20.0,BACK_SIX_OR_LESS_TIMEOUT);

                //goToFirstBeaconBlueLineFollow();
                //goToFirstBeaconBlue();

                doOnce = true;
            }

            telemetry.addData("Beacon Start: " + beaconStart, "Beacon Run: " + beaconRun);
            telemetry.addData("Array List Dim : ", beaconColorsDim.toString());
            telemetry.addData("Array List Not Dim : ", beaconColorsNotDim.toString());
            telemetry.addData("Array List Beacon Color Decision : ", beaconColorDecision.toString());
            telemetry.addData("Beacon color state : ", beaconColorState);
            telemetry.update();

            //getBottomSensorStatus();

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

    public void positionRobotForBeacon() throws InterruptedException {
        do {
            findBeacon();
            beaconColorState = determineBeaconColorState();
            telemetry.addData("Beacon state : ", beaconColorState);
            telemetry.addData("Line state : ", lineState);
            telemetry.addData("Beacon Start : ", beaconStart);
            telemetry.addData("Beacon Run : ", beaconRun);
            telemetry.addData("Going", "Determine beacon state");
            telemetry.update();
            sleep(3000);
            if(beaconStart == 0) {
                encoderDriveFront_TwoEnc_Short(DRIVE_SPEED,-3.0,3.0,FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_LEFT_TURN_THREE,0.5);
                encoderDriveFront_TwoEnc_Short(DRIVE_SPEED,-3.0,-3.0,FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_STRAIGHT_THREE,0.5);
                encoderDriveFront_TwoEnc_Short(DRIVE_SPEED,3.0,-3.0,FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_RIGHT_TURN_THREE,0.5);
            }
            else if(((beaconStart + beaconRun) == (numColorSearchWindows - 1))) {
                encoderDriveFront_TwoEnc_Short(DRIVE_SPEED,3.0,-3.0,FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_RIGHT_TURN_THREE,0.5);
                encoderDriveFront_TwoEnc_Short(DRIVE_SPEED,-3.0,-3.0,FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_STRAIGHT_THREE,0.5);
                encoderDriveFront_TwoEnc_Short(DRIVE_SPEED,-3.0,3.0,FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_LEFT_TURN_THREE,0.5);
            }
            else if(beaconRun < 4) {
                encoderDriveFront_TwoEnc_Short(DRIVE_SPEED, 3.0, 3.0, FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_STRAIGHT_THREE, 0.5);
            }
            else if(beaconRun > 8) {
                encoderDriveFront_TwoEnc_Short(DRIVE_SPEED,-3.0,-3.0,FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_STRAIGHT_THREE,0.5);
            }
            else {
                encoderDriveFront_TwoEnc_Short(DRIVE_SPEED, 1.0, 1.0, FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_STRAIGHT_THREE, 0.5);
            }
        } while ((beaconColorState == beaconState_NotSure) || (beaconRun < 4) || (beaconRun > 8) || (beaconStart == 0) || ((beaconStart + beaconRun) == (numColorSearchWindows - 1)));

        if (beaconColorState == beaconState_RedOnLeft) {
            telemetry.addData("Beacon state : ", beaconColorState);
            telemetry.addData("Line state : ", lineState);
            telemetry.addData("Beacon Start : ", beaconStart);
            telemetry.addData("Beacon Run : ", beaconRun);
            telemetry.addData("Going", "Blue left");
            telemetry.update();
            sleep(1000);
            triggerLeftButton();
        }
        else {
            telemetry.addData("Beacon state : ", beaconColorState);
            telemetry.addData("Line state : ", lineState);
            telemetry.addData("Beacon Start : ", beaconStart);
            telemetry.addData("Beacon Run : ", beaconRun);
            telemetry.addData("Going", "Blue right");
            telemetry.update();
            sleep(1000);
            triggerRightButton();
        }
    }

    public void findBeacon() throws InterruptedException { //assumes that there is only one run of brightness
        beaconStart = -1;
        beaconRun = 0;
        FtcRobotControllerActivity.myPsionics_Vision_OpenCV.setLocateBeacon(true);
        FtcRobotControllerActivity.myPsionics_Vision_OpenCV.setFindBeaconColor(false);
        waitForNextFrameReady();
        FtcRobotControllerActivity.myPsionics_Vision_OpenCV.setOpenCVProcessing(true);
        waitForNextFrameReady();
        for (int i = 0; i < poleSplitColorHSV.length; i++) {
            if (poleSplitColorHSV[i].val[2] >= beaconLocationThreshold) {
                if (beaconRun == 0) {
                    beaconStart = i;
                    /*if(i!=0) {
                        beaconColorsDim.add(poleSplitColorHSV[i-1].val[0]);
                    }*/
                }
                beaconColorsDim.add(poleSplitColorHSV[i].val[0]);
                beaconColorsDimOverall.add(poleSplitColorHSV[i]);
                beaconRun++;
            }
        }
        /*if(((beaconStart+beaconRun) < numColorSearchWindows) && (beaconRun > 0)) {
            beaconColorsDim.add(poleSplitColorHSV[(beaconStart+beaconRun)].val[0]);
        }*/

        if (beaconRun > 0) {
            FtcRobotControllerActivity.myPsionics_Vision_OpenCV.setLocateBeacon(false);
            FtcRobotControllerActivity.myPsionics_Vision_OpenCV.setFindBeaconColor(true);
            waitForNextFrameReady();
            FtcRobotControllerActivity.myPsionics_Vision_OpenCV.setOpenCVProcessing(true);
            waitForNextFrameReady();
            /*if (beaconStart > 0) {
                beaconColorsNotDim.add(poleSplitColorHSV[beaconStart - 1].val[0]);
            }*/
            for (int i = beaconStart; i < (beaconStart + beaconRun); i++) {
                beaconColorsNotDim.add(poleSplitColorHSV[i].val[0]);
                beaconColorsNotDimOverall.add(poleSplitColorHSV[i]);
            }
            /*if (((beaconStart + beaconRun) < numColorSearchWindows) && (beaconRun > 0)) {
                beaconColorsNotDim.add(poleSplitColorHSV[beaconStart + beaconRun].val[0]);
            }*/
        }

        /*telemetry.addData("Beacon Start: " + beaconStart , "Beacon Run: " + beaconRun);
        telemetry.addData("Array List Dim : ", beaconColorsDim.toString());
        telemetry.addData("Array List Not Dim : ", beaconColorsNotDim.toString());
        telemetry.update();*/
        DbgLog.msg("Psionics_Autonomous : Beacon Start: " + beaconStart);
        DbgLog.msg("Psionics_Autonomous : Beacon Run: " + beaconRun);
    }

    public double determineBeaconColorState() {
        double redMax = -1;
        double redMin = -1;
        double blueMax = -1;
        double blueMin = -1;
        double decision;

        if (beaconColorsDim.size() > 0) {
            for (int i = 0; i < beaconColorsDim.size(); i++) {
                decision = beaconColorDecision(i);
                beaconColorDecision.add(decision);
                switch ((int) decision) {
                    case 1:
                        if (blueMin == -1) {
                            blueMin = i;
                        }
                        blueMax = i;
                        break;
                    case 2:
                        if (redMin == -1) {
                            redMin = i;
                        }
                        redMax = i;
                        break;
                    default:
                        break;
                }
            }
            if (((redMin + blueMin) == -2.0)) {
                return beaconState_NotSure;
            } else if (blueMin < 0) {
                if (((redMin + redMax)) >= ((beaconColorDecision.size()))) {
                    return beaconState_RedOnLeft;
                } else {
                    return beaconState_BlueOnLeft;
                }
            } else if (redMin < 0) {
                if ((blueMin + blueMax) >= ((beaconColorDecision.size()))) {
                    return beaconState_BlueOnLeft;
                } else {
                    return beaconState_RedOnLeft;
                }
            } else {
                if (blueMax < redMin) {
                    return beaconState_RedOnLeft;
                } else if (redMax < blueMin) {
                    return beaconState_BlueOnLeft;
                } else {
                    return beaconState_NotSure;
                }
            }
        } else {
            return beaconState_NotSure;
        }
    }


    public double beaconColorDecision(int index) {
        double dimValue = colorDecision_NotBlue_NotRed;
        double noDimValue = colorDecision_NotBlue_NotRed;

        if (beaconColorsDim.get(index) <= blueHueFarMin && beaconColorsDim.get(index) >= blueHueFarMax && beaconColorsDim.get(index) <= redUpperHueMin && beaconColorsDim.get(index) >= redUpperHueMax) {
            dimValue = colorDecision_NotBlue_NotRed;
        } else if (beaconColorsDim.get(index) >= blueHueFarMin && beaconColorsDim.get(index) <= blueHueFarMax) {
            dimValue = colorDecision_Blue;
        } else if (beaconColorsDim.get(index) >= redUpperHueMin && beaconColorsDim.get(index) <= redUpperHueMax) {
            dimValue = colorDecision_Red;
        }

        if (beaconColorsNotDim.get(index) <= blueHueFarMin && beaconColorsNotDim.get(index) >= blueHueFarMax && beaconColorsNotDim.get(index) <= redUpperHueMin && beaconColorsNotDim.get(index) >= redUpperHueMax) {
            noDimValue = colorDecision_NotBlue_NotRed;
        } else if (beaconColorsNotDim.get(index) >= blueHueFarMin && beaconColorsNotDim.get(index) <= blueHueFarMax) {
            noDimValue = colorDecision_Blue;
        } else if (beaconColorsNotDim.get(index) >= redUpperHueMin && beaconColorsNotDim.get(index) <= redUpperHueMax) {
            noDimValue = colorDecision_Red;
        }

        if ((dimValue == colorDecision_NotBlue_NotRed && noDimValue == colorDecision_NotBlue_NotRed) || (dimValue == colorDecision_Blue && noDimValue == colorDecision_Red) || (dimValue == colorDecision_Red && noDimValue == colorDecision_Blue)) {
            return colorDecision_NotBlue_NotRed;
        } else if ((dimValue == colorDecision_Red) || noDimValue == colorDecision_Red) {
            return colorDecision_Red;
        } else {
            return colorDecision_Blue;
        }
    }

    public void triggerLeftButton() throws InterruptedException {
        if((beaconStart + (beaconRun / 2.0)) < midBeaconRangeRightMin) {
            //encoderDriveFront_TwoEnc_Short(TURN_SPEED,FRONT_THIRTY_SECOND_TURN,-FRONT_THIRTY_SECOND_TURN,FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_RIGHT_TURN_THREE,0.5);
            encoderDriveFront_TwoEnc_Short(TURN_SPEED,FRONT_SIXTEENTH_TURN,-FRONT_SIXTEENTH_TURN,FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_RIGHT_TURN_THREE,0.5);
            setLineState();
            telemetry.addData("Beacon state : ", beaconColorState);
            telemetry.addData("Line state : ", lineState);
            telemetry.addData("Beacon Start : ", beaconStart);
            telemetry.addData("Beacon Run : ", beaconRun);
            telemetry.addData("Going", "Right line too far right");
            telemetry.update();
            sleep(1000);

            while(lineState!= lineState_RIGHT && lineState != lineState_RIGHT_BACK && lineState != lineState_RIGHT_FRONT) {
                encoderDriveFront_TwoEnc_Short(DRIVE_SPEED, 1.0, 1.0, FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_STRAIGHT_THREE, 0.5); //orig. 3.0,3.0
                setLineState();
                telemetry.addData("Beacon state : ", beaconColorState);
                telemetry.addData("Line state : ", lineState);
                telemetry.addData("Beacon Start : ", beaconStart);
                telemetry.addData("Beacon Run : ", beaconRun);
                telemetry.addData("Going", "Right line too far right");
                telemetry.update();
                sleep(1000);
            }

            //encoderDriveFront_TwoEnc_Short(TURN_SPEED,-FRONT_THIRTY_SECOND_TURN,FRONT_THIRTY_SECOND_TURN,FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_LEFT_TURN_THREE,0.5);
            encoderDriveFront_TwoEnc_Short(TURN_SPEED,-FRONT_SIXTEENTH_TURN,FRONT_SIXTEENTH_TURN,FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_LEFT_TURN_THREE,0.5);
        }
        else if((beaconStart + (beaconRun / 2.0)) > midBeaconRangeRightMax) {
            //encoderDriveFront_TwoEnc_Short(TURN_SPEED,-FRONT_THIRTY_SECOND_TURN,FRONT_THIRTY_SECOND_TURN,FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_LEFT_TURN_THREE,0.5);
            encoderDriveFront_TwoEnc_Short(TURN_SPEED,-FRONT_SIXTEENTH_TURN,FRONT_SIXTEENTH_TURN,FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_LEFT_TURN_THREE,0.5);
            setLineState();
            telemetry.addData("Beacon state : ", beaconColorState);
            telemetry.addData("Line state : ", lineState);
            telemetry.addData("Beacon Start : ", beaconStart);
            telemetry.addData("Beacon Run : ", beaconRun);
            telemetry.addData("Going", "Right line too far left");
            telemetry.update();
            sleep(1000);

            while(lineState!= lineState_RIGHT && lineState != lineState_RIGHT_BACK && lineState != lineState_RIGHT_FRONT){
                encoderDriveFront_TwoEnc_Short(DRIVE_SPEED,1.0,1.0,FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_LEFT_TURN_THREE,0.5); //orig. 3.0,3.0
                setLineState();
                telemetry.addData("Beacon state : ", beaconColorState);
                telemetry.addData("Line state : ", lineState);
                telemetry.addData("Beacon Start : ", beaconStart);
                telemetry.addData("Beacon Run : ", beaconRun);
                telemetry.addData("Going", "Right line too far left");
                telemetry.update();
                sleep(1000);
            }

            //encoderDriveFront_TwoEnc_Short(TURN_SPEED,FRONT_THIRTY_SECOND_TURN,-FRONT_THIRTY_SECOND_TURN,FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_LEFT_TURN_THREE,0.5);
            encoderDriveFront_TwoEnc_Short(TURN_SPEED,FRONT_SIXTEENTH_TURN,-FRONT_SIXTEENTH_TURN,FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_LEFT_TURN_THREE,0.5);
        }
        else {
            setLineState();
            telemetry.addData("Beacon state : ", beaconColorState);
            telemetry.addData("Line state : ", lineState);
            telemetry.addData("Beacon Start : ", beaconStart);
            telemetry.addData("Beacon Run : ", beaconRun);
            telemetry.addData("Going", "Right line ahead");
            telemetry.update();
            sleep(1000);

            while(lineState!= lineState_RIGHT && lineState != lineState_RIGHT_BACK && lineState != lineState_RIGHT_FRONT) {
                encoderDriveFront_TwoEnc_Short(DRIVE_SPEED,1.0,1.0,FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_STRAIGHT_THREE,0.5); //orig. 3.0,3.0
                setLineState();
                telemetry.addData("Beacon state : ", beaconColorState);
                telemetry.addData("Line state : ", lineState);
                telemetry.addData("Beacon Start : ", beaconStart);
                telemetry.addData("Beacon Run : ", beaconRun);
                telemetry.addData("Going", "Right line ahead");
                telemetry.update();
                sleep(1000);
            }
        }
    }

    public void triggerRightButton() throws InterruptedException {
        encoderDriveFront_TwoEnc_Short(TURN_SPEED,FRONT_SIXTEENTH_TURN,-FRONT_SIXTEENTH_TURN,FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_STRAIGHT_THREE,0.5);

        if((beaconStart + (beaconRun / 2.0)) < midBeaconRangeLeftMin) {
            setLineState();
            telemetry.addData("Beacon state : ", beaconColorState);
            telemetry.addData("Line state : ", lineState);
            telemetry.addData("Beacon Start : ", beaconStart);
            telemetry.addData("Beacon Run : ", beaconRun);
            telemetry.addData("Going", "Left line too far left");
            telemetry.update();
            sleep(1000);
            do {
                encoderDriveFront_TwoEnc_Short(DRIVE_SPEED,1.0,1.0,FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_STRAIGHT_THREE,0.5); //orig. 3.0,3.0
                sleep(1000);
                setLineState();
                telemetry.addData("Beacon state : ", beaconColorState);
                telemetry.addData("Line state : ", lineState);
                telemetry.addData("Beacon Start : ", beaconStart);
                telemetry.addData("Beacon Run : ", beaconRun);
                telemetry.addData("Going", "Left line too far left");
                telemetry.update();
                sleep(1000);
            } while(lineState!= lineState_LEFT && lineState != lineState_LEFT_BACK && lineState != lineState_LEFT_FRONT);
        }
        else if((beaconStart + (beaconRun / 2.0)) > midBeaconRangeLeftMax) {
            setLineState();
            telemetry.addData("Beacon state : ", beaconColorState);
            telemetry.addData("Line state : ", lineState);
            telemetry.addData("Beacon Start : ", beaconStart);
            telemetry.addData("Beacon Run : ", beaconRun);
            telemetry.addData("Going", "Left line too far right");
            telemetry.update();
            sleep(1000);
            do {
                encoderDriveFront_TwoEnc_Short(DRIVE_SPEED,1.0,1.0,FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_STRAIGHT_THREE,0.5); //orig. 3.0,3.0
                sleep(1000);
                setLineState();
                telemetry.addData("Beacon state : ", beaconColorState);
                telemetry.addData("Line state : ", lineState);
                telemetry.addData("Beacon Start : ", beaconStart);
                telemetry.addData("Beacon Run : ", beaconRun);
                telemetry.addData("Going", "Left line too far right");
                telemetry.update();
                sleep(1000);
            } while(lineState!= lineState_LEFT && lineState != lineState_LEFT_BACK && lineState != lineState_LEFT_FRONT);
        }
        else {
            setLineState();
            telemetry.addData("Beacon state : ", beaconColorState);
            telemetry.addData("Line state : ", lineState);
            telemetry.addData("Beacon Start : ", beaconStart);
            telemetry.addData("Beacon Run : ", beaconRun);
            telemetry.addData("Going", "Left line ahead");
            telemetry.update();
            sleep(1000);
            do {
                encoderDriveFront_TwoEnc_Short(DRIVE_SPEED,1.0,1.0,FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_STRAIGHT_THREE,0.5); //orig. 3.0,3.0
                sleep(1000);
                setLineState();
                telemetry.addData("Beacon state : ", beaconColorState);
                telemetry.addData("Line state : ", lineState);
                telemetry.addData("Beacon Start : ", beaconStart);
                telemetry.addData("Beacon Run : ", beaconRun);
                telemetry.addData("Going", "Left line ahead");
                telemetry.update();
                sleep(1000);
            } while(lineState!= lineState_LEFT && lineState != lineState_LEFT_BACK && lineState != lineState_LEFT_FRONT);
        }

        do {
            encoderDriveFront_TwoEnc_Short(TURN_SPEED, -FRONT_THIRTY_SECOND_TURN, FRONT_THIRTY_SECOND_TURN, FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_STRAIGHT_THREE, 0.5);
            sleep(250);
        }while(lineState != lineState_LEFT);

        do {
            encoderDriveFront_TwoEnc_Short(DRIVE_SPEED,1.0,1.0,FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_STRAIGHT_THREE,0.5); //orig. 3.0,3.0
        }while((beaconStart + (beaconRun / 2.0)) < midBeaconRangeLeftMin && (beaconStart + (beaconRun / 2.0)) > midBeaconRangeLeftMax && beaconStart == 0);
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
            newFlickerTarget = currentFlickerPosition + (int) (position);

            DbgLog.msg(String.format("Psionics_Autonomous : Flicker starting at %7d ", currentFlickerPosition));

            robot.flickerMotor.setTargetPosition(newFlickerTarget);

            // Turn On RUN_TO_POSITION
            robot.flickerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            //set robot's speed
            robot.flickerMotor.setPower(Math.abs(speed));

            DbgLog.msg(String.format("Psionics_Autonomous : Flicker running to %7d ", newFlickerTarget));

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

            DbgLog.msg(String.format("Psionics_Autonomous : Flicker finally at %7d", currentFlickerPosition));

            //sleep(2000);   // optional pause after each move
        }
    }

    public void goToFirstBeaconBlueLineFollow() throws InterruptedException {
        SVL1 = robot.odsLF_bottom.getRawLightDetected();
        SVL2 = robot.odsRF_bottom.getRawLightDetected();
        SVR1 = robot.odsLB_bottom.getRawLightDetected();
        SVR2 = robot.odsRB_bottom.getRawLightDetected();
        if (thereIsBlueOnLeft(blueHueFarMin, blueHueFarMax)) {
            //lineFollowF(SVL2,SVR2);
            lineFollowF(0, 0); //left side
        } else if (thereIsBlueOnRight(blueHueFarMin, blueHueFarMax)) {
            //lineFollowB(SVL1,SVR1);
            lineFollowB(0, 0); //right side
        } else {
            encoderDriveBack(DRIVE_SPEED, 2.0, 2.0, FRONT_SIX_OR_LESS_TIMEOUT);
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

        switch ((int) inches) {
            case 1:
                b = 2.3;
                lr = 3;
                f = 2.5;
                break;
            case 2:
                b = 4.6;
                lr = 3;
                f = 5;
                break;
            case 3:
                b = 5.2;
                lr = 4;
                f = 6;
                break;
            case 6:
                b = 5.7;
                lr = 6;
                f = 8.3;
                break;
            default:
                b = 5.7;
                lr = 6;
                f = 8.3;
                break;
        }

        encoderDriveBack(DRIVE_SPEED, -b, -b, BACK_SIX_OR_LESS_TIMEOUT);
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

        switch ((int) inches) {
            case 1:
                b = 2.3;
                lr = 3;
                f = 2.5;
                break;
            case 2:
                b = 4.6;
                lr = 3;
                f = 5;
                break;
            case 3:
                b = 5.2;
                lr = 4;
                f = 6;
                break;
            case 6:
                b = 5.7;
                lr = 6;
                f = 8.3;
                break;
            default:
                b = 5.7;
                lr = 6;
                f = 8.3;
                break;
        }

        encoderDriveBack(DRIVE_SPEED, -b, -b, BACK_SIX_OR_LESS_TIMEOUT);
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
        if (blueHueInRange(0, 7, min, max)) {
            DbgLog.msg("Psionics_Autonomous : Found blue on right");

            return true;
        } else {
            DbgLog.msg("Psionics_Autonomous : Did not find blue on right");

            return false;
        }
    }

    public boolean thereIsBlueOnLeft(double min, double max) throws InterruptedException {
        if (blueHueInRange(5, 7, min, max)) {
            DbgLog.msg("Psionics_Autonomous : Found blue on left");

            return true;
        } else {
            DbgLog.msg("Psionics_Autonomous : Did not find blue on left");

            return false;
        }
    }

    public int findBlueRunOnRight(double min, double max) throws InterruptedException {
        int blueRun = 0;
        int startBlueRun = -1;

        waitForNextFrameReady();
        for (int i = 0; i < numColorSearchEndRight + 1; i++) {
            if ((poleSplitColorHSV[i].val[0] >= min) && (poleSplitColorHSV[i].val[0] <= max)) {
                if (startBlueRun < 0) {
                    startBlueRun = i;
                }
                blueRun++;
            } else if (startBlueRun >= 0) {
                i = 7;
            }
        }

        DbgLog.msg("Psionics_Autonomous : Run " + blueRun + " Start " + startBlueRun);
        return ((blueRun * numColorSearchWindows) + startBlueRun);
    }

    public int findBlueRunMidOnRight(double min, double max) throws InterruptedException {
        int blueMid = 0;
        int blueRun = 0;
        int blueStart = -1;

        waitForNextFrameReady();
        for (int i = 0; i < numColorSearchEndRight + 1; i++) {
            if ((poleSplitColorHSV[i].val[0] >= min) && (poleSplitColorHSV[i].val[0] <= max)) {
                if (blueStart < 0) {
                    blueStart = i;
                }
                blueRun++;
            } else if (blueStart >= 0) {
                i = 7;
            }
        }

        blueMid = blueStart + (blueRun / 2);
        DbgLog.msg("Psionics_Autonomous : Blue Right Middle: " + blueMid);
        return blueMid;
    }

    public int findBlueRunOnLeft(double min, double max) throws InterruptedException {
        int blueRun = 0;
        int startBlueRun = -1;

        waitForNextFrameReady();
        for (int i = numColorSearchStartLeft; i < numColorSearchWindows; i++) {
            if ((poleSplitColorHSV[i].val[0] >= min) && (poleSplitColorHSV[i].val[0] <= max)) {
                if (startBlueRun < 0) {
                    startBlueRun = i;
                }
                blueRun++;
            } else if (startBlueRun >= 0) {
                i = numColorSearchWindows;
            }
        }

        DbgLog.msg("Psionics_Autonomous : Run " + blueRun + " Start " + startBlueRun);
        return ((blueRun * numColorSearchWindows) + startBlueRun);
    }

    public int findBlueRunMidOnLeft(double min, double max) throws InterruptedException {
        int blueMid = 0;
        int blueRun = 0;
        int blueStart = -1;

        waitForNextFrameReady();
        for (int i = numColorSearchStartLeft; i < numColorSearchWindows; i++) {
            if ((poleSplitColorHSV[i].val[0] >= min) && (poleSplitColorHSV[i].val[0] <= max)) {
                if (blueStart < 0) {
                    blueStart = i;
                }
                blueRun++;
            } else if (blueStart >= 0) {
                i = numColorSearchWindows;
            }
        }

        blueMid = blueStart + (blueRun / 2);
        DbgLog.msg("Psionics_Autonomous : Blue Left Middle: " + blueMid);
        return blueMid;
    }

    public int findBlueRun(double min, double max) throws InterruptedException {
        int blueRun = 0;
        int startBlueRun = -1;

        waitForNextFrameReady();
        for (int i = 0; i < numColorSearchWindows; i++) {
            if ((poleSplitColorHSV[i].val[0] >= min) && (poleSplitColorHSV[i].val[0] <= max)) {
                if (startBlueRun < 0) {
                    startBlueRun = i;
                }
                blueRun++;
            } else if (startBlueRun >= 0) {
                i = numColorSearchWindows;
            }
        }

        DbgLog.msg("Psionics_Autonomous : Run " + blueRun + " Start " + startBlueRun);
        return ((blueRun * numColorSearchWindows) + startBlueRun);
    }

    public int findBlueRunMid(double min, double max) throws InterruptedException {
        int blueMid = 0;
        int blueRun = 0;
        int blueStart = -1;

        waitForNextFrameReady();
        for (int i = 0; i < numColorSearchWindows; i++) {
            if ((poleSplitColorHSV[i].val[0] >= min) && (poleSplitColorHSV[i].val[0] <= max)) {
                if (blueStart < 0) {
                    blueStart = i;
                }
                blueRun++;
            } else if (blueStart >= 0) {
                i = numColorSearchWindows;
            }
        }

        blueMid = blueStart + (blueRun / 2);
        DbgLog.msg("Psionics_Autonomous : Blue Middle: " + blueMid);
        return blueMid;
    }

    public boolean blueHueInRange(int start, int run, double min, double max) throws InterruptedException {
        waitForNextFrameReady();
        for (int i = start; i < (start + run); i++) {
            if ((poleSplitColorHSV[i].val[0] >= min) && (poleSplitColorHSV[i].val[0] <= max)) {
                DbgLog.msg("Psionics_Autonomous : Blue in range " + poleSplitColorHSV[i].val[0]);
                return true;
            }
        }
        return false;
    }

    public boolean redHueInRange(int start, int run) throws InterruptedException {
        waitForNextFrameReady();
        for (int i = start; i < (start + run); i++) {
            if (poleSplitColorHSV[i].val[0] >= redUpperHueMin && poleSplitColorHSV[i].val[0] <= redUpperHueMax) {
                return true;
            }
        }
        return false;
    }

    public boolean blueSaturationInRange(int start, int run) throws InterruptedException {
        waitForNextFrameReady();
        for (int i = start; i < (start + run); i++) {
            if (poleSplitColorHSV[i].val[1] >= blueSMin && poleSplitColorHSV[i].val[1] <= blueSMax) {
                return true;
            }
        }
        return false;
    }

    public boolean redSaturationInRange(int start, int run) throws InterruptedException {
        waitForNextFrameReady();
        for (int i = start; i < (start + run); i++) {
            if (poleSplitColorHSV[i].val[1] >= redUpperSMin && poleSplitColorHSV[i].val[1] <= redUpperSMax) {
                return true;
            }
        }
        return false;
    }

    public boolean blueValueInRange(int start, int run) throws InterruptedException {
        waitForNextFrameReady();
        for (int i = start; i < (start + run); i++) {
            if (poleSplitColorHSV[i].val[2] >= blueVMin && poleSplitColorHSV[i].val[2] <= blueVMax) {
                return true;
            }
        }
        return false;
    }

    public boolean redValueInRange(int start, int run) throws InterruptedException {
        waitForNextFrameReady();
        for (int i = start; i < (start + run); i++) {
            if (poleSplitColorHSV[i].val[2] >= redUpperVMin && poleSplitColorHSV[i].val[2] <= redUpperVMax) {
                return true;
            }
        }
        return false;
    }

    public void waitForNextFrameReady() throws InterruptedException {
        DbgLog.msg("Psionics_Autonomous : Waiting for next camera frame");
        FtcRobotControllerActivity.myPsionics_Vision_OpenCV.setNextFrameReady(false);
        while (!(FtcRobotControllerActivity.myPsionics_Vision_OpenCV.getNextFrameReady())) {
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

            if (leftFrontInches >= 0) leftDirection = 1.0;
            else leftDirection = -1.0;
            if (rightFrontInches >= 0) rightDirection = 1.0;
            else rightDirection = -1.0;
            if (leftDirection != rightDirection) directionCompensation = BACK_TURN_COMPENSATION;
            else directionCompensation = 1.0;

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = currentLeftFrontPosition + (int) (leftFrontInches * COUNTS_PER_INCH_FRONT);
            newRightFrontTarget = currentRightFrontPosition + (int) (rightFrontInches * COUNTS_PER_INCH_FRONT);
            newLeftBackTarget = currentLeftBackPosition + (int) (leftFrontInches * COUNTS_PER_INCH_BACK * directionCompensation);
            newRightBackTarget = currentRightBackPosition + (int) (rightFrontInches * COUNTS_PER_INCH_BACK * directionCompensation);

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
            robot.leftFrontMotor.setPower(Math.abs(speed));
            robot.rightFrontMotor.setPower(Math.abs(speed) * FRONT_RIGHT_SPEED_COMPENSATION_FRONT);

            robot.leftBackMotor.setPower(leftDirection * Math.abs(speed));
            robot.rightBackMotor.setPower(rightDirection * Math.abs(speed) * FRONT_RIGHT_SPEED_COMPENSATION_BACK);

            DbgLog.msg(String.format("Psionics_Autonomous : Running to %7d :%7d ", newLeftFrontTarget, newRightFrontTarget));

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

            DbgLog.msg(String.format("Psionics_Autonomous : Finally at %7d :%7d :%7d :%7d", currentLeftFrontPosition, currentRightFrontPosition, currentLeftBackPosition, currentRightBackPosition));

            cumulativeAddlRightRotation_Front += (currentLeftFrontPosition - newLeftFrontTarget) - (currentRightFrontPosition - newRightFrontTarget);
            cumulativeAddlRightRotation_Back += (currentLeftBackPosition - newLeftBackTarget) - (currentRightBackPosition - newRightBackTarget);
            DbgLog.msg(String.format("Psionics_Autonomous : Cumulative additional right rotation front : " + cumulativeAddlRightRotation_Front));
            DbgLog.msg(String.format("Psionics_Autonomous : Cumulative additional right rotation back : " + cumulativeAddlRightRotation_Back));
            DbgLog.msg(String.format("Psionics_Autonomous : Cumulative left front error : " + (newLeftFrontTarget - currentLeftFrontPosition)));
            DbgLog.msg(String.format("Psionics_Autonomous : Cumulative right front error : " + (newRightFrontTarget - currentRightFrontPosition)));
            if ((Math.abs(cumulativeAddlRightRotation_Front) >= FRONT_COMPENSATION_THRESHOLD_FRONT)
                    || (Math.abs(currentLeftFrontPosition - newLeftFrontTarget) >= FRONT_COMPENSATION_THRESHOLD_FRONT)
                    || (Math.abs(currentRightFrontPosition - newRightFrontTarget) >= FRONT_COMPENSATION_THRESHOLD_FRONT)
                    || (Math.abs(cumulativeAddlRightRotation_Back) >= FRONT_COMPENSATION_THRESHOLD_BACK)
                    ) {
                compTargetFront = cumulativeAddlRightRotation_Front;
                cumulativeAddlRightRotation_Front = 0.0;
                compTargetBack = cumulativeAddlRightRotation_Back;
                cumulativeAddlRightRotation_Back = 0.0;
                encoderDrive(DRIVE_SPEED, ((-compTargetFront + newLeftFrontTarget - currentLeftFrontPosition) / (2 * COUNTS_PER_INCH_FRONT)), ((compTargetFront + newRightFrontTarget - currentRightFrontPosition) / (2 * COUNTS_PER_INCH_FRONT)), (-compTargetBack / (2 * COUNTS_PER_INCH_BACK)), (compTargetBack / (2 * COUNTS_PER_INCH_BACK)), STOP_CONDITION.STOP_ON_BACK_MOTORS, FRONT_SIX_OR_LESS_TIMEOUT);
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

            if (leftFrontInches >= 0) leftDirection = 1.0;
            else leftDirection = -1.0;
            if (rightFrontInches >= 0) rightDirection = 1.0;
            else rightDirection = -1.0;
            if (leftDirection != rightDirection) directionCompensation = BACK_TURN_COMPENSATION;
            else directionCompensation = 1.0;

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = currentLeftFrontPosition + (int) (leftFrontInches * COUNTS_PER_INCH_FRONT);
            newRightFrontTarget = currentRightFrontPosition + (int) (rightFrontInches * COUNTS_PER_INCH_FRONT);
            newLeftBackTarget = currentLeftBackPosition + (int) (leftFrontInches * COUNTS_PER_INCH_BACK * directionCompensation);
            newRightBackTarget = currentRightBackPosition + (int) (rightFrontInches * COUNTS_PER_INCH_BACK * directionCompensation);

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
            robot.leftFrontMotor.setPower(Math.abs(speed));
            robot.rightFrontMotor.setPower(Math.abs(speed) * BACK_RIGHT_SPEED_COMPENSATION);
            robot.leftBackMotor.setPower(leftDirection * Math.abs(speed));
            robot.rightBackMotor.setPower(rightDirection * Math.abs(speed) * BACK_RIGHT_SPEED_COMPENSATION);

            DbgLog.msg(String.format("Psionics_Autonomous : Running to %7d :%7d ", newLeftFrontTarget, newRightFrontTarget));

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

            DbgLog.msg(String.format("Psionics_Autonomous : Finally at %7d :%7d :%7d :%7d", currentLeftFrontPosition, currentRightFrontPosition, currentLeftBackPosition, currentRightBackPosition));

            cumulativeAddlRightRotation_Front += (currentLeftFrontPosition - newLeftFrontTarget) - (currentRightFrontPosition - newRightFrontTarget);
            cumulativeAddlRightRotation_Back += (currentLeftBackPosition - newLeftBackTarget) - (currentRightBackPosition - newRightBackTarget);
            DbgLog.msg(String.format("Psionics_Autonomous : Cumulative additional right rotation front : " + cumulativeAddlRightRotation_Front));
            DbgLog.msg(String.format("Psionics_Autonomous : Cumulative additional right rotation back : " + cumulativeAddlRightRotation_Back));
            if ((Math.abs(cumulativeAddlRightRotation_Front) >= BACK_COMPENSATION_THRESHOLD_FRONT) || (Math.abs(cumulativeAddlRightRotation_Back) >= BACK_COMPENSATION_THRESHOLD_BACK)) {
                compTargetFront = cumulativeAddlRightRotation_Front;
                cumulativeAddlRightRotation_Front = 0.0;
                compTargetBack = cumulativeAddlRightRotation_Back;
                cumulativeAddlRightRotation_Back = 0.0;
                encoderDrive(DRIVE_SPEED, (-compTargetFront / (2 * COUNTS_PER_INCH_FRONT)), (compTargetFront / (2 * COUNTS_PER_INCH_FRONT)), (-compTargetBack / (2 * COUNTS_PER_INCH_BACK)), (compTargetBack / (2 * COUNTS_PER_INCH_BACK)), STOP_CONDITION.STOP_ON_BACK_MOTORS, BACK_SIX_OR_LESS_TIMEOUT);
            }

            //sleep(2000);   // optional pause after each move
        }
    }

    public void encoderDriveBack_TwoEnc(double speed, double leftFrontInches, double rightFrontInches, double timeoutS) throws InterruptedException {
        double leftDirection;
        double rightDirection;
        double compTargetFront;
        double compTargetBack;
        int newLeftFrontTarget;
        int newRightFrontTarget;

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
            robot.leftFrontMotor.setPower(Math.abs(speed) * BACK_LEFT_SPEED_COMPENSATION_FRONT);
            robot.rightFrontMotor.setPower(Math.abs(speed) * BACK_RIGHT_SPEED_COMPENSATION);
            robot.leftBackMotor.setPower(leftDirection * Math.abs(speed));
            robot.rightBackMotor.setPower(rightDirection * Math.abs(speed));

            DbgLog.msg(String.format("Psionics_Autonomous : Running to %7d :%7d ", newLeftFrontTarget, newRightFrontTarget));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    ((robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy()))) {
                // Allow time for other processes to run.
                idle();
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

            cumulativeAddlRightRotation_Front += (currentLeftFrontPosition - newLeftFrontTarget) - (currentRightFrontPosition - newRightFrontTarget);
            DbgLog.msg(String.format("Psionics_Autonomous : Cumulative additional right rotation front : " + cumulativeAddlRightRotation_Front));

            if ((Math.abs(cumulativeAddlRightRotation_Front) >= BACK_COMPENSATION_THRESHOLD_FRONT)) {
                compTargetFront = cumulativeAddlRightRotation_Front;
                cumulativeAddlRightRotation_Front = 0.0;
                encoderDriveBack_TwoEnc(TURN_SPEED, (-compTargetFront / (2 * COUNTS_PER_INCH_FRONT)), (compTargetFront / (2 * COUNTS_PER_INCH_FRONT)), 2.0);
            }

            //sleep(2000);   // optional pause after each move
        }
    }

    public void encoderDriveFront_TwoEnc_Short(double speed, double leftFrontInches, double rightFrontInches, double leftPowerCompensation, double timeoutS) throws InterruptedException {
        double leftDirection;
        double rightDirection;
        double compTargetFront;
        double compTargetBack;
        int newLeftFrontTarget;
        int newRightFrontTarget;

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
            newLeftFrontTarget = currentLeftFrontPosition + (int) (leftFrontInches * FRONT_TWO_ENCODER_DISTANCE_COMPENSATION_FRONT_LEFT_SHORT * COUNTS_PER_INCH_FRONT);
            newRightFrontTarget = currentRightFrontPosition + (int) (rightFrontInches * FRONT_TWO_ENCODER_DISTANCE_COMPENSATION_FRONT_RIGHT_SHORT * COUNTS_PER_INCH_FRONT);

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
            robot.leftFrontMotor.setPower(Math.abs(speed) * leftPowerCompensation);
            robot.rightFrontMotor.setPower(Math.abs(speed));
            robot.leftBackMotor.setPower(leftDirection * Math.abs(speed) * leftPowerCompensation);
            robot.rightBackMotor.setPower(rightDirection * Math.abs(speed));

            DbgLog.msg(String.format("Psionics_Autonomous : Running to %7d :%7d ", newLeftFrontTarget, newRightFrontTarget));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    ((robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy()))) {
                // Allow time for other processes to run.
                idle();
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

            /*cumulativeAddlRightRotation_Front += (currentLeftFrontPosition-newLeftFrontTarget)-(currentRightFrontPosition-newRightFrontTarget);
            DbgLog.msg(String.format("Psionics_Autonomous : Cumulative additional right rotation front : " + cumulativeAddlRightRotation_Front));

            if ((Math.abs(cumulativeAddlRightRotation_Front) >= BACK_COMPENSATION_THRESHOLD_FRONT)) {
                compTargetFront = cumulativeAddlRightRotation_Front;
                cumulativeAddlRightRotation_Front = 0.0;
                encoderDriveBack_TwoEnc(TURN_SPEED,(-compTargetFront/(2*COUNTS_PER_INCH_FRONT)),(compTargetFront/(2*COUNTS_PER_INCH_FRONT)),2.0);
            }*/

            //sleep(2000);   // optional pause after each move
        }
    }

    public void lineFollowB(double sensorValueL, double sensorValueR) throws InterruptedException {
        while (opModeIsActive()) {
            sensorValueL = robot.odsLF_bottom.getRawLightDetected();
            sensorValueR = robot.odsLB_bottom.getRawLightDetected();

            double colorL = getColorLB(sensorValueL); //right way to call this?
            double colorR = getColorRB(sensorValueR);
            telemetry.addData("Color L:", colorL);
            telemetry.addData("Color R:", colorR);
            telemetry.update();

            if (colorL == 0.0 && colorR == 1.0) {
                encoderDriveFront(DRIVE_SPEED, 1, 1, 3.0);
                sleep(1000);
            } else if (colorL == 1.0 && colorR == 0.0) {
                encoderDriveBack(DRIVE_SPEED, -3, 3, 3.0);
                encoderDriveFront(DRIVE_SPEED, 3, 3, 3.0);
                encoderDriveBack(DRIVE_SPEED, 3, -3, 3.0);
                sleep(1000);
            } else if (colorL == 1.0 && colorR == 1.0) {
                encoderDriveBack(DRIVE_SPEED, -3, 3, 3.0);
                encoderDriveFront(DRIVE_SPEED, 3, 3, 3.0);
                encoderDriveBack(DRIVE_SPEED, 3, -3, 3.0);
                sleep(1000);
            } else if (colorL == 0.0 && colorR == 0.0) {
                encoderDriveBack(DRIVE_SPEED, -3, 3, 3.0);
                encoderDriveFront(DRIVE_SPEED, 3, 3, 3.0);
                encoderDriveBack(DRIVE_SPEED, 3, -3, 3.0);
                sleep(1000);
            }
        }
    }


    public void lineFollowF(double sensorValueL, double sensorValueR) throws InterruptedException {
        while (1 == 1) {

            sensorValueL = robot.odsRF_bottom.getRawLightDetected();
            sensorValueR = robot.odsRB_bottom.getRawLightDetected();

            double colorL = getColorLF(sensorValueL); //right way to call this?
            double colorR = getColorRF(sensorValueR);
            telemetry.addData("Color L:", colorL);
            telemetry.addData("Color R:", colorR);
            telemetry.update();

            if (colorL == 0.0 && colorR == 1.0) {
                encoderDriveFront(DRIVE_SPEED, 1, 1, 3.0);
                sleep(1000);
            } else if (colorL == 1.0 && colorR == 0.0) {
                encoderDriveBack(DRIVE_SPEED, -3, 3, 3.0);
                encoderDriveFront(DRIVE_SPEED, 3, 3, 3.0);
                encoderDriveBack(DRIVE_SPEED, 3, -3, 3.0);
                sleep(1000);
            } else if (colorL == 1.0 && colorR == 1.0) {
                encoderDriveBack(DRIVE_SPEED, -3, 3, 3.0);
                encoderDriveFront(DRIVE_SPEED, 3, 3, 3.0);
                encoderDriveBack(DRIVE_SPEED, 3, -3, 3.0);
                sleep(1000);
            } else if (colorL == 0.0 && colorR == 0.0) {
                encoderDriveBack(DRIVE_SPEED, -3, 3, 3.0);
                encoderDriveFront(DRIVE_SPEED, 3, 3, 3.0);
                encoderDriveBack(DRIVE_SPEED, 3, -3, 3.0);
                sleep(1000);
                //     encoderDrive(DRIVE_SPEED, -3, -3, -3, -3, 6.0);     //figure out what to do if you don't sense a line
                sleep(1000);
            }
        }
    }

    public double getColorLF(double sensorValue) {
        double color = 0.0;
        if (sensorValue >= sensor_Threshold_LF) {
            color = 1.0;
        }
        return color;
    }

    public double getColorRF(double sensorValue) {
        double color = 0.0;
        if (sensorValue >= sensor_Threshold_RF) {
            color = 1.0;
        }
        return color;
    }

    public double getColorLB(double sensorValue) {
        double color = 0.0;
        if (sensorValue >= sensor_Threshold_LB) {
            color = 1.0;
        }
        return color;
    }

    public double getColorRB(double sensorValue) {
        double color = 0.0;
        if (sensorValue >= sensor_Threshold_RB) {
            color = 1.0;
        }
        return color;
    }

    public void setColorLF() {
        sensorValueLF = robot.odsLF_bottom.getRawLightDetected();
        if (sensorValueLF >= sensor_Threshold_LF) {
            sensorDecisionLF = 1.0;
        } else {
            sensorDecisionLF = 0.0;
        }
    }

    public void setColorLB() {
        sensorValueLB = robot.odsLB_bottom.getRawLightDetected();
        if (sensorValueLB >= sensor_Threshold_LB) {
            sensorDecisionLB = 1.0;
        } else {
            sensorDecisionLB = 0.0;
        }
    }

    public void setColorRF() {
        sensorValueRF = robot.odsRF_bottom.getRawLightDetected();
        if (sensorValueRF >= sensor_Threshold_RF) {
            sensorDecisionRF = 1.0;
        } else {
            sensorDecisionRF = 0.0;
        }
    }

    public void setColorRB() {
        sensorValueRB = robot.odsRB_bottom.getRawLightDetected();
        if (sensorValueRB >= sensor_Threshold_RB) {
            sensorDecisionRB = 1.0;
        } else {
            sensorDecisionRB = 0.0;
        }
    }

    public void getBottomSensorStatus() {
        /*telemetry.addData("Left Front Bottom Raw Value : " + robot.odsLF_bottom.getRawLightDetected(), "Sensor value : " + getColorLF(robot.odsLF_bottom.getRawLightDetected()));
        telemetry.addData("Left Back Bottom Raw Value : " + robot.odsLB_bottom.getRawLightDetected(), "Sensor value : " + getColorLB(robot.odsLB_bottom.getRawLightDetected()));
        telemetry.addData("Right Front Bottom Raw Value : " + robot.odsRF_bottom.getRawLightDetected(), "Sensor value : " + getColorRF(robot.odsRF_bottom.getRawLightDetected()));
        telemetry.addData("Right Back Bottom Raw Value : " + robot.odsRB_bottom.getRawLightDetected(), "Sensor value : " + getColorRB(robot.odsRB_bottom.getRawLightDetected()));*/

        telemetry.addData(String.format("Left Front Raw Value : %.2f", robot.odsLF_bottom.getRawLightDetected()), "Sensor value : " + getColorLF(robot.odsLF_bottom.getRawLightDetected()));
        telemetry.addData(String.format("Left Back Raw Value : %.2f", robot.odsLB_bottom.getRawLightDetected()), "Sensor value : " + getColorLB(robot.odsLB_bottom.getRawLightDetected()));
        telemetry.addData(String.format("Right Front Raw Value : %.2f", robot.odsRF_bottom.getRawLightDetected()), "Sensor value : " + getColorRF(robot.odsRF_bottom.getRawLightDetected()));
        telemetry.addData(String.format("Right Back Raw Value : %.2f", robot.odsRB_bottom.getRawLightDetected()), "Sensor value : " + getColorRB(robot.odsRB_bottom.getRawLightDetected()));
        telemetry.update();
    }

    public void setLineState() {
        setColorLF();
        setColorLB();
        setColorRF();
        setColorRB();

        switch ((int) (sensorDecisionLF + sensorDecisionLB + sensorDecisionRF + sensorDecisionRB)) {
            case 0:
            case 3:
            case 4:
                lineState = lineState_NONE;
                return;
            case 1:
                if (sensorDecisionLF == 1.0) {
                    lineState = lineState_LEFT_FRONT;
                    return;
                }
                if (sensorDecisionLB == 1.0) {
                    lineState = lineState_LEFT_BACK;
                    return;
                }
                if (sensorDecisionRF == 1.0) {
                    lineState = lineState_RIGHT_FRONT;
                    return;
                }
                if (sensorDecisionRB == 1.0) {
                    lineState = lineState_RIGHT_BACK;
                    return;
                }
            case 2:
                if (sensorDecisionLF == 1.0) {
                    if (sensorDecisionLB == 1.0) {
                        lineState = lineState_LEFT;
                        return;
                    }
                    else {
                        switch ((int) (sensorDecisionRB + sensorDecisionRF)) {
                            case 0:
                                lineState = lineState_LEFT_FRONT;
                                return;
                            case 1:
                                lineState = lineState_NONE;
                                return;
                        }
                    }
                }

                if (sensorDecisionLB == 1.0) {
                    switch ((int) (sensorDecisionRB + sensorDecisionRF)) {
                        case 0:
                            lineState = lineState_LEFT_BACK;
                            return;
                        case 1:
                            lineState = lineState_NONE;
                            return;
                    }
                }

                if (sensorDecisionRF == 1.0) {
                    if (sensorDecisionRB == 1.0) {
                        lineState = lineState_RIGHT;
                        return;
                    }
                }
                else {
                    switch ((int) (sensorDecisionLB + sensorDecisionLF)) {
                        case 0:
                            lineState = lineState_RIGHT_FRONT;
                            return;
                        case 1:
                            lineState = lineState_NONE;
                            return;
                    }
                }

                if (sensorDecisionRB == 1.0) {
                    switch ((int) (sensorDecisionLB + sensorDecisionLF)) {
                        case 0:
                            lineState = lineState_RIGHT_BACK;
                            return;
                        case 1:
                            lineState = lineState_NONE;
                            return;
                    }
                }
        }
    }
}
