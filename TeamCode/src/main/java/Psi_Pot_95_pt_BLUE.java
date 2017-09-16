import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.*;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import java.util.ArrayList;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import java.util.ArrayList;

/**
 * Created by Vaishnavi Addala on 2/20/2017.
 */



    /**
     * Created by geeth on 11/5/2016.
     */
    @Autonomous(name="Psi_Pot_95_pt_BLUE", group="Psionics")
//@Disabled
    public class Psi_Pot_95_pt_BLUE extends LinearOpMode {
        Psionics_Robot_Hardware_Regionals robot = new Psionics_Robot_Hardware_Regionals();
        //LineFollowerBR lineFollower = new LineFollowerBR();

        private final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
        private final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        private final double WHEEL_CIRCUMFRENCE_FRONT_INCHES = 15.0; // FR:15.0, FL:15.125
        private final double WHEEL_DIAMETER_BACK_INCHES = 4.0; // For figuring circumference
        private final double DISTANCE_ERROR_COMPENSATION = 1.0; // * (64.5/60.0); //(57.5 / 60) * (60.5 / 60) * (78 / 63); //orig. (57.5/60) * (60.5/60)
        private final double COUNTS_PER_INCH_FRONT = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (DISTANCE_ERROR_COMPENSATION * WHEEL_CIRCUMFRENCE_FRONT_INCHES);
        private final double COUNTS_PER_INCH_BACK = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (DISTANCE_ERROR_COMPENSATION * WHEEL_DIAMETER_BACK_INCHES * 3.1415);

        private final double FLICKER_ONE_REVOLUTION = COUNTS_PER_MOTOR_REV * 758 / 795;
        private final double FLICKER_THREE_FOURTHS_REVOLUTION = FLICKER_ONE_REVOLUTION * 0.75;

        private final double FRONT_WHEEL_BASE_WIDTH = 14.50;
        private final double FRONT_TURN_RADIUS = FRONT_WHEEL_BASE_WIDTH / 2;
        private final double BACK_WHEEl_BASE_WIDTH = 15.25;
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
        private final double DRIVE_SPEED_SHORT_UNTIL_LINE = DRIVE_SPEED_SHORT/2.0;
        private final double TURN_SPEED = 0.2;
    /*private final double FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_STRAIGHT_THREE = 0.50;
    private final double FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_LEFT_TURN_THREE = 0.20;
    private final double FRONT_TWO_ENCODER_LEFT_SPEED_COMPENSATION_RIGHT_TURN_THREE = 1.00;
    private final double FRONT_TWO_ENCODER_DISTANCE_COMPENSATION_FRONT_LEFT_SHORT = 26.0 / (31.0 * 1.2);
    private final double FRONT_TWO_ENCODER_DISTANCE_COMPENSATION_FRONT_RIGHT_SHORT = 26.0 / 27.5;*/

        private final double BACK_TWO_ENCODER_DISTANCE_COMPENSATION_FRONT_LEFT = (30.0/31.5); //(30.0/32.5); //orig. 26.0/31.0
        private final double BACK_TWO_ENCODER_DISTANCE_COMPENSATION_FRONT_RIGHT = (30.0/31.5); //(60.0/65.0) * (29.75/30.5) * (60.0/62.5) * (30.0/24.5); // orig. 26.0 / 27.5
        private final double BACK_TWO_ENCODER_LEFT_SPEED_COMPENSATION_FRONT = 0.5; //0.38; //(30.625/31.875) * 0.75 * (25.5/30.0) * (29.0/30.75) * (31.0/30.0) * (29.0/31.0); //orig. 0.75
        private final double BACK_TWO_ENCODER_RIGHT_SPEED_COMPENSATION_FRONT = 1.00; //(31.875/30.625) * 1.00 * (30.0/23.0) * (31.0/29.0); //orig. 0.75

        private final double FRONT_TWO_ENCODER_SHORT_DISTANCE_COMPENSATION_FRONT_LEFT = (30.0/32.5); //orig. 26.0/31.0
        private final double FRONT_TWO_ENCODER_SHORT_DISTANCE_COMPENSATION_FRONT_RIGHT = (29.75/30.5) * (60.0/62.5) * (30.0/24.5); // orig. 26.0 / 27.5
        private final double FRONT_TWO_ENCODER_SHORT_LEFT_SPEED_COMPENSATION_FRONT = 1.0; //(30.625/31.875) * 0.75 * (25.5/30.0) * (29.0/30.75) * (31.0/30.0) * (29.0/31.0); //orig. 0.75
        private final double FRONT_TWO_ENCODER_SHORT_LEFT_SPEED_COMPENSATION_BACK = 0.5;
        private final double FRONT_TWO_ENCODER_SHORT_RIGHT_SPEED_COMPENSATION_FRONT = 0.8; //(31.875/30.625) * 1.00 * (30.0/23.0) * (31.0/29.0); //orig. 0.75
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

        private double sensorValueLeft;
        private double sensorValueRight;

        private double sensorDecisionLF;
        private double sensorDecisionLB;
        private double sensorDecisionRF;
        private double sensorDecisionRB;

        private double sensorDecisionLeft;
        private double sensorDecisionRight;


        private final double sensor_Threshold_LF = 0.32;
        private final double sensor_Threshold_LB = 0.30;
        private final double sensor_Threshold_RF = 1.20;
        private final double sensor_Threshold_RB = 0.70;

        private final double sensor_Threshold_Left = 2.73;
        private final double sensor_Threshold_Right = 3.90;


        private final int lineState_NONE = 0;
        private final int lineState_LEFT = 1;
        private final int lineState_LEFT_FRONT = 2;
        private final int lineState_LEFT_BACK = 3;
        private final int lineState_RIGHT = 4;
        private final int lineState_RIGHT_FRONT = 5;
        private final int lineState_RIGHT_BACK = 6;
        private final int lineState_RIGHT_SENSOR = 7;
        private final int lineState_LEFT_SENSOR = 8;
        private final int lineState_LEFT_AND_RIGHT_SENSORS = 9;

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
        private final int numColorSearchWindows = org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity.myPsionics_Vision_OpenCV.numColorSearchWindows;
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

        private double distanceSensorState1 = 0.0;
        private double distanceSensorState2 = 0.0;
        private double distanceSensorState3 = 0.0;

        private int distanceReadCount = 0;

        private double initialBackwardDistance = 15.0;
        private double nextBackwardDistance = 0;

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
                FtcRobotControllerActivity.myPsionics_Vision_OpenCV.setBlueMode(true);
                robot.buttonPusher.setPosition(0.86);
                //do color detect
                //FtcRobotControllerActivity.myPsionics_Vision_OpenCV.setColorDetect(false);
                //telemetry.addData("Status", "Run Time: " + runtime.toString());
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

                    //go forward 12 in
              //      encoderDriveBack_TwoEnc(1.0,-15.0,-15.0, STOP_CONDITION.STOP_ON_ANY_FRONT,5.00);
                    //flick one rotation
              //      flickerDoFullRotation(FLICKER_ONE_REVOLUTION,FLICKER_SPEED,5.0);
                    //run intake
               //     intakeTimedDrive(1,1.5);
               //     sleep(500);
                    //flick one rotation
               //     flickerDoFullRotation(FLICKER_ONE_REVOLUTION,FLICKER_SPEED,5.0);
                    //turn 45 degrees right
             //       encoderDriveBack_TwoEnc_Turns(1.0,(FRONT_EIGHTH_TURN),(-FRONT_EIGHTH_TURN), STOP_CONDITION.STOP_ON_ANY_FRONT,5.0);
                    //go forward
              //      encoderDriveBack_TwoEnc(1.0,-69.0,-69.0, STOP_CONDITION.STOP_ON_ANY_FRONT,10.00);
                    //turn left 45 degrees
              //      encoderDriveBack_TwoEnc_Turns(1.0,(-FRONT_EIGHTH_TURN),(FRONT_EIGHTH_TURN), STOP_CONDITION.STOP_ON_ANY_FRONT,5.0);
                    //go forward 36 in while checking for line
                    getToLine();
                    //check color and push beacon with actuator
                    triggerBeacon(3.5);
                    //check if color is right -- if not wait and push again
                    //go back 36 in while checking for line
                    getToLine2();
                    //check color
                    triggerBeacon(3.5);
                    //push beacon with actuator
                    //check if color is right -- if not wait and push again
                    //go back 24 in
                    encoderDriveBack_TwoEnc(1.0,-24.0,-24.0, STOP_CONDITION.STOP_ON_ANY_FRONT,8.00);
                    //right turn left
                    encoderDriveBack_TwoEnc_Turns(1.0,(FRONT_EIGHTH_TURN),(-FRONT_EIGHTH_TURN), STOP_CONDITION.STOP_ON_ANY_FRONT,5.0);
                    encoderDriveBack_TwoEnc_Turns(1.0,(FRONT_EIGHTH_TURN),(-FRONT_EIGHTH_TURN), STOP_CONDITION.STOP_ON_ANY_FRONT,5.0);




                /*encoderDriveBack_TwoEnc(1.0,-15.0,-15.0, STOP_CONDITION.STOP_ON_ANY_FRONT,2.00); //orig. 0.80
                flickerDoFullRotation(FLICKER_ONE_REVOLUTION,FLICKER_SPEED,2.0);
                intakeTimedDrive(1,1.5);
                sleep(500);
                flickerDoFullRotation(FLICKER_ONE_REVOLUTION,FLICKER_SPEED,2.0);
                //encoderDriveBack_TwoEnc(1.0,1.2*(-FRONT_EIGHTH_TURN),1.2*(FRONT_EIGHTH_TURN), STOP_CONDITION.STOP_ON_ANY_FRONT,2.00); //orig. 1.00
                encoderDriveBack_TwoEnc_Turns(1.0,1.7*(FRONT_EIGHTH_TURN),1.7*(-FRONT_EIGHTH_TURN), STOP_CONDITION.STOP_ON_ANY_FRONT,2.00); //orig. 1.00
                encoderDriveBack_TwoEnc(1.0,-51.0,-51.0, STOP_CONDITION.STOP_ON_ANY_FRONT,5.00); //orig. 2.25; orig 51
                //encoderDriveBack_TwoEnc(0.75,1.35*(FRONT_EIGHTH_TURN),1.35*(-FRONT_EIGHTH_TURN), STOP_CONDITION.STOP_ON_ANY_FRONT,2.00); //orig.1.0
                encoderDriveBack_TwoEnc_Turns(1.0,1.40*(-FRONT_EIGHTH_TURN),1.40*(FRONT_EIGHTH_TURN), STOP_CONDITION.STOP_ON_FRONT_MOTORS,1.50); //orig.1.0
                encoderDriveBack_TwoEnc(1.0,5.0,5.0, STOP_CONDITION.STOP_ON_ANY_FRONT,1.70); //orig. 0.8
                //encoderDriveBack_TwoEnc(0.75,0.75*(FRONT_EIGHTH_TURN),0.75*(-FRONT_EIGHTH_TURN), STOP_CONDITION.STOP_ON_ANY_FRONT,2.0); //orig. 1.0
                //encoderDriveBack_TwoEnc_Turns(1.0,0.75*(-FRONT_EIGHTH_TURN),0.75*(FRONT_EIGHTH_TURN), STOP_CONDITION.STOP_ON_ANY_FRONT,2.0); //orig. 1.0
                //encoderDriveBack_TwoEnc(0.75,0.75*(FRONT_EIGHTH_TURN),0.75*(-FRONT_EIGHTH_TURN), STOP_CONDITION.STOP_ON_ANY_FRONT,2.0); //orig/ 1.0
                //encoderDriveBack_TwoEnc_Turns(1.0,0.75*(-FRONT_EIGHTH_TURN),0.75*(FRONT_EIGHTH_TURN), STOP_CONDITION.STOP_ON_ANY_FRONT,2.0); //orig. 1.0
                //encoderDriveBack_TwoEnc(1.0,15.0,15.0, STOP_CONDITION.STOP_ON_ANY_FRONT,2.0); //orig. 0.8
                //encoderDriveBack_TwoEnc(0.75,0.725*(-FRONT_EIGHTH_TURN),0.725*(FRONT_EIGHTH_TURN), STOP_CONDITION.STOP_ON_ANY_FRONT,2.0); //orig. 1.0
                //encoderDriveBack_TwoEnc_Turns(1.0,0.75*(FRONT_EIGHTH_TURN),0.75*(-FRONT_EIGHTH_TURN), STOP_CONDITION.STOP_ON_ANY_FRONT,2.0); //orig. 1.0
                //encoderDriveBack_TwoEnc(0.75,0.725*(-FRONT_EIGHTH_TURN),0.725*(FRONT_EIGHTH_TURN), STOP_CONDITION.STOP_ON_ANY_FRONT,2.0); //orig. 1.0
                //encoderDriveBack_TwoEnc_Turns(1.0,0.75*(FRONT_EIGHTH_TURN),0.75*(-FRONT_EIGHTH_TURN), STOP_CONDITION.STOP_ON_ANY_FRONT,2.0); //orig. 1.0

                //encoderDriveBack_TwoEnc(1.0,6.0,6.0, STOP_CONDITION.STOP_ON_ANY_FRONT,1.75); //orig. 0.8*/

                    //getToLine();

                    triggerBeacon(3.5);

                    //getToLine2();

                    //sleep(50);

                    //
                    //
                    //  triggerBeacon(3.5);



                    doOnce = true;
                }
                setLineStateSide();
                telemetry.addData("Line state :", lineState);
                telemetry.update();
                idle();
            }

            //cleanup
            //FtcRobotControllerActivity.myPsionics_Vision_OpenCV.setColorDetect(false);
        }

        public void getToLine() throws InterruptedException {
            //setLineState();
            //telemetry.addData("Line state : ", lineState);
            //telemetry.update();
            //while(lineState != lineState_LEFT_FRONT && lineState != lineState_RIGHT_FRONT) {
            //encoderDriveFront_TwoEnc_Short_UntilLine_TwoLineTarget(DRIVE_SPEED_SHORT_UNTIL_LINE,-24.0,-24.0,FRONT_TWO_ENCODER_SHORT_RIGHT_SPEED_COMPENSATION_FRONT_STRAIGHT_THREE,lineState_LEFT_FRONT,lineState_RIGHT_FRONT,STOP_CONDITION.STOP_ON_ANY_FRONT,3.60);
            encoderDriveBack_TwoEnc_Three_Line_Target(1.0,-24.0,-24.0,lineState_LEFT_SENSOR,lineState_RIGHT_SENSOR,lineState_LEFT_AND_RIGHT_SENSORS, STOP_CONDITION.STOP_ON_FRONT_MOTORS,1.28);
            encoderDriveBack_TwoEnc_Three_Line_Target(0.3,-12.0,-12.0,lineState_LEFT_SENSOR,lineState_RIGHT_SENSOR,lineState_LEFT_AND_RIGHT_SENSORS, STOP_CONDITION.STOP_ON_FRONT_MOTORS,1.28);
            sleep(50);
            setLineStateSide();
            while(lineState == lineState_NONE) {
                encoderDriveBack_TwoEnc_Three_Line_Target(0.2,1.0,1.0,lineState_LEFT_SENSOR,lineState_RIGHT_SENSOR,lineState_LEFT_AND_RIGHT_SENSORS, STOP_CONDITION.STOP_ON_FRONT_MOTORS,0.25);
                //sleep(50);
                setLineStateSide();
            }
            sleep(50);
        /*while(lineState != lineState_LEFT_AND_RIGHT_SENSORS) {
            if(lineState == lineState_RIGHT_SENSOR) {
                encoderDriveBack_TwoEnc(0.2,1.0,0.0,STOP_CONDITION.STOP_ON_FRONT_MOTORS,0.25);
            }
            else if(lineState == lineState_LEFT_SENSOR) {
                encoderDriveBack_TwoEnc(0.2,0.0,1.0,STOP_CONDITION.STOP_ON_FRONT_MOTORS,0.25);
            }
            else {
                //something's wrong or turning motions caused both sensors to go off line
            }
            setLineStateSide();
        }*/
            //setLineState();
            //telemetry.addData("Line state : ", lineState);
            //telemetry.update();
            //}
        }

        public void getToLine2() throws InterruptedException{
            //setLineState();
            //telemetry.addData("Line state : ", lineState);
            //telemetry.update();
            //while(lineState != lineState_LEFT_FRONT && lineState != lineState_RIGHT_FRONT) {
            //encoderDriveFront_TwoEnc_Short_UntilLine_TwoLineTarget(DRIVE_SPEED_SHORT_UNTIL_LINE,-24.0,-24.0,FRONT_TWO_ENCODER_SHORT_RIGHT_SPEED_COMPENSATION_FRONT_STRAIGHT_THREE,lineState_LEFT_FRONT,lineState_RIGHT_FRONT,STOP_CONDITION.STOP_ON_ANY_FRONT,3.60);
            //encoderDriveBack_TwoEnc(1.0,-24.0,-24.0,STOP_CONDITION.STOP_ON_FRONT_MOTORS,1.28);
            encoderDriveBack_TwoEnc_Three_Line_Target(1.0,-36.0,-36.0,lineState_LEFT_SENSOR,lineState_RIGHT_SENSOR,lineState_LEFT_AND_RIGHT_SENSORS, STOP_CONDITION.STOP_ON_FRONT_MOTORS,1.28);
            encoderDriveBack_TwoEnc_Three_Line_Target(0.3,-12.0,-12.0,lineState_LEFT_SENSOR,lineState_RIGHT_SENSOR,lineState_LEFT_AND_RIGHT_SENSORS, STOP_CONDITION.STOP_ON_FRONT_MOTORS,1.28);
            sleep(50);
            setLineStateSide();
            while(lineState == lineState_NONE) {
                encoderDriveBack_TwoEnc_Three_Line_Target(0.2,1.0,1.0,lineState_LEFT_SENSOR,lineState_RIGHT_SENSOR,lineState_LEFT_AND_RIGHT_SENSORS, STOP_CONDITION.STOP_ON_FRONT_MOTORS,0.25);
                //sleep(50);
                setLineStateSide();
            }
            sleep(50);
        /*while(lineState != lineState_LEFT_AND_RIGHT_SENSORS) {
            if(lineState == lineState_RIGHT_SENSOR) {
                encoderDriveBack_TwoEnc(0.2,1.0,0.0,STOP_CONDITION.STOP_ON_FRONT_MOTORS,0.25);
            }
            else if(lineState == lineState_LEFT_SENSOR) {
                encoderDriveBack_TwoEnc(0.2,0.0,1.0,STOP_CONDITION.STOP_ON_FRONT_MOTORS,0.25);
            }
            else {
                //something's wrong or turning motions caused both sensors to go off line
            }
            setLineStateSide();
        }*/
            //setLineState();
            //telemetry.addData("Line state : ", lineState);
            //telemetry.update();
            //}
        }

    /*public void triggerBeacon() throws InterruptedException {
        double actuator_position;

        findBeacon();
        beaconColorState = determineBeaconColorState();

        if(beaconColorState == 1.0) {
            robot.backRightActuator.setPosition(robot.BACKRIGHTACTUATOR_HOME+robot.ACTUATORS_BUTTON_PUSH_DISTANCE);
            do {
                actuator_position = robot.backRightActuator.getPosition();
            } while(actuator_position != (robot.BACKRIGHTACTUATOR_HOME+robot.ACTUATORS_BUTTON_PUSH_DISTANCE));
            sleep(50);
            robot.backRightActuator.setPosition(robot.BACKRIGHTACTUATOR_HOME);

        }
        else if(beaconColorState == 2.0) {
            robot.frontRightActuator.setPosition(robot.FRONTRIGHTACTUATOR_HOME+robot.ACTUATORS_BUTTON_PUSH_DISTANCE);
            do {
                actuator_position = robot.frontRightActuator.getPosition();
            } while(actuator_position != (robot.FRONTRIGHTACTUATOR_HOME+robot.ACTUATORS_BUTTON_PUSH_DISTANCE));
            sleep(50);
            robot.frontRightActuator.setPosition(robot.FRONTRIGHTACTUATOR_HOME);
        }
    }*/

        public void triggerBeacon(double timeoutS) throws InterruptedException {
            double actuator_position;

            findBeacon();
            beaconColorState = determineBeaconColorState();

            if(beaconColorState == 1.0) {
                while(opModeIsActive() && runtime.seconds() < timeoutS) {
                    robot.backRightActuator.setPosition(robot.FRONTLEFTACTUATOR_HOME + robot.ACTUATORS_BUTTON_PUSH_DISTANCE);
                }
                robot.backRightActuator.setPosition(robot.FRONTLEFTACTUATOR_HOME);
            }
            else if(beaconColorState == 2.0) {
                while(opModeIsActive() && runtime.seconds() < timeoutS){
                    robot.frontRightActuator.setPosition(robot.BACKLEFTACTUATOR_HOME+robot.ACTUATORS_BUTTON_PUSH_DISTANCE);
                }
                robot.frontRightActuator.setPosition(robot.BACKLEFTACTUATOR_HOME);
            }
        }

        public void encoderDriveFront_TwoEnc_Short_UntilLine_TwoLineTarget(double speed, double leftFrontInches, double rightFrontInches, double rightSpeedCompensation, int lineTarget1, int lineTarget2, STOP_CONDITION stop_condition, double timeoutS) throws InterruptedException {
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
                //int currentLeftBackPosition = robot.leftBackMotor.getCurrentPosition();
                //int currentRightBackPosition = robot.rightBackMotor.getCurrentPosition();

                if (leftFrontInches >= 0) leftDirection = 1.0;
                else leftDirection = -1.0;
                if (rightFrontInches >= 0) rightDirection = 1.0;
                else rightDirection = -1.0;

                // Determine new target position, and pass to motor controller
                newLeftFrontTarget = currentLeftFrontPosition + (int) (leftFrontInches * FRONT_TWO_ENCODER_SHORT_DISTANCE_COMPENSATION_FRONT_LEFT * COUNTS_PER_INCH_FRONT);
                newRightFrontTarget = currentRightFrontPosition + (int) (rightFrontInches * FRONT_TWO_ENCODER_SHORT_DISTANCE_COMPENSATION_FRONT_RIGHT * COUNTS_PER_INCH_FRONT);

                //DbgLog.msg(String.format("Psionics_Autonomous : Starting at %7d :%7d :%7d :%7d", currentLeftFrontPosition, currentRightFrontPosition, currentLeftBackPosition, currentRightBackPosition));

                robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
                robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);

                // Turn On RUN_TO_POSITION
                robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                // reset the timeout time and start motion.
                runtime.reset();

                //set robot's speed
                robot.leftFrontMotor.setPower(Math.abs(speed) * FRONT_TWO_ENCODER_SHORT_LEFT_SPEED_COMPENSATION_FRONT);
                robot.rightFrontMotor.setPower(Math.abs(speed) * FRONT_TWO_ENCODER_SHORT_RIGHT_SPEED_COMPENSATION_FRONT * rightSpeedCompensation);
                //robot.leftBackMotor.setPower(leftDirection * Math.abs(speed) * FRONT_TWO_ENCODER_SHORT_LEFT_SPEED_COMPENSATION_BACK);
                //robot.rightBackMotor.setPower(rightDirection * Math.abs(speed) * rightSpeedCompensation);

                //DbgLog.msg(String.format("Psionics_Autonomous : Running to %7d :%7d ", newLeftFrontTarget, newRightFrontTarget));

                setLineState();
                if(stop_condition == STOP_CONDITION.STOP_ON_ANY_FRONT) {
                    // keep looping while we are still active, and there is time left, and both front motors are running.
                    while (opModeIsActive() &&
                            (runtime.seconds() < timeoutS) &&
                            ((robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy()))
                            && lineState != lineTarget1 && lineState != lineTarget2) {
                        // Allow time for other processes to run.
                        setLineState();
                        idle();
                    }
                }
                else if(stop_condition == STOP_CONDITION.STOP_ON_FRONT_MOTORS) {
                    // keep looping while we are still active, and there is time left, and either front motors are running.
                    do {
                        leftFrontMotorBusy = robot.leftFrontMotor.isBusy();
                        rightFrontMotorBusy = robot.rightFrontMotor.isBusy();
                        if (!leftFrontMotorBusy) {
                            //robot.leftBackMotor.setPower(0);
                            robot.leftFrontMotor.setPower(0);
                        } else if (!rightFrontMotorBusy) {
                            //robot.rightBackMotor.setPower(0);
                            robot.rightFrontMotor.setPower(0);
                        }
                        // Allow time for other processes to run.
                        setLineState();
                        idle();
                    }
                    while (opModeIsActive() && (runtime.seconds() < timeoutS) && (leftFrontMotorBusy || rightFrontMotorBusy)
                            && lineState != lineTarget1 && lineState != lineTarget2);
                }

                // Stop all motion;
                //robot.leftBackMotor.setPower(0);
                robot.leftFrontMotor.setPower(0);
                //robot.rightBackMotor.setPower(0);
                robot.rightFrontMotor.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            /*sleep(30);

            currentLeftFrontPosition = robot.leftFrontMotor.getCurrentPosition();
            currentRightFrontPosition = robot.rightFrontMotor.getCurrentPosition();
            currentLeftBackPosition = robot.leftBackMotor.getCurrentPosition();
            currentRightBackPosition = robot.rightBackMotor.getCurrentPosition();*/

                //DbgLog.msg(String.format("Psionics_Autonomous : Finally at %7d :%7d :%7d :%7d", currentLeftFrontPosition, currentRightFrontPosition, currentLeftBackPosition, currentRightBackPosition));
            }
        }

        public void encoderDriveFront_TwoEnc_Short_UntilLine_ThreeLineTarget(double speed, double leftFrontInches, double rightFrontInches, double rightSpeedCompensation, int lineTarget1, int lineTarget2, int lineTarget3, STOP_CONDITION stop_condition, double timeoutS) throws InterruptedException {
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
                //int currentLeftBackPosition = robot.leftBackMotor.getCurrentPosition();
                //int currentRightBackPosition = robot.rightBackMotor.getCurrentPosition();

                if (leftFrontInches >= 0) leftDirection = 1.0;
                else leftDirection = -1.0;
                if (rightFrontInches >= 0) rightDirection = 1.0;
                else rightDirection = -1.0;

                // Determine new target position, and pass to motor controller
                newLeftFrontTarget = currentLeftFrontPosition + (int) (leftFrontInches * FRONT_TWO_ENCODER_SHORT_DISTANCE_COMPENSATION_FRONT_LEFT * COUNTS_PER_INCH_FRONT);
                newRightFrontTarget = currentRightFrontPosition + (int) (rightFrontInches * FRONT_TWO_ENCODER_SHORT_DISTANCE_COMPENSATION_FRONT_RIGHT * COUNTS_PER_INCH_FRONT);

                //DbgLog.msg(String.format("Psionics_Autonomous : Starting at %7d :%7d :%7d :%7d", currentLeftFrontPosition, currentRightFrontPosition, currentLeftBackPosition, currentRightBackPosition));

                robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
                robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);

                // Turn On RUN_TO_POSITION
                robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                // reset the timeout time and start motion.
                runtime.reset();

                //set robot's speed
                robot.leftFrontMotor.setPower(Math.abs(speed) * FRONT_TWO_ENCODER_SHORT_LEFT_SPEED_COMPENSATION_FRONT);
                robot.rightFrontMotor.setPower(Math.abs(speed) * FRONT_TWO_ENCODER_SHORT_RIGHT_SPEED_COMPENSATION_FRONT * rightSpeedCompensation);
                //robot.leftBackMotor.setPower(leftDirection * Math.abs(speed) * FRONT_TWO_ENCODER_SHORT_LEFT_SPEED_COMPENSATION_BACK);
                //robot.rightBackMotor.setPower(rightDirection * Math.abs(speed) * rightSpeedCompensation);

                //DbgLog.msg(String.format("Psionics_Autonomous : Running to %7d :%7d ", newLeftFrontTarget, newRightFrontTarget));

                setLineState();
                if(stop_condition == STOP_CONDITION.STOP_ON_ANY_FRONT) {
                    // keep looping while we are still active, and there is time left, and both front motors are running.
                    while (opModeIsActive() &&
                            (runtime.seconds() < timeoutS) &&
                            ((robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy()))
                            && lineState != lineTarget1 && lineState != lineTarget2 && lineState != lineTarget3) {
                        // Allow time for other processes to run.
                        setLineState();
                        idle();
                    }
                }
                else if(stop_condition == STOP_CONDITION.STOP_ON_FRONT_MOTORS) {
                    // keep looping while we are still active, and there is time left, and either front motors are running.
                    do {
                        leftFrontMotorBusy = robot.leftFrontMotor.isBusy();
                        rightFrontMotorBusy = robot.rightFrontMotor.isBusy();
                        if (!leftFrontMotorBusy) {
                            //robot.leftBackMotor.setPower(0);
                            robot.leftFrontMotor.setPower(0);
                        } else if (!rightFrontMotorBusy) {
                            //robot.rightBackMotor.setPower(0);
                            robot.rightFrontMotor.setPower(0);
                        }
                        // Allow time for other processes to run.
                        setLineState();
                        idle();
                    }
                    while (opModeIsActive() && (runtime.seconds() < timeoutS) && (leftFrontMotorBusy || rightFrontMotorBusy)
                            && lineState != lineTarget1 && lineState != lineTarget2 && lineState != lineTarget3);
                }

                // Stop all motion;
                //robot.leftBackMotor.setPower(0);
                robot.leftFrontMotor.setPower(0);
                //robot.rightBackMotor.setPower(0);
                robot.rightFrontMotor.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            /*sleep(30);

            currentLeftFrontPosition = robot.leftFrontMotor.getCurrentPosition();
            currentRightFrontPosition = robot.rightFrontMotor.getCurrentPosition();
            currentLeftBackPosition = robot.leftBackMotor.getCurrentPosition();
            currentRightBackPosition = robot.rightBackMotor.getCurrentPosition();*/

                //DbgLog.msg(String.format("Psionics_Autonomous : Finally at %7d :%7d :%7d :%7d", currentLeftFrontPosition, currentRightFrontPosition, currentLeftBackPosition, currentRightBackPosition));
            }
        }

        public void positionRobotForBeacon() throws InterruptedException {
            cumulativeBeaconFindingForwardDistance = 0.0;
            beaconColorState = beaconState_NotSure;
        /*do {
            findBeacon();
            if(beaconColorState == beaconState_NotSure) {beaconColorState = determineBeaconColorState();}
            telemetry.addData("Beacon state : ", beaconColorState);
            telemetry.addData("Line state : ", lineState);
            telemetry.addData("Beacon Start : ", beaconStart);
            telemetry.addData("Beacon Run : ", beaconRun);
            telemetry.addData("Going", "Determine beacon state");
            telemetry.update();
            if(beaconRun < 3) { //run is not enough to see beacon well; move closer
                if(beaconStart < 3) {
                    encoderDriveBack_TwoEnc(TURN_SPEED,1.1*(FRONT_QUARTER_TURN),1.1*(-FRONT_QUARTER_TURN),STOP_CONDITION.STOP_ON_ANY_FRONT,2.40);
                    encoderDriveFront_TwoEnc_Short(DRIVE_SPEED_SHORT,6.0,6.0,FRONT_TWO_ENCODER_SHORT_RIGHT_SPEED_COMPENSATION_FRONT_STRAIGHT_THREE,STOP_CONDITION.STOP_ON_ANY_FRONT,0.84);
                    encoderDriveBack_TwoEnc(TURN_SPEED,1.1*(-FRONT_QUARTER_TURN),1.1*(FRONT_QUARTER_TURN),STOP_CONDITION.STOP_ON_ANY_FRONT,2.40);
                }
                else if(beaconStart > 5) {
                    encoderDriveBack_TwoEnc(TURN_SPEED,1.1*(-FRONT_QUARTER_TURN),1.1*(FRONT_QUARTER_TURN),STOP_CONDITION.STOP_ON_ANY_FRONT,2.40);
                    encoderDriveFront_TwoEnc_Short(DRIVE_SPEED_SHORT,6.0,6.0,FRONT_TWO_ENCODER_SHORT_RIGHT_SPEED_COMPENSATION_FRONT_STRAIGHT_THREE,STOP_CONDITION.STOP_ON_ANY_FRONT,0.84);
                    encoderDriveBack_TwoEnc(TURN_SPEED,1.1*(FRONT_QUARTER_TURN),1.1*(-FRONT_QUARTER_TURN),STOP_CONDITION.STOP_ON_ANY_FRONT,2.40);
                }
                encoderDriveFront_TwoEnc_Short(DRIVE_SPEED_SHORT,3.0,3.0,FRONT_TWO_ENCODER_SHORT_RIGHT_SPEED_COMPENSATION_FRONT_STRAIGHT_THREE,STOP_CONDITION.STOP_ON_ANY_FRONT,0.42);
            }
            else if(beaconRun > 4) { //robot is too close; move back
                encoderDriveFront_TwoEnc_Short(DRIVE_SPEED_SHORT,-3.0,-3.0,FRONT_TWO_ENCODER_SHORT_RIGHT_SPEED_COMPENSATION_FRONT_STRAIGHT_THREE,STOP_CONDITION.STOP_ON_ANY_FRONT,0.42);
                cumulativeBeaconFindingForwardDistance -= 3.0;
            }
            else if(beaconStart < 3) {
                encoderDriveBack_TwoEnc(TURN_SPEED,1.1*(FRONT_QUARTER_TURN),1.1*(-FRONT_QUARTER_TURN),STOP_CONDITION.STOP_ON_ANY_FRONT,2.40);
                encoderDriveFront_TwoEnc_Short(DRIVE_SPEED_SHORT,3.0,3.0,FRONT_TWO_ENCODER_SHORT_RIGHT_SPEED_COMPENSATION_FRONT_STRAIGHT_THREE,STOP_CONDITION.STOP_ON_ANY_FRONT,0.84);
                encoderDriveBack_TwoEnc(TURN_SPEED,1.1*(-FRONT_QUARTER_TURN),1.1*(FRONT_QUARTER_TURN),STOP_CONDITION.STOP_ON_ANY_FRONT,2.40);
            }
            else if(beaconStart > 5) {
                encoderDriveBack_TwoEnc(TURN_SPEED,1.1*(-FRONT_QUARTER_TURN),1.1*(FRONT_QUARTER_TURN),STOP_CONDITION.STOP_ON_ANY_FRONT,2.40);
                encoderDriveFront_TwoEnc_Short(DRIVE_SPEED_SHORT,3.0,3.0,FRONT_TWO_ENCODER_SHORT_RIGHT_SPEED_COMPENSATION_FRONT_STRAIGHT_THREE,STOP_CONDITION.STOP_ON_ANY_FRONT,0.84);
                encoderDriveBack_TwoEnc(TURN_SPEED,1.1*(FRONT_QUARTER_TURN),1.1*(-FRONT_QUARTER_TURN),STOP_CONDITION.STOP_ON_ANY_FRONT,2.40);
            }
            else if(beaconColorState == beaconState_NotSure) { //still not sure of what the state of the beacon is; more forward in small movements
                encoderDriveFront_TwoEnc_Short(DRIVE_SPEED_SHORT,1.0,1.0,FRONT_TWO_ENCODER_SHORT_RIGHT_SPEED_COMPENSATION_FRONT_STRAIGHT_THREE,STOP_CONDITION.STOP_ON_ANY_FRONT,0.15);
                cumulativeBeaconFindingForwardDistance += 1.0;
            }
        } while ((beaconColorState == beaconState_NotSure) || (beaconStart < 3) || (beaconStart > 5) || (beaconRun < 3) || (beaconRun > 4));*/

            findBeacon();
            if(beaconColorState == beaconState_NotSure) {beaconColorState = determineBeaconColorState();}

            if (beaconColorState == beaconState_BlueOnLeft) {
            /*telemetry.addData("Beacon state : ", beaconColorState);
            telemetry.addData("Line state : ", lineState);
            telemetry.addData("Beacon Start : ", beaconStart);
            telemetry.addData("Beacon Run : ", beaconRun);
            telemetry.addData("Going", "Blue left");
            telemetry.update();*/
                triggerLeftButton();
            }
            else {
            /*telemetry.addData("Beacon state : ", beaconColorState);
            telemetry.addData("Line state : ", lineState);
            telemetry.addData("Beacon Start : ", beaconStart);
            telemetry.addData("Beacon Run : ", beaconRun);
            telemetry.addData("Going", "Blue right");
            telemetry.update();*/
                triggerRightButton();
            }
        }

        public void findBeacon() throws InterruptedException { //assumes that there is only one run of brightness
            beaconStart = -1;
            beaconRun = 0;
            beaconColorDecision.clear();
            beaconColorsDim.clear();
            beaconColorsDimOverall.clear();
            beaconColorsNotDim.clear();
            beaconColorsNotDimOverall.clear();
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
            //DbgLog.msg("Psionics_Autonomous : Beacon Start: " + beaconStart);
            //DbgLog.msg("Psionics_Autonomous : Beacon Run: " + beaconRun);
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
                    if (((redMin + redMax)) <= ((beaconColorDecision.size()))) {
                        return beaconState_BlueOnLeft;
                    } else {
                        return beaconState_RedOnLeft;
                    }
                } else if (redMin < 0) {
                    if ((blueMin + blueMax) <= ((beaconColorDecision.size()))) {
                        return beaconState_RedOnLeft;
                    } else {
                        return beaconState_BlueOnLeft;
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

        public boolean isAllBlue() throws InterruptedException {
            double colorDecision;
            findBeacon();
            for(int i=0; i<beaconColorsDim.size(); i++) {
                colorDecision = beaconColorDecision(i);
                if(colorDecision != colorDecision_Blue) {
                    return false;
                }
            }
            return true;
        }

        public boolean isAllRed() throws InterruptedException {
            double colorDecision;
            findBeacon();
            for(int i=0; i<beaconColorsDim.size(); i++) {
                colorDecision = beaconColorDecision(i);
                if(colorDecision != colorDecision_Red) {
                    return false;
                }
            }
            return true;
        }

        public void triggerLeftButton() throws InterruptedException {
            //robot.buttonPusher.setPosition(0.42);
            encoderDriveBack_TwoEnc(1.0,1.1*(-FRONT_EIGHTH_TURN),1.1*(FRONT_EIGHTH_TURN), STOP_CONDITION.STOP_ON_FRONT_MOTORS,1.0);
            //setLineState();
            /*telemetry.addData("Beacon state : ", beaconColorState);
            telemetry.addData("Line state : ", lineState);
            telemetry.addData("Beacon Start : ", beaconStart);
            telemetry.addData("Beacon Run : ", beaconRun);
            telemetry.addData("Going", "Right line too far right");
            telemetry.update();*/

            //while(lineState!= lineState_RIGHT && lineState != lineState_RIGHT_BACK && lineState != lineState_RIGHT_FRONT) {
            //encoderDriveFront_TwoEnc_Short(DRIVE_SPEED_SHORT,1.0,1.0,FRONT_TWO_ENCODER_SHORT_RIGHT_SPEED_COMPENSATION_FRONT_STRAIGHT_THREE,STOP_CONDITION.STOP_ON_ANY_FRONT,0.15);
            encoderDriveFront_TwoEnc_Short_UntilLine_ThreeLineTarget(1.0,12.0,12.0,FRONT_TWO_ENCODER_SHORT_RIGHT_SPEED_COMPENSATION_FRONT_STRAIGHT_THREE,lineState_RIGHT,lineState_RIGHT_FRONT,lineState_RIGHT_BACK, STOP_CONDITION.STOP_ON_FRONT_MOTORS,1.80);
            //encoderDriveBack_TwoEnc_Three_Line_Target(DRIVE_SPEED,12.0,12.0,lineState_RIGHT_FRONT,lineState_RIGHT_BACK,lineState_RIGHT,STOP_CONDITION.STOP_ON_FRONT_MOTORS,0.80);
            //cumulativeBeaconFindingForwardDistance += 0.38; //1*sin(pi/8)
            //setLineState();
                /*telemetry.addData("Beacon state : ", beaconColorState);
                telemetry.addData("Line state : ", lineState);
                telemetry.addData("Beacon Start : ", beaconStart);
                telemetry.addData("Beacon Run : ", beaconRun);
                telemetry.addData("Going", "Right line too far right");
                telemetry.update();*/
            //}

            straightenToLineLeft();
        }

        public void triggerRightButton() throws InterruptedException {
            //robot.buttonPusher.setPosition(0.42);
            encoderDriveBack_TwoEnc(1.0, 1.1 * (FRONT_EIGHTH_TURN), 1.1 * (-FRONT_EIGHTH_TURN), STOP_CONDITION.STOP_ON_FRONT_MOTORS, 1.0); //orig. sixteenth
            //setLineState();
        /*telemetry.addData("Beacon state : ", beaconColorState);
        telemetry.addData("Line state : ", lineState);
        telemetry.addData("Beacon Start : ", beaconStart);
        telemetry.addData("Beacon Run : ", beaconRun);
        telemetry.addData("Going", "Left line too far left");
        telemetry.update();*/

            //while (lineState != lineState_LEFT && lineState != lineState_LEFT_BACK && lineState != lineState_LEFT_FRONT) {
            //encoderDriveFront_TwoEnc_Short(DRIVE_SPEED_SHORT, 1.0, 1.0, FRONT_TWO_ENCODER_SHORT_RIGHT_SPEED_COMPENSATION_FRONT_STRAIGHT_THREE, STOP_CONDITION.STOP_ON_ANY_FRONT, 0.15);
            encoderDriveFront_TwoEnc_Short_UntilLine_ThreeLineTarget(1.0,12.0,12.0,FRONT_TWO_ENCODER_SHORT_RIGHT_SPEED_COMPENSATION_FRONT_STRAIGHT_THREE,lineState_LEFT,lineState_LEFT_FRONT,lineState_LEFT_BACK, STOP_CONDITION.STOP_ON_FRONT_MOTORS,1.80);
            //encoderDriveBack_TwoEnc_Three_Line_Target(DRIVE_SPEED,12.0,12.0,lineState_RIGHT_FRONT,lineState_RIGHT_BACK,lineState_RIGHT,STOP_CONDITION.STOP_ON_FRONT_MOTORS,0.80);
            //cumulativeBeaconFindingForwardDistance += 0.71; //1*sin(pi/4)
            //setLineState();
            /*telemetry.addData("Beacon state : ", beaconColorState);
            telemetry.addData("Line state : ", lineState);
            telemetry.addData("Beacon Start : ", beaconStart);
            telemetry.addData("Beacon Run : ", beaconRun);
            telemetry.addData("Going", "Left line too far left");
            telemetry.update();*/
            //}

            straightenToLineRight();
        }

        public void straightenToLineLeft() throws InterruptedException {
            int count = 0;
            int iterationMax = 5;
            double iterationTurnSize = (1.1/iterationMax)*(FRONT_EIGHTH_TURN);
            double iterationBackSize = (-2.0/iterationMax);

            encoderDriveFront_TwoEnc_Short(1.0, -2.0, -2.0, FRONT_TWO_ENCODER_SHORT_RIGHT_SPEED_COMPENSATION_FRONT_STRAIGHT_THREE, STOP_CONDITION.STOP_ON_FRONT_MOTORS, 0.38);
            setLineState();
            while(lineState != lineState_RIGHT && count < iterationMax) {
                encoderDriveBack_TwoEnc(1.0,iterationTurnSize,-iterationTurnSize, STOP_CONDITION.STOP_ON_FRONT_MOTORS,1.5);
                setLineState();
                if(lineState == lineState_RIGHT_FRONT) {
                    encoderDriveFront_TwoEnc_Short(1.0, iterationBackSize, iterationBackSize, FRONT_TWO_ENCODER_SHORT_RIGHT_SPEED_COMPENSATION_FRONT_STRAIGHT_THREE, STOP_CONDITION.STOP_ON_FRONT_MOTORS, 0.38);
                    setLineState();
                }
                count++;
            }
            //count = 0;
            while (distanceSensorState1 < 0.09 && distanceSensorState2 < 0.14 && distanceSensorState3 < 0.12) {
                encoderDriveFront_TwoEnc_Short(1.0, 2.0, 2.0, FRONT_TWO_ENCODER_SHORT_RIGHT_SPEED_COMPENSATION_FRONT_STRAIGHT_THREE, STOP_CONDITION.STOP_ON_FRONT_MOTORS, 0.30); //uncomment
                encoderDriveBack_TwoEnc(1.0,1.0*0.1875*(-iterationTurnSize),1.0*0.1875*(iterationTurnSize), STOP_CONDITION.STOP_ON_FRONT_MOTORS,0.50); //orig 0.125
                //count++;
            /*if(count == 6) {
                encoderDriveBack_TwoEnc(TURN_SPEED,iterationTurnSize,-iterationTurnSize,STOP_CONDITION.STOP_ON_ANY_FRONT,1.5);
                count = 0;
            }*/
                //setDistanceSensorState(); //uncomment

                for(int i=0; i<10; i++) {
                    //updateDistanceSensorState();
                }
                //setLineState();
            /*if(lineState != lineState_RIGHT) {
                encoderDriveBack_TwoEnc(TURN_SPEED,2*(iterationTurnSize),2*(-iterationTurnSize),STOP_CONDITION.STOP_ON_ANY_FRONT,1.5);
            }*/
            } //uncomment
            //robot.buttonPusher.setPosition(0.6);
        }

        public void straightenToLineRight() throws InterruptedException {
            int count = 0;
            int iterationMax = 5;
            double iterationTurnSize = (1.1/iterationMax)*(FRONT_EIGHTH_TURN);
            double iterationBackSize = (-3.8/iterationMax);

            encoderDriveFront_TwoEnc_Short(1.0, -3.8, -3.8, FRONT_TWO_ENCODER_SHORT_RIGHT_SPEED_COMPENSATION_FRONT_STRAIGHT_THREE, STOP_CONDITION.STOP_ON_FRONT_MOTORS, 0.38); //orig. 4.0
            setLineState();
            while(lineState != lineState_LEFT && count < iterationMax) {
                encoderDriveBack_TwoEnc(1.0,-iterationTurnSize,iterationTurnSize, STOP_CONDITION.STOP_ON_FRONT_MOTORS,1.5);
                setLineState();
                if(lineState == lineState_LEFT_FRONT) {
                    encoderDriveFront_TwoEnc_Short(1.0, iterationBackSize, iterationBackSize, FRONT_TWO_ENCODER_SHORT_RIGHT_SPEED_COMPENSATION_FRONT_STRAIGHT_THREE, STOP_CONDITION.STOP_ON_FRONT_MOTORS, 0.38);
                    setLineState();
                }
                count++;
            }
            //count = 0;
            while (distanceSensorState1 < 0.09 && distanceSensorState2 < 0.14 && distanceSensorState3 < 0.12) {
                encoderDriveFront_TwoEnc_Short(1.0, 2.0, 2.0, FRONT_TWO_ENCODER_SHORT_RIGHT_SPEED_COMPENSATION_FRONT_STRAIGHT_THREE, STOP_CONDITION.STOP_ON_FRONT_MOTORS, 0.50);
                encoderDriveBack_TwoEnc(1.0,0.9*1.5*0.1875*(iterationTurnSize),0.9*1.5*0.1875*(-iterationTurnSize), STOP_CONDITION.STOP_ON_FRONT_MOTORS,0.50); //orig 0.125

                //setDistanceSensorState();

                for(int i=0; i<10; i++) {
                    //updateDistanceSensorState();
                }
            /*count++;
            if(count == 6) {
                encoderDriveBack_TwoEnc(TURN_SPEED,iterationTurnSize,-iterationTurnSize,STOP_CONDITION.STOP_ON_ANY_FRONT,1.5);
                count = 0;
            }*/
                //setLineState();

            /*if(lineState != lineState_LEFT) {
                encoderDriveBack_TwoEnc(TURN_SPEED,2*(iterationTurnSize),2*(-iterationTurnSize),STOP_CONDITION.STOP_ON_ANY_FRONT,1.5);
            }*/
            }
            //robot.buttonPusher.setPosition(0.6);
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
                //robot.leftBackMotor.setPower(speed);
                robot.leftFrontMotor.setPower(speed1);
                //robot.rightBackMotor.setPower(speed2);
                robot.rightFrontMotor.setPower(speed3);

                while (opModeIsActive() && (runtime.seconds() < timeoutS)) {
                    //robot.leftBackMotor.setPower(speed);
                    robot.leftFrontMotor.setPower(speed1);
                    //robot.rightBackMotor.setPower(speed2);
                    robot.rightFrontMotor.setPower(speed3);
                }

                // Allow time for other processes to run.
                idle();

                // Stop all motion;
                //robot.leftBackMotor.setPower(0);
                robot.leftFrontMotor.setPower(0);
                //robot.rightBackMotor.setPower(0);
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

                //DbgLog.msg(String.format("Psionics_Autonomous : Flicker starting at %7d ", currentFlickerPosition));

                robot.flickerMotor.setTargetPosition(newFlickerTarget);

                // Turn On RUN_TO_POSITION
                robot.flickerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();

                //set robot's speed
                robot.flickerMotor.setPower(Math.abs(speed));

                //DbgLog.msg(String.format("Psionics_Autonomous : Flicker running to %7d ", newFlickerTarget));

                // keep looping while we are still active, and there is time left, and both motors are running.
                while (opModeIsActive() && (runtime.seconds() < timeoutS) && robot.flickerMotor.isBusy()) {
                    // Allow time for other processes to run.
                    idle();
                }

                // Stop all motion;
                robot.flickerMotor.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.flickerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            /*sleep(30);

            currentFlickerPosition = robot.flickerMotor.getCurrentPosition();*/

                //DbgLog.msg(String.format("Psionics_Autonomous : Flicker finally at %7d", currentFlickerPosition));

                //sleep(2000);   // optional pause after each move
            }
        }

        public void waitForNextFrameReady() throws InterruptedException {
            //DbgLog.msg("Psionics_Autonomous : Waiting for next camera frame");
            FtcRobotControllerActivity.myPsionics_Vision_OpenCV.setNextFrameReady(false);
            while (!(FtcRobotControllerActivity.myPsionics_Vision_OpenCV.getNextFrameReady())) {
                idle();
            }
            //DbgLog.msg("Psionics_Autonomous : Next camera frame ready");
        }

    /*public void encoderDrive(double speed,
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
    }*/

    /*public void encoderDriveBack(double speed, double leftFrontInches, double rightFrontInches, double timeoutS) throws InterruptedException {
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
    }*/

        public void encoderDriveBack_TwoEnc(double speed, double leftFrontInches, double rightFrontInches, STOP_CONDITION stop_condition, double timeoutS) throws InterruptedException {
            int newLeftFrontTarget;
            int newRightFrontTarget;
            boolean leftFrontMotorBusy;
            boolean rightFrontMotorBusy;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {
                int currentLeftFrontPosition = robot.leftFrontMotor.getCurrentPosition();
                int currentRightFrontPosition = robot.rightFrontMotor.getCurrentPosition();

                // Determine new target position, and pass to motor controller
                newLeftFrontTarget = currentLeftFrontPosition + (int) (leftFrontInches * BACK_TWO_ENCODER_DISTANCE_COMPENSATION_FRONT_LEFT * COUNTS_PER_INCH_FRONT);
                newRightFrontTarget = currentRightFrontPosition + (int) (rightFrontInches * BACK_TWO_ENCODER_DISTANCE_COMPENSATION_FRONT_RIGHT * COUNTS_PER_INCH_FRONT);

                //DbgLog.msg(String.format("Psionics_Autonomous : Starting at %7d :%7d", currentLeftFrontPosition, currentRightFrontPosition));

                robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
                robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);

                // Turn On RUN_TO_POSITION
                robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();

                //set robot's speed
                robot.leftFrontMotor.setPower(Math.abs(speed) * BACK_TWO_ENCODER_LEFT_SPEED_COMPENSATION_FRONT);
                robot.rightFrontMotor.setPower(Math.abs(speed) * BACK_TWO_ENCODER_RIGHT_SPEED_COMPENSATION_FRONT);

                //DbgLog.msg(String.format("Psionics_Autonomous : Running to %7d :%7d ", newLeftFrontTarget, newRightFrontTarget));

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
                            robot.leftFrontMotor.setPower(0);
                        } else if (!rightFrontMotorBusy) {
                            robot.rightFrontMotor.setPower(0);
                        }
                        // Allow time for other processes to run.
                        idle();
                    }
                    while (opModeIsActive() && (runtime.seconds() < timeoutS) && (leftFrontMotorBusy || rightFrontMotorBusy));
                }

                // Stop all motion;
                robot.rightFrontMotor.setPower(0);
                robot.leftFrontMotor.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            /*
            sleep(30);

            currentLeftFrontPosition = robot.leftFrontMotor.getCurrentPosition();
            currentRightFrontPosition = robot.rightFrontMotor.getCurrentPosition();
            //currentLeftBackPosition = robot.leftBackMotor.getCurrentPosition();
            //currentRightBackPosition = robot.rightBackMotor.getCurrentPosition();

            DbgLog.msg(String.format("Psionics_Autonomous : Finally at %7d :%7d", currentLeftFrontPosition, currentRightFrontPosition));
            */

            /*cumulativeAddlRightRotation_Front += (currentLeftFrontPosition - newLeftFrontTarget) - (currentRightFrontPosition - newRightFrontTarget);
            DbgLog.msg(String.format("Psionics_Autonomous : Cumulative additional right rotation front : " + cumulativeAddlRightRotation_Front));

            if ((Math.abs(cumulativeAddlRightRotation_Front) >= BACK_COMPENSATION_THRESHOLD_FRONT)) {
                compTargetFront = cumulativeAddlRightRotation_Front;
                cumulativeAddlRightRotation_Front = 0.0;
                encoderDriveBack_TwoEnc(TURN_SPEED, (-compTargetFront / (2 * COUNTS_PER_INCH_FRONT)), (compTargetFront / (2 * COUNTS_PER_INCH_FRONT)), 2.0);
            }*/
            }
        }

        public void encoderDriveBack_TwoEnc_Turns(double speed, double leftFrontInches, double rightFrontInches, STOP_CONDITION stop_condition, double timeoutS) throws InterruptedException {
            int newLeftFrontTarget;
            int newRightFrontTarget;
            boolean leftFrontMotorBusy;
            boolean rightFrontMotorBusy;
            double compTargetFront;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {
                int currentLeftFrontPosition = robot.leftFrontMotor.getCurrentPosition();
                int currentRightFrontPosition = robot.rightFrontMotor.getCurrentPosition();

                // Determine new target position, and pass to motor controller
                newLeftFrontTarget = currentLeftFrontPosition + (int) (leftFrontInches * BACK_TWO_ENCODER_DISTANCE_COMPENSATION_FRONT_LEFT * COUNTS_PER_INCH_FRONT);
                newRightFrontTarget = currentRightFrontPosition + (int) (rightFrontInches * BACK_TWO_ENCODER_DISTANCE_COMPENSATION_FRONT_RIGHT * COUNTS_PER_INCH_FRONT);

                //DbgLog.msg(String.format("Psionics_Autonomous : Starting at %7d :%7d", currentLeftFrontPosition, currentRightFrontPosition));

                robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
                robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);

                // Turn On RUN_TO_POSITION
                robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();

                //set robot's speed
                robot.leftFrontMotor.setPower(Math.abs(speed) * BACK_TWO_ENCODER_LEFT_SPEED_COMPENSATION_FRONT);
                robot.rightFrontMotor.setPower(Math.abs(speed) * BACK_TWO_ENCODER_RIGHT_SPEED_COMPENSATION_FRONT);

                //DbgLog.msg(String.format("Psionics_Autonomous : Running to %7d :%7d ", newLeftFrontTarget, newRightFrontTarget));

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
                            robot.leftFrontMotor.setPower(0);
                        } else if (!rightFrontMotorBusy) {
                            robot.rightFrontMotor.setPower(0);
                        }
                        // Allow time for other processes to run.
                        idle();
                    }
                    while (opModeIsActive() && (runtime.seconds() < timeoutS) && (leftFrontMotorBusy || rightFrontMotorBusy));
                }

                // Stop all motion;
                robot.rightFrontMotor.setPower(0);
                robot.leftFrontMotor.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                sleep(30);

                currentLeftFrontPosition = robot.leftFrontMotor.getCurrentPosition();
                currentRightFrontPosition = robot.rightFrontMotor.getCurrentPosition();
                //currentLeftBackPosition = robot.leftBackMotor.getCurrentPosition();
                //currentRightBackPosition = robot.rightBackMotor.getCurrentPosition();

                //DbgLog.msg(String.format("Psionics_Autonomous : Finally at %7d :%7d", currentLeftFrontPosition, currentRightFrontPosition));


                cumulativeAddlRightRotation_Front += (currentLeftFrontPosition - newLeftFrontTarget) - (currentRightFrontPosition - newRightFrontTarget);
                DbgLog.msg(String.format("Psionics_Autonomous : Cumulative additional right rotation front : " + cumulativeAddlRightRotation_Front));

                if ((Math.abs(cumulativeAddlRightRotation_Front) >= BACK_COMPENSATION_THRESHOLD_FRONT)) {
                    compTargetFront = cumulativeAddlRightRotation_Front;
                    cumulativeAddlRightRotation_Front = 0.0;
                    encoderDriveBack_TwoEnc(TURN_SPEED, (-compTargetFront / (2 * COUNTS_PER_INCH_FRONT)), (compTargetFront / (2 * COUNTS_PER_INCH_FRONT)), STOP_CONDITION.STOP_ON_ANY_FRONT,2.0);
                }
            }
        }

        public void encoderDriveBack_TwoEnc_Two_Line_Target(double speed, double leftFrontInches, double rightFrontInches, int lineTarget1, int lineTarget2, STOP_CONDITION stop_condition, double timeoutS) throws InterruptedException {
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
                //int currentLeftBackPosition = robot.leftBackMotor.getCurrentPosition();
                //int currentRightBackPosition = robot.rightBackMotor.getCurrentPosition();

                if (leftFrontInches >= 0) leftDirection = 1.0;
                else leftDirection = -1.0;
                if (rightFrontInches >= 0) rightDirection = 1.0;
                else rightDirection = -1.0;

                // Determine new target position, and pass to motor controller
                newLeftFrontTarget = currentLeftFrontPosition + (int) (leftFrontInches * BACK_TWO_ENCODER_DISTANCE_COMPENSATION_FRONT_LEFT * COUNTS_PER_INCH_FRONT);
                newRightFrontTarget = currentRightFrontPosition + (int) (rightFrontInches * BACK_TWO_ENCODER_DISTANCE_COMPENSATION_FRONT_RIGHT * COUNTS_PER_INCH_FRONT);

                //DbgLog.msg(String.format("Psionics_Autonomous : Starting at %7d :%7d :%7d :%7d", currentLeftFrontPosition, currentRightFrontPosition, currentLeftBackPosition, currentRightBackPosition));

                robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
                robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);

                // Turn On RUN_TO_POSITION
                robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                // reset the timeout time and start motion.
                runtime.reset();

                //set robot's speed
                robot.leftFrontMotor.setPower(Math.abs(speed) * BACK_TWO_ENCODER_LEFT_SPEED_COMPENSATION_FRONT);
                robot.rightFrontMotor.setPower(Math.abs(speed) * BACK_TWO_ENCODER_RIGHT_SPEED_COMPENSATION_FRONT);
                //robot.leftBackMotor.setPower(leftDirection * Math.abs(speed));
                //robot.rightBackMotor.setPower(rightDirection * Math.abs(speed));

                //DbgLog.msg(String.format("Psionics_Autonomous : Running to %7d :%7d ", newLeftFrontTarget, newRightFrontTarget));
                setLineStateSide();
                if(stop_condition == STOP_CONDITION.STOP_ON_ANY_FRONT) {
                    // keep looping while we are still active, and there is time left, and both front motors are running.
                    while (opModeIsActive() &&
                            (runtime.seconds() < timeoutS) &&
                            ((robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy()))
                            && lineState != lineTarget1 && lineState != lineTarget2) {
                        // Allow time for other processes to run.
                        setLineStateSide();
                        idle();
                    }
                }
                else if(stop_condition == STOP_CONDITION.STOP_ON_FRONT_MOTORS) {
                    // keep looping while we are still active, and there is time left, and either front motors are running.
                    do {
                        leftFrontMotorBusy = robot.leftFrontMotor.isBusy();
                        rightFrontMotorBusy = robot.rightFrontMotor.isBusy();
                        if (!leftFrontMotorBusy) {
                            //robot.leftBackMotor.setPower(0);
                            robot.leftFrontMotor.setPower(0);
                        } else if (!rightFrontMotorBusy) {
                            //robot.rightBackMotor.setPower(0);
                            robot.rightFrontMotor.setPower(0);
                        }
                        // Allow time for other processes to run.
                        setLineStateSide();
                        idle();
                    }
                    while (opModeIsActive() && (runtime.seconds() < timeoutS) && (leftFrontMotorBusy || rightFrontMotorBusy)
                            && lineState != lineTarget1 && lineState != lineTarget2);
                }

                // Stop all motion;
                //robot.rightBackMotor.setPower(0);
                robot.rightFrontMotor.setPower(0);
                //robot.leftBackMotor.setPower(0);
                robot.leftFrontMotor.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            /*sleep(30);

            currentLeftFrontPosition = robot.leftFrontMotor.getCurrentPosition();
            currentRightFrontPosition = robot.rightFrontMotor.getCurrentPosition();
            currentLeftBackPosition = robot.leftBackMotor.getCurrentPosition();
            currentRightBackPosition = robot.rightBackMotor.getCurrentPosition();*/

                //DbgLog.msg(String.format("Psionics_Autonomous : Finally at %7d :%7d :%7d :%7d", currentLeftFrontPosition, currentRightFrontPosition, currentLeftBackPosition, currentRightBackPosition));

            /*cumulativeAddlRightRotation_Front += (currentLeftFrontPosition - newLeftFrontTarget) - (currentRightFrontPosition - newRightFrontTarget);
            DbgLog.msg(String.format("Psionics_Autonomous : Cumulative additional right rotation front : " + cumulativeAddlRightRotation_Front));

            if ((Math.abs(cumulativeAddlRightRotation_Front) >= BACK_COMPENSATION_THRESHOLD_FRONT)) {
                compTargetFront = cumulativeAddlRightRotation_Front;
                cumulativeAddlRightRotation_Front = 0.0;
                encoderDriveBack_TwoEnc(TURN_SPEED, (-compTargetFront / (2 * COUNTS_PER_INCH_FRONT)), (compTargetFront / (2 * COUNTS_PER_INCH_FRONT)), 2.0);
            }*/
            }
        }

        public void encoderDriveBack_TwoEnc_Three_Line_Target(double speed, double leftFrontInches, double rightFrontInches, int lineTarget1, int lineTarget2, int lineTarget3, STOP_CONDITION stop_condition, double timeoutS) throws InterruptedException {
            int newLeftFrontTarget;
            int newRightFrontTarget;
            boolean leftFrontMotorBusy;
            boolean rightFrontMotorBusy;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {
                int currentLeftFrontPosition = robot.leftFrontMotor.getCurrentPosition();
                int currentRightFrontPosition = robot.rightFrontMotor.getCurrentPosition();
                //int currentLeftBackPosition = robot.leftBackMotor.getCurrentPosition();
                //int currentRightBackPosition = robot.rightBackMotor.getCurrentPosition();

                // Determine new target position, and pass to motor controller
                newLeftFrontTarget = currentLeftFrontPosition + (int) (leftFrontInches * BACK_TWO_ENCODER_DISTANCE_COMPENSATION_FRONT_LEFT * COUNTS_PER_INCH_FRONT);
                newRightFrontTarget = currentRightFrontPosition + (int) (rightFrontInches * BACK_TWO_ENCODER_DISTANCE_COMPENSATION_FRONT_RIGHT * COUNTS_PER_INCH_FRONT);

                //DbgLog.msg(String.format("Psionics_Autonomous : Starting at %7d :%7d :%7d :%7d", currentLeftFrontPosition, currentRightFrontPosition, currentLeftBackPosition, currentRightBackPosition));

                robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
                robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);

                // Turn On RUN_TO_POSITION
                robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                // reset the timeout time and start motion.
                runtime.reset();

                //set robot's speed
                robot.leftFrontMotor.setPower(Math.abs(speed) * BACK_TWO_ENCODER_LEFT_SPEED_COMPENSATION_FRONT);
                robot.rightFrontMotor.setPower(Math.abs(speed) * BACK_TWO_ENCODER_RIGHT_SPEED_COMPENSATION_FRONT);
                //robot.leftBackMotor.setPower(leftDirection * Math.abs(speed));
                //robot.rightBackMotor.setPower(rightDirection * Math.abs(speed));

                //DbgLog.msg(String.format("Psionics_Autonomous : Running to %7d :%7d ", newLeftFrontTarget, newRightFrontTarget));
                setLineStateSide();
                if(stop_condition == STOP_CONDITION.STOP_ON_ANY_FRONT) {
                    // keep looping while we are still active, and there is time left, and both front motors are running.
                    while (opModeIsActive() &&
                            (runtime.seconds() < timeoutS) &&
                            ((robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy()))
                            && lineState != lineTarget1 && lineState != lineTarget2 && lineState != lineTarget3) {
                        // Allow time for other processes to run.
                        setLineStateSide();
                        idle();
                    }
                }
                else if(stop_condition == STOP_CONDITION.STOP_ON_FRONT_MOTORS) {
                    // keep looping while we are still active, and there is time left, and either front motors are running.
                    do {
                        leftFrontMotorBusy = robot.leftFrontMotor.isBusy();
                        rightFrontMotorBusy = robot.rightFrontMotor.isBusy();
                        if (!leftFrontMotorBusy) {
                            //robot.leftBackMotor.setPower(0);
                            robot.leftFrontMotor.setPower(0);
                        } else if (!rightFrontMotorBusy) {
                            //robot.rightBackMotor.setPower(0);
                            robot.rightFrontMotor.setPower(0);
                        }
                        // Allow time for other processes to run.
                        setLineStateSide();
                        idle();
                    }
                    while (opModeIsActive() && (runtime.seconds() < timeoutS) && (leftFrontMotorBusy || rightFrontMotorBusy)
                            && lineState != lineTarget1 && lineState != lineTarget2 && lineState != lineTarget3);
                }

                // Stop all motion;
                //robot.rightBackMotor.setPower(0);
                robot.rightFrontMotor.setPower(0);
                //robot.leftBackMotor.setPower(0);
                robot.leftFrontMotor.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            /*sleep(30);

            currentLeftFrontPosition = robot.leftFrontMotor.getCurrentPosition();
            currentRightFrontPosition = robot.rightFrontMotor.getCurrentPosition();
            currentLeftBackPosition = robot.leftBackMotor.getCurrentPosition();
            currentRightBackPosition = robot.rightBackMotor.getCurrentPosition();*/

                //DbgLog.msg(String.format("Psionics_Autonomous : Finally at %7d :%7d :%7d :%7d", currentLeftFrontPosition, currentRightFrontPosition, currentLeftBackPosition, currentRightBackPosition));

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
                //int currentLeftBackPosition = robot.leftBackMotor.getCurrentPosition();
                //int currentRightBackPosition = robot.rightBackMotor.getCurrentPosition();

                if (leftFrontInches >= 0) leftDirection = 1.0;
                else leftDirection = -1.0;
                if (rightFrontInches >= 0) rightDirection = 1.0;
                else rightDirection = -1.0;

                // Determine new target position, and pass to motor controller
                newLeftFrontTarget = currentLeftFrontPosition + (int) (leftFrontInches * FRONT_TWO_ENCODER_SHORT_DISTANCE_COMPENSATION_FRONT_LEFT * COUNTS_PER_INCH_FRONT);
                newRightFrontTarget = currentRightFrontPosition + (int) (rightFrontInches * FRONT_TWO_ENCODER_SHORT_DISTANCE_COMPENSATION_FRONT_RIGHT * COUNTS_PER_INCH_FRONT);

                //DbgLog.msg(String.format("Psionics_Autonomous : Starting at %7d :%7d :%7d :%7d", currentLeftFrontPosition, currentRightFrontPosition, currentLeftBackPosition, currentRightBackPosition));

                robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
                robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);

                // Turn On RUN_TO_POSITION
                robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                // reset the timeout time and start motion.
                runtime.reset();

                //set robot's speed
                robot.leftFrontMotor.setPower(Math.abs(speed) * FRONT_TWO_ENCODER_SHORT_LEFT_SPEED_COMPENSATION_FRONT);
                robot.rightFrontMotor.setPower(Math.abs(speed) * FRONT_TWO_ENCODER_SHORT_RIGHT_SPEED_COMPENSATION_FRONT * rightSpeedCompensation);
                //robot.leftBackMotor.setPower(0);
                //robot.rightBackMotor.setPower(0);

                //robot.leftBackMotor.setPower(leftDirection * Math.abs(speed) * FRONT_TWO_ENCODER_SHORT_LEFT_SPEED_COMPENSATION_BACK);
                //robot.rightBackMotor.setPower(rightDirection * Math.abs(speed) * rightSpeedCompensation);

                //DbgLog.msg(String.format("Psionics_Autonomous : Running to %7d :%7d ", newLeftFrontTarget, newRightFrontTarget));

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
                            //robot.leftBackMotor.setPower(0);
                            robot.leftFrontMotor.setPower(0);
                        } else if (!rightFrontMotorBusy) {
                            //robot.rightBackMotor.setPower(0);
                            robot.rightFrontMotor.setPower(0);
                        }
                        // Allow time for other processes to run.
                        idle();
                    }
                    while (opModeIsActive() && (runtime.seconds() < timeoutS) && (leftFrontMotorBusy || rightFrontMotorBusy));
                }

                // Stop all motion;
                //robot.leftBackMotor.setPower(0);
                robot.leftFrontMotor.setPower(0);
                //robot.rightBackMotor.setPower(0);
                robot.rightFrontMotor.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            /*sleep(30);

            currentLeftFrontPosition = robot.leftFrontMotor.getCurrentPosition();
            currentRightFrontPosition = robot.rightFrontMotor.getCurrentPosition();
            currentLeftBackPosition = robot.leftBackMotor.getCurrentPosition();
            currentRightBackPosition = robot.rightBackMotor.getCurrentPosition();*/

                //DbgLog.msg(String.format("Psionics_Autonomous : Finally at %7d :%7d :%7d :%7d", currentLeftFrontPosition, currentRightFrontPosition, currentLeftBackPosition, currentRightBackPosition));

            /*cumulativeAddlRightRotation_Front += (currentLeftFrontPosition - newLeftFrontTarget) - (currentRightFrontPosition - newRightFrontTarget);
            DbgLog.msg(String.format("Psionics_Autonomous : Cumulative additional right rotation front : " + cumulativeAddlRightRotation_Front));

            if ((Math.abs(cumulativeAddlRightRotation_Front) >= BACK_COMPENSATION_THRESHOLD_FRONT)) {
                compTargetFront = cumulativeAddlRightRotation_Front;
                cumulativeAddlRightRotation_Front = 0.0;
                encoderDriveBack_TwoEnc(TURN_SPEED, (-compTargetFront / (2 * COUNTS_PER_INCH_FRONT)), (compTargetFront / (2 * COUNTS_PER_INCH_FRONT)), 2.0);
            }*/
            }
        }

    /*public void lineFollowB(double sensorValueL, double sensorValueR) throws InterruptedException {
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
    }*/

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

    /*public void setColorLF() {
        sensorValueLF = robot.odsLF_bottom.getRawLightDetected();
        if (sensorValueLF >= sensor_Threshold_LF) {
            sensorDecisionLF = 1.0;
        } else {
            sensorDecisionLF = 0.0;
        }
    }*/

    /*public void setColorLB() {
        sensorValueLB = robot.odsLB_bottom.getRawLightDetected();
        if (sensorValueLB >= sensor_Threshold_LB) {
            sensorDecisionLB = 1.0;
        } else {
            sensorDecisionLB = 0.0;
        }
    }*/

    /*public void setColorRF() {
        sensorValueRF = robot.odsRF_bottom.getRawLightDetected();
        if (sensorValueRF >= sensor_Threshold_RF) {
            sensorDecisionRF = 1.0;
        } else {
            sensorDecisionRF = 0.0;
        }
    }*/

    /*public void setColorRB() {
        sensorValueRB = robot.odsRB_bottom.getRawLightDetected();
        if (sensorValueRB >= sensor_Threshold_RB) {
            sensorDecisionRB = 1.0;
        } else {
            sensorDecisionRB = 0.0;
        }
    }*/

        public void setColorLeft() {
            sensorValueLeft = robot.bottomLeft.getRawLightDetected();
            if(sensorValueLeft >= sensor_Threshold_Left) {
                sensorDecisionLeft = 1.0;
            }
            else {
                sensorDecisionLeft = 0.0;
            }
        }

        public void setColorRight() {
            sensorValueRight = robot.bottomRight.getRawLightDetected();
            if(sensorValueRight >= sensor_Threshold_Right) {
                sensorDecisionRight = 1.0;
            }
            else {
                sensorDecisionRight = 0.0;
            }
        }

        public double getColorLeft() {
            double color = 0.0;
            if(sensorValueLeft >= sensor_Threshold_Left) {
                color = 1.0;
            }

            return color;
        }

        public double getColorRight() {
            double color = 0.0;
            if(sensorValueRight >= sensor_Threshold_Right) {
                color = 1.0;
            }

            return color;
        }


        public void getBottomSensorStatus() {
        /*telemetry.addData("Left Front Bottom Raw Value : " + robot.odsLF_bottom.getRawLightDetected(), "Sensor value : " + getColorLF(robot.odsLF_bottom.getRawLightDetected()));
        telemetry.addData("Left Back Bottom Raw Value : " + robot.odsLB_bottom.getRawLightDetected(), "Sensor value : " + getColorLB(robot.odsLB_bottom.getRawLightDetected()));
        telemetry.addData("Right Front Bottom Raw Value : " + robot.odsRF_bottom.getRawLightDetected(), "Sensor value : " + getColorRF(robot.odsRF_bottom.getRawLightDetected()));
        telemetry.addData("Right Back Bottom Raw Value : " + robot.odsRB_bottom.getRawLightDetected(), "Sensor value : " + getColorRB(robot.odsRB_bottom.getRawLightDetected()));*/

            //telemetry.addData(String.format("Left Front Raw Value : %.2f", robot.odsLF_bottom.getRawLightDetected()), "Sensor value : " + getColorLF(robot.odsLF_bottom.getRawLightDetected()));
            //telemetry.addData(String.format("Left Back Raw Value : %.2f", robot.odsLB_bottom.getRawLightDetected()), "Sensor value : " + getColorLB(robot.odsLB_bottom.getRawLightDetected()));
            //telemetry.addData(String.format("Right Front Raw Value : %.2f", robot.odsRF_bottom.getRawLightDetected()), "Sensor value : " + getColorRF(robot.odsRF_bottom.getRawLightDetected()));
            //telemetry.addData(String.format("Right Back Raw Value : %.2f", robot.odsRB_bottom.getRawLightDetected()), "Sensor value : " + getColorRB(robot.odsRB_bottom.getRawLightDetected()));
            //telemetry.update();
        }

        public void setLineState() {
            //setColorLF();
            //setColorLB();
            //setColorRF();
            //setColorRB();

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

        public void setLineStateSide() {
            setColorLeft();
            setColorRight();

            switch((int) (sensorDecisionLeft + sensorDecisionRight)) {
                case 1:
                    if(sensorDecisionLeft == 1.0) {
                        lineState = lineState_LEFT_SENSOR;
                        break;
                    }
                    if(sensorDecisionRight == 1.0) {
                        lineState = lineState_RIGHT_SENSOR;
                        break;
                    }
                case 2:
                    lineState = lineState_LEFT_AND_RIGHT_SENSORS;
                    break;
                case 0:
                default:
                    lineState = lineState_NONE;
                    break;
            }
        }
    }


