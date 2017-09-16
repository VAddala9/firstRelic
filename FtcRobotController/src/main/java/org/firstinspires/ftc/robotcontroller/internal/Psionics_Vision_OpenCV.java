package org.firstinspires.ftc.robotcontroller.internal;

import android.util.Log;
import android.view.SurfaceView;
import android.widget.LinearLayout;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * Created by geeth on 11/6/2016.
 */

public class Psionics_Vision_OpenCV implements CameraBridgeViewBase.CvCameraViewListener2 {
    private final int dimBy = -150;
    private final int frameWidth = 288;
    private final int frameHeight = 384;

    private Mat mRgba;
    private Mat workingFrame;
    private Scalar mRightScreenColor;
    private Scalar mCenterScreenColor;
    private Scalar mLeftScreenColor;
    private Scalar dim = new Scalar(dimBy,dimBy,dimBy,0);
    ColorBlobDetector mDetector = new ColorBlobDetector();
    private boolean workingFrameCloned = false;
    private boolean findBeaconColor = false;
    private boolean locateBeacon = false;
    private boolean doOpenCVProcessing = false;
    private boolean redMode = false;
    private boolean blueMode = false;
    public static final int numColorSearchWindows = 12;
    public static Mat[] poleSplit = new Mat[numColorSearchWindows];
    public static Mat[] poleSplitRgb = new Mat[numColorSearchWindows];
    public static Scalar[] poleSplitColorRGB = new Scalar[numColorSearchWindows];
    public static Scalar[] poleSplitColorHSV = new Scalar[numColorSearchWindows];
    public static Mat[] polesplitcolorLabels = new Mat[numColorSearchWindows];
    private boolean nextOpenCvFrameReady = false;
    public Psionics_Vision_OpenCV(CameraBridgeViewBase c) {
        c.setVisibility(SurfaceView.VISIBLE);
        c.setCvCameraViewListener(this);
    }

    @Override
    public void onCameraViewStarted(int width, int height) {

    }

    @Override
    public void onCameraViewStopped() {

    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        mRgba = inputFrame.rgba();
        if(doOpenCVProcessing) {
            if(workingFrameCloned == false) {
                workingFrame = mRgba.clone();
                workingFrameCloned = true;
            }
            else {
                mRgba.copyTo(workingFrame);
            }
            if(blueMode) {
                for (int i = 0; i < poleSplitColorRGB.length; i++) {
                    if (locateBeacon && !findBeaconColor) { //dim
                        poleSplit[i] = workingFrame.submat((i * 24), ((i + 1) * 24), 120, 144);
                        Core.add(poleSplit[i], dim, poleSplit[i]);
                        poleSplitColorRGB[i] = mDetector.getAverageRgb(poleSplit[i]);
                        polesplitcolorLabels[i] = workingFrame.submat((i * 24), ((i + 1) * 24), 360, 384);
                        polesplitcolorLabels[i].setTo(poleSplitColorRGB[i]);
                        poleSplitRgb[i] = workingFrame.submat((i * 24), ((i * 24) + 1), 360, 361);
                        poleSplitColorHSV[i] = mDetector.getAverageHsv(poleSplitRgb[i]);
                    } else if (findBeaconColor && !locateBeacon) { //no dim
                        poleSplit[i] = workingFrame.submat((i * 24), ((i + 1) * 24), 120, 144);
                        poleSplitColorRGB[i] = mDetector.getAverageRgb(poleSplit[i]);
                        polesplitcolorLabels[i] = workingFrame.submat((i * 24), ((i + 1) * 24), 360, 384);
                        polesplitcolorLabels[i].setTo(poleSplitColorRGB[i]);
                        poleSplitRgb[i] = workingFrame.submat((i * 24), ((i * 24) + 1), 360, 361);
                        poleSplitColorHSV[i] = mDetector.getAverageHsv(poleSplitRgb[i]);
                    }
                }
            }
            else if(redMode) {
                for (int i = 0; i < poleSplitColorRGB.length; i++) {
                    if (locateBeacon && !findBeaconColor) { //dim
                        poleSplit[i] = workingFrame.submat((i * 24), ((i + 1) * 24), 264, 288);
                        Core.add(poleSplit[i], dim, poleSplit[i]);
                        poleSplitColorRGB[i] = mDetector.getAverageRgb(poleSplit[i]);
                        polesplitcolorLabels[i] = workingFrame.submat((i * 24), ((i + 1) * 24), 360, 384);
                        polesplitcolorLabels[i].setTo(poleSplitColorRGB[i]);
                        poleSplitRgb[i] = workingFrame.submat((i * 24), ((i * 24) + 1), 360, 361);
                        poleSplitColorHSV[i] = mDetector.getAverageHsv(poleSplitRgb[i]);
                    } else if (findBeaconColor && !locateBeacon) { //no dim
                        poleSplit[i] = workingFrame.submat((i * 24), ((i + 1) * 24), 264, 288);
                        poleSplitColorRGB[i] = mDetector.getAverageRgb(poleSplit[i]);
                        polesplitcolorLabels[i] = workingFrame.submat((i * 24), ((i + 1) * 24), 360, 384);
                        polesplitcolorLabels[i].setTo(poleSplitColorRGB[i]);
                        poleSplitRgb[i] = workingFrame.submat((i * 24), ((i * 24) + 1), 360, 361);
                        poleSplitColorHSV[i] = mDetector.getAverageHsv(poleSplitRgb[i]);
                    }
                }
            }
            doOpenCVProcessing = false;
        }
        nextOpenCvFrameReady = true;
        return workingFrame;
    }

    public void setFindBeaconColor(boolean v) { findBeaconColor = v; }

    public boolean getFindBeaconColor() {
        return findBeaconColor;
    }

    public void setLocateBeacon(boolean v) { locateBeacon = v; }

    public boolean getLocateBeacon() {
        return findBeaconColor;
    }

    public void setOpenCVProcessing(boolean v) { doOpenCVProcessing = v; }

    public boolean getOpenCVProcessing() {
        return doOpenCVProcessing;
    }

    public void setNextFrameReady(boolean b) {nextOpenCvFrameReady = b;}

    public boolean getNextFrameReady() {
        return nextOpenCvFrameReady;
    }

    public void setBlueMode(boolean b) { blueMode = b; }

    public boolean getBlueMode() { return blueMode; }

    public void setRedMode(boolean b) { redMode = b; }

    public boolean getRedMode() { return redMode; }


}
