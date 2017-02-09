package org.firstinspires.ftc.rmrobotics.opmodes.AutoNav;

import android.content.Context;
import android.util.AttributeSet;
import android.view.SurfaceView;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.RotatedRect;
import org.opencv.core.Size;

import java.util.List;

import pm.BeaconRecognizer;
import pm.ButtonFinder;

/**
 * Created by Peter on 1/23/2017.
 */

interface Tracker {
    public enum State {RECOGNIZING, RECOGNIZED, TRACKING, LOST};
    public Mat Recognize(Mat m);
    public Mat Track(Mat m);
    public State GetState();
}

// OpenCV calls methods from this class to detect buttons when new frame is available
// runs on a separate thread
// We control what to do with the frame by setting trackingState
// RECOGNIZING - initial state, OpenCV passes the frame to Mat Recognize(Mat img)
// RECOGNIZED - Used to report that recognition is complete OpenCV does nothing in this state
// TRACKING - Trying to track button. BeaconRecognizer stores button position parameters in static variables
// and compares each new frame to previous
//
//
public class BeaconTracker implements Tracker {
    BeaconRecognizer br; // All OpenCV image recognition code is in this class
    private volatile RotatedRect btnPosition;
    private volatile State trackingState;

    private OpenCVVideo ocv = null;
    private boolean teamIsRed;
    private ButtonFinder.EllipseLocationResult btn = null;

    BeaconTracker(CameraBridgeViewBase view, LinearOpMode om, boolean isRed) {
        teamIsRed = isRed;
        br = new BeaconRecognizer();
        trackingState = State.RECOGNIZING;
        ocv = new OpenCVVideo(
                view,
                this,
                om
        );
    }

    synchronized public Mat Recognize(Mat img) {
        ButtonFinder.EllipseLocationResult btn0 = br.detectButtons(img, teamIsRed);
        if (btn0 != null) {
            trackingState = State.RECOGNIZED;
            btn = btn0;
        }

        return img;
    }

    synchronized public Mat Track(Mat img) {
        btnPosition = br.TrackButton(img);
        if(btnPosition == null) trackingState = State.LOST;
        else trackingState = State.TRACKING;
        return img;
    }

    synchronized  public RotatedRect getBtnPosition() {
        RotatedRect ret = btnPosition;
        btnPosition = null;
        return ret;
    }

    synchronized  public ButtonFinder.EllipseLocationResult button() {
        return btn;
    }

    synchronized public State GetState() {
        return trackingState;
    }
    synchronized public void SetState(State st) { trackingState = st; }

    public void Close() {
        ocv.Close();
    }
}


// Class for capturing OpenCV frames
class OpenCVVideo implements CameraBridgeViewBase.CvCameraViewListener2 {
    static private volatile OpenCVVideo obj;
    static private CameraBridgeViewBase openCvCameraView;

    private long mFrameNum = 0;
    private Tracker eTracker = null;
    private LinearOpMode opmode;

    OpenCVVideo(CameraBridgeViewBase cv, Tracker trc, LinearOpMode om) {
        obj = this;
        openCvCameraView = cv;
        opmode = om;
        eTracker = trc;

        ((FtcRobotControllerActivity) opmode.hardwareMap.appContext).runOnUiThread(new Runnable() {
            @Override
            public void run() {
                openCvCameraView.setVisibility(SurfaceView.VISIBLE);
                openCvCameraView.setCvCameraViewListener(obj);

                openCvCameraView.setCameraIndex(openCvCameraView.CAMERA_ID_FRONT);
                openCvCameraView.enableView();
            }
        });
    }

    public void Close() {
        ((FtcRobotControllerActivity) opmode.hardwareMap.appContext).runOnUiThread(new Runnable() {
            @Override
            public void run() {
                openCvCameraView.disableView();
            }
        });
    }

    @Override
    public void onCameraViewStarted(int width, int height) {

    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        mFrameNum++;

//        opmode.ADBLog("Got frame");

        Mat img = inputFrame.rgba();
        Core.flip(img, img, 1);
        img = img.t();

        if (mFrameNum > 3) {
            switch (eTracker.GetState()) {
                case RECOGNIZING:
                    img = eTracker.Recognize(img);
                    break;
                case RECOGNIZED:
                    break;
                case LOST:
                case TRACKING:
                    img = eTracker.Track(img);
                    break;
            }
        }

        img = img.t();
        return img;
    }

    @Override
    public void onCameraViewStopped() {

    }


}
//to make sure that the picture is the right size
class RMCameraView extends JavaCameraView {
    public RMCameraView(Context context, int cameraId) {
        super(context, cameraId);
    }

    public RMCameraView(Context context, AttributeSet attrs) {
        super(context, attrs);
    }

    /**
     * This helper method can be called by subclasses to select camera preview size.
     * It goes over the list of the supported preview sizes and selects the maximum one which
     * fits both values set via setMaxFrameSize() and surface frame allocated for this view
     *
     * @param supportedSizes
     * @param surfaceWidth
     * @param surfaceHeight
     * @return optimal frame size
     */

    @Override
    protected Size calculateCameraFrameSize(List<?> supportedSizes, ListItemAccessor accessor, int surfaceWidth, int surfaceHeight) {
        int calcWidth = 640;
        int calcHeight = 480;
/*
            int maxAllowedWidth = (mMaxWidth != MAX_UNSPECIFIED && mMaxWidth < surfaceWidth)? mMaxWidth : surfaceWidth;
            int maxAllowedHeight = (mMaxHeight != MAX_UNSPECIFIED && mMaxHeight < surfaceHeight)? mMaxHeight : surfaceHeight;

            for (Object size : supportedSizes) {
                int width = accessor.getWidth(size);
                int height = accessor.getHeight(size);

                if (width <= maxAllowedWidth && height <= maxAllowedHeight) {
                    if (width >= calcWidth && height >= calcHeight) {
                        calcWidth = (int) width;
                        calcHeight = (int) height;
                    }
                }
            }*/

        return new Size(calcWidth, calcHeight);
    }
}


