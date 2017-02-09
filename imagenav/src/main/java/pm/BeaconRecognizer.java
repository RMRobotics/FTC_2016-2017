package pm;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

/**
 * Created by Peter on 1/19/2017.
 */

public class BeaconRecognizer {
    Mat mGray0; // Only adjusted for contrast and brightness
    Mat mGray; // Enhanced for button recognition
    Mat mRgbaF;
    public static Mat mDisp; // Display output
    Mat mRgbaT; // Transposed for output
    double prevBtnX = 0;
    double prevBtnY = 0;
    double prevBtnD = 0;
    double prevBtnH = 0;
    double contrast = 1;
    double brightness = 0;
    int expBtnDiam = 40;
//    Rect searchArea = new Rect(0, 0, 160, 640);
    Rect searchArea = new Rect(0, 250, 160, 500);
    int expBtnOffset = 80; // Offset from the top of search rectangle
    Size btnSurround = new Size(100, 110); // Box to check color around button
    Mat buttonMask = null;
    Mat surrMask = null;

    public BeaconRecognizer() {
        mGray0 = new Mat();
        mGray = new Mat();
        mRgbaF = new Mat();
        mDisp = new Mat();

        mRgbaT = new Mat();
    }

    public ButtonFinder.EllipseLocationResult detectButtons(Mat img, boolean getRed) {
        Rect trackArea = new Rect (
                searchArea.x,
                searchArea.y,
                searchArea.width,
                searchArea.height);
        // Set display to original
        mDisp = img;

        ButtonFinder.EllipseLocationResult left = null;
        ButtonFinder.EllipseLocationResult right = null;

        // Target area
        Mat trackMat = img.submat(trackArea);
        // Preserve original
        Mat origMat = trackMat.clone();

        // Convert it to gray
        Imgproc.cvtColor(trackMat, mGray, Imgproc.COLOR_BGR2GRAY);
        mGray0 = mGray.clone();

        mGray.convertTo(mGray, -1, contrast, brightness);
/*
            switch(retr) {
                case 1: mGray.convertTo(mGray, -1, 1, 30); break;
                case 2: mGray.convertTo(mGray, -1, 1, 60); break;
                case 3: mGray.convertTo(mGray, -1, 1, -30); break;
                case 4: mGray.convertTo(mGray, -1, 1, -60); break;
            }*/

        // Select correct ellipses
        double bD = trackArea.width * expBtnDiam / searchArea.width;
        double bX = trackArea.height * expBtnOffset / searchArea.height;
        ArrayList<ButtonFinder.EllipseLocationResult> res = ButtonFinder.locateEllipses(mDisp, mGray, mGray0, bX, bD, 0.0, trackArea);
        // Result is sorted arbitrary

        ArrayList<ButtonFinder.EllipseLocationResult[]> pairs = new ArrayList<ButtonFinder.EllipseLocationResult[]>();
        for (int i = 0; i < res.size(); i++) {
            if (res.get(i).ellipse.center.y > mGray.rows() / 2) {
                // For all circles from the left side, match best circle from the right side
                ButtonFinder.EllipseLocationResult[] p = new ButtonFinder.EllipseLocationResult[2];
                p[0] = res.get(i);
                p[1] = null;
                double error = 1000;
                for (int j = 0; j < res.size(); j++) {
                    if (res.get(j).ellipse.center.y < mGray.rows() / 2) {
                        double e = Math.abs((res.get(i).Xdiff - res.get(j).Xdiff) / bD * 1.2);
                        e += Math.abs((res.get(i).Ddiff - res.get(j).Ddiff) / bD);
                        if (e < error) {
                            p[1] = res.get(j);
                            error = e;
                        }
                    }
                }
                if (p[1] != null) pairs.add(p);
            }
        }

        // Draw expected button centers
        Imgproc.line(trackMat, new Point(bX, 0), new Point(bX, trackMat.rows() - 1), new Scalar(20, 255, 20), 2);

        Imgproc.rectangle(trackMat, new Point(0,0), new Point(trackMat.width(), trackMat.height()), new Scalar(20, 255, 20), 2);
        if (pairs.size() == 0) {
            return null;
        } else {
            double error = 1000;
            for (ButtonFinder.EllipseLocationResult[] p : pairs) {
                double e = Math.abs((p[0].Xdiff - p[1].Xdiff) / bD * 1.2);
                e += Math.abs((p[0].Ddiff - p[1].Ddiff) / bD);
                if (e < error) {
                    left = p[0];
                    right = p[1];
                    error = e;
                }
            }
        }

        boolean redIsLeft = RedOnTheLeft(origMat, left, right);

        // Draw buttons for debugging
        if (redIsLeft) {
            Imgproc.ellipse(trackMat, left.ellipse, new Scalar(255, 20, 20), -1);
            Imgproc.ellipse(trackMat, right.ellipse, new Scalar(20, 20, 255), -1);
        } else {
            Imgproc.ellipse(trackMat, right.ellipse, new Scalar(255, 20, 20), -1);
            Imgproc.ellipse(trackMat, left.ellipse, new Scalar(20, 20, 255), -1);
        }

        ButtonFinder.EllipseLocationResult btn = (getRed ^ redIsLeft) ? right : left;
        btn.isRight = (getRed ^ redIsLeft);
        btn.beaconCenter = (left.ellipse.center.y + right.ellipse.center.y)/2 + trackArea.y;

        brightness = -(200*btn.bInten - 20*btn.sInten)/(btn.sInten - btn.bInten);
        contrast = (20 - brightness)/btn.bInten;

        // Adjust btn coordinates
        btn.ellipse.center.x += trackArea.x;
        btn.ellipse.center.y += trackArea.y;

//          prevBtnH = origBtn.background.val[0];
        prevBtnX = btn.ellipse.center.x;
        prevBtnY = btn.ellipse.center.y;
        prevBtnD = (btn.ellipse.size.height + btn.ellipse.size.width) / 2;

        return btn;

        // Nothing found, do something !!!! Change Canny parameters, move, etc.
    }


    public RotatedRect TrackButton(Mat img) {

        if(prevBtnD < 0.1) return null;

        Rect trackArea = new Rect((int)prevBtnX - (int)prevBtnD * 2, (int)prevBtnY - (int)prevBtnD * 5, (int)prevBtnD * 4, (int)prevBtnD * 10);
        trackArea = IRUtils.intersect(new Rect(0,0,img.width(),img.height()), trackArea);

        // Set display to original
        mDisp = img;

        // Target area
        Mat trackMat = img.submat(trackArea);

        // Convert it to gray
        Imgproc.cvtColor(trackMat, mGray, Imgproc.COLOR_BGR2GRAY);
        mGray0 = mGray.clone();

        mGray.convertTo(mGray, -1, contrast, brightness);

        // Select correct ellipses
        ArrayList<ButtonFinder.EllipseLocationResult> res = ButtonFinder.locateEllipses(mDisp, mGray, mGray0, prevBtnX, prevBtnD, prevBtnH, trackArea);
        // Result is sorted from closest to furthest

        // Pick closest one
        for (ButtonFinder.EllipseLocationResult b : res) {
            if(Math.abs(b.Hdiff)  < 15) {
                if(Math.abs(b.Ddiff/prevBtnD) > 0.5) continue;

                // Adjust btn coordinates
                b.ellipse.center.x += trackArea.x;
                b.ellipse.center.y += trackArea.y;

                double dist = (prevBtnX - b.ellipse.center.x)*(prevBtnX - b.ellipse.center.x) +
                        (prevBtnY - b.ellipse.center.y)*(prevBtnY - b.ellipse.center.y);
                dist = Math.sqrt(dist);
                if(dist > trackArea.size().height/4) continue;

                prevBtnX = b.ellipse.center.x;
                prevBtnY = b.ellipse.center.y;
                prevBtnD = (b.ellipse.size.height + b.ellipse.size.width) / 2;

                brightness = -(200*b.bInten - 20*b.sInten)/(b.sInten - b.bInten);
                contrast = (20 - brightness)/b.bInten;
//                prevBtnH = b.background.val[0];

                Rect focus = b.ellipse.boundingRect();
                Imgproc.rectangle(img, focus.tl(), focus.br(), new Scalar(255, 255, 20), 2);

                return b.ellipse;
            }
        }

        return null;
    }

    public boolean RedOnTheLeft(Mat mRgba, ButtonFinder.EllipseLocationResult left, ButtonFinder.EllipseLocationResult right){
        Rect trackArea = new Rect(0,0,mRgba.width(),mRgba.height());

        Rect leftArea = left.ellipse.boundingRect();
        leftArea = IRUtils.intersect(trackArea, leftArea);
        Scalar leftAreaColor = ButtonFinder.getColor(mRgba, leftArea);

        Rect rightArea = right.ellipse.boundingRect();
        rightArea = IRUtils.intersect(trackArea, rightArea);
        Scalar rightAreaColor = ButtonFinder.getColor(mRgba, rightArea);
        Scalar leftDispColor;
        Scalar rightDispColor;

//        Imgproc.rectangle(mRgba, leftArea.br(), leftArea.tl(), new Scalar(255, 255, 255), -1);
//        Imgproc.rectangle(mRgba, rightArea.br(), rightArea.tl(), new Scalar(255, 255, 255), -1);

        if(leftAreaColor.val[0] > rightAreaColor.val[0]) {
            return true;
        } else {
            return false;
        }
    }
}
