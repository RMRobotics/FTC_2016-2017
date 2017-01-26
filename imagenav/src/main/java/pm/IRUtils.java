package pm;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

/**
 * Created by Peter on 1/19/2017.
 */

public class IRUtils {
    static public Rect intersect(Rect bound, Rect intern) {
        Rect r = intern.clone();
        if(r.x + r.width < bound.x) return null;
        if(r.y + r.height < bound.y) return null;
        if(r.x > bound.x + bound.width) return null;
        if(r.y > bound.y + bound.height) return null;
        if(r.x < bound.x) {
            r.width -= bound.x - r.x;
            r.x = bound.x;
        }
        if(r.y < bound.y) {
            r.height -= bound.y - r.y;
            r.y = bound.y;
        }
        if(r.x + r.width > bound.x + bound.width) r.width = bound.x + bound.width - r.x;
        if(r.y + r.height > bound.y + bound.height) r.height = bound.y + bound.height - r.y;
        return r;
    }


    static public Rect intersectAndAdjust(Rect bound, Rect intern, Rect toAdjust) {
        assert (intern.width == toAdjust.width || intern.height == toAdjust.height);
        Rect r = intersect(bound, intern);
        Point off = new Point(r.x - intern.x, r.y - intern.y);
        toAdjust.x += off.x;
        toAdjust.y += off.y;
        toAdjust.width = r.width;
        toAdjust.height = r.height;
        return r;
    }

    static Mat buttonMask = new Mat();
    static Mat surrMask = new Mat();
    static double[] CalcBrightness(Mat img, RotatedRect ellipse) {
        double bD = (ellipse.size.width + ellipse.size.height)/2;
        Size btnSurround = new Size(bD*3, bD*3);

        // Calculate brightness and contrast adjustments
        // Get button mask
        buttonMask = Mat.zeros((int)bD, (int)bD, CvType.CV_8UC1);
        Imgproc.circle(buttonMask, new Point((int)bD/2, (int)bD/2), (int)bD/2-5, new Scalar(255), -1);

        // First button intensity
        Rect btnBB = new Rect(
                (int)(ellipse.center.x - bD/2),
                (int)(ellipse.center.y - bD/2),
                (int)(bD),
                (int)(bD));
        Rect m = new Rect(0,0,buttonMask.width(), buttonMask.height());
        Rect a = IRUtils.intersectAndAdjust(new Rect(0,0,img.width(),img.height()), btnBB, m);
        Scalar bIntens = Core.mean(img.submat(a), buttonMask.submat(m));
        // Get surroundings mask
        surrMask = Mat.ones(btnSurround, CvType.CV_8UC1);
        Imgproc.circle(surrMask, new Point((int)btnSurround.width/2, (int)btnSurround.height/2), (int)bD/2+5, new Scalar(0), -1);
        // Now surroundings
        btnBB = new Rect(
                (int)(ellipse.center.x - btnSurround.width/2),
                (int)(ellipse.center.y - btnSurround.height/2),
                (int)btnSurround.width,
                (int)btnSurround.height);

        m = new Rect(0,0,surrMask.width(), surrMask.height());
        a = IRUtils.intersectAndAdjust(new Rect(0,0,img.width(),img.height()), btnBB, m);
        Scalar sIntens = Core.mean(img.submat(a), surrMask.submat(m));

        double[] r = new double[2];
        r[0] = bIntens.val[0];
        r[1] = sIntens.val[0];
        return r;
    }


}
