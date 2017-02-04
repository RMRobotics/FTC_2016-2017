package pm;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class ButtonFinder {

    private static int scale = 2;
    private static int dilate_amount = 2;
    private static int ellipse_match_radius = 9;
    private static int contour_match_radius = 5;
    private static int MAX_ELLIPSE = 150;
    private static int MIN_ELLIPSE = 25;


    public static ArrayList<EllipseLocationResult> locateEllipses(Mat origMat, Mat searchAreaG, Mat searchAreaG0, double X, double D, double H, Rect searchRect) {
        Imgproc.GaussianBlur(searchAreaG, searchAreaG, new Size(25,25), 0);

        Mat grgb = new Mat();
        int code = origMat.type() == CvType.CV_8UC4 ? code = Imgproc.COLOR_GRAY2RGBA : Imgproc.COLOR_GRAY2RGB;
        Imgproc.cvtColor(searchAreaG, grgb, code);
        Point offset = searchRect.tl();
        grgb.copyTo(origMat.submat((int)offset.y, grgb.rows() + (int)offset.y,
                (int)offset.x, grgb.cols() + (int)offset.x));

//        Imgproc.Canny(gray, gray, 20, 90);
        Imgproc.Canny(searchAreaG, searchAreaG, 20, 60);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT,
                new Size(2 * dilate_amount + 1, 2 * dilate_amount + 1),
                new Point(dilate_amount, dilate_amount));
        Imgproc.dilate(searchAreaG, searchAreaG, kernel);

        Imgproc.cvtColor(searchAreaG, grgb, code);
        int xd1 = (int)offset.x + grgb.cols();
        if(xd1 > origMat.width()) xd1 = origMat.width();
        int xd2 = (int)offset.x + 2*grgb.cols();
        if(xd2 > origMat.width()) xd2 = origMat.width();
        grgb.copyTo(origMat.submat((int)offset.y, grgb.rows() + (int)offset.y, xd1, xd2));

        Mat cacheHierarchy = new Mat();

        ArrayList<MatOfPoint> origContours = new ArrayList<>();
//        ArrayList<MatOfPoint> selContours = new ArrayList<>();
        ArrayList<EllipseLocationResult> selEllipses = new ArrayList<>();
        ArrayList<EllipseLocationResult> noOverlapEllipses = new ArrayList<>();

        //Find contours - the parameters here are very important to compression and retention
        Imgproc.findContours(searchAreaG, origContours, cacheHierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_TC89_KCOS);

        //Find ellipses by finding fit
        Mat compMat;
        int i = 0;
        for (MatOfPoint co : origContours) {
            //Contour must have at least 6 points for fitEllipse
            if (co.toArray().length < 6)
                continue;
            //Copy MatOfPoint to MatOfPoint2f
            MatOfPoint2f matOfPoint2f = new MatOfPoint2f(co.toArray());
            //Fit an ellipse to the current contour

            RotatedRect el = Imgproc.fitEllipse(matOfPoint2f);

            double r = Math.min(el.size.height, el.size.width)/Math.max(el.size.height, el.size.width);
            // Should be round enough
            if(r < 0.7) continue;
            // Match expected size
            if((el.size.height + el.size.width)/2 > MAX_ELLIPSE || (el.size.height + el.size.width)/2 < MIN_ELLIPSE) continue;

            double clen = Imgproc.arcLength(matOfPoint2f, true);
            double elen = Math.PI * Math.sqrt(0.5*(el.size.height*el.size.height + el.size.width*el.size.width));

            Rect contBoundRect = Imgproc.boundingRect(co);
            Rect ellipBoundRect = el.boundingRect();

            Point tl = contBoundRect.tl();
            if(tl.x > ellipBoundRect.tl().x) tl.x = ellipBoundRect.tl().x;
            if(tl.y > ellipBoundRect.tl().y) tl.y = ellipBoundRect.tl().y;
            if(tl.x < 0) tl.x = 0;
            if(tl.y < 0) tl.y = 0;
            Point br = contBoundRect.br();
            if(br.x < ellipBoundRect.br().x) br.x = ellipBoundRect.br().x;
            if(br.y < ellipBoundRect.br().y) br.y = ellipBoundRect.br().y;
            if(br.y >= searchAreaG.height()) br.y = searchAreaG.height() - 1;
            if(br.x >= searchAreaG.width()) br.x = searchAreaG.width() - 1;

            double compMatWidth = br.x - tl.x;
            double compMatHeight = br.y - tl.y;
            if(compMatHeight < 0 || compMatWidth < 0){
                continue;
            }

            compMat = new Mat((int)compMatHeight, (int)compMatWidth, CvType.CV_8UC1, new Scalar(0,0,0));

            RotatedRect shiftedEl = el.clone();
            shiftedEl.center.x = shiftedEl.center.x - tl.x;
            shiftedEl.center.y = shiftedEl.center.y - tl.y;

            // Draw fuzzy ellipse
            for(int j = ellipse_match_radius; j >= 0; j--) {
                Imgproc.ellipse(compMat, shiftedEl, new Scalar(255 - j, 255 - j, 255 - j), 2*j + 1);
            }

            // Calculate contour distance from ellipse
            List<Point> opoints = co.toList();
            double error = 0;
            for (Point pnt : opoints)
            {
                // First shift point into comparison mat space
                double x = pnt.x - tl.x;
                double y = pnt.y - tl.y;

                //check if point is inside mat
                if(x >= compMatWidth || y >= compMatHeight) continue;

                // Now get distance - further away from ellipse point lies, the lower intensity
                // of "fuzzy" ellipse it hits
                double e = 255 - compMat.get((int)Math.round(y), (int)Math.round(x))[0];
                if(((int)e) == 255) e = 255 - ellipse_match_radius -2;
                error += e;
            }
            error = error/opoints.size();
            if(error > 3) continue;

            // Calculate visible portion of ellipse covered by contour
            compMat = new Mat((int)compMatHeight, (int)compMatWidth, CvType.CV_8UC4, new Scalar(0,0,0));
            Imgproc.ellipse(compMat, shiftedEl, new Scalar(255, 255, 255), 1);

            int visEl = 0;
            int notCoveredEl = 0;
            byte buff[] = new byte[(int)compMat.total() * compMat.channels()];
            compMat.get(0, 0, buff);
            for(int j = 0; j < buff.length; j++)
            {
                if(buff[j] != 0) visEl++;
            }

            List<MatOfPoint> t = new ArrayList<>();
            t.add(co);
            Imgproc.drawContours(compMat, t, 0, new Scalar(0,0,0), contour_match_radius, 8, new Mat(), 0, new  Point(-tl.x, -tl.y));

//            compMat.copyTo(TestActivity.mDisp.submat((int)tl.y, (int)br.y, (int)tl.x, (int)br.x));

            compMat.get(0, 0, buff);
            for(int j = 0; j < buff.length; j++)
            {
                if(buff[j] != 0) notCoveredEl++;
            }

            double contDiff = ((double)notCoveredEl)/(double)visEl;
            if(contDiff > 0.7) continue;

            // Calculate brightness

            double[] intensity = IRUtils.CalcBrightness(searchAreaG0, el);
            // Surroundings should be brighter
            if(intensity[0] > intensity[1]) continue;

            selEllipses.add(new EllipseLocationResult(el, intensity[0], intensity[1]));
//            selContours.add(co);

            i++;
        }

        // Remove overlapping ellipses
        if(selEllipses.size() > 1) {
            Comparator<EllipseLocationResult> comparator = new Comparator<EllipseLocationResult>() {
                public int compare(EllipseLocationResult r1, EllipseLocationResult r2) {
                    long r1h = Math.round(r1.ellipse.size.height);
                    long r1w = Math.round(r1.ellipse.size.width);
                    long r2h = Math.round(r2.ellipse.size.height);
                    long r2w = Math.round(r2.ellipse.size.width);
                    if (Math.max(r1h, r1w) > Math.max(r2h, r2w)) return -1;
                    if (Math.max(r1h, r1w) < Math.max(r2h, r2w)) return 1;
                    return Math.min(r1h, r1w) > Math.min(r2h, r2w) ? -1 : 1;
                }
            };
            // Sort biggest first
            Collections.sort(selEllipses, comparator);

            compMat = new Mat(searchAreaG.rows() + MAX_ELLIPSE*4, searchAreaG.cols() + MAX_ELLIPSE*4, CvType.CV_8UC1, new Scalar(0, 0, 0));

            // First is the biggest, add it
            noOverlapEllipses.add(selEllipses.get(0));
            Imgproc.ellipse(compMat, ShiftEllipse(selEllipses.get(0).ellipse, MAX_ELLIPSE*2, MAX_ELLIPSE*2), new Scalar(255, 255, 255), -1);

            for (int j = 1; j < selEllipses.size(); j++) {
                if(compMat.get((int)selEllipses.get(j).ellipse.center.y + MAX_ELLIPSE*2 - 1, (int)selEllipses.get(j).ellipse.center.x + MAX_ELLIPSE*2 - 1)[0] == 0) {
                    noOverlapEllipses.add(selEllipses.get(j));
                    Imgproc.ellipse(compMat, ShiftEllipse(selEllipses.get(j).ellipse, MAX_ELLIPSE*2, MAX_ELLIPSE*2), new Scalar(255, 255, 255), -1);
                }
            }
        }
        else {
            if(selEllipses.size() == 1) noOverlapEllipses.add(selEllipses.get(0));
        }

        // Draw contours and ellipses
/*
        for(int j = 0; j < selContours.size(); j++) {
            Imgproc.drawContours(TestActivity.mDisp, selContours, j, new Scalar(170, 170, 255), 5);
        }

        for(RotatedRect el : noOverlapEllipses) {
            Imgproc.ellipse(TestActivity.mDisp, el, new Scalar(255, 20, 20), 2);
        }
*/
        for(EllipseLocationResult el : noOverlapEllipses) {
            el.Ddiff = (el.ellipse.size.height + el.ellipse.size.width)/2 - D;
            el.Xdiff = el.ellipse.center.x - X;
        }

        Comparator<EllipseLocationResult> comparator = new Comparator<EllipseLocationResult>() {
            public int compare(EllipseLocationResult r1, EllipseLocationResult r2) {
                double diff1 = Math.abs(r1.Xdiff) + Math.abs(r1.Ddiff) * 2;// + Math.abs(r1.Hdiff) * 5;
                double diff2 = Math.abs(r2.Xdiff) + Math.abs(r2.Ddiff) * 2;// + Math.abs(r2.Hdiff) * 5;

                if(diff1 < diff2) return -1;
                return 1;
            }
        };
        // Sort biggest first
        Collections.sort(noOverlapEllipses, comparator);

        return noOverlapEllipses;
    }

    private static RotatedRect ShiftEllipse(RotatedRect orig, double x, double y)
    {
        return new RotatedRect(
                new Point(orig.center.x + x, orig.center.y + y),
                orig.size,
                orig.angle);
    }

    static Scalar GetBackground(Mat img, RotatedRect button, Size box, double minD) {
        Point tl = new Point(button.center.x - box.width / 2, button.center.y - box.height / 2);
        Point br = new Point(button.center.x + box.width / 2, button.center.y + box.height / 2);
        if (tl.x < 0) tl.x = 0;
        if (tl.y < 0) tl.y = 0;
        if (br.x + 1 > img.width()) br.x = img.width() - 1;
        if (br.y + 1 > img.height()) br.y = img.height() - 1;

//        Imgproc.circle(img, new Point(button.center.x, button.center.y), (int) minD / 2, new Scalar(0, 0, 0), -1);
//        Imgproc.rectangle(img, tl, br, new Scalar(150, 150, 255), 1);

//        Mat ColorBox = img.submat((int) tl.y, (int) (br.y), (int) tl.x, (int) (br.x));
        return getColor(img, new Rect(tl, br));

/*
        double r = 0;
        double g = 0;
        double b = 0;
        int n = 0;
        for (int i = 0; i < ColorBox.rows(); i++)
            for (int j = 0; j < ColorBox.cols(); j++) {
                double[] p = ColorBox.get(i, j);
                if (p[0] > 1 && p[1] > 1 && p[2] > 1) {
                    r += p[0];
                    g += p[1];
                    b += p[2];
                    n++;
                }
            }
        r /= n;
        g /= n;
        b /= n;
        return new Scalar(r, g, b);
        */
    }

    static public Scalar getColor(Mat img, Rect area) {
        //Selects subregion defined by 'area'
        Mat selectedRegion = img.submat(area);

        //converts to HSV Color Format
        Mat convertedRegion = new Mat();
        Imgproc.cvtColor(selectedRegion, convertedRegion, Imgproc.COLOR_RGB2HSV_FULL);

        //sums rgb values of each pixel
        Scalar colorTotals = Core.sumElems(convertedRegion);
        int numPixels = area.width * area.height;
        for (int i = 0; i < colorTotals.val.length; i++) {
            //divide each element of colorTotals scalar by total number of pixels
            colorTotals.val[i] /= numPixels;
        }
        //convert back to RGBA format
//        colorTotals = converScalarHsv2Rgba(colorTotals);

        return colorTotals;
    }


    public static class EllipseLocationResult {
        public RotatedRect ellipse;
        public Scalar background;
        public double Xdiff = 0;
        public double Ddiff = 0;
        public double Hdiff = 0;
        public double bInten = 0;
        public double sInten = 0;
        public boolean isRight = false;
        public double beaconCenter = 0;

        EllipseLocationResult(RotatedRect ellipse, double bi, double si) {
            this.ellipse = ellipse;
            this.bInten = bi;
            this.sInten = si;
        }
    }
}
