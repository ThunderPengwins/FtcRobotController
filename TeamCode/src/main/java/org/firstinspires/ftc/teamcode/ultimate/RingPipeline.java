package org.firstinspires.ftc.teamcode.ultimate;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class RingPipeline extends OpenCvPipeline {
    /*
     * Our working image buffers
     */
    Mat orangeMat = new Mat();//blue mat
    Mat thresholdMat = new Mat();
    Mat morphedThreshold = new Mat();
    Mat contoursOnPlainImageMat = new Mat();
    //
    /*
     * Threshold values
     */
    static final int CB_CHAN_MASK_THRESHOLD = 80;
    static final double DENSITY_UPRIGHT_THRESHOLD = 0.03;
    //
    /*
     * The elements we use for noise reduction
     */
    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));
    //
    /*
     * Colors
     */
    static final Scalar WHITE = new Scalar(255, 255, 255);
    static final Scalar TEAL = new Scalar(3, 148, 252);
    static final Scalar PURPLE = new Scalar(158, 52, 235);
    static final Scalar RED = new Scalar(255, 0, 0);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar ORANGE = new Scalar(255, 204, 0);//less saturated: (255, 220, 82), even less: (255, 230, 130)
    //
    static final int CONTOUR_LINE_THICKNESS = 2;
    static final int CB_CHAN_IDX = 2;
    //
    static class AnalyzedStone {
        double area;
        int position;
        Rect bound;
        double height;
        double width;
    }
    //
    ArrayList<AnalyzedStone> internalStoneList = new ArrayList<>();
    volatile ArrayList<AnalyzedStone> clientStoneList = new ArrayList<>();
    //
    /*
     * Some stuff to handle returning our various buffers
     */
    enum Stage {
        FINAL,
        Cb,
        MASK,
        MASK_NR,
        CONTOURS;
    }
    //
    Stage[] stages = Stage.values();
    //
    // Keep track of what stage the viewport is showing
    int stageNum = 0;
    //
    @Override
    public void onViewportTapped() {
        /*
         * Note that this method is invoked from the UI thread
         * so whatever we do here, we must do quickly.
         */

        int nextStageNum = stageNum + 1;
        //
        if(nextStageNum >= stages.length)
        {
            nextStageNum = 0;
        }
        //
        stageNum = nextStageNum;
    }
    //
    @Override//updated in loop
    public Mat processFrame(Mat input) {//constantly called
        // We'll be updating this with new data below
        internalStoneList.clear();
        //
        /*
         * Run the image processing
         */
        for(MatOfPoint contour : findContours(input)) {
            analyzeContour(contour, input);
        }
        //
        clientStoneList = new ArrayList<>(internalStoneList);
        //
        /*
         * Decide which buffer to send to the viewport
         */
        switch (stages[stageNum]) {
            case Cb: {
                return orangeMat;
            }
            //
            case FINAL: {
                return input;
            }
            //
            case MASK: {
                return thresholdMat;
            }
            //
            case MASK_NR: {
                return morphedThreshold;
            }
            //
            case CONTOURS: {
                return contoursOnPlainImageMat;
            }
        }
        //
        return input;
    }
    //
    public ArrayList<AnalyzedStone> getDetectedStones() {
        return clientStoneList;
    }
    //
    ArrayList<MatOfPoint> findContours(Mat input) {//called in update loop
        // A list we'll be using to store the contours we find
        ArrayList<MatOfPoint> contoursList = new ArrayList<>();
        //
        // Convert the input image to YCrCb color space, then extract the Cb channel (luma, red, and blue, luma is light sun)
        Imgproc.cvtColor(input, orangeMat, Imgproc.COLOR_RGB2YCrCb);
        //Core.extractChannel(cbMat, cbMat, CB_CHAN_IDX);
        //cbMat = cbMat.colRange(ORANGE, WHITE);
        Core.inRange(input, new Scalar(0,0,0), new Scalar(255,255,255), orangeMat);
        //Core.invert(orangeMat, orangeMat);
        //
        // Threshold the Cb channel to form a mask, then run some noise reduction
        Imgproc.threshold(orangeMat, thresholdMat, CB_CHAN_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY_INV);
        morphMask(thresholdMat, morphedThreshold);
        //
        // Ok, now actually look for the contours! We only look for external contours.
        Imgproc.findContours(morphedThreshold, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
        //
        // We do draw the contours we find, but not to the main input buffer.
        input.copyTo(contoursOnPlainImageMat);
        Imgproc.drawContours(contoursOnPlainImageMat, contoursList, -1, BLUE, CONTOUR_LINE_THICKNESS, 8);
        //
        return contoursList;
    }
    //
    void morphMask(Mat input, Mat output) {//called during find contours
        /*
         * Apply some erosion and dilation for noise reduction
         */
        //
        Imgproc.erode(input, output, erodeElement);
        Imgproc.erode(output, output, erodeElement);
        //
        Imgproc.dilate(output, output, dilateElement);
        Imgproc.dilate(output, output, dilateElement);
    }
    //
    void analyzeContour(MatOfPoint contour, Mat input) {//called from update loop
        //
        AnalyzedStone analyzedStone = new AnalyzedStone();
        analyzedStone.area = Imgproc.contourArea(contour);
        int position = 0;
        for(AnalyzedStone analStone : internalStoneList){//find position based on area (scored by biggest)
            if(analStone.area < analyzedStone.area){
                break;
            }
            position++;
        }
        analyzedStone.position = position;
        Rect bound = Imgproc.boundingRect(contour);
        analyzedStone.bound = bound;
        analyzedStone.height = bound.height;
        analyzedStone.width = bound.width;
        internalStoneList.add(position, analyzedStone);//add by area position
    }

    static class ContourRegionAnalysis {
        /*
         * This class holds the results of analyzeContourRegion()
         */
        //
        double hullArea;
        double contourArea;
        double density;
        List<MatOfPoint> listHolderOfMatOfPoint;
    }
    //
    static ContourRegionAnalysis analyzeContourRegion(ArrayList<Point> contourPoints) {
        // drawContours() requires a LIST of contours (there's no singular drawContour()
        // method), so we have to make a list, even though we're only going to use a single
        // position in it...
        MatOfPoint matOfPoint = new MatOfPoint();
        matOfPoint.fromList(contourPoints);
        List<MatOfPoint> listHolderOfMatOfPoint = Arrays.asList(matOfPoint);

        // Compute the convex hull of the contour
        MatOfInt hullMatOfInt = new MatOfInt();
        Imgproc.convexHull(matOfPoint, hullMatOfInt);

        // Was the convex hull calculation successful?
        if(hullMatOfInt.toArray().length > 0)
        {
            // The convex hull calculation tells us the INDEX of the points which
            // which were passed in eariler which form the convex hull. That's all
            // well and good, but now we need filter out that original list to find
            // the actual POINTS which form the convex hull
            Point[] hullPoints = new Point[hullMatOfInt.rows()];
            List<Integer> hullContourIdxList = hullMatOfInt.toList();

            for (int i = 0; i < hullContourIdxList.size(); i++)
            {
                hullPoints[i] = contourPoints.get(hullContourIdxList.get(i));
            }

            ContourRegionAnalysis analysis = new ContourRegionAnalysis();
            analysis.listHolderOfMatOfPoint = listHolderOfMatOfPoint;

            // Compute the hull area
            analysis.hullArea = Imgproc.contourArea(new MatOfPoint(hullPoints));

            // Compute the original contour area
            analysis.contourArea = Imgproc.contourArea(listHolderOfMatOfPoint.get(0));

            // Compute the contour density. This is the ratio of the contour area to the
            // area of the convex hull formed by the contour
            analysis.density = analysis.contourArea / analysis.hullArea;

            return analysis;
        }
        else
        {
            return null;
        }
    }
    //
    static Point computeDisplacementForSecondPointOfStoneOrientationLine(RotatedRect rect, double unambiguousAngle) {
        // Note: we return a point, but really it's not a point in space, we're
        // simply using it to hold X & Y displacement values from the middle point
        // of the bounding rect.
        Point point = new Point();

        // Figure out the length of the short side of the rect
        double shortSideLen = Math.min(rect.size.width, rect.size.height);

        // We draw a line that's 3/4 of the length of the short side of the rect
        double lineLength = shortSideLen * .75;

        // The line is to be drawn at 90 deg relative to the midline running through
        // the rect lengthwise
        point.x = (int) (lineLength * Math.cos(Math.toRadians(unambiguousAngle+90)));
        point.y = (int) (lineLength * Math.sin(Math.toRadians(unambiguousAngle+90)));

        return point;
    }
    //
    static void drawTagText(RotatedRect rect, String text, Mat mat) {
        Imgproc.putText(
                mat, // The buffer we're drawing on
                text, // The text we're drawing
                new Point( // The anchor point for the text
                        rect.center.x-50,  // x anchor point
                        rect.center.y+25), // y anchor point
                Imgproc.FONT_HERSHEY_PLAIN, // Font
                1, // Font size
                TEAL, // Font color
                1); // Font thickness
    }
}
