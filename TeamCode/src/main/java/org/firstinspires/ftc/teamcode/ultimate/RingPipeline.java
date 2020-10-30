package org.firstinspires.ftc.teamcode.ultimate;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class RingPipeline extends OpenCvPipeline {
    //Our working image buffers
    Mat orangeMat = new Mat();//blue mat
    Mat thresholdMat = new Mat();
    Mat morphedThreshold = new Mat();
    Mat contoursOnPlainImageMat = new Mat();
    Mat convertedMat = new Mat();
    //
    //Threshold values
    static final int CB_CHAN_MASK_THRESHOLD = 80;
    static final double DENSITY_UPRIGHT_THRESHOLD = 0.03;
    //
    //The elements we use for noise reduction
    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));
    //
    //Colors
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
    static class AnalyzedRingGr {
        double area;
        int position;
        Rect bound;
        double height;
        double width;
    }
    //
    ArrayList<AnalyzedRingGr> internalRingList = new ArrayList<>();
    volatile ArrayList<AnalyzedRingGr> clientRingList = new ArrayList<>();
    //
    //Some stuff to handle returning our various buffers
    enum Stage {
        FINAL,
        Cb,
        MASK,
        MASK_NR,
        CONTOURS,
        CONVERTED;
    }
    //
    Stage[] stages = Stage.values();
    //
    AnalyzedRingGr bestRingGr = null;
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
        internalRingList.clear();
        //
        jeremy.cvReady = true;
        //
        ArrayList<MatOfPoint> conlist = findContours(input);
        //
        for(MatOfPoint contour : conlist) {
            analyzeContour(contour, input);
        }
        //
        if (internalRingList.size() > 0) {
            Imgproc.drawContours(input, conlist, -1, BLUE, CONTOUR_LINE_THICKNESS, 8);
            Imgproc.rectangle(input, internalRingList.get(0).bound,GREEN, 10);
            bestRingGr = internalRingList.get(0);
        }else{
            bestRingGr = null;
        }
        //
        clientRingList = new ArrayList<>(internalRingList);
        //
        /*
         * Decide which buffer to send to the viewport
         */
        switch (stages[stageNum]) {
            case Cb: {
                return convertedMat;
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
            //
            case CONVERTED: {
                return orangeMat;
            }
        }
        //
        return input;
    }
    //
    public ArrayList<AnalyzedRingGr> getDetectedRings() {
        return clientRingList;
    }
    //
    ArrayList<MatOfPoint> findContours(Mat input) {//called in update loop
        // A list we'll be using to store the contours we find
        ArrayList<MatOfPoint> contoursList = new ArrayList<>();
        //convert to ycrcb and isolate yellow channel
        Imgproc.cvtColor(input, orangeMat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(orangeMat, convertedMat, CB_CHAN_IDX);
        //
        Imgproc.threshold(convertedMat, thresholdMat, CB_CHAN_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY_INV);
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
        AnalyzedRingGr analyzedRing = new AnalyzedRingGr();
        analyzedRing.area = Imgproc.contourArea(contour);
        int position = 0;
        for(AnalyzedRingGr analRing : internalRingList){//find position based on area (scored by biggest)
            if(analRing.area < analyzedRing.area){
                break;
            }
            position++;
        }
        //
        analyzedRing.position = position;
        Rect bound = Imgproc.boundingRect(contour);
        analyzedRing.bound = bound;
        analyzedRing.height = bound.height;
        analyzedRing.width = bound.width;
        Imgproc.rectangle(input, analyzedRing.bound, BLUE, 8);
        internalRingList.add(position, analyzedRing);//add by area position
    }
    //
    ArrayList<AnalyzedRingGr> getRingList(){
        return internalRingList;
    }
    //
    AnalyzedRingGr getBestRing(){
        return bestRingGr;
    }
}
