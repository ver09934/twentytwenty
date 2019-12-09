package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class Detector extends OpenCvPipeline {

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private Mat hsv = new Mat();
    private Mat thresholdedStone = new Mat();
    private Mat thresholdedSkystone = new Mat();

    private List<MatOfPoint> stoneContours = new ArrayList<>();
    private List<MatOfPoint> skyContours = new ArrayList<>();

    private static final double[] y_bounds = {.35, .65};
    private static int history = 1;

    private int position;
    private Telemetry telemetry;
    private boolean showUI;


    Detector(Telemetry telemetry, boolean showUI) {
        super();
        this.telemetry = telemetry;
        this.showUI = showUI;
    }

    public synchronized List<MatOfPoint> getStoneContours() {
        List<MatOfPoint> stoneContoursCopy = new ArrayList<>();
        for (MatOfPoint contourMat : stoneContours) {
            MatOfPoint tempMat = new MatOfPoint();
            contourMat.copyTo(tempMat);
            stoneContoursCopy.add(tempMat);
        }
        return stoneContoursCopy;
    }

    public synchronized List<MatOfPoint> getSkyContours() {
        List<MatOfPoint> skyContoursCopy = new ArrayList<>();
        for (MatOfPoint contourMat : skyContours) {
            MatOfPoint tempMat = new MatOfPoint();
            contourMat.copyTo(tempMat);
            skyContoursCopy.add(tempMat);
        }
        return skyContoursCopy;
    }

    public int getPosition() {
        return position;
    }

    @Override
    public Mat processFrame(Mat rgba) {

        telemetry.addLine("---Detection Algorithm Begins---");
        System.out.println("---Detection Algorithm Begins---");

        // Clear contour lists
        stoneContours.clear();
        skyContours.clear();

        // Change colorspace from RGBA to HSV
        Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV, 3);

        // Blur thresholded image
        Imgproc.blur(hsv, hsv, new Size(10, 10));

        // TODO get ranges of the SkyRect contours
        Core.inRange(hsv, new Scalar(SKYRANGE), new Scalar(SKYRANGE_2), thresholdedSkystone);
        Imgproc.erode(thresholdedSkystone, thresholdedSkystone, new Mat(), new Point(-1, -1), 5, Core.BORDER_CONSTANT);
        Imgproc.dilate(thresholdedSkystone, thresholdedSkystone, new Mat(), new Point(-1, -1), 5, Core.BORDER_CONSTANT);

        Imgproc.findContours(thresholdedSkystone, skyContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.fillPoly(thresholdedSkystone, skyContours, new Scalar(255, 255, 255));

        // Find the centers of the bounding rectangles
        for (int i = 0; i < skyContours.size(); i++) {
            RotatedRect skyRect = Imgproc.minAreaRect(skyContours.get(i));
            Point skyCenter = skyRect.center;
        }

        // TODO get ranges of the Rect contours
        Core.inRange(hsv, new Scalar(STONERANGE), new Scalar(STONERANGE_2), thresholdedStone);
        Imgproc.erode(thresholdedStone, thresholdedStone, new Mat(), new Point(-1, -1), 5, Core.BORDER_CONSTANT);
        Imgproc.dilate(thresholdedStone, thresholdedStone, new Mat(), new Point(-1, -1), 5, Core.BORDER_CONSTANT);

        Imgproc.findContours(thresholdedStone, stoneContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.fillPoly(thresholdedStone, stoneContours, new Scalar(255, 255, 255));

        // Find the centers of the bounding rectangles
        for (int j = 0; j < stoneContours.size(); j++) {
            RotatedRect stoneRect = Imgproc.minAreaRect(stoneContours.get(j));
            Point stoneCenter = stoneRect.center;
        }

        if (showUI) {
            Imgproc.circle(rgba, stoneCenter, 3, new Scalar(0, 255, 0));
        }


        return null;
    }
}
