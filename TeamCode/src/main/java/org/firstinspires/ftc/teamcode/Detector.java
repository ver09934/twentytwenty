package org.firstinspires.ftc.robotcontroller.external.samples;

import com.google.gson.internal.$Gson$Preconditions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.*;
import org.opencv.dnn.Dnn;
import org.opencv.dnn.Net;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.TreeMap;

public class Detector extends OpenCvPipeline {

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private Mat result = new Mat();
    private List<Mat> recognitions = new ArrayList<>();

    private int position;
    private Telemetry telemetry;
    private boolean showUI;


    Detector(Telemetry telemetry, boolean showUI) {
        super();
        this.telemetry = telemetry;
        this.showUI = showUI;
    }

    public int getPosition() { return position; }

    @Override
    public Mat processFrame(Mat input) {

        telemetry.addLine("---Detection Algorithm Begins---");
        System.out.println("---Detection Algorithm Begins---");

        Size imageSize = input.size();
        double w = imageSize.width;
        double h = imageSize.height;

        Net model = Dnn.readNetFromTensorflow;

        model.setInput(input);
        result = model.forward();

        return null;
    }
}
