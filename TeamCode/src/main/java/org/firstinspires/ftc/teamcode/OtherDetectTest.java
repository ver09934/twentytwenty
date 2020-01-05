package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@TeleOp(name = "Detect Other Test", group = "Concept")
public class OtherDetectTest extends LinearOpMode {

    private ElapsedTime runtime;
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private double targetHeightRatio = .8;

    private static final String VUFORIA_KEY =
            "ARjw1eT/////AAABmTT9RZG8ZEFjviGE0IUocPIFdi7CCM1gAaiqVAQ1ys4VFImCtEPPtXDdL2pT3U3lejCSeEFD02QN5EWRHrSCvorLvhWonQqGVzsoJS7VMfIkRVvc81cJ9C4njQfYw/B8JrmTp8UIarY42jfMPpXU3Bjq04E8XOG6d+NizKPooRsb4zHlUheenmroS2FeusBkGwsKtWd328bj+R8Jf0AJcmNCZQuDAeJjXZvhrDH/1G11qxz6w1A4EEvlQ8mgeupdnLZkggUYOF8apgpxA7YMkZ4pCxljLWhH0GQ4UAatJWSBbO+9LHn5ShwfH3iOOBrZakD8kEL4x1zQlV2m2sTWg3okrVqmID2hotfGTPmzUOmK";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private double objectAngle;
    private double objectHeight;
    private double objectHeightRatio;
    private int skyCount;
    private double imageHeight;

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.4;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    @Override
    public void runOpMode() {

        // --- Init ---

        // Initial telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize utilities

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        if (updatedRecognitions.size() > 0) {
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel() == "Skystone") {
                                    skyCount++;
                                    objectAngle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                                    telemetry.addData("Estimated Angle", objectAngle);

                                    if (objectAngle > 0) {
                                        telemetry.addData("Direction", "Right");
                                    } else {
                                        telemetry.addData("Direction", "Left");
                                    }

                                    // TODO Motor Power to Angle

                                    imageHeight = recognition.getImageHeight();
                                    objectHeight = recognition.getHeight();
                                    objectHeightRatio = objectHeight / imageHeight;
                                    telemetry.addData("Height Ratio", objectHeightRatio);

                                    if (objectHeightRatio < targetHeightRatio - .05) {
                                        telemetry.addData("Distance", "Not close enough");
                                        // TODO FINISH
                                    }
                                }
                            }

                            // step through the list of recognitions and display boundary info.
                            int i = 0;
                            for (Recognition recognition : updatedRecognitions) {
                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                        recognition.getLeft(), recognition.getTop());
                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                        recognition.getRight(), recognition.getBottom());

                                if (recognition.getLabel() == "Skystone") {
                                    telemetry.addData("Center", Math.round(recognition.getRight() - recognition.getLeft()) + " " + Math.round(recognition.getTop() - recognition.getBottom()));
                                }
                            }

                            telemetry.update();
                        }
                    }
                }
            }

            if (tfod != null) {
                tfod.shutdown();
            }
        }

    }

}
