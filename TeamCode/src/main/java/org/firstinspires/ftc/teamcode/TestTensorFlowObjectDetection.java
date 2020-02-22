/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 * <p>
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@TeleOp(name = "Skystone Detect Test", group = "Concept")
// @Disabled
public class TestTensorFlowObjectDetection extends LinearOpMode {

    private ElapsedTime runtime;
    //private DriverFunction driverFunction;
    //private DriverFunction.Steering steering;

    public static final double MAX_COAST_SECONDS = 6;

    public static final double NORMAL_SPEED_RATIO = 0.3;
    public static final double MEDIUM_SPEED_RATIO = 0.5;
    public static final double FAST_SPEED_RATIO = 0.7;
    public int skyPos;

    public static final long MOVE_DELAY_MS = 50;
    public static final long LONG_DELAY_MS = 300;

    private static final long CRATER_ADVANCE_MS = 1100;
    private static final long CRATER_RETREAT_MS = 850;

    private static final long DEPOT_TURN_MS = 3000;
    private static final long MARKER_DROP_DELAY_MS = 800;

    private static final int CV_ITERATIONS = 5;
    private static final long CV_LOOP_DELAY = 200;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";


    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "ARjw1eT/////AAABmTT9RZG8ZEFjviGE0IUocPIFdi7CCM1gAaiqVAQ1ys4VFImCtEPPtXDdL2pT3U3lejCSeEFD02QN5EWRHrSCvorLvhWonQqGVzsoJS7VMfIkRVvc81cJ9C4njQfYw/B8JrmTp8UIarY42jfMPpXU3Bjq04E8XOG6d+NizKPooRsb4zHlUheenmroS2FeusBkGwsKtWd328bj+R8Jf0AJcmNCZQuDAeJjXZvhrDH/1G11qxz6w1A4EEvlQ8mgeupdnLZkggUYOF8apgpxA7YMkZ4pCxljLWhH0GQ4UAatJWSBbO+9LHn5ShwfH3iOOBrZakD8kEL4x1zQlV2m2sTWg3okrVqmID2hotfGTPmzUOmK";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

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

                        ArrayList<Pair> sort = new ArrayList<Pair>();

                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            i++;
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());


                            if (recognition.getLabel() == "Skystone") {
                                telemetry.addData("Center", Math.round(recognition.getRight() - recognition.getLeft()) + " " + Math.round(recognition.getTop() - recognition.getBottom()));
                                sort.add(new Pair("Skystone", recognition.getLeft()));
                            }
                            if (recognition.getLabel() == "Stone") {
                                sort.add(new Pair("Stone", recognition.getLeft()));
                            }

                            Collections.sort(sort);
                            try {
                                if (sort.get(0).getString().equals("Skystone")) {
                                    skyPos = 1;
                                } else if (sort.get(1).getString().equals("Skystone")) {
                                    skyPos = 2;
                                } else if (sort.get(2).getString().equals("Skystone")) {
                                    skyPos = 3;
                                } else {
                                    skyPos = 5321;
                                }
                            } catch (IndexOutOfBoundsException e) {
                                
                            }

                            telemetry.addData("Skystone Position: ", skyPos);

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


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

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
}

class Pair implements Comparable<Pair>{
    private Float afloat;
    private String string;

    public Pair(String str, Float flt) {
        string = str;
        afloat = flt;
    }

    public Float getFloat() {
        return afloat;
    }

    public String getString() {
        return string;
    }
    public String toString() {
        return string + ", " + afloat;
    }

    @Override
    public int compareTo(Pair x) {
        return (int) (this.afloat - x.getFloat());
    }

}

