/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/**
 * In this sample, we demonstrate how to use the {@link OpenCvPipeline#onViewportTapped()}
 * callback to switch which stage of a pipeline is rendered to the viewport for debugging
 * purposes. We also show how to get data from the pipeline to your OpMode.
 */
@TeleOp
public class PipelineStageSwitchingPOWERPLAY extends LinearOpMode
{
    OpenCvWebcam webCam;
    StageSwitchingPipeline stageSwitchingPipeline;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        
        stageSwitchingPipeline = new StageSwitchingPipeline();
        
        webCam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webCam.setPipeline(stageSwitchingPipeline);
                //webCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                webCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Current Stage Shown In Viewport: ", stageSwitchingPipeline.getStage());
            telemetry.addData("Num HSV contours found", stageSwitchingPipeline.getNumHSVContoursFound());

            telemetry.update();
            sleep(100);
        }
    }

    /*
     * With this pipeline, we demonstrate how to change which stage of
     * is rendered to the viewport when the viewport is tapped. This is
     * particularly useful during pipeline development. We also show how
     * to get data from the pipeline to your OpMode.
     */
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat hsvMat = new Mat();        
        Mat thresholdHSVMat = new Mat();
        Mat contoursFromHSVOnFrameMat = new Mat(); 
        List<MatOfPoint> contoursHSVList = new ArrayList<>();
        int numContoursHSVFound = 0;

        enum Stage
        {
            HSV,
            THRESHOLD_HSV,            
            CONTOURS_FROM_HSV_OVERLAYED_ON_FRAME,            
            RAW_IMAGE
        }

        private Stage stageToRenderToViewport = Stage.HSV;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursHSVList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */
            // get hsv as image
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

            Core.inRange(hsvMat, new Scalar(0, 162, 191), new Scalar(93, 255, 255), thresholdHSVMat);
            Imgproc.findContours(thresholdHSVMat, contoursHSVList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            numContoursHSVFound = contoursHSVList.size();
            
            input.copyTo(contoursFromHSVOnFrameMat);
            Imgproc.drawContours(contoursFromHSVOnFrameMat, contoursHSVList, -1, new Scalar(0, 255, 0), 3, 8);
            
            switch (stageToRenderToViewport)
            {
                case HSV:
                {
                    return hsvMat;
                }

                case THRESHOLD_HSV:
                {
                    return thresholdHSVMat;
                }

                case CONTOURS_FROM_HSV_OVERLAYED_ON_FRAME:
                {
                    return contoursFromHSVOnFrameMat;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

        public int getNumHSVContoursFound()
        {
            return numContoursHSVFound;
        }

        public String getStage()
        {
            switch (stageToRenderToViewport)
            {
                case HSV:
                {
                    return "HSV";
                }

                case THRESHOLD_HSV:
                {
                    return "Threshhold using HSV";
                }

                case CONTOURS_FROM_HSV_OVERLAYED_ON_FRAME:
                {
                    return "Contours Found using HSV Threshold";
                }

                case RAW_IMAGE:
                {
                    return "Raw Image";
                }

                default:
                {
                    return "Unknown";
                }
            }
        }        
    }
}
