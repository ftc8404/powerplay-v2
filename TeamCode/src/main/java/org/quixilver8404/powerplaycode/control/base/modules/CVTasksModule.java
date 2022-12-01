package org.quixilver8404.powerplaycode.control.base.modules;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.videoio.VideoCapture;
import org.opencv.imgcodecs.Imgcodecs;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;
import org.quixilver8404.powerplaycode.control.base.HardwareCollection;
import org.quixilver8404.powerplaycode.control.base.Robot;

import java.util.Arrays;
import java.util.concurrent.TimeUnit;

public class CVTasksModule extends OpenCvPipeline {
    // Reading the next video frame from the camera
    //variables
    double[] currentPixel;
    int topCornerX = 10;
    int topCornerY = 10;
    int length = 300;
    int height = 300;
    public int redCounter = 0;
    public int blueCounter = 0;
    public int greenCounter = 0;
    int blueThreshold = 90;
    int redThreshold = 100;
    int greenThreshold = 100;
    double[] red = new double[] {0.0, 0.0, 255.0, 0.0};

    HardwareCollection hardwareCollection;

    public CVTasksModule(HardwareCollection hwMap) {
        hardwareCollection = hwMap;
    }

    public synchronized int getVariant() {
        if (redCounter > blueCounter && redCounter > greenCounter) {
            return 1;
        } else if (greenCounter > redCounter && greenCounter > blueCounter) {
            return 2;
        } else {
            return 3;
        }
    }

    @Override
    public Mat processFrame(Mat input) {

        for (int i = topCornerX; i < topCornerX + length; i++){

            //showing which part of imgage cv is looking at by bordering it with red
            input.put(topCornerY - 1, i - 1, red);
            input.put(topCornerY + height + 1, i + 1, red);

            //looping through pixels to see what color each one is
            for (int j = topCornerY; j < topCornerY + height; j++){
                currentPixel = input.get(j, i);
                if (currentPixel[0] > blueThreshold && currentPixel[1] < currentPixel[0] * 0.8 && currentPixel[2] < currentPixel[0] * 0.8) { blueCounter++; }
                else if (currentPixel[0] < currentPixel[2] * 0.5 && currentPixel[1] < currentPixel[2] * 0.5 && currentPixel[2] > redThreshold) { redCounter++; }
                else if (currentPixel[0] < currentPixel[1] * 0.5 && currentPixel[1] > greenThreshold && currentPixel[2] < currentPixel[1] * 0.5) { greenCounter++; }
            }
        }
        return input;
    }
}
//class SamplePipeline extends OpenCvPipeline {
//
//    Mat YCrCb = new Mat();
//    Mat Y = new Mat();
//    int avg;
//
//
//    /*
//     * This function takes the RGB frame, converts to YCrCb,
//     * and extracts the Y channel to the 'Y' variable
//     */
//    void inputToY(Mat input) {
//        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
//        ArrayList<Mat> yCrCbChannels = new ArrayList<Mat>(3);
//        Core.split(YCrCb, yCrCbChannels);
//        Y = yCrCbChannels.get(0);
//
//    }
//
//    @Override
//    public void init(Mat firstFrame) {
//        inputToY(firstFrame);
//    }
//
//    @Override
//    public Mat processFrame(Mat input) {
//        inputToY(input);
//        System.out.println("processing requested");
//        avg = (int) Core.mean(Y).val[0];
//        YCrCb.release(); // don't leak memory!
//        Y.release(); // don't leak memory!
//        return input;
//    }
//
//    public int getAnalysis() {
//        return avg;
//    }
//}