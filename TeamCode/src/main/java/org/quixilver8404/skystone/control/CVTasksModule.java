package org.quixilver8404.skystone.control;

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
import org.quixilver8404.skystone.control.HardwareCollection;
import org.quixilver8404.skystone.control.BaseRobot;

import java.util.Arrays;
import java.util.concurrent.TimeUnit;

public class CVTasksModule extends OpenCvPipeline {
    // Reading the next video frame from the camera
    //variables
    double[] currentPixel;

    int topCornerX = 520;
    int topCornerY = 215;
    int length = 50;
    int height = 90;

    public int redCounter = 0;
    public int blueCounter = 0;
    public int greenCounter = 0;
    public int finalRedCounter = 0;
    public int finalBlueCounter = 0;
    public int finalGreenCounter = 0;
    int blueThreshold = 90;
    int redThreshold = 110;
    int greenThreshold = 20;
    double[] border = new double[] {255.0, 0.0, 0.0, 0.0};

    HardwareCollection hardwareCollection;

    public CVTasksModule(HardwareCollection hwMap) {
        hardwareCollection = hwMap;
    }

    public synchronized int getVariant() {
        if (finalRedCounter > finalBlueCounter && finalRedCounter > finalGreenCounter && finalRedCounter >= 1000) {
            return 1;
        } else if (finalBlueCounter > finalRedCounter && finalBlueCounter > finalGreenCounter && finalBlueCounter >= 1000) {
            return 3;
        } else {
            return 2;
        }
    }

    @Override
    public Mat processFrame(Mat input) {

        for (int i = topCornerX; i < topCornerX + length; i++){

            //showing which part of imgage cv is looking at by bordering it with red
            input.put(topCornerY - 1, i - 1, border);
            input.put(topCornerY + height + 1, i + 1, border);

            //looping through pixels to see what color each one is
            for (int j = topCornerY; j < topCornerY + height; j++){
                currentPixel = input.get(j, i);
                if (currentPixel[0] > redThreshold && currentPixel[1] < currentPixel[0] * 0.8 && currentPixel[2] < currentPixel[0] * 0.8) { redCounter++; }
                else if (currentPixel[0] < currentPixel[2] * 0.5 && currentPixel[1] < currentPixel[2] * 0.5 && currentPixel[2] > blueThreshold) { blueCounter++; }
                else if (currentPixel[0] < currentPixel[1] * 0.8 && currentPixel[1] > greenThreshold && currentPixel[2] < currentPixel[1] * 0.8) { greenCounter++; }
            }
        }
        finalRedCounter = redCounter;
        finalBlueCounter = blueCounter;
        finalGreenCounter = greenCounter;
        redCounter = 0;
        blueCounter = 0;
        greenCounter = 0;
        return input;
    }
}
