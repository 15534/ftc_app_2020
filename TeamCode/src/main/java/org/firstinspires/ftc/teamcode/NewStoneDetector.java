package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class NewStoneDetector extends OpenCvPipeline {
    private Mat workingMat = new Mat();
    public double frontTotal, middleTotal, backTotal;
    private boolean isRed;
    public int position = 0;

    public NewStoneDetector(boolean isRed) {
        this.isRed = isRed;
    }

    @Override
    public final Mat processFrame(Mat input) {
//        size = input.size();
        input.copyTo(workingMat);

        if(workingMat.empty()){
            return input;
        }

        Imgproc.cvtColor(workingMat, workingMat, Imgproc.COLOR_RGB2YCrCb);

        if (isRed) {
            // red
            Mat matRight = workingMat.submat(120, 150, 10, 50);
            Mat matMiddle = workingMat.submat(120, 150, 80, 120);
            Mat matLeft = workingMat.submat(120, 150, 150, 190);

            frontTotal = Core.sumElems(matRight).val[2];
            middleTotal = Core.sumElems(matMiddle).val[2];
            backTotal = Core.sumElems(matLeft).val[2];

            Imgproc.rectangle(workingMat, new Rect(10, 120, 40, 30), new Scalar(0,255,255));
            Imgproc.rectangle(workingMat, new Rect(80, 120, 40, 30), new Scalar(0,255,255));
            Imgproc.rectangle(workingMat, new Rect(150, 120, 40, 30), new Scalar(0,255,255));
        } else {
            Mat matRight = workingMat.submat(120, 150, 30, 70);
            Mat matMiddle = workingMat.submat(120, 150, 90, 130);
            Mat matLeft = workingMat.submat(120, 150, 150, 190);

            frontTotal = Core.sumElems(matLeft).val[2];
            middleTotal = Core.sumElems(matMiddle).val[2];
            backTotal = Core.sumElems(matRight).val[2];

            // blue
            Imgproc.rectangle(workingMat, new Rect(30, 120, 40, 30), new Scalar(0,255,255));
            Imgproc.rectangle(workingMat, new Rect(90, 120, 40, 30), new Scalar(0,255,255));
            Imgproc.rectangle(workingMat, new Rect(150, 120, 40, 30), new Scalar(0,255,255));
        }


        if (frontTotal > middleTotal) {
            if (frontTotal > backTotal) {
                position = 0;
            } else {
                position = 2;
            }
        } else {
            if (middleTotal > backTotal) {
                position = 1;
            } else {
                position = 2;
            }
        }

        return workingMat;
    }
}
