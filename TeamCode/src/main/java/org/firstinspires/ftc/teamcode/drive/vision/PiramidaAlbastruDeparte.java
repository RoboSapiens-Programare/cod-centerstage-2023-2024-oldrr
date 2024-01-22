package org.firstinspires.ftc.teamcode.drive.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class PiramidaAlbastruDeparte extends OpenCvPipeline {
    //Telemetry telemetry;
    Mat mat = new Mat();

    private final Scalar LOW_BLUE = new Scalar(100, 50, 50);
    private final Scalar HIGH_BLUE = new Scalar(160, 255, 255);

    //TODO de aflat valoarea minima de galben dintr-un dreptunghi, fara ratoi
    private static final double PERCENT_COLOR_THRESHOLD = 0.2;

    public enum Location {
        LEFT,
        CENTER,
        RIGHT
    }

    private PiramidaAlbastruDeparte.Location location = PiramidaAlbastruDeparte.Location.LEFT;

    //TODO de gasit punctele pentru dreptunghiuri
    static final Rect RIGHT_ROI = new Rect(
            new Point(1180, 400),
            new Point(1280, 500
            )
    );

    static final Rect CENTER_ROI = new Rect(
            new Point (500, 400),
            new Point(600, 500)
    );

    @Override
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = LOW_BLUE;
        Scalar highHSV = HIGH_BLUE;

        //dreptunghiuri regiuni
        final Scalar BLUE = new Scalar(0, 0, 255);
        Imgproc.rectangle(input, RIGHT_ROI, BLUE, 2);
//            Imgproc.rectangle(input, CENTER_ROI, BLUE, 2);
        Imgproc.rectangle(input, CENTER_ROI, BLUE, 2);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat right = mat.submat(RIGHT_ROI);
//            Mat center = mat.submat(CENTER_ROI);
        Mat center = mat.submat(CENTER_ROI);

        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;
//            double centerValue = Core.sumElems(center).val[0] / CENTER_ROI.area() / 255;
        double centerValue = Core.sumElems(center).val[0] / CENTER_ROI.area() / 255;

        right.release();
//            center.release();
        center.release();

            /*telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
            telemetry.addData("Center raw value", (int) Core.sumElems(left).val[0]);
            telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
            telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
            //telemetry.addData("Center percentage", Math.round(centerValue * 100) + "%");
            telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");*/

        boolean propRight = rightValue > PERCENT_COLOR_THRESHOLD;
//            boolean duckCenter = centerValue > PERCENT_COLOR_THRESHOLD;
        boolean propCenter = centerValue > PERCENT_COLOR_THRESHOLD;

        if(propCenter && centerValue > rightValue) {
            location = PiramidaAlbastruDeparte.Location.CENTER;
            //telemetry.addData("Duck Location", "left");
        }
//            else if(duckCenter) {
//                location = Location.CENTER;
//                //telemetry.addData("Duck Location", "center");
//            }
        else if(propRight && rightValue > centerValue) {
            location = PiramidaAlbastruDeparte.Location.RIGHT;
            //telemetry.addData("Duck Location", "right");
        }
        else if(centerValue < PERCENT_COLOR_THRESHOLD && rightValue < PERCENT_COLOR_THRESHOLD){
            location = PiramidaAlbastruDeparte.Location.LEFT;
        }
        //telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar notRata = new Scalar(255, 0, 0);
        Scalar rata = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, RIGHT_ROI, location == PiramidaAlbastruDeparte.Location.RIGHT? rata:notRata);
        Imgproc.rectangle(mat, CENTER_ROI, location == PiramidaAlbastruDeparte.Location.CENTER? rata:notRata);

        return input;
    }
    public Location getLocationBlue(){
        return location;
    }
}
