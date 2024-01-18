package org.firstinspires.ftc.teamcode.drive.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class PiramidaRosu extends OpenCvPipeline {
    //Telemetry telemetry;
    Mat mat = new Mat();

    private final Scalar LOW_RED = new Scalar(0, 50, 30);
    private final Scalar HIGH_RED = new Scalar(8, 255, 255);

    //TODO de aflat valoarea minima de galben dintr-un dreptunghi, fara ratoi
    private static final double PERCENT_COLOR_THRESHOLD = 0.001;

    public enum Location {
        LEFT,
        CENTER,
        RIGHT
    }

    private PiramidaRosu.Location location = PiramidaRosu.Location.LEFT;

    //TODO de gasit punctele pentru dreptunghiuri
    static final Rect CENTER_ROI = new Rect(
            new Point(400, 470),
            new Point(500, 570)
    );

    static final Rect RIGHT_ROI = new Rect(
            new Point (1070, 470),
            new Point(1200, 570)
    );

    @Override
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = LOW_RED;
        Scalar highHSV = HIGH_RED;

        //dreptunghiuri regiuni
        final Scalar RED = new Scalar(0, 0, 255);
        Imgproc.rectangle(input, CENTER_ROI, RED, 2);
//            Imgproc.rectangle(input, CENTER_ROI, BLUE, 2);
        Imgproc.rectangle(input, RIGHT_ROI, RED, 2);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat center = mat.submat(CENTER_ROI);
//            Mat center = mat.submat(CENTER_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double centerValue = Core.sumElems(center).val[0] / CENTER_ROI.area() / 255;
//            double centerValue = Core.sumElems(center).val[0] / CENTER_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        center.release();
//            center.release();
        right.release();

            /*telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
            telemetry.addData("Center raw value", (int) Core.sumElems(left).val[0]);
            telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
            telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
            //telemetry.addData("Center percentage", Math.round(centerValue * 100) + "%");
            telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");*/

        boolean propCenter = centerValue > PERCENT_COLOR_THRESHOLD;
//            boolean duckCenter = centerValue > PERCENT_COLOR_THRESHOLD;
        boolean propRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if(propCenter) {
            location = PiramidaRosu.Location.CENTER;
            //telemetry.addData("Duck Location", "left");
        }
//            else if(duckCenter) {
//                location = Location.CENTER;
//                //telemetry.addData("Duck Location", "center");
//            }
        else if(propRight) {
            location = PiramidaRosu.Location.RIGHT;
            //telemetry.addData("Duck Location", "right");
        }
        else{
            location = PiramidaRosu.Location.LEFT;
        }
        //telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar notRata = new Scalar(255, 0, 0);
        Scalar rata = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, CENTER_ROI, location == PiramidaRosu.Location.CENTER? rata:notRata);
        Imgproc.rectangle(mat, RIGHT_ROI, location == PiramidaRosu.Location.RIGHT? rata:notRata);

        return input;
    }
    public Location getLocationRed(){
        return location;
    }
}
