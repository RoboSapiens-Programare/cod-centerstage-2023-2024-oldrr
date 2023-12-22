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

    private final Scalar LOW_BLUE = new Scalar(5,255,255);
    private final Scalar HIGH_BLUE = new Scalar(180,255,255);

    //TODO de aflat valoarea minima de galben dintr-un dreptunghi, fara ratoi
    private static final double PERCENT_COLOR_THRESHOLD = 0.007;

    public enum Location {
        LEFT,
        CENTER,
        RIGHT
    }

    private PiramidaRosu.Location location = PiramidaRosu.Location.RIGHT;

    //TODO de gasit punctele pentru dreptunghiuri
    static final Rect LEFT_ROI = new Rect(
            new Point(150, 180),
            new Point(440, 480)
    );

    static final Rect CENTER_ROI = new Rect(
            new Point(520, 240),
            new Point(760, 480)
    );

    static final Rect RIGHT_ROI = new Rect(
            new Point (840, 240),
            new Point(1080, 480)
    );

    @Override
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = LOW_BLUE;
        Scalar highHSV = HIGH_BLUE;

        //dreptunghiuri regiuni
        final Scalar BLUE = new Scalar(0, 0, 255);
        Imgproc.rectangle(input, LEFT_ROI, BLUE, 2);
//            Imgproc.rectangle(input, CENTER_ROI, BLUE, 2);
        Imgproc.rectangle(input, RIGHT_ROI, BLUE, 2);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
//            Mat center = mat.submat(CENTER_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
//            double centerValue = Core.sumElems(center).val[0] / CENTER_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
//            center.release();
        right.release();

            /*telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
            telemetry.addData("Center raw value", (int) Core.sumElems(left).val[0]);
            telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
            telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
            //telemetry.addData("Center percentage", Math.round(centerValue * 100) + "%");
            telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");*/

        boolean propLeft = leftValue > PERCENT_COLOR_THRESHOLD;
//            boolean duckCenter = centerValue > PERCENT_COLOR_THRESHOLD;
        boolean propCenter = rightValue > PERCENT_COLOR_THRESHOLD;

        if(propLeft) {
            location = PiramidaRosu.Location.LEFT;
            //telemetry.addData("Duck Location", "left");
        }
//            else if(duckCenter) {
//                location = Location.CENTER;
//                //telemetry.addData("Duck Location", "center");
//            }
        else if(propCenter) {
            location = PiramidaRosu.Location.CENTER;
            //telemetry.addData("Duck Location", "right");
        }
        else{
            location = PiramidaRosu.Location.RIGHT;
        }
        //telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar notRata = new Scalar(255, 0, 0);
        Scalar rata = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == PiramidaRosu.Location.LEFT? rata:notRata);
        Imgproc.rectangle(mat, CENTER_ROI, location == PiramidaRosu.Location.CENTER? rata:notRata);
        Imgproc.rectangle(mat, RIGHT_ROI, location == PiramidaRosu.Location.RIGHT? rata:notRata);

        return input;
    }
    public Location getLocationRed(){
        return location;
    }
}
