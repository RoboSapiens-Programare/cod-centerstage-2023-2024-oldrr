package org.firstinspires.ftc.teamcode.drive.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class PiramidaAlbastruAproape extends OpenCvPipeline {
    //Telemetry telemetry;
    Mat mat = new Mat();

    public int x1 = 100, y1 = 430, x2 = 200, y2 = 530;

    private final Scalar LOW_BLUE = new Scalar(110, 50, 50);
    private final Scalar HIGH_BLUE = new Scalar(130, 255, 255);

    //TODO de aflat valoarea minima de galben dintr-un dreptunghi, fara ratoi
    private static final double PERCENT_COLOR_THRESHOLD = 0.2;

    public enum Location {
        LEFT,
        CENTER,
        RIGHT
    }

    private PiramidaAlbastruAproape.Location location = PiramidaAlbastruAproape.Location.RIGHT;

    //TODO de gasit punctele pentru dreptunghiuri
    static Rect LEFT_ROI = new Rect(
            new Point(10, 330),
            new Point(110, 430
            )
    );

    static Rect CENTER_ROI = new Rect(
            new Point (550, 380),
            new Point(650, 480)
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
        Imgproc.rectangle(input, CENTER_ROI, BLUE, 2);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
//            Mat center = mat.submat(CENTER_ROI);
        Mat center = mat.submat(CENTER_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
//            double centerValue = Core.sumElems(center).val[0] / CENTER_ROI.area() / 255;
        double centerValue = Core.sumElems(center).val[0] / CENTER_ROI.area() / 255;

        left.release();
//            center.release();
        center.release();

            /*telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
            telemetry.addData("Center raw value", (int) Core.sumElems(left).val[0]);
            telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
            telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
            //telemetry.addData("Center percentage", Math.round(centerValue * 100) + "%");
            telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");*/

        boolean propLeft = leftValue > PERCENT_COLOR_THRESHOLD;
//            boolean duckCenter = centerValue > PERCENT_COLOR_THRESHOLD;
        boolean propCenter = centerValue > PERCENT_COLOR_THRESHOLD;

        if(propCenter && centerValue > leftValue) {
            location = PiramidaAlbastruAproape.Location.CENTER;
            //telemetry.addData("Duck Location", "left");
        }
//            else if(duckCenter) {
//                location = Location.CENTER;
//                //telemetry.addData("Duck Location", "center");
//            }
        else if(propLeft & leftValue > centerValue) {
            location = PiramidaAlbastruAproape.Location.LEFT;
            //telemetry.addData("Duck Location", "right");
        }
        else if(centerValue < PERCENT_COLOR_THRESHOLD && leftValue < PERCENT_COLOR_THRESHOLD){
            location = PiramidaAlbastruAproape.Location.RIGHT;
        }
        //telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar notRata = new Scalar(255, 0, 0);
        Scalar rata = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == PiramidaAlbastruAproape.Location.LEFT? rata:notRata);
        Imgproc.rectangle(mat, CENTER_ROI, location == PiramidaAlbastruAproape.Location.CENTER? rata:notRata);

        return input;
    }
    public Location getLocationBlue(){
        return location;
    }
}
