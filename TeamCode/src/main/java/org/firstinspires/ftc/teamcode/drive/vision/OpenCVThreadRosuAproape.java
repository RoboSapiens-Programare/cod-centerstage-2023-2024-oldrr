package org.firstinspires.ftc.teamcode.drive.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class OpenCVThreadRosuAproape extends Thread {
    static OpenCvWebcam Webcam;
    static PiramidaRosuAproape pipeline;

    private PiramidaRosuAproape.Location location;


    //Constructor
    public OpenCVThreadRosuAproape(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1"); //TODO get camera name
        Webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new PiramidaRosuAproape();
        Webcam.setPipeline(pipeline);

        Webcam.openCameraDeviceAsync(new OpenCvWebcam.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                Webcam.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    //Destructor
    @Override
    public void finalize() throws Throwable {
        super.finalize();
        Webcam.stopStreaming();
        Webcam.closeCameraDevice();
    }

    public PiramidaRosuAproape.Location getLocation(){
        return location;
    }


    //Dis the RUN method
    @Override
    public void run() {
        while(this.isAlive()){
            this.location = pipeline.getLocationRed();
        }


        Webcam.stopStreaming();
        Webcam.closeCameraDevice();
    }

}
