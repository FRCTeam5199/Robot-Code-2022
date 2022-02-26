package frc.vision.camera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import frc.misc.ISubsystem;
import frc.misc.SubsystemStatus;
import frc.misc.UserInterface;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

public class CameraViewer implements ISubsystem {
    UsbCamera camera;
    Thread thread;


    public CameraViewer(){
        addToMetaList();
        init();
    }

    @Override
    public void init() {
        camera = CameraServer.startAutomaticCapture(0);
        camera.setResolution(640, 480);
        UserInterface.SMART_DASHBOARD.add("CameraViewer", camera);
        /*thread = new Thread("Camera thread");
        thread.start();
         */
    }

    @Override
    public SubsystemStatus getSubsystemStatus() {
        return null;
    }

    @Override
    public void updateTest() {

    }

    @Override
    public void updateTeleop() {
        /*
        Mat source = new Mat();
        Mat output = new Mat();

        CvSink cvSink = CameraServer.getVideo();
        CvSource outputStream = CameraServer.putVideo("Blur", 640, 480);

        while (!Thread.interrupted()) {
            cvSink.grabFrame(source);
            Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
            outputStream.putFrame(output);
        }
         */
    }

    @Override
    public void updateAuton() {

    }

    @Override
    public void updateGeneric() {

    }

    @Override
    public void initTest() {

    }

    @Override
    public void initTeleop() {

    }

    @Override
    public void initAuton() {

    }

    @Override
    public void initDisabled() {

    }

    @Override
    public void initGeneric() {

    }

    @Override
    public String getSubsystemName() {
        return "Viewable Camera";
    }
}
