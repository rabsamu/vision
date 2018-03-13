/**
 * Starter Code for Robot Class.
 * Thursday, October 12, 2017
 */

package org.strykeforce.vision;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.vision.VisionThread;
import edu.wpi.cscore.UsbCamera;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Rect;
import java.util.Arrays;


public class Robot extends TimedRobot {

    private VisionThread visionThread;
    private final Object imgLock = new Object();
    public UsbCamera camera;
    public Rect r;
    public int rEdge;
    public enum Side {//this currently says if you want the right or left-most block.
        left,
        right
    }
    public Side side = Side.right; //FIXME make better
    public boolean validRead;

    public double xCoordinate;
    public double[] xArray = new double [5];//potentially make length configurable

    public double cubeAngle;




    @Override
    public void robotInit() {
        camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution(320, 240);
        camera.setExposureManual(50);
    }

    @Override
    public void autonomousInit() {
        visionThread = new VisionThread(camera, new GripCode(), pipeline -> {
            int m = pipeline.filterContoursOutput().size();//find the number of contours

            if (!pipeline.filterContoursOutput().isEmpty()) {//if a contour is found
                synchronized (imgLock) {
                    if (side == Side.right) {
                        rEdge = 0;
                        for(int n = 0 ; n < m; n++){//find the right-most contour
                            if (Imgproc.boundingRect(pipeline.filterContoursOutput().get(n)).x > rEdge){
                                r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(n));
                                rEdge = r.x + r.width;
                            }
                        }
                    }
                    if (side == Side.left) {
                        rEdge = 320;
                        for(int n = 0 ; n < m; n++){//find the left-most block
                            if (Imgproc.boundingRect(pipeline.filterContoursOutput().get(n)).x < rEdge){
                                r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(n));
                                rEdge = r.x;
                            }
                        }
                    }
                }
            }

        });
        visionThread.start();
        CameraServer.getInstance().startAutomaticCapture();


        for(int l = 0; l< xArray.length; l++) {//take (currently) 5 readings
            synchronized (imgLock) {
                xCoordinate = this.rEdge;
                System.out.println("edge: " + xCoordinate);
                xArray[l] = xCoordinate;
            }
            if (xCoordinate != 0 && xCoordinate != 320) {//if the reading is valid
                validRead = true;
                Arrays.sort(xArray);
                xCoordinate = xArray[3];//find the median of the edge readings, if the length of xArray is changed, the 3 will have to change
            }
        }
    }

    @Override
    public void autonomousPeriodic() {

        //System.out.println("edge: " + xCoordinate);

        cubeAngle = (xCoordinate -160)*30/160 ; //calculate cube position as a scaler from -30 to 30
        System.out.println("Angle: " + cubeAngle);

    }

}

