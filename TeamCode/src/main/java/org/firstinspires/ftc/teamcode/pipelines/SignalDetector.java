package org.firstinspires.ftc.teamcode.pipelines;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Size;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.HashMap;

@Config
public class SignalDetector extends OpenCvPipeline {
  Mat hslMat = new Mat();

  Mat blueMask = new Mat();
  Mat yellowMask = new Mat();
  Mat magentaMask = new Mat();

  public static double lbh = 85;
  public static double lbs = 108;
  public static double lbl = 49;

  public static double hbh = 116;
  public static double hbs = 152;
  public static double hbl = 128;
  
  public static double lyh = 3;
  public static double lys = 57;
  public static double lyl = 26;

  public static double hyh = 31;
  public static double hys = 255;
  public static double hyl = 183;

  public static double lmh = 142;
  public static double lms = 57;
  public static double lml = 26;

  public static double hmh = 194;
  public static double hms = 180;
  public static double hml = 130;

  public Out stage = Out.RAW;

  public int signalVal = 0;

  private Telemetry telemetry;

  enum Out {
    RAW, 
    BLUE,
    YELLOW,
    MAGENTA
  }


@Override
  public Mat processFrame(Mat input) {
    Imgproc.blur(input, input, new Size(3, 3));
    Imgproc.cvtColor(input, hslMat, Imgproc.COLOR_RGB2HLS);

    Core.inRange(hslMat, new Scalar(lbh, lbs, lbl), new Scalar(hbh, hbs, hbl), blueMask);
    Core.inRange(hslMat, new Scalar(lyh, lys, lyl), new Scalar(hyh, hys, hyl), yellowMask);
    Core.inRange(hslMat, new Scalar(lyh, lys, lyl), new Scalar(hmh, hms, hml), magentaMask);

    final double bluePixels = Core.countNonZero(blueMask);
    final double yellowPixels = Core.countNonZero(yellowMask);
    final double magentaPixels = Core.countNonZero(magentaMask);

    if(bluePixels > yellowPixels && bluePixels > magentaPixels) 
      signalVal = 1;
    else if(magentaPixels > yellowPixels && magentaPixels > bluePixels) 
      signalVal = 2;
    else if(yellowPixels > bluePixels && yellowPixels > magentaPixels) 
      signalVal = 3;

    telemetry.addData("blue", bluePixels);
    telemetry.addData("yellow", yellowPixels);
    telemetry.addData("magenta", magentaPixels);
    telemetry.addData("signal value", signalVal);
    telemetry.update();

    switch (stage){
      default: return input;
      case BLUE: return blueMask;
      case YELLOW: return yellowMask;
      case MAGENTA: return magentaMask;
    }
  }
}
