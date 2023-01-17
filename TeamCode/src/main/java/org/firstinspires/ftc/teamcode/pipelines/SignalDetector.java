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

  public static double lb1 = 100;
  public static double lb2 = 0;
  public static double lb3 = 50;

  public static double hb1 = 115;
  public static double hb2 = 255;
  public static double hb3 = 255;
  
  public static double ly1 = 0;
  public static double ly2 = 0;
  public static double ly3 = 80;

  public static double hy1 = 100;
  public static double hy2 = 255;
  public static double hy3 = 180;

  public static double lm1 = 140;
  public static double lm2 = 50;
  public static double lm3 = 30;

  public static double hm1 = 170;
  public static double hm2 = 255;
  public static double hm3 = 255;

  public static int x = 60;
  public static int y = 80;
  public static int w = 200;
  public static int h = 100;

  public static Out stage = Out.RAW;

  public int signalVal = 0;

  enum Out {
    RAW, 
    BLUE,
    YELLOW,
    MAGENTA
  }


@Override
  public Mat processFrame(Mat input) {
    Imgproc.blur(input, input, new Size(3, 3));
    input = input.submat(y, y + h, x, x + w);
    Imgproc.cvtColor(input, hslMat, Imgproc.COLOR_RGB2HLS);

    Core.inRange(hslMat, new Scalar(lb1, lb2, lb3), new Scalar(hb1, hb2, hb3), blueMask);
    Core.inRange(hslMat, new Scalar(ly1, ly2, ly3), new Scalar(hy1, hy2, hy3), yellowMask);
    Core.inRange(hslMat, new Scalar(lm1, lm2, lm3), new Scalar(hm1, hm2, hm3), magentaMask);

    final double bluePixels = Core.countNonZero(blueMask);
    final double yellowPixels = Core.countNonZero(yellowMask);
    final double magentaPixels = Core.countNonZero(magentaMask);

    if(bluePixels > yellowPixels && bluePixels > magentaPixels) 
      signalVal = 1;
    else if(magentaPixels > yellowPixels && magentaPixels > bluePixels) 
      signalVal = 2;
    else if(yellowPixels > bluePixels && yellowPixels > magentaPixels) 
      signalVal = 3;

    // telemetry.addData("blue", bluePixels);
    // telemetry.addData("yellow", yellowPixels);
    // telemetry.addData("magenta", magentaPixels);
    // telemetry.addData("signal value", signalVal);
    // telemetry.update();

    switch (stage){
      default: return input;
      case BLUE: return blueMask;
      case YELLOW: return yellowMask;
      case MAGENTA: return magentaMask;
    }
  }
}
