package org.firstinspires.ftc.teamcode.pipelines

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.opencv.core.Core
import org.opencv.core.Point
import org.opencv.core.Size
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline

@Config
class AlignmentDetector : OpenCvPipeline(){
  var cvtMat = Mat()

  var blueMask = Mat()
  var redMask = Mat()
  var yellowMask = Mat()

  var mergedMask = Mat()

  enum class Stage{
      RAW,
      RED_MASK,
      BLUE_MASK,
      YELLOW_MASK,
      CONTOURS,
      OBJECTS
  }

  enum class CaptureMode{
      RED,
      BLUE,
      ALL
  }


  @JvmField var lr1 = 0.0
  @JvmField var lr2 = 0.0
  @JvmField var lr3 = 0.0

  @JvmField var hr1 = 0.0
  @JvmField var hr2 = 0.0
  @JvmField var hr3 = 0.0

  @JvmField var lb1 = 0.0
  @JvmField var lb2 = 0.0
  @JvmField var lb3 = 0.0

  @JvmField var hb1 = 0.0
  @JvmField var hb2 = 0.0
  @JvmField var hb3 = 0.0

  @JvmField var ly1 = 0.0
  @JvmField var ly2 = 0.0
  @JvmField var ly3 = 0.0

  @JvmField var hy1 = 0.0
  @JvmField var hy2 = 0.0
  @JvmField var hy3 = 0.0

  @JvmField var minArea = 100.0

  @JvmField var captureMode = CaptureMode.ALL
  @JvmField var stage = Stage.OBJECTS


  private val contours = mutableListOf<MatOfPoint>()
  var objects = listOf<Point>()
    private set

  override fun processFrame(input: Mat): Mat{
    contours.clear()

    Imgproc.blur(input, input, Size(5.0, 5.0))
    Imgproc.cvtColor(input, cvtMat, Imgproc.COLOR_RGB2YCrCb)


    when (captureMode){
      CaptureMode.RED -> Core.inRange(cvtMat, Scalar(lr1, lr2, lr3), Scalar(hr1, hr2, hr3), mergedMask)
      CaptureMode.BLUE -> Core.inRange(cvtMat, Scalar(lb1, lb2, lb3), Scalar(hb1, hb2, hb3), mergedMask)
      CaptureMode.ALL -> {
        Core.inRange(cvtMat, Scalar(lr1, lr2, lr3), Scalar(hr1, hr2, hr3), redMask)
        Core.inRange(cvtMat, Scalar(lb1, lb2, lb3), Scalar(hb1, hb2, hb3), blueMask)
        Core.inRange(cvtMat, Scalar(ly1, ly2, ly3), Scalar(hy1, hy2, hy3), yellowMask)

        Core.bitwise_or(redMask, blueMask, mergedMask)
        Core.bitwise_or(mergedMask, yellowMask, mergedMask)
      }
    }

    Imgproc.findContours(mergedMask, contours, Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE)
    contours.removeAll(contours.filter{ Imgproc.contourArea(it) < minArea })
    objects = contours.map{ contour ->
      val points = contour.toList() 
      var maxY = points[0].y
      var maxX = points[0].x
      var minX = points[0].x

      for(i in 1 until points.size){
        val p = points[i]
        if (p.y > maxY) maxY = p.y
        if (p.x > maxX) maxX = p.x
        if (p.x < maxY) minX = p.x
      }

      Point((minX + maxX - input.width()) * 0.5, input.height() * 0.5 - maxY)
    }


    return when (stage){
      Stage.RED_MASK -> redMask
      Stage.BLUE_MASK -> blueMask
      Stage.YELLOW_MASK -> yellowMask
      Stage.CONTOURS -> {
        Imgproc.drawContours(input, contours, -1, Scalar(200.0, 0.0, 200.0), 2, 8)
        input
      }
      Stage.OBJECTS -> {
        objects.forEach {
          Imgproc.ellipse(
            input,
            Point(it.x - input.width() * 0.5, input.height() * 0.5 - it.y),
            Size(3.0, 3.0),
            0.0,
            0.0,
            360.0,
            Scalar(255.0, 0.0, 255.0),
            1.unaryMinus()
          )
        }
        input
      }
      Stage.RAW -> input
    }
  }
}
