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
import kotlin.math.abs

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

  companion object {
    @JvmField var rlcr = 180.0
    @JvmField var rlcb = 0.0

    @JvmField var rhcr = 255.0
    @JvmField var rhcb = 255.0

    @JvmField var blcr = 100.0
    @JvmField var blcb = 170.0

    @JvmField var bhcr = 150.0
    @JvmField var bhcb = 255.0

    @JvmField var ylcr = 140.0
    @JvmField var ylcb = 50.0

    @JvmField var yhcr = 180.0
    @JvmField var yhcb = 100.0

    @JvmField var minArea = 500.0

    @JvmField var captureMode = CaptureMode.ALL
    @JvmField var stage = Stage.CONTOURS
  }


  private val contours = mutableListOf<MatOfPoint>()
  var objects = listOf<Point>()
    private set
  var center: Point? = null
    private set

  override fun processFrame(input: Mat): Mat{
    contours.clear()

    Imgproc.blur(input, input, Size(5.0, 5.0))
    Imgproc.cvtColor(input, cvtMat, Imgproc.COLOR_RGB2YCrCb)


    when (captureMode){
      CaptureMode.RED -> Core.inRange(cvtMat, Scalar(0.0, rlcr, rlcb), Scalar(255.0, rhcr, rhcb), mergedMask)
      CaptureMode.BLUE -> Core.inRange(cvtMat, Scalar(0.0, blcr, blcb), Scalar(255.0, bhcr, bhcb), mergedMask)
      CaptureMode.ALL -> {
        Core.inRange(cvtMat, Scalar(0.0, rlcr, rlcb), Scalar(255.0, rhcr, rhcb), redMask)
        Core.inRange(cvtMat, Scalar(0.0, blcr, blcb), Scalar(255.0, bhcr, bhcb), blueMask)
        Core.inRange(cvtMat, Scalar(0.0, ylcr, ylcb), Scalar(255.0, yhcr, yhcb), yellowMask)
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
        
    // find object closest to the center
    if(objects.isNotEmpty()){
      var center = objects[0]
      for(i in 1 until objects.size){
        val obj = objects[i]
          if (obj.y < center.y) center = obj
      }
      this.center = center
    } else {
      this.center = null
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
