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

  enum class Stage{
      RAW,
      RED_MASK,
      BLUE_MASK,
      YELLOW_MASK,
      CONTOURS,
  }

  companion object {
    @JvmField var rlcr = 170.0
    @JvmField var rlcb = 0.0

    @JvmField var rhcr = 255.0
    @JvmField var rhcb = 255.0

    @JvmField var blcr = 0.0
    @JvmField var blcb = 160.0

    @JvmField var bhcr = 120.0
    @JvmField var bhcb = 255.0

    @JvmField var ylcr = 50.0
    @JvmField var ylcb = 0.0

    @JvmField var yhcr = 180.0
    @JvmField var yhcb = 110.0

    @JvmField var minArea = 1200.0

    @JvmField var stage = Stage.CONTOURS
  }


  private val blueContours = mutableListOf<MatOfPoint>()
  private val redContours = mutableListOf<MatOfPoint>()
  private val yellowContours = mutableListOf<MatOfPoint>()

  var blueCones = listOf<Point>()
    private set
  var redCones = listOf<Point>()
    private set
  var poles = listOf<Point>()
    private set

  override fun processFrame(input: Mat): Mat{
    blueContours.clear()
    redContours.clear()
    yellowContours.clear()

    Imgproc.blur(input, input, Size(5.0, 5.0))
    Imgproc.cvtColor(input, cvtMat, Imgproc.COLOR_RGB2YCrCb)

    Core.inRange(cvtMat, Scalar(0.0, rlcr, rlcb), Scalar(255.0, rhcr, rhcb), redMask)
    Core.inRange(cvtMat, Scalar(0.0, blcr, blcb), Scalar(255.0, bhcr, bhcb), blueMask)
    Core.inRange(cvtMat, Scalar(0.0, ylcr, ylcb), Scalar(255.0, yhcr, yhcb), yellowMask)

    Imgproc.findContours(blueMask, blueContours, Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE)
    Imgproc.findContours(redMask, redContours, Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE)
    Imgproc.findContours(yellowMask, yellowContours, Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE)

    blueContours.removeAll(blueContours.filter{ Imgproc.contourArea(it) < minArea })
    redContours.removeAll(redContours.filter{ Imgproc.contourArea(it) < minArea })
    yellowContours.removeAll(yellowContours.filter{ Imgproc.contourArea(it) < minArea })

    fun List<MatOfPoint>.positions() = map {contour ->
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

    blueCones = blueContours.positions()
    redCones = redContours.positions()
    poles = yellowContours.positions()


    return when (stage){
      Stage.RED_MASK -> redMask
      Stage.BLUE_MASK -> blueMask
      Stage.YELLOW_MASK -> yellowMask
      Stage.CONTOURS -> {
        Imgproc.drawContours(input, blueContours, -1, Scalar(0.0, 0.0, 200.0), 2, 8)
        Imgproc.drawContours(input, redContours, -1, Scalar(200.0, 0.0, 0.0), 2, 8)
        Imgproc.drawContours(input, yellowContours, -1, Scalar(200.0, 200.0, 0.0), 2, 8)
        input
      }
      Stage.RAW -> input
    }
  }
}
