// package org.firstinspires.ftc.teamcode.pipelines
//
// import com.qualcomm.robotcore.eventloop.opmode.Disabled
// import com.acmerobotics.dashboard.config.Config
// import org.firstinspires.ftc.robotcore.external.Telemetry
// import org.opencv.core.Core
// import org.opencv.core.Point
// import org.opencv.core.Size
// import org.opencv.core.Mat
// import org.opencv.core.MatOfPoint
// import org.opencv.core.Scalar
// import org.opencv.imgproc.Imgproc
// import org.openftc.easyopencv.OpenCvPipeline
// import kotlin.math.abs
//
// @Config
// class ConeStackDetector : OpenCvPipeline() {
//   var cvtMat = Mat()
//
//   var blueMask = Mat()
//   var redMask = Mat()
//   var yellowMask = Mat()
//
//   enum class Stage{
//       RAW,
//       RED_THRESHOLD,
//       BLUE_MASK,
//       CONTOURS,
//   }
//
//   companion object {
//     @JvmField var minArea = 1200.0
//
//     @JvmField var stage = Stage.CONTOURS
//   }
//
//   var blueCones = listOf<Point>()
//     private set
//   var redCones = listOf<Point>()
//     private set
//
//   override fun processFrame(input: Mat): Mat{
//     blueContours.clear()
//     redContours.clear()
//     yellowContours.clear()
//
//     Imgproc.blur(input, input, Size(5.0, 5.0))
//     Imgproc.cvtColor(input, cvtMat, Imgproc.COLOR_RGB2YCrCb)
//     
//     AlignmentDetector.apply{
//       Core.inRange(cvtMat, Scalar(0.0, rlcr, rlcb), Scalar(255.0, rhcr, rhcb), redMask)
//       Core.inRange(cvtMat, Scalar(0.0, blcr, blcb), Scalar(255.0, bhcr, bhcb), blueMask)
//     }
//
//     Imgproc.findContours(blueMask, blueContours, Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE)
//     Imgproc.findContours(redMask, redContours, Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE)
//     Imgproc.findContours(yellowMask, yellowContours, Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE)
//
//     blueContours.removeAll(blueContours.filter{ Imgproc.contourArea(it) < minArea })
//     redContours.removeAll(redContours.filter{ Imgproc.contourArea(it) < minArea })
//
//     return when (stage){
//       Stage.RED_MASK -> redMask
//       Stage.BLUE_MASK -> blueMask
//       Stage.YELLOW_MASK -> yellowMask
//       Stage.CONTOURS -> {
//         Imgproc.drawContours(input, blueContours, -1, Scalar(0.0, 0.0, 200.0), 2, 8)
//         Imgproc.drawContours(input, redContours, -1, Scalar(200.0, 0.0, 0.0), 2, 8)
//         Imgproc.drawContours(input, yellowContours, -1, Scalar(200.0, 200.0, 0.0), 2, 8)
//         input
//       }
//       Stage.RAW -> input
//     }
//   }
// }
