package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap
import org.openftc.easyopencv.OpenCvWebcam
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraRotation
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.teamcode.core.Component
import org.firstinspires.ftc.teamcode.pipelines.SignalDetector
import org.firstinspires.ftc.teamcode.pipelines.AlignmentDetector
import org.opencv.core.Point
import kotlinx.coroutines.CoroutineScope
import java.util.concurrent.TimeUnit
import kotlin.math.PI
import kotlin.math.pow
import kotlin.math.cos
import kotlin.math.abs
import kotlin.math.tan

class Webcam(
  map: HardwareMap,
  webcam: String,
  mount: String,
  val robotVel: () -> Double = { 0.0 }
) : Component {

  var height = 0.0
  var kp = 0.5 // how sensitive camera turret is to lock onto object
  var signalTheta = 80.0

  val webcam = OpenCvCameraFactory.getInstance().createWebcam(map.get(WebcamName::class.java, webcam))
  val mount = map.servo.get(mount)

  val signalPipeline = SignalDetector()
  val alignmentPipeline = AlignmentDetector()

  val objects: List<Point>
    get() = alignmentPipeline.objects
  val signal: Int
    get() = signalPipeline.signalVal

  var theta = 90.0 
  var aligning = false

  val dist: Double
    get() = height * tan(theta.rad)

  var cameraRunning = false
    private set

  private var lastTime = System.nanoTime() * 1e-9

  fun startCamera(){
    webcam.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
        override fun onOpened(){
          webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
          Thread.sleep(1000)
          cameraRunning = true
        }

        override fun onError(errorCode: kotlin.Int) {}
    })
  }

  fun runAlignment() {
    if (!cameraRunning) return
    webcam.setPipeline(alignmentPipeline)
  }

  fun runSignal() {
    if (!cameraRunning) return
    webcam.setPipeline(signalPipeline)
  }

  override fun init(scope: CoroutineScope) {}
  override fun start(scope: CoroutineScope) {}


  override fun update(scope: CoroutineScope) {
    val time = System.nanoTime() * 1e-9
    val deltaTime = time - lastTime
    lastTime = time

    if (aligning) {
      val center = alignmentPipeline.center
      if (center != null){
        // change in theta as a function of velocity is v * cos(theta) / height
        val dTheta = robotVel() * cos(theta.rad).pow(2.0) / height
        theta += dTheta * deltaTime + center.y * kp
      } else{
        // if there are no objects detected move the camera up to look at poles
        theta = 90.0
      }
    }
    mount.setPosition(0.5 + (90.0 - theta) / (300.0))
  }
}

private val Double.rad: Double
  get() = this / 180.0 * PI
