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
import kotlin.math.sqrt
import kotlin.math.atan
import com.roshanah.jerky.math.*

class Webcam(
  map: HardwareMap,
  webcam: String,
  mount: String,
  val slidesPos: () -> Double,
  val robotVel: () -> Double = { 0.0 }
) : Component {

  var height = 9.25 
  var diagnalFOV = 55.0.deg
  var kp = 0.1 // how sensitive camera turret is to lock onto object
  var phi = 0.0.deg

  val vfov = run {
    val p = sqrt(320.0.pow(2.0) + 240.0.pow(2.0)) / (2 * tan(diagnalFOV * 0.5))
    atan(120.0 / (2 * p)).rad * 2.0
  }

  val hfov = run {
    val p = sqrt(320.0.pow(2.0) + 240.0.pow(2.0)) / (2 * tan(diagnalFOV * 0.5))
    atan(160.0 / (2 * p)).rad * 2.0
  }

  val webcam = OpenCvCameraFactory.getInstance().createWebcam(map.get(WebcamName::class.java, webcam))
  val mount = map.servo.get(mount)

  val signalPipeline = SignalDetector()
  val alignmentPipeline = AlignmentDetector()

  val signal: Int
    get() = signalPipeline.signalVal

  enum class Alignment {
    BLUE_CONES,
    RED_CONES,
    POLES,
    ALL,
    NONE,
  }

  var alignment = Alignment.NONE

  var blueCones = listOf<Projection>()
  var redCones = listOf<Projection>()
  var poles = listOf<Projection>()

  val cones: List<Projection>
    get() = blueCones + redCones

  val objects: List<Projection>
    get() = blueCones + redCones + poles

  var closest: Projection? = null
    private set

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

    val slides = slidesPos()

    blueCones = alignmentPipeline.blueCones.project(0.0)
    redCones = alignmentPipeline.redCones.project(0.0)
    poles = alignmentPipeline.poles.project(3.0)

    if (alignment != Alignment.NONE){
      if (slides < 0.1 || slides > 0.3){
        closest = when(alignment){
          Alignment.BLUE_CONES -> blueCones.closest
          Alignment.RED_CONES -> redCones.filter { it.phi.deg > 90.0 }.minByOrNull { it.xy.magnitude }
          Alignment.POLES -> poles.filter { it.phi.deg > 90.0 }.minByOrNull { it.xy.magnitude }
          Alignment.ALL -> objects.filter { it.phi.deg > 90.0 }.minByOrNull { it.xy.magnitude }
          Alignment.NONE -> null
        }
      }
      phi = closest?.let {
        val hz = height - it.z
        val dPhidT = robotVel() * hz / (it.xy.y.pow(2.0) + hz.pow(2.0)) 
        val delta = (it.phi - 90.0.deg - phi).rad * kp + (dPhidT * deltaTime)

        val slidesMax = if (slides < 0.1) 30.0.deg else 70.0.deg
        // println("max: ${slidesMax.deg} phi: ${(phi + delta.rad).deg}")

        if (phi.rad + delta > slidesMax.rad) slidesMax    
        else phi + delta.rad
      } ?: (phi - (30.0.deg * deltaTime)).deg.coerceAtLeast(10.0).deg
    }


    mount.setPosition(0.5 + (phi.deg) / (300.0))
  }

  private fun List<Point>.project(zPlane: Double) = map {
    val p = sqrt(320.0.pow(2.0) + 240.0.pow(2.0)) / (2 * tan(diagnalFOV * 0.5))
    val theta = atan(it.x / (2 * p)).rad
    val phi = atan(-it.y / (2 * p)).rad + 90.0.deg + phi
    Projection(Vec2(sin(theta) * tan(phi), cos(theta) * tan(phi)) * (zPlane - height), zPlane, phi, theta)
  }

  data class Projection(val xy: Vec2, val z: Double, val phi: Angle, val theta: Angle)
}

val List<Webcam.Projection>.closest: Webcam.Projection?
  get() = filter { it.phi.deg > 90.0 }.minByOrNull { it.xy.magnitude }
