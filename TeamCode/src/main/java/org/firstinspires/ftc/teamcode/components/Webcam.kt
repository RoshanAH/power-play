package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.openftc.easyopencv.OpenCvWebcam
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraRotation
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl
import org.firstinspires.ftc.teamcode.core.Component
import org.firstinspires.ftc.teamcode.pipelines.SignalDetector
import org.firstinspires.ftc.teamcode.pipelines.AlignmentDetector
import org.firstinspires.ftc.teamcode.opmodes.testing.AlignmentTest
import org.opencv.core.Point
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.launch
import kotlinx.coroutines.delay
import java.util.concurrent.TimeUnit
import kotlin.math.PI
import kotlin.math.pow
import kotlin.math.sqrt
import kotlin.math.atan
import com.roshanah.jerky.math.*

class Webcam(
  map: HardwareMap,
  webcam: String,
  encoder: String,
  mount: String,
  val slidesPos: () -> Double,
  val turretTheta: () -> Angle,
) : Component {

  var ticksPerDegree = 22.75555
  var mountPos = Vec3(-2.798, 3.603, 13.407)
  var armLength = 2.719
  var cameraTilt = -20.0.deg

  var diagnalFOV = 55.0.deg

  var targetPhi = 100.0.deg
  var phi = 100.0.deg
    private set

  var target: Vec3? = null

  private val p = sqrt(320.0.pow(2.0) + 240.0.pow(2.0)) / (tan(diagnalFOV * 0.5))

  val vfov = atan(120.0 / (2 * p)).rad * 2.0
  val hfov = atan(160.0 / (2 * p)).rad * 2.0

  val orientation: Mat3
    get() = Mat3.run { rotationZ(turretTheta()) * rotationX(-(phi - 90.0.deg)) * rotationZ(cameraTilt) }

  val pos: Vec3
    get() = (mountPos + Vec3(0.0, armLength, 0.0).rotateX(-(phi - 90.0.deg))).rotateZ(turretTheta())

  val webcam = OpenCvCameraFactory.getInstance().createWebcam(map.get(WebcamName::class.java, webcam))
  val mount = map.servo.get(mount)
  val encoder = map.dcMotor.get(encoder)

  val signalPipeline = SignalDetector()
  val alignmentPipeline = AlignmentDetector()

  val signal: Int
    get() = signalPipeline.signalVal

  enum class Alignment {
    BLUE_CONES,
    RED_CONES,
    POLES,
    NONE,
  }

  var alignment = Alignment.NONE

  var blueCones = listOf<Vec3>()
  var redCones = listOf<Vec3>()
  var poles = listOf<Vec3>()

  val cones: List<Vec3>
    get() = blueCones + redCones


  private var cameraZeroed = false
  private var cameraStarted = false

  val cameraActive: Boolean
    get() = cameraZeroed && cameraStarted

  private var lastTime = System.nanoTime() * 1e-9

  fun startCamera(){
    webcam.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
        override fun onOpened(){
          webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
          Thread.sleep(1000)
          cameraStarted = true
        }
        override fun onError(errorCode: kotlin.Int) {}
    })
  }
  override fun init(scope: CoroutineScope) {
    encoder.direction = DcMotorSimple.Direction.REVERSE
    scope.launch{
      mount.setPosition(0.5)
      delay(500L)
      encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
      encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
    }
  }
  
  override fun start(scope: CoroutineScope) {}

  override fun update(scope: CoroutineScope) {
    val time = System.nanoTime() * 1e-9
    val deltaTime = time - lastTime
    lastTime = time

    blueCones = alignmentPipeline.blueCones.map { it.project(0.0) }
    redCones = alignmentPipeline.redCones.map { it.project(0.0) }
    poles = alignmentPipeline.poles.map { it.project(3.0) }

    phi = (encoder.getCurrentPosition() / ticksPerDegree + 90.0).deg

   if (alignment != Alignment.NONE){
        target = when(alignment){
          Alignment.BLUE_CONES -> blueCones.closest
          Alignment.RED_CONES -> redCones.closest
          Alignment.POLES -> poles.closest
          Alignment.NONE -> null
        }

        targetPhi = target?.let {
          phi + (it - pos).phi
        } ?: (targetPhi - (30.0.deg * deltaTime))
      }

    // mount.setPosition(0.5 + (targetPhi.deg.coerceAtLeast(100.0) - 90.0) / (300.0)) old one that works
    mount.setPosition(0.5 + (targetPhi.deg.coerceAtLeast(100.0) - 90.0) / (243.0))
  }
  
  private fun Point.project(zPlane: Double): Vec3{
    println("($x, $y)")
    val localDirection = Vec3(x, p, y).unit // points in the positive y direction
    val absoluteDirection = orientation.inv * localDirection
    val pos = pos
    return absoluteDirection * ((zPlane - pos.z) / absoluteDirection.z) + pos
  }
}

val List<Vec3>.closest: Vec3?
  get() = minByOrNull { it.r }
