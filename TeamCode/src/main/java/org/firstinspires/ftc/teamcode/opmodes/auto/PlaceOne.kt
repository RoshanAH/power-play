package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.roshanah.jerky.utils.DriveValues
import com.roshanah.jerky.utils.PSVAController
import com.roshanah.jerky.math.deg
import com.roshanah.jerky.math.Vec2
import com.roshanah.jerky.math.Pose
import com.roshanah.jerky.math.bisectionMethodSolve
import com.roshanah.jerky.math.rad
import com.roshanah.jerky.math.Angle
import com.roshanah.jerky.math.sin
import com.roshanah.jerky.math.cos
import com.roshanah.jerky.profiling.buildProfile
import org.firstinspires.ftc.teamcode.components.Webcam
import org.firstinspires.ftc.teamcode.core.BaseOpmode
import org.firstinspires.ftc.teamcode.robots.Jaws
import org.firstinspires.ftc.teamcode.pipelines.AlignmentDetector
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.launch
import com.acmerobotics.dashboard.FtcDashboard
import kotlinx.coroutines.yield
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.GlobalScope
import kotlin.math.sqrt
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sign

@Autonomous
class PlaceOn : BaseOpmode(){
  val robot = Jaws().apply { active = { opModeIsActive() } }
  var signal: Int? = null

  override fun setRobot() = robot

  override fun onInit(scope: CoroutineScope){
    robot.slides.claw.setPosition(robot.slides.close)
    robot.camera.mount.setPosition(0.53)
    robot.slides.close()
    robot.camera.webcam.setPipeline(robot.camera.signalPipeline)

    scope.launch{
      robot.camera.startCamera()
      while(opModeInInit() && !robot.camera.cameraRunning){
        telemetry.addLine("Waiting for camera to start...")
        telemetry.update()
        yield()
      }

      FtcDashboard.getInstance().startCameraStream(robot.camera.webcam, 30.0)
      
      while(opModeInInit() || signal == null || signal == 0){
        signal = robot.camera.signal
        telemetry.addData("signal", robot.camera.signal)
        telemetry.update()
        yield()
      }
    }
  }

  override fun onStart(scope: CoroutineScope){
    val sideSign = if(robot.side == "right") 1.0 else -1.0

    scope.launch{
      while(opModeIsActive() && signal == null) yield() // wait until the camera has a signal

      robot.camera.webcam.setPipeline(robot.camera.alignmentPipeline) // switch the camera to alignment mode
      robot.camera.alignment = Webcam.Alignment.ALL
      robot.slides.targetPosition = robot.high

      robot.follow{ // approach the junction
        to(-40.0 * sideSign, 5.0, 0.0)
        val angle = 25.0.deg * sideSign
        turnTo(0.0, 35.0, angle)
      }

      robot.place()
      delay(250L)
      // after we place, we want to put the slides down after a second and only look at blue cones in order to pick up from the stack
      // we call launch here because we want to wait a second without blocking the code below it
      launch { 
        delay(500L)
        robot.slides.targetPosition = 0.01
      }

      // back up from the junction, and move and turn to look at the cone stack
      robot.follow(theta=robot.imu.heading){
        to(Pose(-(90.0.deg + theta).dir * 30.0, 0.0))
        turnTo((10.0 + abs(cos(theta)) * 5.0) * sideSign, 5.0, -theta)
        to(0.0, 30.0, 0.0)
        turnTo(20.0 * sideSign, 0.0, -90.0.deg * sideSign)
      }

      robot.drivetrain.move(DriveValues.zero)
      robot.follow{
        val displacement = ((signal!! - 2) * 18.0) * sideSign + 5.0
        to(0.0, sqrt(constants.maxAcceleration * abs(displacement)) * sign(displacement), 0.0)
        stop()
      }
      robot.drivetrain.move(DriveValues.zero)
    }
  }

  override fun onUpdate(scope: CoroutineScope){
    telemetry.addData("heading", robot.imu.heading.deg)
    telemetry.update()
  }

  override fun onStop(scope: CoroutineScope){
    robot.camera.webcam.closeCameraDevice() 
  }
}
