package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.roshanah.jerky.math.Pose
import com.roshanah.jerky.math.deg
import com.roshanah.jerky.math.rad
import org.firstinspires.ftc.teamcode.robots.Jaws
import org.firstinspires.ftc.teamcode.core.BaseOpmode
import org.firstinspires.ftc.teamcode.components.Webcam
import kotlinx.coroutines.*
import kotlin.math.sqrt
import kotlin.math.pow
import kotlin.math.abs


@TeleOp(group = "testing")
@Config
class PlacementTest : BaseOpmode() {


  val robot = Jaws()
  var placing = false
  
  override fun setRobot() = robot

  override fun onInit(scope: CoroutineScope){
    robot.camera.startCamera()
    scope.launch{ 
      robot.camera.apply{
        while(!cameraRunning && !isStopRequested()){
          telemetry.addLine("Waiting for camera to start")
          telemetry.update()
          yield()
        } 
        println("made it past while loop")
        webcam.setPipeline(robot.camera.alignmentPipeline)
        alignment = Webcam.Alignment.POLES
      }
    }

    val dash = FtcDashboard.getInstance()
    telemetry = MultipleTelemetry(telemetry, dash.getTelemetry())
    dash.startCameraStream(robot.camera.webcam, 30.0)
  }

  override fun onStart(scope: CoroutineScope){
    gamepadListener1.a.onPress = {
      scope.launch{
        placing = true
        robot.place() { opModeIsActive() }
        placing = false
      }
    }
  }

  var lastTime = System.nanoTime() * 1e-9

  override fun onUpdate(scope: CoroutineScope){

    if(!robot.camera.cameraRunning) return

    val time = System.nanoTime() * 1e-9
    val deltaTime = time - lastTime
    lastTime = time

    robot.slides.apply {
      if (gamepad1.dpad_up) targetPosition = robot.high
      else if (gamepad1.dpad_down) targetPosition = 0.01
      else if (gamepad1.dpad_left) targetPosition = robot.low
      else if (gamepad1.dpad_right) targetPosition = robot.medium
    }

    if(!placing){
      robot.drivetrain.drive(
        gamepad1.left_stick_x.toDouble(),
        -gamepad1.left_stick_y.toDouble(),
        gamepad1.right_stick_x * 0.7,
      )
    }

    val v = robot.drivetrain.relativeVel
    telemetry.addData("vel", v)

    robot.camera.closest?.let{
      var x = it.xy
      val targetVel = (it.theta + 90.0.deg).dir * sqrt(2 * robot.constants.maxAcceleration * abs(x.magnitude - 5.0))
      val tw = -(v.x * x.y + v.y * x.x) / x.magnitude

      telemetry.addData("tv", Pose(targetVel, tw))
      telemetry.addData("tw", tw)
      telemetry.addData("dist", x.magnitude)
      telemetry.addData("closest", it.xy)
      telemetry.addData("theta", it.theta.deg)
    }

    telemetry.addData("powers", robot.drivetrain.powers)

    telemetry.update()

  }
} 
