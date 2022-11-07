package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import org.firstinspires.ftc.teamcode.robots.Jaws
import org.firstinspires.ftc.teamcode.core.BaseOpmode
import kotlinx.coroutines.*


@TeleOp(group = "testing")
@Config
class PlacementTest : BaseOpmode() {


  companion object{
    @JvmField var dist = 3.0
  }

  val robot = Jaws()
  var placing = false
  
  override fun setRobot() = robot

  override fun onInit(scope: CoroutineScope){
    robot.camera.startCamera()
    scope.launch{ 
      robot.camera.apply{
        while(!cameraRunning) yield()
        webcam.setPipeline(robot.camera.alignmentPipeline)
        aligning = true
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
        robot.alignAndPlace(dist)
        placing = false
      }
    }

  }

  override fun onUpdate(scope: CoroutineScope){
    if(!placing){
      robot.drivetrain.drive(
        gamepad1.left_stick_x.toDouble(),
        -gamepad1.left_stick_y.toDouble(),
        gamepad1.right_stick_x * 0.7,
      )
    }

    telemetry.addData("distance", robot.camera.dist)
    telemetry.update()

  }
} 
