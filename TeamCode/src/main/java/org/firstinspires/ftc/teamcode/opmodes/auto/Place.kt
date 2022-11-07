package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.core.BaseOpmode
import org.firstinspires.ftc.teamcode.robots.Jaws
import kotlinx.coroutines.*

@Autonomous
class Place : BaseOpmode(){

  val robot = Jaws()

  override fun setRobot() = robot

  override fun onInit(scope: CoroutineScope){
    robot.camera.apply{
      startCamera()
      mount.setPosition(0.45)
      webcam.setPipeline(signalPipeline)
      theta = signalTheta

      telemetry.addData("side", robot.side)
      telemetry.update()

      scope.launch{
        // telemetry.addLine("Launching camera...")
        // telemetry.update()
        while(!cameraRunning) yield()   
        while(opModeInInit()){
          // telemetry.addData("Signal Value", signal)
          // telemetry.update()
          yield()
        }
      }
    }

    robot.slides.claw.setPosition(robot.slides.close)
    robot.slides.close()
  }

  override fun onStart(scope: CoroutineScope){
    scope.launch{
      while(!robot.camera.cameraRunning) yield()
      val signal = robot.camera.signalPipeline.signalVal
      robot.camera.runAlignment()
      robot.camera.aligning = true

      telemetry.addData("Signal Value", signal)
      telemetry.update()

      val sign = if (robot.side == "left") 1.0 else -1.0

      robot.drivetrain.apply{
        forwardUntilDone(10.0, 0.03, 3.0)
        strafeUntilDone(35.0 * sign, 0.03, 5.0)
        forwardUntilDone(15.0, 0.03, 5.0)
        turnUntilDone(30.0 * sign, 0.02, 10.0)
        robot.alignAndPlace(8.0)
        forwardUntilDone(-10.0, 0.02, 3.0)
        delay(1000L)
        robot.slides.targetPosition = 0.0
      }


    }
  }

  override fun onUpdate(scope: CoroutineScope){
  }

}
