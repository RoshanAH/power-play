package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.core.BaseOpmode
import org.firstinspires.ftc.teamcode.robots.Jaws
import kotlinx.coroutines.*

@Autonomous
class Park : BaseOpmode(){

  val robot = Jaws()

  override fun setRobot() = robot

  override fun onInit(scope: CoroutineScope){
    robot.camera.apply{
      startCamera()
      mount.setPosition(0.45)
      webcam.setPipeline(signalPipeline)
      theta = signalTheta

      scope.launch{
        telemetry.addLine("Launching camera...")
        telemetry.update()
        while(!cameraRunning) yield()   
        while(opModeInInit()){
          telemetry.addData("Signal Value", signal)
          telemetry.update()
        }
      }
    }

    robot.camera.mount.setPosition(0.56)
    robot.slides.claw.setPosition(robot.slides.close)
    robot.slides.close()
  }

  override fun onStart(scope: CoroutineScope){
    scope.launch{
      while(!robot.camera.cameraRunning) yield()
      val signal = robot.camera.signalPipeline.signalVal

      telemetry.addData("Signal Value", signal)
      telemetry.update()

      robot.drivetrain.apply{
        forwardUntilDone(35.0, 0.02, 5.0)
        strafeUntilDone(35.0 * (signal - 2), 0.03, 5.0)
      }
    }
  }

  override fun onUpdate(scope: CoroutineScope){
  }

}
