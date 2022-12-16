package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.util.ReadWriteFile
import org.firstinspires.ftc.robotcore.internal.system.AppUtil

fun readFile(name: String): String{ 
  val file = AppUtil.getInstance().getSettingsFile(name)
  return ReadWriteFile.readFile(file)
}

fun writeFile(name: String, content: String){
  val file = AppUtil.getInstance().getSettingsFile(name)
  return ReadWriteFile.writeFile(file, content)
}
