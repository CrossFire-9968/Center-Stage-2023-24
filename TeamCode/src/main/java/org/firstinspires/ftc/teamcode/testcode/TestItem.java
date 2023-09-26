package org.firstinspires.ftc.teamcode.testcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
   This is an abstract class for defining a selectable test item that will show up on the driver station. The purpose
   is to enable the team to test individual components or assemblies to troubleshoot hardware and wiring.
*/
abstract public class TestItem
{
   // Context that will show up on the display
   private String description;

   protected TestItem(String description)
   {
      this.description = description;
   }

   public String getDescription()
   {
      return this.description;
   }

   abstract public void run(boolean runTest, Telemetry testTelemetry);
}
