//---------------------------------------------------------------------------------
// Copyright (c) August 2018, devMobile Software
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
//---------------------------------------------------------------------------------
namespace devMobile.NetMF.Rfm9X.RegisterRead
{
   using System;
   using System.Threading;
   using Microsoft.SPOT;
   using Microsoft.SPOT.Hardware;

   public class Program
   {
      public static void Main()
      {
         OutputPort reset = new OutputPort(Pins.GPIO_PIN_D9, true);
         uint frequency = 100;
         bool ChipSelectActiveState = false;
         uint ChipSelectSetupTime = 0;
         uint ChipSelectHoldTime = 0;
         bool ClokIdelState = false;
         bool ClockEdge = true;

         for (frequency = 50; frequency <= 500; frequency = frequency + 50)
         {
            for (ChipSelectSetupTime = 0; ChipSelectSetupTime <= 50; ChipSelectSetupTime = ChipSelectSetupTime + 5)
            {
               for (ChipSelectHoldTime = 0; ChipSelectHoldTime <= 50; ChipSelectHoldTime = ChipSelectHoldTime + 5)
               {
                  for (int ChipSelectActiveStateCount = 0; ChipSelectActiveStateCount < 2; ChipSelectActiveStateCount++)
                  {
                     for (int ClokIdelStateCount = 0; ClokIdelStateCount < 2; ClokIdelStateCount++)
                     {
                        for (int ClockEdgeCount = 0; ClockEdgeCount < 2; ClockEdgeCount++)
                        {
                           reset.Write(false);
                           Thread.Sleep(10);
                           reset.Write(true);
                           Thread.Sleep(10);

                           using (SPI spiPort = new SPI(new SPI.Configuration(Pins.GPIO_PIN_D10, ChipSelectActiveState, ChipSelectSetupTime, ChipSelectHoldTime, ClokIdelState, ClockEdge, frequency, SPI.SPI_module.SPI1)))
                           {
                              Thread.Sleep(50);

                              byte[] writeBuffer = new byte[] { 0x42 };
                              byte[] readBuffer = new byte[1];

                              spiPort.WriteRead(writeBuffer, readBuffer);
                              Debug.Print("regVerion = " + BytesToHexString(readBuffer));

                              Debug.Print(ChipSelectActiveState.ToString() + ' ' + ChipSelectSetupTime.ToString() + ' ' + ChipSelectHoldTime.ToString() + ' ' + ClokIdelState.ToString() + " " + ClockEdge.ToString() + ' ' + frequency.ToString());

                              //if (readBuffer[0] == 0x12)
                              if (readBuffer[0] != 0x0)
                              {
                                 Thread.Sleep(Timeout.Infinite);
                              }

                              spiPort.Dispose();
                           }
                           ClockEdge = !ClockEdge;
                        }
                        ClokIdelState = !ClokIdelState;
                     }
                     ChipSelectActiveState = !ChipSelectActiveState;
                  }
               }
            }
         }

         /*

         Thread.Sleep(100);

         while (true)
         {
            byte[] writeBuffer = new byte[] { 0x42 };
            byte[] readBuffer = new byte[1];

            spiPort.WriteRead(writeBuffer, readBuffer);

            Debug.Print("regVerion = " + BytesToHexString(readBuffer));

            Thread.Sleep(1000);
         }
          */
      }


      private static string BytesToHexString(byte[] bytes)
      {
         string hexString = string.Empty;

         // Create a character array for hexidecimal conversion.
         const string hexChars = "0123456789ABCDEF";

         // Loop through the bytes.
         for (byte b = 0; b < bytes.Length; b++)
         {
            if (b > 0)
               hexString += "-";

            // Grab the top 4 bits and append the hex equivalent to the return string.        
            hexString += hexChars[bytes[b] >> 4];

            // Mask off the upper 4 bits to get the rest of it.
            hexString += hexChars[bytes[b] & 0x0F];
         }

         return hexString;
      }
   }
}
