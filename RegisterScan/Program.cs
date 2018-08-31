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
namespace devMobile.IoT.NetMF.Rfm9X.RegisterScan
{
   using System;
   using System.Threading;
   using Microsoft.SPOT;
   using Microsoft.SPOT.Hardware;
   using SecretLabs.NETMF.Hardware.Netduino;

   public sealed class Rfm9XDevice
   {
      private SPI rfm9XLoraModem = null;

      public Rfm9XDevice(Cpu.Pin chipSelect)
      {
         this.rfm9XLoraModem = new SPI(new SPI.Configuration(chipSelect, false, 0, 0, false, true, 500, SPI.SPI_module.SPI1));

         Thread.Sleep(100);
      }

      public Byte RegisterReadByte(byte registerAddress)
      {
         byte[] writeBuffer = new byte[] { registerAddress };
         byte[] readBuffer = new byte[1];
         Debug.Assert(rfm9XLoraModem != null);

         rfm9XLoraModem.WriteRead(writeBuffer, readBuffer, 1);

         return readBuffer[0];
      }
   }

   public class Program
   {
      public static void Main()
      {
         Rfm9XDevice rfm9XDevice = new Rfm9XDevice(Pins.GPIO_PIN_D10);

         while (true)
         {
            Debug.Print("---Registers 0x00 thru 0x42---");
            for (byte registerIndex = 0; registerIndex <= 0x42; registerIndex++)
            {
               byte registerValue = rfm9XDevice.RegisterReadByte(registerIndex);

               Debug.Print("Register 0x" + ByteToHexString(registerIndex) + " - Value 0X" + ByteToHexString(registerValue));
            }

            Thread.Sleep(10000);
         }
      }


      private static string ByteToHexString(byte singlebyte)
      {
         string hexString = string.Empty;

         // Create a character array for hexidecimal conversion.
         const string hexChars = "0123456789ABCDEF";

         // Grab the top 4 bits and append the hex equivalent to the return string.        
         hexString += hexChars[singlebyte >> 4];

         // Mask off the upper 4 bits to get the rest of it.
         hexString += hexChars[singlebyte & 0x0F];

         return hexString;
      }
   }
}
