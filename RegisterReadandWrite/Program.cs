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
namespace devMobile.IoT.NetMF.Rfm9X.RegisterReadandWrite
{
   using System;
   using System.Threading;
   using Microsoft.SPOT;
   using Microsoft.SPOT.Hardware;
   using SecretLabs.NETMF.Hardware.Netduino;

   public sealed class Rfm9XDevice
   {
      private const byte RegisterAddressReadMask = 0X7f;
      private const byte RegisterAddressWriteMask = 0x80;

      private SPI Rfm9XLoraModem = null;
      private OutputPort ResetGpioPin = null;

      public Rfm9XDevice(Cpu.Pin chipSelect, Cpu.Pin reset)
      {
         // Factory reset pin configuration
         ResetGpioPin = new OutputPort(Pins.GPIO_PIN_D9, true);
         ResetGpioPin.Write(false);
         Thread.Sleep(10);
         ResetGpioPin.Write(true);
         Thread.Sleep(10);

         //this.Rfm9XLoraModem = new SPI(new SPI.Configuration(chipSelect, false, 0, 0, false, true, 2000, SPI.SPI_module.SPI1));
         this.Rfm9XLoraModem = new SPI(new SPI.Configuration(chipSelect, false, 0, 0, false, false, 2000, SPI.SPI_module.SPI1));

         Thread.Sleep(100);
      }

      public Byte RegisterReadByte(byte registerAddress)
      {
         byte[] writeBuffer = new byte[] { registerAddress };
         byte[] readBuffer = new byte[1];
         Debug.Assert(Rfm9XLoraModem != null);

         //Rfm9XLoraModem.WriteRead(writeBuffer, readBuffer, 1);
         Rfm9XLoraModem.WriteRead(writeBuffer, readBuffer, 1 );

         return readBuffer[0];
      }

      public ushort RegisterReadWord(byte address)
      {
         byte[] writeBuffer = new byte[] { address &= RegisterAddressReadMask };
         byte[] readBuffer = new byte[2];
         Debug.Assert(Rfm9XLoraModem != null);

         //Rfm9XLoraModem.WriteRead(readBuffer, writeBuffer, 4 ); // Check this

         readBuffer[0] = RegisterReadByte( address ) ;
         readBuffer[1] = RegisterReadByte(address+=1);

         return (ushort)(readBuffer[1] + (readBuffer[0] << 8));
      }

      public byte[] RegisterRead(byte address, int length)
      {
         byte[] writeBuffer = new byte[] { address &= RegisterAddressReadMask };
         byte[] readBuffer = new byte[length];
         Debug.Assert(Rfm9XLoraModem != null);

         //Rfm9XLoraModem.WriteRead(readBuffer, writeBuffer, 1); // Check this

         for (byte index = 0; index < length; index++)
         {
            readBuffer[index] = RegisterReadByte(address+=1 );
         }

         return readBuffer;
      }
     
      public void RegisterWriteByte(byte address, byte value)
      {
         byte[] writeBuffer = new byte[] { address |= RegisterAddressWriteMask, value };
         Debug.Assert(Rfm9XLoraModem != null);

         Rfm9XLoraModem.Write(writeBuffer);
      }

      public void RegisterWriteWord(byte address, ushort value)
      {
         byte[] valueBytes = BitConverter.GetBytes(value);
         byte[] writeBuffer = new byte[] { address |= RegisterAddressWriteMask, valueBytes[0], valueBytes[1] };
         Debug.Assert(Rfm9XLoraModem != null);

         Rfm9XLoraModem.Write(writeBuffer);
      }

      public void RegisterWrite(byte address, byte[] bytes)
      {
         byte[] writeBuffer = new byte[1 + bytes.Length];
         Debug.Assert(Rfm9XLoraModem != null);

         Array.Copy(bytes, 0, writeBuffer, 1, bytes.Length);
         writeBuffer[0] = address |= RegisterAddressWriteMask;

         Rfm9XLoraModem.Write(writeBuffer);
      }

      public void RegisterDump()
      {
         Debug.Print("---Registers 0x00 thru 0x42---");
         for (byte registerIndex = 0; registerIndex <= 0x42; registerIndex++)
         {
            byte registerValue = this.RegisterReadByte(registerIndex);

            Debug.Print("Register 0x" + ByteToHexString(registerIndex) + " - Value 0X" + ByteToHexString(registerValue));
         }
      }

      public class Program
      {
         public static void Main()
         {
            Rfm9XDevice rfm9XDevice = new Rfm9XDevice(Pins.GPIO_PIN_D10, Pins.GPIO_PIN_D9);

            while (true)
            {
               rfm9XDevice.RegisterDump();

               Debug.Print("Read RegOpMode (read byte)");
               Byte regOpMode = rfm9XDevice.RegisterReadByte(0x1);
               Debug.Print("RegOpMode 0x" + ByteToHexString(regOpMode));

               Debug.Print("Set LoRa mode and sleep mode (write byte)");
               rfm9XDevice.RegisterWriteByte(0x01, 0x80); // 10000000

               Debug.Print("Read the preamble (read word)");
               ushort preamble = rfm9XDevice.RegisterReadWord(0x20);
               Debug.Print("Preamble 0x" + WordToHexString(preamble));

               Debug.Print("Set the preamble to 0x80 (write word)");
               rfm9XDevice.RegisterWriteWord(0x20, 0x80);

               Debug.Print("Read the centre frequency (read byte array)");
               byte[] frequencyReadBytes = rfm9XDevice.RegisterRead(0x06, 3);
               Debug.Print("Frequency Msb 0x" + ByteToHexString(frequencyReadBytes[0]) + " Mid 0x" + ByteToHexString(frequencyReadBytes[1]) + " Lsb 0x" + ByteToHexString(frequencyReadBytes[2]));

               Debug.Print("Set the centre frequency to 916MHz (write byte array)");
               byte[] frequencyWriteBytes = { 0xE4, 0xC0, 0x00 };
               rfm9XDevice.RegisterWrite(0x06, frequencyWriteBytes);

               rfm9XDevice.RegisterDump();

               Thread.Sleep(10000);
            }
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

      private static string WordToHexString(ushort singleword)
      {
         string hexString = string.Empty;

         byte[] bytes = BitConverter.GetBytes(singleword);

         hexString += ByteToHexString(bytes[1]);

         hexString += ByteToHexString(bytes[0]);

         return hexString;
      }

   }
}
