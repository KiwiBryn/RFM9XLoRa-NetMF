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
namespace devMobile.IoT.NetMF.Rfm9X.ReceiveBasic
{
   using System;
   using System.Text;
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

         this.Rfm9XLoraModem = new SPI(new SPI.Configuration(chipSelect, false, 0, 0, false, false, 2000, SPI.SPI_module.SPI1));

         Thread.Sleep(100);
      }

      public Byte RegisterReadByte(byte registerAddress)
      {
         byte[] writeBuffer = new byte[] { registerAddress };
         byte[] readBuffer = new byte[1];
         Debug.Assert(Rfm9XLoraModem != null);

         Rfm9XLoraModem.WriteRead(writeBuffer, readBuffer, 1);

         return readBuffer[0];
      }

      public ushort RegisterReadWord(byte address)
      {
         byte[] writeBuffer = new byte[] { address &= RegisterAddressReadMask };
         byte[] readBuffer = new byte[2];
         Debug.Assert(Rfm9XLoraModem != null);

         readBuffer[0] = RegisterReadByte(address);
         readBuffer[1] = RegisterReadByte(address += 1);

         return (ushort)(readBuffer[1] + (readBuffer[0] << 8));
      }

      public byte[] RegisterRead(byte address, int length)
      {
         byte[] writeBuffer = new byte[] { address &= RegisterAddressReadMask };
         byte[] readBuffer = new byte[length];
         Debug.Assert(Rfm9XLoraModem != null);

         for (byte index = 0; index < length; index++)
         {
            readBuffer[index] = RegisterReadByte(address += 1);
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

            // Put device into LoRa + Sleep mode
            rfm9XDevice.RegisterWriteByte(0x01, 0x80); // RegOpMode 

            // Set the frequency to 915MHz
            byte[] frequencyWriteBytes = { 0xE4, 0xC0, 0x00 }; // RegFrMsb, RegFrMid, RegFrLsb
            rfm9XDevice.RegisterWrite(0x06, frequencyWriteBytes);

            rfm9XDevice.RegisterWriteByte(0x0F, 0x0); // RegFifoRxBaseAddress 

            rfm9XDevice.RegisterWriteByte(0x01, 0x85); // RegOpMode set LoRa & RxContinuous

            while (true)
            {
               // Wait until a packet is received, no timeouts in PoC
               Debug.Print("Receive-Wait");
               byte IrqFlags = rfm9XDevice.RegisterReadByte(0x12); // RegIrqFlags
               while ((IrqFlags & 0x40) == 0)  // wait until RxDone cleared
               {
                  Thread.Sleep(100);
                  IrqFlags = rfm9XDevice.RegisterReadByte(0x12); // RegIrqFlags
                  Debug.Print(".");
               }
               Debug.Print("");

               Debug.Print("Receive-Message");
               byte currentFifoAddress = rfm9XDevice.RegisterReadByte(0x10); // RegFifiRxCurrent
               rfm9XDevice.RegisterWriteByte(0x0d, currentFifoAddress); // RegFifoAddrPtr

               byte numberOfBytes = rfm9XDevice.RegisterReadByte(0x13); // RegRxNbBytes

               // Allocate buffer for message
               byte[] messageBytes = new byte[numberOfBytes];

               for (int i = 0; i < numberOfBytes; i++)
               {
                  messageBytes[i] = rfm9XDevice.RegisterReadByte(0x00); // RegFifo
               }
               string messageText = new string(UTF8Encoding.UTF8.GetChars(messageBytes));
               Debug.Print("Received " + messageBytes.Length + " byte message " + messageText);

               rfm9XDevice.RegisterWriteByte(0x12, 0x40); // RegIrqFlags clear RxDone bit

               Debug.Print("Receive-Done");
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

