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
namespace devMobile.IoT.NetMF.Rfm9X.RefactorRegisterManager
{
   using System;
   using System.Runtime.CompilerServices;
   using System.Text;
   using System.Threading;
   using devMobile.IoT.NetMF.SPI;
   using Microsoft.SPOT;
   using Microsoft.SPOT.Hardware;
   using SecretLabs.NETMF.Hardware.Netduino;

   public sealed class Rfm9XDevice
   {
      public RegisterManager Rfm9XLoraModem = null; // FIx this later
      private OutputPort ResetGpioPin = null;
      private InterruptPort InterruptPin = null;

      public Rfm9XDevice(Cpu.Pin chipSelect, Cpu.Pin resetPin, Cpu.Pin interruptPin)
      {
         this.Rfm9XLoraModem = new RegisterManager(chipSelect);

         // Factory reset pin configuration
         ResetGpioPin = new OutputPort(Pins.GPIO_PIN_D9, true);
         ResetGpioPin.Write(false);
         Thread.Sleep(10);
         ResetGpioPin.Write(true);
         Thread.Sleep(10);

         InterruptPin = new InterruptPort(interruptPin, false, Port.ResistorMode.Disabled, Port.InterruptMode.InterruptEdgeHigh);

         InterruptPin.OnInterrupt += InterruptPin_OnInterrupt;

         Thread.Sleep(100);
      }

      public void RegisterDump()
      {
         Debug.Print("---Registers 0x00 thru 0x42---");
         Rfm9XLoraModem.RegisterDump(0x00, 0x42);
      }

      [MethodImpl( MethodImplOptions.Synchronized)] 
      void InterruptPin_OnInterrupt(uint data1, uint data2, DateTime time)
      {
         byte IrqFlags = this.Rfm9XLoraModem.ReadByte(0x12); // RegIrqFlags
         //Debug.Print("RegIrqFlags " + ByteToHexString(IrqFlags));

         if ((IrqFlags & 0x40) == 0x40)  // RxDone
         {
            //Debug.Print("Receive-Message");
            byte currentFifoAddress = this.Rfm9XLoraModem.ReadByte(0x10); // RegFifiRxCurrent
            this.Rfm9XLoraModem.WriteByte(0x0d, currentFifoAddress); // RegFifoAddrPtr

            byte numberOfBytes = this.Rfm9XLoraModem.ReadByte(0x13); // RegRxNbBytes

            // Allocate buffer for message
            byte[] messageBytes = new byte[numberOfBytes];

            for (int i = 0; i < numberOfBytes; i++)
            {
               messageBytes[i] = this.Rfm9XLoraModem.ReadByte(0x00); // RegFifo
            }

            string messageText = new string(UTF8Encoding.UTF8.GetChars(messageBytes));
            Debug.Print("Received " + messageBytes.Length.ToString() + " byte message " + messageText);
         }

         if ((IrqFlags & 0x08) == 0x08)  // TxDone
         {
            this.Rfm9XLoraModem.WriteByte(0x01, 0x85); // RegOpMode set LoRa & RxContinuous
            Debug.Print("Transmit-Done");
         }

         this.Rfm9XLoraModem.WriteByte(0x40, 0x0);
         this.Rfm9XLoraModem.WriteByte(0x12, 0xff);// RegIrqFlags
      }


      public class Program
      {
         public static void Main()
         {
            Rfm9XDevice rfm9XDevice = new Rfm9XDevice(Pins.GPIO_PIN_D10, Pins.GPIO_PIN_D9, Pins.GPIO_PIN_D2);
            byte MessageCount = Byte.MinValue;

            // Put device into LoRa + Sleep mode
            rfm9XDevice.Rfm9XLoraModem.WriteByte(0x01, 0x80); // RegOpMode 

            // Set the frequency to 915MHz
            byte[] frequencyWriteBytes = { 0xE4, 0xC0, 0x00 }; // RegFrMsb, RegFrMid, RegFrLsb
            rfm9XDevice.Rfm9XLoraModem.Write(0x06, frequencyWriteBytes);

            rfm9XDevice.Rfm9XLoraModem.WriteByte(0x0F, 0x0); // RegFifoRxBaseAddress 

            // More power - PA_BOOST
            rfm9XDevice.Rfm9XLoraModem.WriteByte(0x09, 0x80); // RegPaConfig

            //rfm9XDevice.RegisterWriteByte(0x40, 0x0);

            rfm9XDevice.Rfm9XLoraModem.WriteByte(0x01, 0x85); // RegOpMode set LoRa & RxContinuous


            while (true)
            {
               rfm9XDevice.Rfm9XLoraModem.WriteByte(0x0E, 0x0); // RegFifoTxBaseAddress 

               // Set the Register Fifo address pointer
               rfm9XDevice.Rfm9XLoraModem.WriteByte(0x0D, 0x0); // RegFifoAddrPtr 

               string messageText = "Hello NetMF LoRa! " + MessageCount.ToString();
               MessageCount += 1;

               // load the message into the fifo
               byte[] messageBytes = UTF8Encoding.UTF8.GetBytes(messageText);
               foreach (byte b in messageBytes)
               {
                  rfm9XDevice.Rfm9XLoraModem.WriteByte(0x0, b); // RegFifo
               }

               // Set the length of the message in the fifo
               rfm9XDevice.Rfm9XLoraModem.WriteByte(0x22, (byte)messageBytes.Length); // RegPayloadLength

               rfm9XDevice.Rfm9XLoraModem.WriteByte(0x40, 0x40); // RegDioMapping1 

               /// Set the mode to LoRa + Transmit
               rfm9XDevice.Rfm9XLoraModem.WriteByte(0x01, 0x83); // RegOpMode 
               Debug.Print("Sending " + messageBytes.Length + " bytes message " + messageText);

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
