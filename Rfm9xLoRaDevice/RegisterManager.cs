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
namespace devMobile.IoT.NetMF.SPI
{
   using System;
   using System.Text;
   using System.Threading;
   using Microsoft.SPOT;
   using Microsoft.SPOT.Hardware;

   public sealed class RegisterManager
   {
      private const byte RegisterAddressReadMask = 0X7f;
      private const byte RegisterAddressWriteMask = 0x80;
      private SPI Device = null;

      public RegisterManager(SPI.SPI_module spiModule, Cpu.Pin chipSelect)
      {
         this.Device = new SPI(new SPI.Configuration(chipSelect, false, 0, 0, false, false, 500, spiModule));

         Thread.Sleep(100);
      }

      public Byte ReadByte(byte registerAddress)
      {
         byte[] writeBuffer = new byte[] { registerAddress };
         byte[] readBuffer = new byte[1];
         Debug.Assert(Device != null);

         Device.WriteRead(writeBuffer, readBuffer, 1);

         return readBuffer[0];
      }

      public ushort ReadWord(byte address)
      {
         byte[] writeBuffer = new byte[] { address &= RegisterAddressReadMask };
         byte[] readBuffer = new byte[2];
         Debug.Assert(Device != null);

         readBuffer[0] = this.ReadByte(address);
         readBuffer[1] = this.ReadByte(address += 1);

         return (ushort)(readBuffer[1] + (readBuffer[0] << 8));
      }

      public byte[] Read(byte address, int length)
      {
         byte[] writeBuffer = new byte[] { address &= RegisterAddressReadMask };
         byte[] readBuffer = new byte[length];
         Debug.Assert(Device != null);

         for (byte index = 0; index < length; index++)
         {
            readBuffer[index] = this.ReadByte(address += 1);
         }

         return readBuffer;
      }

      public void WriteByte(byte address, byte value)
      {
         byte[] writeBuffer = new byte[] { address |= RegisterAddressWriteMask, value };
         Debug.Assert(Device != null);

         Device.Write(writeBuffer);
      }

      public void WriteWord(byte address, ushort value)
      {
         byte[] valueBytes = BitConverter.GetBytes(value);
         byte[] writeBuffer = new byte[] { address |= RegisterAddressWriteMask, valueBytes[0], valueBytes[1] };
         Debug.Assert(Device != null);

         Device.Write(writeBuffer);
      }

      public void Write(byte address, byte[] bytes)
      {
         byte[] writeBuffer = new byte[1 + bytes.Length];
         Debug.Assert(Device != null);

         Array.Copy(bytes, 0, writeBuffer, 1, bytes.Length);
         writeBuffer[0] = address |= RegisterAddressWriteMask;

         Device.Write(writeBuffer);
      }

      public void RegisterDump(byte start, byte finish)
      {
         for (byte registerIndex = start; registerIndex <= finish; registerIndex++)
         {
            byte registerValue = this.ReadByte(registerIndex);

            Debug.Print("Register 0x" + ByteToHexString(registerIndex) + " - Value 0X" + ByteToHexString(registerValue));
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
