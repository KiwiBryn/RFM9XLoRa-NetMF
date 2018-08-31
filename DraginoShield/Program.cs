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
namespace devMobile.NetMF.Rfm9X.DraginoShield
{
   using System;
   using System.Threading;
   using Microsoft.SPOT;
   using Microsoft.SPOT.Hardware;
   using SecretLabs.NETMF.Hardware;
   using SecretLabs.NETMF.Hardware.Netduino;

   public class Program
   {

      public static void Main()
      {
         //OutputPort reset = new OutputPort(Pins.GPIO_PIN_D9, true);
         OutputPort chipSelect = null;
         //chipSelect = new OutputPort(Pins.GPIO_PIN_D10, true);
         //SPI spiPort = new SPI(new SPI.Configuration(Pins.GPIO_NONE, false, 0, 0, true, true, 500, SPI.SPI_module.SPI1));
         SPI spiPort = new SPI(new SPI.Configuration(Pins.GPIO_PIN_D10, false, 0, 0, false, true, 500, SPI.SPI_module.SPI1));

         Thread.Sleep(100);

         while (true)
         {
            //byte[] writeBuffer = new byte[] { 0x42 }; // RegVersion exptecing 0x12
            byte[] writeBuffer = new byte[] { 0x06 }; // RegFreqMsb expecting 0x6C
            //byte[] writeBuffer = new byte[] { 0x07 }; // RegFreqMid expecting 0x80
            //byte[] writeBuffer = new byte[] { 0x08 }; // RegFreqLsb expecting 0x00
            byte[] readBuffer = new byte[1];

            if (chipSelect != null)
            {
               chipSelect.Write(false);
            }
            spiPort.WriteRead(writeBuffer, readBuffer,1);
            if (chipSelect != null)
            {
               chipSelect.Write(true);
            }

            Debug.Print("Value = 0x" + BytesToHexString(readBuffer));

            Thread.Sleep(1000);
         }
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



/*
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
namespace devMobile.NetMF.Rfm9X.DraginoShield
{
   using System;
   using System.Threading;
   using Microsoft.SPOT;
   using Microsoft.SPOT.Hardware;
   using SecretLabs.NETMF.Hardware;
   using SecretLabs.NETMF.Hardware.Netduino;

   public class Program
   {

      public static void Main()
      {
         OutputPort reset = new OutputPort(Pins.GPIO_PIN_D9, true);
         OutputPort chipSelect = null;
         chipSelect = new OutputPort(Pins.GPIO_PIN_D10, true);
         //SPI spiPort = new SPI(new SPI.Configuration(Pins.GPIO_PIN_D10, false, 0, 0, false, true, 500, SPI.SPI_module.SPI1));
         SPI spiPort = new SPI(new SPI.Configuration(Pins.GPIO_NONE, false, 0, 0, true, true, 500, SPI.SPI_module.SPI1));
         //Thread.Sleep(100);

         spiPort.Write(new byte[] { 0x00 });

         while (true)
         {
            byte[] writeBuffer = new byte[] { 0x42,0 }; // RegVersion
            byte[] readBuffer = new byte[1];

            if (chipSelect != null)
            {
               chipSelect.Write(false);
            }
            spiPort.WriteRead(writeBuffer, readBuffer,1);
            if (chipSelect != null)
            {
               chipSelect.Write(true);
            }

            Debug.Print("regVerion = 0x" + BytesToHexString(readBuffer));

            Thread.Sleep(1000);
         }
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



/*
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
namespace devMobile.NetMF.Rfm9X.DraginoShield
{
   using System;
   using System.Threading;
   using Microsoft.SPOT;
   using Microsoft.SPOT.Hardware;
   using SecretLabs.NETMF.Hardware;
   using SecretLabs.NETMF.Hardware.Netduino;

   public class Program
   {

      public static void Main()
      {
         OutputPort reset = new OutputPort(Pins.GPIO_PIN_D9, true);
         OutputPort chipSelect = null;
         chipSelect = new OutputPort(Pins.GPIO_PIN_D10, true);
         //SPI spiPort = new SPI(new SPI.Configuration(Pins.GPIO_PIN_D10, false, 0, 0, false, true, 500, SPI.SPI_module.SPI1));
         SPI spiPort = new SPI(new SPI.Configuration(Pins.GPIO_NONE, false, 0, 0, true, true, 500, SPI.SPI_module.SPI1));
         //Thread.Sleep(100);

         spiPort.Write(new byte[] { 0x00 });

         while (true)
         {
            byte[] writeBuffer = new byte[] { 0x42,0 }; // RegVersion
            byte[] readBuffer = new byte[1];

            if (chipSelect != null)
            {
               chipSelect.Write(false);
            }
            spiPort.WriteRead(writeBuffer, readBuffer,1);
            if (chipSelect != null)
            {
               chipSelect.Write(true);
            }

            Debug.Print("regVerion = 0x" + BytesToHexString(readBuffer));

            Thread.Sleep(1000);
         }
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



/*
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
namespace devMobile.NetMF.Rfm9X.DraginoShield
{
   using System;
   using System.Threading;
   using Microsoft.SPOT;
   using Microsoft.SPOT.Hardware;
   using SecretLabs.NETMF.Hardware;
   using SecretLabs.NETMF.Hardware.Netduino;

   public class Program
   {

      public static void Main()
      {
         OutputPort reset = new OutputPort(Pins.GPIO_PIN_D9, true);
         OutputPort chipSelect = null;
         chipSelect = new OutputPort(Pins.GPIO_PIN_D10, true);
         //SPI spiPort = new SPI(new SPI.Configuration(Pins.GPIO_PIN_D10, false, 0, 0, false, true, 500, SPI.SPI_module.SPI1));
         SPI spiPort = new SPI(new SPI.Configuration(Pins.GPIO_NONE, false, 0, 0, true, true, 500, SPI.SPI_module.SPI1));
         //Thread.Sleep(100);

         spiPort.Write(new byte[] { 0x00 });

         while (true)
         {
            byte[] writeBuffer = new byte[] { 0x42,0 }; // RegVersion
            byte[] readBuffer = new byte[1];

            if (chipSelect != null)
            {
               chipSelect.Write(false);
            }
            spiPort.WriteRead(writeBuffer, readBuffer,1);
            if (chipSelect != null)
            {
               chipSelect.Write(true);
            }

            Debug.Print("regVerion = 0x" + BytesToHexString(readBuffer));

            Thread.Sleep(1000);
         }
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
 */
