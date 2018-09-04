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
namespace devMobile.IoT.NetMF.Rfm9X.Client
{
   using System;
   using System.Text;
   using System.Threading;
   using devMobile.IoT.NetMF.ISM;
   using Microsoft.SPOT;
   using SecretLabs.NETMF.Hardware.Netduino;

   public class Program
   {
      public static void Main()
      {
         Rfm9XDevice rfm9XDevice = new Rfm9XDevice(Pins.GPIO_PIN_D10, Pins.GPIO_PIN_D9, Pins.GPIO_PIN_D2);
         byte MessageCount = Byte.MinValue;

         rfm9XDevice.Initialise( frequency:915000000, paBoost: true, rxPayloadCrcOn: true);
#if ADDRESSED_MESSAGES_PAYLOAD 
         rfm9XDevice.Receive(Encoding.UTF8.GetBytes("Netduino"));
#else
         rfm9XDevice.Receive();
#endif
         rfm9XDevice.OnDataReceived += rfm9XDevice_OnDataReceived;
         rfm9XDevice.OnTransmit += rfm9XDevice_OnTransmit;

         while (true)
         {
            string messageText = "Hello NetMF LoRa! " + MessageCount.ToString();
            MessageCount += 1;
            byte[] messageBytes = UTF8Encoding.UTF8.GetBytes(messageText);
            Debug.Print("Sending " + messageBytes.Length + " bytes message " + messageText);

#if ADDRESSED_MESSAGES_PAYLOAD 
            rfm9XDevice.Send(UTF8Encoding.UTF8.GetBytes("LoRaIoT1"), messageBytes);
#else
            rfm9XDevice.Send(messageBytes);
#endif
            Thread.Sleep(10000);
         }
      }

      static void rfm9XDevice_OnTransmit()
      {
         Debug.Print("Transmit-Done");
      }


#if ADDRESSED_MESSAGES_PAYLOAD 
      static void rfm9XDevice_OnDataReceived(byte[] address, float packetSnr, int packetRssi, int rssi, byte[] data)
#else
      static void rfm9XDevice_OnDataReceived(float packetSnr, int packetRssi, int rssi,  byte[] data)
#endif
      {
         try
         {
            string messageText = new string(UTF8Encoding.UTF8.GetChars(data));
#if ADDRESSED_MESSAGES_PAYLOAD 
            string addressText = new string(UTF8Encoding.UTF8.GetChars(address));

            Debug.Print(DateTime.UtcNow.ToString("HH:MM:ss") + "-From " + addressText + " PacketSnr " + packetSnr.ToString("F1") + " Packet RSSI " + packetRssi + "dBm RSSI " + rssi + "dBm = " + data.Length + " byte message " + @"""" + messageText + @"""") ;
#else
            Debug.Print(DateTime.UtcNow.ToString("HH:MM:ss") + "-Rfm9X PacketSnr " + packetSnr.ToString("F1") + " Packet RSSI " + packetRssi + "dBm RSSI " + rssi + "dBm = " + data.Length + " byte message " + @"""" + messageText + @"""") ;
#endif
         }
         catch (Exception ex)
         {
            Debug.Print(ex.Message);
         }
      }
   }
}

namespace System.Diagnostics
{
   public enum DebuggerBrowsableState
   {
      Never = 0,
      Collapsed = 2,
      RootHidden = 3
   }
}
