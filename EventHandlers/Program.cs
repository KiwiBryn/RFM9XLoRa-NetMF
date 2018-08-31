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
namespace devMobile.IoT.NetMF.Rfm9X.EventHandlers
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
      public delegate void EventHandler();
      public delegate void OnDataRecievedHandler(byte[] data);

      // Registers from SemTech SX127X Datasheet
      private enum Registers : byte
      {
         MinValue = RegOpMode,
         RegFifo = 0x0,
         RegOpMode = 0x01,
         //Reserved 0x02-0x06 
         RegFrMsb = 0x06,
         RegFrMid = 0x7,
         RegFrLsb = 0x08,
         RegPAConfig = 0x09,
         //RegPARamp = 0x0A, // not inlcuded as FSK/OOK functionality
         RegOcp = 0x0B,
         RegLna = 0x0C,
         RegFifoAddrPtr = 0x0D,
         //RegFifoTxBaseAddr = 0x0E
         RegFifoRxCurrent = 0x10,
         RegIrqFlagsMask = 0x11,
         RegIrqFlags = 0x12,
         RegRxNbBytes = 0x13,
         // RegRxHeaderCnValueMsb=0x14
         // RegRxHeaderCnValueLsb=0x15
         // RegRxPacketCntValueMsb=0x16
         // RegRxPacketCntValueMsb=0x17
         // RegModemStat=0x18
         // RegPktSnrVale=0x19
         // RegPktRssiValue=0x1A
         // RegRssiValue=0x1B
         // RegHopChannel=0x1C
         RegModemConfig1 = 0x1D,
         RegModemConfig2 = 0x1E,
         RegSymbTimeout = 0x1F,
         RegPreambleMsb = 0x20,
         RegPreambleLsb = 0x21,
         RegPayloadLength = 0x22,
         RegMaxPayloadLength = 0x23,
         RegHopPeriod = 0x24,
         // RegFifiRxByteAddr = 0x25
         RegModemConfig3 = 0x26,
         RegPpmCorrection = 0x27,
         // RegFeiMsb = 0x28
         // RegFeiMid = 0x29
         // RegFeiLsb = 0x2A
         // Reserved 0x2B
         // RegRssiWideband = 0x2C
         // Reserved 0x2D-0x30
         RegDetectOptimize = 0x31,
         // Reserved 0x32
         RegInvertIQ = 0x33,
         // Reserved 0x34-0x36
         RegDetectionThreshold = 0x37,
         // Reserved 0x38
         RegSyncWord = 0x39,
         RegDioMapping1 = 0x40,
         RegVersion = 0x42,

         MaxValue = RegVersion,
      }

      // RegOpMode mode flags
      private const byte RegOpModeLongRangeModeLoRa = 0x80; // 0b10000000;
      private const byte RegOpModeLongRangeModeFskOok = 0x00; //0b00000000;
      private const byte RegOpModeLongRangeModeDefault = RegOpModeLongRangeModeFskOok;

      private const byte RegOpModeAcessSharedRegLoRa = 0x00; // 0b00000000;
      private const byte RegOpModeAcessSharedRegFsk = 0x40; // 0b01000000;
      private const byte RegOpModeAcessSharedRegDefault = RegOpModeAcessSharedRegLoRa;

      private const byte RegOpModeLowFrequencyModeOnHighFrequency = 0x00; // 0b00000000;
      private const byte RegOpModeLowFrequencyModeOnLowFrequency = 0x08; // 0b00001000;
      private const byte RegOpModeLowFrequencyModeOnDefault = RegOpModeLowFrequencyModeOnLowFrequency;

      [Flags]
      public enum RegOpModeMode : byte
      {
         Sleep = 0x00, // 0b00000000,
         StandBy = 0x01, // 0b00000001,
         FrequencySynthesisTX = 0x02, //0b00000010,
         Transmit = 0x03, // 0b00000011,
         FrequencySynthesisRX = 0x04, //0b00000100,
         ReceiveContinuous = 0x05, //0b00000101,
         ReceiveSingle = 0x06, // 0b00000110,
         ChannelActivityDetection = 0x07, //0b00000111,
      };

      // Frequency configuration magic numbers from Semtech SX127X specs
      private const double RH_RF95_FXOSC = 32000000.0;
      private const double RH_RF95_FSTEP = RH_RF95_FXOSC / 524288.0;

      // RegFrMsb, RegFrMid, RegFrLsb
      private const double FrequencyDefault = 434000000.0;

      // RegPAConfig
      private const Byte RegPAConfigPASelectRFO = 0x00; //0b00000000;
      private const Byte RegPAConfigPASelectPABoost = 0x80; //0b10000000;
      private const Byte RegPAConfigPASelectDefault = RegPAConfigPASelectRFO;

      private const byte RegPAConfigMaxPowerMin = 0x00; //0b00000000;
      private const byte RegPAConfigMaxPowerMax = 0x70; // 0b01110000;
      private const byte RegPAConfigMaxPowerDefault = 0x40; //0b01000000;

      private const byte RegPAConfigOutputPowerMin = 0x00; // 0b00000000;
      private const byte RegPAConfigOutputPowerMax = 0x0F; //0b00001111;
      private const byte RegPAConfigOutputPowerDefault = RegPAConfigOutputPowerMax;

      // RegPaRamp appears to be for FSK only ?

      // RegOcp
      private const byte RegOcpOn = 0x20; // 0b00100000;
      private const byte RegOcpOff = 0x00; // 0b00000000;
      private const byte RegOcpDefault = RegOcpOn;

      private const byte RegOcpOcpTrimMin = 0x00; // 0b00000000;
      private const byte RegOcpOcpTrimMax = 0x0F; // 0b00011111;
      private const byte RegOcpOcpTrimDefault = 0x0B; //0b00001011;

      // RegLna
      [Flags]
      public enum RegLnaLnaGain : byte
      {
         G1 = 0x01, // 0b00000001,
         G2 = 0x02, // 0b00000010,
         G3 = 0x03, // 0b00000011,
         G4 = 0x04, // 0b00000100,
         G5 = 0x05, // 0b00000101,
         G6 = 0x06, // 0b00000110
      }
      // Note the gain is backwards less = more
      private const RegLnaLnaGain LnaGainDefault = RegLnaLnaGain.G1;

      private const byte RegLnaLnaBoostLfOn = 0x18; // 0b00011000;
      private const byte RegLnaLnaBoostLfOff = 0x00; // 0b00000000;
      private const byte RegLnaLnaBoostLfDefault = RegLnaLnaBoostLfOff;

      private const byte RegLnaLnaBoostHfOn = 0x03; // 0b00000011;
      private const byte RegLnaLnaBoostHfOff = 0x00; // 0b00000000;
      private const byte RegLnaLnaBoostHfDefault = RegLnaLnaBoostHfOff;

      [Flags]
      enum RegIrqFlagsMask : byte
      {
         RxTimeoutMask = 0x80, // 0b10000000,
         RxDoneMask = 0x40, // 0b01000000,
         PayLoadCrcErrorMask = 0x20, // 0b00100000,
         ValidHeadrerMask = 0x10, // 0b00010000,
         TxDoneMask = 0x08, // 0b00001000,
         CadDoneMask = 0x04, // 0b00000100,
         FhssChangeChannelMask = 0x02, // 0b00000010,
         CadDetectedMask = 0x01, //0b00000001,
      }

      [Flags]
      enum RegIrqFlags : byte
      {
         RxTimeout = 0x80, //0b10000000,
         RxDone = 0x40, //0b01000000,
         PayLoadCrcError = 0x20, //0b00100000,
         ValidHeadrer = 0x10, //0b00010000,
         TxDone = 0x08, //0b00001000,
         CadDone = 0x04, //0b00000100,
         FhssChangeChannel = 0x02, // 0b00000010,
         CadDetected = 0x01, // 0b00000001,
         ClearAll = 0xFF, // 0b11111111,
      }

      // RegModemConfig1
      public enum RegModemConfigBandwidth : byte
      {
         _7_8KHz = 0x00, // 0b00000000,
         _10_4KHz = 0x10,	// 0b00010000,
         _15_6KHz = 0x20, // 0b00100000,
         _20_8KHz = 0x30, // 0b00110000,
         _31_25KHz = 0x40, // 0b01000000,
         _41_7KHz = 0x50, // 0b01010000,
         _62_5KHz = 0x60, // b01100000,
         _125KHz = 0x70, // 0b01110000,
         _250KHz = 0x80, // 0b10000000,
         _500KHz = 0x90, // 0b10010000
      }
      private const RegModemConfigBandwidth RegModemConfigBandwidthDefault = RegModemConfigBandwidth._125KHz;

      public enum RegModemConfigCodingRate
      {
         _4of5 = 0x02, // 0b00000010,
         _4of6 = 0x04, // 0b00000100,
         _4of7 = 0x06, // 0b00000110,
         _4of8 = 0x08,  // 0b00001000,
      }
      private const RegModemConfigCodingRate RegModemConfigCodingRateDefault = RegModemConfigCodingRate._4of5;

      public enum RegModemConfigImplicitHeaderModeOn
      {
         ExplicitHeaderMode = 0x00, // 0b00000000,
         ImplicitHeaderMode = 0x01, // 0b00000001,
      }
      private const RegModemConfigImplicitHeaderModeOn RegModemConfigImplicitHeaderModeOnDefault = RegModemConfigImplicitHeaderModeOn.ExplicitHeaderMode;

      // RegModemConfig2
      public enum RegModemConfig2SpreadingFactor : byte
      {
         _64ChipsPerSymbol = 0x60, // 0b01100000,
         _128ChipsPerSymbol = 0x70, // 0b01110000,
         _256ChipsPerSymbol = 0x80, // 0b10000000,
         _512ChipsPerSymbol = 0x90, // 0b10010000,
         _1024ChipsPerSymbol = 0xA0, // 0b10100000,
         _2048ChipsPerSymbol = 0xB0, // 0b10110000,
         _4096ChipsPerSymbol = 0xC0, // 0b11000000,
      }
      private const RegModemConfig2SpreadingFactor RegModemConfig2SpreadingFactorDefault = RegModemConfig2SpreadingFactor._128ChipsPerSymbol;

      private const byte RegModemConfig2TxContinuousModeOn = 0x08; // 0b00001000;
      private const byte RegModemConfig2TxContinuousModeOff = 0x00; //0b00000000;
      private const byte RegModemConfig2TxContinuousModeDefault = RegModemConfig2TxContinuousModeOff;

      private const byte RegModemConfig2RxPayloadCrcOn = 0x04; // 0b00000100;
      private const byte RegModemConfig2RxPayloadCrcOff = 0x00; // 0b00000000;
      private const byte RegModemConfig2RxPayloadCrcDefault = RegModemConfig2RxPayloadCrcOff;

      // RegModemConfig2 for MSb RegSymbTimeoutLsb for LSB
      private const byte SymbolTimeoutDefault = 0x64;

      // RegReambleMsb & RegReambleLsb
      private const ushort PreambleLengthDefault = 0x08;

      // RegPayloadLength
      private const byte PayloadLengthDefault = 0x01;

      // RegMaxPayloadLength
      private const byte PayloadMaxLengthDefault = 0xff;

      // RegHopPeriod
      private const byte FreqHoppingPeriodDefault = 0x0;

      // RegModemConfig3
      private const byte RegModemConfig3LowDataRateOptimizeOn = 0x08; //0b00001000;
      private const byte RegModemConfig3LowDataRateOptimizeOff = 0x00; //0b00000000;
      private const byte RegModemConfig3LowDataRateOptimizeDefault = RegModemConfig3LowDataRateOptimizeOff;

      private const byte RegModemConfig3AgcAutoOn = 0x04; // 0b00000100;
      private const byte RegModemConfig3AgcAutoOff = 0x00; // 0b00000000;
      private const byte RegModemConfig3AgcAutoDefault = RegModemConfig3AgcAutoOff;

      // RegPpmCorrection
      private const byte ppmCorrectionDefault = 0x0;

      // RegDetectOptimize
      public enum RegDetectOptimizeDectionOptimize
      {
         _SF7toSF12 = 0x03,
         _SF6 = 0x05,
      };
      private const RegDetectOptimizeDectionOptimize RegDetectOptimizeDectionOptimizeDefault = RegDetectOptimizeDectionOptimize._SF7toSF12;

      // RegInvertId
      private const byte InvertIqOn = 0x40; // 0b01000000;
      private const byte InvertIqOff = 0x00; // 0b00000000;
      private const byte InvertIqDefault = InvertIqOn;

      // RegDetectionThreshold
      public enum RegisterDetectionThreshold
      {
         _SF7toSF12 = 0x0A,
         _SF6 = 0x0c,
      }
      private const RegisterDetectionThreshold RegisterDetectionThresholdDefault = RegisterDetectionThreshold._SF7toSF12;

      // RegSyncWord Syncword default for public networks
      private const byte RegSyncWordDefault = 0x12;

      // The Semtech ID Relating to the Silicon revision
      private const byte RegVersionValueExpected = 0x12;


      public RegisterManager Rfm9XLoraModem = null; // FIx this later
      private OutputPort ResetGpioPin = null;
      private InterruptPort InterruptPin = null;
      private RegOpModeMode RegOpModeAfterInitialise = RegOpModeMode.Sleep;
      private double Frequency = FrequencyDefault;
      public event OnDataRecievedHandler OnDataReceived = delegate { };
      public event EventHandler OnTransmit = delegate { };


      public void SetMode(RegOpModeMode mode)
      {
         byte regOpModeValue;

         regOpModeValue = RegOpModeLongRangeModeLoRa;
         regOpModeValue |= RegOpModeAcessSharedRegLoRa;
         regOpModeValue |= RegOpModeLowFrequencyModeOnHighFrequency;
         regOpModeValue |= (byte)mode;
         Rfm9XLoraModem.WriteByte((byte)Registers.RegOpMode, regOpModeValue);
      }

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

      public void Initialise(RegOpModeMode regOpModeAfterInitialise, // RegOpMode
         double frequency = FrequencyDefault, // RegFrMsb, RegFrMid, RegFrLsb
         bool paBoost = false, byte maxPower = RegPAConfigMaxPowerDefault, byte outputPower = RegPAConfigOutputPowerDefault, // RegPaConfig
         bool ocpOn = true, byte ocpTrim = RegOcpOcpTrimDefault, // RegOcp
         RegLnaLnaGain lnaGain = LnaGainDefault, bool lnaBoostLF = false, bool lnaBoostHf = false, // RegLna
         RegModemConfigBandwidth bandwidth = RegModemConfigBandwidthDefault, RegModemConfigCodingRate codingRate = RegModemConfigCodingRateDefault, RegModemConfigImplicitHeaderModeOn implicitHeaderModeOn = RegModemConfigImplicitHeaderModeOnDefault, //RegModemConfig1
         RegModemConfig2SpreadingFactor spreadingFactor = RegModemConfig2SpreadingFactorDefault, bool txContinuousMode = false, bool rxPayloadCrcOn = false,
         ushort symbolTimeout = SymbolTimeoutDefault,
         ushort preambleLength = PreambleLengthDefault,
         byte payloadLength = PayloadLengthDefault,
         byte payloadMaxLength = PayloadMaxLengthDefault,
         byte freqHoppingPeriod = FreqHoppingPeriodDefault,
         bool lowDataRateOptimize = false, bool agcAutoOn = false,
         byte ppmCorrection = ppmCorrectionDefault,
         RegDetectOptimizeDectionOptimize detectionOptimize = RegDetectOptimizeDectionOptimizeDefault,
         bool invertIQ = false,
         RegisterDetectionThreshold detectionThreshold = RegisterDetectionThresholdDefault,
         byte syncWord = RegSyncWordDefault)
      {
         Frequency = frequency; // Store this away for RSSI adjustments
         RegOpModeAfterInitialise = regOpModeAfterInitialise;

         byte regVersionValue = Rfm9XLoraModem.ReadByte((byte)Registers.RegVersion);
         if (regVersionValue != RegVersionValueExpected)
         {
            throw new ApplicationException("Semtech SX127X not found");
         }

         ResetGpioPin.Write(false);
         Thread.Sleep(10);
         ResetGpioPin.Write(true);
         Thread.Sleep(10);

         // Put the device into sleep mode so registers can be changed
         SetMode(RegOpModeMode.Sleep);

         // Configure RF Carrier frequency 
         if (frequency != FrequencyDefault)
         {
            byte[] bytes = BitConverter.GetBytes((long)(frequency / RH_RF95_FSTEP));
            Rfm9XLoraModem.WriteByte((byte)Registers.RegFrMsb, bytes[2]);
            Rfm9XLoraModem.WriteByte((byte)Registers.RegFrMid, bytes[1]);
            Rfm9XLoraModem.WriteByte((byte)Registers.RegFrLsb, bytes[0]);
         }

         // Set RegPaConfig if any of the associated settings are nto the default values
         if ((paBoost != false) || (maxPower != RegPAConfigMaxPowerDefault) || (outputPower != RegPAConfigOutputPowerDefault))
         {
            byte regPaConfigValue = maxPower;
            if (paBoost)
            {
               regPaConfigValue |= RegPAConfigPASelectPABoost;
            }
            regPaConfigValue |= outputPower;
            Rfm9XLoraModem.WriteByte((byte)Registers.RegPAConfig, regPaConfigValue);
         }

         // Set RegOcp if any of the settings not defaults
         if ((ocpOn != true) || (ocpTrim != RegOcpOcpTrimDefault))
         {
            byte regOcpValue = ocpTrim;
            if (ocpOn)
            {
               regOcpValue |= RegOcpOn;
            }
            Rfm9XLoraModem.WriteByte((byte)Registers.RegOcp, regOcpValue);
         }

         // Set RegLna if any of the settings not defaults
         if ((lnaGain != LnaGainDefault) || (lnaBoostLF != false) || (lnaBoostHf != false))
         {
            byte regLnaValue = (byte)lnaGain;
            if (lnaBoostLF)
            {
               regLnaValue |= RegLnaLnaBoostLfOn;
            }
            if (lnaBoostHf)
            {
               regLnaValue |= RegLnaLnaBoostHfOn;
            }
            Rfm9XLoraModem.WriteByte((byte)Registers.RegLna, regLnaValue);
         }

         // Set regModemConfig1 if any of the settings not defaults
         if ((bandwidth != RegModemConfigBandwidthDefault) || (codingRate != RegModemConfigCodingRateDefault) || (implicitHeaderModeOn != RegModemConfigImplicitHeaderModeOnDefault))
         {
            byte regModemConfig1Value = (byte)bandwidth;
            regModemConfig1Value |= (byte)codingRate;
            regModemConfig1Value |= (byte)implicitHeaderModeOn;
            Rfm9XLoraModem.WriteByte((byte)Registers.RegModemConfig1, regModemConfig1Value);
         }

         // Set regModemConfig2 if any of the settings not defaults
         if ((spreadingFactor != RegModemConfig2SpreadingFactorDefault) || (txContinuousMode != false) | (rxPayloadCrcOn != false) || (symbolTimeout != SymbolTimeoutDefault))
         {
            byte RegModemConfig2Value = (byte)spreadingFactor;
            if (txContinuousMode)
            {
               RegModemConfig2Value |= RegModemConfig2TxContinuousModeOn;
            }
            if (rxPayloadCrcOn)
            {
               RegModemConfig2Value |= RegModemConfig2RxPayloadCrcOn;
            }
            // Get the MSB of SymbolTimeout
            byte[] symbolTimeoutBytes = BitConverter.GetBytes(symbolTimeout);
            RegModemConfig2Value |= symbolTimeoutBytes[1];
            Rfm9XLoraModem.WriteByte((byte)Registers.RegModemConfig2, RegModemConfig2Value);
         }

         // RegModemConfig2.SymbTimout + RegSymbTimeoutLsb
         if (symbolTimeout != SymbolTimeoutDefault)
         {
            // Get the LSB of SymbolTimeout
            byte[] symbolTimeoutBytes = BitConverter.GetBytes(symbolTimeout);
            Rfm9XLoraModem.WriteByte((byte)Registers.RegSymbTimeout, symbolTimeoutBytes[0]);
         }

         // RegPreambleMsb + RegPreambleLsb
         if (preambleLength != PreambleLengthDefault)
         {
            byte[] premableBytes = BitConverter.GetBytes(preambleLength);
            Rfm9XLoraModem.WriteByte((byte)Registers.RegPreambleMsb, premableBytes[1]);
            Rfm9XLoraModem.WriteByte((byte)Registers.RegPreambleLsb, premableBytes[0]);
         }

         // RegPayloadLength
         if (payloadLength != PayloadLengthDefault)
         {
            Rfm9XLoraModem.WriteByte((byte)Registers.RegPayloadLength, payloadLength);
         }

         // RegMaxPayloadLength
         if (payloadMaxLength != PayloadMaxLengthDefault)
         {
            Rfm9XLoraModem.WriteByte((byte)Registers.RegMaxPayloadLength, payloadMaxLength);
         }

         // RegHopPeriod
         if (freqHoppingPeriod != FreqHoppingPeriodDefault)
         {
            Rfm9XLoraModem.WriteByte((byte)Registers.RegHopPeriod, freqHoppingPeriod);
         }

         // RegModemConfig3
         if ((lowDataRateOptimize != false) || (agcAutoOn != false))
         {
            byte regModemConfig3Value = 0;
            if (lowDataRateOptimize)
            {
               regModemConfig3Value |= RegModemConfig3LowDataRateOptimizeOn;
            }
            if (agcAutoOn)
            {
               regModemConfig3Value |= RegModemConfig3AgcAutoOn;
            }
            Rfm9XLoraModem.WriteByte((byte)Registers.RegModemConfig3, regModemConfig3Value);
         }

         // RegPpmCorrection
         if (ppmCorrection != ppmCorrectionDefault)
         {
            Rfm9XLoraModem.WriteByte((byte)Registers.RegPpmCorrection, ppmCorrection);
         }

         // RegDetectOptimize
         if (detectionOptimize != RegDetectOptimizeDectionOptimizeDefault)
         {
            Rfm9XLoraModem.WriteByte((byte)Registers.RegDetectOptimize, (byte)detectionOptimize);
         }

         // RegInvertIQ
         if (invertIQ != false)
         {
            Rfm9XLoraModem.WriteByte((byte)Registers.RegInvertIQ, (byte)InvertIqOn);
         }

         // RegDetectionThreshold
         if (detectionThreshold != RegisterDetectionThresholdDefault)
         {
            Rfm9XLoraModem.WriteByte((byte)Registers.RegDetectionThreshold, (byte)detectionThreshold);
         }

         // RegSyncWordDefault 
         if (syncWord != RegSyncWordDefault)
         {
            Rfm9XLoraModem.WriteByte((byte)Registers.RegSyncWord, syncWord);
         }

         // TODO revist this split & move to onReceive function
         Rfm9XLoraModem.WriteByte((byte)Registers.RegDioMapping1, 0x00); // RegDioMapping1 0b00000000 DI0 RxReady & TxReady

         // Configure RegOpMode before returning
         SetMode(regOpModeAfterInitialise);
      }

      public void RegisterDump()
      {
         Debug.Print("---SX1276 User Registers ---");
         Rfm9XLoraModem.RegisterDump((byte)Registers.MinValue, (byte)Registers.MaxValue);
      }

      [MethodImpl(MethodImplOptions.Synchronized)]
      void InterruptPin_OnInterrupt(uint data1, uint data2, DateTime time)
      {
         RegIrqFlags IrqFlags = (RegIrqFlags)this.Rfm9XLoraModem.ReadByte((byte)Registers.RegIrqFlags);
         //Debug.Print("RegIrqFlags " + ByteToHexString((byte)IrqFlags));

         if ((IrqFlags & RegIrqFlags.RxDone) == RegIrqFlags.RxDone)  // RxDone
         {
            byte currentFifoAddress = this.Rfm9XLoraModem.ReadByte((byte)Registers.RegFifoRxCurrent);
            this.Rfm9XLoraModem.WriteByte((byte)Registers.RegFifoAddrPtr, currentFifoAddress); // RegFifoAddrPtr
            byte numberOfBytes = this.Rfm9XLoraModem.ReadByte((byte)Registers.RegRxNbBytes); // RegRxNbBytes

            // Allocate buffer for message
            byte[] messageBytes = new byte[numberOfBytes];

            for (int i = 0; i < numberOfBytes; i++)
            {
               messageBytes[i] = this.Rfm9XLoraModem.ReadByte(0x00); // RegFifo
            }

            OnDataReceived(messageBytes);
         }

         if ((IrqFlags & RegIrqFlags.TxDone) == RegIrqFlags.TxDone) 
         {
            this.SetMode(RegOpModeAfterInitialise);
            OnTransmit();
         }

         this.Rfm9XLoraModem.WriteByte((byte)Registers.RegDioMapping1, 0x0);
         this.Rfm9XLoraModem.WriteByte((byte)Registers.RegIrqFlags, (byte)RegIrqFlags.ClearAll);
      }

      public void SendMessage(byte[] messageBytes)
      {
         this.Rfm9XLoraModem.WriteByte(0x0E, 0x0); // RegFifoTxBaseAddress 

         // Set the Register Fifo address pointer
         this.Rfm9XLoraModem.WriteByte((byte)Registers.RegFifoAddrPtr, 0x0);

         foreach (byte b in messageBytes)
         {
            this.Rfm9XLoraModem.WriteByte((byte)Registers.RegFifo, b);
         }

         // Set the length of the message in the fifo
         this.Rfm9XLoraModem.WriteByte((byte)Registers.RegPayloadLength, (byte)messageBytes.Length); // RegPayloadLength

         // TODO need to set this based on RX/TX mode handlers. Set based on OnTransmit, onReceive != null
         this.Rfm9XLoraModem.WriteByte((byte)Registers.RegDioMapping1, 0x40); // RegDioMapping1 0b00000000 DI0 RxReady & TxReady

         SetMode(RegOpModeMode.Transmit);
      }

      public class Program
      {
         public static void Main()
         {
            Rfm9XDevice rfm9XDevice = new Rfm9XDevice(Pins.GPIO_PIN_D10, Pins.GPIO_PIN_D9, Pins.GPIO_PIN_D2);
            byte MessageCount = Byte.MinValue;

            rfm9XDevice.Initialise(RegOpModeMode.ReceiveContinuous, 915000000, paBoost: true, rxPayloadCrcOn:true);
            rfm9XDevice.OnDataReceived += rfm9XDevice_OnDataReceived;
            rfm9XDevice.OnTransmit += rfm9XDevice_OnTransmit;

            while (true)
            {
               string messageText = "Hello NetMF LoRa! " + MessageCount.ToString();
               MessageCount += 1;
               byte[] messageBytes = UTF8Encoding.UTF8.GetBytes(messageText);
               Debug.Print("Sending " + messageBytes.Length + " bytes message " + messageText);
               rfm9XDevice.SendMessage(messageBytes);

               Thread.Sleep(10000);
            }
         }

         static void rfm9XDevice_OnTransmit()
         {
            Debug.Print("Transmit-Done");
         }

         static void rfm9XDevice_OnDataReceived(byte[] data)
         {
            try
            {
               string messageText = new string(UTF8Encoding.UTF8.GetChars(data));
               Debug.Print("Received " + data.Length.ToString() + " byte message " + messageText);
            }
            catch (Exception ex)
            {
               Debug.Print(ex.Message);
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
   }
}
