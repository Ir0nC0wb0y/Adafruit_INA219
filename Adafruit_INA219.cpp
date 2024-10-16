/*!
 * @file Adafruit_INA219.cpp
 *
 * @mainpage Adafruit INA219 current/power monitor IC
 *
 * @section intro_sec Introduction
 *
 *  Driver for the INA219 current sensor
 *
 *  This is a library for the Adafruit INA219 breakout
 *  ----> https://www.adafruit.com/product/904
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing
 *  products from Adafruit!
 *
 * @section author Author
 *
 * Written by Bryan Siepert and Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "Arduino.h"

#include <Wire.h>

#include "Adafruit_INA219.h"

/*!
 *  @brief  Instantiates a new INA219 class
 *  @param addr the I2C address the device can be found on. Default is 0x40
 */
Adafruit_INA219::Adafruit_INA219(uint8_t addr) {
  ina219_i2caddr = addr;
  ina219_currentDivider_mA = 0;
  ina219_powerMultiplier_mW = 0.0f;
}

/*!
 *  @brief INA219 class destructor
 */
Adafruit_INA219::~Adafruit_INA219() { delete i2c_dev; }

/*!
 *  @brief  Sets up the HW (defaults to 32V and 2A for calibration values)
 *  @param theWire the TwoWire object to use
 *  @return true: success false: Failed to start I2C
 */
bool Adafruit_INA219::begin(TwoWire *theWire) {
  if (!i2c_dev) {
    i2c_dev = new Adafruit_I2CDevice(ina219_i2caddr, theWire);
  }

  if (!i2c_dev->begin()) {
    return false;
  }
  //init();
  return true;
}

/*!
 *  @brief  begin I2C and set up the hardware
 */
void Adafruit_INA219::init() {
  // Set chip to large range config values to start
  setCalibration_32V_2A();
}

/*!
 *  @brief  Gets the raw bus voltage (16-bit signed integer, so +-32767)
 *  @return the raw bus voltage reading
 */
int16_t Adafruit_INA219::getBusVoltage_raw() {
  uint16_t value;

  Adafruit_BusIO_Register bus_voltage_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_BUSVOLTAGE, 2, MSBFIRST);
  _success = bus_voltage_reg.read(&value);

  // Shift to the right 3 to drop CNVR and OVF and multiply by LSB
  return (int16_t)((value >> 3) * 4);
}

/*!
 *  @brief  Gets the raw shunt voltage (16-bit signed integer, so +-32767)
 *  @return the raw shunt voltage reading
 */
int16_t Adafruit_INA219::getShuntVoltage_raw() {
  uint16_t value;
  Adafruit_BusIO_Register shunt_voltage_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_SHUNTVOLTAGE, 2, MSBFIRST);
  _success = shunt_voltage_reg.read(&value);
  return value;
}

/*!
 *  @brief  Gets the raw current value (16-bit signed integer, so +-32767)
 *  @return the raw current reading
 */
int16_t Adafruit_INA219::getCurrent_raw() {
  uint16_t value;

  // Sometimes a sharp load will reset the INA219, which will
  // reset the cal register, meaning CURRENT and POWER will
  // not be available ... avoid this by always setting a cal
  // value even if it's an unfortunate extra step
  Adafruit_BusIO_Register calibration_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CALIBRATION, 2, MSBFIRST);
  calibration_reg.write(ina219_calValue, 2);

  // Now we can safely read the CURRENT register!
  Adafruit_BusIO_Register current_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CURRENT, 2, MSBFIRST);
  _success = current_reg.read(&value);
  return value;
}

/*!
 *  @brief  Gets the raw power value (16-bit signed integer, so +-32767)
 *  @return raw power reading
 */
int16_t Adafruit_INA219::getPower_raw() {
  uint16_t value;

  // Sometimes a sharp load will reset the INA219, which will
  // reset the cal register, meaning CURRENT and POWER will
  // not be available ... avoid this by always setting a cal
  // value even if it's an unfortunate extra step
  Adafruit_BusIO_Register calibration_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CALIBRATION, 2, MSBFIRST);
  calibration_reg.write(ina219_calValue, 2);

  // Now we can safely read the POWER register!
  Adafruit_BusIO_Register power_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_POWER, 2, MSBFIRST);
  _success = power_reg.read(&value);
  return value;
}

/*!
 *  @brief  Gets the shunt voltage in mV (so +-327mV)
 *  @return the shunt voltage converted to millivolts
 */
float Adafruit_INA219::getShuntVoltage_mV() {
  int16_t value;
  value = getShuntVoltage_raw();
  return value * 0.01;
}

/*!
 *  @brief  Gets the bus voltage in volts
 *  @return the bus voltage converted to volts
 */
float Adafruit_INA219::getBusVoltage_V() {
  int16_t value = getBusVoltage_raw();
  return value * 0.001;
}

/*!
 *  @brief  Gets the current value in mA, taking into account the
 *          config settings and current LSB
 *  @return the current reading convereted to milliamps
 */
float Adafruit_INA219::getCurrent_mA() {
  float valueDec = getCurrent_raw();
  valueDec /= ina219_currentDivider_mA;
  return valueDec;
}

/*!
 *  @brief  Gets the power value in mW, taking into account the
 *          config settings and current LSB
 *  @return power reading converted to milliwatts
 */
float Adafruit_INA219::getPower_mW() {
  float valueDec = getPower_raw();
  valueDec *= ina219_powerMultiplier_mW;
  return valueDec;
}

/*!
 *  @brief  Gets the raw bus voltage (16-bit signed integer, so +-32767)
 *  @return the raw bus voltage reading
 */
int16_t Adafruit_INA219::getConfigRegister(bool print) {
  //int16_t getConfigRegister(bool print = true);
  uint16_t value;

  Adafruit_BusIO_Register config_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CONFIG, 2, MSBFIRST);
  _success = config_reg.read(&value);

  if (print) {
    Serial.print("Config Register: "); Serial.println((int16_t)value);
  }
  // Shift to the right 3 to drop CNVR and OVF and multiply by LSB
  return (int16_t)(value);
}

/*!
 *  @brief  Configures to INA219 to be able to measure up to 32V and 2A
 *          of current.  Each unit of current corresponds to 100uA, and
 *          each unit of power corresponds to 2mW. Counter overflow
 *          occurs at 3.2A.
 *  @note   These calculations assume a 0.1 ohm resistor is present
 */
void Adafruit_INA219::setCalibration_manual(uint32_t calValue,
                                            uint32_t currentDivider_mA,
                                            float powerMultiplier_mW,
                                            int bVoltRange,
                                            int gain,
                                            int bADCRes,
                                            int sADCRes,
                                            int mode) {
  // Using this function requires understanding the calculations.
  // The settings will be pushed to the sensor and used for internal adjustments.
  // Inputs, defaults highlighted by (d):
  //  -calValue: adjustment used by sensor
  //  -currentDivider_mA: adjustment used by library to convert raw to [mA]
  //  -powerMultiplier_mW: adjustment used by library to convert raw to [mW]
  //  -bVoltRange: bus voltage range
  //    -0: 16V
  //    -1(d): 32V
  //  -gain: range of expected shunt voltage, in mV
  //    -0: 40, 80, 160, 320
  //    -1: 80
  //    -2: 160
  //    -3(d): 320
  //  -bADCRes: bus Voltage ADC resolution
  //    - 0:  9 bit, 1 sample
  //    - 1: 10 bit, 1 sample
  //    - 2: 11 bit, 1 sample
  //    - 3(d): 12 bit, 1 sample
  //    - 4: 12 bit, 2 samples
  //    - 5: 12 bit, 4 samples
  //    - 6: 12 bit, 8 samples
  //    - 7: 12 bit, 16 samples
  //    - 8: 12 bit, 32 samples
  //    - 9: 12 bit, 64 samples
  //    -10: 12 bit, 128 samples
  //  -sADCRes: shunt Voltage ADC resolution
  //    - 0:  9 bit, 1 sample
  //    - 1: 10 bit, 1 sample
  //    - 2: 11 bit, 1 sample
  //    - 3(d): 12 bit, 1 sample
  //    - 4: 12 bit, 2 samples
  //    - 5: 12 bit, 4 samples
  //    - 6: 12 bit, 8 samples
  //    - 7: 12 bit, 16 samples
  //    - 8: 12 bit, 32 samples
  //    - 9: 12 bit, 64 samples
  //    -10: 12 bit, 128 samples
  //  -mode: instructions for sensor operation
  //    -1: shunt voltage, triggered
  //    -2: bus voltage, triggered
  //    -3: shunt and bus voltage, triggered
  //    -5: shunt voltage, continuous
  //    -6: bus voltage, continuous
  //    -7(d): shunt and bus voltage, continuous

  // Set multipliers to convert raw current/power values
  ina219_calValue = calValue;
  ina219_currentDivider_mA = currentDivider_mA; // Current LSB = 100uA per bit (1000/100 = 10)
  ina219_powerMultiplier_mW = powerMultiplier_mW; // Power LSB = 1mW per bit (2/1)

  // Set Calibration register to 'Cal' calculated above
  Adafruit_BusIO_Register calibration_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CALIBRATION, 2, MSBFIRST);
  calibration_reg.write(ina219_calValue, 2);

  // Set Config register to take into account the settings above
  uint16_t config = 0;
  Serial.println("Config: ");

  // Config: Bus Voltage Range
  //  -bVoltRange: bus voltage range
  //    -1: 16V
  //    -2(d): 32V
  switch (bVoltRange) {
    case 0:
      config = INA219_CONFIG_BVOLTAGERANGE_16V;
      Serial.println("  Voltage: 16V");
      break;
    case 1:
      config = INA219_CONFIG_BVOLTAGERANGE_32V;
      Serial.println("  Voltage: 32V");
      break;

    default:
      config = INA219_CONFIG_BVOLTAGERANGE_32V;
      Serial.println("  Default voltage");
      break;
  }

  // Config: Gain
  //  -gain: range of expected shunt voltage, in mV
  //    -0: 40, 80, 160, 320
  //    -1: 80
  //    -2: 160
  //    -3(d): 320
  switch (gain) {
    case 0:
      config = config | INA219_CONFIG_GAIN_1_40MV;
      Serial.println("  Gain: 1, 40mV");
      break;

    case 1:
      config = config | INA219_CONFIG_GAIN_2_80MV;
      Serial.println("  Gain: 2, 80mV");
      break;

    case 2:
      config = config | INA219_CONFIG_GAIN_4_160MV;
      Serial.println("  Gain: 4, 160mV");
      break;

    case 3:
      config = config | INA219_CONFIG_GAIN_8_320MV;
      Serial.println("  Gain: 8, 320mV");
      break;

    default:
      config = config | INA219_CONFIG_GAIN_8_320MV;
      Serial.println("  Gain: default (8, 320mV)");
      break;
  }

  // Config: Bus ADC Resolution
  //  -bADCRes: bus Voltage ADC resolution
  //    - 0:  9 bit, 1 sample
  //    - 1: 10 bit, 1 sample
  //    - 2: 11 bit, 1 sample
  //    - 3(d): 12 bit, 1 sample
  //    - 4: 12 bit, 2 samples
  //    - 5: 12 bit, 4 samples
  //    - 6: 12 bit, 8 samples
  //    - 7: 12 bit, 16 samples
  //    - 8: 12 bit, 32 samples
  //    - 9: 12 bit, 64 samples
  //    -10: 12 bit, 128 samples
  switch (bADCRes)  {
    case 0:
      config = config | INA219_CONFIG_BADCRES_9BIT;
      Serial.println("  bADCRes: 9 bit");
      break;

    case 1:
      config = config | INA219_CONFIG_BADCRES_10BIT;
      Serial.println("  bADCRes: 10 bit");
      break;

    case 2:
      config = config | INA219_CONFIG_BADCRES_11BIT;
      Serial.println("  bADCRes: 11 bit");
      break;
    
    case 3:
      config = config | INA219_CONFIG_BADCRES_12BIT;
      Serial.println("  bADCRes: 12 bit");
      break;

    case 4:
      config = config | INA219_CONFIG_BADCRES_12BIT_2S_1060US;
      Serial.println("  bADCRes: 12 bit, 2 samples");
      break;

    case 5:
      config = config | INA219_CONFIG_BADCRES_12BIT_4S_2130US;
      Serial.println("  bADCRes: 12 bit, 4 samples");
      break;

    case 6:
      config = config | INA219_CONFIG_BADCRES_12BIT_8S_4260US;
      Serial.println("  bADCRes: 12 bit, 8 samples");
      break;

    case 7:
      config = config | INA219_CONFIG_BADCRES_12BIT_16S_8510US;
      Serial.println("  bADCRes: 12 bit, 16 samples");
      break;

    case 8:
      config = config | INA219_CONFIG_BADCRES_12BIT_32S_17MS;
      Serial.println("  bADCRes: 12 bit, 32 samples");
      break;

    case 9:
      config = config | INA219_CONFIG_BADCRES_12BIT_64S_34MS;
      Serial.println("  bADCRes: 12 bit, 64 samples");
      break;

    case 10:
      config = config | INA219_CONFIG_BADCRES_12BIT_128S_69MS;
      Serial.println("  bADCRes: 12 bit, 128 samples");
      break;
    
    default:
      config = config | INA219_CONFIG_BADCRES_12BIT;
      Serial.println("  bADCRes: default 12 bit");
      break;
  }

  // Config: Shunt ADC Resolution
  //  -sADCRes: shunt Voltage ADC resolution
  //    - 0:  9 bit, 1 sample
  //    - 1: 10 bit, 1 sample
  //    - 2: 11 bit, 1 sample
  //    - 3(d): 12 bit, 1 sample
  //    - 4: 12 bit, 2 samples
  //    - 5: 12 bit, 4 samples
  //    - 6: 12 bit, 8 samples
  //    - 7: 12 bit, 16 samples
  //    - 8: 12 bit, 32 samples
  //    - 9: 12 bit, 64 samples
  //    -10: 12 bit, 128 samples
  switch (sADCRes) {
    case 0:
      config = config | INA219_CONFIG_SADCRES_9BIT_1S_84US;
      Serial.println("  sADCRes: 9 bit");
      break;

    case 1:
      config = config | INA219_CONFIG_SADCRES_10BIT_1S_148US;
      Serial.println("  sADCRes: 10 bit");
      break;

    case 2:
      config = config | INA219_CONFIG_SADCRES_11BIT_1S_276US;
      Serial.println("  sADCRes: 11 bit");
      break;

    case 3:
      config = config | INA219_CONFIG_SADCRES_12BIT_1S_532US;
      Serial.println("  sADCRes: 12 bit");
      break;

    case 4:
      config = config | INA219_CONFIG_SADCRES_12BIT_2S_1060US;
      Serial.println("  sADCRes: 12 bit, 2 samples");
      break;

    case 5:
      config = config | INA219_CONFIG_SADCRES_12BIT_4S_2130US;
      Serial.println("  sADCRes: 12 bit, 4 samples");
      break;

    case 6:
      config = config | INA219_CONFIG_SADCRES_12BIT_8S_4260US;
      Serial.println("  sADCRes: 12 bit, 8 samples");
      break;

    case 7:
      config = config | INA219_CONFIG_SADCRES_12BIT_16S_8510US;
      Serial.println("  sADCRes: 12 bit, 16 samples");
      break;

    case 8:
      config = config | INA219_CONFIG_SADCRES_12BIT_32S_17MS;
      Serial.println("  sADCRes: 12 bit, 32 samples");
      break;

    case 9:
      config = config | INA219_CONFIG_SADCRES_12BIT_64S_34MS;
      Serial.println("  sADCRes: 12 bit, 64 samples");
      break;

    case 10:
      config = config | INA219_CONFIG_SADCRES_12BIT_128S_69MS;
      Serial.println("  sADCRes: 12 bit, 128 samples");
      break;

    default:
      config = config | INA219_CONFIG_SADCRES_12BIT_1S_532US;
      Serial.println("  sADCRes: default 12 bit");
      break;
  }

  // Config: Sensor Operating Mode
  //  -mode: instructions for sensor operation
  //    -1: shunt voltage, triggered
  //    -2: bus voltage, triggered
  //    -3: shunt and bus voltage, triggered
  //    -5: shunt voltage, continuous
  //    -6: bus voltage, continuous
  //    -7: shunt and bus voltage, continuous
  switch (mode) {
    case 1:
      config = config | INA219_CONFIG_MODE_SVOLT_TRIGGERED;
      Serial.println("  mode: s trig");
      break;

    case 2:
      config = config | INA219_CONFIG_MODE_BVOLT_TRIGGERED;
      Serial.println("  mode: b trig");
      break;

    case 3:
      config = config | INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED;
      Serial.println("  mode: s&b trig");
      break;

    case 5:
      config = config | INA219_CONFIG_MODE_SVOLT_CONTINUOUS;
      Serial.println("  mode: s cont");
      break;

    case 6:
      config = config | INA219_CONFIG_MODE_BVOLT_CONTINUOUS;
      Serial.println("  mode: b cont");
      break;

    case 7:
      config = config | INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
      Serial.println("  mode: s&b cont");
      break;
    
    default:
      config = config | INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
      Serial.println("   mode: default s&b cont");
      break;
  }
  
  uint16_t config_default = INA219_CONFIG_BVOLTAGERANGE_32V |
                    INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

  Serial.print("Default config: "); Serial.println(config_default);
  Serial.print("New config    : "); Serial.println(config);

  Adafruit_BusIO_Register config_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CONFIG, 2, MSBFIRST);
  _success = config_reg.write(config, 2);
}


/*!
 *  @brief  Configures to INA219 to be able to measure up to 32V and 2A
 *          of current.  Each unit of current corresponds to 100uA, and
 *          each unit of power corresponds to 2mW. Counter overflow
 *          occurs at 3.2A.
 *  @note   These calculations assume a 0.1 ohm resistor is present
 */
void Adafruit_INA219::setCalibration_32V_2A() {
  // By default we use a pretty huge range for the input voltage,
  // which probably isn't the most appropriate choice for system
  // that don't use a lot of power.  But all of the calculations
  // are shown below if you want to change the settings.  You will
  // also need to change any relevant register settings, such as
  // setting the VBUS_MAX to 16V instead of 32V, etc.

  Serial.println("Setting config for 32V @ 2A");

  // VBUS_MAX = 32V             (Assumes 32V, can also be set to 16V)
  // VSHUNT_MAX = 0.32          (Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
  // RSHUNT = 0.1               (Resistor value in ohms)

  // 1. Determine max possible current
  // MaxPossible_I = VSHUNT_MAX / RSHUNT
  // MaxPossible_I = 3.2A

  // 2. Determine max expected current
  // MaxExpected_I = 2.0A

  // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
  // MinimumLSB = MaxExpected_I/32767
  // MinimumLSB = 0.000061              (61uA per bit)
  // MaximumLSB = MaxExpected_I/4096
  // MaximumLSB = 0,000488              (488uA per bit)

  // 4. Choose an LSB between the min and max values
  //    (Preferrably a roundish number close to MinLSB)
  // CurrentLSB = 0.0001 (100uA per bit)

  // 5. Compute the calibration register
  // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
  // Cal = 4096 (0x1000)

  ina219_calValue = 4096;

  // 6. Calculate the power LSB
  // PowerLSB = 20 * CurrentLSB
  // PowerLSB = 0.002 (2mW per bit)

  // 7. Compute the maximum current and shunt voltage values before overflow
  //
  // Max_Current = Current_LSB * 32767
  // Max_Current = 3.2767A before overflow
  //
  // If Max_Current > Max_Possible_I then
  //    Max_Current_Before_Overflow = MaxPossible_I
  // Else
  //    Max_Current_Before_Overflow = Max_Current
  // End If
  //
  // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
  // Max_ShuntVoltage = 0.32V
  //
  // If Max_ShuntVoltage >= VSHUNT_MAX
  //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
  // Else
  //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
  // End If

  // 8. Compute the Maximum Power
  // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
  // MaximumPower = 3.2 * 32V
  // MaximumPower = 102.4W

  // Set multipliers to convert raw current/power values
  ina219_currentDivider_mA = 10; // Current LSB = 100uA per bit (1000/100 = 10)
  ina219_powerMultiplier_mW = 2; // Power LSB = 1mW per bit (2/1)

  // Set Calibration register to 'Cal' calculated above
  Adafruit_BusIO_Register calibration_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CALIBRATION, 2, MSBFIRST);
  calibration_reg.write(ina219_calValue, 2);

  // Set Config register to take into account the settings above
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                    INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
  Adafruit_BusIO_Register config_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CONFIG, 2, MSBFIRST);
  _success = config_reg.write(config, 2);
}

/*!
 *  @brief  Set power save mode according to parameters
 *  @param  on
 *          boolean value
 */
void Adafruit_INA219::powerSave(bool on) {
  Adafruit_BusIO_Register config_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CONFIG, 2, MSBFIRST);

  Adafruit_BusIO_RegisterBits mode_bits =
      Adafruit_BusIO_RegisterBits(&config_reg, 3, 0);
  if (on) {
    _success = mode_bits.write(INA219_CONFIG_MODE_POWERDOWN);
  } else {
    _success = mode_bits.write(INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS);
  }
}

/*!
 *  @brief  Configures to INA219 to be able to measure up to 32V and 1A
 *          of current.  Each unit of current corresponds to 40uA, and each
 *          unit of power corresponds to 800uW. Counter overflow occurs at
 *          1.3A.
 *  @note   These calculations assume a 0.1 ohm resistor is present
 */
void Adafruit_INA219::setCalibration_32V_1A() {
  // By default we use a pretty huge range for the input voltage,
  // which probably isn't the most appropriate choice for system
  // that don't use a lot of power.  But all of the calculations
  // are shown below if you want to change the settings.  You will
  // also need to change any relevant register settings, such as
  // setting the VBUS_MAX to 16V instead of 32V, etc.

  // VBUS_MAX = 32V		(Assumes 32V, can also be set to 16V)
  // VSHUNT_MAX = 0.32	(Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
  // RSHUNT = 0.1			(Resistor value in ohms)

  // 1. Determine max possible current
  // MaxPossible_I = VSHUNT_MAX / RSHUNT
  // MaxPossible_I = 3.2A

  // 2. Determine max expected current
  // MaxExpected_I = 1.0A

  // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
  // MinimumLSB = MaxExpected_I/32767
  // MinimumLSB = 0.0000305             (30.5uA per bit)
  // MaximumLSB = MaxExpected_I/4096
  // MaximumLSB = 0.000244              (244uA per bit)

  // 4. Choose an LSB between the min and max values
  //    (Preferrably a roundish number close to MinLSB)
  // CurrentLSB = 0.0000400 (40uA per bit)

  // 5. Compute the calibration register
  // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
  // Cal = 10240 (0x2800)

  ina219_calValue = 10240;

  // 6. Calculate the power LSB
  // PowerLSB = 20 * CurrentLSB
  // PowerLSB = 0.0008 (800uW per bit)

  // 7. Compute the maximum current and shunt voltage values before overflow
  //
  // Max_Current = Current_LSB * 32767
  // Max_Current = 1.31068A before overflow
  //
  // If Max_Current > Max_Possible_I then
  //    Max_Current_Before_Overflow = MaxPossible_I
  // Else
  //    Max_Current_Before_Overflow = Max_Current
  // End If
  //
  // ... In this case, we're good though since Max_Current is less than
  // MaxPossible_I
  //
  // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
  // Max_ShuntVoltage = 0.131068V
  //
  // If Max_ShuntVoltage >= VSHUNT_MAX
  //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
  // Else
  //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
  // End If

  // 8. Compute the Maximum Power
  // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
  // MaximumPower = 1.31068 * 32V
  // MaximumPower = 41.94176W

  // Set multipliers to convert raw current/power values
  ina219_currentDivider_mA = 25;    // Current LSB = 40uA per bit (1000/40 = 25)
  ina219_powerMultiplier_mW = 0.8f; // Power LSB = 800uW per bit

  // Set Calibration register to 'Cal' calculated above
  Adafruit_BusIO_Register calibration_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CALIBRATION, 2, MSBFIRST);
  calibration_reg.write(ina219_calValue, 2);

  // Set Config register to take into account the settings above
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                    INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
  Adafruit_BusIO_Register config_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CONFIG, 2, MSBFIRST);
  _success = config_reg.write(config, 2);
}

/*!
 *  @brief set device to alibration which uses the highest precision for
 *     current measurement (0.1mA), at the expense of
 *     only supporting 16V at 400mA max.
 */
void Adafruit_INA219::setCalibration_16V_400mA() {

  // Calibration which uses the highest precision for
  // current measurement (0.1mA), at the expense of
  // only supporting 16V at 400mA max.

  // VBUS_MAX = 16V
  // VSHUNT_MAX = 0.04          (Assumes Gain 1, 40mV)
  // RSHUNT = 0.1               (Resistor value in ohms)

  // 1. Determine max possible current
  // MaxPossible_I = VSHUNT_MAX / RSHUNT
  // MaxPossible_I = 0.4A

  // 2. Determine max expected current
  // MaxExpected_I = 0.4A

  // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
  // MinimumLSB = MaxExpected_I/32767
  // MinimumLSB = 0.0000122              (12uA per bit)
  // MaximumLSB = MaxExpected_I/4096
  // MaximumLSB = 0.0000977              (98uA per bit)

  // 4. Choose an LSB between the min and max values
  //    (Preferrably a roundish number close to MinLSB)
  // CurrentLSB = 0.00005 (50uA per bit)

  // 5. Compute the calibration register
  // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
  // Cal = 8192 (0x2000)

  ina219_calValue = 8192;

  // 6. Calculate the power LSB
  // PowerLSB = 20 * CurrentLSB
  // PowerLSB = 0.001 (1mW per bit)

  // 7. Compute the maximum current and shunt voltage values before overflow
  //
  // Max_Current = Current_LSB * 32767
  // Max_Current = 1.63835A before overflow
  //
  // If Max_Current > Max_Possible_I then
  //    Max_Current_Before_Overflow = MaxPossible_I
  // Else
  //    Max_Current_Before_Overflow = Max_Current
  // End If
  //
  // Max_Current_Before_Overflow = MaxPossible_I
  // Max_Current_Before_Overflow = 0.4
  //
  // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
  // Max_ShuntVoltage = 0.04V
  //
  // If Max_ShuntVoltage >= VSHUNT_MAX
  //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
  // Else
  //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
  // End If
  //
  // Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
  // Max_ShuntVoltage_Before_Overflow = 0.04V

  // 8. Compute the Maximum Power
  // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
  // MaximumPower = 0.4 * 16V
  // MaximumPower = 6.4W

  // Set multipliers to convert raw current/power values
  ina219_currentDivider_mA = 20;    // Current LSB = 50uA per bit (1000/50 = 20)
  ina219_powerMultiplier_mW = 1.0f; // Power LSB = 1mW per bit

  // Set Calibration register to 'Cal' calculated above
  Adafruit_BusIO_Register calibration_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CALIBRATION, 2, MSBFIRST);
  calibration_reg.write(ina219_calValue, 2);
  // Set Config register to take into account the settings above
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
                    INA219_CONFIG_GAIN_1_40MV | INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

  Adafruit_BusIO_Register config_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CONFIG, 2, MSBFIRST);
  _success = config_reg.write(config, 2);
}

/*!
 *  @brief  Provides the the underlying return value from the last operation
 *          called on the device.
 *  @return true: Last operation was successful false: Last operation failed
 *  @note   For function calls that have intermediary device operations,
 *          e.g. calibration before read/write, only the final operation's
 *          result is stored.
 */
bool Adafruit_INA219::success() { return _success; }
