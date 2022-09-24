// Copyright (C) 2022 Toitware ApS. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

/**
A driver for the PN532 and PN533 NFC reader.
*/

import uart
import gpio
import i2c
import reader
import writer

abstract class Communication_:
  static HEADER_SIZE_ ::= 5  // Preamble, start code * 2, length, length checksum.
  static FOOTER_SIZE_ ::= 2  // Data checksum and postamble.

  abstract read_frame_ max_size/int -> ByteArray
  abstract write_frame frame/ByteArray
  abstract wakeup

  frame_size_ data/ByteArray:
    return data[3] + 6

  /**
  Reads data from the NFC reader.

  The frame must not have more than $max_data_size data bytes.
  */
  read --max_data_size/int -> ByteArray:
    max_header_size := 5  // Preamble, start code * 2, length and length checksum.
    max_footer_size := 2  // Data checksum and postamble.
    frame := read_frame_ (HEADER_SIZE_ + max_data_size + FOOTER_SIZE_)

  // Remove header and footer from frame.
    if frame[0] != 0x00 or frame[1] != 0x00 or frame[2] != 0xFF:
      throw "Invalid frame"
    size := frame[3]

    if size == 0: // Ack frame.
      if frame[4] != 0xFF or frame[5] != 0x00:  // Ack frame.
        throw "Invalid frame"
      return #[]

    if size > 0 and frame[4] != 0x100 - size:
      throw "Invalid frame"
    result := frame[5..size + 5]

    dcs := frame[5 + size]
    for i := 0; i < size; i++:
      dcs = (dcs + result[i]) & 0xFF
    if dcs != 0:
      throw "Invalid frame"

    if frame[5 + size + 1] != 0x00:
      throw "Invalid frame"

    return result

  write data/ByteArray:
    header_size := 5
    footer_size := 2
    frame_size := header_size + data.size + footer_size

    frame := ByteArray frame_size

    index := 0
    frame[index++] = 0x00 // Preamble: 1 byte. Equal to 0x00
    frame[index++] = 0x00 // Start code. 2 bytes.
    frame[index++] = 0xFF
    frame[index++] = data.size // Length. 1 byte.
    frame[index++] = 0x100 - data.size // Length checksum. 1 byte.
    assert: index == header_size
    frame.replace index data
    index += data.size

    data_checksum := 0
    for i := 0; i < data.size; i++:
      data_checksum = (data_checksum + data[i]) & 0xFF
    frame[index++] = 0x100 - data_checksum // Data checksum. 1 byte.
    frame[index++] = 0x00 // Postamble. 1 byte.

    write_frame frame

class I2cCommunication_ extends Communication_:
  device_/i2c.Device
  irq_/gpio.Pin

  constructor .device_ .irq_:
    irq_.configure --input

  write_frame frame/ByteArray -> none:
    device_.write frame

  read_frame_ max_size/int -> ByteArray:
    // When reading, the PN532 prefixes the data with a status byte.
    // If it's 0x00, we have to ignore the byte and continue reading until the
    // byte is 0x01.
    // This means that the size of the data we need to read is not known beforehand.
    // Due to the i2c protocol we can't just read single bytes one by one. Instead,
    // we have to use the interrupt pin to know when we can start reading.
    // print "waiting for irq to become 0 $irq_.get"
    to_exception := catch: with_timeout --ms=15_000:
      irq_.wait_for 0
    if to_exception: print "*****************  to_exception: $to_exception $irq_.get"
    // print "irq is 0"
    data /ByteArray? := null
    for i := 0; i < 100; i++:
      data = device_.read (max_size + 1)
      if data[0] != 0x00: break
      // TODO(florian): why does the IRQ go high, if we don't have data yet?
      print "*** NEED LOOP *** $irq_.get"
      sleep --ms=10
    if data.size != (max_size + 1): print "not expected size $data.size != $max_size + 1"
    if data[0] == 0x00: throw "Unexpected status byte"
    return data[1..]

  wakeup:
    // Do nothing.

class UartCommunication_ extends Communication_:
  port_/uart.Port?
  reader_/reader.BufferedReader
  writer_/writer.Writer

  constructor .port_:
    reader_ = reader.BufferedReader port_
    writer_ = writer.Writer port_

  wakeup:
    // Libnfc claims that this is the wakeup preamble.
    // TODO(florian): find this information in the spec and document it.
    // Found the section: Section 7.2.11.
    writer_.write #[0x55, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

  counter := 0
  write_frame frame/ByteArray -> none:
    writer_.write frame

  read_frame_ max_size/int -> ByteArray:
    // TODO(florian): synchronize on the preamble and start-code.
    // Probably not really necessary, as we don't expect communication errors.

    // Peek into the stream to get the size of the frame.
    frame_data_size := reader_.byte 3
    if  frame_data_size == 0:
      // Ack frame.
      ack := reader_.read_bytes 6
      return ack

    frame_size := Communication_.HEADER_SIZE_ + frame_data_size + Communication_.FOOTER_SIZE_
    frame := reader_.read_bytes frame_size
    return frame

/*
class SpiCommunication_ extends Communication_:
  wakeup:
    // TODO(florian): Adafruit claims that they have to do a dummy command to get synced
    // up in SPI mode.
    // I'm assuming that's to wake the device up.
*/

class Pn53x:
  /**
  The I2C address of the PN53x.
  Note that the user manual lists the address as 0x48, which corresponds to the shifted address.
  */
  static I2C_ADDRESS ::= 0x24
  static I2C_MAX_SPEED ::= 400_000

  static COMMAND_DIAGNOSE_ ::= 0x00
  static COMMAND_GET_FIRMWARE_VERSION_ ::= 0x02
  static COMMAND_GET_GENERAL_STATUS_ ::= 0x04
  static COMMAND_SAM_CONFIGURATION_ ::= 0x14
  static COMMAND_IN_DESELECT_ ::= 0x44
  static COMMAND_IN_LIST_PASSIVE_TARGET_ ::= 0x4A

  static POWER_MODE_NORMAL_ ::= 0
  // Only on PN532:
  /**
  The chip is powered down and needs to be woken up using $Communication_.wakeup.
  */
  static POWER_MODE_POWER_DOWN_ ::= 1
  /**
  The chip is powered down and needs to be woken up using $Communication_.wakeup.
  To avoid it going back to sleep, the SAMConfiguration command needs to be sent as well.
  */
  static POWER_MODE_LOW_VBAT_ ::= 2

  static SAM_MODE_NORMAL_ ::= 0x01
  static SAM_MODE_VIRTUAL_CARD_ ::= 0x02
  static SAM_MODE_WIRED_CARD_ ::= 0x03
  static SAM_MODE_DUAL_CARD_ ::= 0x04

  static CHIP_PN532_ ::= 0x00
  static CHIP_PN533_ ::= 0x01

  // TODO(florian): test with PN533 as well.
  chip_type_/int ::= CHIP_PN532_

  communication_/Communication_
  reset_pin_/gpio.Pin?

  /**
  The current power mode of the device.

  If null, then the device was not turned on yet (see $on).

  We assume that the device is reset when an instance of this class is created.
    As such, the initial power mode is $POWER_MODE_LOW_VBAT_.

  User manual 141520, Rev. 02. Section 3.1.3.3, page 13.
  */
  power_mode_/int? := null

  constructor.uart port/uart.Port --reset/gpio.Pin?=null:
    communication_ = UartCommunication_ port
    reset_pin_ = reset

  constructor.i2c device/i2c.Device --irq/gpio.Pin --reset/gpio.Pin?=null:
    communication_ = I2cCommunication_ device irq
    reset_pin_ = reset

  on:
    reset_
    wakeup

  /**
  Resets the device if a reset pin was given.
  */
  reset_:
    if reset_pin_:
      reset_pin_.set 0
      sleep --ms=10
      reset_pin_.set 1

    power_mode_ = (chip_type_ == CHIP_PN532_) ? POWER_MODE_LOW_VBAT_ : POWER_MODE_NORMAL_

    // TODO(florian): we are waiting 10ms here, to be sure that the device is ready.
    // We don't need to do this, if the board was powered on earlier and no reset pin
    // was given.
    sleep --ms=10

  off:
    // TODO(florian): send powerdown command.

  wakeup:
    // Wakeup the chip so we can communicate with it.
    // It's safe to do this even if the chip is already awake.
    communication_.wakeup

    if power_mode_ == POWER_MODE_LOW_VBAT_:
      // Switch to normal mode.
      // This is done by sending a SAMConfiguration command.
      // See user manual 141520, Rev. 02. Section 3.1.3.3, page 13.
      set_sam_configuration_ SAM_MODE_NORMAL_
    power_mode_ = POWER_MODE_NORMAL_

  /**
  Configures the SAM (Security Access Module).

  It is also used to set the power mode of the device to normal.

  The timeout is only required for the Virtual Card mode.
  */
  // TODO(florian): the timeout is in 50ms units. It may be null.
  // Should we take `--timeout_ms` instead?
  set_sam_configuration_ mode/int --timeout/int?=null -> none:
    if timeout or mode == SAM_MODE_VIRTUAL_CARD_: throw "UNIMPLEMENTED"
    // In theory we could also take over the P70_IRQ pin, but we don't need that yet.
    data := #[mode]
    send_command COMMAND_SAM_CONFIGURATION_ data --response_size=0

  /*
  // TODO(florian): can't find good parameters for this.
  // Also seems to require that the board is powered by 5V.
  /**
  Runs a self-diagnostic test, checking the continuity of the transmission paths of the antenna.

  Returns whether the antenna was detected.

  # Advanced:
  This test
  */
  self_test --antenna/bool threshold/int -> bool:
    if not 0 <= threshold <= 0xFF: throw "INVALID_ARGUMENT"

    if not antenna: throw "INVALID_ARGUMENT"
    ANTENNA_TEST ::= 0x07
    response := send_command COMMAND_DIAGNOSE #[ANTENNA_TEST] --response_size=1
    return response[0] == 0x00
  */

  /**
  Runs a diagnostic test, sending $data to the device and checking that the
    data is echoed back.

  The data must have at most 262 bytes.
  */
  self_test --communication_line/bool data/ByteArray -> bool:
    if not communication_line: throw "INVALID_ARGUMENT"
    if data.size > 262: throw "INVALID_ARGUMENT"

    if data.size >= 254: throw "UNIMPLEMENTED"

    COMMUNICATION_LINE_TEST ::= 0x00
    diagnostic_data := #[COMMUNICATION_LINE_TEST] + data
    response := send_command COMMAND_DIAGNOSE_ diagnostic_data --response_size=(data.size + 1)
    return diagnostic_data == response

  /**
  Runs a self-diagnosis, checking that the ROM is correct.

  Returns whether the test was successful.
  */
  self_test --rom/bool -> bool:
    if not rom: throw "INVALID_ARGUMENT"
    ROM_TEST ::= 0x01
    response := send_command COMMAND_DIAGNOSE_ #[ROM_TEST] --response_size=1
    return response[0] == 0x00

  /**
  Runs a self-diagnosis, checking that the RAM is working correctly.

  Returns whether the test was successful.
  */
  self_test --ram/bool -> bool:
    if not ram: throw "INVALID_ARGUMENT"
    RAM_TEST ::= 0x02
    response := send_command COMMAND_DIAGNOSE_ #[RAM_TEST] --response_size=1
    return response[0] == 0x00

  firmware_version -> FirmwareVersion:
    response := send_command COMMAND_GET_FIRMWARE_VERSION_ #[] --response_size=4
    return FirmwareVersion response[0] response[1] response[2] response[3]

  general_status -> GeneralStatus:
    min_response_size := 1 // Error code.
        + 1  // Field, indicates if an external RF field is present. (1 or 0).
        + 1  // NbTg, number of targets currently controlled by the PN532 acting as initiator.
        + 1  // SAM status
    max_response_size := min_response_size + 2 * 4  // Up to two target infos.
    response := send_command COMMAND_GET_GENERAL_STATUS_ #[] --max_response_size=max_response_size
    if response.size < 4: throw "INVALID_RESPONSE"
    index := 0
    error_code := response[index++]
    field_present := response[index++] != 0
    nb_targets := response[index++]
    if not 0 <= nb_targets <= 2: throw "Unexpected number of controlled targets $nb_targets"
    if response.size < min_response_size + nb_targets * 4: throw "INVALID_RESPONSE"
    target_infos := List nb_targets:
      logical_number := response[index++]
      encoded_bit_rate_reception := response[index++]
      bit_rate_reception/int := ?
      if encoded_bit_rate_reception == 0x00: bit_rate_reception = 106_000
      else if encoded_bit_rate_reception == 0x01: bit_rate_reception = 212_000
      else if encoded_bit_rate_reception == 0x02: bit_rate_reception = 424_000
      else: throw "Unexpected bit-rate value."
      encoded_bit_rate_transmission := response[index++]
      bit_rate_transmission/int := ?
      if encoded_bit_rate_transmission == 0x00: bit_rate_transmission = 106_000
      else if encoded_bit_rate_transmission == 0x01: bit_rate_transmission = 212_000
      else if encoded_bit_rate_transmission == 0x02: bit_rate_transmission = 424_000
      else: throw "Unexpected bit-rate value."
      modulation_type := response[index++]
      TargetInfo logical_number bit_rate_reception bit_rate_transmission modulation_type

    sam_status := SamStatus response[index++]

    return GeneralStatus error_code field_present target_infos sam_status

  // TODO(florian): change name.
  // I think the meaning is: list of new (in) passive targets.
  in_list_passive_targets:
    max_targets := 1
    baud_rate_modulation_type := 0x00  // Mifare.
    min_response_size := 1  // NbTg, equal to 0.
    max_target_data_size := 1 // Target-number
        + 1   // SEL_RES  (TODO(florian): what is this?). Seems to be 0x08. Probably SAK.
        + 1   // NFCIDLength
        + 10  // NFCID  (TODO(florian): is 10 the most we can have?)
        + 1   // ATS. Seems to be only present when the target requires a RATS.
    max_response_size := 1  // nbTg, number of initialized targets.
        + 2 * max_target_data_size

    // Next bytes could be used to specify the UID of the target to look for.
    response := send_command COMMAND_IN_LIST_PASSIVE_TARGET_
        #[max_targets, baud_rate_modulation_type]
        --max_response_size=max_response_size

    return response

  // target_number == 0: deselect all targets.
  in_deselect --target_number/int -> bool:
    if not 0 <= target_number <= 2: throw "INVALID_ARGUMENT"
    response := send_command COMMAND_IN_DESELECT_ #[target_number] --response_size=1
    print "deselect response code: $response[0]"
    return response[0] == 0x00

  send_command command/int data/ByteArray --max_response_size/int -> ByteArray:
    frame_data := ByteArray data.size + 2
    frame_data[0] = 0xD4  // Frame identifier (TFI). 0xD4 for system controller to PN532; 0xD5 for responses.
    frame_data[1] = command
    frame_data.replace 2 data
    communication_.write frame_data
    read_ack
    return read_response_ command --max_size=max_response_size

  send_command command/int data/ByteArray --response_size/int -> ByteArray:
    response := send_command command data --max_response_size=response_size
    if response.size != response_size: throw "INVALID_RESPONSE"
    return response

  read_ack:
    // print (communication_.read_frame_ 6)
    // return
    // An ack-frame doesn't have any data, but we could also receive an
    // error frame which sends us 1 byte.
    ack := communication_.read --max_data_size=0
    if not ack.is_empty:
      // We got an error frame.
      // TODO(florian: extract error code).
      throw "Error frame: $ack"

  read_response_ command/int --max_size/int -> ByteArray:
    // Responses are prefixed by `0xD5` and the command+1 that was sent.
    response_size := max_size + 2
    data := communication_.read --max_data_size=response_size
    if data[0] != 0xD5: throw "Unexpected response to $command"
    if data[1] != command + 1: throw "Unexpected response to $command"
    return data[2..]

class FirmwareVersion:
  ic/int
  version/int
  revision/int
  support/int

  constructor .ic .version .revision .support:

  supports_iso14443_type_a -> bool:
    return support & 0x01 != 0

  supports_iso14443_type_b -> bool:
    return support & 0x02 != 0

  supports_is18092 -> bool:
    return support & 0x04 != 0

  stringify -> string:
    ic_name/string := ?
    if ic == 0x32:
      ic_name = "PN532"
    else:
      // TODO(florian): Check if PN533 has ic equal to 0x33
      ic_name = "Unknown IC: $ic"

    supports_str := ""
    if supports_iso14443_type_a: supports_str += " ISO14443A"
    if supports_iso14443_type_b: supports_str += " ISO14443B"
    if supports_is18092: supports_str += " ISO18092"
    return "IC: $ic_name, Version: $version, Revision: $revision, Supports:$supports_str"

class GeneralStatus:
  error_code/int
  field_present/bool
  target_infos/List
  sam_status/SamStatus

  constructor .error_code .field_present .target_infos .sam_status:

  stringify -> string:
    return "Error code: $error_code, Field present: $field_present, Target infos: $target_infos, SAM status: $sam_status"

class SamStatus:
  status_bits/int

  constructor .status_bits:

  /**
  Whether a full negative pulse has been detected on the CLAD line.
  */
  negative_pulse_detected -> bool: return status_bits & 0x01 != 0

  /**
  Whether an external RF field has been detected and switched off during or after a transaction.
  */
  external_rf_detected_and_off -> bool: return status_bits & 0x02 != 0
  /**
  Whether a timeout has been detected after SigActIRQ has fell down.
  */
  timeout_after_sig_act_irg -> bool: return status_bits & 0x04 != 0

  /**
  The CLAD line level. High if 1, low if 0.
  */
  clad_line_level -> int: return status_bits >> 7

  stringify -> string:
    return "Negative pulse detected: $negative_pulse_detected, External RF detected and off: $external_rf_detected_and_off, Timeout after SigActIRQ: $timeout_after_sig_act_irg, CLAD line level: $clad_line_level"

class TargetInfo:
  logical_number/int
  bit_rate_reception/int
  bit_rate_transmission/int
  modulation_type/int

  constructor .logical_number .bit_rate_reception .bit_rate_transmission .modulation_type:

  modulation_type_as_string -> string:
    // User manual, seciton 7.2.3, page 74
    if modulation_type == 0x00:
      return "Mifare, Iso14443-3 Type A, Iso14443-3 Type B, Iso18092 passive 106 kpbs"
    else if modulation_type == 0x10:
      return "FeliCa, Iso18092 passive 212/424 kpbs"
    else if modulation_type == 0x01:
      return "Iso18092 Active mode"
    else if modulation_type == 0x02:
      return "Innovision Jewel tag"
    return "Uknown modulation type"

  stringify -> string:
    return "Logical number: $logical_number, Bit rate reception: $bit_rate_reception, Bit rate transmission: $bit_rate_transmission, Modulation type: $modulation_type_as_string"
