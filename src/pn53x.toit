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
    sleep --ms=5
    print "IRQ: $irq_.get"
    print "Writing frame $frame"
    device_.write frame
    print "writing done"

  read_frame_ max_size/int -> ByteArray:
    // When reading, the PN532 prefixes the data with a status byte.
    // If it's 0x00, we have to ignore the byte and continue reading until the
    // byte is 0x01.
    // This means that the size of the data we need to read is not known beforehand.
    // Due to the i2c protocol we can't just read single bytes one by one. Instead,
    // we have to use the interrupt pin to know when we can start reading.
    print "IRQ: $irq_.get"
    irq_.wait_for 0
    sleep --ms=5
    data := device_.read (max_size + 1)
    print "read from device: $data"
    if data.size != (max_size + 1): print "not expected size $data.size != $max_size + 1"
    if data[0] == 0x00: throw "Unexpected status byte"
    return data[1..]

  wakeup:
    // Do nothing.

class UartCommunication_ extends Communication_:
  port_/uart.Port?
  reader_/reader.BufferedReader
  writer_/writer.Writer
  is_first := true

  constructor .port_:
    reader_ = reader.BufferedReader port_
    writer_ = writer.Writer port_

  wakeup:
    // Libnfc claims that this is the wakeup preamble.
    // TODO(florian): find this information in the spec and document it.
    // Found the section: Section 7.2.11.
    print "Writing long line to wake up"
    writer_.write #[0x55, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

  counter := 0
  write_frame frame/ByteArray -> none:
    print "writing frame to uart"
    sleep --ms=5
    if is_first:
      if counter++ > 10:
        is_first = false
      // TODO(florian): for some reason I always have to send the wakeup line.
      // Otherwise the PN532 doesn't respond.
      // Turns out that the PN532 is automatically in LowVbat mode after power on.
      // Unless changed with a SAMConfiguration, it will automatically power down, which makes the
      // wakeup necessary.
      // See 3.1.3.3; page 13.
      writer_.write
        #[0x55, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
      // writer_.write
      //   #[0x00, 0x00, 0x00, 0x00, 0x00]
    writer_.write frame

  read_frame_ max_size/int -> ByteArray:
    // TODO(florian): synchronize on the preamble and start-code.
    // Probably not really necessary, as we don't expect communication errors.

    // Peek into the stream to get the size of the frame.
    print "reading frame from uart ($max_size)"
    frame_data_size := reader_.byte 3
    if  frame_data_size == 0:
      // Ack frame.
      ack := reader_.read_bytes 6
      print "received ack frame $ack"
      return ack

    frame_size := Communication_.HEADER_SIZE_ + frame_data_size + Communication_.FOOTER_SIZE_
    frame := reader_.read_bytes frame_size
    print "read frame: $frame"
    return frame

class Pn53x:
  /**
  The I2C address of the PN53x.
  Note that the user manual lists the address as 0x48, which corresponds to the shifted address.
  */
  static I2C_ADDRESS ::= 0x24
  static I2C_MAX_SPEED ::= 400_000

  static COMMAND_DIAGNOSE ::= 0x00
  static COMMAND_GET_FIRMWARE_VERSION ::= 0x02

  communication_/Communication_

  constructor.uart port/uart.Port:
    communication_ = UartCommunication_ port

  constructor.i2c device/i2c.Device --irq/gpio.Pin:
    communication_ = I2cCommunication_ device irq

  on:
    // TODO(florian): Adafruit claims that they have to do a dummy command to get synced
    // up in SPI mode.

    // TODO(florian): if a reset pin is given pull it low for 400ms. Then wait 10ms for the
    // chip to be ready.

    // TODO(florian): we are waiting 10ms here, to be sure that the device is ready.
    // We don't need to do this, if the board was powered on earlier.
    communication_.wakeup
    sleep --ms=10

  off:

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
    print "sending: $diagnostic_data"
    response := send_command COMMAND_DIAGNOSE diagnostic_data --response_size=(data.size + 1)
    print response
    return true

  /**
  Runs a self-diagnosis, checking that the ROM is correct.

  Returns whether the test was successful.
  */
  self_test --rom/bool -> bool:
    if not rom: throw "INVALID_ARGUMENT"
    ROM_TEST ::= 0x01
    response := send_command COMMAND_DIAGNOSE #[ROM_TEST] --response_size=1
    return response[0] == 0x00

  /**
  Runs a self-diagnosis, checking that the RAM is working correctly.

  Returns whether the test was successful.
  */
  self_test --ram/bool -> bool:
    if not ram: throw "INVALID_ARGUMENT"
    RAM_TEST ::= 0x02
    response := send_command COMMAND_DIAGNOSE #[RAM_TEST] --response_size=1
    return response[0] == 0x00

  get_firmware_version:
    response := send_command COMMAND_GET_FIRMWARE_VERSION #[] --response_size=4
    print "response: $response"
    if response.size != 4:
      throw "Unexpected response size: $response.size"
    print "IC: $response[0] Version: $response[1] Revision: $response[2] Support: $(%x response[3])"

  send_command command/int data/ByteArray --response_size/int -> ByteArray:
    frame_data := ByteArray data.size + 2
    frame_data[0] = 0xD4  // Frame identifier (TFI). 0xD4 for system controller to PN532; 0xD5 for responses.
    frame_data[1] = command
    frame_data.replace 2 data
    communication_.write frame_data
    read_ack
    return read_response command --size=response_size

  read_ack:
    // print (communication_.read_frame_ 6)
    // return
    // An ack-frame doesn't have any data, but we could also receive an
    // error frame which sends us 1 byte.
    ack := communication_.read --max_data_size=1
    if not ack.is_empty:
      // We got an error frame.
      // TODO(florian: extract error code).
      throw "Error frame: $ack"

  read_response command/int --size/int -> ByteArray:
    // Responses are prefixed by `0xD5` and the command+1 that was sent.
    response_size := size + 2
    data := communication_.read --max_data_size=response_size
    print "response data: $data"
    if data.size != response_size: throw "Unexpected response to $command"
    if data[0] != 0xD5: throw "Unexpected response to $command"
    if data[1] != command + 1: throw "Unexpected response to $command"
    return data[2..]
