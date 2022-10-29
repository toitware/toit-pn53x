import gpio
import i2c
import uart
import .pn53x

SDA ::= 16
SCL ::= 17

IRQ ::= 34
// IRQ ::= 25


using_irc [block]:
  sda := gpio.Pin SDA
  scl := gpio.Pin SCL
  irq := gpio.Pin IRQ

  bus := i2c.Bus --sda=sda --scl=scl --frequency=100_000
  // print bus.scan

  device := bus.device Pn53x.I2C_ADDRESS

  pn53x := Pn53x.i2c device --irq=irq

  block.call pn53x

  device.close
  bus.close
  // port.close
  irq.close
  scl.close
  sda.close

RX ::= 16
TX ::= 17

using_uart [block]:
  rx := gpio.Pin RX
  tx := gpio.Pin TX

  port := uart.Port --rx=rx --tx=tx --baud_rate=115_200

  pn53x := Pn53x.uart port

  block.call pn53x

  port.close
  tx.close
  rx.close

main:
  print "starting"
  // port := uart.Port --rx=(gpio.Pin RX) --tx=(gpio.Pin TX) --baud_rate=115200
  // pn53x := Pn53x.uart port

  // using_irc: | pn53x/Pn53x |
  using_uart: | pn53x/Pn53x |
    pn53x.on

    /*
    if not pn53x.miscellaneous.self_test --ram: throw "ram test failed"
    print "RAM SELF TEST PASSED"

    if not pn53x.miscellaneous.self_test --rom: throw "rom test failed"
    print "ROM SELF TEST PASSED"

    if not pn53x.miscellaneous.self_test --communication_line "foo".to_byte_array:
      throw "communication line test failed"
    print "COMMUNICATION LINE SELF TEST PASSED"

    print pn53x.miscellaneous.firmware_version

    print pn53x.miscellaneous.general_status
    */

    print "Waiting for cards..."
    while true:
      bytes := pn53x.initiator.list_passive_targets
      print "found target: $bytes"
      uid := bytes[6..]

      card := MifareCard pn53x uid
      for i := 63; i >= 0; i--:
        is_first_block_in_sector := (i + 1) % 4 == 0
        if is_first_block_in_sector:
          exception := catch: card.authenticate --block=i
          if exception:
            print "Sector $i encrypted"
            print "Communication with card is compromised"
            break
        print (card.read --block=i)
      deselect_worked := pn53x.initiator.deselect --target_number=0

//      serial_print_heap_report
      print "powering down for 1000ms"
      pn53x.power.power_down
      sleep --ms=1000
      print "waking up"
      pn53x.power.wakeup

    pn53x.off

class MifareCard:
  pn53x_/Pn53x
  uid/ByteArray

  constructor .pn53x_ .uid:

  /** Default key for Mifare cards. */
  static DEFAULT_KEY := #[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]

  authenticate --block/int key/ByteArray=DEFAULT_KEY:
    MIFARE_CMD_AUTH_A_ ::= 0x60
    MIFARE_CMD_AUTH_B_ ::= 0x61
    data := #[MIFARE_CMD_AUTH_A_, block]  // Target 1.
    data += key
    data += uid
    pn53x_.initiator.data_exchange data --target_number=1 --max_response_size=0

  read --block/int:
    // TODO(florian): check validity of block.
    MIFARE_READ ::= 0x30
    // TODO(florian): we probably don't need the "0x00" at the end.
    // In the MFRC522 library, they are used to store the CRC (bad API).
    bytes := #[ MIFARE_READ, block, 0x00, 0x00]
    result := pn53x_.initiator.data_exchange bytes --target_number=1 --max_response_size=16
    return result
