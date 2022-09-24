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

    if not pn53x.self_test --ram: throw "ram test failed"
    print "RAM SELF TEST PASSED"

    if not pn53x.self_test --rom: throw "rom test failed"
    print "ROM SELF TEST PASSED"

    if not pn53x.self_test --communication_line "foo".to_byte_array:
      throw "communication line test failed"
    print "COMMUNICATION LINE SELF TEST PASSED"

    print pn53x.firmware_version

    print pn53x.general_status

    while true:
      bytes := pn53x.in_list_passive_targets
      print "found target: $bytes"
      deselect_worked := pn53x.in_deselect --target_number=0

//      serial_print_heap_report
      sleep --ms=100

    pn53x.off
