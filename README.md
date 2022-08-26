# Modbus RTU for Julia

See [MODBUS Application Protocol](https://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf) and [MODBUS Over Serial Line](https://www.modbus.org/docs/Modbus_over_serial_line_V1_02.pdf).

Use `ModbusRTU.open("/dev/tty.usbserial-1410")` to open a serial port using [UnixIO.jl](https://github.com/notinaboat/UnixIO.jl).

If an alternate method is used to open the serial port, then methods of the following functions must be implemented: `ModbusRTU.drain`,  `ModbusRTU.flush`, `ModbusRTU.set_baud`.

## Open a Serial Port

    open(port, [speed=9600]) -> UnixIO.IO

Open a serial port for use with ModbusRTU.jl.

e.g. `ModbusRTU.open("/dev/tty.usbserial-1410")`

## Set Serial Port Speed

    set_baud(io, speed_bps)

Set baud rate of `io` to `speed_bps`.

## Trial Serial Port Speed

    try_baud(io, server, speed_bps) -> Bool

Attempt to contact `server` using `speed_bps`.
Return `true` on success.

## Detect Serial Port Speed

    auto_baud(io, server, speed_bps) -> Bool

Attempt to detect the baud rate used by `server` (9600 or 38400 bps).
Return `true` on success.

## Modbus Requests

    request(io, server, function, [data]; [attempt_count=5], [timeout=0.5])

Send `function` request to `server`.

e.g. Send function 8 (Diagnostics) to server address 7:
`request(io, 7, 8, UInt16[0, 1, 2, 3])`
