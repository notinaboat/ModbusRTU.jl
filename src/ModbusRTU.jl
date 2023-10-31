"""
# Modbus RTU for Julia

See [MODBUS Application Protocol](https://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf)
and [MODBUS Over Serial Line](https://www.modbus.org/docs/Modbus_over_serial_line_V1_02.pdf).


Use `ModbusRTU.open("/dev/tty.usbserial-1410")` to open a serial port using
[UnixIO.jl](https://github.com/notinaboat/UnixIO.jl).

If an alternate method is used to open the serial port, then methods of
the following functions must be implemented: `ModbusRTU.drain`, 
`ModbusRTU.flush`, `ModbusRTU.set_baud`.


"""
module ModbusRTU


using ReadmeDocs
using Retry
using UnixIO
using UnixIO: C
using CRC
crc_16 = crc(CRC_16_MODBUS)

greet() = print("Hello World!")


const READ_HOLDING_REGISTERS = 3
const READ_INPUT_REGISTERS = 4
const WRITE_SINGLE_COIL = 5
const WRITE_SINGLE_REGISTER = 6

const SERVER_DEVICE_FAILURE = 4
const ILLEGAL_DATA_VALUE = 3
const ILLEGAL_DATA_ADDRESS = 2
const ILLEGAL_FUNCTION = 1

# Exceptions

abstract type ModbusException <: Exception end
struct ModbusCRCError <: ModbusException end
struct ModbusTimeout <: ModbusException end

struct ModbusRequestError <: ModbusException
    code::UInt8
end



# Serial Port Interface Functions

README"""
## Open a Serial Port

    open(port, [speed=9600]) -> UnixIO.IO

Open a serial port for use with ModbusRTU.jl.

e.g. `ModbusRTU.open("/dev/tty.usbserial-1410")`
"""
function open(port; speed=9600)
    io = UnixIO.open(port, C.O_RDWR | C.O_NOCTTY)
    UnixIO.tcsetattr(io) do attr
        UnixIO.setraw(attr)
        attr.speed=speed
        attr.c_cflag |= (C.CLOCAL | C.CREAD)
    end
    return io
end


"""
Ensure that bytes buffered by `io` are transmitted now.
"""
drain(io::T) where T <: UnixIO.IO = UnixIO.tcdrain(io)


"""
Discard bytes buffered by `io`.
"""
function flush(io::T) where T <: UnixIO.IO
    UnixIO.tcflush(io, C.TCOFLUSH);
    UnixIO.tcflush(io, C.TCIFLUSH);
    while bytesavailable(io) > 0
        readavailable(io)
    end
end


README"""
## Set Serial Port Speed

    set_baud(io, speed_bps)

Set baud rate of `io` to `speed_bps`.
"""
function set_baud(io::T, speed_bps) where T <: UnixIO.IO
    flush(io)
    UnixIO.tcsetattr(io) do attr
        attr.speed=speed_bps
    end
end


README"""
## Trial Serial Port Speed

    try_baud(io, server, speed_bps) -> Bool

Attempt to contact `server` using `speed_bps`.
Return `true` on success.
"""
function try_baud(io, server, speed_bps)
    set_baud(io, speed_bps)
    ping(io, server)
end


README"""
## Detect Serial Port Speed

    auto_baud(io, server, speed_bps) -> Bool

Attempt to detect the baud rate used by `server` (9600 or 38400 bps).
Return `true` on success.
"""
function auto_baud(io, server)
    try_baud(io, server, 38400) ||
    try_baud(io, server, 9600)
end



# MODBUS Diagnostic Functions

"""
See [6.8 0x08 Diagnostics, p21](https://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf)
"""
function echo_query_data(io, server, data; kw...)
    r = request(io, server, 8, UInt16[0, data...]; kw...)
    if isodd(length(r))
        return nothing
    end
    reinterpret(UInt16, r)[2:end]
end


"""
True if `server` is reachable.
"""
function ping(io, server; attempt_count=10)
    try
        echo_query_data(io, server, [1234, 5678];
                        attempt_count,
                        timeout=0.1) == [1234, 5678]
    catch e
        if e isa ModbusTimeout
            false
        else
            rethrow(e)
        end
    end
end



# MODBUS Request Processing


# MODBUS framing code assumes little-endian.
@assert Base.ENDIAN_BOM == 0x04030201

const function_code = Dict{Symbol, Int16}()

README"""
## Modbus Requests

    request(io, server, function, [data]; [attempt_count=5], [timeout=0.5])

Send `function` request to `server`.

e.g. Send function 8 (Diagnostics) to server address 7:
`request(io, 7, 8, UInt16[0, 1, 2, 3])`
"""
function request(io, server, func, data=UInt8[]; kw...)
    if func isa Symbol
        global function_code
        func = function_code[func]
    end
    request(io, [UInt8[server, func]; data]; kw...)
end

request(io, a, f, d::Vector{UInt16}; kw...) =
    request(io, a, f, reinterpret(UInt8, d); kw...)

request(io, a, f, d::UInt16; kw...) =
    request(io, a, f, [d]; kw...)

function request(io, frame; attempt_count=5, timeout=0.5)

    @repeat attempt_count try

        @repeat attempt_count try
            send_frame(io, frame)
            response = read_frame(io; timeout)
            if (response[2] & 0x80) != 0x00
                throw(ModbusRequestError(response[3]))
            else
                return response[3:end]
            end
        catch err
            @retry if err isa ModbusCRCError
                @warn err
            end
        end

    catch err
        if err isa ModbusRequestError
            @warn ModbusRequestError frame
        end
        @retry if err isa ModbusRequestError &&
                  err.code == SERVER_DEVICE_FAILURE
            @warn "ModbusRTU SERVER_DEVICE_FAILURE"
        end
        @retry if err isa ModbusTimeout
            @warn err
            sleep(0.01)
        end
    end

end


function read_registers(io, a, f, register, count)
    r = request(io, a, f, UInt16[register, count])
    reinterpret(UInt16, r[2:end])
end

read_register(io, a, register) = read_registers(io, a, register, 1)[1]

read_registers(io, a, register, count) =
    read_registers(io, a, READ_HOLDING_REGISTERS, register, count)

read_input_register(io, a, register) = read_input_registers(io, a, register, 1)[1]

read_input_registers(io, a, register, count) =
    read_registers(io, a, READ_INPUT_REGISTERS, register, count)


function write_register(io, a, register, value)
    r = request(io, a, WRITE_SINGLE_REGISTER, UInt16[register, value])
    nothing
end


function write_coil(io, a, coil, value)
    r = request(io, a, WRITE_SINGLE_COIL, UInt16[coil, value ? 0xFF00 : 0x0000])
    nothing
end


# MODBUS RTU Framing


"""
Append CRC-16 to MODBUS `frame` and send to `io.
"""
function send_frame(io, frame::Vector{UInt8})

    flush(io)
    write(io, frame, crc_16(frame))
    drain(io)
end


"""
1.75ms delay separates frames. See [MODBUS Over Serial Line, p13](https://www.modbus.org/docs/Modbus_over_serial_line_V1_02.pdf).
"""
const modbus_inter_frame_delay = 0.00175


"""
Wait for MODBUS response frame.
"""
function read_frame(io; timeout = 0.5)

    deadline = time() + timeout
    frame = UInt8[]

    while time() < deadline

        if bytesavailable(io) == 0
            sleep(modbus_inter_frame_delay)
        end

        append!(frame, readavailable(io))

        if (length(frame) >= 4) && (crc_16(frame) == 0)
            return frame[1:end-2]
        end
        # FIXME ModbusCRCError is no longer thrown.
        # Need to reconsider timeout vs CRC error.
    end

    throw(ModbusTimeout());
end

end # module
