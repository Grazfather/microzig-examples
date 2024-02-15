const std = @import("std");
const microzig = @import("microzig");
const rp2040 = microzig.hal;
const gpio = rp2040.gpio;
const Pio = rp2040.pio.Pio;
const StateMachine = rp2040.pio.StateMachine;
const dma = rp2040.dma;

const ws2812_program = blk: {
    @setEvalBranchQuota(10000);
    const p = rp2040.pio.assemble(
        \\.program ws2812
        \\.side_set 1 opt
        \\
        \\    pull block                  ; pull the next data (number of LEDs - 1)
        \\    out y, 32                   ; and put it to "Y"
        \\
        \\.wrap_target
        \\    mov x, y                    ; initialise the LED counter "X" with the value of "Y"
        \\next_led:
        \\    pull block                  ; pull the next data (RGB value for a LED)
        \\    irq 0                       ; set INT 0 flag
        \\    out null, 8                 ; ignore the trailing 8 bits
        \\
        \\next_bit:
        \\    mov pins, !null             ; set the GPIO to "1"
        \\; this could also be: "nop side 1"
        \\
        \\    out pins, 1                 ; the middle 1/3 is equal to actual value we send
        \\    jmp !osre next_bit side 0   ; set the GPIO to "0", jump if the shift register contains more bits
        \\                                ; shift register empty -> no more data to send for a LED
        \\
        \\    jmp x-- next_led            ; jump if more LEDs to send data to
        \\                                ; here we are done - introduce the 50us reset delay
        \\                                ; wait 120 cycles (50us = 40 bit lengths * 3)
        \\    set x, 14                   ; 120 = 15 * 8 (set "X" to 15-1)
        \\delay_loop:
        \\    jmp x-- delay_loop [7]      ; 1 + 7 cycles in each iteration
        \\
        \\    irq clear 0                 ; clear INT 0 flag
        \\.wrap
    , .{}).get_program_by_name("ws2812");
    @compileLog(p);
    break :blk p;
};

const pio: Pio = .pio0;
const sm: StateMachine = .sm0;
const led_pin = gpio.num(26);

pub fn main() void {
    pio.gpio_init(led_pin);
    sm_set_consecutive_pindirs(pio, sm, @intFromEnum(led_pin), 1, true);

    const div = (@as(f32, @floatFromInt(rp2040.clock_config.sys.?.output_freq)) / 1e6) * 1.25 / 3.0;

    // std.fmt.comptimePrint("xxxx{?}", .{ws2812_program.side_set});
    pio.sm_load_and_start_program(sm, ws2812_program, .{
        .clkdiv = rp2040.pio.ClkDivOptions.from_float(div),
        .pin_mappings = .{
            .side_set = .{
                .base = @intFromEnum(led_pin),
                .count = 1,
            },
        },
        .shift = .{
            .out_shiftdir = .left,
            .autopull = true,
            .pull_threshold = 24,
            .join_tx = true,
        },
    }) catch unreachable;

    // pio.sm_set_exec_options(sm, .{
    //     .side_set_optional = true,
    // });
    pio.sm_set_enabled(sm, true);

    const LED_COUNT = 8;
    const colours = [_]u32{ 0x00ACCC, 0x1D5DC5, 0x541ECB, 0x9C1CEE, 0xB30D8B, 0xBC1303, 0xD6AB01, 0xBEFE00, 0x69FF03, 0x26FF00, 0x00FF19 };
    var led_buffer = [_]u32{0} ** LED_COUNT;
    var first_colour: u32 = 0;

    // Config DMA to copy LED buffer to the PIO statemachine's FIFO
    // dma_channel_config dma_ch0 = dma_channel_get_default_config(led_dma_chan);
    // const DREQ_PIO0_TX0 = 0;
    // var dma_ch0 = dma.channel(0);
    // var dma_config = dma.Channel.TransferConfig{
    //     .transfer_size_bytes = 2,
    //     .enable = true,
    //     .read_increment = true,
    //     .write_increment = false,
    //     .dreq = @enumFromInt(DREQ_PIO0_TX0),
    // };
    // channel_config_set_transfer_data_size(&dma_ch0, DMA_SIZE_32);
    // channel_config_set_read_increment(&dma_ch0, true);
    // channel_config_set_write_increment(&dma_ch0, false);
    // channel_config_set_dreq(&dma_ch0, DREQ_PIO0_TX0);

    // Tell PIO how many LEDs we have
    pio.sm_blocking_write(sm, LED_COUNT - 1);

    while (true) {
        for (0..LED_COUNT) |i| {
            led_buffer[i] = colours[(i + first_colour) % LED_COUNT];
        }
        // dma_channel_configure(led_dma_chan, &dma_ch0, &pio->txf[sm], led_buffer, LED_COUNT, true);
        // var addr = pio.sm_get_tx_fifo(sm);
        // dma_ch0.trigger_transfer(@intFromPtr(addr), @intFromPtr(&led_buffer), LED_COUNT, dma_config);

        first_colour = (first_colour + 1) % LED_COUNT;

        pio.sm_blocking_write(sm, led_buffer[0]); //red
        rp2040.time.sleep_ms(500);

        // rp2040.time.sleep_ms(1000);
        // pio.sm_blocking_write(sm, 0xff0000 << 8); //green
        // rp2040.time.sleep_ms(1000);
        // pio.sm_blocking_write(sm, 0x0000ff << 8); //blue
        // rp2040.time.sleep_ms(1000);

    }
}

fn sm_set_consecutive_pindirs(_pio: Pio, _sm: StateMachine, pin: u5, count: u3, is_out: bool) void {
    const sm_regs = _pio.get_sm_regs(_sm);
    const pinctrl_saved = sm_regs.pinctrl.raw;
    sm_regs.pinctrl.modify(.{
        .SET_BASE = pin,
        .SET_COUNT = count,
    });
    _pio.sm_exec(_sm, rp2040.pio.Instruction{
        .tag = .set,
        .delay_side_set = 0,
        .payload = .{
            .set = .{
                .data = @intFromBool(is_out),
                .destination = .pindirs,
            },
        },
    });
    sm_regs.pinctrl.raw = pinctrl_saved;
}
