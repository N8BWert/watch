//!
//! Code that runs on the watch I'm making.  Currently, this runs on a Pico
//! but that is obviously way larger of a microcontroller than I need so I'll
//! be switching to something else in the future.  I just have a ton of Picos
//! on hand
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use panic_probe as _;

use defmt_rtt as _;

use embedded_alloc::LlffHeap as Heap;
#[global_allocator]
static HEAP: Heap = Heap::empty();

extern crate alloc;
use alloc::format;

#[rtic::app(
    device = rp_pico::hal::pac,
    peripherals = true,
    dispatchers = [SW0_IRQ, SW1_IRQ]
)]
mod app {
    use super::*;
    use core::mem::MaybeUninit;

    use bsp::hal::{clocks::init_clocks_and_plls, gpio::Pins, sio::Sio, watchdog::Watchdog, Clock};
    use fugit::RateExtU32;
    use rp_pico::{
        self as bsp,
        hal::{
            gpio::Interrupt, rtc::{DateTimeFilter, RealTimeClock}, I2C
        },
    };

    use embedded_graphics::{
        mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
        pixelcolor::BinaryColor,
        prelude::*,
        text::{Baseline, Text},
    };
    use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306Async};

    use rtic_monotonics::rp2040::prelude::*;

    use watch::{peripherals::{BackButton, DecrementButton, Display, ForwardButton, IncrementButton, PowerSwitch}, RealtimeDatetime, State};

    const EXTERNAL_XTAL_FREQ_HZ: u32 = 12_000_000u32;

    const HEAP_SIZE: usize = 4096;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    rp2040_timer_monotonic!(Mono);

    /// The default datetime the watch will use if it is wiped or newly flashed
    static mut REFERENCE_DATETIME: RealtimeDatetime = RealtimeDatetime::new();

    #[shared]
    struct Shared {
        display: Option<Display>,
        rtc: RealTimeClock,
        state: State,
        alarm_time: Option<(u8, u8, u8)>,
    }

    #[local]
    struct Local {
        power_switch: PowerSwitch,
        forward_button: ForwardButton,
        back_button: BackButton,
        increment_button: IncrementButton,
        decrement_button: DecrementButton,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        unsafe {
            HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE);
        }

        Mono::start(ctx.device.TIMER, &mut ctx.device.RESETS);

        let mut watchdog = Watchdog::new(ctx.device.WATCHDOG);

        let clocks = init_clocks_and_plls(
            EXTERNAL_XTAL_FREQ_HZ,
            ctx.device.XOSC,
            ctx.device.CLOCKS,
            ctx.device.PLL_SYS,
            ctx.device.PLL_USB,
            &mut ctx.device.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let sio = Sio::new(ctx.device.SIO);
        let pins = Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut ctx.device.RESETS,
        );

        let i2c = I2C::i2c1(
            ctx.device.I2C1,
            pins.gpio18.reconfigure(),
            pins.gpio19.reconfigure(),
            400.kHz(),
            &mut ctx.device.RESETS,
            125_000_000.Hz(),
        );

        let display = Ssd1306Async::new(
            I2CDisplayInterface::new(i2c),
            DisplaySize128x64,
            DisplayRotation::Rotate0,
        )
        .into_buffered_graphics_mode();

        let rtc = RealTimeClock::new(
            ctx.device.RTC,
            clocks.rtc_clock,
            &mut ctx.device.RESETS,
            unsafe { REFERENCE_DATETIME.to_datetime() },
        )
        .unwrap();

        // Use pin0 as a power switch to switch the device between sleep and operations mode
        let power_switch = pins.gpio0.into_pull_down_input();
        power_switch.set_interrupt_enabled(Interrupt::EdgeHigh, true);
        power_switch.set_interrupt_enabled(Interrupt::EdgeLow, true);

        let forward_button = pins.gpio10.into_pull_down_input();
        forward_button.set_interrupt_enabled(Interrupt::EdgeHigh, true);

        let back_button = pins.gpio11.into_pull_down_input();
        back_button.set_interrupt_enabled(Interrupt::EdgeHigh, true);

        let increment_button = pins.gpio12.into_pull_down_input();
        increment_button.set_interrupt_enabled(Interrupt::EdgeHigh, true);

        let decrement_button = pins.gpio13.into_pull_down_input();
        decrement_button.set_interrupt_enabled(Interrupt::EdgeHigh, true);

        initialize_display::spawn().ok();

        (
            Shared {
                display: Some(display),
                rtc,
                state: State::default(),
                alarm_time: None,
            },
            Local {
                forward_button,
                back_button,
                increment_button,
                decrement_button,
                power_switch,
            },
        )
    }

    /// Initialize the OLED Display.
    #[task(
        shared = [display],
        priority = 1
    )]
    async fn initialize_display(mut ctx: initialize_display::Context) {
        let mut display = ctx.shared.display.lock(|display| display.take().unwrap());

        display.init().await.unwrap();

        // Display start text
        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(BinaryColor::On)
            .build();

        Text::with_baseline(
            "Starting Watch",
            Point::zero(),
            text_style,
            Baseline::Middle,
        )
        .draw(&mut display)
        .unwrap();

        Mono::delay(1_000u64.millis()).await;

        ctx.shared.display.lock(|d| d.replace(display));

        display_time::spawn().ok();
    }

    /// Visualization loop that actually updates the information on the OLED Display
    #[task(
        shared = [rtc, display, state, alarm_time],
        priority = 1
    )]
    async fn display_time(mut ctx: display_time::Context) {
        let mut display = ctx.shared.display.lock(|display| display.take().unwrap());
        loop {
            let current_time = ctx
                .shared
                .rtc
                .lock(|rtc| RealtimeDatetime::from(rtc.now().unwrap()));

            display.flush().await.unwrap();

            let text_style = MonoTextStyleBuilder::new()
                .font(&FONT_6X10)
                .text_color(BinaryColor::On)
                .build();

            match ctx.shared.state.lock(|state| *state) {
                State::Sleep => (),
                State::Time => {
                    // Display Date
                    Text::with_baseline(
                        current_time.date().as_str(),
                        Point::zero(),
                        text_style,
                        Baseline::Top,
                    )
                    .draw(&mut display)
                    .unwrap();

                    // Display Time
                    Text::with_baseline(
                        current_time.time().as_str(),
                        Point::new(0, 16),
                        text_style,
                        Baseline::Top,
                    )
                    .draw(&mut display)
                    .unwrap();
                },
                State::SettingAlarm => {
                    if let Some(alarm_time) = ctx.shared.alarm_time.lock(|alarm_time| *alarm_time) {
                        Text::with_baseline(
                            format!(
                                "{}:{}:{}",
                                alarm_time.0,
                                alarm_time.1,
                                alarm_time.2,
                            ).as_str(),
                            Point::zero(),
                            text_style,
                            Baseline::Middle,
                        )
                        .draw(&mut display)
                        .unwrap();
                    } else {
                        ctx.shared.alarm_time.lock(|alarm_time| alarm_time.replace((0, 0, 0)));
                        Text::with_baseline(
                            "00:00:00",
                            Point::zero(),
                            text_style,
                            Baseline::Middle,
                        )
                        .draw(&mut display)
                        .unwrap();
                    }
                },
                State::Alarm => {
                    // Display Time left in Alarm
                    Text::with_baseline(
                        format!("").as_str(),
                        Point::zero(),
                        text_style,
                        Baseline::Middle,
                    )
                    .draw(&mut display)
                    .unwrap();
                },
                State::AlarmTriggered => {
                    // Display text that the alarm has triggered
                    Text::with_baseline(
                        "!!! ALARM !!!",
                        Point::zero(),
                        text_style,
                        Baseline::Middle,
                    )
                    .draw(&mut display)
                    .unwrap();
                },
            }

            display.flush().await.unwrap();

            Mono::delay(16u64.millis()).await;
        }
    }

    /// Interrupt triggered whenever a button is pressed.
    /// 
    /// This can mean incrementing a value on screen or showing an alarm, etc.
    #[task(
        shared = [alarm_time, rtc, state],
        local = [power_switch, forward_button, back_button, increment_button, decrement_button],
        priority = 2,
        binds = IO_IRQ_BANK0
    )]
    fn gpio_interrupt(ctx: gpio_interrupt::Context) {
        (
            ctx.shared.alarm_time,
            ctx.shared.rtc,
            ctx.shared.state,
        ).lock(|alarm_time, rtc, state| {
            if let Some(time) = alarm_time {
                rtc.clear_interrupt();
                rtc.schedule_alarm(DateTimeFilter {
                    year: None,
                    month: None,
                    day: None,
                    day_of_week: None,
                    hour: Some(time.0),
                    minute: Some(time.1),
                    second: Some(time.2),
                });
                rtc.enable_interrupt();
                *state = State::Alarm;
            }
        });
    }

    /// Interrupt triggered when the alarm is triggered
    #[task(
        shared = [rtc, state],
        priority = 2,
        binds = RTC_IRQ
    )]
    fn alarm_interrupt(ctx: alarm_interrupt::Context) {
        (
            ctx.shared.rtc,
            ctx.shared.state,
        ).lock(|rtc, state| {
            rtc.clear_interrupt();
            rtc.disable_interrupt();
            rtc.disable_alarm();
            *state = State::AlarmTriggered;
        });
    }
}
