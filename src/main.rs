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

/// Convert a measured adc value to a voltage
pub fn adc_to_voltage(adc_value: u16) -> f32 {
    3.3 * (adc_value as f32) / 65535.0
}

/// Convert a measured adc value to a temperature measurement (C)
pub fn adc_to_temperature(adc_value: u16) -> f32 {
    let voltage = adc_to_voltage(adc_value);
    27.0 - (voltage - 0.706) / 0.001721
}

#[rtic::app(
    device = rp_pico::hal::pac,
    peripherals = true,
    dispatchers = [SW0_IRQ, SW1_IRQ]
)]
mod app {
    use super::*;
    use core::mem::MaybeUninit;

    use bsp::hal::{
        adc::{Adc, AdcPin},
        clocks::init_clocks_and_plls,
        gpio::Pins,
        sio::Sio,
        watchdog::Watchdog,
    };
    use embedded_hal::digital::OutputPin;
    use embedded_hal_02::adc::OneShot;
    use fugit::RateExtU32;
    use rp_pico::{
        self as bsp,
        hal::{
            adc::TempSense,
            gpio::Interrupt,
            rtc::{DateTimeFilter, RealTimeClock},
            I2C,
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

    use watch::{
        peripherals::{
            AlarmButton, BackButton, BatteryReader, DecrementButton, Display, EditButton,
            ForwardButton, IncrementButton, Pin23, PowerSwitch, VibrationMotor,
        },
        AlarmSelection, DatetimeSelection, RealtimeDatetime, State,
    };

    const EXTERNAL_XTAL_FREQ_HZ: u32 = 12_000_000u32;

    const HEAP_SIZE: usize = 4096;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    /// the amount of time to wait to debounce peripherals
    const DEBOUNCE_TIME_MS: u64 = 10;

    rp2040_timer_monotonic!(Mono);

    /// The default datetime the watch will use if it is wiped or newly flashed
    static mut REFERENCE_DATETIME: RealtimeDatetime = RealtimeDatetime::new();
    /// The default alarm length the watch will use if it is wiped or newly flashed
    static mut REFERENCE_ALARM: (u8, u8, u8) = (0, 1, 0);

    #[shared]
    struct Shared {
        // The OLED Display
        display: Option<Display>,
        // The real-time clock
        rtc: RealTimeClock,
        // The current state of the watch
        state: State,
        // The most recently stored value for the alarm
        alarm_time: (u8, u8, u8),
        // The currently selected value in the alarm time to edit
        alarm_selection: Option<AlarmSelection>,
        // The most recently stored value for the reference clock
        reference_time: RealtimeDatetime,
        // The time the most recent alarm was started
        alarm_start: Option<RealtimeDatetime>,
        // The currently selected value in the reference time to edit
        reference_selection: Option<DatetimeSelection>,
        // The current temperature measurement
        temperature: f32,
        // The current battery level
        battery_voltage: f32,

        // The switch used to turn on and off the watch
        power_switch: PowerSwitch,
        // The forward button to toggle through the states of the watch
        forward_button: ForwardButton,
        // The back button to toggle through the states of the watch
        back_button: BackButton,
        // The button to increment the currently selected value when editing
        // timer values
        increment_button: IncrementButton,
        // The button to decrement the currently selected value when editing
        // timer values
        decrement_button: DecrementButton,
        // The button to toggle into and out of edit mode for the current state
        edit_button: EditButton,
        // The button to start the alarm
        alarm_button: AlarmButton,
    }

    #[local]
    struct Local {
        // The vibration motor that is triggered when an alarm goes off
        vibration_motor: VibrationMotor,
        // Pin 23 used to put the pico into the correct power mode for ADC measurements
        pin23: Pin23,
        // The ADC
        adc: Adc,
        // The temperature sensor
        temperature_sensor: TempSense,
        // The battery voltage reader
        battery_reader: BatteryReader,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        #[allow(static_mut_refs)]
        unsafe {
            HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE);
        }

        Mono::start(ctx.device.TIMER, &ctx.device.RESETS);

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
            #[allow(static_mut_refs)]
            unsafe {
                REFERENCE_DATETIME.to_datetime()
            },
        )
        .unwrap();

        let mut adc = Adc::new(ctx.device.ADC, &mut ctx.device.RESETS);
        let mut pin23 = pins.gpio23.into_push_pull_output();
        let mut temperature_sensor = adc.take_temp_sensor().unwrap();
        let mut battery_reader = AdcPin::new(pins.gpio26.into_floating_input()).unwrap();

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

        let edit_button = pins.gpio14.into_pull_down_input();
        edit_button.set_interrupt_enabled(Interrupt::EdgeHigh, true);

        let alarm_button = pins.gpio15.into_pull_down_input();
        alarm_button.set_interrupt_enabled(Interrupt::EdgeHigh, true);

        let vibration_motor = pins.gpio16.into_push_pull_output();

        // Take the initial measurements
        pin23.set_high().unwrap();
        let temperature = adc.read(&mut temperature_sensor).unwrap();
        let battery_level = adc.read(&mut battery_reader).unwrap();
        pin23.set_low().unwrap();

        initialize_display::spawn().ok();

        (
            Shared {
                display: Some(display),
                rtc,
                state: State::default(),
                alarm_time: unsafe { REFERENCE_ALARM },
                alarm_selection: None,
                reference_time: unsafe { REFERENCE_DATETIME },
                reference_selection: None,
                alarm_start: None,
                temperature: adc_to_temperature(temperature),
                battery_voltage: adc_to_voltage(battery_level),
                forward_button,
                back_button,
                increment_button,
                decrement_button,
                power_switch,
                edit_button,
                alarm_button,
            },
            Local {
                vibration_motor,
                pin23,
                adc,
                temperature_sensor,
                battery_reader,
            },
        )
    }

    /// Idle loop waiting for interrupts
    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
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
        shared = [
            rtc,
            display,
            state,
            alarm_time,
            alarm_selection,
            alarm_start,
            reference_time,
            reference_selection,
            temperature,
            battery_voltage,
        ],
        local = [
            vibration_motor
        ],
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

            let state = ctx.shared.state.lock(|state| *state);
            match state {
                State::Sleep => display.clear(BinaryColor::Off).unwrap(),
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
                }
                State::EditTime => {
                    let selection = ctx
                        .shared
                        .reference_selection
                        .lock(|selection| selection.unwrap_or(DatetimeSelection::Year));
                    let reference_time = ctx.shared.reference_time.lock(|time| *time);
                    if selection.date() {
                        // Display Date
                        Text::with_baseline(
                            reference_time.date().as_str(),
                            Point::zero(),
                            text_style,
                            Baseline::Top,
                        )
                        .draw(&mut display)
                        .unwrap();

                        // Display Selector
                        match selection {
                            DatetimeSelection::Year => {
                                Text::with_baseline(
                                    "          ^^^^",
                                    Point::new(0, 16),
                                    text_style,
                                    Baseline::Top,
                                )
                                .draw(&mut display)
                                .unwrap();
                            }
                            DatetimeSelection::Month => {
                                Text::with_baseline(
                                    "      ^^^     ",
                                    Point::new(0, 16),
                                    text_style,
                                    Baseline::Top,
                                )
                                .draw(&mut display)
                                .unwrap();
                            }
                            DatetimeSelection::Day => {
                                Text::with_baseline(
                                    "   ^^^        ",
                                    Point::new(0, 16),
                                    text_style,
                                    Baseline::Top,
                                )
                                .draw(&mut display)
                                .unwrap();
                            }
                            _ => {
                                Text::with_baseline(
                                    "^^            ",
                                    Point::new(0, 16),
                                    text_style,
                                    Baseline::Top,
                                )
                                .draw(&mut display)
                                .unwrap();
                            }
                        }
                    } else {
                        // Display Time
                        Text::with_baseline(
                            reference_time.time().as_str(),
                            Point::zero(),
                            text_style,
                            Baseline::Top,
                        )
                        .draw(&mut display)
                        .unwrap();

                        // Display Selector
                        match selection {
                            DatetimeSelection::Hour => {
                                Text::with_baseline(
                                    "^^      ",
                                    Point::new(0, 16),
                                    text_style,
                                    Baseline::Top,
                                )
                                .draw(&mut display)
                                .unwrap();
                            }
                            DatetimeSelection::Minute => {
                                Text::with_baseline(
                                    "   ^^   ",
                                    Point::new(0, 16),
                                    text_style,
                                    Baseline::Top,
                                )
                                .draw(&mut display)
                                .unwrap();
                            }
                            _ => {
                                Text::with_baseline(
                                    "      ^^",
                                    Point::new(0, 16),
                                    text_style,
                                    Baseline::Top,
                                )
                                .draw(&mut display)
                                .unwrap();
                            }
                        }
                    }
                }
                State::SettingAlarm => {
                    let alarm_time = ctx.shared.alarm_time.lock(|alarm_time| *alarm_time);
                    let alarm_selection = ctx
                        .shared
                        .alarm_selection
                        .lock(|selection| selection.unwrap_or(AlarmSelection::Hour));
                    // Display Alarm
                    Text::with_baseline(
                        format!(
                            "{:02}:{:02}:{:02}",
                            alarm_time.0, alarm_time.1, alarm_time.2,
                        )
                        .as_str(),
                        Point::zero(),
                        text_style,
                        Baseline::Top,
                    )
                    .draw(&mut display)
                    .unwrap();

                    // Display Alarm Selection
                    match alarm_selection {
                        AlarmSelection::Hour => {
                            Text::with_baseline(
                                "^^      ",
                                Point::new(0, 16),
                                text_style,
                                Baseline::Top,
                            )
                            .draw(&mut display)
                            .unwrap();
                        }
                        AlarmSelection::Minute => {
                            Text::with_baseline(
                                "   ^^   ",
                                Point::new(0, 16),
                                text_style,
                                Baseline::Top,
                            )
                            .draw(&mut display)
                            .unwrap();
                        }
                        _ => {
                            Text::with_baseline(
                                "      ^^",
                                Point::new(0, 16),
                                text_style,
                                Baseline::Top,
                            )
                            .draw(&mut display)
                            .unwrap();
                        }
                    }
                }
                State::Alarm => {
                    // Display time remaining in the alarm
                    let remaining_time = ctx
                        .shared
                        .alarm_start
                        .lock(|start| current_time - start.unwrap_or(current_time));
                    Text::with_baseline(
                        format!(
                            "{:02}:{:02}:{:02}",
                            remaining_time.0, remaining_time.1, remaining_time.2
                        )
                        .as_str(),
                        Point::zero(),
                        text_style,
                        Baseline::Middle,
                    )
                    .draw(&mut display)
                    .unwrap();
                }
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
                }
            }

            if state == State::AlarmTriggered {
                ctx.local.vibration_motor.set_high().unwrap();
            } else {
                ctx.local.vibration_motor.set_low().unwrap();
            }

            if state != State::Sleep {
                Text::with_baseline(
                    format!(
                        "{}C         {}V",
                        ctx.shared.temperature.lock(|temp| *temp),
                        ctx.shared.battery_voltage.lock(|batt| *batt),
                    )
                    .as_str(),
                    Point::zero(),
                    text_style,
                    Baseline::Bottom,
                )
                .draw(&mut display)
                .unwrap();
            }

            display.flush().await.unwrap();

            Mono::delay(16u64.millis()).await;
        }
    }

    /// Interrupt triggered whenever a button is pressed.
    ///
    /// This can mean incrementing a value on screen or showing an alarm, etc.
    #[task(
        shared = [
            alarm_time,
            alarm_selection,
            reference_time,
            reference_selection,
            alarm_start,
            rtc,
            state,
            power_switch,
            forward_button,
            back_button,
            increment_button,
            decrement_button,
            edit_button,
            alarm_button,
        ],
        priority = 2,
        binds = IO_IRQ_BANK0
    )]
    fn gpio_interrupt(ctx: gpio_interrupt::Context) {
        (
            ctx.shared.alarm_time,
            ctx.shared.alarm_selection,
            ctx.shared.reference_time,
            ctx.shared.reference_selection,
            ctx.shared.alarm_start,
            ctx.shared.rtc,
            ctx.shared.state,
            ctx.shared.power_switch,
            ctx.shared.forward_button,
            ctx.shared.back_button,
            ctx.shared.increment_button,
            ctx.shared.decrement_button,
            ctx.shared.edit_button,
            ctx.shared.alarm_button,
        )
            .lock(
                |alarm_time,
                 alarm_selection,
                 reference_time,
                 reference_selection,
                 alarm_start,
                 rtc,
                 state,
                 power_switch,
                 forward_button,
                 back_button,
                 increment_button,
                 decrement_button,
                 edit_button,
                 alarm_button| {
                    // Power state transitions
                    if power_switch.interrupt_status(Interrupt::EdgeHigh) {
                        disable_button_interrupts::spawn().unwrap();
                        *state = State::Time;
                        power_switch.clear_interrupt(Interrupt::EdgeHigh);
                        power_switch.set_interrupt_enabled(Interrupt::EdgeHigh, false);
                        power_switch.set_interrupt_enabled(Interrupt::EdgeLow, false);
                        power_switch_cooldown::spawn().unwrap();
                        return;
                    } else if power_switch.interrupt_status(Interrupt::EdgeLow) {
                        enable_button_interrupts::spawn().unwrap();
                        *state = State::Sleep;
                        power_switch.clear_interrupt(Interrupt::EdgeLow);
                        power_switch.set_interrupt_enabled(Interrupt::EdgeHigh, false);
                        power_switch.set_interrupt_enabled(Interrupt::EdgeLow, false);
                        power_switch_cooldown::spawn().unwrap();
                        return;
                    }

                    // Was the forward button pressed
                    if forward_button.interrupt_status(Interrupt::EdgeHigh) {
                        match *state {
                            State::Time => *state = State::Alarm,
                            State::EditTime => {
                                if let Some(selection) = *reference_selection {
                                    *reference_selection = Some(selection.next());
                                } else {
                                    *reference_selection = Some(DatetimeSelection::Year);
                                }
                            }
                            State::Alarm => *state = State::Time,
                            State::SettingAlarm => {
                                if let Some(selection) = *alarm_selection {
                                    *alarm_selection = Some(selection.next());
                                } else {
                                    *alarm_selection = Some(AlarmSelection::Hour);
                                }
                            }
                            State::AlarmTriggered => *state = State::Time,
                            _ => (),
                        }
                        forward_button.clear_interrupt(Interrupt::EdgeHigh);
                        forward_button.set_interrupt_enabled(Interrupt::EdgeHigh, false);
                        debounce_forward::spawn().unwrap();
                        return;
                    }

                    // Was the back button pressed
                    if back_button.interrupt_status(Interrupt::EdgeHigh) {
                        match *state {
                            State::Time => *state = State::Alarm,
                            State::EditTime => {
                                if let Some(selection) = *reference_selection {
                                    *reference_selection = Some(selection.last());
                                } else {
                                    *reference_selection = Some(DatetimeSelection::Year);
                                }
                            }
                            State::Alarm => *state = State::Time,
                            State::SettingAlarm => {
                                if let Some(selection) = *alarm_selection {
                                    *alarm_selection = Some(selection.last());
                                } else {
                                    *alarm_selection = Some(AlarmSelection::Hour);
                                }
                            }
                            State::AlarmTriggered => *state = State::Alarm,
                            _ => (),
                        }
                        back_button.clear_interrupt(Interrupt::EdgeHigh);
                        back_button.set_interrupt_enabled(Interrupt::EdgeHigh, false);
                        debounce_back::spawn().unwrap();
                        return;
                    }

                    // Was the edit button pressed
                    if edit_button.interrupt_status(Interrupt::EdgeHigh) {
                        match *state {
                            State::Time => *state = State::EditTime,
                            State::EditTime => {
                                // Update the current time
                                *state = State::Time;
                                rtc.clear_interrupt();
                                rtc.disable_alarm();
                                rtc.disable_interrupt();
                                rtc.set_datetime(reference_time.to_datetime()).unwrap();
                                unsafe { REFERENCE_DATETIME = *reference_time };
                            }
                            State::Alarm => *state = State::SettingAlarm,
                            State::SettingAlarm => {
                                *state = State::Alarm;
                                rtc.clear_interrupt();
                                rtc.schedule_alarm(DateTimeFilter {
                                    year: None,
                                    month: None,
                                    day: None,
                                    day_of_week: None,
                                    hour: Some(alarm_time.0),
                                    minute: Some(alarm_time.1),
                                    second: Some(alarm_time.2),
                                });
                                rtc.enable_interrupt();
                            }
                            State::AlarmTriggered => *state = State::SettingAlarm,
                            _ => (),
                        }
                        edit_button.clear_interrupt(Interrupt::EdgeHigh);
                        edit_button.set_interrupt_enabled(Interrupt::EdgeHigh, false);
                        debounce_edit::spawn().unwrap();
                        return;
                    }

                    // Was the increment button pressed
                    if increment_button.interrupt_status(Interrupt::EdgeHigh) {
                        match *state {
                            State::EditTime => {
                                let selection =
                                    reference_selection.unwrap_or(DatetimeSelection::Year);
                                reference_time.increment(selection);
                            }
                            State::SettingAlarm => {
                                let selection = alarm_selection.unwrap_or(AlarmSelection::Hour);
                                match selection {
                                    AlarmSelection::Hour => alarm_time.0 = (alarm_time.0 + 1) % 24,
                                    AlarmSelection::Minute => {
                                        alarm_time.1 = (alarm_time.1 + 1) % 60
                                    }
                                    AlarmSelection::Second => {
                                        alarm_time.2 = (alarm_time.2 + 1) % 60
                                    }
                                }
                            }
                            _ => (),
                        }
                        increment_button.clear_interrupt(Interrupt::EdgeHigh);
                        increment_button.set_interrupt_enabled(Interrupt::EdgeHigh, false);
                        debounce_increment::spawn().unwrap();
                        return;
                    }

                    // Was the decrement button pressed
                    if decrement_button.interrupt_status(Interrupt::EdgeHigh) {
                        match *state {
                            State::EditTime => {
                                let selection =
                                    reference_selection.unwrap_or(DatetimeSelection::Year);
                                reference_time.decrement(selection);
                            }
                            State::SettingAlarm => {
                                let selection = alarm_selection.unwrap_or(AlarmSelection::Hour);
                                match selection {
                                    AlarmSelection::Hour => {
                                        alarm_time.0 = alarm_time.0.checked_sub(1).unwrap_or(23)
                                    }
                                    AlarmSelection::Minute => {
                                        alarm_time.1 = alarm_time.1.checked_sub(1).unwrap_or(59)
                                    }
                                    AlarmSelection::Second => {
                                        alarm_time.2 = alarm_time.2.checked_sub(1).unwrap_or(59)
                                    }
                                }
                            }
                            _ => (),
                        }
                        decrement_button.clear_interrupt(Interrupt::EdgeHigh);
                        decrement_button.set_interrupt_enabled(Interrupt::EdgeHigh, false);
                        debounce_decrement::spawn().unwrap();
                        return;
                    }

                    // Was the alarm button pressed
                    if alarm_button.interrupt_status(Interrupt::EdgeHigh) {
                        *state = State::Alarm;
                        *alarm_start = Some(rtc.now().unwrap().into());
                        rtc.clear_interrupt();
                        rtc.schedule_alarm(DateTimeFilter {
                            year: None,
                            month: None,
                            day: None,
                            day_of_week: None,
                            hour: Some(alarm_time.0),
                            minute: Some(alarm_time.1),
                            second: Some(alarm_time.2),
                        });
                        rtc.enable_interrupt();
                        alarm_button.clear_interrupt(Interrupt::EdgeHigh);
                        alarm_button.set_interrupt_enabled(Interrupt::EdgeHigh, false);
                        debounce_alarm::spawn().unwrap();
                    }
                },
            );
    }

    /// Read the temperature and battery voltage every once in a while if we aren't sleeping
    #[task(
        shared = [
            state,
            temperature,
            battery_voltage,
        ],
        local = [
            pin23,
            adc,
            temperature_sensor,
            battery_reader,
        ],
        priority = 1
    )]
    async fn take_adc_measurements(mut ctx: take_adc_measurements::Context) {
        if ctx.shared.state.lock(|state| *state != State::Sleep) {
            ctx.local.pin23.set_high().unwrap();
            let temperature = ctx.local.adc.read(ctx.local.temperature_sensor).unwrap();
            let battery_level = ctx.local.adc.read(ctx.local.battery_reader).unwrap();
            ctx.local.pin23.set_low().unwrap();

            (ctx.shared.temperature, ctx.shared.battery_voltage).lock(|temp, batt_level| {
                *temp = adc_to_temperature(temperature);
                *batt_level = adc_to_voltage(battery_level);
            });
        }
    }

    /// Interrupt triggered when the alarm is triggered
    #[task(
        shared = [rtc, state],
        priority = 2,
        binds = RTC_IRQ
    )]
    fn alarm_interrupt(ctx: alarm_interrupt::Context) {
        (ctx.shared.rtc, ctx.shared.state).lock(|rtc, state| {
            rtc.clear_interrupt();
            rtc.disable_interrupt();
            rtc.disable_alarm();
            if *state == State::Sleep {
                enable_button_interrupts::spawn().unwrap();
            }
            *state = State::AlarmTriggered;
        });
    }

    /// Disable all button interrupts
    #[task(
        shared = [
            forward_button,
            back_button,
            increment_button,
            decrement_button,
            edit_button,
            alarm_button,
        ],
        priority = 2,
    )]
    async fn disable_button_interrupts(ctx: disable_button_interrupts::Context) {
        (
            ctx.shared.forward_button,
            ctx.shared.back_button,
            ctx.shared.increment_button,
            ctx.shared.decrement_button,
            ctx.shared.edit_button,
            ctx.shared.alarm_button,
        )
            .lock(
                |forward_button,
                 back_button,
                 increment_button,
                 decrement_button,
                 edit_button,
                 alarm_button| {
                    forward_button.set_interrupt_enabled(Interrupt::EdgeHigh, false);
                    back_button.set_interrupt_enabled(Interrupt::EdgeHigh, false);
                    increment_button.set_interrupt_enabled(Interrupt::EdgeHigh, false);
                    decrement_button.set_interrupt_enabled(Interrupt::EdgeHigh, false);
                    edit_button.set_interrupt_enabled(Interrupt::EdgeHigh, false);
                    alarm_button.set_interrupt_enabled(Interrupt::EdgeHigh, false);
                },
            );
    }

    /// Enable all button interrupts
    #[task(
        shared = [
            forward_button,
            back_button,
            increment_button,
            decrement_button,
            edit_button,
            alarm_button,
        ],
        priority = 2
    )]
    async fn enable_button_interrupts(ctx: enable_button_interrupts::Context) {
        (
            ctx.shared.forward_button,
            ctx.shared.back_button,
            ctx.shared.increment_button,
            ctx.shared.decrement_button,
            ctx.shared.edit_button,
            ctx.shared.alarm_button,
        )
            .lock(
                |forward_button,
                 back_button,
                 increment_button,
                 decrement_button,
                 edit_button,
                 alarm_button| {
                    forward_button.set_interrupt_enabled(Interrupt::EdgeHigh, true);
                    back_button.set_interrupt_enabled(Interrupt::EdgeHigh, true);
                    increment_button.set_interrupt_enabled(Interrupt::EdgeHigh, true);
                    decrement_button.set_interrupt_enabled(Interrupt::EdgeHigh, true);
                    edit_button.set_interrupt_enabled(Interrupt::EdgeHigh, true);
                    alarm_button.set_interrupt_enabled(Interrupt::EdgeHigh, true);
                },
            );
    }

    /// Software task to debounce the power switch
    #[task(
        shared = [power_switch],
        priority = 1
    )]
    async fn power_switch_cooldown(mut ctx: power_switch_cooldown::Context) {
        Mono::delay(DEBOUNCE_TIME_MS.millis()).await;
        ctx.shared.power_switch.lock(|power_switch| {
            power_switch.set_interrupt_enabled(Interrupt::EdgeHigh, true);
            power_switch.set_interrupt_enabled(Interrupt::EdgeLow, true);
        });
    }

    /// Software task to debounce the forward button
    #[task(
        shared = [forward_button],
        priority = 1
    )]
    async fn debounce_forward(mut ctx: debounce_forward::Context) {
        Mono::delay(DEBOUNCE_TIME_MS.millis()).await;
        ctx.shared.forward_button.lock(|forward_button| {
            forward_button.set_interrupt_enabled(Interrupt::EdgeHigh, true);
        });
    }

    /// Software task to debounce the back button
    #[task(
        shared = [back_button],
        priority = 1
    )]
    async fn debounce_back(mut ctx: debounce_back::Context) {
        Mono::delay(DEBOUNCE_TIME_MS.millis()).await;
        ctx.shared.back_button.lock(|back_button| {
            back_button.set_interrupt_enabled(Interrupt::EdgeHigh, true);
        });
    }

    /// Software task to debounce the increment button
    #[task(
        shared = [increment_button],
        priority = 1
    )]
    async fn debounce_increment(mut ctx: debounce_increment::Context) {
        Mono::delay(DEBOUNCE_TIME_MS.millis()).await;
        ctx.shared.increment_button.lock(|increment_button| {
            increment_button.set_interrupt_enabled(Interrupt::EdgeHigh, true);
        });
    }

    /// Software task to debounce the decrement button
    #[task(
        shared = [decrement_button],
        priority = 1
    )]
    async fn debounce_decrement(mut ctx: debounce_decrement::Context) {
        Mono::delay(DEBOUNCE_TIME_MS.millis()).await;
        ctx.shared.decrement_button.lock(|decrement_button| {
            decrement_button.set_interrupt_enabled(Interrupt::EdgeHigh, true);
        });
    }

    /// Software task to debounce the edit button
    #[task(
        shared = [edit_button],
        priority = 1
    )]
    async fn debounce_edit(mut ctx: debounce_edit::Context) {
        Mono::delay(DEBOUNCE_TIME_MS.millis()).await;
        ctx.shared.edit_button.lock(|edit_button| {
            edit_button.set_interrupt_enabled(Interrupt::EdgeHigh, true);
        });
    }

    /// Software task to debounce the alarm button
    #[task(
        shared = [alarm_button],
        priority = 1
    )]
    async fn debounce_alarm(mut ctx: debounce_alarm::Context) {
        Mono::delay(DEBOUNCE_TIME_MS.millis()).await;
        ctx.shared.alarm_button.lock(|alarm_button| {
            alarm_button.set_interrupt_enabled(Interrupt::EdgeHigh, true);
        });
    }
}
