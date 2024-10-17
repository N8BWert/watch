//!
//! List of the peripherals in use by the watch project
//!

use rp_pico::{
    hal::{
        gpio::{
            bank0::{Gpio0, Gpio10, Gpio11, Gpio12, Gpio13, Gpio14, Gpio15, Gpio18, Gpio19}, FunctionI2C, FunctionSio, Pin, PullDown, PullUp, SioInput
        },
        I2C,
    },
    pac::I2C1,
};
use ssd1306::{mode::BufferedGraphicsModeAsync, prelude::*, Ssd1306Async};

/// The watch display
pub type Display = Ssd1306Async<
    I2CInterface<
        I2C<
            I2C1,
            (
                Pin<Gpio18, FunctionI2C, PullUp>,
                Pin<Gpio19, FunctionI2C, PullUp>,
            ),
        >,
    >,
    DisplaySize128x64,
    BufferedGraphicsModeAsync<DisplaySize128x64>,
>;

/// The pin the power switch is connected to.  If the power switch
/// is low, the device should enter sleep only to be interrupted when
/// the switch is high
pub type PowerSwitch = Pin<Gpio0, FunctionSio<SioInput>, PullDown>;

/// The pin set to move forward in the current selection or interface
pub type ForwardButton = Pin<Gpio10, FunctionSio<SioInput>, PullDown>;
/// The pin set to move backward in the current selection or interface
pub type BackButton = Pin<Gpio11, FunctionSio<SioInput>, PullDown>;
/// The pin set to increment the current selection
pub type IncrementButton = Pin<Gpio12, FunctionSio<SioInput>, PullDown>;
/// The pin set to decrement the current selection
pub type DecrementButton = Pin<Gpio13, FunctionSio<SioInput>, PullDown>;
/// The pin set to enter edit mode for the current selection
pub type EditButton = Pin<Gpio14, FunctionSio<SioInput>, PullDown>;
/// The pin to start the alarm
pub type AlarmButton = Pin<Gpio15, FunctionSio<SioInput>, PullDown>;