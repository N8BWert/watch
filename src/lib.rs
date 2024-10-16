//!
//! Watch is my dumb-watch project that I'm using to simply keep
//! track of time
//!

#![no_std]

extern crate alloc;

use alloc::{format, string::String};
use rp_pico::hal::rtc::{DateTime, DayOfWeek};

pub mod peripherals;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    /// The watch is in sleep state so it shouldn't show anything
    Sleep,
    /// Displaying the current clock time + calendar
    Time,
    /// Displaying the number of minutes and seconds to set an
    /// alarm for
    SettingAlarm,
    /// Displaying the alarm screen
    Alarm,
    /// Displaying the alarm has triggered
    AlarmTriggered,
}

impl Default for State {
    fn default() -> Self {
        Self::Time
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RealtimeDatetime {
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub day_of_week: DayOfWeek,
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
}

impl Default for RealtimeDatetime {
    fn default() -> Self {
        Self::new()
    }
}

impl From<DateTime> for RealtimeDatetime {
    fn from(value: DateTime) -> Self {
        Self {
            year: value.year,
            month: value.month,
            day: value.day,
            day_of_week: value.day_of_week,
            hour: value.hour,
            minute: value.minute,
            second: value.second,
        }
    }
}

impl RealtimeDatetime {
    pub const fn new() -> Self {
        Self {
            year: 2024,
            month: 10,
            day: 13,
            day_of_week: DayOfWeek::Sunday,
            hour: 14,
            minute: 1,
            second: 0,
        }
    }

    pub fn to_datetime(&self) -> DateTime {
        DateTime {
            year: self.year,
            month: self.month,
            day: self.day,
            day_of_week: self.day_of_week,
            hour: self.hour,
            minute: self.minute,
            second: self.second,
        }
    }

    pub fn date(&self) -> String {
        format!(
            "{} {} {}",
            match self.day_of_week {
                DayOfWeek::Monday => "M",
                DayOfWeek::Tuesday => "T",
                DayOfWeek::Wednesday => "W",
                DayOfWeek::Thursday => "R",
                DayOfWeek::Friday => "F",
                DayOfWeek::Saturday => "Sa",
                DayOfWeek::Sunday => "Su",
            },
            self.day,
            match self.month {
                1 => "Jan",
                2 => "Feb",
                3 => "Mar",
                4 => "Apr",
                5 => "May",
                6 => "June",
                7 => "July",
                8 => "Aug",
                9 => "Sep",
                10 => "Oct",
                11 => "Nov",
                _ => "Dec",
            }
        )
    }

    pub fn time(&self) -> String {
        format!("{}:{}:{}", self.hour, self.minute, self.second,)
    }
}
