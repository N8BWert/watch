//!
//! Watch is my dumb-watch project that I'm using to simply keep
//! track of time
//!

#![no_std]

extern crate alloc;

use core::ops::Sub;

use alloc::{format, string::String};

use rp_pico::hal::rtc::{DateTime, DayOfWeek};

pub mod peripherals;

/// The current state of the watch
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    /// The watch is in sleep state so it shouldn't show anything
    Sleep,
    /// Displaying the current clock time + calendar
    Time,
    /// Modifying the current clock time + calendar
    EditTime,
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

/// The currently selected value in the datetime to edit
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DatetimeSelection {
    // The current year
    Year,
    // The current month
    Month,
    // The current day
    Day,
    // The current day of the week
    DayOfWeek,
    // The current hour
    Hour,
    // The current minute
    Minute,
    // The current second
    Second,
}

impl DatetimeSelection {
    /// Switch to the next datetime selection
    pub fn next(self) -> Self {
        match self {
            Self::Year => Self::Month,
            Self::Month => Self::Day,
            Self::Day => Self::DayOfWeek,
            Self::DayOfWeek => Self::Hour,
            Self::Hour => Self::Minute,
            Self::Minute => Self::Second,
            Self::Second => Self::Year,
        }
    }

    /// Switch to the previous datetime selection
    pub fn last(self) -> Self {
        match self {
            Self::Year => Self::Second,
            Self::Month => Self::Year,
            Self::Day => Self::Month,
            Self::DayOfWeek => Self::Day,
            Self::Hour => Self::DayOfWeek,
            Self::Minute => Self::Hour,
            Self::Second => Self::Minute,
        }
    }

    /// True if the selection is a date
    pub fn date(&self) -> bool {
        self.eq(&Self::Year) || self.eq(&Self::Month) || self.eq(&Self::Day) || self.eq(&Self::DayOfWeek)
    }

    /// True if the selection is a time
    pub fn time(&self) -> bool {
        self.eq(&Self::Hour) || self.eq(&Self::Minute) || self.eq(&Self::Second)
    }
}

/// The currently selected value in the alarm to edit
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AlarmSelection {
    Hour,
    Minute,
    Second,
}

impl AlarmSelection {
    /// Switch to the next alarm selection
    pub fn next(self) -> Self {
        match self {
            Self::Hour => Self::Minute,
            Self::Minute => Self::Second,
            Self::Second => Self::Hour,
        }
    }

    /// Switch to the last alarm selection
    pub fn last(self) -> Self {
        match self {
            Self::Hour => Self::Second,
            Self::Minute => Self::Hour,
            Self::Second => Self::Minute,
        }
    }
}

/// An local wrapper for the rp2040 DateTime that allows me to
/// implement various methods
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
    /// Create a new default realtime datetime
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

    /// Convert the days, hours, minutes, and seconds of the datetime
    /// into seconds
    pub fn to_seconds(&self) -> u128 {
        (self.day as u128) * 24 * 60 * 60 +
            (self.hour as u128) * 60 * 60 +
            (self.minute as u128) * 60 +
            (self.second as u128)
    }

    /// Convert the RealtimeDatetime into an rp2040 DateTime
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

    /// Get a string representing the date of the RealtimeDatetime
    pub fn date(&self) -> String {
        format!(
            "{} {:02} {} {:04}",
            match self.day_of_week {
                DayOfWeek::Monday => "M ",
                DayOfWeek::Tuesday => "T ",
                DayOfWeek::Wednesday => "W ",
                DayOfWeek::Thursday => "R ",
                DayOfWeek::Friday => "F ",
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
                6 => "Jun",
                7 => "Jul",
                8 => "Aug",
                9 => "Sep",
                10 => "Oct",
                11 => "Nov",
                _ => "Dec",
            },
            self.year,
        )
    }

    /// Get the time represented by the RealtimeDatetime
    pub fn time(&self) -> String {
        format!("{:02}:{:02}:{:02}", self.hour, self.minute, self.second,)
    }

    /// Increment the specified selection for the RealtimeDatetime
    pub fn increment(&mut self, selection: DatetimeSelection) {
        match selection {
            DatetimeSelection::Year => self.year += 1,
            DatetimeSelection::Month => self.month = ((self.month + 1) % 12) + 1,
            DatetimeSelection::Day => self.day = ((self.day + 1) % 31) + 1,
            DatetimeSelection::DayOfWeek => {
                self.day_of_week = match self.day_of_week {
                    DayOfWeek::Monday => DayOfWeek::Tuesday,
                    DayOfWeek::Tuesday => DayOfWeek::Wednesday,
                    DayOfWeek::Wednesday => DayOfWeek::Thursday,
                    DayOfWeek::Thursday => DayOfWeek::Friday,
                    DayOfWeek::Friday => DayOfWeek::Saturday,
                    DayOfWeek::Saturday => DayOfWeek::Sunday,
                    DayOfWeek::Sunday => DayOfWeek::Monday,
                }
            },
            DatetimeSelection::Hour => self.hour = (self.hour + 1) % 24,
            DatetimeSelection::Minute => self.minute = (self.minute + 1) % 60,
            DatetimeSelection::Second => self.second = (self.second + 1) % 60,
        }
    }

    /// Decrease the specified selection for the RealtimeDatetime
    pub fn decrement(&mut self, selection: DatetimeSelection) {
        match selection {
            DatetimeSelection::Year => self.year = self.year.wrapping_sub(1),
            DatetimeSelection::Month => self.month = self.month.checked_sub(1).unwrap_or(12),
            DatetimeSelection::Day => self.day = self.day.checked_sub(1).unwrap_or(31),
            DatetimeSelection::DayOfWeek => {
                self.day_of_week = match self.day_of_week {
                    DayOfWeek::Monday => DayOfWeek::Sunday,
                    DayOfWeek::Tuesday => DayOfWeek::Monday,
                    DayOfWeek::Wednesday => DayOfWeek::Tuesday,
                    DayOfWeek::Thursday => DayOfWeek::Wednesday,
                    DayOfWeek::Friday => DayOfWeek::Thursday,
                    DayOfWeek::Saturday => DayOfWeek::Friday,
                    DayOfWeek::Sunday => DayOfWeek::Saturday,
                }
            },
            DatetimeSelection::Hour => self.hour = self.hour.checked_sub(1).unwrap_or(23),
            DatetimeSelection::Minute => self.minute = self.minute.checked_sub(1).unwrap_or(59),
            DatetimeSelection::Second => self.second = self.second.checked_sub(1).unwrap_or(59),
        }
    }
}

impl Sub for RealtimeDatetime {
    type Output = (u8, u8, u8);
    
    // TODO: Fix ths to work across day boundaries.  I think my current
    // implementation breaks down across date boundaries, but I'm not
    // planning on using this over those boundaries so I don't think it
    // is a big deal at the moment
    fn sub(self, rhs: Self) -> Self::Output {
        let seconds = self.to_seconds().saturating_sub(rhs.to_seconds());
        let hours = seconds / 60 / 60;
        let minutes = (seconds - hours * 60 * 60) / 60;
        let seconds = seconds % 60;
        (
            hours as u8,
            minutes as u8,
            seconds as u8,
        )
    }
}
