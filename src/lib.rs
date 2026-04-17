#![no_std]

mod uninitialized;
pub use uninitialized::*;

mod asleep;
pub use asleep::*;
mod ready;
pub use ready::*;

use dw3000_ng::configs::{
    BitRate, PhrMode, PhrRate, PreambleLength, PulseRepetitionFrequency, SfdSequence, UwbChannel,
};
use embedded_hal::digital::OutputPin;
use embedded_hal_async::{digital::Wait, spi::SpiDevice};
use smoltcp::wire::ieee802154::{Address, Pan};

// The dw3000_ng crate has both a high and low level API for interacting with the DW3000.
// Unfortunately the high level API is buggy, not particularly well designed, and is lacking many
// essential features such as encryption or deep sleep. So we use the low level API.
use dw3000_ng::ll;

#[derive(Clone)]
pub struct Config {
    /// Enable diagnostic features such as signal power estimation. This is disabled by default as
    /// it reduces performance and increases power consumption. Generally diagnostics should be
    /// enabled on anchors, but not battery operated devices.
    pub enable_diagnostics: bool,

    pub pan: Pan,
    pub channel: UwbChannel,
    pub pulse_repetition_frequency: PulseRepetitionFrequency,
    pub preamble_length: PreambleLength,
    pub phr_mode: PhrMode,
    pub phr_rate: PhrRate,
    pub bit_rate: BitRate,
    pub sfd_timeout: u16,
    pub tx_preamble_code: u8,
    pub rx_preamble_code: u8,
    pub sfd_sequence: SfdSequence,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            enable_diagnostics: false,
            pan: Pan(12345),
            channel: UwbChannel::Channel9,
            pulse_repetition_frequency: PulseRepetitionFrequency::Mhz64,
            preamble_length: PreambleLength::Symbols64,
            phr_mode: PhrMode::Standard,
            phr_rate: PhrRate::Standard,
            bit_rate: BitRate::Kbps6800,
            sfd_timeout: 129,
            tx_preamble_code: 9,
            rx_preamble_code: 9,
            sfd_sequence: SfdSequence::Ieee,
        }
    }
}

pub struct Device<SPI, O, IRQ, State> {
    address: [u8; 8],
    config: Config,
    dev: ll::DW3000<SPI>,
    irq: IRQ,
    rst: O,
    state: State,
}

impl<SPI, O, IRQ, State> Device<SPI, O, IRQ, State>
where
    SPI: SpiDevice,
    O: OutputPin,
    IRQ: Wait,
{
    /// Fully resets the device.
    pub async fn reset(self) -> Result<SPI, Device<SPI, O, IRQ, ReadyState>> {
        let config = self.config.clone();
        self.with_state(UninitializedState).init(config).await
    }
}

impl<SPI, O, IRQ, State> Device<SPI, O, IRQ, State> {
    /// Returns the device's IEEE 802.15.4 address.
    pub fn address(&self) -> Address {
        Address::Extended(self.address)
    }

    pub(crate) fn with_state<NewState>(self, new_state: NewState) -> Device<SPI, O, IRQ, NewState> {
        Device {
            address: self.address,
            config: self.config,
            dev: self.dev,
            irq: self.irq,
            rst: self.rst,
            state: new_state,
        }
    }
}

#[derive(thiserror::Error, defmt::Format, Debug)]
pub enum Error<DE> {
    Device(#[from] DE),
    Unknown,
    InvalidConfiguration,
    MalformedFrame,
    PgfCalibrationFailed,
    UnsupportedDevice,
    Fcs,
    Phy,
    ReedSolomon,
    SfdTimeout,
    Overrun,
    PreambleDetectionTimeout,
    PayloadTooLarge,
}

pub type Result<SPI, T> = core::result::Result<T, Error<ll::Error<SPI>>>;
