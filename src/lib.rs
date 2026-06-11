#![no_std]

mod uninitialized;
pub use uninitialized::*;

mod asleep;
pub use asleep::*;
mod receiving;
pub use receiving::*;
mod ready;
pub use ready::*;
mod timestamp;
pub use timestamp::*;

pub use dw3000_ng::configs::{
    BitRate, PhrMode, PhrRate, PreambleLength, PulseRepetitionFrequency, SfdSequence,
    UwbChannel as Channel,
};
pub use smoltcp::wire::ieee802154;

use embedded_hal::digital::OutputPin;
use embedded_hal_async::{delay::DelayNs, digital::Wait, spi::SpiDevice};
use futures::FutureExt;
use smoltcp::wire::ieee802154::{Address, Pan};

// The dw3000_ng crate has both a high and low level API for interacting with the DW3000.
// Unfortunately we've had issues with the high level API and found it to be lacking essential
// features such as deep sleep. So we use the low level API.
use dw3000_ng::ll;

#[non_exhaustive]
#[derive(Clone)]
pub struct Config {
    /// Enable diagnostic features such as signal power estimation. This is disabled by default as
    /// it reduces performance and increases power consumption. Generally diagnostics should be
    /// enabled on anchors, but not battery operated devices.
    pub enable_diagnostics: bool,

    pub address: Address,
    pub pan: Option<Pan>,
    pub channel: Channel,
    pub pulse_repetition_frequency: PulseRepetitionFrequency,
    pub preamble_length: PreambleLength,
    pub phr_mode: PhrMode,
    pub phr_rate: PhrRate,
    pub bit_rate: BitRate,
    pub sfd_timeout: u16,
    pub tx_preamble_code: u8,
    pub rx_preamble_code: u8,
    pub sfd_sequence: SfdSequence,

    /// If true, the device will be configured to output a 31.25 MHz clock on GPIO0.
    pub enable_clock_output: bool,

    /// If true, GPIO6 will be configured as an external RX enable pin and driven high when the
    /// device is in RX mode.
    pub enable_external_rx: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            enable_diagnostics: false,
            address: Address::Absent,
            pan: None,
            channel: Channel::Channel9,
            pulse_repetition_frequency: PulseRepetitionFrequency::Mhz64,
            preamble_length: PreambleLength::Symbols64,
            phr_mode: PhrMode::Standard,
            phr_rate: PhrRate::Standard,
            bit_rate: BitRate::Kbps6800,
            sfd_timeout: 129,
            tx_preamble_code: 9,
            rx_preamble_code: 9,
            sfd_sequence: SfdSequence::Ieee,
            enable_clock_output: false,
            enable_external_rx: false,
        }
    }
}

pub struct Device<SPI, O, IRQ, Delay, State> {
    config: Config,
    dev: ll::DW3000<SPI>,
    irq: IRQ,
    rst: O,
    delay: Delay,
    state: State,
}

impl<SPI, O, IRQ, Delay, State> Device<SPI, O, IRQ, Delay, State>
where
    SPI: SpiDevice,
    O: OutputPin,
    IRQ: Wait,
    Delay: DelayNs,
{
    /// Fully resets the device.
    pub async fn reset(self) -> Result<SPI, Device<SPI, O, IRQ, Delay, ReadyState>> {
        let config = self.config.clone();
        self.with_state(UninitializedState).init(config).await
    }
}

impl<SPI, O, IRQ, Delay, State> Device<SPI, O, IRQ, Delay, State>
where
    SPI: SpiDevice,
    IRQ: Wait,
    Delay: DelayNs,
{
    pub(crate) async fn wait_for_irq_with_timeout_us(
        &mut self,
        timeout_us: u32,
    ) -> Result<SPI, ()> {
        futures::select_biased! {
             _ = self.irq.wait_for_high().fuse() => Ok(()),
             _ = self.delay.delay_us(timeout_us).fuse() => Err(Error::IrqTimeout),
        }
    }
}

impl<SPI, O, IRQ, Delay, State> Device<SPI, O, IRQ, Delay, State>
where
    SPI: SpiDevice,
{
    /// Reads bytes directly from a register. The given buffer should be 2 bytes plus the length of
    /// the data to be read, and the data will be read into the buffer starting at index 2.
    pub(crate) async fn read_reg(&mut self, id: u8, offset: u8, buf: &mut [u8]) -> Result<SPI, ()> {
        buf[0] = (1 << 6) | (id << 1) | (offset >> 6);
        buf[1] = offset << 2;
        self.dev
            .bus()
            .transfer_in_place(buf)
            .await
            .map_err(ll::Error::Transfer)?;
        Ok(())
    }

    pub(crate) async fn read_reg_u32(&mut self, id: u8, offset: u8) -> Result<SPI, u32> {
        let mut buf = [0; 6];
        self.read_reg(id, offset, &mut buf).await?;
        Ok(u32::from_le_bytes(buf[2..6].try_into().unwrap()))
    }

    pub(crate) async fn read_reg_u64(&mut self, id: u8, offset: u8) -> Result<SPI, u64> {
        let mut buf = [0; 10];
        self.read_reg(id, offset, &mut buf).await?;
        Ok(u64::from_le_bytes(buf[2..10].try_into().unwrap()))
    }
}

impl<SPI, O, IRQ, Delay, State> Device<SPI, O, IRQ, Delay, State> {
    /// Returns the device's IEEE 802.15.4 address.
    pub fn address(&self) -> Address {
        self.config.address
    }

    pub(crate) fn with_state<NewState>(
        self,
        new_state: NewState,
    ) -> Device<SPI, O, IRQ, Delay, NewState> {
        Device {
            config: self.config,
            dev: self.dev,
            irq: self.irq,
            rst: self.rst,
            delay: self.delay,
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
    PreambleDetectionTimeout,
    PayloadTooLarge,
    FrameWaitTimeout,
    IrqTimeout,
}

pub type Result<SPI, T> = core::result::Result<T, Error<ll::Error<SPI>>>;
