use crate::{Device, Error, ReadyState, Result, Timestamp};

use dw3000_ng::{configs, FastCommand};
use embedded_hal_async::{digital::Wait, spi::SpiDevice};
use smoltcp::wire::ieee802154;

#[allow(unused)]
use num_traits::Float;

pub struct ReceivingState {
    pub(crate) wait_timeout_ms: Option<u32>,
    pub(crate) ready_state: ReadyState,
    pub(crate) signal_power_config: SignalPowerConfig,
}

/// The device configuration that signal power estimates depend on. It's read once up front so that
/// it doesn't have to be read for every frame.
#[derive(Default)]
pub(crate) struct SignalPowerConfig {
    cia_ipatov: bool,
    rx_tune_en: bool,
}

#[derive(Clone, defmt::Format)]
pub struct SignalPower {
    pub receive_dbm: f32,
    pub first_path_dbm: f32,
}

pub struct ReceiveBuffer {
    reg: [u8; 129],
    len: usize,
}

impl AsRef<[u8]> for ReceiveBuffer {
    fn as_ref(&self) -> &[u8] {
        &self.reg[2..2 + self.len]
    }
}

pub struct Frame {
    buffer: ReceiveBuffer,
    pub timestamp: Timestamp,
    pub signal_power: Option<SignalPower>,
}

impl Frame {
    /// Returns the raw IEEE 802.15.4 frame bytes.
    pub fn raw(&self) -> &[u8] {
        self.buffer.as_ref()
    }

    /// Returns the parsed IEEE 802.15.4 frame.
    pub fn parsed(&self) -> smoltcp::wire::Result<ieee802154::Frame<&[u8]>> {
        ieee802154::Frame::new_checked(self.buffer.as_ref())
    }
}

impl defmt::Format for Frame {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "Frame {{ timestamp: {}, signal_power: {} }}",
            self.timestamp,
            self.signal_power
        );
    }
}

const FCS_SIZE: usize = 2;

struct FrameMetadata {
    /// The length of the frame, excluding the FCS.
    len: usize,
    timestamp: Timestamp,
    signal_diagnostics: Option<SignalDiagnostics>,
}

/// The raw values that signal power is estimated from.
struct SignalDiagnostics {
    f1: u32,
    f2: u32,
    f3: u32,
    n: u32,
    c: u32,
    dgc_decision: u8,
}

impl<SPI, O, IRQ, Delay> Device<SPI, O, IRQ, Delay, ReceivingState>
where
    SPI: SpiDevice,
    IRQ: Wait,
{
    /// Stops receiving and goes back to the ready state.
    pub async fn stop(mut self) -> Result<SPI, Device<SPI, O, IRQ, Delay, ReadyState>> {
        self.dev
            .fast_command(FastCommand::CMD_TXRXOFF as u8)
            .await?;

        self.dev
            .rdb_status()
            .write(|w| {
                w.rxfcg0(1)
                    .rxfr0(1)
                    .ciadone0(1)
                    .cp_err0(1)
                    .rxfcg1(1)
                    .rxfr1(1)
                    .ciadone1(1)
                    .cp_err1(1)
            })
            .await?;

        self.dev
            .sys_status()
            .write(|w| {
                w.rxfcg(1)
                    .rxphe(1)
                    .rxfce(1)
                    .rxfsl(1)
                    .rxfto(1)
                    .rxovrr(1)
                    .rxpto(1)
                    .rxsto(1)
                    .arfe(1)
            })
            .await?;

        self.dev.sys_enable().write(|w| w).await?;

        let ready_state = self.state.ready_state.clone();
        Ok(self.with_state(ready_state))
    }

    pub(crate) async fn init(&mut self) -> Result<SPI, ()> {
        self.dev
            .sys_cfg()
            .modify(|_, w| {
                w.rxwtoe(match self.state.wait_timeout_ms {
                    Some(_) => 1,
                    None => 0,
                })
                .dis_drxb(0)
            })
            .await?;

        if let Some(timeout_ms) = self.state.wait_timeout_ms {
            self.dev
                .rx_fwto()
                .write(|w| w.value(timeout_ms * 499200 / 512))
                .await?;
        }

        self.state.signal_power_config = SignalPowerConfig {
            cia_ipatov: self.dev.sys_cfg().read().await?.cia_ipatov() != 0,
            rx_tune_en: self.dev.dgc_cfg().read().await?.rx_tune_en() != 0,
        };

        self.dev
            .sys_enable()
            .write(|w| {
                w.rxfr_en(1)
                    .rxfce_en(1)
                    .rxphe_en(1)
                    .rxrfsl_en(1)
                    .rxovrr_en(1)
                    .rxsto_en(1)
                    .rxpto_en(1)
                    .rxfto_en(1)
                    .arfe_en(1)
            })
            .await?;

        self.begin_rx().await
    }

    // Nothing can be received between a frame arriving and this completing, so the fewer
    // transactions that happen in that time, the better.
    async fn begin_rx(&mut self) -> Result<SPI, ()> {
        self.dev
            .fast_command(FastCommand::CMD_CLR_IRQS as u8)
            .await?;
        self.dev.fast_command(FastCommand::CMD_RX as u8).await?;
        Ok(())
    }

    /// Waits for a frame to be ready and returns the buffer number (0 or 1).
    async fn wait_for_frame_ready(&mut self) -> Result<SPI, u8> {
        loop {
            let _ = self.irq.wait_for_high().await;

            // check for a good frame first, since that's when the receiver is waiting on us
            let rdb_status = self.dev.rdb_status().read().await?;
            if rdb_status.rxfcg0() == 1 {
                return Ok(0);
            } else if rdb_status.rxfcg1() == 1 {
                return Ok(1);
            }

            let status = self.dev.sys_status().read().await?;
            if status.rxfce() == 1 {
                return Err(Error::Fcs);
            } else if status.rxphe() == 1 {
                return Err(Error::Phy);
            } else if status.rxfsl() == 1 {
                return Err(Error::ReedSolomon);
            } else if status.rxsto() == 1 {
                return Err(Error::SfdTimeout);
            } else if status.rxpto() == 1 {
                return Err(Error::PreambleDetectionTimeout);
            } else if status.rxfto() == 1 {
                return Err(Error::FrameWaitTimeout);
            } else if status.arfe() == 1 {
                // Counerintuitively, automatic frame filtering stops receiving on rejecting a
                // frame. So we restart it here and wait for another frame.
                self.begin_rx().await?;
                continue;
            }
            return Err(Error::Unknown);
        }
    }

    /// Reads everything about a frame that the device keeps in its diagnostic register sets.
    ///
    /// Nothing can be received while this runs, so it has to be quick. Counterintuitively, that
    /// means several small reads rather than one large one: small transactions finish before
    /// the task has a chance to yield, whereas after a large one the task has to wait for the
    /// executor to get back to it, which takes hundreds of microseconds on a busy system.
    async fn read_frame_metadata(&mut self, buffer_number: u8) -> Result<SPI, FrameMetadata> {
        let db_register_set_id = match buffer_number {
            0 => 0x18,
            _ => 0x1d,
        };

        let frame_len = (self.read_reg_u32(db_register_set_id, 0).await? & 0x1ff) as usize;
        if !(FCS_SIZE..=127).contains(&frame_len) {
            return Err(Error::MalformedFrame);
        }

        let rx_stamp = self.read_reg_u64(db_register_set_id, 4).await? & 0xffffffffff;

        let signal_diagnostics = if self.config.enable_diagnostics {
            Some(SignalDiagnostics {
                f1: self
                    .read_reg_u32(db_register_set_id, 0x44 /* IP_DIAG_2 */)
                    .await?
                    & 0x3fffff,
                f2: self
                    .read_reg_u32(db_register_set_id, 0x48 /* IP_DIAG_3 */)
                    .await?
                    & 0x3fffff,
                f3: self
                    .read_reg_u32(db_register_set_id, 0x4c /* IP_DIAG_4 */)
                    .await?
                    & 0x3fffff,
                n: self
                    .read_reg_u32(db_register_set_id, 0x1c /* IP_DIAG_12 */)
                    .await?
                    & 0xfff,
                c: self
                    .read_reg_u32(db_register_set_id, 0x40 /* IP_DIAG_1 */)
                    .await?
                    & 0x1ffff,
                // unlike the rest, this isn't kept per buffer, so the next frame will replace it
                dgc_decision: self.dev.dgc_dbg().read().await?.dgc_decision(),
            })
        } else {
            None
        };

        Ok(FrameMetadata {
            len: frame_len - FCS_SIZE,
            timestamp: Timestamp::from_ticks(rx_stamp),
            signal_diagnostics,
        })
    }

    async fn read_frame(
        &mut self,
        buffer_number: u8,
        metadata: FrameMetadata,
    ) -> Result<SPI, Frame> {
        let rx_buffer_reg_id = match buffer_number {
            0 => 0x12,
            _ => 0x13,
        };

        let mut rx_buffer_reg = [0u8; 2 + 127];
        self.read_reg(rx_buffer_reg_id, 0, &mut rx_buffer_reg[..2 + metadata.len])
            .await?;

        Ok(Frame {
            buffer: ReceiveBuffer {
                reg: rx_buffer_reg,
                len: metadata.len,
            },
            timestamp: metadata.timestamp,
            signal_power: metadata
                .signal_diagnostics
                .and_then(|diagnostics| self.estimate_signal_power(diagnostics)),
        })
    }

    /// Returns the signal power in the first path and the total received signal power.
    fn estimate_signal_power(&self, diagnostics: SignalDiagnostics) -> Option<SignalPower> {
        let SignalDiagnostics {
            f1,
            f2,
            f3,
            n,
            c,
            dgc_decision,
        } = diagnostics;

        if !self.state.signal_power_config.cia_ipatov || n == 0 {
            return None;
        }

        let a: f32 = match self.config.pulse_repetition_frequency {
            configs::PulseRepetitionFrequency::Mhz16 => 113.8,
            configs::PulseRepetitionFrequency::Mhz64 => 121.7,
        };

        let d6 = if self.state.signal_power_config.rx_tune_en {
            6.0 * dgc_decision as f32
        } else {
            0.0
        };

        let n_squared = (n * n) as f32;

        let first_path_dbm =
            10.0 * (((f1 * f1 + f2 * f2 + f3 * f3) as f32) / n_squared).log10() + d6 - a;

        let receive_dbm = 10.0 * ((c as f32 * (1 << 21) as f32) / n_squared).log10() + d6 - a;

        Some(SignalPower {
            receive_dbm,
            first_path_dbm,
        })
    }

    /// Waits for a single IEEE 802.15.4 frame.
    pub async fn wait(&mut self) -> Result<SPI, Frame> {
        let buffer_number = match self.wait_for_frame_ready().await {
            Ok(buffer_number) => buffer_number,
            Err(e) => {
                self.begin_rx().await?;
                return Err(e);
            }
        };

        // XXX: The metadata must be read before the receiver is enabled again. The device keeps
        // it in memory that it also uses while processing an incoming frame. If that happens
        // during our reads, they can return garbage, and the device can fail to store the
        // incoming frame's metadata, leaving that frame with the metadata of an earlier one.
        let metadata_result = self.read_frame_metadata(buffer_number).await;

        // the rest isn't affected by an incoming frame, so go ahead and begin the next receive asap
        self.begin_rx().await?;

        let frame_result = match metadata_result {
            Ok(metadata) => self.read_frame(buffer_number, metadata).await,
            Err(e) => Err(e),
        };

        // let the device know we're done with this buffer
        {
            self.dev
                .rdb_status()
                .write(|w| {
                    if buffer_number == 0 {
                        w.rxfcg0(1).rxfr0(1).ciadone0(1).cp_err0(1)
                    } else {
                        w.rxfcg1(1).rxfr1(1).ciadone1(1).cp_err1(1)
                    }
                })
                .await?;

            self.dev
                .fast_command(FastCommand::CMD_DB_TOGGLE as u8)
                .await?;
        }

        frame_result
    }
}
