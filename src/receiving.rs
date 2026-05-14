use crate::{Device, Error, ReadyState, Result, Timestamp};

use dw3000_ng::{configs, FastCommand};
use embedded_hal_async::{digital::Wait, spi::SpiDevice};
use smoltcp::wire::ieee802154;

#[allow(unused)]
use num_traits::Float;

pub struct ReceivingState {
    pub(crate) wait_timeout_ms: Option<u32>,
    pub(crate) ready_state: ReadyState,
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

        self.dev.rdb_status().write(|w| w.rxfr0(1).rxfr1(1)).await?;

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

        self.begin_rx().await
    }

    async fn begin_rx(&mut self) -> Result<SPI, ()> {
        let dev = &mut self.dev;

        dev.sys_status()
            .write(|w| {
                w.rxfr(1)
                    .rxfcg(1)
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

        dev.sys_enable()
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

        dev.fast_command(FastCommand::CMD_RX as u8).await?;

        Ok(())
    }

    /// Waits for a frame to be ready and returns the buffer number (0 or 1).
    async fn wait_for_frame_ready(&mut self) -> Result<SPI, u8> {
        loop {
            let _ = self.irq.wait_for_high().await;

            let status = self.dev.sys_status().read().await?;
            if status.rxfcg() == 1 {
                break;
            } else {
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

        // check which frame was received
        let buffer_number = if self.dev.rdb_status().read().await?.rxfr0() != 0 {
            0
        } else {
            1
        };

        Ok(buffer_number)
    }

    async fn read_frame(&mut self, buffer_number: u8) -> Result<SPI, Frame> {
        let (rx_buffer_reg_id, db_register_set_id) = match buffer_number {
            0 => (0x12, 0x18),
            _ => (0x13, 0x1d),
        };

        let mut rx_buffer_reg = [0u8; 2 + 127];
        self.read_reg(rx_buffer_reg_id, 0, &mut rx_buffer_reg)
            .await?;

        let frame_len = self.read_reg_u32(db_register_set_id, 0).await? & 0x1ff;
        if frame_len > 127 {
            return Err(Error::MalformedFrame);
        }

        let rx_stamp = self.read_reg_u64(db_register_set_id, 4).await? & 0xffffffffff;
        let timestamp = Timestamp::from_ticks(rx_stamp);

        let signal_power = if self.config.enable_diagnostics {
            self.estimate_signal_power(db_register_set_id).await?
        } else {
            None
        };

        if (frame_len as usize) < FCS_SIZE {
            return Err(Error::MalformedFrame);
        }

        Ok(Frame {
            buffer: ReceiveBuffer {
                reg: rx_buffer_reg,
                len: frame_len as usize - FCS_SIZE,
            },
            timestamp,
            signal_power,
        })
    }

    /// Returns the signal power in the first path and the total received signal power.
    async fn estimate_signal_power(
        &mut self,
        db_register_set_id: u8,
    ) -> Result<SPI, Option<SignalPower>> {
        if self.dev.sys_cfg().read().await?.cia_ipatov() == 0 {
            return Ok(None);
        };

        let a: f32 = match self.config.pulse_repetition_frequency {
            configs::PulseRepetitionFrequency::Mhz16 => 113.8,
            configs::PulseRepetitionFrequency::Mhz64 => 121.7,
        };

        let f1 = self
            .read_reg_u32(db_register_set_id, 0x44 /* IP_DIAG_2 */)
            .await?
            & 0x3fffff;
        let f2 = self
            .read_reg_u32(db_register_set_id, 0x48 /* IP_DIAG_3 */)
            .await?
            & 0x3fffff;
        let f3 = self
            .read_reg_u32(db_register_set_id, 0x4c /* IP_DIAG_4 */)
            .await?
            & 0x3fffff;
        let n = self
            .read_reg_u32(db_register_set_id, 0x1c /* IP_DIAG_12 */)
            .await?
            & 0xfff;
        let c = (self
            .read_reg_u32(db_register_set_id, 0x40 /* IP_DIAG_1 */)
            .await?
            & 0x1ffff) as f32;

        if n == 0 {
            return Ok(None);
        }

        let d6: u32 = if self.dev.dgc_cfg().read().await?.rx_tune_en() != 0 {
            let d: u32 = self.dev.dgc_dbg().read().await?.dgc_decision().into();
            6u32 * d
        } else {
            0u32
        };

        let first_path_dbm = 10.0
            * (((f1 * f1 + f2 * f2 + f3 * f3) as f32) / ((u32::from(n) * u32::from(n)) as f32))
                .log10()
            + (d6 as f32)
            - a;

        let receive_dbm = 10.0
            * ((c * (1 << 21) as f32) / ((u32::from(n) * u32::from(n)) as f32)).log10()
            + (d6 as f32)
            - a;

        Ok(Some(SignalPower {
            receive_dbm,
            first_path_dbm,
        }))
    }

    /// Waits for a single IEEE 802.15.4 frame.
    pub async fn wait(&mut self) -> Result<SPI, Frame> {
        // wait for a frame and succeed or fail, go ahead and begin the next receive asap
        let buffer_number = {
            let wait_result = self.wait_for_frame_ready().await;
            self.begin_rx().await?;
            wait_result?
        };

        // read the received frame
        let frame_result = self.read_frame(buffer_number).await;

        // let the device know we're done with this buffer
        {
            self.dev
                .rdb_status()
                .write(|w| {
                    if buffer_number == 0 {
                        w.rxfr0(1)
                    } else {
                        w.rxfr1(1)
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
