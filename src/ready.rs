use crate::{AsleepState, Config, Device, Error, Result};

use dw3000_ng::{configs, ll, time::Instant, FastCommand};
use embedded_hal_async::{digital::Wait, spi::SpiDevice};
use num_traits::Float;
use smoltcp::wire::{
    ieee802154::{self, Address},
    Ieee802154Frame,
};

pub struct ReadyState {
    pub(crate) next_seq: u8,
}

#[derive(defmt::Format)]
pub struct SignalPower {
    receive_dbm: f32,
    first_path_dbm: f32,
}

/// Returns the signal power in the first path and the total received signal power.
async fn estimate_signal_power<SPI: SpiDevice>(
    dev: &mut ll::DW3000<SPI>,
    config: &Config,
) -> Result<SPI, Option<SignalPower>> {
    #[derive(Copy, Clone)]
    enum Method {
        Ipatov,
        Sts,
    }

    let method: Method = if dev.sys_cfg().read().await?.cia_ipatov() != 0 {
        Method::Ipatov
    } else if dev.sys_cfg().read().await?.cia_sts() != 0 {
        Method::Sts
    } else {
        return Ok(None);
    };

    let a: f32 = match (config.pulse_repetition_frequency, method) {
        (configs::PulseRepetitionFrequency::Mhz16, _) => 113.8,
        (configs::PulseRepetitionFrequency::Mhz64, Method::Ipatov) => 121.7,
        (configs::PulseRepetitionFrequency::Mhz64, Method::Sts) => 120.7,
    };

    let f1;
    let f2;
    let f3;
    let n;
    let c;

    match method {
        Method::Ipatov => {
            f1 = dev.ip_diag_2().read().await?.ip_fp1m();
            f2 = dev.ip_diag_3().read().await?.ip_fp2m();
            f3 = dev.ip_diag_4().read().await?.ip_fp3m();
            n = dev.ip_diag_12().read().await?.ip_nacc();
            c = dev.ip_diag_1().read().await?.ip_carea() as f32;
        }
        Method::Sts => {
            f1 = dev.sts_diag_2().read().await?.cp0_fp1m();
            f2 = dev.sts_diag_3().read().await?.cp0_fp2m();
            f3 = dev.sts_diag_4().read().await?.cp0_fp3m();
            n = dev.sts_diag_12().read().await?.cp0_nacc();
            c = dev.sts_diag_1().read().await?.cp0_carea() as f32;
        }
    }

    if n == 0 {
        return Ok(None);
    }

    let d6: u32 = if dev.dgc_cfg().read().await?.rx_tune_en() != 0 {
        let d: u32 = dev.dgc_dbg().read().await?.dgc_decision().into();
        6u32 * d
    } else {
        0u32
    };

    let first_path_dbm = 10.0
        * (((f1 * f1 + f2 * f2 + f3 * f3) as f32) / ((u32::from(n) * u32::from(n)) as f32)).log10()
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

pub struct ReceiveBuffer {
    data: ll::rx_buffer_0::R,
    len: usize,
}

impl AsRef<[u8]> for ReceiveBuffer {
    fn as_ref(&self) -> &[u8] {
        &self.data.data()[..self.len]
    }
}

pub struct Frame {
    buffer: ReceiveBuffer,
    pub time: Instant,
    pub signal_power: Option<SignalPower>,
}

impl Frame {
    /// Returns the raw IEEE 802.15.4 frame bytes.
    pub fn raw(&self) -> &[u8] {
        self.buffer.as_ref()
    }

    /// Returns the parsed IEEE 802.15.4 frame.
    pub fn parsed(&self) -> ieee802154::Frame<&[u8]> {
        ieee802154::Frame::new_unchecked(self.buffer.as_ref())
    }
}

impl defmt::Format for Frame {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "Frame {{ time: {}, signal_power: {} }}",
            self.time,
            self.signal_power
        );
    }
}

const FCS_SIZE: usize = 2;

impl<SPI, O, IRQ> Device<SPI, O, IRQ, ReadyState>
where
    SPI: SpiDevice,
    IRQ: Wait,
{
    /// Enters into deep sleep.
    pub async fn sleep(mut self) -> Result<SPI, Device<SPI, O, IRQ, AsleepState>> {
        let dev = &mut self.dev;

        dev.aon_cfg().write(|w| w.sleep_en(1)).await?;
        dev.aon_ctrl().write(|w| w.cfg_upload(1)).await?;

        Ok(self.with_state(AsleepState))
    }

    /// Receives a single IEEE 802.15.4 frame.
    pub async fn receive(&mut self) -> Result<SPI, Frame> {
        let dev = &mut self.dev;
        let config = &self.config;

        dev.sys_status()
            .write(|w| {
                w.rxfcg(1)
                    .rxphe(1)
                    .rxfce(1)
                    .rxfsl(1)
                    .rxfto(1)
                    .rxovrr(1)
                    .rxpto(1)
                    .rxsto(1)
            })
            .await?;

        dev.sys_enable().write(|w| w.rxfr_en(1)).await?;

        dev.fast_command(FastCommand::CMD_RX as u8).await?;

        loop {
            let _ = self.irq.wait_for_high().await;

            let status = dev.sys_status().read().await?;
            if status.rxfcg() == 1 {
                break;
            } else if status.rxfce() == 1 {
                if status.rxfce() == 1 {
                    return Err(Error::Fcs);
                } else if status.rxphe() == 1 {
                    return Err(Error::Phy);
                } else if status.rxfsl() == 1 {
                    return Err(Error::ReedSolomon);
                } else if status.rxsto() == 1 {
                    return Err(Error::SfdTimeout);
                } else if status.rxovrr() == 1 {
                    return Err(Error::Overrun);
                } else if status.rxpto() == 1 {
                    return Err(Error::PreambleDetectionTimeout);
                }
                return Err(Error::Unknown);
            }
        }

        let time = dev.rx_time().read().await?.rx_stamp();
        let time = Instant::new(time).expect("received timestamp should always be valid");

        let signal_power = if config.enable_diagnostics {
            estimate_signal_power(dev, config).await?
        } else {
            None
        };

        let rx_finfo = dev.rx_finfo().read().await?;
        let rx_buffer = dev.rx_buffer_0().read().await?;
        let rx_buffer_len = rx_finfo.rxflen() as usize;

        Ok(Frame {
            buffer: ReceiveBuffer {
                data: rx_buffer,
                len: rx_buffer_len - FCS_SIZE,
            },
            time,
            signal_power,
        })
    }

    /// Sends a single IEEE 802.15.4 frame.
    pub async fn send(&mut self, buf: &[u8]) -> Result<SPI, ()> {
        let dev = &mut self.dev;
        let config = &self.config;

        let seq = self.state.next_seq;
        self.state.next_seq = self.state.next_seq.wrapping_add(1);

        let frame_repr = ieee802154::Repr {
            frame_type: smoltcp::wire::Ieee802154FrameType::Data,
            frame_version: smoltcp::wire::Ieee802154FrameVersion::Ieee802154_2006,
            security_enabled: false,
            sequence_number: Some(seq),
            frame_pending: false,
            ack_request: false,
            pan_id_compression: true,
            dst_addr: Some(Address::BROADCAST),
            src_addr: Some(Address::Extended(self.address)),
            src_pan_id: Some(config.pan),
            dst_pan_id: Some(config.pan),
        };

        let tx_buf_len = frame_repr.buffer_len() + buf.len() + FCS_SIZE;
        if tx_buf_len > core::mem::size_of::<ll::tx_buffer::W>() {
            return Err(Error::PayloadTooLarge);
        }

        dev.tx_buffer()
            .write(|w| {
                let mut tx_buf = w.data();

                // header
                let mut frame = Ieee802154Frame::new_unchecked(&mut tx_buf);
                frame_repr.emit(&mut frame);

                // payload
                tx_buf[frame_repr.buffer_len()..frame_repr.buffer_len() + buf.len()]
                    .copy_from_slice(buf);

                w
            })
            .await?;

        dev.tx_fctrl()
            .modify(|_, w| {
                w.txflen(tx_buf_len as _)
                    .txbr(config.bit_rate as u8)
                    .tr(0)
                    .txb_offset(0)
                    .txpsr(config.preamble_length as u8)
                    .fine_plen(0)
            })
            .await?;

        dev.sys_status().write(|w| w.txfrs(1)).await?;
        dev.sys_enable().write(|w| w.txfrs_en(1)).await?;

        dev.fast_command(FastCommand::CMD_TX as u8).await?;

        loop {
            let _ = self.irq.wait_for_high().await;

            let status = dev.sys_status().read().await?;
            if status.txfrs() == 1 {
                break;
            }
        }

        Ok(())
    }
}
