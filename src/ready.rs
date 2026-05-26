use crate::{AsleepState, Device, Error, ReceivingState, Result, Timestamp};

use dw3000_ng::{ll, FastCommand};
use embedded_hal_async::{delay::DelayNs, digital::Wait, spi::SpiDevice};
use smoltcp::wire::{
    ieee802154::{self, Address},
    Ieee802154Frame,
};

#[derive(Clone)]
pub struct ReadyState {
    pub(crate) next_seq: u8,
}

const FCS_SIZE: usize = 2;

impl<SPI, O, IRQ, Delay> Device<SPI, O, IRQ, Delay, ReadyState>
where
    SPI: SpiDevice,
    IRQ: Wait,
    Delay: DelayNs,
{
    async fn configure_aon(&mut self) -> Result<SPI, ()> {
        let dev = &mut self.dev;

        dev.seq_ctrl().modify(|_, w| w.ainit2idle(0)).await?;
        dev.aon_dig_cfg().modify(|_, w| w.onw_aon_dld(1)).await?;
        dev.aon_cfg().write(|w| w.sleep_en(1).wake_csn(1)).await?;

        Ok(())
    }

    /// Enters into deep sleep.
    pub async fn sleep(mut self) -> Result<SPI, Device<SPI, O, IRQ, Delay, AsleepState>> {
        self.configure_aon().await?;

        self.dev.sys_status().write(|w| w.spirdy(1)).await?;
        self.dev.sys_enable().write(|w| w.spirdy_en(1)).await?;

        self.dev.aon_ctrl().write(|w| w.save(1)).await?;

        let ready_state = self.state.clone();
        Ok(self.with_state(AsleepState { ready_state }))
    }

    /// Puts the device into receive mode.
    ///
    /// The device will use double buffered mode to maximize time spent with the receiver enabled.
    pub async fn receive(self) -> Result<SPI, Device<SPI, O, IRQ, Delay, ReceivingState>> {
        self.receive_impl(None).await
    }

    /// Puts the device into receive mode, with a per-frame wait timeout.
    pub async fn receive_with_wait_timeout_ms(
        self,
        timeout_ms: u32,
    ) -> Result<SPI, Device<SPI, O, IRQ, Delay, ReceivingState>> {
        self.receive_impl(Some(timeout_ms)).await
    }

    async fn receive_impl(
        self,
        wait_timeout_ms: Option<u32>,
    ) -> Result<SPI, Device<SPI, O, IRQ, Delay, ReceivingState>> {
        let ready_state = self.state.clone();
        let mut ret = self.with_state(ReceivingState {
            wait_timeout_ms,
            ready_state,
        });
        ret.init().await?;
        Ok(ret)
    }

    /// Sends a single IEEE 802.15.4 frame, returning the timestamp it was given.
    pub async fn send(&mut self, buf: &[u8]) -> Result<SPI, Timestamp> {
        self.prepare_tx(buf).await?;
        self.dev.fast_command(FastCommand::CMD_TX as u8).await?;
        self.wait_for_tx_timeout_us(20_000).await
    }

    /// Sends a single IEEE 802.15.4 frame.
    ///
    /// This function will wait until the given time to send the frame (accurate to ~4 ns).
    pub async fn send_at(&mut self, buf: &[u8], timestamp: Timestamp) -> Result<SPI, Timestamp> {
        self.prepare_tx(buf).await?;
        self.dev
            .dx_time()
            .write(|w| w.value((timestamp.as_ticks() >> 8) as _))
            .await?;
        self.dev.fast_command(FastCommand::CMD_DTX as u8).await?;
        self.wait_for_tx_timeout_us(20_000_000).await
    }

    /// Sends a single IEEE 802.15.4 frame.
    ///
    /// This function will wait until the given interval has passed since the last frame was sent,
    /// and then send the frame (accurate to ~4 ns).
    pub async fn send_after_interval(
        &mut self,
        buf: &[u8],
        interval_ns: u64,
    ) -> Result<SPI, Timestamp> {
        self.prepare_tx(buf).await?;
        self.dev
            .dx_time()
            .write(|w| w.value(((interval_ns + 2) * (4992 / 2) / 10000) as _))
            .await?;
        self.dev.fast_command(FastCommand::CMD_DTX_TS as u8).await?;
        self.wait_for_tx_timeout_us(20_000_000).await
    }

    async fn prepare_tx(&mut self, buf: &[u8]) -> Result<SPI, ()> {
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
            src_addr: Some(self.config.address),
            dst_addr: Some(Address::BROADCAST),
            src_pan_id: config.pan,
            dst_pan_id: config.pan,
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

        Ok(())
    }

    async fn wait_for_tx_timeout_us(&mut self, timeout_us: u32) -> Result<SPI, Timestamp> {
        self.wait_for_irq_with_timeout_us(timeout_us).await?;

        let status = self.dev.sys_status().read().await?;
        if status.txfrs() != 1 {
            return Err(Error::Unknown);
        }

        let timestamp = Timestamp::from_ticks(self.dev.tx_time().read().await?.tx_stamp());

        Ok(timestamp)
    }
}
